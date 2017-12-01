%% Sequence description
% 1. PCD align
% 2. Top Bottom range searching
% 3. Top Bottom plane extraction
% 4. Wall Extraction (Wall range내에서 최대 직선을 갖는 range extraciton)
% 5. 2D projection
% 6. Image Preprocessing & Edge detection
% 7. Curve extraction
% 8. Line fitting
% 9. Converting to dwg file

%% Initializing

Wall_Offset = 0.2;
scale = 16;
ptCloud = pcread('j_engineering_all.pcd');

%% XY-plane에 PCD Align 시키기
fprintf('1. PCD alignment (1/9)\n');

%plane fit
XYZ=ptCloud.Location;
c=mean(XYZ,1);
Pc=bsxfun(@minus,XYZ,c);    
[~,~,V]=svd(Pc,0);

%normal벡터 계산
n=V(:,end);

%orientation convention
n=n*sign(dot(n,[0,0,1]));

%rotate/align with xy-plane
u=cross(n,[0,0,1]);
deg=acosd(dot(n,[0,0,1]));
XYZnew=AxelRot(XYZ.',deg,u,[0,0,0]).';
ptAlign = pointCloud(XYZnew);

% figure
% pcshow(ptAlign);

% https://kr.mathworks.com/matlabcentral/answers/232828-how-to-fit-a-plane-to-my-point-cloud-data-and-rotate-it-so-that-the-plane-is-parallel-to-the-x-y-pla
% https://kr.mathworks.com/matlabcentral/fileexchange/30864-3d-rotation-about-shifted-axis

pcwrite(ptAlign,'central_plaza.pcd','Encoding','ascii');

%% Top Bottom range searching
fprintf('2. Top Bottom range searching (2/9)\n');

height_start = floor(ptAlign.ZLimits(1)*10)/10;
height_end = floor(ptAlign.ZLimits(2)*10)/10;

height_count = (height_start:0.1:height_end)';
height_count(:,2) = 0;

for i = 1 : ptAlign.Count
    line_1 = floor(floor(ptAlign.Location(i,3)*10) - height_start*10 + 1);  %소숫점 1번째에서 내림 (0.1m 단위로 height 체킹위함)
    height_count(line_1, 2) = height_count(line_1, 2)+1;
end

windowSize = 3;                                                             % windowsize 3으로 Average filter 돌리기
height_count = filter((1/windowSize)*ones(1,windowSize),1,height_count);

% height_count_diff = diff(height_count);                                     % 1차 미분 적용
% height_count_diff = [0 0 ; height_count_diff];

[pks,locs] = findpeaks((height_count(:,2)'));                               % Local maximum 찾기
pks = pks';
locs = locs';
local_max_height = [locs pks];

[~,maxHeight_index] = max(local_max_height);                                    % Local maximum내 첫번째 두번째 최댓값 찾기
first_maxHeight = height_count(local_max_height(maxHeight_index(1,2),1),1);
local_max_height(maxHeight_index(2), : ) = [0 0];
[~,maxHeight_index] = max(local_max_height);
second_maxHeight = height_count(local_max_height(maxHeight_index(1,2),1),1);
local_max_height(maxHeight_index(2), : ) = [0 0];

if first_maxHeight > second_maxHeight
    [second_maxHeight, first_maxHeight] = deal(first_maxHeight, second_maxHeight);
end

fprintf('3. Top Bottom plane extraction (3/9)\n');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 이 부분에 top & bottom extraction (RANSAC이용) 코드 들어야가 함
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


fprintf('4. Wall Extraction (4/9)\n');

pointWall = zeros(ptAlign.Count,3);
j = 1;

for i = 1 : ptAlign.Count
    if ptAlign.Location(i,3) > first_maxHeight + Wall_Offset && ptAlign.Location(i,3) < second_maxHeight - Wall_Offset
        pointWall(j,1:3) = ptAlign.Location(i,1:3);
        j = j+1;
    end
end
ptWall = pointCloud(pointWall);

% figure
% pcshow(ptWall);

%% 2D Projection

px=pointWall(:,1);
py=pointWall(:,2);

Ir = max(py)-min(py);
Ic = max(px)-min(px);
numr = round(Ir * scale);
numc = round(Ic * scale);

fprintf('5. 2D projection (5/9)\n');
% grid construction
tol = 10*2;
xl = min(px); xr = max(px); yl = min(py); yr = max(py);
xx = linspace(xl,xr,numc); yy = linspace(yl,yr,numr);
[X,Y] = meshgrid(xx,yy);
grid_centers = [X(:),Y(:)];

% classification
class = knnsearch(grid_centers,[px,py]); 

% data_grouping
class_stat = zeros(numr*numc, 1);
class_stat(class) = 1;

% 2D reshaping
class_stat_M = reshape(class_stat, size(X)); 
img = 1-class_stat_M(end:-1:1,:);

rstimg = ones(numr+tol, numc+tol);
rstimg((tol/2)+1:(tol/2)+numr,(tol/2)+1:(tol/2)+numc) = img;

% figure
% imshow(rstimg)

%% edge_line start

fprintf('6. Image Preprocessing & Edge detection (6/9)\n');

img_edge = bwmorph(rstimg,'thin');
% afterOpening = imopen(gpuArray(img_edge),se);

se = strel('square',2);
img_edge = imclose(img_edge,se);
img_edge = edge(img_edge, 'log');
img_edge = bwareaopen(img_edge, 30);
figure
imshow(img_edge)
