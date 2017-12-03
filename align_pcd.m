%% Sequence description
% 1. PCD align
% 2. Top Bottom range searching
% 3. Top Bottom plane extraction
% 4. Wall Extraction (Wall range내에서 최대 직선을 갖는 range extraciton)
% 5. 2D projection
% 6. Image Preprocessing & Edge detection
% 7. Line fitting
% 8. Converting to dwg file

%% Initializing

Wall_Offset = 1;
scale = 16;
% ptCloud = pcread('j_engineering_all.pcd');
ptCloud = pcread('smallhouse.pcd');
% ptCloud = pcread('central_plaza.pcd');

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

[~,maxHeight_index] = max(local_max_height(:,2));                                    % Local maximum내 첫번째 두번째 최댓값 찾기
first_maxHeight_index = local_max_height(maxHeight_index,1);
first_maxHeight = height_count(first_maxHeight_index,1);
local_max_height(maxHeight_index, : ) = [0 0];
[~,maxHeight_index] = max(local_max_height(:,2));
second_maxHeight_index = local_max_height(maxHeight_index,1);
second_maxHeight = height_count(second_maxHeight_index,1);
local_max_height(maxHeight_index, : ) = [0 0];

if first_maxHeight_index > second_maxHeight_index
    [second_maxHeight, first_maxHeight] = deal(first_maxHeight, second_maxHeight);
    [second_maxHeight_index, first_maxHeight_index] = deal(first_maxHeight_index, second_maxHeight_index);
end

fprintf('3. Top Bottom plane extraction (3/9)\n');
maxDistance = 1.5;
referenceVector = [0,0,1];
maxAngularDistance = 5;
[model1,inlierIndices,outlierIndices] = pcfitplane(ptAlign,maxDistance,referenceVector,maxAngularDistance);
plane1 = select(ptAlign,inlierIndices);
remainPtCloud = select(ptAlign,outlierIndices);
[model2,inlierIndices,outlierIndices] = pcfitplane(remainPtCloud,maxDistance,referenceVector,maxAngularDistance);
plane2 = select(remainPtCloud,inlierIndices);
remainPtCloud = select(remainPtCloud,outlierIndices);
ptCloudB = pcdenoise(remainPtCloud);
% figure
% pcshow(ptAlign);
% figure
% pcshow(plane1);
% figure
% pcshow(plane2);
% figure;
% pcshow(ptCloudB);

topbottom_range = [plane1.ZLimits plane2.ZLimits];
[topbottom_min_value,topbottom_min_index] = min(topbottom_range);
wall_lower_value = topbottom_min_value;
topbottom_range(topbottom_min_index) = Inf;
[topbottom_min_value,topbottom_min_index] = min(topbottom_range);
wall_lower_value = (wall_lower_value + topbottom_min_value)/2;
topbottom_range(topbottom_min_index) = Inf;
[topbottom_min_value,topbottom_min_index] = min(topbottom_range);
wall_upper_value = topbottom_min_value;
topbottom_range(topbottom_min_index) = Inf;
[topbottom_min_value,topbottom_min_index] = min(topbottom_range);
wall_upper_value = (wall_upper_value + topbottom_min_value)/2;
topbottom_range(topbottom_min_index) = Inf;

wall_range_count=[];

for i= 1 : length(height_count)
    if height_count(i,1) > wall_lower_value && height_count(i,1) < wall_upper_value
        wall_range_count = [wall_range_count; height_count(i,:)];           
    end
end

% wall_range_count = height_count(first_maxHeight_index:second_maxHeight_index,:);
[~, wall_center_index] = min(wall_range_count(:,2));

wall_mean = wall_range_count(wall_center_index,2);
wall_upper_index = wall_center_index;
wall_lower_index = wall_center_index;

for i = wall_center_index : 1 : length(wall_range_count)
    if abs(wall_range_count(i,2) - wall_mean) > wall_mean*0.03
        wall_upper_index = i;
        break;
    end
end
for i = wall_center_index : -1 : 1
    if abs(wall_range_count(i,2) - wall_mean) > wall_mean*0.03
        wall_lower_index = i;
        break;
    end
end

fprintf('4. Wall Extraction (4/9)\n');

pointWall = zeros(ptCloudB.Count,3);
j = 1;

for i = 1 : ptCloudB.Count
%     if ptCloudB.Location(i,3) > wall_lower_value && ptCloudB.Location(i,3) < wall_upper_value
    if ptCloudB.Location(i,3) > wall_range_count(wall_lower_index,1) && ptCloudB.Location(i,3) < wall_range_count(wall_upper_index,1)
        pointWall(j,1:3) = ptCloudB.Location(i,1:3);
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

figure
imshow(rstimg)

%% edge_line start

fprintf('6. Image Preprocessing & Edge detection (6/9)\n');

img_edge = bwmorph(rstimg,'thin');
img_edge(1,:) = 1;
img_edge(end,:) = 1;
img_edge(:,1) = 1;
img_edge(:,end) = 1;
% afterOpening = imopen(gpuArray(img_edge),se);
% se = strel('square',3);
% img_edge = imclose(img_edge,se);
img_edge = imcomplement(img_edge);
img_edge = imfill(img_edge,'holes');
figure
imshow(img_edge)

% img_edge = edge(img_edge, 'log');
% figure
% imshow(img_edge)
% img_edge = bwareaopen(img_edge, 30);
% img_edge = bwmorph(img_edge,'skel',Inf);
% figure
% imshow(img_edge)
% se = strel('square',5);
% img_edge = imclose(img_edge,se);
% img_edge = imfill(img_edge,'holes');
% figure
% imshow(img_edge)
% img_edge = bwmorph(img_edge,'skel',Inf);
% figure
% imshow(img_edge)


