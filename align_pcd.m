%% Initializing

Wall_Offset = 0.3;
scale = 16;
ptCloud = pcread('j_engineering_all.pcd');

%% XY-plane에 PCD Align 시키기

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

%% Wall height extraction

height_start = floor(ptAlign.ZLimits(1)*10)/10;
height_end = floor(ptAlign.ZLimits(2)*10)/10;

height_count = (height_start:0.1:height_end)';
height_count(:,2) = 0;

for i = 1 : ptAlign.Count
    line_1 = floor(floor(ptAlign.Location(i,3)*10) - height_start*10 + 1);         %소숫점 1번째에서 내림 (0.1m 단위로 height 체킹위함)
    height_count(line_1, 2) = height_count(line_1, 2)+1;
end

[temp,first_maxHeight] = max(height_count);                                     % Z height내 첫번째 두번째 최댓값 찾기
height_count(first_maxHeight(2),:) = [];
[temp,second_maxHeight] = max(height_count);
height_count(second_maxHeight(2),:) = [];

pointWall = [];

for i = 1 : ptCloudB.Count
    if floor(ptCloudB.Location(i,3)) - height_start > first_maxHeight(2)/10 + Wall_Offset && floor(ptCloudB.Location(i,3)) - height_start < second_maxHeight(2)/10 - Wall_Offset
        pointWall = [pointWall; ptCloudB.Location(i,1:3)];
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

fprintf('2. pt2image start (2/4)\n')
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