%%초기 변수 선언
%%=========================================================================

wallLayerHeight = 3;

%%=========================================================================


ptCloud = pcread('j_engineering_all.pcd');

XYZ=ptCloud.Location;
%%plane fit
c=mean(XYZ,1);
Pc=bsxfun(@minus,XYZ,c);    
[~,~,V]=svd(Pc,0);
n=V(:,end);  %normal
n=n*sign(dot(n,[0,0,1])); %orientation convention
%rotate/align with xy-plane
u=cross(n,[0,0,1]);
deg=acosd(dot(n,[0,0,1]));
XYZnew=AxelRot(XYZ.',deg,u,[0,0,0]).';
ptAlign = pointCloud(XYZnew);
% figure
% pcshow(ptAlign);

% https://kr.mathworks.com/matlabcentral/answers/232828-how-to-fit-a-plane-to-my-point-cloud-data-and-rotate-it-so-that-the-plane-is-parallel-to-the-x-y-pla
% https://kr.mathworks.com/matlabcentral/fileexchange/30864-3d-rotation-about-shifted-axis


maxDistance = 0.3;
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

%===============================================Wall height extraction

height_start = floor(ptCloudB.ZLimits(1));
height_end = floor(ptCloudB.ZLimits(2));

height_count = (height_start:1:height_end)';
height_count(:,2) = 0;

for i = 1 : ptCloudB.Count
    line_1 = floor(ptCloudB.Location(i,3)) - height_start + 1;
    height_count(line_1, 2) = height_count(line_1, 2)+1;
end
[temp,maxHeight] = max(height_count);
pointWall = [];

wallLayerHeight = abs((wallLayerHeight + 1)/2);

for i = 1 : ptCloudB.Count
    if floor(ptCloudB.Location(i,3)) - height_start + 1 > maxHeight(2) - wallLayerHeight && floor(ptCloudB.Location(i,3)) - height_start + 1 < maxHeight(2) + wallLayerHeight
        pointWall = [pointWall; ptCloudB.Location(i,1:3)];
    end
end
ptWall = pointCloud(pointWall);

figure
pcshow(ptWall);

