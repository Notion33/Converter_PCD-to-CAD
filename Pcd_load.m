ptCloud = pcread('j_engineering_all.pcd');

maxDistance = 0.2;
referenceVector = [0,0,1];
maxAngularDistance = 10;
[model1,inlierIndices,outlierIndices] = pcfitplane(ptCloud,maxDistance,referenceVector,maxAngularDistance);
plane1 = select(ptCloud,inlierIndices);

remainPtCloud = select(ptCloud,outlierIndices);
roi = [-inf,inf;-inf,inf;-inf,inf];
sampleIndices = findPointsInROI(remainPtCloud,roi);
[model2,inlierIndices,outlierIndices] = pcfitplane(remainPtCloud,maxDistance,referenceVector,maxAngularDistance);
plane2 = select(remainPtCloud,inlierIndices);
remainPtCloud = select(remainPtCloud,outlierIndices);

normals = pcnormals(plane1);
v = normnd(plane1.Location);
% mean_plane1 = [mean(plane1.Location(:,1)), mean(plane1.Location(:,2)), mean(plane1.Location(:,3))];
% mean_plane = [plane1.Location(:,1)-mean_plane1(1),plane1.Location(:,2)-mean_plane1(2),plane1.Location(:,3)-mean_plane1(3)];
% mean_ptCloud1 = pointCloud(mean_plane);

quaternion = angle2quat(v(3),v(2),v(1),'ZYX');
% tform = eul2tform([v(3) v(2) v(1)],'ZYX');
% tform1 = affine3d(tform);
% plane1Out = pctransform(mean_ptCloud1,tform1);

% tform = affine3d([v(1) 0 0 0; 0 v(2) 0 0; 0 0 v(3) 0; 0 0 0 1]);
% figure
% pcshow(ptCloud);
% figure
% pcshow(plane1);
% figure
% pcshow(plane2);
figure
pcshow(remainPtCloud);

ptCloudB = pcdenoise(remainPtCloud);
figure;
pcshow(ptCloudB);


% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% v = v';
% Plot
%scatter3(plane1.Location(:,1),plane1.Location(:,2),plane1.Location(:,3)); 
% hold on; quiver3(0 , 0, 0,v(1),v(2),v(3));

% x = plane1.Location(1:10:end,1:10:end,1);
% y = plane1.Location(1:10:end,1:10:end,2);
% z = plane1.Location(1:10:end,1:10:end,3);
% u = normals(1:10:end,1:10:end,1);
% v = normals(1:10:end,1:10:end,2);
% w = normals(1:10:end,1:10:end,3);
% quiver3(x,y,z,u,v,w);
% 
% figure
% pcshow(plane1)
% title('First Plane')

% figure
% pcshow(plane2)
% title('Second Plane')
% figure
% pcshow(remainPtCloud)
% title('Remaining Point Cloud')