function aa = quat2aa(quat)
% function aa = quat2aa(quat)
% quat: unit quaternion. Nx4. qw >= 0 
% aa: axis-angle. Nx4. theta >= 0

theta = 2*acos(quat(:,1));
axis = quat(:,2:4)./repmat(sin(theta./2),1,3);
axis = normalize(axis);

aa = [theta,axis];

end