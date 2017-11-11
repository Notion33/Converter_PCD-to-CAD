function rot = quat2rot(quat)
% function rot = quat2rot(quat)
% quat: unit quaternion. Nx4. qw >= 0 
% rot: rotation matrix. 3x3xN

% rot=quat2dcm(quat)';

rot = zeros(3,3,size(quat,1));

rot(1,1,:) = quat(:,1).^2 + quat(:,2).^2 - quat(:,3).^2 - quat(:,4).^2;
rot(1,2,:) = 2.*(quat(:,2).*quat(:,3) - quat(:,1).*quat(:,4));
rot(1,3,:) = 2.*(quat(:,2).*quat(:,4) + quat(:,1).*quat(:,3));
rot(2,1,:) = 2.*(quat(:,2).*quat(:,3) + quat(:,1).*quat(:,4));
rot(2,2,:) = quat(:,1).^2 - quat(:,2).^2 + quat(:,3).^2 - quat(:,4).^2;
rot(2,3,:) = 2.*(quat(:,3).*quat(:,4) - quat(:,1).*quat(:,2));
rot(3,1,:) = 2.*(quat(:,2).*quat(:,4) - quat(:,1).*quat(:,3));
rot(3,2,:) = 2.*(quat(:,3).*quat(:,4) + quat(:,1).*quat(:,2));
rot(3,3,:) = quat(:,1).^2 - quat(:,2).^2 - quat(:,3).^2 + quat(:,4).^2;
 
end