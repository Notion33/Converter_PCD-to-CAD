function quat = aa2quat(aa)
% function quat = aa2quat(aa)
% aa: axis-angle. Nx4. theta >= 0
% quat: unit quaternion. Nx4. qw >= 0 

quat = [cos(aa(:,1)./2),repmat(sin(aa(:,1)./2),1,3).*aa(:,2:4)]; % repmat is faster than bsxfun in this case
quat = normalize(quat);

end