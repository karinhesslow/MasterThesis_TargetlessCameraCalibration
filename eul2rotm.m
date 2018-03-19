function [ rotm ] = eul2rotm( eul )
% Converts Euler angles to rotation matrix
% eul = [alpha, beta, gamma] = [yaw, pitch, roll]
alpha = eul(:,1); beta = eul(:,2); gamma = eul(:,3);

rotm = zeros(3,3,numel(alpha));

for i = 1:numel(alpha)
    
    Rz = [cos(alpha(i)) -sin(alpha(i)) 0; sin(alpha(i)) cos(alpha(i)) 0; 0 0 1];
    Ry = [cos(beta(i)) 0 sin(beta(i)); 0 1 0; -sin(beta(i)) 0 cos(beta(i))];
    Rx = [1 0 0; 0 cos(gamma(i)) -sin(gamma(i)); 0 sin(gamma(i)) cos(gamma(i))];

    rotm(:,:,i) = Rz*Ry*Rx;
    
end
end

