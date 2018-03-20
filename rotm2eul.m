function [ eul ] = rotm2eul( rotm )
% Converts a rotation matrix(3x3) to Euler angles(alpha, beta gamma) = [z y x]
yaw = atan2(rotm(1,2),rotm(1,1));
pitch = atan2(-rotm(1,3),sqrt(rotm(3,2)^2) + rotm(3,3)^2);
roll = atan2(rotm(2,3),rotm(3,3));

eul = [roll pitch yaw];
end

