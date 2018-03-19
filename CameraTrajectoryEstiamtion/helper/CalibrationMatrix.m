function [ K ] = CalibrationMatrix(image_set)
%Summery: Calibration matrix, needs to be set for specific camera
%characteristics, use "imageinfo" for this.

if strcmp(image_set,'synthetic')
    fx = 992.041;
    fy = 996.750;
    resx = 1264;
    resy = 792;
    u0 = 632.0;         % Principal point
    v0 = 395.983;       % Principal point

    K = [fx  0  u0;
         0   fy v0;
         0   0  1];
    %K = [inv(Kinv(1:3,1:3)) [0;0;0]];
elseif strcmp(image_set,'kitti')
    K = [718.86 0 607.19;
         0 718.86 185.22;
         0 0 1];
else
    warning('Unknows calibration parameters!');
    Kinv = 0; K = 0;
    return
end

end

