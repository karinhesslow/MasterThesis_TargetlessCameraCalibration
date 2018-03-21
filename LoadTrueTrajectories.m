function [TrueCameraTrajectory, extrinsic] = LoadTrueTrajectories(seq, imgSeq)
% Summery: Loads ground truth for camera, vehicle and extrinsic parameters
% for Kitti sequenceXX 
% Input:
%   seq                 [00-07] Selected sequence
%   imgSeq              [first:last] Image sequence
% Output:
%   cameraTrajectory    True camera trajecory
%   vehicleTrajectory   True vehicle trajectory
%   extrinsic           True extrinisc parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Data for specified sequence
TrueCameraTrajectory = struct('Name',[],'Time',[],'Orientation',[],'Location',[]);

% Load data of the specified sequence [R1 R2 R3 t1 R4 R5 R6 t2 R7 R8 R9 t3]
kittiData = importdata(['data/kitti/' seq '.txt']);
kittiData = kittiData(imgSeq,:);
TrueCameraTrajectory.Name = seq;
Times = importdata('data/kitti/times.txt');
TrueCameraTrajectory.Time = Times(imgSeq);
TrueCameraTrajectory.Location = [kittiData(:,4) kittiData(:,8) kittiData(:,12)];

% Create orientation
orientation = zeros(3,3,numel(kittiData(:,1)));
for i = 1:numel(kittiData(:,1))
    orientation(:,:,i) = [kittiData(i,1) kittiData(i,2) kittiData(i,3);
                          kittiData(i,5) kittiData(i,6) kittiData(i,7);
                          kittiData(i,9) kittiData(i,10) kittiData(i,11)];
end
TrueCameraTrajectory.Orientation = orientation;
% Extrinsic parameters for camera 0
%R_Vehicle2Cam = eul2rotm([-pi/2, 0, -pi/2]); 
R_Vehicle2Cam = eul2rotm([0, 0, 0]); 
t_Vehicle2Cam = [1.08; -0.32; 0.72];

extrinsic = [R_Vehicle2Cam t_Vehicle2Cam];

end
