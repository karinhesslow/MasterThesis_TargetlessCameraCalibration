%% Camera Trajectory estimation
% Summery:
%
% Inspiration: Mathworks - Monocular Visual Odometry
% Date: 2018-03-19
% Creator: Andreas Ellstrom
%
%% Read Input Images Sequence, OxTS data and Ground Truth
clear; clc;
addpath(genpath('Functions/'));
addpath(genpath('Functions/helper'));

% Select image set, ('synthetic','kitti0X','real')
imgSet = 'kitti';

% Initilize camera structure
Camera = InitCameraStructure(imgSet);

%% Load OxTS data and ground truth
seq = '00';
[TrueCameraTrajectory, extrinsic] = LoadTrueTrajectories(seq);

% Create the camera parameter object using the camera intrisics for the
% specified  image set
K = CalibrationMatrix('kitti');
cameraParams = cameraParameters('IntrinsicMatrix',K');

%% Reset
clc;
clearvars -except Camera cameraParams K TrueCameraTrajectory extrinsic
%% Create a View Set Containing the First View of the Sequence
% Create an empty viewSet object to manage the data associated with each view
vSet = viewSet;

% Read and display the first image
Img1 = Camera.imageData(:,:,1)/256;     % Convert to grey scale
player = vision.VideoPlayer();
step(player, Img1);

%% Undistort First Image and Extract Feature
prevImg = undistortImage(Img1, cameraParams);

% Detect features
prevPoints = detectSURFFeatures(prevImg, 'MetricThreshold', 500);

% Select a subset of features, uniformaly distributed throughout the image.
numPoints = 300;
prevPoints = selectUniform(prevPoints, numPoints, size(prevImg));

% Extract features. Using 'Upright' features improves matching quality if
% the camera motion involves little or no in-plane rotation.
prevFeatures = extractFeatures(prevImg, prevPoints,'Upright', true);

% Add the first view. Place the camera associated with the first view at
% the origin, oriented along the Z-axis.
viewId = 1;
vSet = addView(vSet, viewId, 'Points', prevPoints, 'Orientation', eye(3), 'Location', [0 0 0]);

%% Plot initial Camera Pose
clf
figure(1)
axis([-200, 350, -100, 100, -50, 300]);

% Set Y-axis to be vertical down pointing
view(gca, 3);
set(gca, 'CameraUpVector', [0 -1 0]);
camorbit(gca, -120, 0, 'data', [0, 1, 0]);

grid on
xlabel('X (cm)')
ylabel('Y (cm)')
zlabel('Z (cm)')
hold on

% Plot estiamted camera pose
cameraSize = 7;
camEstimated = plotCamera('Size', cameraSize, 'Location', vSet.Views.Location{1}, 'Orientation', vSet.Views.Orientation{1}, 'Color','g','Opacity', 0);

% Plot true camera pose
camActual = plotCamera('Size', cameraSize, 'Location', TrueCameraTrajectory.Location(1,:), 'Orientation', TrueCameraTrajectory.Orientation(:,:,1), 'Color', 'b', 'Opacity', 0);

% Initialize camera trajectories
trajectoryEstimated = plot3(0, 0, 0, 'g-');
trajectoryActual = plot3(0, 0, 0, 'b-');

legend('Estimate Trajectory', 'True Trajectory');
title('Camera Trajectory')

%% Estiamte pose of the Second View
viewId = 2;
Img2 = Camera.imageData(:,:,viewId)/256;
step(player,Img2);

% Undistort image
currImg = undistortImage(Img2, cameraParams);

% Match features between the previous and current image
[currPoints, currFeatures, indexPairs] = helperDetectAndMatchFeatures(prevFeatures, currImg);

% Estimate the pose of the current view relative to the previous view
%[E, inlierIdx, status] = estimateEssentialMatrix(matchedPoints1, matchedPoints2, cameraParams);
[orient, loc, inlierIdx] = helperEstimateRelativePose(prevPoints(indexPairs(:,1)), currPoints(indexPairs(:,2)), cameraParams);

% Exclude epipolar outliers
inlierIndexPairs = indexPairs(inlierIdx, :);

% Add the current view to the view set
vSet = addView(vSet, viewId, 'Points', currPoints, 'Orientation', orient, 'Location', loc);
% Store the point matched between the previous and the current views
vSet = addConnection(vSet, viewId-1, viewId, 'Matches', inlierIndexPairs);

% Normalize view set, Update camera plots and update camera trajectory
%vSet = helperNormalizeViewSet(vSet, TrueCameraTrajectory);

helperUpdateCameraPlots(viewId, camEstimated, camActual, poses(vSet), TrueCameraTrajectory);
helperUpdateCameraTrajectories(viewId, trajectoryEstimated, trajectoryActual, poses(vSet), TrueCameraTrajectory)

% Step to the next view
prevImg = currImg;
prevFeatures = currFeatures;
prevPoints = currPoints;

%% Bootstrap Estimating Camera Trajectory Using Global Bundle Adjustment
for viewId = 3:15
% Read and display the next image
Img = Camera.imageData(:,:,viewId)/256;     % Normalize image [0,1]
step(player, Img)

% Undistort image
currImg = undistortImage(Img, cameraParams);

% Match points between the previous and the current image
[currPoints, currFeatures, indexPairs] = helperDetectAndMatchFeatures(prevFeatures, currImg);

% Eliminate ouliers from feaure matches
inlierIdx = helperFindEpipolarInliers(prevPoints(indexPairs(:,1)), currPoints(indexPairs(:,2)), cameraParams);
inlierIndexPairs = indexPairs(inlierIdx,:);

% Triangulate points from the previous two views, and find the
% corresponding points in the current view
[worldPoints, imagePoints] = helperFind3Dto2DCorrespondences(vSet, cameraParams, inlierIndexPairs, currPoints);

% Ignore RANSAC warning
warningstate = warning('off','vision:ransac:maxTrialsReached');

% Estimate the world camera pose for the current view
[orient, loc] = estimateWorldCameraPose(imagePoints, worldPoints, cameraParams, 'Confidence', 99.99, 'MaxReprojectionError', 0.8);

% Restore the original warning state
warning(warningstate)

% Store the points matched between the previous and the current view
vSet = addView(vSet, viewId, 'Points', currPoints, 'Orientation', orient, 'Location', loc);
% Store the point matched between the previous and the current views
vSet = addConnection(vSet, viewId-1, viewId, 'Matches', inlierIndexPairs);

tracks = findTracks(vSet); % Find point tracks spanning multiple view
camPoses = poses(vSet);    % Get camera poses for all view

% Triangulate initial locations for the 3D world points
xyzPoints = triangulateMultiview(tracks, camPoses, cameraParams);

% Refine camera poses using bundle adjustment
[~, camPoses] = bundleAdjustment(xyzPoints, tracks, camPoses, cameraParams, 'PointsUndistorted', true, ...
    'AbsoluteTolerance', 1e-9, 'RelativeTolerance', 1e-9, 'Maxiterations', 300);

vSet = updateView(vSet, camPoses); % Update view set

% Normalize view set, Update camera plots and update camera trajectory
%vSet = helperNormalizeViewSet(vSet, TrueCameraTrajectory);

helperUpdateCameraPlots(viewId, camEstimated, camActual, poses(vSet), TrueCameraTrajectory);
helperUpdateCameraTrajectories(viewId, trajectoryEstimated, trajectoryActual, poses(vSet), TrueCameraTrajectory)
% Update camera step
prevImg = currImg;
prevFeatures = currFeatures;
prevPoints = currPoints;

end

%% Estiamte Remaining Camera Trajectory Using Windowed Bundle Adjustment
for viewId = 16:numel(Camera.imageData(1,1,:))
% Read and display the next image
Img = Camera.imageData(:,:,viewId)/256;     % Normalize image [0,1]
step(player, Img)

% Undistort image
currImg = undistortImage(Img, cameraParams);

% Match points between the previous and the current image
[currPoints, currFeatures, indexPairs] = helperDetectAndMatchFeatures(prevFeatures, currImg);

% Eliminate ouliers from feaure matches
inlierIdx = helperFindEpipolarInliers(prevPoints(indexPairs(:,1)), currPoints(indexPairs(:,2)), cameraParams);
inlierIndexPairs = indexPairs(inlierIdx,:);

% Triangulate points from the previous two views, and find the
% corresponding points in the current view
[worldPoints, imagePoints] = helperFind3Dto2DCorrespondences(vSet, cameraParams, inlierIndexPairs, currPoints);

% Ignore RANSAC warning
warningstate = warning('off','vision:ransac:maxTrialsReached');

% Estimate the world camera pose for the current view
[orient, loc] = estimateWorldCameraPose(imagePoints, worldPoints, cameraParams,'MaxNumTrials', 5000,'Confidence', 99.99, 'MaxReprojectionError', 1.0);
% Restore the original warning state
warning(warningstate)

% Store the points matched between the previous and the current view
vSet = addView(vSet, viewId, 'Points', currPoints, 'Orientation', orient, 'Location', loc);
% Store the point matched between the previous and the current views
vSet = addConnection(vSet, viewId-1, viewId, 'Matches', inlierIndexPairs);

% Refine estimated camera poses using windowed bundle adjustment. Run the
% optimization every 7th view.
if mod(viewId, 7) == 0
    % Find point tracks in the last 15 views of triangulation
    viewId
    windowSize = 10;
    startFrame = max(1, viewId - windowSize);
    tracks = findTracks(vSet, startFrame:viewId);
    camPoses = poses(vSet, startFrame:viewId);
    [xyzPoints, reprojErrors] = triangulateMultiview(tracks, camPoses, cameraParams);
    
    % Hold the first two poses fixed, to keep the same scale
    fixedIds = [startFrame, startFrame+1];
    
    % Exclude points and tracks with high reprojection errors
    idx = reprojErrors < 2;
    
    [~, camPoses] = bundleAdjustment(xyzPoints(idx,:), tracks(idx), camPoses, cameraParams, ...
        'FixedViewIDs', fixedIds, 'PointsUndistorted', true, 'AbsoluteTolerance', 1e-9, ...
        'RelativeTolerance', 1e-9, 'Maxiterations', 300);
    vSet = updateView(vSet, camPoses);
     
end

    % Update camera trajectory
    % Normalize view set, Update camera plots and update camera trajectory
    %vSet = helperNormalizeViewSet(vSet, TrueCameraTrajectory);

    helperUpdateCameraPlots(viewId, camEstimated, camActual, poses(vSet), TrueCameraTrajectory);
    helperUpdateCameraTrajectories(viewId, trajectoryEstimated, trajectoryActual, poses(vSet), TrueCameraTrajectory)
    
    
    % Update time step
    prevImg = currImg;
    prevFeatures = currFeatures;
    prevPoints = currPoints;

end
%%
HistogramOfMatches(vSet)

%%
for i = 1:515
   states(i,:) = vSet.Views.Location{i}-vSet.Views.Location{1};
   trueStates(i,:) = TrueCameraTrajectory.Location(i,:);
 end
%% Scale Estimation
for i = 1:514
dStates(i,:) = states(i+1,:) - states(i,:); 
dTrueStates(i,:) = trueStates(i+1,:) - trueStates(i,:);
end
xScale = dTrueStates(:,1)./dStates(:,1);
zScale = dTrueStates(:,3)./dStates(:,3);
xMean = mean(xScale); xMedian = median(xScale);
zMean = mean(zScale); zMedian = median(zScale);

%% Plot Scaling
clf
xMeanStates = zeros(1,515);
xMedianStates = zeros(1,515);
zMeanStates = zeros(1,515);
zMedianStates = zeros(1,515);
xTrueStates = zeros(1,515);
zTrueStates = zeros(1,515);

for i = 1:514
   xMeanStates(i+1) = xMeanStates(i) + xMean*dStates(i,1);
   zMeanStates(i+1) = zMeanStates(i) + zMean*dStates(i,3);
   xMedianStates(i+1) = xMedianStates(i) + xMedian*dStates(i,1);
   zMedianStates(i+1) = zMedianStates(i) + zMedian*dStates(i,3);
   xTrueStates(i+1) = xTrueStates(i) + xScale(i)*dStates(i,1);
   zTrueStates(i+1) = zTrueStates(i) + zScale(i)*dStates(i,3);
end

plot(xMeanStates,zMeanStates,'-black')
hold on
axis equal
grid on
plot(xMedianStates,zMedianStates,'-g')
plot(xTrueStates, zTrueStates,'-r')
plot(trueStates(:,1), trueStates(:,3),'--b')
%hold on
%plot(xScale.*states(2:end,1),zScale.*states(2:end,3),'.g')