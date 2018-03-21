function [Camera] = InitCameraStructure(imgSet, imgSeq)
% Initilize camera structure

if strcmp(imgSet,'synthetic')
% Add path to selected data set
    addpath(genpath(['data/' imgSet]));
    image_list = dir(['data/' imgSet '/*.png']);             %Find name of all images in data set but reads them in wrong order

elseif strcmp(imgSet,'kitti')
% Add path to selected data set
    addpath(genpath('data/kitti/images'));
    image_list = dir('data/kitti/images/*.png');
else
   error 
end

Nr_img = numel(imgSeq);                                           %Number of images  
initTrans = zeros(Nr_img,1);                                      %Initilize translation

% Declare camera structure
Camera = struct('imageSet',[],'index',[],'time',[],'imageData',[],'x',[],'y',[],'z',[],'R',[]);
Camera.imageSet = imgSet;
Camera.x = initTrans; Camera.y = initTrans; Camera.z = initTrans;
Camera.R = zeros(3,3,Nr_img);

% Store all images data into the struct
j = 1;
for i = imgSeq
    Camera.index{j} = image_list(i).name;
    Camera.imageData(:,:,j) = imread(image_list(i).name);
    j = j+1;
end

end

