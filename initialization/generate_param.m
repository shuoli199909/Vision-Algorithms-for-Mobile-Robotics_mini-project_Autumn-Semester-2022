function [params] = generate_param(name_data) 
%% Description 
% generating the parameters of the whole pipeline 
%--Input: 
% name_data: string. different names correspond to different datasets. 
%          - 'kitti_0': kitti dataset, image_0 
%          - 'kitti_1': kitti dataset, image_1 
%          - 'malaga_raw': malaga dataset, raw images 
%          - 'malaga_small': malaga dataset, small images 
%          - 'malaga_large': malaga dataset, large images 
%          - 'parking': parking dataset 
%          - 'private': private dataset 
%--Output: 
% list_img: directories of images in the dataset. size = [num_img, 1] 
% poses: extrinsic matrix. [R|t]. size = [num_img, 12]. to reshape it, we 
%        should use reshape(pose_current, [4,3])'. and the we can get the 
%        [3,4] matrix. 
% K: intrinsic matrix, size = [3, 4]. 
%% Implementation 
% dataset parameters 
[list_img, poses, K] = data_initialization(name_data); 
params.dataset = name_data; 
params.list_img = list_img; 
params.poses = poses; % extrinsic matrices. [1, 12] -> [3, 4]
                      % pose_tmp = reshape(pose_tmp, [4, 3])'; 
params.K = K(:, 1:3); % intrinsic matrix. [3, 4] -> [3, 3] 
% initial visualization of triangulation 
params.init_visualization = true; 
% different settings for different datasets 
if name_data == 'kitti_0' 
    % KLT tracker parameters 
    params.feature = 'Harris'; % descriptors 
    params.span_init = 6; % span of frames used for initialization 
    params.MaxBidirectionalError = 0.8; % maximal bidirectional error 
    params.NumPyramidLevels = 4; % number of pyramid level
    params.BlockSize = [25, 25];  % size of neighborhood around each point being tracked
    params.MaxIterations = 50; % maximum number of search iterations for each point 
    % relative pose estimation  
    params.MethodForEstimation = 'LMedS'; 
    % MethodForEstimation: size = char(). methods of feature points refinement. 
    %          'LMedS':      Least Median of Squares
    %          'MSAC':       M-estimator SAmple Consensus 
    %          'Norm8Point': Normalized eight-point algorithm 
    params.NumTrials = 20000; % number of trials 
    params.DistanceThreshold = 0.015; % treshold of distance 
    params.Confidence = 90; % confidence of matching 
    
    
elseif name_data == 'kitti_1' 
    % KLT tracker parameters 
    params.MaxBidirectionalError = 0.8; % maximal bidirectional error 
    params.NumPyramidLevels = 4; % number of pyramid level
    params.BlockSize = [25, 25];  % size of neighborhood around each point being tracked
    params.MaxIterations = 50; % maximum number of search iterations for each point 
    
elseif name_data == 'malaga_raw' 

elseif name_data == 'malaga_small' 
    
elseif name_data == 'malaga_large' 
    
elseif name_data == 'parking' 
    
elseif name_data == 'private' 
    
else 
    disp('Not a valid name!'); 
end 
% create a camera object 
params.camera = cameraParameters('IntrinsicMatrix', (params.K)'); 

return; 

end 