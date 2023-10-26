function [params] = gen_param(name_data) 
%% Description 
% generating the parameters of the whole pipeline 
%-Input: 
% name_data: string. different names correspond to different datasets. 
%          - 'kitti_0': kitti dataset, image_0 
%          - 'kitti_1': kitti dataset, image_1 
%          - 'malaga_raw': malaga dataset, raw images 
%          - 'malaga_small': malaga dataset, small images 
%          - 'malaga_large': malaga dataset, large images 
%          - 'parking': parking dataset 
%          - 'private': private dataset 
%-Output: 
% list_img: directories of images in the dataset. size = [num_img, 1] 
% poses: extrinsic matrix. [R|t]. size = [num_img, 12]. to reshape it, we 
%        should use reshape(pose_current, [4,3])'. and the we can get the 
%        [3,4] matrix. 
% K: intrinsic matrix, size = [3, 4]. 
%% Implementation 
% dataset parameters 
[list_img, poses, traj_gt, K] = data_initialization(name_data); 
params.dataset = name_data; 
params.list_img = list_img; 
params.poses = poses; 
params.traj_gt = traj_gt; 
params.K = K(:, 1:3); % intrinsic matrix. [3, 4] -> [3, 3] 
% DEBUG settings 
params.init_visualization = false; % initial visualization of triangulation 
params.associate_visualization = true; % visualization of association 
% different settings for different datasets 
if strcmp(name_data, 'kitti_0') == 1 
    % KLT tracker parameters 
    params.feature = 'Harris'; % descriptors 
    params.span_init = 6; % span of frames used for initialization 
    params.MaxBidirectionalError = 0.8; % maximal bidirectional error 
    params.NumPyramidLevels = 4; % number of pyramid level
    params.BlockSize = [23, 23];  % size of neighborhood around each point being tracked
    params.MaxIterations = 50; % maximum number of search iterations for each point 
    % relative pose estimation  
    params.MethodForEstimation = 'LMedS'; 
    % MethodForEstimation: size = char(). methods of feature points refinement. 
    %          'LMedS':      Least Median of Squares
    %          'MSAC':       M-estimator SAmple Consensus 
    %          'Norm8Point': Normalized eight-point algorithm 
    params.NumTrials = 1000; % number of trials 
    params.DistanceThreshold = 0.015; % treshold of distance 
    params.Confidence = 92; % confidence of matching 
    params.Z_max = 120; 
    % frames for visualization  
    params.frame_gtVSvo = 40; 
    % ROI shape 
    params.ROI = [2, 2]; 
    % maximum number of feature points per image 
    params.num_kpt = 200; 
elseif strcmp(name_data, 'kitti_1') == 1 
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
    params.Z_max = 120; 
    % frames for visualization  
    params.frame_gtVSvo = 20; 
    % ROI shape 
    params.ROI = [2, 2]; 
elseif strcmp(name_data, 'malaga_raw') == 1 

elseif strcmp(name_data, 'malaga_small') == 1 
    % KLT tracker parameters 
    params.feature = 'SURF'; % descriptors 
    params.span_init = 6; % span of frames used for initialization 
    params.MaxBidirectionalError = 0.8; % maximal bidirectional error 
    params.NumPyramidLevels = 4; % number of pyramid level
    params.BlockSize = [21, 21];  % size of neighborhood around each point being tracked
    params.MaxIterations = 50; % maximum number of search iterations for each point 
    % relative pose estimation  
    params.MethodForEstimation = 'LMedS'; 
    % MethodForEstimation: size = char(). methods of feature points refinement. 
    %          'LMedS':      Least Median of Squares
    %          'MSAC':       M-estimator SAmple Consensus 
    %          'Norm8Point': Normalized eight-point algorithm 
    params.NumTrials = 1000; % number of trials 
    params.DistanceThreshold = 0.03; % treshold of distance 
    params.Confidence = 70; % confidence of matching 
    params.Z_max = 400; 
    % ROI shape 
    params.ROI = [2, 2]; 
elseif strcmp(name_data, 'malaga_large') == 1 
    
elseif strcmp(name_data, 'parking') == 1 
    % KLT tracker parameters 
    params.feature = 'Harris'; % descriptors 
    params.span_init = 6; % span of frames used for initialization 
    params.MaxBidirectionalError = 0.8; % maximal bidirectional error 
    params.NumPyramidLevels = 4; % number of pyramid level
    params.BlockSize = [21, 21];  % size of neighborhood around each point being tracked
    params.MaxIterations = 50; % maximum number of search iterations for each point 
    % relative pose estimation  
    params.MethodForEstimation = 'LMedS'; 
    % MethodForEstimation: size = char(). methods of feature points refinement. 
    %          'LMedS':      Least Median of Squares
    %          'MSAC':       M-estimator SAmple Consensus 
    %          'Norm8Point': Normalized eight-point algorithm 
    params.NumTrials = 1000; % number of trials 
    params.DistanceThreshold = 0.015; % treshold of distance 
    params.Confidence = 90; % confidence of matching 
    params.Z_max = 200; 
    % ROI shape 
    params.ROI = [2, 2]; 
elseif strcmp(name_data, 'private') == 1 
    % KLT tracker parameters 
    params.feature = 'Harris'; % descriptors 
    params.span_init = 6; % span of frames used for initialization 
    params.MaxBidirectionalError = 0.8; % maximal bidirectional error 
    params.NumPyramidLevels = 4; % number of pyramid level
    params.BlockSize = [21, 21];  % size of neighborhood around each point being tracked
    params.MaxIterations = 50; % maximum number of search iterations for each point 
    % relative pose estimation  
    params.MethodForEstimation = 'LMedS'; 
    % MethodForEstimation: size = char(). methods of feature points refinement. 
    %          'LMedS':      Least Median of Squares
    %          'MSAC':       M-estimator SAmple Consensus 
    %          'Norm8Point': Normalized eight-point algorithm 
    params.NumTrials = 1000; % number of trials 
    params.DistanceThreshold = 0.015; % treshold of distance 
    params.Confidence = 90; % confidence of matching 
    % ROI shape 
    params.ROI = [2, 2]; 
else 
    disp('Not a valid name!'); 
end 
% create a camera object 
params.camera = cameraParameters('IntrinsicMatrix', (params.K)'); 
% separately detect feature points according to ROI shape 
params.visualization_roi = false; 
list_ROI = []; 
[H, W, ~] = size(imread(params.list_img(1))); 
H_step = floor(H/params.ROI(1)); 
W_step = floor(W/params.ROI(2)); 
for h = 0:1:(size(params.ROI, 1) - 1) 
    for w = 0:1:(size(params.ROI, 2) - 1) 
        roi_tmp = [w*W_step, h*H_step, W_step, H_step]; 
        list_ROI = [list_ROI; roi_tmp]; 
    end 
end 
params.list_ROI = list_ROI; 

return; 

end 