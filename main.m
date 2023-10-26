clear; 
close all; 
clc; 

%% I. Initialization 
% add all folders into path 
addpath(genpath(pwd)); 
% data initialization 
name_data = 'kitti_0'; 
params = gen_param(name_data); 
params.list_img = params.list_img(1:end); 

% 3D landmarks initialization 
[p_0, p_1, pose_relative_init, params] = Feature_Match_KLT(params); % KLT feature point matching
img_init = imread_gray([params.list_img(1).folder, '\', ...
    params.list_img(1).name]); 
img_span = imread_gray([params.list_img(params.span_init).folder, '\', ...
    params.list_img(params.span_init).name]); 
[p_0_inlier, p_1_inlier, p_0_outlier, p_1_outlier, X] = ...
    init_3d(p_0, p_1, pose_relative_init, img_init, img_span, params); % 3D triangulation 

%% II. Continuous operation 
% initialize struct S 
S.X_old = X; 
S.P_prev_old = unique(round(p_0_inlier), 'rows')'; 
S.history_X = zeros(params.frame_gtVSvo, 1);
% relative camera pose of last 20 frames 
S.T_prev_crt = repmat(eye(3, 4), 1, params.frame_gtVSvo); 
% loop over all images 
img_prev = imread_gray([params.list_img(1).folder, '\', params.list_img(1).name]); 
figure(1); 
for idx_img = 2:1:length(params.list_img) 
    % previous frame 
    eval(['kpt_prev = detect', params.feature, 'Features(img_prev); ']); 
    % current frame 
    img_crt = imread_gray([params.list_img(idx_img).folder, '\', ...
        params.list_img(idx_img).name]); 
    [S] = processFrame(img_prev, img_crt, S, idx_img, params); 
    % update trajectory 
    [S] = compute_trajectory(S, params); 
    % visualization 
    visualization(img_prev, img_crt, S, params); 
    % come to the next frame 
    S.X_old = [S.X_old, S.X_new]; 
    S.P_prev_old = S.P_crt_all; 
    img_prev = img_crt; 
end 
