function [T_prev_crt, T_crt_prev, p_prev_obj, p_crt_obj] = compute_relativepose(p_prev, p_crt, params) 
%% Description 
% computing relative pose between two images. frame_crt -> frame_prev (WCS) 
%-Input: 
% p_prev: 2D keypoints of previous frame. size = [num_kpt, 2]. 
% p_crt: 2D keypoints of current frame. size = [num_kpt, 2].  
% params: structure parameter of the whole pipeline. 
%-Output: 
% T_prev_crt: transform current frame to previous frame. size = [3, 4] = [R|t]. 
% T_crt_prev: transform previous frame to current frame. size = [3, 4] = [R|t]. 
%% Implementation 
% extract relative pose (T_framek_frame0)
% Save the points in a cornerPoints object
p_prev_obj = cornerPoints(p_prev); 
p_crt_obj = cornerPoints(p_crt); 
% Estimate fundamental matrix 
[F, inliers_idx] = estimateFundamentalMatrix(p_prev_obj, ... 
    p_crt_obj,'Method',params.MethodForEstimation, 'NumTrials',params.NumTrials,...
    'DistanceThreshold',params.DistanceThreshold,'Confidence', params.Confidence); 
% Estimate relative camera pose, pose of camera relative to its previous pose
[R_t, T_t, valid_points_fraction]= relativeCameraPose(F, params.camera, ...
    p_prev_obj(inliers_idx, :), p_crt_obj(inliers_idx, :)); % only take the first solution 
T_prev_crt = [R_t(:,:,1)', T_t(1, :)']; % frame_crt -> frame_prev (WCS) 
T_crt_prev = pinv([T_prev_crt; [0, 0, 0, 1]]); 
T_crt_prev = T_crt_prev(1:3, :); % frame_prev -> frame_crt 

return; 

end 