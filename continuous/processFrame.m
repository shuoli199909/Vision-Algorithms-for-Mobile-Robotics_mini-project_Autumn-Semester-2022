function [S] = processFrame(img_prev, img_crt, S, idx_img, params) 
%% Description 
%-Input: 
% img_prev: previous image. size = [H, W]. 
% img_crt: current image. size = [H, W]. 
% S: state of the previous frame. 
% params: structure parameter of the whole pipeline. 
%-Output: 
% S: struct S contraining 2D keypoints and 3D landmarks. 
%% Implementation 
tic; % computing time 
% KLT tracker 
% detect features of previous frame 
eval(['kpt_prev = detect', params.feature, 'Features(img_prev); ']); 
tracker_KLT = vision.PointTracker('MaxBidirectionalError', params.MaxBidirectionalError, ...
                                  'NumPyramidLevels', params.NumPyramidLevels, ...
                                  'BlockSize', params.BlockSize, ...
                                  'MaxIterations', params.MaxIterations);
initialize(tracker_KLT, kpt_prev.Location, img_prev); 
[kpt_crt,validity_crt] = tracker_KLT(img_crt); 

% Update struct S 
% update 2D keypoints of previous frame & current frame 
S.P_prev_all = round(kpt_prev.Location(validity_crt, :)'); 
S.P_crt_all = round(kpt_crt(validity_crt, :)'); 
% delete repeated coordinates 
P_prev_crt_all = [S.P_prev_all; S.P_crt_all]; 
[~, idx_unique_prev, ~] = unique(P_prev_crt_all(1:2, :)', 'rows'); 
P_prev_crt_all = P_prev_crt_all(:, idx_unique_prev); 
[~, idx_unique_crt, ~] = unique(P_prev_crt_all(3:4, :)', 'rows'); 
P_prev_crt_all = P_prev_crt_all(:, idx_unique_crt); 
S.P_prev_all = P_prev_crt_all(1:2, :); 
S.P_crt_all = P_prev_crt_all(3:4, :); 
% find triangulated landmarks in all detected feature points   
idx_all2old = ismember(S.P_prev_all', S.P_prev_old', 'rows'); 
P_prev_old = S.P_prev_all(:, idx_all2old); 
idx_updateX = ismember(S.P_prev_old', P_prev_old', 'rows'); 
S.X_old = S.X_old(:, idx_updateX); 
% old feature points (triangulated) in previous frame 
S.P_prev_old = P_prev_old; 
% new feature points (untriangulated) in previous frame 
S.P_prev_new = S.P_prev_all(:, ~idx_all2old); 
% old feature points (triangulated) in current frame 
S.P_crt_old = S.P_crt_all(:, idx_all2old); 
% new feature points (untriangulated) in current frame 
S.P_crt_new = S.P_crt_all(:, ~idx_all2old); 

% Estimation of current pose and triangulation 
% estimating the current pose 
[T_prev_crt, T_crt_prev, ~, ~] = compute_relativepose(S.P_prev_all', S.P_crt_all', params); 
% triangulate new 3D landmarks 
[P_prev_new_inlier, P_crt_new_outlier, ~, ~, X_new] = init_3d(S.P_prev_new', ...
    S.P_crt_new', T_prev_crt, img_prev, img_crt, params); 

% Update struct S 
S.X_old = T_crt_prev * [S.X_old; ones(1, width(S.X_old))]; 
S.X_new = T_crt_prev * [X_new; ones(1, width(X_new))]; 
S.P_prev_new = round(P_prev_new_inlier'); 
S.P_crt_new = round(P_crt_new_outlier'); 
S.P_prev_all = [S.P_prev_old, S.P_prev_new]; 
S.P_crt_all = [S.P_crt_old, S.P_crt_new]; 
% relative pose estimation 
S.T_prev_crt(:, 1:end-4) = S.T_prev_crt(:, 5:end); 
S.T_prev_crt(:, end-3:end) = T_prev_crt; % frame_crt -> frame_prev (WCS)
S.T_crt_prev = T_crt_prev; % frame_prev -> frame_crt 
S.SecPerFrame = toc; 
S.frame = idx_img; % current frame 
S.history_X = [S.history_X; width(S.X_old) + width(S.X_new)]; 
S.history_X = S.history_X(2:end);

return; 

end 