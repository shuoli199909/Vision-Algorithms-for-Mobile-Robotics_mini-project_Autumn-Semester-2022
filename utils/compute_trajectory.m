function [S] = compute_trajectory(S, params) 
%% Description 
% update trajectory in the WCS. 
%-Input: 
% S: struct S contraining the state information. 
% params: structure parameter of the whole pipeline. 
%-Output: 
% S: struct S contraining the state information. 
%% Implementation 
% adjust step size according to the ground truth poses 
S.T_prev_crt(:, end) = S.T_prev_crt(:, end) * ...
    norm(params.traj_gt(:, S.frame - 1) - params.traj_gt(:, S.frame), 2); 
% frame_i -> frame_init 
T_init_frame = [S.T_prev_crt(:, 1:4)]; 
if S.frame <= params.frame_gtVSvo 
    % groundtruth location of initial frame 
    S.traj_vo = params.traj_gt(:, 1); 
    % frame_init -> WCS  
    T_wcs_init = reshape(params.poses(1, :), [4, 3])'; 
else 
    S.traj_vo = params.traj_gt(:, S.frame - params.frame_gtVSvo);
    T_wcs_init = reshape(params.poses(S.frame - params.frame_gtVSvo, :), [4, 3])'; 
end 
T_wcs_frame = T_wcs_init; 
% compute VO trajectory 
for frame_i = 2:1:params.frame_gtVSvo 
    % frame_crt -> frame_prev 
    T_tmp = [S.T_prev_crt(:, 4*frame_i-3:4*frame_i); [0, 0, 0, 1]]; 
    % frame_prev -> frame_init
    T_init_frame = [T_init_frame, T_init_frame(:, end-3:end) * T_tmp]; 
    % frame_init -> WCS 
    T_wcs_frame = [T_wcs_frame, T_wcs_init * ...
        [T_init_frame(:, end-3:end); [0, 0, 0, 1]]]; 
    S.traj_vo = [S.traj_vo, T_wcs_frame(:, end)]; 
end 
S.T_wcs_frame = T_wcs_frame; 

return; 

end 