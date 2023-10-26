function [ ] = visualization(img_prev, img_crt, S, params) 
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
% Visualization 
set(gcf,'outerposition',get(0,'screensize'));
sct = subplot(2,2,1); 
cla; 
% prev -> WCS 
X_old_wcs = S.T_wcs_frame(:, end-7:end-4) * [S.X_old;ones(1, width(S.X_old))]; 
X_new_wcs = S.T_wcs_frame(:, end-7:end-4) * [S.X_new;ones(1, width(S.X_new))]; 
scatter3(X_old_wcs(1,:), X_old_wcs(2,:), X_old_wcs(3,:), 10, ...
    [0 0.4470 0.7410], 'filled'); 
scatter3(X_new_wcs(1,:), X_new_wcs(2,:), X_new_wcs(3,:), 10, ...
    [0.6350 0.0780 0.1840], 'filled'); 
plot3(S.traj_vo(1, :), S.traj_vo(2, :), S.traj_vo(3, :), ...
    '-' , 'Color' , 'b' , 'MarkerSize' ,8); 
title(['3D Visualization. Frame: ', num2str(S.frame), '/', ...
    num2str(length(params.list_img)), '. ', num2str(S.SecPerFrame), ' sec/frame.']); 
view(0, 0); 
xlim([min(S.traj_vo(1, :)) - 50, max(S.traj_vo(1, :) + 50)]); xlabel('X'); 
ylim([min(S.traj_vo(2, :)) - 50, max(S.traj_vo(2, :) + 50)]); ylabel('Y'); 
zlim([min(S.traj_vo(3, :)) - 50, max(S.traj_vo(3, :) + 50)]); zlabel('Z');
hold on; rotate3d on; grid off;  
legend('Old Landmarks', 'New landmarks'); 
% plot cameras 
%plotCamera('Location', S.T_wcs_frame(:, end), ...
%    'Orientation', [S.T_wcs_frame(:, end-3:end-2), ...
%    round(S.T_wcs_frame(:, end-1))], 'Label','Frame (current)', ...
%    'AxesVisible', true); % current frame 


% matched points 
mp = subplot(2,2,2); 
cla; 
showMatchedFeatures(img_prev, img_crt, S.P_prev_all', S.P_crt_all', 'falsecolor'); hold on; 
title('Image (previous v.s. current)');   

% trajectory
traj = subplot(2,2,3); 
cla; 
if S.frame <= params.frame_gtVSvo 
    plot3(params.traj_gt(1, 1:S.frame), params.traj_gt(2, 1:S.frame), ...
        params.traj_gt(3, 1:S.frame), ...
        S.traj_vo(1, :), S.traj_vo(2, :), S.traj_vo(3, :)); 
else 
    plot3(params.traj_gt(1, S.frame-params.frame_gtVSvo:S.frame), ...
        params.traj_gt(2, S.frame-params.frame_gtVSvo:S.frame), ...
        params.traj_gt(3, S.frame-params.frame_gtVSvo:S.frame), ...
        S.traj_vo(1, :), S.traj_vo(2, :), S.traj_vo(3, :)); 
end
legend('Ground Truth', 'Vision Odometry'); 
view(0, 0); 
xlim([min(S.traj_vo(1, :)) - 5, max(S.traj_vo(1, :) + 5)]); xlabel('X'); 
ylim([min(S.traj_vo(2, :)) - 5, max(S.traj_vo(2, :) + 5)]); ylabel('Y'); 
zlim([min(S.traj_vo(3, :)) - 5, max(S.traj_vo(3, :) + 5)]); zlabel('Z'); hold on;
title('Trajectory (groundtruth v.s. VO)');   

% tracked landmarks 
subplot(2,2,4); 
cla; 
plot(1-params.frame_gtVSvo:1:0, S.history_X); 
title(['# tracked landmarks over last ', num2str(params.frame_gtVSvo)...
       ' frames.']); 

return; 

end 