function [p_0_inlier, p_1_inlier, p_0_outlier, p_1_outlier, X] = init_3d(p_0, p_1, pose_relative, img_0, img_1, params) 
%% Description 
%-Input: 
% params: structure parameter of the whole pipeline. 
%-Output: 
% p_0: feature point locations of the first frame. size = [num_point, 2].
% [u, v]. 
% p_1: corresponding feature point locations of the last frame in the
% initialization span. 
% pose_relative: relative pose between two frames. size = [3, 4] = [R|t]. 
%% Implementation 
% create objects 
p_0_obj = cornerPoints(p_0); 
p_1_obj = cornerPoints(p_1); 
% T_f0_fk -> T_fk_f0 (relative pose -> extrinsic parameters)
[R_fk_f0, t_fk_f0] = cameraPoseToExtrinsics(pose_relative(:, 1:3)', pose_relative(:, 4)');
camMatrix0 = cameraMatrix(params.camera, eye(3), zeros(3,1)); 
camMatrix1 = cameraMatrix(params.camera, R_fk_f0, t_fk_f0'); 
[X, errors, valid_idx] = triangulate(p_0_obj, p_1_obj, camMatrix0, camMatrix1); % triangulate 3D landmarks 
p_0_inlier = p_0(valid_idx, :); 
p_1_inlier = p_1(valid_idx, :); 
X = X(valid_idx, :)'; % size = [3, num_point] 
idx_filter = ((X(3, :) > 0) & (X(3, :) < params.Z_max)); % only preserve the points in front of the camera 
% if not valid, accept landmarks which are far away. 
if sum(idx_filter) == 0 
    idx_filter = (X(3, :) > 0); 
end 
X = X(:, idx_filter); 
p_0_inlier = p_0_inlier(idx_filter, :); 
p_1_inlier = p_1_inlier(idx_filter, :);
errors = errors((X(3, :) > 0) & (X(3, :) < params.Z_max)); 
p_0_outlier = setdiff(p_0, p_0_inlier, 'rows'); 
p_1_outlier = setdiff(p_1, p_1_inlier, 'rows'); 
% DEBUG visualization 
if params.init_visualization == true 
    figure(1); 
    % cameras 
    subplot(1,3,1); 
    scatter3(X(1,:), X(2,:), X(3,:),8, errors, 'filled'); 
    view(0, 0); 
    xlim([-2*max(X(1, :)), 2*max(X(1, :))]); xlabel('X'); 
    ylim([-2*max(X(2, :)), 2*max(X(2, :))]); ylabel('Y'); 
    zlim([0, 1.05*max(X(3, :))]); zlabel('Z'); hold on; 
    % plot cameras 
    plotCamera('Location',[0 0 0], 'Orientation',eye(3), ...
        'Label','Frame (previous)', 'AxesVisible', true); % previous frame 
    plotCamera('Location',pose_relative(:, 4), ...
        'Orientation',pose_relative(:, 1:3), ...
        'Label','Frame (current)', ...
        'AxesVisible', true); % current frame 
    axis equal
    rotate3d on;
    grid on; 
    % matched points 
    subplot(1,3,2); 
    imshow(img_0); hold on; 
    plot(p_0(:,1), p_0(:,2), 'g+');
    title('Image (previous)'); 
    subplot(1,3,3);
    imshow(img_1); hold on;
    plot(p_1(:,1), p_1(:,2), 'g+');
    title('Image (current)'); 
% DEBUG visualization 

end 

return; 

end 