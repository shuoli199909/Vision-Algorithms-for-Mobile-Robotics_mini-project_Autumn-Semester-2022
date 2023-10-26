function [P] = init_3d(p_0, p_1, p_0_obj, p_1_obj, pose_relative, params) 
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
% T_f0_fk -> T_fk_f0 (relative pose -> extrinsic parameters)
[R_fk_f0, t_fk_f0] = cameraPoseToExtrinsics(pose_relative(:, 1:3)', pose_relative(:, 4)');
camMatrix0 = cameraMatrix(params.camera, eye(3), zeros(3,1)); 
camMatrix1 = cameraMatrix(params.camera, R_fk_f0, t_fk_f0'); 
[P, errors] = triangulate(p_0_obj, p_1_obj, camMatrix0, camMatrix1); % triangulate 3D landmarks 
P = P'; % size = [3, num_point] 
P = P(:, (P(3, :) > 0) & (P(3, :) < 120)); % only preserve the points in front of the camera 
errors = errors((P(3, :) > 0) & (P(3, :) < 120));
% initial visualization of triangulation 
if params.init_visualization == true 
    figure(); 
    % cameras 
    subplot(1,3,1); 
    scatter3(P(1,:), P(2,:), P(3,:),8, errors, 'filled'); 
    view(0, 0); 
    xlim([-2*max(P(1, :)), 2*max(P(1, :))]); 
    ylim([-2*max(P(2, :)), 2*max(P(2, :))]); 
    zlim([0, 1.05*max(P(3, :))]); hold on; 
    % plot cameras 
    plotCamera('Location',[0 0 0], 'Orientation',eye(3), ...
        'Label','Frame_0', 'AxesVisible', true); % frame 0 
    plotCamera('Location',pose_relative(:, 4), ...
        'Orientation',pose_relative(:, 1:3), ...
        'Label',['Frame_', num2str(params.span_init)], ...
        'AxesVisible', true); % frame span_init 
    axis equal
    rotate3d on;
    grid on; 
    % matched points 
    subplot(1,3,2); 
    img_init = imread([params.list_img(1).folder, '\', params.list_img(1).name]); 
    imshow(img_init); hold on; 
    plot(p_0(:,1), p_0(:,2), 'g+');
    title('Image_0'); 
    subplot(1,3,3);
    img_span = imread([params.list_img(params.span_init ).folder, '\', params.list_img(params.span_init ).name]); 
    imshow(img_span); hold on;
    plot(p_1(:,1), p_1(:,2), 'g+');
    title(['Image_', num2str(params.span_init)]); 

end 

return; 

end 