function [p_0, p_1, pose_relative, params] = Feature_Match_KLT(params) 
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
% find the corners. 
img_init = imread_gray([params.list_img(1).folder, '\', params.list_img(1).name]); 
eval(['points_init = detect', params.feature, 'Features(img_init); ']); 
% extract the neighborhood features 
[~, valid_points_init] = extractFeatures(img_init, points_init); 
% initialize KLT tracker 
tracker_KLT = vision.PointTracker('MaxBidirectionalError', params.MaxBidirectionalError, ...
                                  'NumPyramidLevels', params.NumPyramidLevels, ...
                                  'BlockSize', params.BlockSize, ...
                                  'MaxIterations', params.MaxIterations);
initialize(tracker_KLT,valid_points_init.Location,img_init); 
% start KLT tracking 
for frame_tmp = 1:1:params.span_init 
    img_tmp = imread_gray([params.list_img(frame_tmp).folder, '\', params.list_img(frame_tmp).name]); 
    [points,validity] = tracker_KLT(img_tmp); 
end 
p_0 = valid_points_init.Location(validity, :); 
p_1 = points(validity, :); 
% extract relative pose (T_framek_frame0)
[pose_relative, p_0_obj, p_1_obj] = compute_relativepose(p_0, p_1, params); 

return; 

end 