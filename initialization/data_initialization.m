function [list_img, poses, traj_gt, K] = data_initialization(name_data) 
%% Description 
% data initialization according to different names 
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
% poses: poses (frame -> WCS). size = [num_frame, 12]. 
% traj_gt: groundtruth trajectory. size = [3, num_frame]. 
% K: intrinsic matrix, size = [3, 4]. 
%% Implementation 
dir_current = matlab.desktop.editor.getActiveFilename; 
dir_current = dir_current(1:end-11); 
if strcmp(name_data, 'kitti_0') == 1 
    dir_img = [dir_current, 'kitti\05\image_0\']; 
    list_img = dir([dir_img, '*.png']); % location of images 
    dir_poses = [dir_current, 'kitti\poses\05.txt']; 
    poses = load(dir_poses); 
    traj_gt = []; % groundtruth trajectory 
    for f_tmp = 1:1:length(poses) 
        traj_tmp = reshape(poses(f_tmp, :), [4, 3])'; 
        traj_gt = [traj_gt, traj_tmp(:, 4)]; 
    end 
    dir_K = [dir_current, 'kitti\05\calib.txt']; 
    K_tmp = textread(dir_K, '%s', 'delimiter', ' '); 
    K = []; % camera parameters 
    for i = 2:1:13 
        K = [K, str2double(K_tmp{i})]; 
    end
    K = reshape(K, [4, 3])';
elseif strcmp(name_data, 'kitti_1') == 1 
    dir_img = [dir_current, 'kitti\05\image_1\']; 
    list_img = dir([dir_img, '*.png']); 
    dir_poses = [dir_current, 'kitti\poses\05.txt']; 
    dir_K = [dir_current, 'kitti\05\calib.txt']; 
    K_tmp = textread(dir_K, '%s', 'delimiter', ' '); 
    K = []; 
    for i = 15:1:26 
        K = [K, str2double(K_tmp{i})]; 
    end
    K = reshape(K, [4, 3])';
elseif strcmp(name_data, 'malaga_raw') == 1 
    dir_img = [dir_current, 'malaga-urban-dataset-extract-07\Images\']; 
    list_img = dir([dir_img, '*.png']); 
    dir_poses = [dir_current, 'kitti\poses\05.txt']; 
    dir_K = [dir_current, 'kitti\05\calib.txt']; 
    K = load(dir_K); 
    K = [K, [0;0;0]]; 
elseif strcmp(name_data, 'malaga_small') == 1 
    dir_img = [dir_current, ['malaga-urban-dataset-extract-07\'...
               'malaga-urban-dataset-extract-07_rectified_800x600_Images\']]; 
    list_img = dir([dir_img, 'img_CAMERA1_*_left.jpg']); 
    K = [621.18428, 0,         404.0076, 0;
         0,         621.18428, 309.05989,0; 
         0,         0,         1,        0;]; 
elseif strcmp(name_data, 'malaga_large') == 1 
    
elseif strcmp(name_data, 'parking') == 1 
    dir_img = [dir_current, 'parking\images\']; 
    list_img = dir([dir_img, 'img_*.png']); 
    dir_poses = [dir_current, 'parking\poses.txt']; 
    poses = load(dir_poses); 
    dir_K = [dir_current, 'parking\K.txt']; 
    K = load(dir_K); 
    K = [K, [0;0;0]]; 
elseif strcmp(name_data, 'private') == 1 
    
else 
    disp('Not a valid name!'); 
end 

return; 

end 