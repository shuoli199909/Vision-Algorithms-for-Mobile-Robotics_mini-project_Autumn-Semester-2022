function [list_img, poses, K] = data_initialization(name_data) 
%% Description 
% data initialization according to different names 
% name_data: string. different names correspond to different datasets. 
%          - 'kitti_0': kitti dataset, image_0 
%          - 'kitti_1': kitti dataset, image_1 
%          - 'malaga_raw': malaga dataset, raw images 
%          - 'malaga_small': malaga dataset, small images 
%          - 'malaga_large': malaga dataset, large images 
%          - 'parking': parking dataset 
%          - 'private': private dataset 
dir_current = pwd; 
dir_current = dir_current(1:end-4); 
if name_data == 'kitti_0' 
    dir_img = [dir_current, 'kitti\05\image_0\']; 
    list_img = dir([dir_img, '*.png']); 
    dir_poses = [dir_current, 'kitti\poses\05.txt']; 
    poses = load(dir_poses); 
    dir_K = [dir_current, 'kitti\05\calib.txt']; 
    K = textread(dir_K, '%s'); 
elseif name_data == 'kitti_1' 
    dir_img = [dir_current, 'kitti\05\image_1\']; 
    list_img = dir([dir_img, '*.png']); 
    dir_poses = [dir_current, 'kitti\poses\05.txt']; 
    poses = load(dir_poses); 
    dir_K = [dir_current, 'kitti\05\calib.txt']; 
    K = load(dir_K); 
elseif name_data == 'malaga_raw' 
    dir_img = [dir_current, 'malaga-urban-dataset-extract-07\Images\']; 
    list_img = dir([dir_img, '*.png']); 
    dir_poses = [dir_current, 'kitti\poses\05.txt']; 
    poses = load(dir_poses); 
    dir_K = [dir_current, 'kitti\05\calib.txt']; 
    K = load(dir_K); 
elseif name_data == 'malaga_small' 
    
elseif name_data == 'malaga_large' 
    
elseif name_data == 'parking' 
    
elseif name_data == 'private' 
    
else 
    disp('Not a valid name!'); 
end 

return; 

end 