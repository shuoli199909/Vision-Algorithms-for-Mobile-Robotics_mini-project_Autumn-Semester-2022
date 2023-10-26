function [img_gray] = imread_gray(loc_img) 
%% Description 
% reading an image according to the location. the output should be a
% grayscale image. 
%-Input: 
% loc_img: location of the image. 
%-Output: 
% img_gray: grayscale image. size = [H, W]. 
%% Implementation 
img_gray = imread(loc_img); 
size_img = size(img_gray); 
if numel(size_img) == 3 
    img_gray = rgb2gray(img_gray); 
end 

return; 

end 