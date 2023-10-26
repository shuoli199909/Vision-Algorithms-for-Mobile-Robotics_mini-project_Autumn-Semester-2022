function [kpt] = detectFeatures_VO(img, params)
%% Description 
%-Input: 
% img: input image. size = [H, W]. 
% params: structure parameter of the whole pipeline. 
%-Output: 
% Location: location of detected points. size = [num_point, 2]. 
%% Implementation 
% find the corners according to different ROI. 
kpt = []; 
num_kpt_roi = round(params.num_kpt/length(params.list_ROI));
for roi_tmp = 1:1:length(params.list_ROI) 
    eval(['points_tmp = detect', params.feature, 'Features(img, ', ...
          char("'MinQuality', "), '0, ', char("'ROI', "), 'params.list_ROI(roi_tmp, :))']); 
    kpt = [kpt.selectStrongest(num_kpt_roi); points_tmp]; 
end 

if params.visualization_roi == true 
    figure(); 
    subplot(1,2,1); 
    eval(['kpt_whole = detect', params.feature, 'Features(img)']); 
    imshow(img); hold on; 
    plot(kpt_full.Location(:,1), kpt_full.Location(:,2), 'ro'); 
    subplot(1,2,2); 
    imshow(img); hold on; 
    plot(kpt.Location(:,1), kpt.Location(:,2), 'bx'); 
    legend('Keypoint detection in the whole image', 'Keypoint detection separately'); 
end 

return; 

end 