function [p_1, p_2] = Feature_Match(img_1, img_2, feature, Method, ...
                      NumTrials, DistanceThreshold, Confidence) 
%% Description 
%-Input: 
% img_1, img_2: size = [H, W]. Input grayscale images to find 
% correspondences. 
% feature: size = char(). Feature type we want to use to extract features. 
%          'BRISK':      detectBRISKFeatures 
%          'FAST':       detectFASTFeatures 
%          'Harris':     detectHarrisFeatures 
%          'KAZE':       detectKAZEFeatures 
%          'MinEigen':   detectMinEigenFeatures 
%          'MSER':       detectMSERFeatures 
%          'ORB':        detectORBFeatures 
%          'SIFT':       detectSIFTFeatures 
%          'SURF':       detectSURFFeatures 
% Method: size = char(). methods of feature points refinement. 
%          'LMedS':      Least Median of Squares
%          'MSAC':       M-estimator SAmple Consensus 
%          'Norm8Point': Normalized eight-point algorithm 
% NumTrials: size = [1]. number of random trials for finding outliers 
% DistanceThreshold: size = [1]. distance threshold for finding outliers 
% Confidence: size = [1]. desired confidence for finding maximum number of inliers 
%% Implementation 
% find the corners. 
eval(['points1 = detect', feature, 'Features(img_1); ']); 
eval(['points2 = detect', feature, 'Features(img_2); ']); 
% extract the neighborhood features 
[features1, valid_points1] = extractFeatures(img_1, points1); 
[features2, valid_points2] = extractFeatures(img_2, points2); 
% match the features 
indexPairs = matchFeatures(features1,features2, Unique=true); 
% retrieve the locations of the corresponding points for each image 
matchedPoints1 = valid_points1(indexPairs(:,1),:); 
matchedPoints2 = valid_points2(indexPairs(:,2),:); 
% find inliers 
[~, inliers] = estimateFundamentalMatrix(matchedPoints1, ...
                    matchedPoints2, Method=Method, NumTrials=NumTrials, ...
                    DistanceThreshold=DistanceThreshold, Confidence=Confidence); 
% macthed points after refinement 
p_1 = valid_points1(inliers); 
p_2 = valid_points2(inliers); 

return; 

end