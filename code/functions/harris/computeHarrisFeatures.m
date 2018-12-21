function [keypoints] = computeHarrisFeatures(image, harris_params)
%COMPUTEHARRISFEATURES Computes harris keypoints in input image
%
% INPUT: 
%   - image (H, W): input grayscale image
%   - harris_params(struct): parameters for harris keypoints detection
%
% OUTPUT:
%   - keypoints (M, 2): detected keypoints in [u, v] coordinates

harris_score = computeHarrisScores(image, harris_params.patch_size, harris_params.kappa);
keypoints = selectKeypoints(harris_score, harris_params.num_keypoints, ...
                                harris_params.nonmaximum_supression_radius);

% convert pixel locations from [row, col] into [u, v]
keypoints = fliplr(keypoints);

end

