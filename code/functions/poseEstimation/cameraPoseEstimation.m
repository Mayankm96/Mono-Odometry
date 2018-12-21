function [R_C_W, t_C_W, best_inlier_mask] = cameraPoseEstimation(keypoints, landmarks, K, p3p_params)
%%CAMERAPOSEESTIMATION Estimate the camera pose using P3P Algorithm
%
% INPUT:
%   - keypoints (M, 2): keypoints in image coordinates [u, v]
%   - landmarks (M, 3): corresponding 3D landmarks in world frame
%   - K (3, 3): Intrisic camera matrix
%   - p3p_params (struct): parameters for P3P algorithm with RANSAC
%
% OUTPUT:
%   - R_C_W (3, 3): rotation matrix from cam to the world frame
%   - t_C_W (3, 1): translation vector from cam to world frame
%   - best_inlier_mask (M, 1): 

% Define parameters for algorithm
num_iterations = p3p_params.num_iterations;
pixel_tolerance = p3p_params.pixel_tolerance;
min_inlier_count = p3p_params.min_inlier_count;

% In P3P: 3 points required
num_sample_points = 3;

% Number of input keypoints 
M = size(keypoints, 1);

% initialize variables for RANSAC
best_inlier_mask = zeros(M, 1);
max_num_inliers = 0;

% using RANSAC to find best R & T
for i = 1:num_iterations
    % sample keypoints and corresponding landmarks to estimate model
    [landmark_sample, idx] = datasample(landmarks, num_sample_points, 'Replace', false);
    keypoint_sample = keypoints(idx, :);
    
    % backproject keypoints to unit bearing vectors
    normalized_bearings = K \ [keypoint_sample'; ones(1, num_sample_points)];
    for index = 1:num_sample_points
        normalized_bearings(:, index) = normalized_bearings(:, index) / ...
            norm(normalized_bearings(:, index), 2);
    end
    
    % compute pose using P3P algorithm
    poses = p3p(landmark_sample', normalized_bearings);

    % disambigaute p3p output (two solutions possible)
    [R, t, is_inlier] = disambiguateP3P(keypoints, landmarks, poses, K, pixel_tolerance);
    
    if nnz(is_inlier) > max_num_inliers && nnz(is_inlier) >= min_inlier_count
        max_num_inliers = nnz(is_inlier);        
        best_inlier_mask = is_inlier;
        R_C_W = R;
        t_C_W = t;
    end
end

if max_num_inliers == 0
    R_C_W = [];
    t_C_W = [];
end

end
