function [landmarks, I2_matched_kpts] = bootstrap(I2, I1, bootstrap_params, K)
%%BOOTSTRAP Initialization module to extract initial set of 2D-3D
%%correspondences from the initial frames of the sequence
% 
% INPUT:
%   - I2: frame at a time instant t+k (typically k = 1)
%   - I1: frame at a time instant t
%   - boostrap_params (struct): parameters for bootsrapping
%   - K (3, 3): intrinsic camera matrix
%
% Output: 
%   - landmarks(N, 3): 3D landmarks detected using input frames
%   - I2_matched_kpts(N, 2): corresponding keypoints to the landmarks in image 2
%
% NOTE: output points are in image's [u, v] notation

%% Step 1. Compute features using Harris corners detection
% compute the harris score and find the keypoints for prev_img
I1_harris_score = computeHarrisScores(I1, bootstrap_params.harris.patch_size, bootstrap_params.harris.kappa);
I1_keypoints = selectKeypoints(I1_harris_score, bootstrap_params.harris.num_keypoints, ...
                                bootstrap_params.harris.nonmaximum_supression_radius);

% convert pixel locations from [row, col] into [u, v]
I1_keypoints = fliplr(I1_keypoints);

%% Step 2. Match features using KLT tracking
tracker = vision.PointTracker('NumPyramidLevels', bootstrap_params.KLT.num_pyramid_levels, ...
                              'MaxBidirectionalError', bootstrap_params.KLT.max_bidirectional_error, ...
                              'BlockSize', bootstrap_params.KLT.block_size, ...
                              'MaxIterations', bootstrap_params.KLT.max_iterations);
initialize(tracker, I1_keypoints, I1);
[I2_keypoints, pts_matched] = tracker(I2);
release(tracker);

I1_matched_kps = I1_keypoints(pts_matched, :);
I2_matched_kpts = I2_keypoints(pts_matched, :);

%% Step 3. Apply RANSAC to avoid inital points on dynamic objects
% Number of points matched 
M = size(I1_matched_kps, 1);
% convert coordinates into homogenised system [u v] to [u v 1]
I1_matched_kps = [I1_matched_kps, ones(M, 1)];
I2_matched_kpts = [I2_matched_kpts, ones(M, 1)];

% parameters to perform RANSAC
min_inlier_count = bootstrap_params.RANSAC.min_inlier_points;
pixel_tolerance = bootstrap_params.RANSAC.pixel_tolerance;
num_iterations = bootstrap_params.RANSAC.num_iterations;
num_sampling_points = bootstrap_params.RANSAC.num_sampling_points;

% initialize variables for RANSAC
best_inlier_mask = zeros(M, 1);
max_num_inliers = 0;

% using RANSAC to find best R & T
for iter = 1: num_iterations
    % sample data from keypoints detected in frame 1 and 2
    [I1_kpts_sampled, idx] = datasample(I1_matched_kps, num_sampling_points, 'Replace', false);
    I2_kpts_sampled = I2_matched_kpts(idx, :);
    
    % compute Essential Matrix and decompose it to obtain relative pose of 
    % cam2 (I2) with respect to cam1 (I1)
    E = estimateEssentialMatrix(I1_kpts_sampled', I2_kpts_sampled', K, K);
    [Rots, u3] = decomposeEssentialMatrix(E);
    [R_21, t_21] = disambiguateRelativePose(Rots, u3, I1_kpts_sampled', I2_kpts_sampled', K, K);
    
    % compute the projection matrices for two camera poses 1 and 2
    M1 = K * eye(3,4);
    M2 = K * [R_21, t_21];
    
    % triangulate matched points to get 3D landmarks relative to cam1 
    X_c1 = linearTriangulation(I1_matched_kps', I2_matched_kpts', M1, M2);
    
    % compute reprojection error for landmarks in cam2
    pts2 = M2 * X_c1;
    pts2 = bsxfun (@rdivide, pts2, pts2(3,:));
    errors = sqrt(sum((I2_matched_kpts' - pts2).^2, 1));
    
    % calculate how many keypoints are within tolerance
    num_inlier = nnz(errors < pixel_tolerance);
    
    % check if the model is better 
    if num_inlier > max_num_inliers && num_inlier >= min_inlier_count
        max_num_inliers = num_inlier;        
        best_inlier_mask = errors < pixel_tolerance;
    end
end

% select the keypoints with most inliers 
I1_matched_kps = I1_matched_kps(best_inlier_mask, :);
I2_matched_kpts = I2_matched_kpts(best_inlier_mask, :);

%% Step 4. Triangulate filtered features to obtain initial 3D landmarks

% compute Essential Matrix and decompose it to obtain relative pose of 
% cam2 (I2) with respect to cam1 (I1)
E = estimateEssentialMatrix(I1_matched_kps', I2_matched_kpts', K, K);
[Rots, u3] = decomposeEssentialMatrix(E);
[R_21, t_21] = disambiguateRelativePose(Rots, u3, I1_matched_kps', I2_matched_kpts', K, K);

% compute the projection matrices for two camera poses 1 and 2
M1 = K * eye(3,4);
M2 = K * [R_21, t_21];

% triangulate matched points to get 3D landmarks relative to cam1 
X_c1 = linearTriangulation(I1_matched_kps',I2_matched_kpts', M1, M2);

% remove landmarks that are behind the camera
front_landmarks =  find(X_c1(3,:) > 0);
X_c1 = X_c1(1:3, front_landmarks);
I2_matched_kpts = I2_matched_kpts(front_landmarks, :);

%% convert computed landmarks and keypoints into desired shapes
landmarks = X_c1';
I2_matched_kpts = I2_matched_kpts(:, 1:2);

end
