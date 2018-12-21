function [landmarks, curr_matched_kpts] = bootstrap(curr_frame, prev_frame, bootstrap_params, K)
%%BOOTSTRAP Initialization module to extract initial set of 2D-3D
%%correspondences from the initial frames of the sequence
% 
% INPUT:
%   - curr_frame: frame at a time instant t+k (typically k = 1)
%   - prev_frame: frame at a time instant t
%   - boostrap_params (struct): parameters for bootsrapping
%   - K (3, 3): intrinsic camera matrix
%
% Output: 
%   - landmarks(N, 3): 3D landmarks detected using input frames
%   - curr_matched_kpts(N, 2): corresponding keypoints to the landmarks in image 2
%
% NOTE: output points are in image's [u, v] notation

%% Step 1. Compute features using Harris corners detection
prev_keypoints = computeHarrisFeatures(prev_frame, bootstrap_params.harris);

%% Step 2. Match features using KLT tracking
tracker = vision.PointTracker('NumPyramidLevels', bootstrap_params.KLT.num_pyramid_levels, ...
                              'MaxBidirectionalError', bootstrap_params.KLT.max_bidirectional_error, ...
                              'BlockSize', bootstrap_params.KLT.block_size, ...
                              'MaxIterations', bootstrap_params.KLT.max_iterations);
initialize(tracker, prev_keypoints, prev_frame);
[curr_keypoints, pts_matched] = tracker(curr_frame);
release(tracker);

prev_matched_kpts = prev_keypoints(pts_matched, :);
curr_matched_kpts = curr_keypoints(pts_matched, :);

%% Step 3. Apply RANSAC to avoid inital points on dynamic objects
[~, inliersIndex] = estimateFundamentalMatrix(prev_matched_kpts, curr_matched_kpts, ...
                        'Method','RANSAC', ...
                        'NumTrials', bootstrap_params.RANSAC.num_iterations, ...
                        'DistanceThreshold', bootstrap_params.RANSAC.pixel_tolerance);

% select the keypoints with most inliers 
prev_matched_kpts = prev_matched_kpts(inliersIndex, :);
curr_matched_kpts = curr_matched_kpts(inliersIndex, :);

% Number of points matched 
M = size(prev_matched_kpts, 1);
% convert coordinates into homogenised system [u v] to [u v 1]
prev_matched_kpts = [prev_matched_kpts, ones(M, 1)];
curr_matched_kpts = [curr_matched_kpts, ones(M, 1)];

%% Step 4. Triangulate filtered features to obtain initial 3D landmarks

% compute Essential Matrix and decompose it to obtain relative pose of 
% cam2 (I2) with respect to cam1 (I1)
E = estimateEssentialMatrix(prev_matched_kpts', curr_matched_kpts', K, K);
[Rots, u3] = decomposeEssentialMatrix(E);
[R_21, t_21] = disambiguateRelativePose(Rots, u3, prev_matched_kpts', curr_matched_kpts', K, K);

% compute the projection matrices for two camera poses 1 and 2
M1 = K * eye(3,4);          % assumption that initial point is starting coordinate
M2 = K * [R_21, t_21];

% triangulate matched points to get 3D landmarks relative to cam1 
X_C1 = linearTriangulation(prev_matched_kpts',curr_matched_kpts', M1, M2);

% remove landmarks that are behind the camera
front_landmarks =  find(X_C1(3,:) > 0);
X_C1 = X_C1(1:3, front_landmarks);
curr_matched_kpts = curr_matched_kpts(front_landmarks, :);

%% convert computed landmarks and keypoints into desired shapes
landmarks = X_C1';
curr_matched_kpts = curr_matched_kpts(:, 1:2);

end
