function [state, pose, num_p3p_inliers] = processFrame(curr_frame, prev_frame, ...
                                            prev_state, K, process_params)
%%PROCESSFRAME Perform continuous VO pipeline with a markov assumption over
%%the state of the VO algorithm in which new landmarks are added to
%%maintain consistency
% 
% INPUT:
%   - curr_frame(H, W): grey scale image matrix at time t
%   - prev_frame(H, W): grey scale image matrix at time t-1
%   - prev_state(struct):  inculdes state.of the pipeline at t-1
%       - P(M, 2): keypoints in image at t-1 in [u, v] coordinates
%       - X(M, 3): asssociated landmarks at t-1
%       - C(M, 2): candidate keypoints 
%   - K(3, 3): camera intrinsic matrix
%   - params: struct includes all parameters under "process" scope
%
% OUTPUT:
%   - state(struct): inculdes state.P(K, 2) & state.X(K, 3)
%   - pose (3, 4): pose of camera with respect to world frame
%   - num_p3p_inliers: number of inliers in P3P using RANSAC

%% Construct the state S strcut
% P: 2D tracked keypoints array (N, 2)
% X: 3D tracked landmarks array (N, 3)
state = struct('P', [], 'X', [], 'C', [], 'T', [], 'F', []);

%% Step 1: Feature Tracking (KLT)
% construct KLT object
tracker = vision.PointTracker('NumPyramidLevels', process_params.KLT.num_pyramid_levels, ...
                              'MaxBidirectionalError', process_params.KLT.max_bidirectional_error, ...
                              'BlockSize', process_params.KLT.block_size, ...
                              'MaxIterations', process_params.KLT.max_iterations);
                          
% compute the tracked keypoints in current frame from previous frame
initialize(tracker, prev_state.P, prev_frame);
[curr_keypts, tracked_indices] = tracker(curr_frame);
release(tracker);

% copy tracked keypoints and associated landmarks into the current state 
state.P = curr_keypts(tracked_indices, :);
state.X = prev_state.X(tracked_indices, :);

% plot matching for sanity check!
% figure(2);
% % false color
% imgOverlay = imfuse(prev_frame, curr_frame);
% create red-cyan image instead of the imfuse default
% imgOverlay(:,:,1) = imgOverlay(:,:,2);
% imgOverlay(:,:,2) = imgOverlay(:,:,3);
% imshow(imgOverlay);
% hold on;
% plot([state.P(:, 1)'; prev_state.P(tracked_indices, 1)'], [state.P(:, 2)'; prev_state.P(tracked_indices, 2)'], '-y');
% plot(prev_state.P(tracked_indices, 1), prev_state.P(tracked_indices, 2), '+r', 'LineWidth', 2);
% plot(state.P(:, 1), state.P(:, 2), '+g', 'LineWidth', 2);

%% Step 2: Camera Pose Estimation (PnP)
% rotation and translation from camera to world frame
[R_C2_W, t_C2_W, p3p_inlier_mask] = cameraPoseEstimation(state.P, state.X, K, process_params.p3p);
T_C2_W = [R_C2_W, t_C2_W];
num_p3p_inliers = nnz(p3p_inlier_mask);

% camera pose with respect to world 
pose = [R_C2_W', - R_C2_W' * t_C2_W];

% Remove landmarks that lie behind the camera
points_3D = R_C2_W * state.X' + t_C2_W;
state.X = state.X(points_3D(3, :) > 0, :);
state.P = state.P(points_3D(3, :) > 0, :);

%% Step 3: Triangulating new landmarks from candidate keypoints in previous state

if ~isempty(prev_state.C)
    
    % compute the tracked keypoints in current frame for candidate points
    initialize(tracker, prev_state.C, prev_frame);
    [pts_C, tracked_indices] = tracker(curr_frame);
    release(tracker)
    
    % copy candidate keypoints and poses into the current state 
    state.F = prev_state.C(tracked_indices, :);
    state.T = prev_state.T(tracked_indices, :);
    state.C = pts_C(tracked_indices, :);
    
    % sanity check!
    assert(size(state.C, 1) == size(state.F, 1));
    
    if ~isempty(state.C)
        
        % count number of candidate keypoits present 
        N = size(state.C, 1);

        % create array for bookeeping candidates added to landmarks and
        % candidate points which are no longer visible to current camera
        bookkeeping = false(N, 1);
        
        % iterate over each candidate keypoint
        for i = 1:N
            % compute transformation from world to cam
            T_C1_W = reshape(state.T(i, :), [3, 4]);

            % compute projection matrices
            M1 = K * T_C1_W;
            M2  = K * T_C2_W;
            
            % convert to homogenous coordinates
            keypt_C1 = [state.F(i, :)'; 1];
            keypt_C2 = [state.C(i, :)'; 1];
            
            % triangulate 3D points using pose from current and first observation
            X_W = linearTriangulation(keypt_C1, keypt_C2, M1, M2);
            
            % compute location of landmark in current and first
            % observations
            X_C2 = T_C2_W(:, 1:3) * X_W(1:3, :) + T_C2_W(:, 4);
            X_C1 = T_C1_W(:, 1:3) * X_W(1:3, :) + T_C1_W(:, 4);

            % check if 3D point is infront of the camera 
            if X_C2(3) > 0                
                % compute angle between bearing vectors to landmark
                alpha = atan2(norm(cross(X_C1, X_C2)), ...
                              dot(X_C1, X_C2));
                          
                % add triangulated landmark to state if angle is above thrshold
                if abs(alpha) >= process_params.landmarks.bearing_threshold
                    state.X = [state.X; X_W(1:3, :)'];
                    state.P = [state.P; keypt_C2(1:2, :)'];
                    bookkeeping(i) = true;
                end
            else
                bookkeeping(i) = true;
            end
        end
        
        % remove candidate points which have been added
        state.F = state.F(~bookkeeping, :);
        state.T = state.T(~bookkeeping, :);
        state.C = state.C(~bookkeeping, :);
    end
end

%% Step 4: Add new candidate points 

% Step 4a. Compute features using Harris corners detection (similar to bootstrapping)
prev_keypoints = computeHarrisFeatures(prev_frame, process_params.harris);

% Step 4b. Match features using KLT tracking
tracker = vision.PointTracker('NumPyramidLevels', process_params.KLT.num_pyramid_levels, ...
                              'MaxBidirectionalError', process_params.KLT.max_bidirectional_error, ...
                              'BlockSize', process_params.KLT.block_size, ...
                              'MaxIterations', process_params.KLT.max_iterations);
initialize(tracker, prev_keypoints, prev_frame);
[curr_keypoints, pts_matched] = tracker(curr_frame);
release(tracker);

prev_matched_kpts = prev_keypoints(pts_matched, :);
curr_matched_kpts = curr_keypoints(pts_matched, :);

% Step 4c. Check if candidate points already exist in state.P
M = size(curr_matched_kpts, 1);
bookkeeping = false(M, 1);
dist_candidate_keypts = pdist2(curr_matched_kpts, state.P);

for i = 1:M
    if all(dist_candidate_keypts(i, :) > process_params.new_candidate_tolerance)
        bookkeeping(i) = true;
    end
end

% Step 4d: Update state with candidates
state.C = [state.C; curr_matched_kpts(bookkeeping, :)];
state.F = [state.F; prev_matched_kpts(bookkeeping, :)];
state.T = [state.T; repmat(reshape(T_C2_W, [1, 12]), [nnz(bookkeeping), 1]) ];

% status display
fprintf('\n----------- Summary -----------');
fprintf('\n[Previous] Total keypoints: %d, Total candidates: %d', length(prev_state.P), length(prev_state.C));
fprintf('\n[Current] Total keypoints: %d, Total candidates: %d', length(state.P), length(state.C));
fprintf('\n-------------------------------\n');

end
