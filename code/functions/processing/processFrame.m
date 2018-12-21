function [state, pose, p3p_inlier_mask, tracked_indices] = processFrame(curr_frame, prev_frame, ...
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
%   - state(struct): inculdes state.P(2xk) & state.X(3xk)
%   - pose (3, 4): an array to represent transform from camera to world
%   - p3p_inlier_mask(1, K): mark keypoint used for p3p estimation
%   - tracked_indices(K, 1): indices of keypoints tracked through KLT

%% Construct the state S strcut
% P: 2D tracked keypoints array (N, 2)
% X: 3D tracked landmarks array (N, 3)
state = struct('P', [], 'X', [], 'C', []);

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

%% Step 2: Camera Pose Estimation (PnP)
[R_C_W, t_C_W, p3p_inlier_mask] = cameraPoseEstimation(state.P, state.X, K, process_params);
pose = [R_C_W, t_C_W];

% Output: H_W_C
% pose = [R_C_W', -R_C_W'*t_C_W]; % convert from R_C_W to R_W_C

%% Step 3: Triangulating new points
if ~isempty(prev_state.C)
    % initialize KLT object
    initialize(tracker, fliplr(prev_state.C'), prev_frame);
    % step KLT object to track keypoints from previous frame
    [C, tracked_indices] = step(tracker, curr_frame);
    
    % update candidate coordinates
    state.C = flipud(C(tracked_indices,:)');
    state.F = flipud(C(tracked_indices,:)');
    state.T = prev_state.T(tracked_indices, :);
    
    if ~isempty(state.C)
        % create M x 3 array for storing landmarks
        X_C = zeros(size(state.C, 1), 3);
        for i = 1:size(state.C,1)
            disp(X_C);
        end
    end
end

end