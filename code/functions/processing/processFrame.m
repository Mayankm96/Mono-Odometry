function [state, pose, p3p_inlier_mask, klt_tracked_indices] = processFrame(curr_frame, prev_frame, prev_state, params)

% Construct the state S strcut
% P: 2D tracked keypoints array [2 x k]
% X: 3D tracked landmarks array [3 x k]
state = struct('P', [], 'X', []);

%% Step 1: Feature Tracking (KLT)
% construct KLT object
klt_tracker = vision.PointTracker('NumPyramidLevels',params.process.KLT.num_pyramid_levels, ...
                                  'MaxBidirectionalError',params.process.KLT.max_bidirectional_error, ...
                                  'BlockSize',params.process.KLT.block_size, ...
                                  'MaxIterations',params.process.KLT.max_iterations);
% initialize KLT object
initialize(klt_tracker, fliplr(prev_state.P'), prev_frame);
% step KLT object to track keypoints from previous frame
[P, tracked_indices] = step(klt_tracker, curr_frame);
% copy tracked keypoints and landmarks into current state S
state.P = flipud(P(tracked_indices,:)');
state.X = prev_state.X(:,tracked_indices);
% release the KLT object
release(klt_tracker);
klt_tracked_indices = tracked_indices;
%% Step 2: Camera Pose Estimation (PnP)
[R_C_W, t_C_W, p3p_inlier_mask] = cameraPoseEstimation(state.P, state.X, params);
pose = [R_C_W, t_C_W];
% Output: H_W_C
% pose = [R_C_W', -R_C_W'*t_C_W]; % convert from R_C_W to R_W_C
end