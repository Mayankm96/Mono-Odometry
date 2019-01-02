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

P = keypoints';
X = landmarks';

% RANSAC
best_R = [];
best_t = [];
max_num_inliers = 0;
best_inlier_mask = false(1, size(X,2));

for i = 1:num_iterations
    % Model from k samples (3P)
    [X_sample, idx] = datasample(...
        X, num_sample_points, 2, 'Replace', false);
    P_sample = P(:, idx);

    % Backproject keypoints to unit bearing vectors.
    normalized_bearings = K\[P_sample; ones(1, 3)];
    normalized_bearings = normalized_bearings ./ vecnorm(normalized_bearings);
    
    poses = p3p(X_sample, normalized_bearings);

    % Decode p3p output
    R_C_W_guess = zeros(3, 3, 4);
    t_C_W_guess = zeros(3, 1, 4);
    for ii = 0:3        
        R_W_C_ii = real(poses(:, (2+ii*4):(4+ii*4)));
        t_W_C_ii = real(poses(:, (1+ii*4)));
        R_C_W_guess(:,:,ii+1) = R_W_C_ii';
        t_C_W_guess(:,:,ii+1) = -R_W_C_ii'*t_W_C_ii;
    end

    % Count inliers:
    num_solutions = 0;
    for ii=1:4
        landmarks_in_new_system = R_C_W_guess(:,:,ii) * X + repmat(t_C_W_guess(:,:,ii), ...
            [1 size(X, 2)]);
        
        % count points with negative z
        num_points_behind_camera = sum(landmarks_in_new_system(3,:) < 0);
        
        if num_points_behind_camera == 0
            num_solutions = num_solutions + 1;
            projected_points = projectPoints(landmarks_in_new_system , K);
        
            difference = P - projected_points;
            errors = sum(difference.^2, 1);
            is_inlier = errors < pixel_tolerance^2;

            if nnz(is_inlier) > max_num_inliers && ...
                 nnz(is_inlier) >= min_inlier_count
                max_num_inliers = nnz(is_inlier);        
                best_inlier_mask = is_inlier;
                best_R = R_C_W_guess(:,:,ii);
                best_t = t_C_W_guess(:,:,ii);
            end
        end  
    end
    
end

R_C_W = best_R;
t_C_W = best_t;
    
end
