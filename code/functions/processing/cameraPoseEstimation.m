function [R_C_W, t_C_W, best_inlier_mask] = cameraPoseEstimation(keypoints, landmarks, params)
% Input
% keypoints: 2 x k
% landmarks: 3 x k

K = params.camera.intrinsic;
num_iterations = params.process.p3p.num_iterations;
pixel_tolerance = params.process.p3p.pixel_tolerance;
min_inlier_count = params.process.p3p.min_inlier_count;
k = 3; % P3P: 3 points required

% Initialize RANSAC. (best_inlier_mask: N*2 matrix)
best_inlier_mask = zeros(1, size(keypoints, 2));
% (row, col) to (u, v)
keypoints = flipud(keypoints);
max_num_inliers_history = zeros(1, num_iterations);
max_num_inliers = 0;

% RANSAC
for i = 1:num_iterations
    % Model from k samples
    [landmark_sample, idx] = datasample(landmarks, k, 2, 'Replace', false);
    keypoint_sample = keypoints(:, idx);
    

    % Backproject keypoints to unit bearing vectors.
    normalized_bearings = K\[keypoint_sample; ones(1, 3)]; % -> [u;v;1]
    for ii = 1:3
        normalized_bearings(:, ii) = normalized_bearings(:, ii) / ...
            norm(normalized_bearings(:, ii), 2);
    end

    poses = p3p(landmark_sample, normalized_bearings);

    % Decode p3p output (two solutions)
    % we can simply count inliers with both solutions, 
    % and pick the solution with more inliers.
    R_C_W_guess = zeros(3, 3, 2);
    t_C_W_guess = zeros(3, 1, 2);
    for ii = 0:1
        % Given noisy data, P3P may return complex values. 
        % In this case you should retain only the real part.
        R_W_C_ii = real(poses(:, (2+ii*4):(4+ii*4)));
        t_W_C_ii = real(poses(:, (1+ii*4)));
        R_C_W_guess(:,:,ii+1) = R_W_C_ii';
        t_C_W_guess(:,:,ii+1) = -R_W_C_ii'*t_W_C_ii;
    end

    
    % Count inliers:
    % [projected_points] = projectPoints(points_3d, K, D)
    projected_points = projectPoints(...
        (R_C_W_guess(:,:,1) * landmarks) + ...
        repmat(t_C_W_guess(:,:,1), ...
        [1 size(landmarks, 2)]), K);
    difference = keypoints - projected_points;
    errors = sum(difference.^2, 1); % 1D vector
    is_inlier = errors < pixel_tolerance^2;
    
    % consider inliers for the alternative solution.
    projected_points = projectPoints(...
        (R_C_W_guess(:,:,2) * landmarks) + ...
        repmat(t_C_W_guess(:,:,2), ...
        [1 size(landmarks, 2)]), K);
    difference = keypoints - projected_points;
    errors = sum(difference.^2, 1);
    alternative_is_inlier = errors < pixel_tolerance^2;
    if nnz(alternative_is_inlier) > nnz(is_inlier)
        is_inlier = alternative_is_inlier;
    end
    
    if nnz(is_inlier) > max_num_inliers && ...
            nnz(is_inlier) >= min_inlier_count
        max_num_inliers = nnz(is_inlier);        
        best_inlier_mask = is_inlier;
    end
    
    max_num_inliers_history(i) = max_num_inliers;
end

if max_num_inliers == 0
    R_C_W = [];
    t_C_W = [];
else
    M_C_W = estimatePoseDLT(...
        keypoints(:, best_inlier_mask>0)', ...
        landmarks(:, best_inlier_mask>0)', K);
    R_C_W = M_C_W(:, 1:3);
    t_C_W = M_C_W(:, end);
end

end

