function [best_R, best_t, best_is_inlier] = disambiguateP3P(keypoints, landmarks, poses, K, pixel_tolerance)
%%DISAMBIGUATEP3P Disambiguate between the solutions obtained using P3P algorithm
%%by taking the one that ensures maximum points in front of the camera
%
% INPUT:
%   - keypoints (N, 2): keypoints in [u, v] coordinates
%   - landmarks (N, 3): 3D landmarks in world frame
%   - poses (3, 16): p3p solutions from the cam to the world frame
%                   [ 3x1 position(solution1) 3x3 orientation(solution1)...
%                     3x1 position(solution2) 3x3 orientation(solution2)]
%   - K (3, 3): intrinsic camera matrix
%   - pixel_tolerance: pixel tolerance to consider as inlier
%
% OUTPUT:
%   - best_R(3, 3): Rotation matrix corresponding to best solution
%   - best_t(3, 1): Translation vector corresponding to best solution
%   - best_is_inlier(N, 1): mask over keypoints that correspond to best solution

% extract rotation and translation solutions
Rots = zeros(3, 3, 2);
ts = zeros(3, 2);
for index = 0:1
    % Given noisy data, P3P may return complex values: In this case
    % retain only the real part.
    R_C_W_ii = real(poses(:, (2 + index * 4):(4 + index * 4)));
    t_C_W_ii = real(poses(:, (1 + index * 4)));
    % convert into world to camera frame
    Rots(:, :, index + 1) = R_C_W_ii';
    ts(:, index + 1) = - R_C_W_ii' * t_C_W_ii;
end

% number of input landmarks
num_points = size(landmarks, 1);

% initialize variables
max_inliers = 0;
best_is_inlier = zeros(num_points, 1);
best_R = eye(3,3);
best_t = zeros(3, 1);

for index = 1:size(Rots, 3)
    R = Rots(:, :, index);
    t = ts(:, index);

    % compute projection matrix
    M = K * [R, t];

    % project landmarks to the image plane
    pts = M * [landmarks'; ones(1, num_points)];
    pts = bsxfun (@rdivide, pts, pts(3,:));

    % compute error
    errors = sqrt(sum((keypoints' - pts(1:2, :)).^2, 1));

    % find inlier points
    is_inlier = errors < pixel_tolerance;

    if nnz(is_inlier) > max_inliers
        max_inliers = nnz(is_inlier);
        best_is_inlier = is_inlier;
        best_R = R;
        best_t = t;
    end
end

end
