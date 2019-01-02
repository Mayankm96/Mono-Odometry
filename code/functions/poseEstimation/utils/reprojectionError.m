function error = reprojectionError(x, landmarks, keypoints, K)
%%REPROJECTIONERROR Function to compute reprojection error for input
%%keypoint locations and corresponding 3D triangulated points
%
% INPUT:
%   - x (6, 1): Twist pose of world to camera frame
%   - landmarks (2, N): landmarks in the world frame 
%   - P (2, N): Keypoint coordinates in camera frame
%   - K (3, 3): Intrinsic camera matrix
%
% OUTPUT:
%   - error: reprojection error

% Sanity check for right input size
assert(size(landmarks,2) == size(keypoints,2));

% Convert keypoints coordinates to 
P_uv = flipud(keypoints);

% Obtain world to camera transformation matrix
T_C_W = twist2HomogMatrix(x);

% perform projection of 3D landmarks to camera frame
M_C_W = K*T_C_W(1:3,:);
p_projected_hom = M_C_W * [landmarks; ones(1,size(landmarks,2))];
p_projected = p_projected_hom(1:2,:)./p_projected_hom(3,:);

% compute reprojection error
error = P_uv - p_projected;

end