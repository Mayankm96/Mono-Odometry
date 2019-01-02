function [R_C_W, t_C_W] = nonlinearOptimization(keypoints, landmarks, R_C_W_guess, t_C_W_guess, K)
%%NONLINEAROPTIMIZATION Improve the estimated pose by minimizing the
%%reprojection error using Levenberg-Marquardt method
%
% INPUT:
%   - keypoints(N, 2): keypoints in image frame
%   - landmarks(N, 3): Corresponding 3D landmarks in world frame 
%   - R_C_W_guess(3,3): Initial guess for rotation (obtained using P3P)
%   - t_C_W_guess(3, 1): Initial guess for translation (obtained using P3P)
%   - K (3, 3): Intrinsic paramters camera matrix
%
% OUTPUT:
%   - R_C_W: Refined rotation matrix from world to camera frame
%   - t_C_W: Refined transalation vector from world to camera frame

% convert to 2 x M matrix with keypoints in [row, col] representation
P = keypoints';
P = flipud(P);
X = landmarks';

% initial guess for optimization solution
x_init = homogMatrix2twist([R_C_W_guess t_C_W_guess; [0 0 0 1]]);

% define objectove function
error_terms = @(x) reprojectionError(x, X, P, K);

% perform optimization
options = optimoptions(@lsqnonlin, 'Display','off','MaxIter', 20);
x_optim = lsqnonlin(error_terms, x_init, [], [], options);

% convert pose from twist to homogenous matrices
T_C_W_optim = twist2HomogMatrix(x_optim);
R_C_W = T_C_W_optim(1:3,1:3);
t_C_W = T_C_W_optim(1:3,4);

end
