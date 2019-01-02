function [R_C_W, t_C_W] = nonlinearOptimization(keypoints, landmarks, R_C_W_guess, t_C_W_guess, K)

P = keypoints';
P = flipud(P);
X = landmarks';

x_init = HomogMatrix2twist([R_C_W_guess t_C_W_guess; [0 0 0 1]]);

error_terms = @(x) reprojectionError(x, X, P, K);

options = optimoptions(@lsqnonlin, 'Display','off','MaxIter', 20);

x_optim = lsqnonlin(error_terms, x_init, [], [], options);

T_C_W_optim = twist2HomogMatrix(x_optim);

R_C_W = T_C_W_optim(1:3,1:3);
t_C_W = T_C_W_optim(1:3,4);


end