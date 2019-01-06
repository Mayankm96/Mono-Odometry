% ------------------------------------------------------------------------------
% Configuration File for Visual Odometry Algorithm
% KITTI Dataset
% -------------------------------------------------------------------------------

%% Camera Intrinsic Parameters Matrix
K = load([data_params.parking_path '/K.txt']);

%% [Bootstrap]

% Bootsrap frames
bootstrap_frames = [1, 14];

% For harris keypoint selector
vo_params.bootstrap.harris.patch_size = 9;
vo_params.bootstrap.harris.kappa = 0.08;
vo_params.bootstrap.harris.num_keypoints = 1000;
vo_params.bootstrap.harris.nonmaximum_supression_radius = 10;

% For KLT tracking
vo_params.bootstrap.KLT.num_pyramid_levels = 3;
vo_params.bootstrap.KLT.max_bidirectional_error = 6;
vo_params.bootstrap.KLT.block_size = [31 31];
vo_params.bootstrap.KLT.max_iterations = 30;

% For RANSAC
vo_params.bootstrap.RANSAC.num_iterations = 50;
vo_params.bootstrap.RANSAC.pixel_tolerance = 3;

%% [ProcessFrame]
% For KLT tracker
% See: https://ch.mathworks.com/help/vision/ref/vision.pointtracker-system-object.html
vo_params.process.KLT.num_pyramid_levels = 3;
vo_params.process.KLT.max_bidirectional_error = 6;
vo_params.process.KLT.block_size = [31 31];
vo_params.process.KLT.max_iterations = 30;

% For P3P algorithm
vo_params.process.p3p.num_iterations = 200;
vo_params.process.p3p.pixel_tolerance = 20;
vo_params.process.p3p.min_inlier_count = 6;

% For adding new landmarks
vo_params.process.landmarks.bearing_min = 0.15 * pi/180;
vo_params.process.landmarks.bearing_max = 6.5 * pi/180;
vo_params.process.new_candidate_tolerance = 10;

% For harris keypoint selection and matching
vo_params.process.harris.patch_size = 9;
vo_params.process.harris.kappa = 0.08;
vo_params.process.harris.num_keypoints = 1000;
vo_params.process.harris.nonmaximum_supression_radius = 10;
vo_params.process.harris.descriptor_radius = 9;
vo_params.process.harris.match_lambda = 10;

%% [Additional Parameters]
additional_fix.flag = false;
additional_fix.bootstrap_interval = 35;

%% [Pretty Plotting]
plot_params.max_landmarks = 800;
plot_params.margin_full = 5;
plot_params.margin_local = 5;