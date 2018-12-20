% ------------------------------------------------------------------------------
% Configuration File for Visual Odometry Algorithm
% -------------------------------------------------------------------------------
%% Parameters for DataSet
% Choose which dataset to evaluate VO on
ds = 0; % 0: KITTI, 1: Malaga, 2: parking
% Path to the directories containing images
% For KITTI dataset
% data_params.kitti_path = 'C:\Users\haoch\Documents\DataSet\kitti';
data_params.kitti_path = '../data/kitti';
% For Malaga dataset
data_params.malaga_path = '../data/malaga-urban-dataset-extract-07';
% For Parking dataset
data_params.parking_path = '../data/parking';
% Set flag to 1 to plot groundtruth as well
data_params.show_gt_flag = 1;
% Use parallel threads (requires Parallel Processing Toolbox)
% !! TO-DO: fix parfor and for loops for this functionality!
data_params.use_multithreads = 0;                % 0: disabled, 1: enabled

%% [Bootstrap]
% harris keypoint selector
vo_params.bootstrap.harris.patch_size = 9;
vo_params.bootstrap.harris.kappa = 0.08;
vo_params.bootstrap.harris.num_keypoints = 500;
vo_params.bootstrap.harris.nonmaximum_supression_radius = 10;

% KLT tracking 
vo_params.bootstrap.KLT.num_pyramid_levels = 3;
vo_params.bootstrap.KLT.max_bidirectional_error = 1;
vo_params.bootstrap.KLT.block_size = [15 15];
vo_params.bootstrap.KLT.max_iterations = 60;

% using RANSAC parameters
vo_params.bootstrap.RANSAC.num_iterations = 500;
vo_params.bootstrap.RANSAC.num_sampling_points = 12;
vo_params.bootstrap.RANSAC.min_inlier_points = 6;
vo_params.bootstrap.RANSAC.pixel_tolerance = 5;

%% [ProcessFrame] 
% For KLT tracker
% See: https://ch.mathworks.com/help/vision/ref/vision.pointtracker-system-object.html
vo_params.process.KLT.num_pyramid_levels = 3;
vo_params.process.KLT.max_bidirectional_error = inf;
vo_params.process.KLT.block_size = [31 31];
vo_params.process.KLT.max_iterations = 30;

% For P3P algorithm
vo_params.process.p3p.num_iterations = 1000;
vo_params.process.p3p.pixel_tolerance = 10;
vo_params.process.p3p.min_inlier_count = 40;
