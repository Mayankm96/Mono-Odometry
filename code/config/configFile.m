% ------------------------------------------------------------------------------
% Configuration File for Visual Odometry Algorithm
% -------------------------------------------------------------------------------
%% Parameters for DataSet
% Choose which dataset to evaluate VO on
ds = 0; % 0: KITTI, 1: Malaga, 2: parking
% Path to the directories containing images
% For KITTI dataset
 data_params.kitti_path = 'C:\Users\haoch\Documents\DataSet\kitti';
%data_params.kitti_path = '../data/kitti';
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
% For harris keypoint selector
vo_params.bootstrap.harris.patch_size = 9;
vo_params.bootstrap.harris.kappa = 0.08;
vo_params.bootstrap.harris.num_keypoints = 1000;
vo_params.bootstrap.harris.nonmaximum_supression_radius = 10;

% For KLT tracking 
vo_params.bootstrap.KLT.num_pyramid_levels = 4;
vo_params.bootstrap.KLT.max_bidirectional_error = 1;
vo_params.bootstrap.KLT.block_size = [21 21];
vo_params.bootstrap.KLT.max_iterations = 30;

% For RANSAC
vo_params.bootstrap.RANSAC.num_iterations = 500;
vo_params.bootstrap.RANSAC.pixel_tolerance = 10;

%% [ProcessFrame] 
% For KLT tracker
% See: https://ch.mathworks.com/help/vision/ref/vision.pointtracker-system-object.html
vo_params.process.KLT.num_pyramid_levels = 3;
vo_params.process.KLT.max_bidirectional_error = 6;
vo_params.process.KLT.block_size = [31 31];
vo_params.process.KLT.max_iterations = 30;

% For P3P algorithm
vo_params.process.p3p.num_iterations = 200;
vo_params.process.p3p.pixel_tolerance = 8;
vo_params.process.p3p.min_inlier_count = 6;

% For adding new landmarks
vo_params.process.landmarks.bearing_min = 0.4 * pi/180;
vo_params.process.landmarks.bearing_max = 3.5 * pi/180;
vo_params.process.new_candidate_tolerance = 10;

% For harris keypoint selection and matching
vo_params.process.harris.patch_size = 9;
vo_params.process.harris.kappa = 0.08;
vo_params.process.harris.num_keypoints = 700;
vo_params.process.harris.nonmaximum_supression_radius = 8;
vo_params.process.harris.descriptor_radius = 9;
vo_params.process.harris.match_lambda = 4;
