%% ------------------------------------------------------------------------------
% Configuration File for Visual Odometry Algorithm
%% -------------------------------------------------------------------------------

% Choose which dataset to evaluate VO on
ds = 0; % 0: KITTI, 1: Malaga, 2: parking

% Path to the directories containing images
% For KITTI dataset
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

%% Parameters for Feature Extraction
vo_params.feature.nms_n = 8;                      % non-max-suppression: min. distance between maxima (in pixels)
vo_params.feature.nms_tau = 50;                   % non-max-suppression: interest point peakiness threshold
vo_params.feature.margin = 21;                    % leaving margin for safety while computing features ( >= 25)

%% Parameters for Feature Matching
vo_params.matcher.match_binsize = 50;             % matching bin width/height (affects efficiency only)
vo_params.matcher.match_radius = 200;             % matching radius (du/dv in pixels)

%% Paramters for Feature Selection using bucketing
vo_params.bucketing.max_features = 1;             % maximal number of features per bucket
vo_params.bucketing.bucket_width = 50;            % width of bucket
vo_params.bucketing.bucket_height = 50;           % height of bucket

%% Paramters for motion estimation
% estimate translation using weighted optimization equation
vo_params.estim.ransac_iters = 200;              % number of RANSAC iterations
vo_params.estim.inlier_threshold = 2.0;          % fundamental matrix inlier threshold
vo_params.estim.reweighing = 1;                  % lower border weights (more robust to calibration errors)
