% ------------------------------------------------------------------------------
% Configuration File for Visual Odometry Algorithm
% -------------------------------------------------------------------------------

%% General
% Use parallel threads (requires Parallel Processing Toolbox)
% !! TO-DO: fix parfor and for loops for this functionality!
use_multithreads = 0;                % 0: disabled, 1: enabled

% Print log messages
verbose = 1;                         % 0: disabled, 1: enabled

% Set flag to 1 to plot groundtruth as well
show_gt_flag = 1;                    % 0: disabled, 1: enabled

% Choose which dataset to evaluate VO on
ds = 0;                              % 0: KITTI, 1: Malaga, 2: parking

%% Paths for Dataset
% Path to the directories containing images
% For KITTI dataset
% data_params.kitti_path = 'C:\Users\haoch\Documents\DataSet\kitti';
data_params.kitti_path = '../data/kitti';
% For Malaga dataset
data_params.malaga_path = '../data/malaga-urban-dataset-extract-07';
% For Parking dataset
data_params.parking_path = '../data/parking';

%% Load parameters specific to chosen dataset
if ds == 0
    % KITTI Dataset
    paramsKITTI;
elseif ds == 1
    % Malaga Urban Dataset
    paramsMalaga;
elseif ds == 2
    % Parking Dataset
    paramsParking;
else
    assert(false);
end
