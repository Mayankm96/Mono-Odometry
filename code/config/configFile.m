% ------------------------------------------------------------------------------
% Configuration File for Visual Odometry Algorithm
% -------------------------------------------------------------------------------

%% General

% Choose which dataset to evaluate VO on
ds = 0;                              % 0: KITTI, 1: Malaga, 2: parking, 3: indoor

% Print log messages
verbose = 1;                         % 0: disabled, 1: enabled

% Set flag to 1 to plot groundtruth as well
show_gt_flag = 0;                    % 0: disabled, 1: enabled

%% Paths for Dataset
% Path to the directories containing images

% For KITTI dataset
% data_params.kitti_path = 'C:\Users\haoch\Documents\DataSet\kitti';
data_params.kitti_path = '../data/kitti';

% For Malaga dataset
data_params.malaga_path = '../data/malaga-urban-dataset-extract-07';

% For Parking dataset
data_params.parking_path = '../data/parking';

% For Indoor dataset
data_params.indoor_path = '../data/va4mr_new';

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
elseif ds == 3
    % Indoor Dataset
    paramsIndoor;
else
    assert(false);
end
