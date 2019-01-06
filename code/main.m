clc;
clear;
close all;

%% Execute the configuration file to read parameters for data paths
addpath(genpath(cd)); % load all functions
configFile; % load parameters

%% For full screen plotting
fig = figure('units','normalized','outerposition',[0 0 1 1]);

%% Setup
if ds == 0
    % need to set kitti_path to folder containing "00" and "poses"
    kitti_path = data_params.kitti_path;
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/00.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
elseif ds == 1
    % Path containing the many files of Malaga 7.
    malaga_path = data_params.malaga_path;
    assert(exist('malaga_path', 'var') ~= 0);
    images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    last_frame = length(left_images);
elseif ds == 2
    % Path containing images, depths and all...
    parking_path = data_params.parking_path;
    assert(exist('parking_path', 'var') ~= 0);
    last_frame = 598;
    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
elseif ds == 3
    % Path containing images, depths and all...
    indoor_path = data_params.indoor_path;
    assert(exist('indoor_path', 'var') ~= 0);
    images = dir(indoor_path);
    left_images = images(3:2:end);
    last_frame = length(left_images);
else
    assert(false);
end

%% Bootstrap
% need to set bootstrap_frames
fprintf('\nWaiting for Bootstrapping ...');
if ds == 0
    img0 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img1 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
elseif ds == 1
    img0 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    img1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));
    % Since ground truth we cannot plot it. Making this behavior explicit
    show_gt_flag = 0;
elseif ds == 2
    img0 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
elseif ds == 3
    img0 = imread([indoor_path '/' left_images(bootstrap_frames(1)).name]);
    img1 = imread([indoor_path '/' left_images(bootstrap_frames(2)).name]);
    % Since ground truth we cannot plot it. Making this behavior explicit
    show_gt_flag = 0;
else
    assert(false);
end

[landmarks, I2_keypts] = bootstrap(img0, img1, vo_params.bootstrap, K);
% Create initial state using the output of bootstrapping
prev_state.P = I2_keypts;   % NOTE: M x 2 with [u, v] notation
prev_state.X = landmarks;   % NOTE: M x 3
prev_state.C = [];
prev_state.F = [];
prev_state.T = [];
prev_image = img1;

fprintf('\n\nBootstrap finished !');

%% Continuous operation

% 12xM matrix to record entire valid trajectory
trajectory = [];

% Cell to record landmarks from latest 20 states
num_of_latest_states = 20;
pointcloud{1, num_of_latest_states} = [];
pointcloud_cell_index = 1;

% frame to start continuous operation from
start_frame = bootstrap_frames(2) + 1;

for i = start_frame:last_frame
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    % read frames from the appropriate chosen dataset
    if ds == 0
        curr_image = imread([kitti_path '/00/image_0/' sprintf('%06d.png',i)]);
        prev_image = imread([kitti_path '/00/image_0/' sprintf('%06d.png',i-1)]);
    elseif ds == 1
        curr_image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2
        curr_image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
    elseif ds == 3
        curr_image = im2uint8(imread([indoor_path '/' ...
            left_images(i).name]));    
    else
        assert(false);
    end

    % process the input frame
    [state, pose] = processFrame(curr_image, prev_image, prev_state, K, vo_params.process, verbose);

    % Check camera pose
    if (~isempty(pose))
        % append to the trajectory cell
        trajectory = [reshape(pose, [12, 1]), trajectory];
        % Save landmarks from latest 20 states
        % TODO! Fix: Inefficient hack to make plotting easier
        if pointcloud_cell_index > num_of_latest_states
            for j = 1:(num_of_latest_states - 1)
                pointcloud{j} = pointcloud{j+1};
            end
            pointcloud_cell_index = num_of_latest_states;
        end
        pointcloud{pointcloud_cell_index} = state.X;
        pointcloud_cell_index = pointcloud_cell_index + 1;
    else
        warning(['Frame ' num2str(i) ' failed tracking!']);
    end

    % plot the result
    if show_gt_flag
        plotOverview(curr_image, state, trajectory, pointcloud, ...
            num_of_latest_states, plot_params, ground_truth(i, :));
    else
        plotOverview(curr_image, state, trajectory, pointcloud, ...
            num_of_latest_states, plot_params);
    end
    
    % update the state and image for next iteration
    prev_state = state;
    
    % Loop safety (unnecessary)
    pause(0.001);
    
    % Save image for videomaking
    % saveas(fig, sprintf('logging/parking/%06d.png', i));
end
