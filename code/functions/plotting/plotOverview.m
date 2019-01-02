function plotOverview(curr_image, curr_state, prev_state, tracked_state_keypts, trajectory, pointcloud, num_of_latest_states)
%%PLOTOVERVIEW Pretty plotting 
%
% INPUT:
%   - curr_image(H, W): grey scale image matrix at current time t
%   - curr_state(struct): inculdes state.P(2xk) & state.X(3xk)
%   - prev_state(struct): inculdes state.P(2xk) & state.X(3xk)
%   - R_W_C(3, 3): rotation matrix of camera wrt world frame
%   - t_W_C(3, 1): translation of camera wrt world frame
%   - trajectory(3, N): matrix storing entire estimated trajectory poses
%   - tracked_state_keypts(K, 1): indices of tracked keypoints in state

%% Show image with keypoints tracking
sp_1 = subplot(2,2,[1 2]);
cla(sp_1);
imshow(curr_image);
hold on;      
% convert matrix shape from M x 2 to 2 x M
prev_keypoints = prev_state.P(tracked_state_keypts, :)';
curr_keypoints = curr_state.P';
plot(prev_keypoints(1, :), prev_keypoints(2, :), '+r', 'LineWidth', 2)
plot(curr_keypoints(1, :), curr_keypoints(2, :), '+g', 'LineWidth', 2)

%% Plot latest coordinate and landmarks
sp_2 = subplot(2,2,3); 
cla(sp_2);
subplot(2,2,3); 
hold on;  grid on;  
axis vis3d;
for i = 1:min(num_of_latest_states, size(trajectory,2))
    % convert landmarks array from M x 3 to 3 x M and plot them
    landmarks = pointcloud{i}';
    scatter3(landmarks(1, :), landmarks(2, :), landmarks(3, :), 5, 'k');
    pose = reshape(trajectory(:,i), [3,4]);
    R_W_C = pose(:,1:3);
    t_W_C = pose(:,4);
    plotCoordinateFrame(R_W_C, t_W_C, 2);
end

% set axis limit for cleaner plots
t_W_C = trajectory(10:12,1);
axis([t_W_C(1)-10, t_W_C(1)+10, ... 
      t_W_C(2)-10, t_W_C(2)+10, ...
      t_W_C(3)-10, t_W_C(3)+10]);
title(['Landmarks & Coordinates of latest ', num2str(num_of_latest_states),' frames']);  
set(gcf, 'GraphicsSmoothing', 'on');
view(0,0);
    
%% Plot full trajectory
sp_3 = subplot(2,2,4); 
hold on; grid on; axis equal;      
plot(trajectory(10,1),trajectory(12,1),'b.-')
title('Full Trajectory'); axis equal; grid on;
xlabel('x'); ylabel('z');
end