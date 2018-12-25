function plotOverview(curr_image, curr_state, prev_state, R_W_C, t_W_C, trajectory, tracked_state_keypts)
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
figure(1);

%% Show image with keypoints tracking
subplot(2,2,[1 2]);
imshow(curr_image);
hold on;      
% convert matrix shape from M x 2 to 2 x M
prev_keypoints = prev_state.P(tracked_state_keypts, :)';
curr_keypoints = curr_state.P';
plot(prev_keypoints(1, :), prev_keypoints(2, :), '+r', 'LineWidth', 2)
plot(curr_keypoints(1, :), curr_keypoints(2, :), '+g', 'LineWidth', 2)

%% Plot trajectory and landmarks
subplot(2,2,3); 
hold on;  grid on;  
axis vis3d;
% set axis limit for cleaner plots
axis([min(trajectory(1,:))-10, max(trajectory(1,:))+10, ... 
      min(trajectory(2,:))-10, max(trajectory(2,:))+10, ...
      min(trajectory(3,:))-10, max(trajectory(3,:))+10]);
  
% convert landmarks array from M x 3 to 3 x M and plot them
landmarks = curr_state.X';
% scatter3(landmarks(1, :), landmarks(2, :), landmarks(3, :), 5, 'k');
set(gcf, 'GraphicsSmoothing', 'on');

% plot trajectory
if (numel(R_W_C) > 0)
    plotCoordinateFrame(R_W_C, t_W_C, 2);
    view(0,0);
end
    
end