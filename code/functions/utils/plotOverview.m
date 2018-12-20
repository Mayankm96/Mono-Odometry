function plotOverview(curr_image, curr_state, prev_state, tracked_indices, R_C_W, t_C_W, trajectory)
%%PLOTOVERVIEW Pretty plotting 
%
% INPUT:
%   - curr_image(H, W): grey scale image matrix at current time t
%   - curr_state(struct): inculdes state.P(2xk) & state.X(3xk)
%   - prev_state(struct): inculdes state.P(2xk) & state.X(3xk)
%   - tracked_indices(K, 1): indices of the tracked keypoints using KLT
%   - R_C_W(3, 3): matrix to represent rotation from camera to world
%   - t_C_W(3, 1): vector to represent translation from camera to world
%   - trajectory(3, N): matrix storing entire estimated trajectory poses

%% Show image with keypoints tracking
subplot(2,2,[1 2]);
imshow(curr_image);
hold on;      
% convert coordinates from [u, v] to [row, col]
prev_keypoints = fliplr(prev_state.P(tracked_indices, :));
curr_keypoints = fliplr(curr_state.P);
plotMatches(1:size(curr_keypoints, 1), curr_keypoints', prev_keypoints');

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
scatter3(landmarks(1, :), landmarks(2, :), landmarks(3, :), 5, 'k');
set(gcf, 'GraphicsSmoothing', 'on');

% plot pose
if (numel(R_C_W) > 0)
    plotCoordinateFrame(R_C_W', -R_C_W'*t_C_W, 2); % convert from R_C_W to R_W_C
    view(0,0);
end
    
end