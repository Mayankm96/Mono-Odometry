function plotOverview(curr_image, curr_state, prev_state, tracked_indices, R_C_W, t_C_W, trajectory)
% Show image with keypoints tracking
figure(1); 
subplot(2,2,[1 2]); 
hold on;      
imshow(curr_image);
keypoints = flipud(prev_state.P);
prev_keypoints = keypoints(:,tracked_indices);
curr_keypoints = flipud(curr_state.P);
plotMatches(1:size(curr_keypoints, 2), flipud(curr_keypoints), flipud(prev_keypoints));

% Plot trajectory and landmarks
subplot(2,2,3); 
hold on;  grid on;  axis vis3d;
axis([min(trajectory(1,:))-10, max(trajectory(1,:))+10, ... 
      min(trajectory(2,:))-10, max(trajectory(2,:))+10, ...
      min(trajectory(3,:))-10, max(trajectory(3,:))+10]);
scatter3(curr_state.X(1, :), curr_state.X(2, :), curr_state.X(3, :), 5, 'k');
set(gcf, 'GraphicsSmoothing', 'on');
if (numel(R_C_W) > 0)
    plotCoordinateFrame(R_C_W', -R_C_W'*t_C_W, 2); % convert from R_C_W to R_W_C
    view(0,0);
end
    
end