function [landmarks, frame2_kps] = bootstrap(prev_img, later_image, params)
%% 
% Input (1) prev_img: first frame
%       (2) later_image: second frame
%       (3) parameters: vo_params.bootstrap struct
% Output 
%       (1) landmarks: Glandmarks(point cloud), shape [3, N]
%       (2) frame2_kps: Gsecond frame keypoints repect to landmarks, shape [2, N]  
%% Harris 
% compute the harris score and find the keypoints for prev_img
K = params.camera.intrinsic;
prev_img_harris_score = harris_score(prev_img, params.bootstrap.harris.patch_size, params.bootstrap.harris.kappa);
prev_img_keypoints = selectKeypoints(prev_img_harris_score, params.bootstrap.harris.num_keypoints, params.bootstrap.harris.nonmaximum_supression_radius);

% KLT matching
tracker = vision.PointTracker('NumPyramidLevels',params.bootstrap.KLT.num_pyramid_levels, ...
                              'MaxBidirectionalError',params.bootstrap.KLT.max_bidirectional_error, ...
                              'BlockSize',params.bootstrap.KLT.block_size, ...
                              'MaxIterations',params.bootstrap.KLT.max_iterations);
initialize(tracker,fliplr(prev_img_keypoints'),prev_img);
[later_img_keypoints, point_validity] = step(tracker,later_image);
later_img_keypoints = fliplr(later_img_keypoints)';
later_matched_kps = later_img_keypoints(:,point_validity);
pre_matched_kps = prev_img_keypoints(:,point_validity);

%% plot for the keypoints for both image
if params.bootstrap.plotting
    figure(2);
    imshow(prev_img);
    hold on;
    plotMatches(1:size(pre_matched_kps,2), later_matched_kps(1:2,:), pre_matched_kps(1:2,:));
end

%% RANSAC part
% prepare for RANSAC inputs
% (row, col) --> (u,v)
pre_matched_kps = flipud(pre_matched_kps);
later_matched_kps = flipud(later_matched_kps);
% [u v] --> [u v 1]
pre_matched_kps = [pre_matched_kps; ones(1, length(pre_matched_kps))];
later_matched_kps = [later_matched_kps; ones(1, length(later_matched_kps))];

% prepare for RANSAC outputs
best_inlier_mask = zeros(1,size(later_matched_kps, 2));
max_num_inliers_history = zeros(1, params.bootstrap.RANSAC.num_iterations);
max_num_inliers = 0;
min_inlier_count = 6;
pixel_tolerance = 1;

% using RANSAC to find best R & T
for iter = 1: params.bootstrap.RANSAC.num_iterations
    % sample data RANSAC.points_needed = 8
    [pre_kp_sampled, idx] = datasample(pre_matched_kps, params.bootstrap.RANSAC.points_needed, 2,'Replace',false);
    match_kp_sampled = later_matched_kps(:,idx);
    % compute the guess of Essential Matrix and decompose it
    E_guess = estimateEssentialMatrix(pre_kp_sampled, match_kp_sampled, K, K);
    [Rots,u3] = decomposeEssentialMatrix(E_guess);
    [R_C2_W,T_C2_W] = disambiguateRelativePose(Rots,u3,pre_kp_sampled,match_kp_sampled,K,K);
    % compute the guess 3D points
    M_prev = K*eye(3,4);
    M_later = K*[R_C2_W,T_C2_W];
    P_W_guess = linearTriangulation(pre_matched_kps,later_matched_kps,M_prev,M_later);
    % do the reprojection
    repro_P_W = K*[R_C2_W,T_C2_W]*P_W_guess(1:4,:);
    norm_repro_P_W_u = repro_P_W(1,:)./repro_P_W(3,:);
    norm_repro_P_W_v = repro_P_W(2,:)./repro_P_W(3,:);
    norm_repro_P_W = [norm_repro_P_W_u ; norm_repro_P_W_v];
    % calculate the difference of the reprojection and ground truth
    difference = abs(later_matched_kps(1:2,:)-norm_repro_P_W(1:2,:));
    errors = sum(difference.^2, 1);
    % calculate how mant inliers are there
    is_inlier = errors < pixel_tolerance^2;
    if nnz(is_inlier) > max_num_inliers && nnz(is_inlier) >= min_inlier_count
        max_num_inliers = nnz(is_inlier);        
        best_inlier_mask = is_inlier;
    end
    % store the history of inliers
    max_num_inliers_history(iter) = max_num_inliers;
end

% select the keypoints with most inliers 
selected_pre_matched_kps = pre_matched_kps(:,best_inlier_mask>0);
selected_later_matched_kps = later_matched_kps(:,best_inlier_mask>0);

%% Find the 3D points

% find the essential matrix
E = estimateEssentialMatrix(selected_pre_matched_kps, selected_later_matched_kps, K, K);
% Get the true R & T
[Rots,u3] = decomposeEssentialMatrix(E);
[R_C2_W,T_C2_W] = disambiguateRelativePose(Rots,u3,selected_pre_matched_kps,selected_later_matched_kps,K,K);
% prepare for the triangulation
M1 = K * eye(3,4);
M2 = K * [R_C2_W, T_C2_W];

% Get the landmark
P_C1 = linearTriangulation(selected_pre_matched_kps,selected_later_matched_kps,M1,M2);
% delete landmarks that are negative
saved_P =  find(P_C1(3,:)>0);
P_C1 = P_C1(:,saved_P);

%% output of the initialization
landmarks = P_C1(1:3,:);
frame2_kps = selected_later_matched_kps(:,saved_P);
frame2_kps = frame2_kps(1:2,:);

%% plot the points 
if params.bootstrap.plotting
    figure(3)
    plot3(P_C1(1,:), P_C1(2,:), P_C1(3,:), 'o');

    plotCoordinateFrame(eye(3),zeros(3,1), 0.8);
    text(-0.1,-0.1,-0.1,'Cam 1','fontsize',10,'color','k','FontWeight','bold');

    %center_cam2_W = -R_C2_W'*T_C2_W;
    center_cam2_W = -R_C2_W'*T_C2_W;
    plotCoordinateFrame(R_C2_W',center_cam2_W, 0.8);
    text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,'Cam 2','fontsize',10,'color','k','FontWeight','bold');
    axis equal
    axis square
    rotate3d on;
    view(90,0)
    grid
end 

end

