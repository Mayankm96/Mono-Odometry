function keypoints = selectKeypoints(scores, num, r)
% Selects the num best scores as keypoints and performs non-maximum 
% supression of a (2r + 1)*(2r + 1) box around the current maximum.
% INPUT (1)scores ¡Gharris score array 
%             (2) num¡Gnumberof keypoints we want to select
%             (3) r ¡Gnonmaximum_supression_radius
% OUTPUT keypoint array [2, num]

% using this method we won't have to worry about 
% exceeding boundary problem
expand_scores = padarray(scores, [r r]);

keypoints = zeros(2, num);

for i = 1:num
    % search for the highest harris score
    [~, ind] = max(expand_scores(:));
    % find the index of it
    [index_y,index_x] = ind2sub(size(expand_scores), ind);
    kp = [index_y, index_x];
    % we have to minus r because we expand the score matrix by r 
    keypoints(:,i) = kp-r;
    % we have to set the surrounding pixels to zero
    % otherwise we may select the surrounding pixels with large values
    expand_scores(kp(1)-r : kp(1)+r, kp(2)-r : kp(2)+r) = 0;
end
end

