function keypts = selectKeypoints(scores, max_num, radius)
%%SELECTKEYPOINTS Select a certain number of points on the basis of their 
% harris scores through non-maximum supression on a (2r + 1)*(2r + 1) box
% to obatin only the strongest features
% 
% INPUT: 
%   - scores (H, W) : an array comprising of harris score at each pixel
%   - max_num: number of keypoints required
%   - radius: radius of the searching block in non-maximum supression
%
% OUTPUT: 
%   - keypts (M, 2): an array of detected keypoints and their location

keypts = zeros(max_num, 2);
% pad the input array by r
temp_scores = padarray(scores, [radius radius]);

for i = 1:max_num
    % search for the keypoint with the highest harris score
    [~, index] = max(temp_scores(:));
    % find the location of the maxima
    [row, col] = ind2sub(size(temp_scores), index);
    kp = [row, col];
    % account for the padding in the score matrix and store keypoint 
    keypts(i, :) = kp - radius;
    % set all pixels around the maxima to zero to prevent them from being selected again
    temp_scores(kp(1)-radius : kp(1)+radius, kp(2)-radius : kp(2)+radius) = 0;
end

end

