function descriptors = describeKeypoints(img, keypoints, r)
%%DESCRIBEKEYPOINTS Returns a image patch descriptors for input keypoints
%
% INPUT: 
%   - img(H, W): input image
%   - keypoint(2, N): keypoint coordinates in [u, v] coordinates
%   - r: patch radius
%
% OUTPUT:
%   - descriptors(2r+1)^2, N): patch descriptors

% convert into [rol, col] coordinates
keypoints = flipud(keypoints);

N = size(keypoints, 2);
descriptors = uint8(zeros((2*r+1) ^ 2, N));
padded = padarray(img, [r, r]);

for i = 1:N
    kp = int64(keypoints(:, i)) + r;
    descriptors(:,i) = reshape(padded(kp(1)-r:kp(1)+r, kp(2)-r:kp(2)+r), [], 1);
end

end
