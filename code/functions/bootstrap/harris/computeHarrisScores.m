function scores = computeHarrisScores(img, patch_size, kappa)
%%COMPUTEHARRISSCORES Compute harris scores of each pixel in the image
%
% INPUT:
%   - img (H, W): Input image
%   - patch_size:  size of a square patch in one dimension
%   - kappa: hyperparameter to calculate the harris score
% OUTPUT:
%   - scores (H, W): harris scores for each pixel in the image

% defining the sobel filters
sobel_para = [-1 0 1];
sobel_orth = [1 2 1];

% perform convolution of the images with sobel filters
Ix = conv2(sobel_orth', sobel_para, img, 'valid');
Iy = conv2(sobel_para', sobel_orth, img, 'valid');
Ixx = double(Ix .^ 2);
Iyy = double(Iy .^ 2);
Ixy = double(Ix .* Iy);

% compute sum(Ix^2) sum(Iy^2) and sum(Ix*Iy) through covolution
patch = ones(patch_size, patch_size);
pr = floor(patch_size / 2);  % patch radius
sIxx = conv2(Ixx, patch, 'valid');
sIyy = conv2(Iyy, patch, 'valid');
sIxy = conv2(Ixy, patch, 'valid');

% compute Harris score 
% R = det(M) - kappa * trace(M)^2
scores = (sIxx .* sIyy - sIxy .^ 2) ... determinant
    - kappa * (sIxx + sIyy) .^ 2;  % square trace

scores(scores < 0) = 0;

scores = padarray(scores, [1+pr 1+pr]);

end
