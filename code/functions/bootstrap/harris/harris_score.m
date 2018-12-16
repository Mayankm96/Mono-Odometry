function scores = harris_score(img, patch_size, kappa)
% INPUT¡G(1) img¡Gimage we want to calculate harris score
%                (2) patch_size¡Gpatchsize we want to scan through the image
%                (3) kappa¡Gparam to calculate the harris score
%OUTPUT¡Gscores 
%                  size [patch_size+floor(patch_size/2)+1, patch_size+floor(patch_size/2)+1]


% setting up sobel filter to do the difference method
sobel_x = -fspecial('sobel')';
sobel_y = -fspecial('sobel');

% convolute sobel with img to get Ix and Iy respectively
Ix = conv2(double(img),double(sobel_x),'valid');
Iy = conv2(double(img),double(sobel_y),'valid');

% getting the M matrix elements
Ix2 = Ix.^2;
Iy2 = Iy.^2;
Ixy = Ix.*Iy;

% using box patch and convolution to get sum(Ix^2) sum(Iy^2) and sum(Ix*Iy)
box = ones(patch_size);
sum_Ix2 = conv2(double(Ix2),box,'valid');
sum_Iy2 = conv2(double(Iy2),box,'valid');
sum_Ixy = conv2(double(Ixy),box,'valid');

% calculate the harris score R = det(M) - kappa*trace(M)^2
R = (sum_Ix2 .* sum_Iy2 - sum_Ixy .^ 2) - kappa * (sum_Ix2 + sum_Iy2) .^ 2;  
R(R<0) = 0; 
scores = padarray(R,[floor(patch_size/2)+1,floor(patch_size/2)+1],0,'both');
end

