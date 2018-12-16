function [R,u3] = decomposeEssentialMatrix(E)
% Given an essential matrix, compute the camera motion, i.e.,  R and T such
% that E ~ T_x R
% 
% Input:
%   - E(3,3) : Essential matrix
%
% Output:
%   - R(3,3,2) : the two possible rotations
%   - u3(3,1)   : a vector with the translation information
[U,~,V] = svd(E);
W = [0 -1 0; 1 0 0; 0 0 1];
R = zeros(3,3,2);
R(:,:,1) = U*W'*V';
R(:,:,2) = U*W*V';
% make sure that the determine is 1
if det(R(:,:,1)) == -1
    R(:,:,1) = -R(:,:,1);
end
if det(R(:,:,2)) == -1
    R(:,:,2) = -R(:,:,2);
end

u3 = U(:,end);
if norm(u3) ~= 0
    u3 = u3/norm(u3);
end