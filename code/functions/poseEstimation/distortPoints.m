function x_d = distortPoints(x, D)
% x_d = distortPoints(x, D); 
% Applies lens distortion to 2D points x on the image plane.
% Input:
%   x, 2xN undistorted points
%   D, 2x1 distortion vector
% Output:
%   x_d, 2xN distorted points


k1 = D(1); k2 = D(2);

xp = x(1,:); yp = x(2,:);

r2 = xp.^2 + yp.^2;
xpp = xp .* (1 + k1*r2 + k2*r2.^2);
ypp = yp .* (1 + k1*r2 + k2*r2.^2);

x_d = [xpp; ypp];

end

