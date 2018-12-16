function [R,T] = disambiguateRelativePose(Rots,u3,points0_h,points1_h,K0,K1)
% DISAMBIGUATERELATIVEPOSE- finds the correct relative camera pose (among
% four possible configurations) by returning the one that yields points
% lying in front of the image plane (with positive depth).
%
% Arguments:
%   Rots -  3x3x2: the two possible rotations returned by decomposeEssentialMatrix
%   u3   -  a 3x1 vector with the translation information returned by decomposeEssentialMatrix
%   p1   -  3xN homogeneous coordinates of point correspondences in image 1
%   p2   -  3xN homogeneous coordinates of point correspondences in image 2
%   K1   -  3x3 calibration matrix for camera 1
%   K2   -  3x3 calibration matrix for camera 2
%
% Returns:
%   R -  3x3 the correct rotation matrix
%   T -  3x1 the correct translation vector
%
%   where [R|t] = T_C1_C0 = T_C1_W is a transformation that maps points
%   from the world coordinate system (identical to the coordinate system of camera 0)
%   to camera 1.

M1 = K0*[eye(3); 0 0 0]';
R = zeros(3,4,4);
R(:,:,1) = cat(2,Rots(:,:,2),u3);
R(:,:,2) = cat(2,Rots(:,:,1),u3);
R(:,:,3) = cat(2,Rots(:,:,2),-u3);
R(:,:,4) = cat(2,Rots(:,:,1),-u3);
P = zeros(4,size(points0_h,2),4);
num_small_zero = [0 0 0 0];
for i = 1:4
    M2 = K1*R(:,:,i);
    P(:,:,i)= linearTriangulation(points0_h,points1_h,M1,M2);
    num_small_zero(i) = nnz(P(3,:,i)>u3(3));
end
[~, argmax] = max(num_small_zero);
T = R(1:3,4,argmax);
R = R(1:3,1:3,argmax);


