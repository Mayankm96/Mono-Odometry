function F = fundamentalEightPoint(p1,p2)
% fundamentalEightPoint  The 8-point algorithm for the estimation of the fundamental matrix F
%
% The eight-point algorithm for the fundamental matrix with a posteriori
% enforcement of the singularity constraint (det(F)=0).
% Does not include data normalization.
%
% Reference: "Multiple View Geometry" (Hartley & Zisserman 2000), Sect. 10.1 page 262.
%
% Input: point correspondences
%  - p1(3,N): homogeneous coordinates of 2-D points in image 1
%  - p2(3,N): homogeneous coordinates of 2-D points in image 2
%
% Output:
%  - F(3,3) : fundamental matrix
Q = [;];
for i = 1: size(p1,2)
    Q = [Q; transpose(kron(p1(:,i), p2(:,i)))];
end
[~,~,V] = svd(Q,'econ');
vecF = V(:,end);
F = reshape(vecF,3,3);
[u,s,v] = svd(F);
s(3,3) = 0;
F = u*s*v';







