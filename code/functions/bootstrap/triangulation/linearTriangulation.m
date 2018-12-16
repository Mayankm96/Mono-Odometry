function P = linearTriangulation(p1,p2,M1,M2)
% LINEARTRIANGULATION  Linear Triangulation
%
% Input:
%  - p1(3,N): homogeneous coordinates of points in image 1
%  - p2(3,N): homogeneous coordinates of points in image 2
%  - M1(3,4): projection matrix corresponding to first image
%  - M2(3,4): projection matrix corresponding to second image
%
% Output:
%  - P(4,N): homogeneous coordinates of 3-D point
P = zeros(4,size(p1,2));

for i = 1:size(p1,2)
    pa = cross2Matrix(p1(:,i))*M1;
    pb = cross2Matrix(p2(:,i))*M2;
    A = [pa;pb];
    [~,~,V] = svd(A, 0);
    P(:,i) = V(:,4);
end
P = P./repmat(P(4,:),4,1); % Dehomogeneize (P is expressed in homogeneous coordinates)
end
