function [P_supp, mask] = nonMaxSuppression(P_new, P_fix, r, image_size)
% Input:
% P_fix - 2xN points which shall not be changed. P(1,:) is y. P(2,:) is x.
% P_new - points that will be added if not within non max suppression
% radius if points in P_fix
% r - radius of non max suppression
% image_size - [height width] of image
%
% Applies non maximum suppression to given points and returns the resulting
% subset of P_new and the mask that indicates which points are in the subset
%
P_fix_int = round(flip(P_fix)) + r; % padding
P_new_int = round(flip(P_new)) + r;

A = zeros(image_size + 2*r);
index = sub2ind(image_size + 2*r, P_fix_int(1,:),P_fix_int(2,:));
A(index) = 1;
A(A<1) = 0;
mask = zeros(1,size(P_new,2));
for i=1:size(P_new_int,2)
    p = P_new_int(:,i);
    num_non_zero = nnz(A(p(1)-r:p(1)+r, p(2)-r:p(2)+r));
    if num_non_zero == 0
        A(p(1),p(2)) = 1;
        mask(i) = 1;
    end
end
mask = logical(mask);
P_supp = P_new(:,mask);