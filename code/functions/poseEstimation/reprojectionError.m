function error = reprojectionError(x, X, P, K)
assert(size(X,2) == size(P,2));
P_uv = flipud(P);
T_C_W = twist2HomogMatrix(x);

M_C_W = K*T_C_W(1:3,:);
p_projected_hom = M_C_W * [X; ones(1,size(X,2))];
p_projected = p_projected_hom(1:2,:)./p_projected_hom(3,:);

error = P_uv - p_projected;
end