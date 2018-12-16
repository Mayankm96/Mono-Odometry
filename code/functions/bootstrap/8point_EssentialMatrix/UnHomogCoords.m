function res = UnHomogCoords(x)

res=x ./ ( ones(size(x,1),1) * x(end,:));