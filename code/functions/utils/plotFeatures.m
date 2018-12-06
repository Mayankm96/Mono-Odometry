function plotFeatures(pts,  ptSyle, ptWidth)
%PLOTFEATURES Plot features using MATLAB
%
% INPUT:
%   - pts(1, N): array of structure containing keypoints
%   - ptSyle: defines color and type of scatter plot; eg: '+r'
%   - ptWidth: defines thickness of scatter points
assert (size(pts, 2) > size(pts, 1), 'Input expected to array of shape (1, N)')

% get locations of points
locations = vertcat(pts(:).location);
class_pt = vertcat(pts(:).class);


scatter(locations(:, 2), locations(:, 1), ptSyle, 'LineWidth', ptWidth);

end
