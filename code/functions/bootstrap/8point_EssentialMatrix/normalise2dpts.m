function [newpts, T] = normalise2dpts(pts)
% NORMALISE2DPTS - normalises 2D homogeneous points
%
% Function translates and normalises a set of 2D homogeneous points 
% so that their centroid is at the origin and their mean distance from 
% the origin is sqrt(2).
%
% Usage:   [newpts, T] = normalise2dpts(pts)
%
% Argument:
%   pts -  3xN array of 2D homogeneous coordinates
%
% Returns:
%   newpts -  3xN array of transformed 2D homogeneous coordinates.
%   T      -  The 3x3 transformation matrix, newpts = T*pts

ptseuc = UnHomogCoords(pts);
mean_pt = mean(ptseuc,2);
pt_euc_cen = ptseuc - mean_pt;

std_p = mean(sqrt(sum(pt_euc_cen.^2))); % mean standard deviation
std_s = sqrt(2)/std_p;
T = [std_s 0 -std_s*mean_pt(1);  0 std_s -std_s*mean_pt(2); 0 0 1];
newpts = T*pts;




