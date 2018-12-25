function [query_indices, database_indices] = matchDescriptors(query_descriptors, ...
                                        database_descriptors, lambda, unique)
%%MATCHDESCRIPTORS Matches query features with database features. Match
%%exists if SSD < lambda * min(SSD)
% 
% INPUT:
%   - query_descriptors(M, Q): query descriptors
%   - database_descriptors(M, D): database descriptors
%   - lamda: matching coefficient
%   - unique: to match uniquely or not
%
% OUTPUT:
%   - query_indices(1, N): indices for unique matches in query 
%   - database_indices(1, N): indices for unique matches in database 

[dists,matches] = pdist2(double(database_descriptors)', ...
    double(query_descriptors)', 'euclidean', 'Smallest', 1);

sorted_dists = sort(dists);
sorted_dists = sorted_dists(sorted_dists~=0);
min_non_zero_dist = sorted_dists(1);

matches(dists >= lambda * min_non_zero_dist) = 0;

% remove double matches
if unique
    unique_matches = zeros(size(matches));
    [~,unique_match_idxs,~] = unique(matches, 'stable');
    unique_matches(unique_match_idxs) = matches(unique_match_idxs);

    matches = unique_matches;
end

[~, query_indices, database_indices] = find(matches);

end
