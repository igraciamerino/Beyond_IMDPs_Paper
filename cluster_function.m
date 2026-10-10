function [lookup_table_clustered] = cluster_function(Y_table, p_lower, p_upper, tol)
% This function clusters data from lookup_table if the coordinates of the
% data are closer than tol. It takes the minimum value of p_lower and the
% maximum of p_lower and assigns it to the mean point of Y_table

aux_uniquetol = [Y_table' p_lower' p_upper'];

DS = [1 1 1 Inf Inf];
[~,ia] = uniquetol(aux_uniquetol, tol, 'ByRows', true, ...
    'OutputAllIndices', true, 'DataScale', DS);

for k = 1:length(ia)
    lookup_table_clustered.Y_table(k,:) = mean(aux_uniquetol(ia{k},1:3),1);
    lookup_table_clustered.p_lower(k,1) = min(aux_uniquetol(ia{k},4));
    lookup_table_clustered.p_upper(k,1) = max(aux_uniquetol(ia{k},5));
end

end

