function dist = mmd_dist_to_grid_superbrief(xu, xgrid, sigmaf, sigmal)
    % Find the 2 closest points in xgrid to xu
    dists = abs(xgrid - xu);
    [~, sortedIdx] = sort(dists);
    closest_indices = sortedIdx(1:2);
    xv = xgrid(closest_indices);
    uu = 1;
    if ( xv(1) > xv(2) )
        aux = xv(1);
        xv(1) = xv(2);
        xv(2) = aux;
    end
    dist = mmd_dist_to_grid_brief(uu, xu, xv, sigmaf, sigmal);
end