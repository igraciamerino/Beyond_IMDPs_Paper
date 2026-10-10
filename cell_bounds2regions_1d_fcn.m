function [regions, length_output] = cell_bounds2regions_1d_fcn(n_cells_x_lb, n_cells_x_ub)

regions = n_cells_x_lb:n_cells_x_ub;
regions = regions';
length_output = length(regions);

end
