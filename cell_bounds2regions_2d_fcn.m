function [regions, length_output] = cell_bounds2regions_2d_fcn(n_cells_x_lb, n_cells_x_ub, n_cells_y_lb, n_cells_y_ub, Nx)

length_output = (n_cells_x_ub - n_cells_x_lb + 1)*(n_cells_y_ub - n_cells_y_lb + 1);
regions = zeros(length_output,1);

index = 1;
for iterator_y = n_cells_y_lb:n_cells_y_ub % Vertical loop (y)
    aux_n_cells = (iterator_y - 1)*Nx; % -1 so we are at the end of previous row (in y) of the discretization
    for iterator_x = n_cells_x_lb:n_cells_x_ub % Horizontal loop (x)
        regions(index) = aux_n_cells + iterator_x; % qp
        index = index + 1;
    end
end


end

