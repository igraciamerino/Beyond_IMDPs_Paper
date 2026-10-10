function [regions, length_output] = cell_bounds2regions_3d_fcn(n_cells_x_lb, n_cells_x_ub, n_cells_y_lb, n_cells_y_ub, n_cells_th_lb, n_cells_th_ub, Nx, Ny)

length_output = (n_cells_x_ub - n_cells_x_lb + 1)*(n_cells_y_ub - n_cells_y_lb + 1)*(n_cells_th_ub - n_cells_th_lb + 1);
regions = zeros(length_output,1);

index = 1;
aux_n_cells = zeros(3,1);
for iterator_th = n_cells_th_lb:n_cells_th_ub % Vertical loop (th)
    aux_n_cells(3) = (iterator_th - 1)*Nx*Ny; % -1 so we are at the end of previous plane (in th) of the discretization
    for iterator_y = n_cells_y_lb:n_cells_y_ub % Horizontal2 loop (y)
        aux_n_cells(2) = (iterator_y - 1)*Nx; % -1 so we are at the end of previous row (in y) of the discretization
        for iterator_x = n_cells_x_lb:n_cells_x_ub % Horizontal loop (x)
            regions(index) = aux_n_cells(3) + aux_n_cells(2) + iterator_x; % qp
            index = index + 1;
        end
    end
end

end

