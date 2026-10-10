function [regions, length_output] = cell_bounds2regions_coarse_unicycle3d_fcn(n_cells_x_lb, n_cells_x_ub, n_cells_y_lb, n_cells_y_ub, n_cells_th_lb, n_cells_th_ub, Nx_coarse, Ny_coarse, Nth_coarse, coarse_Deltax_ratio, coarse_Deltay_ratio, coarse_Deltath_ratio)

n_cells_x_lb = max(n_cells_x_lb - coarse_Deltax_ratio + 1, 1);
n_cells_x_ub = min(n_cells_x_ub + coarse_Deltax_ratio - 1, Nx_coarse);
n_cells_y_lb = max(n_cells_y_lb - coarse_Deltay_ratio + 1, 1);
n_cells_y_ub = min(n_cells_y_ub + coarse_Deltay_ratio - 1, Ny_coarse);
%THIS ONE WRAPS, SO DON'T MODIFY IT
length_output = (n_cells_x_ub - n_cells_x_lb + 1)*(n_cells_y_ub - n_cells_y_lb + 1)*(n_cells_th_ub - n_cells_th_lb + 1);


% if ( (n_cells_x_ub - n_cells_x_lb + 1 - 2 < 1 ) || (n_cells_y_ub - n_cells_y_lb + 1 - 2 < 0) ) % Q_nd doesn't fully cover a coarse region
%     regions = [];
%     return;
% end
regions = zeros(length_output,1);

index = 1;
aux_n_cells = zeros(3,1);
for iterator_th = n_cells_th_lb:n_cells_th_ub % Vertical loop (th)
    aux_n_cells(3) = (iterator_th - 1)*Nx_coarse*Ny_coarse; % -1 so we are at the end of previous plane (in th) of the discretization
    for iterator_y = n_cells_y_lb:n_cells_y_ub % Horizontal2 loop (y)
        aux_n_cells(2) = (iterator_y - 1)*Nx_coarse; % -1 so we are at the end of previous row (in y) of the discretization
        for iterator_x = n_cells_x_lb:n_cells_x_ub % Horizontal loop (x)
            regions(index) = aux_n_cells(3) + aux_n_cells(2) + iterator_x; % qp
            index = index + 1;
        end
    end
end

% if ( isempty( regions) )
%     regions = [];
% end


end

