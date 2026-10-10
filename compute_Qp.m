function [Q_intersects, error_flag] = compute_Qp(a_Post, b_Post, Deltax, Deltay, Nx, Ny, Nq_coarse, Nx_coarse, Ny_coarse, precision, coarse_Deltax_ratio, coarse_Deltay_ratio, a_X, b_X)
% Compute bounds for transitions starting from q, under a

a_Post = a_Post - precision;
b_Post = b_Post + precision;

% New stuff

might_intersect_X = ( (b_Post(1) >= a_X(1)) && (a_Post(1) <= b_X(1)) && (b_Post(2) >= a_X(2)) && (a_Post(2) <= b_X(1)) ); % Post_q_a_w (might have) vertices inside bounds in x and y (might intersect X)
intersects_unsafe = ( (a_Post(1) <= a_X(1)) || (b_Post(1) >= b_X(1)) || (a_Post(2) <= a_X(2)) || (b_Post(2) >= b_X(2)) ); % In case that Post_q_a_w intersects unsafe set

if (might_intersect_X)

    % Logic to determine cell ranges in (x,y)

    if (a_Post(1) > 0)
        n_cells_x_lb = ceil(a_Post(1)/Deltax);
    else
        n_cells_x_lb = 1; % The horizontal iterator will add on top of this one after having substracted 1
    end

    if (b_Post(1) < 1)
        n_cells_x_ub = ceil(b_Post(1)/Deltax);
    else
        n_cells_x_ub = Nx; % The horizontal iterator will add on top of this one after having substracted 1
    end

    if (a_Post(2) > 0)
        n_cells_y_lb = ceil(a_Post(2)/Deltay);
    else
        n_cells_y_lb = 1; % The horizontal iterator will add on top of this one after having substracted 1
    end

    if (b_Post(2) < 1)
        n_cells_y_ub = ceil(b_Post(2)/Deltay);
    else
        n_cells_y_ub = Ny; % The horizontal iterator will add on top of this one after having substracted 1
    end
    
    [regions, ~] = cell_bounds2regions_coarse_fcn(n_cells_x_lb, n_cells_x_ub, n_cells_y_lb, n_cells_y_ub, Nx_coarse, Ny_coarse, coarse_Deltax_ratio, coarse_Deltay_ratio);

    % Define Q_intersects

    if (intersects_unsafe)
        Q_intersects = [regions; Nq_coarse+1];
    else
        Q_intersects = regions;
    end
    error_flag = isempty(Q_intersects);

else % Post completely unsafe
    Q_intersects = Nq_coarse + 1;
    error_flag = false;
end

if (Q_intersects(end) < 65535)
    Q_intersects = uint16(Q_intersects);
else
    Q_intersects = uint32(Q_intersects);
end

end