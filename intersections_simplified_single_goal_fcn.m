function [Q_intersects, error_flag] = intersections_simplified_single_goal_fcn(a_Post, b_Post, Deltax, Deltay, Nx, Ny, Nq, aX, bX, precision, user_param)
% Compute bounds for transitions starting from q, under a

a_Post = a_Post - precision;
b_Post = b_Post + precision;
n_target_blocks = user_param.n_target_blocks;
S_target_blocks = user_param.S_target_blocks;
s_target_blocks = user_param.s_target_blocks;

might_intersect_X = ( (b_Post(1) >= aX(1)) && (a_Post(1) <= bX(1)) && (b_Post(2) >= aX(2)) && (a_Post(2) <= bX(2)) ); % Post_q_a_w (might have) vertices inside bounds in x and y (might intersect X)
intersects_unsafe = ( (a_Post(1) <= aX(1)) || (b_Post(1) >= bX(1)) || (a_Post(2) <= aX(2)) || (b_Post(2) >= bX(2)) ); % In case that Post_q_a_w intersects unsafe set

if (might_intersect_X)

    % Logic to determine cell ranges in (x,y)

    if (a_Post(1) > aX(1))
        n_cells_x_lb = ceil(a_Post(1)/Deltax);
    else
        n_cells_x_lb = 1; % The horizontal iterator will add on top of this one after having substracted 1
    end

    if (b_Post(1) < bX(1))
        n_cells_x_ub = ceil(b_Post(1)/Deltax);
    else
        n_cells_x_ub = Nx; % The horizontal iterator will add on top of this one after having substracted 1
    end

    if (a_Post(2) > aX(2))
        n_cells_y_lb = ceil(a_Post(2)/Deltay);
    else
        n_cells_y_lb = 1; % The horizontal iterator will add on top of this one after having substracted 1
    end

    if (b_Post(2) < bX(2))
        n_cells_y_ub = ceil(b_Post(2)/Deltay);
    else
        n_cells_y_ub = Ny; % The horizontal iterator will add on top of this one after having substracted 1
    end
    
    [regions, ~] = cell_bounds2regions_fcn(n_cells_x_lb, n_cells_x_ub, n_cells_y_lb, n_cells_y_ub, Nx);

    % Define Q_intersects

    if (intersects_unsafe)
        Q_intersects = [regions; Nq+1];
    else
        Q_intersects = regions;
    end
    error_flag = isempty(Q_intersects);

else % Post completely unsafe
    Q_intersects = Nq + 1;
    error_flag = false;
end

% Lump target states into a single one
for blocki = 1:n_target_blocks
    S_target_block = S_target_blocks{blocki};
    s_target_block = s_target_blocks{blocki};
    ind_target = ismember(Q_intersects, S_target_block);
    if ( any( ind_target ) )
        Q_intersects = [s_target_block; Q_intersects(~ind_target)];
    end
end

if (Nq < 65535)
    Q_intersects = uint16(Q_intersects);
else
    Q_intersects = uint32(Q_intersects);
end

end