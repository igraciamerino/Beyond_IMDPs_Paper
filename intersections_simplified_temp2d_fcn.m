function [Q_intersects, error_flag] = intersections_simplified_temp2d_fcn(a_Post, b_Post, Deltax, Nx, Ns, aX, bX, precision)
% Compute bounds for transitions starting from q, under a

a_Post = a_Post - precision;
b_Post = b_Post + precision;

% might_intersect_X = ( (b_Post(1) >= aX(1)) && (a_Post(1) <= bX(1)) && (b_Post(2) >= aX(2)) && (a_Post(2) <= bX(2)) && (b_Post(3) >= aX(3)) && (a_Post(3) <= bX(3)) && (b_Post(4) >= aX(4)) && (a_Post(4) <= bX(4)) ); % Post_q_a_w (might have) vertices inside bounds in x and y (might intersect X)
% intersects_unsafe = ( (a_Post(1) <= aX(1)) || (b_Post(1) >= bX(1)) || (a_Post(2) <= aX(2)) || (b_Post(2) >= bX(2)) || (a_Post(3) <= aX(3)) || (b_Post(3) >= bX(3)) || (a_Post(4) <= aX(4)) || (b_Post(4) >= bX(4)) ); % In case that Post_q_a_w intersects unsafe set

might_intersect_X = ( ~any(b_Post < aX) && ~any(a_Post > bX) ); % Post_q_a_w (might have) vertices inside bounds in x and y (might intersect X)
intersects_unsafe = ( any(a_Post <= aX) || any(b_Post >= bX) ); % In case that Post_q_a_w intersects unsafe set


if (might_intersect_X)

    % Logic to determine cell ranges in (x,y, vel)

    if (a_Post(1) > aX(1))
        n_cells_x_lb = ceil( ( a_Post(1) - aX(1) )/Deltax{1});
    else
        n_cells_x_lb = 1; % The horizontal iterator will add on top of this one after having substracted 1
    end

    if (b_Post(1) < bX(1))
        n_cells_x_ub = ceil( ( b_Post(1) - aX(1) )/Deltax{1});
    else
        n_cells_x_ub = Nx{1}; % The horizontal iterator will add on top of this one after having substracted 1
    end

    if (a_Post(2) > aX(2))
        n_cells_y_lb = ceil( ( a_Post(2) - aX(2) )/Deltax{2});
    else
        n_cells_y_lb = 1; % The horizontal iterator will add on top of this one after having substracted 1
    end

    if (b_Post(2) < bX(2))
        n_cells_y_ub = ceil( ( b_Post(2) - aX(2) )/Deltax{2});
    else
        n_cells_y_ub = Nx{2}; % The horizontal iterator will add on top of this one after having substracted 1
    end

    [regions, ~] = cell_bounds2regions_2d_fcn(n_cells_x_lb, n_cells_x_ub, n_cells_y_lb, n_cells_y_ub, Nx{1});
    
    % Define Q_intersects

    if (intersects_unsafe)
        Q_intersects = [regions; Ns+1];
    else
        Q_intersects = regions;
    end
    error_flag = isempty(Q_intersects);

else % Post completely unsafe
    Q_intersects = Ns + 1;
    error_flag = false;
end

if (Ns < 65535)
    Q_intersects = uint16(Q_intersects);
else
    Q_intersects = uint32(Q_intersects);
end

end