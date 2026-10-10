function [Q_intersects, error_flag] = intersections_simplified_unicycle3d_single_goal_and_obs_fcn(a_Post, b_Post, Deltax, Deltay, Deltath, Nx, Ny, Nth, Nq, aX, bX, precision, user_param)
% Compute bounds for transitions starting from q, under a

a_Post = a_Post - precision;
b_Post = b_Post + precision;

might_intersect_X = ( (b_Post(1) >= aX(1)) && (a_Post(1) <= bX(1)) && (b_Post(2) >= aX(2)) && (a_Post(2) <= bX(2)) ); % Post_q_a_w (might have) vertices inside bounds in x and y (might intersect X)
intersects_unsafe = ( (a_Post(1) <= aX(1)) || (b_Post(1) >= bX(1)) || (a_Post(2) <= aX(2)) || (b_Post(2) >= bX(2)) ); % In case that Post_q_a_w intersects unsafe set

if (might_intersect_X)

    % Logic to determine wrapping
    wraps_completely = false;
    if (a_Post(3) > aX(3))
        n_cells_th_lb = ceil(a_Post(3)/Deltath);
        wraps_dn = false;
    elseif (b_Post(3) > aX(3)) % Post_q_u_w intersects th <= 0 but also with th > 0
        n_cells_th_lb = 1;
        wraps_dn = true;
    else % Post_q_u_w wraps below X
        wraps_dn = true;
        wraps_completely = true;
    end

    if (b_Post(3) < bX(3))
        n_cells_th_ub = ceil(b_Post(3)/Deltath);
        wraps_up = false;
    elseif (a_Post(3) < bX(3)) % Post_q_u_w goes beyond th >= 1 but also intersects th < 1
        n_cells_th_ub = Nth;
        wraps_up = true;
    else % Post_q_u_w wraps above X
        wraps_up = true;
        wraps_completely = true;
    end

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

    % First, case of no wrapping
    if (~wraps_completely)
        [regions, ~] = cell_bounds2regions_3d_unicycle_fcn(n_cells_x_lb, n_cells_x_ub, n_cells_y_lb, n_cells_y_ub, n_cells_th_lb, n_cells_th_ub, Nx, Ny);
    else
        regions = [];
    end
    % Next, case of wrapping

    if ( (wraps_up)||(wraps_dn) )

        if (wraps_dn)
            a_Post = a_Post + Deltath*Nth*[0; 0; 1];
            b_Post = b_Post + Deltath*Nth*[0; 0; 1];
            n_cells_th_lb = ceil(a_Post(3)/Deltath);
            if (wraps_completely)
                n_cells_th_ub = ceil(b_Post(3)/Deltath);
            else % Post_q_u_w wrapped intersects th >= 1
                n_cells_th_ub = Nth;
            end
        else % Wraps up
            a_Post = a_Post - Deltath*Nth*[0; 0; 1];
            b_Post = b_Post - Deltath*Nth*[0; 0; 1];
            n_cells_th_ub = ceil(b_Post(3)/Deltath);
            if (wraps_completely)
                n_cells_th_lb = ceil(a_Post(3)/Deltath);
            else % Post_q_u_w wrapped intersects th <= 0
                n_cells_th_lb = 1;
            end
        end

        [regions_wrapped, ~] = cell_bounds2regions_3d_unicycle_fcn(n_cells_x_lb, n_cells_x_ub, n_cells_y_lb, n_cells_y_ub, n_cells_th_lb, n_cells_th_ub, Nx, Ny);

    else
        regions_wrapped = [];
    end

    % Define Q_intersects

    if (intersects_unsafe)
        Q_intersects = [regions; regions_wrapped; Nq+1];
    else
        Q_intersects = [regions; regions_wrapped];
    end
    error_flag = isempty(Q_intersects);

else % Post completely unsafe
    Q_intersects = Nq + 1;
    error_flag = false;
end

% Lump target states into a single one
ind_target = ismember(Q_intersects, user_param.S_target);
if ( any( ind_target ) )
    Q_intersects = [user_param.s_target; Q_intersects(~ind_target)];
end

% Lump obstacle states into a single one
ind_obstacles = ismember(Q_intersects, user_param.S_obstacles);
if ( any( ind_obstacles ) )
    Q_intersects = [user_param.s_obstacle; Q_intersects(~ind_obstacles)];
end

if (Nq < 65535)
    Q_intersects = uint16(Q_intersects);
else
    Q_intersects = uint32(Q_intersects);
end

end