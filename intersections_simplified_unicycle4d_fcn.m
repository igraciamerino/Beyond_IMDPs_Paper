function [Q_intersects, error_flag] = intersections_simplified_unicycle4d_fcn(a_Post, b_Post, Deltax, Nx, Ns, aX, bX, precision)
% Compute bounds for transitions starting from q, under a

a_Post = a_Post - precision;
b_Post = b_Post + precision;

% New stuff

might_intersect_X = ( (b_Post(1) >= aX(1)) && (a_Post(1) <= bX(1)) && (b_Post(2) >= aX(2)) && (a_Post(2) <= bX(2)) && (b_Post(4) >= aX(4)) && (a_Post(4) <= bX(4)) ); % Post_q_a_w (might have) vertices inside bounds in x and y (might intersect X)
intersects_unsafe = ( (a_Post(1) <= aX(1)) || (b_Post(1) >= bX(1)) || (a_Post(2) <= aX(2)) || (b_Post(2) >= bX(2)) || (a_Post(4) <= aX(4)) || (b_Post(4) >= bX(4)) ); % In case that Post_q_a_w intersects unsafe set

if (might_intersect_X)

    % Logic to determine wrapping
    wraps_completely = false;
    if ( a_Post(3) > aX(3) )
        n_cells_th_lb = ceil(a_Post(3)/Deltax{3});
        wraps_dn = false;
    elseif (b_Post(3) > aX(3)) % Post_q_u_w intersects th <= 0 but also with th > 0
        n_cells_th_lb = 1;
        wraps_dn = true;
    else % Post_q_u_w wraps below X
        wraps_dn = true;
        wraps_completely = true;
    end

    if ( b_Post(3) < bX(3) )
        n_cells_th_ub = ceil(b_Post(3)/Deltax{3});
        wraps_up = false;
    elseif (a_Post(3) < bX(3)) % Post_q_u_w goes beyond th >= 1 but also intersects th < 1
        n_cells_th_ub = Nx{3};
        wraps_up = true;
    else % Post_q_u_w wraps above X
        wraps_up = true;
        wraps_completely = true;
    end

    % Logic to determine cell ranges in (x,y, vel)

    if (a_Post(1) > aX(1))
        n_cells_x_lb = ceil(a_Post(1)/Deltax{1});
    else
        n_cells_x_lb = 1; % The horizontal iterator will add on top of this one after having substracted 1
    end

    if (b_Post(1) < bX(1))
        n_cells_x_ub = ceil(b_Post(1)/Deltax{1});
    else
        n_cells_x_ub = Nx{1}; % The horizontal iterator will add on top of this one after having substracted 1
    end

    if (a_Post(2) > aX(2))
        n_cells_y_lb = ceil(a_Post(2)/Deltax{2});
    else
        n_cells_y_lb = 1; % The horizontal iterator will add on top of this one after having substracted 1
    end

    if (b_Post(2) < bX(2))
        n_cells_y_ub = ceil(b_Post(2)/Deltax{2});
    else
        n_cells_y_ub = Nx{2}; % The horizontal iterator will add on top of this one after having substracted 1
    end

    if (a_Post(4) > aX(4))
        n_cells_vel_lb = ceil(a_Post(4)/Deltax{4});
    else
        n_cells_vel_lb = 1; % The horizontal iterator will add on top of this one after having substracted 1
    end

    if (b_Post(4) < bX(4))
        n_cells_vel_ub = ceil(b_Post(4)/Deltax{4});
    else
        n_cells_vel_ub = Nx{4}; % The horizontal iterator will add on top of this one after having substracted 1
    end

    % First, case of no wrapping
    if (~wraps_completely)
        [regions, ~] = cell_bounds2regions_4d_unicycle_fcn(n_cells_x_lb, n_cells_x_ub, n_cells_y_lb, n_cells_y_ub, n_cells_th_lb, n_cells_th_ub, n_cells_vel_lb, n_cells_vel_ub, Nx);
    else
        regions = [];
    end
    % Next, case of wrapping

    if ( (wraps_up)||(wraps_dn) )

        if (wraps_dn)
            a_Post = a_Post + Deltax{3}*Nx{3}*[0; 0; 1; 0];
            b_Post = b_Post + Deltax{3}*Nx{3}*[0; 0; 1; 0];
            n_cells_th_lb = ceil(a_Post(3)/Deltax{3});
            if (wraps_completely)
                n_cells_th_ub = ceil(b_Post(3)/Deltax{3});
            else % Post_q_u_w wrapped intersects th >= 1
                n_cells_th_ub = Nx{3};
            end
        else % Wraps up
            a_Post = a_Post - Deltax{3}*Nx{3}*[0; 0; 1; 0];
            b_Post = b_Post - Deltax{3}*Nx{3}*[0; 0; 1; 0];
            n_cells_th_ub = ceil(b_Post(3)/Deltax{3});
            if (wraps_completely)
                n_cells_th_lb = ceil(a_Post(3)/Deltax{3});
            else % Post_q_u_w wrapped intersects th <= 0
                n_cells_th_lb = 1;
            end
        end

        [regions_wrapped, ~] = cell_bounds2regions_4d_unicycle_fcn(n_cells_x_lb, n_cells_x_ub, n_cells_y_lb, n_cells_y_ub, n_cells_th_lb, n_cells_th_ub, n_cells_vel_lb, n_cells_vel_ub, Nx);

    else
        regions_wrapped = [];
    end

    % Define Q_intersects

    if (intersects_unsafe)
        Q_intersects = [regions; regions_wrapped; Ns+1];
    else
        Q_intersects = [regions; regions_wrapped];
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