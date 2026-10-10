function [Q_intersects, error_flag] = intersections_simplified_pendulum_fcn(a_Post, b_Post, Deltax, Deltay, Nx, Ny, Ns, aX, bX, precision, user_param)
% Compute bounds for transitions starting from q, under a

a_Post = a_Post - precision;
b_Post = b_Post + precision;

intersects_unsafe = false;
Q_intersects = [];

if ( (b_Post(2) >= aX(2)) && (a_Post(2) <= bX(2)) ) % Post_q_a_w has vertices inside bounds in y (intersects X)

    % First, we consider no wrapping

    % Some logic to determine wrapping and limits
    wraps_completely = false;
    if (a_Post(1) > 0)
        n_cells_x_lb = ceil(a_Post(1)/Deltax);
        wraps_left = false;
    elseif (b_Post(1) > 0) % Post_q_u_w intersects th <= 0 but also with th > 0
        n_cells_x_lb = 1;
        wraps_left = true;
    else % Post_q_u_w wraps to the left of X
        wraps_left = true;
        wraps_completely = true;
    end

    if (b_Post(1) < 1)
        n_cells_x_ub = ceil(b_Post(1)/Deltax);
        wraps_right = false;
    elseif (a_Post(1) < 1) % Post_q_u_w goes beyond th >= 1 but also intersects th < 1
        n_cells_x_ub = Nx;
        wraps_right = true;
    else % Post_q_u_w wraps to the right of X
        wraps_right = true;
        wraps_completely = true;
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

    % Consider only what is within limits in th
    if (~wraps_completely)

        [Q_intersects, ~] = cell_bounds2regions_fcn(n_cells_x_lb, n_cells_x_ub, n_cells_y_lb, n_cells_y_ub, Nx);

        % Transition to the unsafe set
        if ( (a_Post(2) <= 0) || (b_Post(2) >= 1) ) % In case that Post_q_a_w intersects unsafe set
            intersects_unsafe = true;
        end


    elseif( (a_Post(2) <= 0) || (b_Post(2) >= 1) ) % In case that Post_q_a_w wraps completely and intersects unsafe set
        intersects_unsafe = true;

    else  % In case that Post_q_a_w wraps completely but doesn't intersect unsafe set

    end

    % The following functionality accounts for wrapping

    if ( (wraps_right)||(wraps_left) )

        if (wraps_left)
            a_Post = a_Post + Deltax*Nx*[1; 0];
            b_Post = b_Post + Deltax*Nx*[1; 0];
        else % Wraps right
            a_Post = a_Post - Deltax*Nx*[1; 0];
            b_Post = b_Post - Deltax*Nx*[1; 0];
        end

        if (a_Post(1) > 0)
            n_cells_x_lb = ceil(a_Post(1)/Deltax);
        else % Post_q_u_w wrapped intersects th <= 0
            n_cells_x_lb = 1;
        end

        if (b_Post(1) < 1)
            n_cells_x_ub = ceil(b_Post(1)/Deltax);
        else % Post_q_u_w_wrapped goes beyond th >= 1
            n_cells_x_ub = Nx;
        end

        % We already checked that Post_q_a_w inside bounds in y and
        % computed n_cells_y_lb and n_cells_y_ub (we can reuse them)

        [Q_intersects_wrapped, ~] = cell_bounds2regions_fcn(n_cells_x_lb, n_cells_x_ub, n_cells_y_lb, n_cells_y_ub, Nx);

        % Concatenate only elements of Q_intersects_wrapped that are not in Q_intersects
        if(~isempty(Q_intersects_wrapped))
            Q_intersects = [Q_intersects; Q_intersects_wrapped(~ismember(Q_intersects_wrapped, Q_intersects))];
        end
    end
    
else % Post_q_a_w completely outside safe set
    intersects_unsafe = true;
end

if (intersects_unsafe)
    Q_intersects = [Q_intersects; Ns+1];
end


if (Ns < 65535)
    Q_intersects = uint16(Q_intersects);
else
    Q_intersects = uint32(Q_intersects);
end

% Check bounds of the abstraction
if (isempty(Q_intersects))
    disp('Error found while constructing the abstraction')
    error_flag = true;
else
    error_flag = false;
end

end