function [Q_intersects, error_flag] = intersections_q_a_w(Post_q_a_w, vertices_q, Deltax, Deltay, Deltath, Nx, Ny, Nth, Nq, precision)
% Compute bounds for transitions starting from q, under a

a_Post = min(Post_q_a_w,[],2) - precision;
b_Post = max(Post_q_a_w,[],2) + precision;

might_intersect_X = ( (b_Post(1) >= 0) && (a_Post(1) <= 1) && (b_Post(2) >= 0) && (a_Post(2) <= 1) ); % Post_q_a_w (might have) vertices inside bounds in x and y (might intersect X)
intersects_unsafe = ( (a_Post(1) <= 0) || (b_Post(1) >= 1) || (a_Post(2) <= 0) || (b_Post(2) >= 1) ); % In case that Post_q_a_w intersects unsafe set

if (might_intersect_X)

    % Logic to determine wrapping
    wraps_completely = false;
    if (a_Post(3) > 0)
        n_cells_th_lb = ceil(a_Post(3)/Deltath);
        wraps_dn = false;
    elseif (b_Post(3) > 0) % Post_q_u_w intersects th <= 0 but also with th > 0
        n_cells_th_lb = 1;
        wraps_dn = true;
    else % Post_q_u_w wraps below X
        wraps_dn = true;
        wraps_completely = true;
    end

    if (b_Post(3) < 1)
        n_cells_th_ub = ceil(b_Post(3)/Deltath);
        wraps_up = false;
    elseif (a_Post(3) < 1) % Post_q_u_w goes beyond th >= 1 but also intersects th < 1
        n_cells_th_ub = Nth;
        wraps_up = true;
    else % Post_q_u_w wraps above X
        wraps_up = true;
        wraps_completely = true;
    end

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

    % First, case of no wrapping
    if (~wraps_completely)
        [regions, length_regions] = cell_bounds2regions_fcn(n_cells_x_lb, n_cells_x_ub, n_cells_y_lb, n_cells_y_ub, n_cells_th_lb, n_cells_th_ub, Nx, Ny);
    else
        length_regions = 0;
    end
    % Next, case of wrapping

    if ( (wraps_up)||(wraps_dn) )

        if (wraps_dn)
            a_Post = a_Post + Deltath*Nth*[0; 0; 1];
            b_Post = b_Post + Deltath*Nth*[0; 0; 1];
            Post_q_a_w_wrapped = Post_q_a_w + Deltath*Nth*[zeros(2,size(Post_q_a_w,2)); ones(1,size(Post_q_a_w,2))];
            n_cells_th_lb = ceil(a_Post(3)/Deltath);
            if (wraps_completely)
                n_cells_th_ub = ceil(b_Post(3)/Deltath);
            else % Post_q_u_w wrapped intersects th >= 1
                n_cells_th_ub = Nth;
            end
        else % Wraps up
            a_Post = a_Post - Deltath*Nth*[0; 0; 1];
            b_Post = b_Post - Deltath*Nth*[0; 0; 1];
            Post_q_a_w_wrapped = Post_q_a_w - Deltath*Nth*[zeros(2,size(Post_q_a_w,2)); ones(1,size(Post_q_a_w,2))];
            n_cells_th_ub = ceil(b_Post(3)/Deltath);
            if (wraps_completely)
                n_cells_th_lb = ceil(a_Post(3)/Deltath);
            else % Post_q_u_w wrapped intersects th <= 0
                n_cells_th_lb = 1;
            end
        end

        [regions_wrapped, length_regions_wrapped] = cell_bounds2regions_fcn(n_cells_x_lb, n_cells_x_ub, n_cells_y_lb, n_cells_y_ub, n_cells_th_lb, n_cells_th_ub, Nx, Ny);

    else
        length_regions_wrapped = 0;
    end

    % Define Q_intersects

    if (intersects_unsafe)
        Q_intersects = zeros(length_regions + length_regions_wrapped + 1, 1);
    else
        Q_intersects = zeros(length_regions + length_regions_wrapped, 1);
    end

    % Now check intersections

    counter = 1;
    for qp_ind = 1:length_regions
        qp = regions(qp_ind);
        a_qp = vertices_q{qp}(:,1);
        b_qp = vertices_q{qp}(:,8);
        intersects = check_intersection_pol_cube(Post_q_a_w, a_qp, b_qp);
        if (intersects)
            Q_intersects(counter) = qp;
            counter = counter + 1;
        end
    end

    for qp_ind = 1:length_regions_wrapped
        qp = regions_wrapped(qp_ind);
        a_qp = vertices_q{qp}(:,1);
        b_qp = vertices_q{qp}(:,8);
        intersects = check_intersection_pol_cube(Post_q_a_w_wrapped,a_qp,b_qp);
        if (intersects)
            Q_intersects(counter) = qp;
            counter = counter + 1;
        end
    end
    
    % Transition to the unsafe set
    if (intersects_unsafe)
        Q_intersects(counter) = Nq + 1;
    end

    % Remove zeros
    Q_intersects = nonzeros(Q_intersects);
    error_flag = isempty(Q_intersects);

else % Post completely unsafe
    Q_intersects = Nq + 1;
    error_flag = false;
end

end