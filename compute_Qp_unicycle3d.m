function [Q_intersects, error_flag] = compute_Qp_unicycle3d(a_Post, b_Post, Deltax, Deltay, Deltath, Nx, Ny, Nth, Nq, Nx_coarse, Ny_coarse, Nth_coarse, precision, coarse_Deltax_ratio, coarse_Deltay_ratio, coarse_Deltath_ratio, a_X, b_X)
% Compute bounds for transitions starting from q, under a

a_Post = a_Post - precision;
b_Post = b_Post + precision;

% New stuff

might_intersect_X = ( (b_Post(1) >= a_X(1)) && (a_Post(1) <= b_X(1)) && (b_Post(2) >= a_X(2)) && (a_Post(2) <= b_X(2)) ); % Post_q_a_w (might have) vertices inside bounds in x and y (might intersect X)
intersects_unsafe = ( (a_Post(1) <= a_X(1)) || (b_Post(1) >= b_X(1)) || (a_Post(2) <= a_X(2)) || (b_Post(2) >= b_X(2)) ); % In case that Post_q_a_w intersects unsafe set

if (might_intersect_X)

    % Logic to determine wrapping
    wraps_completely = false;
    wraps_dn_special_case_overlapping = false;
    if (a_Post(3) > a_X(3))
        n_cells_th_lb = ceil(a_Post(3)/Deltath);
        wraps_dn = false;
        % CAREFUL!! WITH OVERLAPPING CLUSTERS THERE MIGHT STILL BE
        % INTERSECTIONS!!!! 
        if ( a_Post(3) < a_X(3) + Deltath*(coarse_Deltath_ratio - 1) )
            wraps_dn_special_case_overlapping = true;
        end
    elseif (b_Post(3) > a_X(3)) % Post_q_u_w intersects th <= 0 but also with th > 0
        n_cells_th_lb = 1;
        wraps_dn = true;
    else % Post_q_u_w wraps below X
        wraps_dn = true;
        wraps_completely = true;
    end

    if (b_Post(3) < b_X(3))
        n_cells_th_ub = ceil(b_Post(3)/Deltath);
        wraps_up = false;
    elseif (a_Post(3) < b_X(3)) % Post_q_u_w goes beyond th >= 1 but also intersects th < 1
        n_cells_th_ub = Nth;
        wraps_up = true;
    else % Post_q_u_w wraps above X
        wraps_up = true;
        % CAREFUL!! WITH OVERLAPPING CLUSTERS THERE MIGHT STILL BE
        % INTERSECTIONS!!!! 
        if ( a_Post(3) < b_X(3) + Deltath*(coarse_Deltath_ratio - 1) )
            % Height of the lowest layer with coarse regions that go above
            % b_X(3)
            n_cells_th_lb = ceil(a_Post(3)/Deltath) - coarse_Deltath_ratio + 1; % Must be <= Nth
            n_cells_th_ub = Nth;
        else
            wraps_completely = true;
        end
    end

    % Logic to determine cell ranges in (x,y)

    if (a_Post(1) > a_X(1))
        n_cells_x_lb = ceil(a_Post(1)/Deltax);
    else
        n_cells_x_lb = 1; % The horizontal iterator will add on top of this one after having substracted 1
    end

    if (b_Post(1) < b_X(1))
        n_cells_x_ub = ceil(b_Post(1)/Deltax);
    else
        n_cells_x_ub = Nx; % The horizontal iterator will add on top of this one after having substracted 1
    end

    if (a_Post(2) > a_X(2))
        n_cells_y_lb = ceil(a_Post(2)/Deltay);
    else
        n_cells_y_lb = 1; % The horizontal iterator will add on top of this one after having substracted 1
    end

    if (b_Post(2) < b_X(2))
        n_cells_y_ub = ceil(b_Post(2)/Deltay);
    else
        n_cells_y_ub = Ny; % The horizontal iterator will add on top of this one after having substracted 1
    end

    % First, case of no wrapping
    if ( ~wraps_completely )
        [regions, ~] = cell_bounds2regions_coarse_unicycle3d_fcn(n_cells_x_lb, n_cells_x_ub, n_cells_y_lb, n_cells_y_ub, n_cells_th_lb, n_cells_th_ub, Nx_coarse, Ny_coarse, Nth_coarse, coarse_Deltax_ratio, coarse_Deltay_ratio, coarse_Deltath_ratio);
    else
        regions = [];
    end
    % Next, case of wrapping

    if ( (wraps_up)||(wraps_dn) || wraps_dn_special_case_overlapping )

        if (wraps_dn)
            a_Post = a_Post + Deltath*Nth*[0; 0; 1];
            b_Post = b_Post + Deltath*Nth*[0; 0; 1];
            n_cells_th_lb = ceil(a_Post(3)/Deltath);
            if (wraps_completely)
                n_cells_th_ub = ceil(b_Post(3)/Deltath);
            else % Post_q_u_w wrapped intersects th >= 1
                n_cells_th_ub = Nth;
            end
        elseif ( wraps_dn_special_case_overlapping )
            % CAREFUL!! WITH OVERLAPPING CLUSTERS THERE MIGHT STILL BE
            % INTERSECTIONS!!!!
            a_Post = a_Post + Deltath*Nth*[0; 0; 1];
            %         b_Post = b_Post + Deltath*Nth*[0; 0; 1];
            % Height of the lowest layer with coarse regions that go above
            % b_X(3)
            n_cells_th_lb = ceil(a_Post(3)/Deltath) - coarse_Deltath_ratio + 1; % Must be <= Nth
            n_cells_th_ub = Nth;
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

        [regions_wrapped, ~] = cell_bounds2regions_coarse_unicycle3d_fcn(n_cells_x_lb, n_cells_x_ub, n_cells_y_lb, n_cells_y_ub, n_cells_th_lb, n_cells_th_ub, Nx_coarse, Ny_coarse, Nth_coarse, coarse_Deltax_ratio, coarse_Deltay_ratio, coarse_Deltath_ratio);

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

if (Q_intersects(end) < 65535)
    Q_intersects = uint16(Q_intersects);
else
    Q_intersects = uint32(Q_intersects);
end

end