function [Q_intersects,error_flag] = intersections_q_a_w_2(Post_q_a_w, vertices_q, Deltath, Deltath_dot, Nth, Nth_dot, Nq, precision, support_P)
% Compute bounds for transitions starting from q, under a

a_Post = min(Post_q_a_w,[],2) - precision;
b_Post = max(Post_q_a_w,[],2) + precision;

if ( (b_Post(2) >= 0) && (a_Post(2) <= 1) ) % Post_q_a_w has vertices inside bounds in th_dot (intersects X)

    % First, we consider no wrapping

    % Some logic to determine wrapping and limits
    wraps_completely = false;
    if (a_Post(1) > 0)
        n_cells_th_lb = ceil(a_Post(1)/Deltath);
        wraps_left = false;
    elseif (b_Post(1) > 0) % Post_q_u_w intersects th <= 0 but also with th > 0
        n_cells_th_lb = 1;
        wraps_left = true;
    else % Post_q_u_w wraps to the left of X
        wraps_left = true;
        wraps_completely = true;
    end

    if (b_Post(1) < 1)
        n_cells_th_ub = ceil(b_Post(1)/Deltath);
        wraps_right = false;
    elseif (a_Post(1) < 1) % Post_q_u_w goes beyond th >= 1 but also intersects th < 1
        n_cells_th_ub = Nth;
        wraps_right = true;
    else % Post_q_u_w wraps to the right of X
        wraps_right = true;
        wraps_completely = true;
    end

    if (a_Post(2) > 0)
        n_cells_th_dot_lb = ceil(a_Post(2)/Deltath_dot);
    else
        n_cells_th_dot_lb = 1; % The horizontal iterator will add on top of this one after having substracted 1
    end

    if (b_Post(2) < 1)
        n_cells_th_dot_ub = ceil(b_Post(2)/Deltath_dot);
    else
        n_cells_th_dot_ub = Nth_dot; % The horizontal iterator will add on top of this one after having substracted 1
    end

    % Consider only what is within limits in th
    if (~wraps_completely)

        length_output = (n_cells_th_dot_ub - n_cells_th_dot_lb + 1)*(n_cells_th_ub - n_cells_th_lb + 1) + 1; % +1 to account for the unsafe set
        Q_intersects = zeros(length_output,1);
        % Negative length means for loop does not trigger and Q_intersects is
        % empty. Go to wrapped functionality
        %     if (length_output <= 0)
        %         disp('Negative length found')
        %         error_flag = true;
        %     else
        %         error_flag = false;
        %     end

        index = 1;
        for iterator_n_cells_v = n_cells_th_dot_lb:n_cells_th_dot_ub % Vertical loop (th_dot)
            aux_v = (iterator_n_cells_v - 1)*Nth; % -1 so we are at the end of previous row of the discretization
            for iterator_n_cells_h = n_cells_th_lb:n_cells_th_ub % Horizontal loop (th)
                qp = uint16(aux_v + iterator_n_cells_h);
                if ( (~isinteger(qp)) || (qp < 1) )
                    disp('qp not positive integer')
                end
                if (any(ismember(support_P,qp))) % If qp is in support_P, then it is in Reach_nd_q_u
                    Q_intersects(index) = qp;
                    index = index + 1;
                else
                    a_qp = vertices_q{qp}(:,1);
                    b_qp = vertices_q{qp}(:,3);
                    intersects = check_intersection(Post_q_a_w,a_qp,b_qp);
                    if (intersects)
                        Q_intersects(index) = qp;
                        index = index + 1;
                    end
                end
            end
        end

        % Transition to the unsafe set
        if ( (a_Post(2) <= 0) || (b_Post(2) >= 1) ) % In case that Post_q_a_w intersects unsafe set
            Q_intersects(index) = Nq+1;
        end

        Q_intersects = nonzeros(Q_intersects);

    elseif( (a_Post(2) <= 0) || (b_Post(2) >= 1) ) % In case that Post_q_a_w wraps completely and intersects unsafe set
        length_output = 1;
        Q_intersects = Nq+1;

    else  % In case that Post_q_a_w wraps completely but doesn't intersect unsafe set
        length_output = 1;
        Q_intersects = [];
    end

    % The following functionality accounts for wrapping

    if ( (wraps_right)||(wraps_left) )

        if (wraps_left)
            a_Post = a_Post + Deltath*Nth*[1; 0];
            b_Post = b_Post + Deltath*Nth*[1; 0];
            Post_q_a_w = Post_q_a_w + Deltath*Nth*[ones(1,size(Post_q_a_w,2)); zeros(1,size(Post_q_a_w,2))];
        else % Wraps right
            a_Post = a_Post - Deltath*Nth*[1; 0];
            b_Post = b_Post - Deltath*Nth*[1; 0];
            Post_q_a_w = Post_q_a_w - Deltath*Nth*[ones(1,size(Post_q_a_w,2)); zeros(1,size(Post_q_a_w,2))];
        end

        if (a_Post(1) > 0)
            n_cells_th_lb = ceil(a_Post(1)/Deltath);
        else % Post_q_u_w wrapped intersects th <= 0
            n_cells_th_lb = 1;
        end

        if (b_Post(1) < 1)
            n_cells_th_ub = ceil(b_Post(1)/Deltath);
        else % Post_q_u_w_wrapped goes beyond th >= 1
            n_cells_th_ub = Nth;
        end

        % We already checked that Post_q_a_w inside bounds in th_dot and
        % computed n_cells_th_dot_lb and n_cells_th_dot_ub (we can reuse them)

        length_output_wrapped = (n_cells_th_dot_ub - n_cells_th_dot_lb + 1)*(n_cells_th_ub - n_cells_th_lb + 1); % No need to account for the unsafe set again
        if (length_output <= 0)||(length_output_wrapped <= 0)
            disp('Negative length found')
        error_flag = true;
        else
            error_flag = false;
        end
        Q_intersects_wrapped = zeros(length_output_wrapped,1);

        index = 1;
        for iterator_n_cells_v = n_cells_th_dot_lb:n_cells_th_dot_ub % Vertical loop (th_dot)
            aux_v = (iterator_n_cells_v - 1)*Nth; % -1 so we are at the end of previous row of the discretization
            for iterator_n_cells_h = n_cells_th_lb:n_cells_th_ub % Horizontal loop (th)
                qp = uint16(aux_v + iterator_n_cells_h);
                if ( (~isinteger(qp)) || (qp < 1) )
                    disp('qp not positive integer')
                end
                if (any(ismember(support_P,qp))) % If qp is in support_P, then it is in Reach_nd_q_u
                    Q_intersects_wrapped(index) = qp;
                    index = index + 1;
                else
                    a_qp = vertices_q{qp}(:,1);
                    b_qp = vertices_q{qp}(:,3);
                    intersects = check_intersection(Post_q_a_w,a_qp,b_qp);
                    if (intersects)
                        Q_intersects_wrapped(index) = qp;
                        index = index + 1;
                    end
                end
            end
        end

        % No need to account again for transitions to the unsafe set
        Q_intersects_wrapped = nonzeros(Q_intersects_wrapped);

        % Concatenate only elements of Q_intersects_wrapped that are not in Q_intersects
        if(~isempty(Q_intersects_wrapped))
            Q_intersects = [Q_intersects; Q_intersects_wrapped(~ismember(Q_intersects_wrapped, Q_intersects))];
        end
    end
    
else % Post_q_a_w completely outside safe set
    Q_intersects = Nq + 1;
end

% Check bounds of the abstraction
if (isempty(Q_intersects))
    disp('Error found while constructing the abstraction')
    error_flag = true;
else
    error_flag = false;
end

end