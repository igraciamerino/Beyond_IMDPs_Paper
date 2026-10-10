function [Q_ind_subsumed,error_flag] = subsumed_q_a_w(Post_q_a_w, vertices_q, Q_intersects, Deltath, Deltath_dot, Nth, Nq, precision)
% Compute bounds for transitions starting from q, under a

a_Post = min(Post_q_a_w,[],2) - precision;
b_Post = max(Post_q_a_w,[],2) + precision;

Q_ind_subsumed = false(size(Q_intersects));
if ( (b_Post(1) - a_Post(1) <= Deltath) && (b_Post(2) - a_Post(2) <= Deltath_dot) ) % Post is smaller than rectangle q

    % First, we consider no wrapping

    for qp_ind = 1:length(Q_intersects)
        qp = Q_intersects(qp_ind);
        if (qp ~= Nq+1) % qp is a safe state
            in1 = inhull(vertices_q{qp}', Post_q_a_w');
            if (~any(ismember(in1,0))) % Subsumed
                Q_ind_subsumed(qp_ind) = true;
            end
        end
    end

    % The following functionality accounts for wrapping

    if (a_Post(1) > 0)
        wraps_left = false;
    else % Post_q_u_w intersects th <= 0
        wraps_left = true;
        Post_q_a_w = Post_q_a_w + Deltath*Nth*[ones(1,size(Post_q_a_w,2)); zeros(1,size(Post_q_a_w,2))];
    end

    if (b_Post(1) < 1)
        wraps_right = false;
    else % Post_q_u_w goes beyond th >= 1
        wraps_right = true;
        Post_q_a_w = Post_q_a_w - Deltath*Nth*[ones(1,size(Post_q_a_w,2)); zeros(1,size(Post_q_a_w,2))];
    end

    if ( (wraps_right) || (wraps_left) )

        for qp_ind = 1:length(Q_intersects)
            qp = Q_intersects(qp_ind);
            if (qp ~= Nq+1) % qp is a safe state
                in1 = inhull(vertices_q{qp}', Post_q_a_w');
                if (~any(ismember(in1,0))) % Subsumed
                    Q_ind_subsumed(qp_ind) = true;
                end
            end
        end

    end
elseif (length(Q_intersects) == 1) % Post_q_a_w completely unsafe: qp = qu
    Q_ind_subsumed(1) = true;
end

% Check bounds of the abstraction
if (isempty(Q_ind_subsumed))
    disp('Error found while constructing the abstraction')
    error_flag = true;
else
    error_flag = false;
end

end