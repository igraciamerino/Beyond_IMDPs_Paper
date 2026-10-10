function [Q_subsumed, error_flag] = subsumed_q_a_w_coarse_simplified(a_Post, b_Post, vertices_q_coarse, Q_intersects, bool_Q_subsumed, aux_Q_coarse, a_X, b_X, Nq_coarse, precision)
% Compute bounds for transitions starting from q, under a

a_Post = a_Post + precision;
b_Post = b_Post - precision;

for qp_coarse = Q_intersects
    if (qp_coarse ~= Nq_coarse+1) % qp is a safe state
        verts = vertices_q_coarse{qp_coarse};
        a_qp = verts(:, 1);
        b_qp = verts(:, end);
        if ( ~any(a_qp > a_Post) && ~any(b_qp < b_Post) ) % Subsumed
            bool_Q_subsumed(qp_coarse) = true;
        end
    else
        if ( (b_Post(1) < a_X(1)) || (a_Post(1) > b_X(1)) || (b_Post(2) < a_X(2)) || (a_Post(2) > b_X(2)) ) % Subsumed in qp
            bool_Q_subsumed(end) = true;
        end
    end
end

Q_subsumed = aux_Q_coarse( bool_Q_subsumed );

% Check bounds of the abstraction (THIS TIME IT CAN BE EMPTY)
error_flag = false;
% if (isempty(Q_subsumed))
%     error_flag = true;
% else
%     error_flag = false;
% end

end