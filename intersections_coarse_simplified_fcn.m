function [Qp_intersects, error_flag] = intersections_coarse_simplified_fcn(Sp_intersects, parents_coarse, bool_Qp_intersects, aux_Q_coarse, a_X, b_X, Nq, precision)
% Compute bounds for transitions starting from q, under a

for sp = Sp_intersects
    parents = parents_coarse{sp};
    for qp = parents'
        bool_Qp_intersects(qp) = true;
    end
end

Qp_intersects = aux_Q_coarse( bool_Qp_intersects );

% Check bounds of the abstraction (THIS TIME IT CAN BE EMPTY)

    error_flag = false;
% if (isempty(Q_subsumed))
%     
%     error_flag = false;
% else
%     error_flag = false;
% end

end