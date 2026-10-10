function [Qp_intersects_bool, Qp_subsumed_bool, error_flag] = compute_Qp_multi_imdp_fcn(Sp_intersects_small, Sp_intersects_big, children_q, Nq)
% Compute bounds for transitions starting from q, under a

Qp_intersects_bool = false(Nq, 1);
Qp_subsumed_bool = false(Nq, 1);
for qi = 1:Nq
    if ( any(ismember( Sp_intersects_big, children_q{qi} ) ) )
        Qp_intersects_bool(qi) = true;
        if ( ~any(~ismember( Sp_intersects_small, children_q{qi} ) ) )
            Qp_subsumed_bool(qi) = true;
        end
    end
end
error_flag = false;

end