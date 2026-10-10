function [Qp_intersects_bool, is_subsumed_bool, error_flag] = compute_Qp_multi_imdp_dd_fcn(Sp_intersects, children, Nq)
% Compute bounds for transitions starting from q, under a

Qp_intersects_bool = false(Nq, 1);
is_subsumed_bool = false;
for q = 1:Nq
    if ( any(ismember( Sp_intersects, children{q} ) ) )
        Qp_intersects_bool(q) = true;
    end
end
if ( sum(Qp_intersects_bool) == 1 )
    is_subsumed_bool = true;
end
error_flag = false;

end