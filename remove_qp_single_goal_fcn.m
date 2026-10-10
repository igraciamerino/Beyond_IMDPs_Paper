function [Qp_intersects_pruned, Qp_flag_removed] = remove_qp_single_goal_fcn( Qp_intersects, vertices_q, a_Reach_nd_s_a, b_Reach_nd_s_a, Nq , solver_opts, children, S_target)

Qp_ind_keep = true(size(Qp_intersects));
for qp_ind = 1:length(Qp_intersects)
    qp = Qp_intersects(qp_ind);
    if ( any( ismember( children{qp}, S_target ) ) )
        Qp_ind_keep(qp_ind) = false;
    end
end

Qp_intersects_pruned = Qp_intersects( Qp_ind_keep );
Qp_flag_removed = ~Qp_ind_keep;

if ( isempty( Qp_ind_keep ) )
    disp('ERROR: NO PARENTS ARE FULLY CONTAINED WITHIN Reach_nd_s_a...')
end

end