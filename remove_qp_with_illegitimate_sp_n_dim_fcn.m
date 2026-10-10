function [Qp_intersects_pruned, Qp_flag_removed] = remove_qp_with_illegitimate_sp_n_dim_fcn( Qp_intersects, Sp, children, Nq)

Qp_flag_removed = false(size(Qp_intersects));
for qpi = 1:length(Qp_intersects)
    qp = Qp_intersects(qpi);
    if (qp ~= Nq+1)
        children_q = children{qp};
        if ( any(~ismember(children_q, Sp)) ) % Unrecognized child
            Qp_flag_removed(qpi) = true;
        end
    end
end
Qp_intersects_pruned = Qp_intersects(~Qp_flag_removed);

if ( isempty( Qp_intersects_pruned ) )
    disp('ERROR: NO PARENTS ARE FULLY CONTAINED WITHIN Reach_nd_s_a...')
end

end