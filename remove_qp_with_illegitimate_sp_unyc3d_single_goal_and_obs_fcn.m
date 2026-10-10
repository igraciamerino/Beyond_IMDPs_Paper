% function [Qp_intersects_pruned, Qp_flag_removed] = remove_qp_with_illegitimate_sp_unyc3d_single_goal_and_obs_fcn( Qp_intersects, vertices_q, a_Reach_nd_s_a, b_Reach_nd_s_a, Nq , solver_opts, children, S_target, S_obstacles, a_X, b_X)
function [Qp_intersects_pruned, Qp_flag_removed] = remove_qp_with_illegitimate_sp_unyc3d_single_goal_and_obs_fcn( Qp_intersects, Sp, children, Nq)

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

% Old stuff

% if ( a_Reach_nd_s_a(3) < a_X(3) )
%     wraps = true;
%     a_Reach_nd_s_a_wrapped = a_Reach_nd_s_a + (b_X(3) - a_X(3))*[0; 0; 1];
%     b_Reach_nd_s_a_wrapped = b_Reach_nd_s_a + (b_X(3) - a_X(3))*[0; 0; 1];
% elseif ( b_Reach_nd_s_a(3) > b_X(3) )
%     wraps = true;
%     a_Reach_nd_s_a_wrapped = a_Reach_nd_s_a - (b_X(3) - a_X(3))*[0; 0; 1];
%     b_Reach_nd_s_a_wrapped = b_Reach_nd_s_a - (b_X(3) - a_X(3))*[0; 0; 1];
% else
%     wraps = false;
% end
% 
% Qp_ind_keep = true(size(Qp_intersects));
% for qp_ind = 1:length(Qp_intersects)
%     qp = Qp_intersects(qp_ind);
%     if (qp ~= Nq+1) % qp is a safe state
%         verts = vertices_q{qp};
%         a_qp = verts(:, 1) - solver_opts.precision;
%         b_qp = verts(:, end) + solver_opts.precision;
%         
%         if ( any( a_qp < a_Reach_nd_s_a ) || any( b_qp > b_Reach_nd_s_a ) ) % Not subsumed
%             if ( wraps )
%                 if ( any( a_qp < a_Reach_nd_s_a_wrapped ) || any( b_qp > b_Reach_nd_s_a_wrapped ) ) % Not subsumed
%                     Qp_ind_keep(qp_ind) = false;
%                 end
%             else
%                 Qp_ind_keep(qp_ind) = false;
%             end
%         end
%         % Remove qp that intersect S_target or S_obstacles
%         if ( any( ismember( children{qp}, S_target ) ) || any( ismember( children{qp}, S_obstacles ) ) )
%             Qp_ind_keep(qp_ind) = false;
%         end
%     end
% end
% 
% Qp_intersects_pruned = Qp_intersects( Qp_ind_keep );
% Qp_flag_removed = ~Qp_ind_keep;

if ( isempty( Qp_intersects_pruned ) )
    disp('ERROR: NO PARENTS ARE FULLY CONTAINED WITHIN Reach_nd_s_a...')
end

end