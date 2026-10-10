function [S_ind_subsumed, error_flag] = subsumed_q_a_w_simplified_pendulum_single_goal(a_Post, b_Post, vertices_s, S_intersects, a_X, b_X, Ns, precision, user_param)
% Compute bounds for transitions starting from q, under a

a_Post = a_Post + precision;
b_Post = b_Post - precision;

S_ind_subsumed = true(size(S_intersects));
for sp_ind = 1:length(S_intersects)
    sp = S_intersects(sp_ind);
    if (sp ~= Ns+1) % qp is a safe state
        verts = vertices_s{sp};
        if ( isempty(verts) )
            sp
        end
        a_qp = verts(:, 1);
        b_qp = verts(:, end);
        if ( any(a_qp > a_Post) || any(b_qp < b_Post) ) % Not subsumed
            S_ind_subsumed(sp_ind) = false;
        end
    else
        if ( (b_Post(2) > a_X(2)) && (a_Post(2) < b_X(2)) ) % Not subsumed in qp
            S_ind_subsumed(sp_ind) = false;
        end
    end
end

% if ( (b_Post(1) - a_Post(1) <= Deltath) && (b_Post(2) - a_Post(2) <= Deltath_dot) ) % Post is smaller than rectangle q
% 
%     % First, we consider no wrapping
% 
%     for qp_ind = 1:length(Q_intersects)
%         qp = Q_intersects(qp_ind);
%         if (qp ~= Nq+1) % qp is a safe state
%             in1 = inhull(vertices_q{qp}', Post_q_a_w');
%             if (~any(ismember(in1,0))) % Subsumed
%                 Q_ind_subsumed(qp_ind) = true;
%             end
%         end
%     end
% 
%     % The following functionality accounts for wrapping
% 
%     if (a_Post(1) > 0)
%         wraps_left = false;
%     else % Post_q_u_w intersects th <= 0
%         wraps_left = true;
%         Post_q_a_w = Post_q_a_w + Deltath*Nth*[ones(1,size(Post_q_a_w,2)); zeros(1,size(Post_q_a_w,2))];
%     end
% 
%     if (b_Post(1) < 1)
%         wraps_right = false;
%     else % Post_q_u_w goes beyond th >= 1
%         wraps_right = true;
%         Post_q_a_w = Post_q_a_w - Deltath*Nth*[ones(1,size(Post_q_a_w,2)); zeros(1,size(Post_q_a_w,2))];
%     end
% 
%     if ( (wraps_right) || (wraps_left) )
% 
%         for qp_ind = 1:length(Q_intersects)
%             qp = Q_intersects(qp_ind);
%             if (qp ~= Nq+1) % qp is a safe state
%                 in1 = inhull(vertices_q{qp}', Post_q_a_w');
%                 if (~any(ismember(in1,0))) % Subsumed
%                     Q_ind_subsumed(qp_ind) = true;
%                 end
%             end
%         end
% 
%     end
% elseif (length(Q_intersects) == 1) % Post_q_a_w completely unsafe: qp = qu
%     Q_ind_subsumed(1) = true;
% end

% Check bounds of the abstraction
if (isempty(S_ind_subsumed))
    disp('Error found while constructing the abstraction')
    error_flag = true;
else
    error_flag = false;
end

end