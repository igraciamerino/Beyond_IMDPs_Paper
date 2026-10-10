function [variables_for_matrices] = obtain_variables_for_matrices_for_RVI(user_param, original_sys, state_discretization, support_P, Sp, P_lower, P_upper)

variables_for_matrices.Sp = cell(user_param.Nq,user_param.Na);
variables_for_matrices.P_lower_nom = cell(user_param.Nq,user_param.Na);
variables_for_matrices.P_upper_nom = cell(user_param.Nq,user_param.Na);
variables_for_matrices.s = cell(user_param.Nq,user_param.Na);
variables_for_matrices.epsilon_robust_pow_p = cell(user_param.Nq,user_param.Na);

% Compute minimum distance between any regions q \in Q
% dist = 2*norm(original_sys.state_space_upper_right_corner-original_sys.state_space_lower_left_corner,2)*ones(user_param.Nq+1);
% % dist(end,:) = 0;
% for i = 1:user_param.Nq
%     vertices_qi = state_discretization.vertices_q{i};
%     for j = 1:user_param.Nq
%         vertices_qj = state_discretization.vertices_q{j};
%         for ii = 1:4
%             for jj = 1:4
%                 norm_2 = norm( vertices_qi(:,ii) - vertices_qj(:,jj), user_param.m );
%                 dist(i,j) = min(dist(i,j), norm_2);
%             end
%         end
%     end
% end
% i = user_param.Nq + 1;
% for j = 1:user_param.Nq
%     vertices_qj = state_discretization.vertices_q{j};
%     for jj = 1:4
%         dist_to_outside_of_X = calculate_distance_to_unsafe_fast(original_sys.state_space_lower_left_corner, original_sys.state_space_upper_right_corner, vertices_qj(:,jj));
%         dist(i,j) = min(dist(i,j), dist_to_outside_of_X);
%     end
% end
% dist(end) = 0;
% % dist(end,:) = dist(:,end)';
% variables_for_matrices.dist_pow_p = dist.^user_param.p;

% Compute variables for each q,a
for q = 1:user_param.Nq
    for a = 1:user_param.Na
        variables_for_matrices.Sp{s,a} = support_P{s,a};
        variables_for_matrices.Q_nd{s,a} = Sp{s,a};
        variables_for_matrices.P_lower_nom{s,a} = P_lower{s,a};
        variables_for_matrices.P_upper_nom{s,a} = P_upper{s,a};
        variables_for_matrices.s{s,a} = length(support_P{s,a});
    end
end

end