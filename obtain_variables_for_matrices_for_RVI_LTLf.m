function [variables_for_matrices] = obtain_variables_for_matrices_for_RVI_LTLf(user_param, solution)

% variables_for_matrices.Qs = cell(user_param.Ns,user_param.Na);
variables_for_matrices.Sp = solution.Sp;
variables_for_matrices.P_lower = solution.P_lower;
variables_for_matrices.P_upper = solution.P_upper;
variables_for_matrices.Qp = solution.Qp;
variables_for_matrices.P_lower_coarse = solution.P_lower_coarse;
variables_for_matrices.P_upper_coarse = solution.P_upper_coarse;
% variables_for_matrices.s = cell(user_param.Ns,user_param.Na);
% variables_for_matrices.epsilon_robust_pow_p = cell(user_param.Ns,user_param.Na);
variables_for_matrices.labelDFA = ones(user_param.Ns + 1, 1, 'uint8');
variables_for_matrices.ZpDFA = zeros(4, user_param.Ns + 1,'uint8');
variables_for_matrices.transDFA = zeros(4, 5,'uint8');
variables_for_matrices.z0_PROD = zeros(user_param.Ns + 1, 1,'uint8');
variables_for_matrices.parents = solution.parents;
variables_for_matrices.children = solution.children;

% % dist_pow_p is a big matrix
% variables_for_matrices.dist_pow_p = zeros(user_param.Ns + 1, 'uint8');
% for s = 1:user_param.Ns
%     distX = uint8( abs(state_discretization.X_center/state_discretization.Deltax - state_discretization.X_center(s)/Deltax) - 1 );
%     distY = uint8( abs(state_discretization.Y_center/state_discretization.Deltay - state_discretization.Y_center(s)/Deltay) - 1 );
%     
%     variables_for_matrices.dist_pow_p(1:end-1, s) = ( max([distX distY], [], 2) ).^user_param.p;
%     clearvars distX distY;
%     
%     % Display progress
%     if (rem(s,1000) == 0)
%         disp(s)
%     end
% end
% 
% dist_to_unsafe = zeros(1, user_param.Ns, 'uint8');
% for s = 1:user_param.Ns
%     x = [state_discretization.X_center(s); state_discretization.Y_center(s)];
%     dist_to_unsafe(s) = calculate_distance_to_unsafe_fast(original_sys.state_space_lower_left_corner, original_sys.state_space_upper_right_corner, x, Deltax, Deltay);
% end
% % Add last row and column
% variables_for_matrices.dist_pow_p(user_param.Ns + 1, 1:user_param.Ns) = dist_to_unsafe.^user_param.p;
    
% Compute variables for each q,a
% for s = 1:user_param.Ns
%     for a = 1:user_param.Na
%         variables_for_matrices.Qs{s,a} = support_P{s,a};
%         variables_for_matrices.Sp{s,a} = Sp{s,a};
%         variables_for_matrices.P_lower{s,a} = P_lower{s,a};
%         variables_for_matrices.P_upper{s,a} = P_upper{s,a};
%         variables_for_matrices.Qp{s,a} = Qp{s,a};
%         variables_for_matrices.P_lower_coarse{s,a} = P_lower_coarse{s,a};
%         variables_for_matrices.P_upper_coarse{s,a} = P_upper_coarse{s,a};
%         variables_for_matrices.s{s,a} = length(support_P{s,a});
%         variables_for_matrices.epsilon_robust_pow_p{s,a} = ( original_sys.Lw_q_a{s,a}*user_param.epsilon )^user_param.p;
%     end
% end

% Variables related to specification

% 1 = emptyset
% 2 = target
% 3 = obstacle
% 4 = water
% 5 = carpet

% Labeling fcn
variables_for_matrices.labelDFA(user_param.S_target) = uint8(2)*ones(length(user_param.S_target), 1, 'uint8');
variables_for_matrices.labelDFA(user_param.S_obstacle) = uint8(3)*ones(length(user_param.S_obstacle), 1, 'uint8');
variables_for_matrices.labelDFA(user_param.Ns + 1) = uint8(3);
variables_for_matrices.labelDFA(user_param.S_water) = uint8(4)*ones(length(user_param.S_water), 1, 'uint8');
variables_for_matrices.labelDFA(user_param.S_carpet) = uint8(5)*ones(length(user_param.S_carpet), 1, 'uint8');

% DFA transition fcn (rows -> states, columns -> observations)
variables_for_matrices.transDFA(1,:) = uint8([1 2 4 3 1]); % state z1 (initial)
variables_for_matrices.transDFA(2,:) = uint8([2 2 2 2 2]); % state z2 (accepting)
variables_for_matrices.transDFA(3,:) = uint8([3 4 4 3 1]); % state z3 (wet)
variables_for_matrices.transDFA(4,:) = uint8([4 4 4 4 4]); % state z4 (sink)

% zp = trans(z, label(qp)) (rows -> qp, columns -> z)
for sp = 1:user_param.Ns + 1
    for z = 1:4
        variables_for_matrices.ZpDFA(z,sp) = variables_for_matrices.transDFA(z, variables_for_matrices.labelDFA(sp)); % emptyset -> self transition
    end
end

% Initial state
for s = 1:user_param.Ns + 1
    for z = 1:4
        variables_for_matrices.z0_PROD(s) = variables_for_matrices.transDFA(1, variables_for_matrices.labelDFA(s)); % emptyset -> self transition
    end
end

end
