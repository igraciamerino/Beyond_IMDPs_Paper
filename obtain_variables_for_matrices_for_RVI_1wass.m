function [variables_for_matrices] = obtain_variables_for_matrices_for_RVI_1wass(user_param, original_sys, state_discretization, support_P, Q_nd, P_upper,P_lower)

% THIS ONLY WORKS FOR THE INFTY NORM AND HYPERCUBES

Deltax = state_discretization.Deltax;
Deltay = state_discretization.Deltay;

variables_for_matrices.Qs = cell(user_param.Nq,user_param.Na);
variables_for_matrices.Q_nd = cell(user_param.Nq,user_param.Na);
variables_for_matrices.P_lower_nom = cell(user_param.Nq,user_param.Na);
variables_for_matrices.P_upper_nom = cell(user_param.Nq,user_param.Na);
variables_for_matrices.s = cell(user_param.Nq,user_param.Na);
variables_for_matrices.epsilon_robust_pow_p = cell(user_param.Nq,user_param.Na);

% dist_pow_p is a big matrix
variables_for_matrices.dist_pow_p = zeros(user_param.Nq + 1, 'uint8');
for q = 1:user_param.Nq
    distX = uint8( abs(state_discretization.X_center/state_discretization.Deltax - state_discretization.X_center(q)/Deltax) - 1 );
    distY = uint8( abs(state_discretization.Y_center/state_discretization.Deltay - state_discretization.Y_center(q)/Deltay) - 1 );
    
    variables_for_matrices.dist_pow_p(1:end-1, q) = ( max([distX distY], [], 2) ).^user_param.p;
    clearvars distX distY;
    
    % Display progress
    if (rem(q,1000) == 0)
        disp(q)
    end
end

dist_to_unsafe = zeros(1, user_param.Nq, 'uint8');
for q = 1:user_param.Nq
    x = [state_discretization.X_center(q); state_discretization.Y_center(q)];
    dist_to_unsafe(q) = uint8( floor( calculate_distance_to_unsafe_fast(original_sys.state_space_lower_left_corner, original_sys.state_space_upper_right_corner, x) / Deltax ) );
end
% Add last row and column
variables_for_matrices.dist_pow_p(user_param.Nq + 1, 1:user_param.Nq) = dist_to_unsafe.^user_param.p;
    
% Compute variables for each q,a
for q = 1:user_param.Nq
    for a = 1:user_param.Na
        variables_for_matrices.Qs{q,a} = support_P{q,a};
        variables_for_matrices.Q_nd{q,a} = Q_nd{q,a};
        variables_for_matrices.P_lower_nom{q,a} = P_lower{q,a};
        variables_for_matrices.P_upper_nom{q,a} = P_upper{q,a};
        variables_for_matrices.s{q,a} = length(support_P{q,a});
        variables_for_matrices.epsilon_robust_pow_p{q,a} = ( original_sys.Lw_q_a*user_param.epsilon )^user_param.p;
    end
end



end
