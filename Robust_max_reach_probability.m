function [P_reach_lower, P_reach_upper, sigma] = Robust_max_reach_probability(user_param, initial_condition, variables_for_matrices, state_discretization, tol_VI)
% Solve the maximal reachability probability problem
% Currently it only uses lower bound to compute the optimal strategy

max_iter = user_param.max_iter;
Nq = user_param.Nq;
Na = user_param.Na;

% Initialize interval value
V_lower = initial_condition.V_lower;
V_upper = initial_condition.V_upper;
sigma = initial_condition.sigma;
V1_lower = zeros(user_param.Nq+1,1);
V1_upper = zeros(user_param.Nq+1,1);
V1_lower_q_a = zeros(Na,1);

for q = user_param.Q_target
    V_lower(q) = 1;
    V_upper(q) = 1;
    V1_lower(q) = 1;
    V1_upper(q) = 1;
end

Deltax = state_discretization.Deltax;
d_min_nonzero_pow_p = max(state_discretization.Deltax, state_discretization.Deltay)^user_param.p; % Infty norm

% Interval value iteration

% Iterate the lower bound
P_reach_lower = zeros(Nq+1, 1);
lower_bound = true;
tic
for i = 1:max_iter
    for qi = 1:length(user_param.Q_not_terminal)
        q = user_param.Q_not_terminal(qi);
        % Obtain (infty norm-based) distance by multiplying the number
        % of cells by the corresponding Delta
        for a = 1:Na
            dist_pow_p = double(variables_for_matrices.dist_pow_p(variables_for_matrices.Q_nd{q,a}, variables_for_matrices.Qs{q,a}))*Deltax;
            [V1_lower_q_a(a)] = inner_problem_dual_gs(variables_for_matrices.epsilon_robust_pow_p{q,a}, variables_for_matrices.P_lower_nom{q,a}, variables_for_matrices.P_upper_nom{q,a}, V_lower( variables_for_matrices.Q_nd{q,a} ), variables_for_matrices.s{q,a}, dist_pow_p, d_min_nonzero_pow_p, lower_bound);
        end
        % Maximize in a \in Act
        %             V_lower(q) = max(V_lower_q_a);
        [V1_lower_aux, a_max] = max(V1_lower_q_a);
        if ( V1_lower_aux > V_lower(q) )
            sigma(q) = a_max;
            V1_lower(q) = V1_lower_aux;
        else
            V1_lower(q) = V_lower(q);
        end
    end


    err = max(abs(V1_lower(:) - V_lower(:)));

    V_lower = V1_lower;

    toc
    fprintf('Iteration = %d . \n', i)
    fprintf('Diference between consecutive value functions = %f . \n', err)

    if ( (i == max_iter) || (err < tol_VI) )
        P_reach_lower = V_lower;
        break;
    end
    % Just for debugging purposes
    %if (i == 1)
    %    estimated_time = user_param.max_iter*toc/60;
    %    fprintf('Estimated value iteration time = %d min. \n', estimated_time)
    %end

end
% Or don't iterate
% P_reach_lower = V_lower;

% Iterate the upper bound
lower_bound = false;
for i = 1:max_iter
    % Non-terminal states

    for qi = 1:length(user_param.Q_not_terminal)
        q = user_param.Q_not_terminal(qi);
        % Obtain (infty norm-based) distance by multiplying the number
        % of cells by the corresponding Delta
        a = sigma(q);
        dist_pow_p = double( variables_for_matrices.dist_pow_p(variables_for_matrices.Q_nd{q,a}, variables_for_matrices.Qs{q,a}) )*Deltax;
        V1_upper_aux = -inner_problem_dual_gs(variables_for_matrices.epsilon_robust_pow_p{q,a}, variables_for_matrices.P_lower_nom{q,a}, variables_for_matrices.P_upper_nom{q,a}, -V_upper( variables_for_matrices.Q_nd{q,a} ), variables_for_matrices.s{q,a}, dist_pow_p, d_min_nonzero_pow_p, lower_bound);
        if (V1_upper_aux > V1_upper(q))
            V1_upper(q) = V1_upper_aux;
        
        else
            V1_upper(q) = V_upper(q);
        end
    end

    err = max(abs(V1_upper(:) - V_upper(:)));

    V_upper = V1_upper;

    toc
    fprintf('Iteration = %d . \n', i)
    fprintf('Diference between consecutive value functions = %f . \n', err)

    if ( (i == max_iter) || (err < tol_VI) )
        P_reach_upper = V_upper;
        break;
    end

end

% % Or don't iterate
% P_reach_upper = V_upper;

end

