function [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_2layer_linprog(user_param, initial_condition, solution, state_discretization, tol_VI)
% Solve the maximal reachability probability problem
% Currently it only uses lower bound to compute the optimal strategy

max_iter = user_param.max_iter;
Nq = user_param.Nq;
Na = user_param.Na;
Deltax = state_discretization.Deltax;
Deltay = state_discretization.Deltay;

Nq_coarse = user_param.Nq_coarse;
Deltax_coarse = state_discretization.Deltax_coarse;
Deltay_coarse = state_discretization.Deltay_coarse;

% Initialize interval value
V_lower = initial_condition.V_lower;
V_upper = initial_condition.V_upper;
sigma = initial_condition.sigma;
V1_lower = zeros(user_param.Nq+1,1);
V1_upper = zeros(user_param.Nq+1,1);
V1_lower_q_a = zeros(Na,1);

% If state q \in Q_target
for s = user_param.Q_target
    V_lower(s) = 1;
    V1_lower(s) = 1;
    V_upper(s) = 1;
    V1_upper(s) = 1;
end


if (Deltax ~= Deltay)
    disp('This only works for hypercubes...')
    return;
end

% Interval value iteration

% Iterate the lower bound
tic
for i = 1:max_iter
    % Non-terminal states
    for si = 1:length(user_param.Q_not_terminal)
        s = user_param.Q_not_terminal(si);
        % Obtain (infty norm-based) distance by multiplying the number
        % of cells by the corresponding Delta
        for a = 1:Na
%             s
%             a
%             if ((s == 8) && (a == 1))
%                 100
%             end
            V_lower_proj_Q_nd = V_lower(solution.Q_nd{s,a});
            [A, b, Aeq, beq] = build_matrices_for_RDP(solution.P_lower_coarse{s,a}, solution.P_upper_coarse{s,a}, solution.Q_nd{s,a}, solution.Q_nd_coarse{s,a}, solution.children);
            [V1_lower_q_a(a), ~] = RDP_inner_maximization_linprog(V_lower_proj_Q_nd, A, b, Aeq, beq, solution.P_lower{s,a}, solution.P_upper{s,a});
        end
%             if ((s == 17) && (a == 8))
%                 V1_lower_q_a(a)
%             end
        % Maximize in a \in Act
        %             V_lower(q) = max(V_lower_q_a);
        [V1_lower_max, a_max] = max(V1_lower_q_a);
        V1_lower_max = min(1, V1_lower_max);
        if ( V1_lower_max > V_lower(s) )
            V1_lower(s) = V1_lower_max;
            sigma(s) = a_max;
        else
            V1_lower(s) = V_lower(s);
        end
    end

    err = max(abs(V1_lower - V_lower));
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
for i = 1:max_iter
    % Non-terminal states
    for si = 1:length(user_param.Q_not_terminal)
        s = user_param.Q_not_terminal(si);
        a = sigma(s);
        P_M = zeros(length( solution.P_upper{s,a}), 1); % Assign lower bound to all
        S = sum(P_M); % Get sum of lower bounds for fixed q and a
        % Order interval values
        V_upper_proj_Q_nd = V_upper(solution.Q_nd{s,a});
        [~, order_V_upper] = sortrows(V_upper_proj_Q_nd, 'descend');
        for qp = 1:length(V_upper_proj_Q_nd)
            index = order_V_upper(qp);
            if (S + solution.P_upper{s,a}(index) -0 < 1)
                S = S + solution.P_upper{s,a}(index) - 0;
                P_M(index) = solution.P_upper{s,a}(index);
            else
                P_M(index) = P_M(index) + 1 - S;
                break;
            end
        end
        V1_upper_max = P_M'*V_upper_proj_Q_nd;
        if (V1_upper_max > V_upper(s))
            V1_upper(s) = V1_upper_max;
        else
            V1_upper(s) = V_upper(s);
        end
    end

    err = max(abs(V1_upper - V_upper));
    V_upper = V1_upper;

    toc
    fprintf('Iteration = %d . \n', i)
    fprintf('Diference between consecutive value functions = %f . \n', err)

    if ( (i == max_iter) || (err < tol_VI) )
        P_reach_upper = V_upper;
        break;
    end
    % Just for debugging purposes
    %if (i == 1)
    %    estimated_time = user_param.max_iter*toc/60;
    %    fprintf('Estimated value iteration time = %d min. \n', estimated_time)
    %end

end

% Or don't iterate
% P_reach_upper = V_upper;

end

