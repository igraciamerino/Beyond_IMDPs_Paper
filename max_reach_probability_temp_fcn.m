function [P_reach_lower, P_reach_upper, sigma] = max_reach_probability_temp_fcn(user_param, initial_condition, solution, solver_opts)
% Solve the maximal reachability probability problem
% Currently it only uses lower bound to compute the optimal strategy

tol_VI = solver_opts.tol_VI;
max_iter = user_param.max_iter;
Na = user_param.Na;

% Initialize interval value
V_lower = initial_condition.V_lower;
V_upper = initial_condition.V_upper;
sigma = initial_condition.sigma;
V1_lower = zeros(user_param.Ns+1,1);
V1_upper = zeros(user_param.Ns+1,1);
V1_lower_s_a = zeros(Na,1);

% If state s \in Q_target
for s = user_param.S_terminal
    V_lower(s) = 0;
    V1_lower(s) = 0;
    V_upper(s) = 0;
    V1_upper(s) = 0;
end


% Interval value iteration

% Iterate the lower bound
tic
for i = 1:max_iter
    % Non-terminal states
    for si = 1:length(user_param.S_not_terminal)
        s = user_param.S_not_terminal(si);
        % Obtain (infty norm-based) distance by multiplying the number
        % of cells by the corresponding Delta
        for a = 1:Na
            P_M = solution.P_lower{s,a}; % Assign lower bound to all
            S = sum(P_M); % Get sum of lower bounds for fixed s and a
            % Order interval values
            V_lower_proj_Sp = V_lower(solution.Sp{s,a});
            [~, order_V_lower] = sortrows(V_lower_proj_Sp, 'ascend');
            for sp = 1:length(V_lower_proj_Sp)
                index = order_V_lower(sp);
                if (S + solution.P_upper{s,a}(index) - solution.P_lower{s,a}(index) < 1)
                    S = S + solution.P_upper{s,a}(index) - solution.P_lower{s,a}(index);
                    P_M(index) = solution.P_upper{s,a}(index);
                else
                    P_M(index) = P_M(index) + 1 - S;
                    break;
                end
            end
            V1_lower_s_a(a) = P_M'*V_lower_proj_Sp;
        end
        % Maximize in a \in Act
        [V1_lower_max, sigma(i, s)] = max(V1_lower_s_a);
        V1_lower(s) = min(1, V1_lower_max);
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
    for si = 1:length(user_param.S_not_terminal)
        s = user_param.S_not_terminal(si);
        a = sigma(i, s);
        P_M = solution.P_lower{s,a}; % Assign lower bound to all
        S = sum(P_M); % Get sum of lower bounds for fixed s and a
        % Order interval values
        V_upper_proj_Sp = V_upper(solution.Sp{s,a});
        [~, order_V_upper] = sortrows(V_upper_proj_Sp, 'descend');
        for sp = 1:length(V_upper_proj_Sp)
            index = order_V_upper(sp);
            if (S + solution.P_upper{s,a}(index) - solution.P_lower{s,a}(index) < 1)
                S = S + solution.P_upper{s,a}(index) - solution.P_lower{s,a}(index);
                P_M(index) = solution.P_upper{s,a}(index);
            else
                P_M(index) = P_M(index) + 1 - S;
                break;
            end
        end
        V1_upper_max = P_M'*V_upper_proj_Sp;
        V1_upper(s) = min(1, V1_upper_max);
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

