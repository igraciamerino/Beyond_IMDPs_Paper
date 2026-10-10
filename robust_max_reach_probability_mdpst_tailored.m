function [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_mdpst_tailored(user_param, initial_condition, solution, solver_opts)
% Solve the maximal reachability probability problem
% Currently it only uses lower bound to compute the optimal strategy

max_iter = user_param.max_iter;
tol_VI = solver_opts.tol_VI;
Na = user_param.Na;

% Initialize interval value
V_lower = initial_condition.V_lower;
V_upper = initial_condition.V_upper;
sigma = initial_condition.sigma;
V1_lower = zeros(user_param.Ns+1,1);
V1_upper = zeros(user_param.Ns+1,1);
V1_lower_s_a = zeros(Na,1);

% If state q \in Q_target
for s = user_param.S_target
    V_lower(s) = 1;
    V1_lower(s) = 1;
    V_upper(s) = 1;
    V1_upper(s) = 1;
end


% Interval value iteration

% Iterate the lower bound
tic
for i = 1:max_iter
    % Non-terminal states
    for s = user_param.S_not_terminal
        for a = 1:Na
            % First find the min value of V_lower in each q
            V_lower_c = zeros(user_param.Nc + 1, 1);
            for qp_ind = 1:user_param.Nc + 1
%                 V_lower_c_aux = 1;
                children = solution.Qp{s,a}.children{qp_ind}';
                V_lower_c(qp_ind) = min( V_lower(children) );
%                 for child = children
%                     if (V_lower(child) < V_lower_c_aux )
%                         V_lower_c_aux = V_lower(child);
%                     end
%                     V_lower_c(qp_ind) = V_lower_c_aux;
%                 end
            end
            P_M = solution.P_c; % Assign lower bound to all
            V1_lower_s_a(a) = P_M*V_lower_c;
        end
        % Maximize in a \in Act
        %             V_lower_c(q) = max(V_lower_c_q_a);
        [V1_lower_max, a_max] = max(V1_lower_s_a);
%         V1_lower_max = min(1, V1_lower_max);
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
% P_reach_lower = V_lower_c;

% Iterate the upper bound
for i = 1:max_iter
    % Non-terminal states
    for s = user_param.S_not_terminal
        a = sigma(s);
        % First find the min value of V_lower in each q
        V_upper_c = zeros(user_param.Nc + 1, 1);
        for qp_ind = 1:user_param.Nc
%             V_upper_c_aux = 0;
            children = solution.Qp{s,a}.children{qp_ind}';
            V_upper_c(qp_ind) = max( V_upper(children) );
%             for child = children
%                 if (V_upper(child) > V_upper_c_aux )
%                     V_upper_c_aux = V_upper(child);
%                 end
%                 V_upper_c(qp_ind) = V_upper_c_aux;
%             end
        end
        qp_ind = user_param.Nc + 1;
        V_upper_c(qp_ind) = max(V_upper);
        P_M = solution.P_c; % Assign lower bound to all
        V1_upper(s) = P_M*V_upper_c;
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

