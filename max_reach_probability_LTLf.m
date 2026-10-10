function [P_reach_lower, P_reach_upper, sigma] = max_reach_probability_LTLf(user_param, initial_condition, variables_for_matrices, solver_opts)
% Solve the maximal reachability probability problem
% Currently it only uses lower bound to compute the optimal strategy

tol_VI = solver_opts.tol_VI;
max_iter = user_param.max_iter;
Ns = user_param.Ns;
Na = user_param.Na;

% Initialize interval value
V_lower = initial_condition.V_lower;
V_upper = initial_condition.V_upper;
sigma = initial_condition.sigma_PROD;
V1_lower = zeros(user_param.Ns+1,4);
V1_upper = zeros(user_param.Ns+1,4);
V_lower_proj_S = zeros(Ns+1,1);
V_upper_proj_S = zeros(Ns+1,1);
V1_lower_s_a = zeros(Na,1);

% If state z \in Z_acc
V_lower(:,2) = ones(user_param.Ns+1,1);
V1_lower(:,2) = ones(user_param.Ns+1,1);
V_upper(:,2) = ones(user_param.Ns+1,1);
V1_upper(:,2) = ones(user_param.Ns+1,1);


% Interval value iteration

% Iterate the lower bound
Z_not_sink = [1 3];
tic
for i = 1:max_iter
    % Non-terminal states
    for z = Z_not_sink
        for sp = 1:Ns + 1
            zp = variables_for_matrices.ZpDFA(z,sp);
            V_lower_proj_S(sp) = V_lower(sp,zp);
        end
        for si = 1:length(user_param.S_not_terminal)
            s = user_param.S_not_terminal(si);
            % Obtain (infty norm-based) distance by multiplying the number
            % of cells by the corresponding Delta
            for a = 1:Na
                P_M = variables_for_matrices.P_lower{s,a} ; % Assign lower bound to all
                S = sum(P_M); % Get sum of lower bounds for fixed q and a
                % Order interval values
                V_lower_proj_Sp = V_lower_proj_S(variables_for_matrices.Sp{s,a});
                [~, order_V_lower] = sortrows(V_lower_proj_Sp, 'ascend');
                for sp = 1:length(V_lower_proj_Sp)
                    index = order_V_lower(sp);
                    if (S + variables_for_matrices.P_upper{s,a}(index) - variables_for_matrices.P_lower{s,a}(index) < 1)
                        S = S + variables_for_matrices.P_upper{s,a}(index) - variables_for_matrices.P_lower{s,a}(index);
                        P_M(index) = variables_for_matrices.P_upper{s,a}(index);
                    else
                        P_M(index) = P_M(index) + 1 - S;
                        break;
                    end
                end
                V1_lower_s_a(a) = P_M'*V_lower_proj_Sp;
            end
            % Maximize in a \in Act
            %             V_lower(q) = max(V_lower_q_a);
            [V1_lower(s,z), a_max] = max(V1_lower_s_a);
            if ( V1_lower(s,z) > V_lower(s,z) )
                sigma(s,z) = a_max;
            end
        end
    end

    err = max(abs(V1_lower(:) - V_lower(:)));
%     increasing = any( (V_lower(:) - V1_lower(:)) > 0 );
    increasing = false;
    V_lower = V1_lower;

    toc
    fprintf('Iteration = %d . \n', i)
    fprintf('Diference between consecutive value functions = %f . \n', err)

    if ( (i == max_iter) || (err < tol_VI) || increasing )
        if (increasing)
            disp('WARNING: Value iteration increases...')
        end
        P_reach_lower = V_lower;
        break;
    end

end
% Or don't iterate
% P_reach_lower = V_lower;

% Iterate the upper bound
for i = 1:max_iter
    % Non-terminal states
    for z = Z_not_sink
        for sp = 1:Ns + 1
            zp = variables_for_matrices.ZpDFA(z,sp);
            V_upper_proj_S(sp) = V_upper(sp,zp);
        end
        for si = 1:length(user_param.S_not_terminal)
            s = user_param.S_not_terminal(si);
            a = sigma(s,z);
            P_M = variables_for_matrices.P_lower{s,a}; % Assign lower bound to all
            S = sum(P_M); % Get sum of lower bounds for fixed q and a
            % Order interval values
            V_upper_proj_Sp = V_upper_proj_S(variables_for_matrices.Sp{s,a});
            [~, order_V_upper] = sortrows(V_upper_proj_Sp, 'descend');
            for sp = 1:length(V_upper_proj_Sp)
                index = order_V_upper(sp);
                if (S + variables_for_matrices.P_upper{s,a}(index) - variables_for_matrices.P_lower{s,a}(index) < 1)
                    S = S + variables_for_matrices.P_upper{s,a}(index) - variables_for_matrices.P_lower{s,a}(index);
                    P_M(index) = variables_for_matrices.P_upper{s,a}(index);
                else
                    P_M(index) = P_M(index) + 1 - S;
                    break;
                end
            end
            V1_upper(s,z) = P_M'*V_upper_proj_Sp;
        end
    end

    err = max(abs(V1_upper(:) - V_upper(:)));
    V_upper = V1_upper;

    toc
    fprintf('Iteration = %d . \n', i)
    fprintf('Diference between consecutive value functions = %f . \n', err)

    if ( (i == max_iter) || (err < tol_VI) || increasing )
        if (increasing)
            disp('WARNING: Value iteration increases...')
        end
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

