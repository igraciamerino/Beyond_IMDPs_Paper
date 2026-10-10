function [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_just_gurobi_skip_actions(user_param, initial_condition, solution, solver_opts)
% Solve the maximal reachability probability problem
% Currently it only uses lower bound to compute the optimal strategy

max_iter = user_param.max_iter;
Ns = user_param.Ns;
Nq = user_param.Nq;
Na = user_param.Na;
tol_VI = solver_opts.tol_VI;
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
gamma_prev = cell(Ns, user_param.Na);
model = cell(Ns, user_param.Na);
params.outputflag = 0;
params.Presolve = 1;
% Iterate the lower bound
tic
for i = 1:max_iter
    % Non-terminal states
    for s = user_param.S_not_terminal
        % Obtain (infty norm-based) distance by multiplying the number
        % of cells by the corresponding Delta
        for a = 1:Na
            %             s
            %             a
            %             if ((s == 8) && (a == 8))
            %                 100
            %             end
            V_lower_proj_Sp = V_lower(solution.Sp{s,a});
            if (i == 1)
                [A_aux, b_aux, ~, ~] = build_matrices_for_RDP_2layer(solution.P_lower_coarse{s,a}, solution.P_upper_coarse{s,a}, solution.Sp{s,a}, solution.Qp{s,a}, solution.children, Nq);
                % For umdp
%                 ind = find(abs(b_aux) > 0);
%                 A_aux = [A_aux(ind, :); eye(length(solution.P_upper{s,a}))];
%                 b_aux = [b_aux(ind); solution.P_upper{s,a}];
                A_aux = [A_aux; eye(length(solution.P_upper{s,a}))];
                b_aux = [b_aux; solution.P_upper{s,a}];
                % For IMDP
%                 A_aux = [A_aux(end, :); eye(length(solution.P_upper{s,a}))];
%                 b_aux = [b_aux(end); solution.P_upper{s,a}];
                model{s,a}.A = sparse( A_aux );
                model{s,a}.rhs = b_aux;
                %model{s,a}.lb = solution.P_lower{s,a};
                %model{s,a}.ub = solution.P_upper{s,a};
                model{s,a}.obj = V_lower_proj_Sp;
                result = gurobi(model{s,a}, params);
                if (~strcmp(result.status, 'OPTIMAL'))
                    disp('PROBLEM SOLVING WITH GUROBI...')
                end
                V1_lower_s_a(a) = result.objval;
                model{s,a}.vbasis = result.vbasis;
                model{s,a}.cbasis = result.cbasis;
                gamma_prev{s,a} = result.x;
            elseif ( ~disregard_action_fcn(V_lower_proj_Sp, gamma_prev{s,a}, V_lower(s)) )
                % Just do Gurobi
                    model{s,a}.obj = V_lower_proj_Sp;
                    result = gurobi(model{s,a}, params);
                    V1_lower_s_a(a) = result.objval;
                    model{s,a}.vbasis = result.vbasis;
                    model{s,a}.cbasis = result.cbasis;
                    gamma_prev{s,a} = result.x;
            else
                V1_lower_s_a(a) = 0;
            end
        end
        % Maximize in a \in Act
        %             V_lower(q) = max(V_lower_q_a);
        [V1_lower_max, a_max] = max(V1_lower_s_a);
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
    for s = user_param.S_not_terminal
        % Obtain (infty norm-based) distance by multiplying the number
        % of cells by the corresponding Delta
        a = sigma(s);
        V_upper_proj_Sp = V_upper(solution.Sp{s,a});
        if (i == 1)
            % Flip inequality for sum = 1
            model{s,a}.A(end - length(V_upper_proj_Sp), :) = -model{s,a}.A(end - length(V_upper_proj_Sp), :);
            model{s,a}.rhs(end - length(V_upper_proj_Sp)) = - model{s,a}.rhs(end - length(V_upper_proj_Sp));
            model{s,a}.obj = -V_upper_proj_Sp;
            model{s,a} = rmfield(model{s,a}, 'cbasis');
            model{s,a} = rmfield(model{s,a}, 'vbasis');
            result = gurobi(model{s,a}, params);
            if ( ~isfield(result, 'objval') )
                disp('adaas')
            end
            V1_upper(s) = -result.objval;
            model{s,a}.vbasis = result.vbasis;
            model{s,a}.cbasis = result.cbasis;
            gamma_prev{s,a} = result.x;
        else
            % Just do Gurobi
            model{s,a}.obj = -V_upper_proj_Sp;
            result = gurobi(model{s,a}, params);
            V1_upper(s) = -result.objval;
            model{s,a}.vbasis = result.vbasis;
            model{s,a}.cbasis = result.cbasis;
            gamma_prev{s,a} = result.x;
        end
        if ( V1_upper(s) > 1 )
            V1_upper(s) = 1;
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

