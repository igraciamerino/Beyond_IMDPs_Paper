function [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_multi_imdp_just_gurobi_memory(user_param, initial_condition, solution, solver_opts)
% Solve the maximal reachability probability problem
% Currently it only uses lower bound to compute the optimal strategy

max_iter = user_param.max_iter;
Ns = user_param.Ns;
Nc = user_param.Nc;
Na = user_param.Na;
tol_VI = solver_opts.tol_VI;
n_states_per_group_RDP = solver_opts.n_states_per_group_RDP;
S_not_terminal = user_param.S_not_terminal;
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
params.outputflag = 0;
params.Presolve = 1;
% Iterate the lower bound
tic

for i = 1:max_iter
    for s = S_not_terminal
        for a = 1:Na
            V_lower_proj_Sp = V_lower(solution.Sp{s,a});
            [A_aux, b_aux, ~, ~] = build_matrices_for_RDP_multi_imdp_fcn(solution.P_lower_coarse{s,a}, solution.P_upper_coarse{s,a}, solution.Sp{s,a}, solution.Qp{s,a}.children_q);
            A_aux = [sparse( A_aux ); sparse( -eye(length(solution.P_lower{s,a})) ); sparse( eye(length(solution.P_upper{s,a})) )];
            b_aux = [b_aux; -solution.P_lower{s,a}; solution.P_upper{s,a}];
            A_aux = [sparse( A_aux )];
            model.A = A_aux;
            model.rhs = b_aux;
            model.obj = V_lower_proj_Sp;
            % Solve with gurobi
            result = gurobi(model, params);
            V1_lower_s_a(a) = result.objval;
            [V1_lower_max, a_max] = max(V1_lower_s_a);
            V1_lower_max = min(1, V1_lower_max);
            if ( V1_lower_max > V_lower(s) )
                V1_lower(s) = V1_lower_max;
                sigma(s) = a_max;
            else
                V1_lower(s) = V_lower(s);
            end
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
    for s = S_not_terminal
        a = sigma(s);
        V_upper_proj_Sp = V_upper(solution.Sp{s,a});
        [A_aux, b_aux, ~, ~] = build_matrices_for_RDP_multi_imdp_fcn(solution.P_lower_coarse{s,a}, solution.P_upper_coarse{s,a}, solution.Sp{s,a}, solution.Qp{s,a}.children_q);
        % Flip inequality corresponding sum = 1
        A_aux(end, :) = -A_aux(end, :);
        b_aux(end) = -b_aux(end, :);
        A_aux = [sparse( A_aux ); sparse( -eye(length(solution.P_lower{s,a})) ); sparse( eye(length(solution.P_upper{s,a})) )];
        b_aux = [b_aux; -solution.P_lower{s,a}; solution.P_upper{s,a}];
        A_aux = [sparse( A_aux )];
        model.A = A_aux;
        model.rhs = b_aux;
        model.obj = -V_upper_proj_Sp;
        % Solve with gurobi
        result = gurobi(model, params);
        V1_upper(s) = -result.objval;
    end
    V1_upper = min( V1_upper, 1 );
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
P_reach_upper = V_upper;

end

