function [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_safety_me_umdp_fcn(user_param, initial_condition, solution, solver_opts, state_discretization)
% Solve the maximal reachability probability problem
% Currently it only uses lower bound to compute the optimal strategy

max_iter = user_param.max_iter;
Ns = user_param.Ns;
Na = user_param.Na;
tol_VI = solver_opts.tol_VI;
S_not_terminal = user_param.S_not_terminal;
% Initialize interval value
V_lower = initial_condition.V_lower(1:end-1);
V_upper = initial_condition.V_upper(1:end-1);
sigma = initial_condition.sigma;
V1_lower = zeros(user_param.Ns,1);
V1_upper = zeros(user_param.Ns,1);
V1_lower_s_a = zeros(Na,1);
x_center = state_discretization.X_center{1};
q = solution.q;
r = solution.r;
sigmal = solver_opts.sigmal;
sigmaf = solver_opts.sigmaf;
epsilon = solution.epsilon;
% If state q \in Q_target
for s = user_param.S_terminal(1:end-1)
    V_lower(s) = 0;
    V1_lower(s) = 0;
    V_upper(s) = 0;
    V1_upper(s) = 0;
end

Q = kernel_gauss_fcn(x_center, x_center, sigmal, sigmaf);
Qc = (Q + Q')/2;
Aeq = ones(1, Ns);
beq = 1;

% Interval value iteration
% model = cell(Ns, user_param.Na);
params.outputflag = 0;
params.Presolve = 1;
% Iterate the lower bound
tic
for i = 1:max_iter
    for s = S_not_terminal
        for a = 1:Na
            % Solve with CVX
            cvx_begin quiet
            variable x(size(V_lower))
            minimize( V_lower' * x )
            subject to
            Aeq * x == beq
            x >= 0
            quad_form(x, Qc) + q{s,a}' * x <= epsilon^2 - r{s,a}
            cvx_end
            V1_lower_s_a(a) = V_lower' * x;
        end

        [V1_lower_max, a_max] = max(V1_lower_s_a);
        V1_lower(s) = min(1, V1_lower_max);
        sigma(s) = a_max;
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
% for i = 1:max_iter
%     for groupi = 1:n_groups
%         % Non-terminal states inside group
%         S_group_not_terminal = groups{groupi};
%         if (i == 1) % Construct matrices
%             model_cent{groupi} = [];
%             A_block = [];
%             b_block = [];
%             obj_block = [];
%             ind_interval = zeros(n_states_per_group_RDP, 1);
%             counter_s_a_in_group = 1;
%             for s = S_group_not_terminal
%                 a = sigma(s);
%                 %             s
%                 %             a
%                 %             if ((s == 8) && (a == 8))
%                 %                 100
%                 %             end
%                 V_upper_proj_Sp = V_upper(solution.Sp{s,a});
%                 [A_aux, b_aux, ~, ~] = build_matrices_for_RDP_2layer(solution.P_lower_coarse{s,a}, solution.P_upper_coarse{s,a}, solution.Sp{s,a}, solution.Qp{s,a}, solution.children, Nq);
%                 %                     ind = find(abs(b_aux) > 0);
%                 %                     A_aux = [sparse( A_aux(ind, :) ); sparse( eye(length(solution.P_upper{s,a})) )];
%                 %                     b_aux = [b_aux(ind); solution.P_upper{s,a}];
%                 % Flip inequality corresponding sum = 1
%                 A_aux(end, :) = -A_aux(end, :);
%                 b_aux(end) = -b_aux(end, :);
%                 A_aux = [sparse( A_aux ); sparse( eye(length(solution.P_upper{s,a})) )];
%                 b_aux = [b_aux; solution.P_upper{s,a}];
%                 ind_interval(counter_s_a_in_group, 1) = length(obj_block) + 1;
%                 A_block = blkdiag(A_block, A_aux );
%                 b_block = [b_block; b_aux];
%                 obj_block = [obj_block; V_upper_proj_Sp];
%                 ind_interval(counter_s_a_in_group, 2) = length(obj_block);
%                 counter_s_a_in_group = counter_s_a_in_group + 1;
%             end
%             model_cent{groupi}.A = A_block;
%             model_cent{groupi}.rhs = b_block;
%             model_cent{groupi}.obj = -obj_block;
%         else % Modify only the objective
%             obj_block = [];
%             ind_interval = zeros(n_states_per_group_RDP, 1);
%             counter_s_a_in_group = 1;
%             for s = S_group_not_terminal
%                 a = sigma(s);
%                 %             s
%                 %             a
%                 %             if ((s == 8) && (a == 8))
%                 %                 100
%                 %             end
%                 V_upper_proj_Sp = V_upper(solution.Sp{s,a});
%                 ind_interval(counter_s_a_in_group, 1) = length(obj_block) + 1;
%                 obj_block = [obj_block; V_upper_proj_Sp];
%                 ind_interval(counter_s_a_in_group, 2) = length(obj_block);
%                 counter_s_a_in_group = counter_s_a_in_group + 1;
%             end
%             model_cent{groupi}.obj = -obj_block;
%         end
%         % Clear block matrices
%         A_block = [];
%         b_block = [];
%         obj_block = [];
%         % Solve with gurobi
%         result = gurobi(model_cent{groupi}, params);
%         model_cent{groupi}.vbasis = result.vbasis;
%         model_cent{groupi}.cbasis = result.cbasis;
%         gamma_group = result.x';
%         V_upper_group = model_cent{groupi}.obj;
%         % Extract value for each (s , a) from result
%         counter_s_a_in_group = 1;
%         ind_interval = int64(ind_interval); % To avoid conversion errors
%         for s = S_group_not_terminal
%             gamma_s = gamma_group(ind_interval(counter_s_a_in_group, 1):ind_interval(counter_s_a_in_group, 2));
%             V_upper_s = -V_upper_group(ind_interval(counter_s_a_in_group, 1):ind_interval(counter_s_a_in_group, 2));
%             V1_upper(s) = gamma_s*V_upper_s;
%             counter_s_a_in_group = counter_s_a_in_group + 1;
%         end
%     end
% 
%     V1_upper = min( V1_upper, 1 );
%     err = max(abs(V1_upper - V_upper));
%     V_upper = V1_upper;
% 
%     toc
%     fprintf('Iteration = %d . \n', i)
%     fprintf('Diference between consecutive value functions = %f . \n', err)
% 
%     if ( (i == max_iter) || (err < tol_VI) )
%         P_reach_upper = V_upper;
%         break;
%     end
%     % Just for debugging purposes
%     %if (i == 1)
%     %    estimated_time = user_param.max_iter*toc/60;
%     %    fprintf('Estimated value iteration time = %d min. \n', estimated_time)
%     %end
% 
% end
% Or don't iterate
P_reach_upper = V_upper;

end

