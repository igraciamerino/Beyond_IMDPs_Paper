function solution = RDP(solution, user_param, solver_opts, original_sys, state_discretization)


if ( strcmp(original_sys.name, 'unicycle2d_ltlf' ) )
    % Warm/cold start
    if (solution.warm_start_RDP) % Warm start
        initial_condition.V_lower = solution.P_reach_lower_PROD;
        initial_condition.V_upper = solution.P_reach_upper_PROD;
        initial_condition.sigma_PROD = solution.sigma_PROD;
        disp('Warm starting value iteration')
    else % Cold start
        initial_condition.V_lower = zeros(user_param.Ns+1, 4);
        initial_condition.V_upper = zeros(user_param.Ns+1, 4);
        initial_condition.sigma_PROD = ones(user_param.Ns, 4);
        disp('Starting value iteration')
    end
    if (1) % (~solution.variables_for_matrices_computed)
        disp('Computing variables for matrices')
        tic
        [variables_for_matrices] = obtain_variables_for_matrices_for_RVI_LTLf(user_param, solution);
        time_obtain_variables_for_matrices = toc/60;
        solution.variables_for_matrices_computed = true;
        % (Only for debugging purposes)
        fprintf('Time required to obtain variables for matrices = %d min. \n', time_obtain_variables_for_matrices)
    else
        disp('Variables for matrices are already computed')
    end
elseif ( strcmp(original_sys.name, 'temp' ) || strcmp(original_sys.name, 'fm1' ) || strcmp(original_sys.name, 'fm2' ) || strcmp(original_sys.name, 'lr' ) )
    % Warm/cold start
    if (solution.warm_start_RDP) % Warm start
        initial_condition.V_lower = solution.P_reach_lower;
        initial_condition.V_upper = solution.P_reach_upper;
        initial_condition.sigma = solution.sigma;
        disp('Warm starting value iteration')
    else % Cold start
        initial_condition.V_lower = ones(user_param.Ns+1, 1);
        initial_condition.V_upper = ones(user_param.Ns+1, 1);
        initial_condition.V_lower(user_param.S_terminal) = 0;
        initial_condition.V_upper(user_param.S_terminal) = 0;
        initial_condition.sigma = ones(user_param.max_iter, user_param.Ns + 1);
        disp('Starting value iteration')
        solution.warm_start_RDP = true;
    end
    variables_for_matrices = [];
else
    % Warm/cold start
    if (solution.warm_start_RDP) % Warm start
        initial_condition.V_lower = solution.P_reach_lower;
        initial_condition.V_upper = solution.P_reach_upper;
        initial_condition.sigma = solution.sigma;
        disp('Warm starting value iteration')
    else % Cold start
        initial_condition.V_lower = zeros(user_param.Ns+1, 1);
        initial_condition.V_upper = zeros(user_param.Ns+1, 1);
        initial_condition.sigma = ones(user_param.Ns, 1);
        disp('Starting value iteration')
        solution.warm_start_RDP = true;
    end
    variables_for_matrices = [];
end


% Value Iteration
tic
if ( strcmp(original_sys.name, 'unicycle2d_ltlf' ) )
    if (strcmp(solver_opts.value_iteration_method,'IVI_single_goal') == 1)
        [P_reach_lower_PROD, P_reach_upper_PROD, sigma_PROD] = max_reach_probability_LTLf(user_param, initial_condition, variables_for_matrices, solver_opts);
    elseif (strcmp(solver_opts.value_iteration_method,'RVI_2layer') == 1)
        [P_reach_lower_PROD, P_reach_upper_PROD, sigma_PROD] = robust_max_reach_probability_2layer_ltlf(user_param, initial_condition, variables_for_matrices, solver_opts);
    end
elseif ( strcmp(original_sys.name, 'temp' ) || strcmp(original_sys.name, 'fm1' ) || strcmp(original_sys.name, 'fm2' ) || strcmp(original_sys.name, 'lr' ) )
    if ( strcmp(solver_opts.value_iteration_method,'IVI_single_goal') == 1)
        [P_reach_lower, P_reach_upper, sigma] = max_reach_probability_temp_fcn(user_param, initial_condition, solution, solver_opts);
        p_lower_X0 = find_p_lower_X0(P_reach_lower, state_discretization, original_sys, user_param, solver_opts);
        fprintf('p_lower_X0 = %f. \n\n', p_lower_X0)
    elseif ( strcmp(solver_opts.value_iteration_method,'RVI_2layer') == 1)
        [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_2layer_temp_fcn(user_param, initial_condition, solution, solver_opts);
    elseif (strcmp(solver_opts.value_iteration_method,'RVI_mdpst') == 1)
        [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_mdpst_temp(user_param, initial_condition, solution, solver_opts);
    elseif (strcmp(solver_opts.value_iteration_method,'RVI_just_gurobi_multi_imdp_cluster') == 1)
        [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_temp_multi_imdp(user_param, initial_condition, solution, solver_opts);
    elseif ( strcmp(solver_opts.value_iteration_method,'RVI_cme_umdp') == 1)
        [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_safety_cme_umdp_fcn(user_param, initial_condition, solution, solver_opts, state_discretization);
%         [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_safety_cme_umdp_nystrom_fcn(user_param, initial_condition, solution, solver_opts, state_discretization);
    end
else
    if (strcmp(solver_opts.value_iteration_method,'IVI_w_support') == 1)
        [P_reach_lower, P_reach_upper, sigma] = max_reach_probability(user_param, initial_condition, solution, solver_opts);
    elseif (strcmp(solver_opts.value_iteration_method,'IVI_single_goal') == 1)
        [P_reach_lower, P_reach_upper, sigma] = max_reach_probability(user_param, initial_condition, solution, solver_opts);
    elseif (strcmp(solver_opts.value_iteration_method,'RVI_2layer') == 1)
        [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_2layer(user_param, initial_condition, solution, solver_opts);
    elseif (strcmp(solver_opts.value_iteration_method,'RVI_linprog') == 1)
        [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_2layer_linprog(user_param, initial_condition, solution, state_discretization, solver_opts.tol_VI);
    elseif (strcmp(solver_opts.value_iteration_method,'RVI_gurobi_overlapping') == 1)
        [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_2layer_gurobi_lb(user_param, initial_condition, solution, state_discretization, solver_opts.tol_VI);
    elseif (strcmp(solver_opts.value_iteration_method,'RVI_just_gurobi_overlapping_skip_actions') == 1)
        [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_just_gurobi_skip_actions(user_param, initial_condition, solution, solver_opts);
    elseif (strcmp(solver_opts.value_iteration_method,'RVI_just_gurobi_overlapping_cluster') == 1)
        [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_just_gurobi_cluster(user_param, initial_condition, solution, solver_opts);
    elseif (strcmp(solver_opts.value_iteration_method,'RVI_just_gurobi_multi_imdp_cluster') == 1)
        [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_multi_imdp_just_gurobi_cluster(user_param, initial_condition, solution, solver_opts);
%         [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_multi_imdp_just_gurobi(user_param, initial_condition, solution, solver_opts);
%         [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_multi_imdp_just_gurobi_memory(user_param, initial_condition, solution, solver_opts);
    elseif (strcmp(solver_opts.value_iteration_method,'RVI_tailored_overlapping') == 1)
        [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_overlapping_tailored(user_param, initial_condition, solution, state_discretization, solver_opts.tol_VI);
    elseif (strcmp(solver_opts.value_iteration_method,'RVI_tailored_overlapping2') == 1)
        [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_overlapping_tailored2(user_param, initial_condition, solution, state_discretization, solver_opts.tol_VI);
    elseif (strcmp(solver_opts.value_iteration_method,'RVI_mdpist2_tailored') == 1)
        [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_mdpist2_tailored(user_param, initial_condition, solution, solver_opts);
    elseif (strcmp(solver_opts.value_iteration_method,'RVI_mdpst') == 1)
        [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_mdpst_tailored(user_param, initial_condition, solution, solver_opts);
%         [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_mdpst_just_gurobi(user_param, initial_condition, solution, solver_opts);
    elseif (strcmp(solver_opts.value_iteration_method,'RVI_mdpust_2layer_overlapping') == 1)
        [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_mdpust_2layer_overlapping(user_param, initial_condition, solution, solver_opts);
    elseif (strcmp(solver_opts.value_iteration_method,'RVI_mdpust_2layer_overlapping_cluster') == 1)
        [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_mdpust_2layer_overlapping_cluster(user_param, initial_condition, solution, solver_opts);
    elseif (strcmp(solver_opts.value_iteration_method,'RVI_imdpst_ss') == 1)
        [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_imdpst_ss(user_param, initial_condition, solution, solver_opts);
    elseif (strcmp(solver_opts.value_iteration_method,'RVI_imdpst_ss_coarse') == 1)
        [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_imdpst_ss_coarse(user_param, initial_condition, solution, solver_opts);
    elseif (strcmp(solver_opts.value_iteration_method,'RVI_imdpst'))
        [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_imdpst(user_param, initial_condition, solution, solver_opts);
    elseif (strcmp(solver_opts.value_iteration_method,'RVI_imdpst_partition_W'))
        [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_imdpst_partition_W(user_param, initial_condition, solution, solver_opts);
    elseif (strcmp(solver_opts.value_iteration_method,'RVI') == 1)
        % Robust Value Iteration
        [P_reach_lower, P_reach_upper, sigma] = Robust_max_reach_probability(user_param, initial_condition, variables_for_matrices, state_discretization, solver_opts.tol_VI);
    end
end

VI_time = toc/60;
if ( strcmp(original_sys.name, 'unicycle2d_ltlf' ) )
    solution.P_reach_lower_PROD = P_reach_lower_PROD;
    solution.P_reach_upper_PROD = P_reach_upper_PROD;
    solution.sigma_PROD = sigma_PROD;
    % Project solution on S
    P_reach_lower = zeros(user_param.Ns+1,1);
    P_reach_upper = zeros(user_param.Ns+1,1);
    for s = 1:user_param.Ns
        P_reach_lower(s) = solution.P_reach_lower_PROD(s, variables_for_matrices.z0_PROD(s));
        P_reach_upper(s) = solution.P_reach_upper_PROD(s, variables_for_matrices.z0_PROD(s));
        sigma(s) = sigma_PROD(s,  variables_for_matrices.z0_PROD(s));
    end
end
e_avg = mean(P_reach_upper(user_param.S_not_terminal) - P_reach_lower(user_param.S_not_terminal));

fprintf('original_sys.Actual value iteration time = %d min. \n', VI_time)

% for s = user_param.S_target
%     P_reach_lower(s) = 1;
%     P_reach_upper(s) = 1;
% end

if ( any( P_reach_lower < 0 - solver_opts.precision ) || any( P_reach_lower > 1 + solver_opts.precision ) || any( P_reach_upper < 0 - solver_opts.precision ) || any( P_reach_upper > 1 + solver_opts.precision ) )
    disp('ERROR: Values not in the range [0, 1]')
else
    disp('RDP terminated correctly')
end

solution.P_reach_lower = P_reach_lower;
solution.P_reach_upper = P_reach_upper;
solution.sigma = sigma;
solution.VI_time = VI_time;
solution.e_avg = e_avg;


end