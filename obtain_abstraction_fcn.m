function [solution, error_flags] = obtain_abstraction_fcn(user_param, state_discretization, disturbance_discretization, original_sys, solver_opts, samples, solution)

if ( strcmp(original_sys.name, 'unicycle2d' ) )

    if ( strcmp( solver_opts.abstraction_choice, 'IMDP_naive') )
        [solution, error_flags] = obtain_abstraction_IMDP_naive_single_goal_fcn(user_param, state_discretization, original_sys, solver_opts, samples);
    elseif ( strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal') )
        if ( solver_opts.group_terminal_states_together )
            [solution, error_flags] = obtain_abstraction_IMDP_single_goal_fcn(user_param, state_discretization, original_sys, solver_opts, samples);
        else
            [solution.P_lower, solution.P_upper, solution.Sp, solution.P_lower_coarse, solution.P_upper_coarse, solution.Qp, solution.children, solution.parents, solution.n_transitions, solution.beta, solution.epsilon_support, error_flags] = obtain_abstraction_IMDP_single_goal_fcn(user_param, state_discretization, original_sys, solver_opts, samples);
        end
    elseif ( strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_l4dc2025') )
        [solution, error_flags] = obtain_abstraction_IMDP_single_goal_l4dc2025_fcn(user_param, state_discretization, original_sys, solver_opts, samples);
    elseif ( strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs_l4dc2025') )
        [solution, error_flags] = obtain_abstraction_IMDP_single_goal_and_obs_l4dc2025_fcn(user_param, state_discretization, original_sys, solver_opts, samples);
    elseif ( strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs') )
%        if ( solver_opts.lipschitz_reach )
%            [solution, error_flags] = obtain_abstraction_IMDP_single_goal_and_obs_lipschitz_reach_fcn(user_param, state_discretization, original_sys, solver_opts, samples);
%        else
            [solution, error_flags] = obtain_abstraction_IMDP_single_goal_and_obs_fcn(user_param, state_discretization, original_sys, solver_opts, samples);
%        end
    elseif ( strcmp( solver_opts.abstraction_choice, '2layer') )
        [solution.P_lower, solution.P_upper, solution.Sp, solution.P_lower_coarse, solution.P_upper_coarse, solution.Qp, solution.children, solution.parents, solution.n_transitions, solution.beta, solution.epsilon_support, error_flags] = obtain_abstraction_2layer_fcn(user_param, state_discretization, original_sys, solver_opts, samples, solution);
    elseif ( strcmp( solver_opts.abstraction_choice, '2layer_single_goal') )
        [solution, error_flags] = obtain_abstraction_2layer_single_goal_fcn(user_param, state_discretization, original_sys, solver_opts, samples, solution);
    elseif ( strcmp( solver_opts.abstraction_choice, '2layer_single_goal_and_obs') )
        [solution, error_flags] = obtain_abstraction_2layer_single_goal_and_obs_fcn(user_param, state_discretization, original_sys, solver_opts, samples, solution);
    elseif ( strcmp( solver_opts.abstraction_choice, '2layer_single_goal_l4dc2025') )
        [solution, error_flags] = obtain_abstraction_2layer_single_goal_l4dc2025_fcn(user_param, state_discretization, original_sys, solver_opts, samples, solution);
    elseif ( strcmp( solver_opts.abstraction_choice, '2layer_overlapping') )
        [solution.P_lower, solution.P_upper, solution.Sp, solution.P_lower_coarse, solution.P_upper_coarse, solution.Qp, solution.children, solution.parents, solution.n_transitions, solution.beta, solution.epsilon_support, error_flags] = obtain_abstraction_2layer_overlapping_fcn(user_param, state_discretization, original_sys, solver_opts, samples);
    elseif ( strcmp( solver_opts.abstraction_choice, 'multi_imdp' ) )
        [solution, error_flags] = obtain_abstraction_multi_imdp_fcn(user_param, state_discretization, original_sys, solver_opts, samples);
    elseif ( strcmp( solver_opts.abstraction_choice, 'multi_imdp_single_goal_and_obs' ) )
        [solution, error_flags] = obtain_abstraction_multi_imdp_single_goal_and_obs_fcn(user_param, state_discretization, original_sys, solver_opts, samples, solution);
    elseif ( strcmp( solver_opts.abstraction_choice, 'mdpist2') )
        [solution, error_flags] = obtain_abstraction_mdpist2_fcn(user_param, state_discretization, disturbance_discretization, original_sys, solver_opts, samples, solution);
    elseif ( strcmp( solver_opts.abstraction_choice, 'mdpst') )
        [solution, error_flags] = obtain_abstraction_mdpst_fcn(user_param, state_discretization, disturbance_discretization, original_sys, solver_opts, samples, solution);
    elseif ( strcmp( solver_opts.abstraction_choice, 'mdpust_2layer_overlapping') )
        [solution.P_lower_c, solution.P_upper_c, solution.Qp, solution.P_lower_c_coarse, solution.P_upper_c_coarse, solution.Qp_coarse, solution.children, solution.parents, solution.n_transitions, solution.beta, solution.epsilon_support, error_flags] = obtain_abstraction_mdpust_2layer_overlapping_fcn(user_param, state_discretization, disturbance_discretization, original_sys, solver_opts, samples);
    elseif ( strcmp( solver_opts.abstraction_choice, 'imdpst_ss') )
        [solution, error_flags] = obtain_abstraction_imdpst_ss_fcn(user_param, state_discretization, original_sys, solver_opts, samples);
    elseif ( strcmp( solver_opts.abstraction_choice, 'imdpst_ss_coarse') )
        [solution, error_flags] = obtain_abstraction_imdpst_ss_coarse_fcn(user_param, state_discretization, original_sys, solver_opts, samples);
    elseif ( strcmp( solver_opts.abstraction_choice, 'imdpst') )
        [solution, error_flags] = obtain_abstraction_imdpst_fcn(user_param, state_discretization, original_sys, solver_opts, samples);
    elseif ( strcmp( solver_opts.abstraction_choice, 'imdpst_partition_W') )
        [solution, error_flags] = obtain_abstraction_imdpst_partition_W_fcn(user_param, state_discretization, disturbance_discretization, original_sys, solver_opts, samples, solution);
    elseif ( strcmp( solver_opts.abstraction_choice, 'multi_imdp_dd') )
        [solution, error_flags] = obtain_abstraction_multi_imdp_dd_fcn(user_param, state_discretization, original_sys, solver_opts, samples);
end

elseif ( strcmp(original_sys.name, 'unicycle3d_nl_friction' ) )
    
    if ( strcmp( solver_opts.abstraction_choice, 'IMDP_naive') )
        [solution, error_flags] = obtain_abstraction_IMDP_naive_single_goal_unicycle3d_fcn(user_param, state_discretization, original_sys, solver_opts, samples);
    elseif ( strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs') || strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal') )
        [solution, error_flags] = obtain_abstraction_IMDP_single_goal_unicycle3d_fcn(user_param, state_discretization, original_sys, solver_opts, samples, solution);
%         if ( solver_opts.group_terminal_states_together )
%             [solution, error_flags] = obtain_abstraction_IMDP_single_goal_unicycle3d_fcn(user_param, state_discretization, original_sys, solver_opts, samples, solution);
%         else
%             disp('No functionality for this one...')
%         end
    elseif ( strcmp( solver_opts.abstraction_choice, '2layer') )
        [solution.P_lower, solution.P_upper, solution.Sp, solution.P_lower_coarse, solution.P_upper_coarse, solution.Qp, solution.children, solution.parents, solution.n_transitions, solution.beta, solution.epsilon_support, error_flags] = obtain_abstraction_unicycle3d_2layer_fcn(user_param, state_discretization, original_sys, solver_opts, samples);
    elseif ( strcmp( solver_opts.abstraction_choice, '2layer_single_goal_and_obs') )
        [solution, error_flags] = obtain_abstraction_unicycle3d_2layer_single_goal_and_obs_fcn(user_param, state_discretization, original_sys, solver_opts, samples, solution);
    elseif ( strcmp( solver_opts.abstraction_choice, '2layer_overlapping') )
        [solution.P_lower, solution.P_upper, solution.Sp, solution.P_lower_coarse, solution.P_upper_coarse, solution.Qp, solution.children, solution.parents, solution.n_transitions, solution.beta, solution.epsilon_support, error_flags] = obtain_abstraction_unicycle3d_2layer_overlapping_fcn(user_param, state_discretization, original_sys, solver_opts, samples, solution);
    elseif ( strcmp( solver_opts.abstraction_choice, 'mdpist2') )
%         [solution.P_lower_c, solution.P_upper_c, solution.Qp, solution.P_lower_c_coarse, solution.P_upper_c_coarse, solution.Qp_coarse, solution.children, solution.parents, solution.n_transitions, solution.beta, solution.epsilon_support, error_flags] = obtain_abstraction_mdpist2_fcn(user_param, state_discretization, disturbance_discretization, original_sys, solver_opts, samples);
    elseif ( strcmp( solver_opts.abstraction_choice, 'mdpust_2layer_overlapping') )
        [solution.P_lower_c, solution.P_upper_c, solution.Qp, solution.P_lower_c_coarse, solution.P_upper_c_coarse, solution.Qp_coarse, solution.children, solution.parents, solution.n_transitions, solution.beta, solution.epsilon_support, error_flags] = obtain_abstraction_unicycle3d_mdpust_2layer_overlapping_fcn(user_param, state_discretization, disturbance_discretization, original_sys, solver_opts, samples, solution);
    elseif ( strcmp( solver_opts.abstraction_choice, 'mdpst') )
        [solution, error_flags] = obtain_abstraction_mdpst_unicycle3d_fcn(user_param, state_discretization, disturbance_discretization, original_sys, solver_opts, samples, solution);
    elseif ( strcmp( solver_opts.abstraction_choice, 'multi_imdp_single_goal_and_obs' ) )
        [solution, error_flags] = obtain_abstraction_multi_imdp_single_goal_and_obs_uni3d_fcn(user_param, state_discretization, original_sys, solver_opts, samples, solution);
%         [solution, error_flags] = obtain_abstraction_multi_imdp_uni3d_fcn(user_param, state_discretization, original_sys, solver_opts, samples, solution);
    end
elseif ( strcmp(original_sys.name, 'unicycle4d_nl_friction' ) )
    
    if ( strcmp( solver_opts.abstraction_choice, 'mdpst') )
        [solution, error_flags] = obtain_abstraction_mdpst_unicycle4d_fcn(user_param, state_discretization, disturbance_discretization, original_sys, solver_opts, samples, solution);
    end
elseif ( strcmp(original_sys.name, 'pendulum' ) )
    
    if ( strcmp( solver_opts.abstraction_choice, 'IMDP_naive') )
        [solution, error_flags] = obtain_abstraction_IMDP_naive_single_goal_pendulum_fcn(user_param, state_discretization, original_sys, solver_opts, samples);
    elseif ( strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal') )
        [solution, error_flags] = obtain_abstraction_IMDP_single_goal_pendulum_fcn(user_param, state_discretization, original_sys, solver_opts, samples, solution);
    elseif ( strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_l4dc2025') )
        [solution, error_flags] = obtain_abstraction_IMDP_single_goal_pendulum_l4dc2025_fcn(user_param, state_discretization, original_sys, solver_opts, samples, solution);
    elseif ( strcmp( solver_opts.abstraction_choice, '2layer') )
        [solution.P_lower, solution.P_upper, solution.Sp, solution.P_lower_coarse, solution.P_upper_coarse, solution.Qp, solution.children, solution.parents, solution.n_transitions, solution.beta, solution.epsilon_support, error_flags] = obtain_abstraction_pendulum_2layer_fcn(user_param, state_discretization, original_sys, solver_opts, samples, solution);
    elseif ( strcmp( solver_opts.abstraction_choice, '2layer_single_goal_l4dc2025') )
        [solution, error_flags] = obtain_abstraction_pendulum_2layer_single_goal_l4dc2025_fcn(user_param, state_discretization, original_sys, solver_opts, samples, solution);
    elseif ( strcmp( solver_opts.abstraction_choice, '2layer_overlapping') )
        %[solution.P_lower, solution.P_upper, solution.Sp, solution.P_lower_coarse, solution.P_upper_coarse, solution.Qp, solution.children, solution.parents, solution.n_transitions, solution.beta, solution.epsilon_support, error_flags] = obtain_abstraction_unicycle3d_2layer_overlapping_fcn(user_param, state_discretization, original_sys, solver_opts, samples, solution);
    elseif ( strcmp( solver_opts.abstraction_choice, 'mdpist2') )
%         [solution.P_lower_c, solution.P_upper_c, solution.Qp, solution.P_lower_c_coarse, solution.P_upper_c_coarse, solution.Qp_coarse, solution.children, solution.parents, solution.n_transitions, solution.beta, solution.epsilon_support, error_flags] = obtain_abstraction_mdpist2_fcn(user_param, state_discretization, disturbance_discretization, original_sys, solver_opts, samples);
    elseif ( strcmp( solver_opts.abstraction_choice, 'mdpust_2layer_overlapping') )
        %[solution.P_lower_c, solution.P_upper_c, solution.Qp, solution.P_lower_c_coarse, solution.P_upper_c_coarse, solution.Qp_coarse, solution.children, solution.parents, solution.n_transitions, solution.beta, solution.epsilon_support, error_flags] = obtain_abstraction_unicycle3d_mdpust_2layer_overlapping_fcn(user_param, state_discretization, disturbance_discretization, original_sys, solver_opts, samples, solution);
    end

elseif ( strcmp(original_sys.name, 'mult_noise' ) )

    if ( strcmp( solver_opts.abstraction_choice, 'IMDP_naive') )
        %[solution, error_flags] = obtain_abstraction_IMDP_naive_single_goal_mult_noise_fcn(user_param, state_discretization, original_sys, solver_opts, samples);
    elseif ( strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal') )
        if ( solver_opts.group_terminal_states_together )
            %[solution, error_flags] = obtain_abstraction_IMDP_single_goal_mult_noise_fcn(user_param, state_discretization, original_sys, solver_opts, samples);
        else
            disp('No functionality for this one...')
        end
    elseif ( strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs_l4dc2025') )
        [solution, error_flags] = obtain_abstraction_IMDP_single_goal_obs_mult_noise_l4dc2025_fcn(user_param, state_discretization, original_sys, solver_opts, samples);
    elseif ( strcmp( solver_opts.abstraction_choice, '2layer') )
        [solution.P_lower, solution.P_upper, solution.Sp, solution.P_lower_coarse, solution.P_upper_coarse, solution.Qp, solution.children, solution.parents, solution.n_transitions, solution.beta, solution.epsilon_support, error_flags] = obtain_abstraction_mult_noise_2layer_fcn(user_param, state_discretization, original_sys, solver_opts, samples, solution);
    elseif ( strcmp( solver_opts.abstraction_choice, '2layer_single_goal_and_obs_l4dc2025' ) )
        [solution, error_flags] = obtain_abstraction_mult_noise_2layer_single_goal_and_obs_fcn(user_param, state_discretization, original_sys, solver_opts, samples, solution);
    elseif ( strcmp( solver_opts.abstraction_choice, '2layer_overlapping') )
        [solution.P_lower, solution.P_upper, solution.Sp, solution.P_lower_coarse, solution.P_upper_coarse, solution.Qp, solution.children, solution.parents, solution.n_transitions, solution.beta, solution.epsilon_support, error_flags] = obtain_abstraction_2layer_overlapping_fcn(user_param, state_discretization, original_sys, solver_opts, samples);
    elseif ( strcmp( solver_opts.abstraction_choice, 'mdpist2') )
        [solution, error_flags] = obtain_abstraction_mdpist2_fcn(user_param, state_discretization, disturbance_discretization, original_sys, solver_opts, samples);
    elseif ( strcmp( solver_opts.abstraction_choice, 'mdpust_2layer_overlapping') )
        [solution.P_lower_c, solution.P_upper_c, solution.Qp, solution.P_lower_c_coarse, solution.P_upper_c_coarse, solution.Qp_coarse, solution.children, solution.parents, solution.n_transitions, solution.beta, solution.epsilon_support, error_flags] = obtain_abstraction_mdpust_2layer_overlapping_fcn(user_param, state_discretization, disturbance_discretization, original_sys, solver_opts, samples);
    elseif ( strcmp( solver_opts.abstraction_choice, 'imdpst_ss') )
        [solution, error_flags] = obtain_abstraction_imdpst_ss_fcn(user_param, state_discretization, original_sys, solver_opts, samples);
    elseif ( strcmp( solver_opts.abstraction_choice, 'imdpst_ss_coarse') )
        [solution, error_flags] = obtain_abstraction_imdpst_ss_coarse_fcn(user_param, state_discretization, original_sys, solver_opts, samples);
    end
elseif ( strcmp(original_sys.name, 'unicycle2d_ltlf') )
    if ( strcmp( solver_opts.abstraction_choice, 'IMDP_naive') )
%         [solution, error_flags] = obtain_abstraction_IMDP_single_goal_mult_noise_fcn(user_param, state_discretization, original_sys, solver_opts, samples);
    elseif ( strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs_l4dc2025') )
        [solution, error_flags] = obtain_abstraction_IMDP_single_goal_obs_2d_ltlf_l4dc2025_fcn(user_param, state_discretization, original_sys, solver_opts, samples);
    elseif ( strcmp( solver_opts.abstraction_choice, '2layer_single_goal_and_obs_l4dc2025' ) )
       [solution, error_flags] = obtain_abstraction_2layer_single_goal_and_obs_2d_ltlf_fcn(user_param, state_discretization, original_sys, solver_opts, samples, solution);
    end
elseif ( strcmp(original_sys.name, 'temp' ) )
    if ( strcmp( solver_opts.abstraction_choice, 'mdpst') )
        %[solution, error_flags] = obtain_abstraction_mdpst_temp4d_fcn(user_param, state_discretization, disturbance_discretization, original_sys, solver_opts, samples, solution);
        [solution, error_flags] = obtain_abstraction_mdpst_temp4d_fcn(user_param, state_discretization, disturbance_discretization, original_sys, solver_opts, samples, solution);
    elseif ( strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal') )
%         [solution, error_flags] = obtain_abstraction_IMDP_temp4d_fcn(user_param, state_discretization, disturbance_discretization, original_sys, solver_opts, samples, solution);
        [solution, error_flags] = obtain_abstraction_IMDP_temp_n_dim_fcn(user_param, state_discretization, disturbance_discretization, original_sys, solver_opts, samples, solution);
    elseif ( strcmp( solver_opts.abstraction_choice, '2layer_single_goal') )
        [solution, error_flags] = obtain_abstraction_temp_n_dim_2layer_fcn(user_param, state_discretization, disturbance_discretization, original_sys, solver_opts, samples, solution);
    elseif ( strcmp( solver_opts.abstraction_choice, 'multi_imdp_single_goal_and_obs' ) )
        [solution, error_flags] = obtain_abstraction_multi_imdp_temp_n_dim_fcn(user_param, state_discretization, disturbance_discretization, original_sys, solver_opts, samples, solution);
    end
elseif ( strcmp(original_sys.name, 'fm1' ) )
    if ( strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs') )
        [solution, error_flags] = obtain_abstraction_IMDP_fm1_fcn(user_param, state_discretization, disturbance_discretization, original_sys, solver_opts, samples, solution);
    elseif ( strcmp( solver_opts.abstraction_choice, 'cme_umdp') )
        [solution, error_flags] = obtain_abstraction_cme_rmdp_fm1_fcn(user_param, state_discretization, disturbance_discretization, original_sys, solver_opts, samples, solution);
    end
elseif ( strcmp(original_sys.name, 'fm2' ) )
    if ( strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs') )
        [solution, error_flags] = obtain_abstraction_IMDP_fm2_fcn(user_param, state_discretization, disturbance_discretization, original_sys, solver_opts, samples, solution);
    end
elseif ( strcmp(original_sys.name, 'lr' ) )
    if ( strcmp( solver_opts.abstraction_choice, 'cme_umdp') )
        [solution, error_flags] = obtain_abstraction_cme_rmdp_lr_fcn(user_param, state_discretization, disturbance_discretization, original_sys, solver_opts, samples, solution);
    end
end

solution.abstraction_time = toc/60;
fprintf('abstraction time = %d \n\n', solution.abstraction_time)

% Obtain memory complexity
ss = 0;
if ( strcmp(solver_opts.abstraction_choice, 'mdpst') )
    for s = user_param.S_not_terminal
        for a = 1:user_param.Na
            for c = 1:user_param.Nc + 1
                ss = ss + 2*length(solution.Qp{s,a}.children{c});
            end
        end
    end
    ss = ss + 8*length( solution.P_c );
elseif ( strcmp(solver_opts.abstraction_choice, 'IMDP_w_support_single_goal') || strcmp(solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs') )
    for s = user_param.S_not_terminal
        for a = 1:user_param.Na
            ss = ss + 2*length(solution.Sp{s,a}) + 2*8*length(solution.Sp{s,a});
        end
    end
elseif ( strcmp(solver_opts.abstraction_choice, '2layer_single_goal') || strcmp(solver_opts.abstraction_choice, '2layer_single_goal_and_obs') )
    for s = user_param.S_not_terminal
        for a = 1:user_param.Na
            ss = ss + 2*length(solution.Sp{s,a}) + 2*8*length(solution.Sp{s,a}) + 2*length(solution.Qp{s,a}) + 2*8*length(solution.Qp{s,a});
        end
    end
    for q = 1:user_param.Nq + 1
        ss = ss + 2*length( solution.children{q} );
    end
    ss = ss + 2*length( solution.parents );
elseif ( strcmp(solver_opts.abstraction_choice, 'multi_imdp_single_goal_and_obs') )
    for s = user_param.S_not_terminal
        for a = 1:user_param.Na
            ss = ss + 2*length(solution.Sp{s,a}) + 2*8*length(solution.Sp{s,a}) + 2*8*length(solution.P_lower_coarse{s,a});
            for c_i = 1:length( solution.Qp{s,a}.children_q )
                ss = ss + 2*length(solution.Qp{s,a}.children_q{c_i});
            end
        end
    end
end
solution.memory_complexity = ss/1e9;
fprintf('Memory usage = %f \n\n', ss/1e9)

end


    