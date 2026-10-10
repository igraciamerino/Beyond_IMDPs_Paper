function [] = plot_discretization(state_discretization, original_sys, plot_grid, solver_opts, user_param)

if ( strcmp( solver_opts.abstraction_choice, 'IMDP_naive') || strcmp( solver_opts.abstraction_choice, 'mdpst') || strcmp( solver_opts.abstraction_choice, 'imdpst') ||strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal') ||strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs') ||strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs_l4dc2025') ||strcmp( solver_opts.abstraction_choice, '2layer') ||strcmp( solver_opts.abstraction_choice, '2layer_single_goal_l4dc2025') ||strcmp( solver_opts.abstraction_choice, '2layer_single_goal_and_obs_l4dc2025') || strcmp( solver_opts.abstraction_choice, 'imdpst_ss_coarse') || strcmp( solver_opts.abstraction_choice, 'multi_imdp_single_goal_and_obs' )  || strcmp( solver_opts.abstraction_choice, 'multi_imdp' ) || strcmp( solver_opts.abstraction_choice, 'imdpst_partition_W') || strcmp(solver_opts.abstraction_choice, 'cme_umdp') )
    plot_discretization_IMDP_n_dim_fcn(state_discretization, user_param, plot_grid, original_sys);
elseif ( strcmp( solver_opts.abstraction_choice, '2layer_single_goal')  || strcmp( solver_opts.abstraction_choice, '2layer_single_goal_and_obs')   || strcmp(solver_opts.abstraction_choice, 'multi_imdp_dd'))
    plot_discretization_IMDP_n_dim_fcn(state_discretization, user_param, plot_grid, original_sys);
end
return;

if ( original_sys.n < 3 )
    if ( strcmp( solver_opts.abstraction_choice, 'IMDP_naive') || strcmp( solver_opts.abstraction_choice, 'mdpst') || strcmp( solver_opts.abstraction_choice, 'imdpst') ||strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal') ||strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs') ||strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs_l4dc2025') ||strcmp( solver_opts.abstraction_choice, '2layer') ||strcmp( solver_opts.abstraction_choice, '2layer_single_goal_l4dc2025') ||strcmp( solver_opts.abstraction_choice, '2layer_single_goal_and_obs_l4dc2025') || strcmp( solver_opts.abstraction_choice, 'imdpst_ss_coarse') )
        plot_discretization_2layer(state_discretization, plot_grid);
    elseif ( strcmp( solver_opts.abstraction_choice, '2layer_overlapping') )
        plot_discretization_overlapping(state_discretization, original_sys, plot_grid);
    end
elseif ( strcmp(original_sys.name, 'temp') )
    if ( strcmp( solver_opts.abstraction_choice, 'IMDP_naive') || strcmp( solver_opts.abstraction_choice, 'mdpst') || strcmp( solver_opts.abstraction_choice, 'imdpst') ||strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal') ||strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs') ||strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs_l4dc2025') ||strcmp( solver_opts.abstraction_choice, '2layer') ||strcmp( solver_opts.abstraction_choice, '2layer_single_goal_l4dc2025') ||strcmp( solver_opts.abstraction_choice, '2layer_single_goal_and_obs_l4dc2025') || strcmp( solver_opts.abstraction_choice, 'imdpst_ss_coarse') )
        plot_discretization_temp_fcn(state_discretization, user_param, plot_grid);
    end
elseif ( strcmp(original_sys.name, 'unicycle3d_nl_friction') )
    if ( strcmp( solver_opts.abstraction_choice, 'IMDP_naive') || strcmp( solver_opts.abstraction_choice, 'mdpst') || strcmp( solver_opts.abstraction_choice, 'imdpst') ||strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal') ||strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs') ||strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs_l4dc2025') ||strcmp( solver_opts.abstraction_choice, '2layer') ||strcmp( solver_opts.abstraction_choice, '2layer_single_goal_l4dc2025') ||strcmp( solver_opts.abstraction_choice, '2layer_single_goal_and_obs_l4dc2025') || strcmp( solver_opts.abstraction_choice, 'imdpst_ss_coarse') )
        plot_discretization_3d(state_discretization, user_param, plot_grid);
    end
else
    if ( strcmp( solver_opts.abstraction_choice, 'IMDP_naive') || strcmp( solver_opts.abstraction_choice, 'mdpst') || strcmp( solver_opts.abstraction_choice, 'imdpst') ||strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal') ||strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs') ||strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs_l4dc2025') ||strcmp( solver_opts.abstraction_choice, '2layer') ||strcmp( solver_opts.abstraction_choice, '2layer_single_goal_l4dc2025') ||strcmp( solver_opts.abstraction_choice, '2layer_single_goal_and_obs_l4dc2025') || strcmp( solver_opts.abstraction_choice, 'imdpst_ss_coarse') )
        plot_discretization_4d(state_discretization, plot_grid);
    end
end

end
