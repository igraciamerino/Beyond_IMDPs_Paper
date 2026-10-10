function [user_param, state_discretization, solver_opts] = define_roi(user_param, original_sys, state_discretization, solver_opts)

if ( strcmp(solver_opts.abstraction_choice, 'IMDP_w_support_single_goal') || strcmp(solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_l4dc2025') || strcmp(solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs_l4dc2025')|| strcmp(solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs') || strcmp(solver_opts.abstraction_choice, '2layer_single_goal') || strcmp(solver_opts.abstraction_choice, '2layer_single_goal_l4dc2025') || strcmp(solver_opts.abstraction_choice, '2layer_single_goal_and_obs') || strcmp(solver_opts.abstraction_choice, '2layer_single_goal_and_obs_l4dc2025') || strcmp( solver_opts.abstraction_choice, 'multi_imdp_single_goal_and_obs' )  || strcmp( solver_opts.abstraction_choice, 'multi_imdp_dd' ) )
    disp('Grouping goal states into a single one...')
    solver_opts.group_terminal_states_together = true;
else
    solver_opts.group_terminal_states_together = false;
end

if ( strcmp(solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs') ||strcmp(solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs_l4dc2025') || strcmp(solver_opts.abstraction_choice, '2layer_single_goal_and_obs_l4dc2025') || strcmp(solver_opts.abstraction_choice, '2layer_single_goal_and_obs') || strcmp(solver_opts.abstraction_choice, '2layer_single_goal_and_obs_l4dc2025') || strcmp( solver_opts.abstraction_choice, 'multi_imdp_single_goal_and_obs' )  || strcmp( solver_opts.abstraction_choice, 'multi_imdp_dd' ) )
    disp('Grouping obstacle states into blocks...')
    solver_opts.group_obs_states_together = true;
else
    solver_opts.group_obs_states_together = false;
end

if ( strcmp(original_sys.name, 'unicycle2d' ) )
    [user_param, state_discretization] = define_roi_unicycle2d(user_param, state_discretization, solver_opts, original_sys);
elseif ( strcmp(original_sys.name, 'unicycle3d_nl_friction' ) )
    [user_param, state_discretization] = define_roi_unicycle3d_nl_friction(user_param, state_discretization, solver_opts, original_sys);
elseif ( strcmp(original_sys.name, 'unicycle4d_nl_friction' ) )
    [user_param, state_discretization] = define_roi_unicycle4d_nl_friction(user_param, state_discretization, solver_opts, original_sys);
elseif ( strcmp(original_sys.name, 'pendulum' ) )
    [user_param, state_discretization] = define_roi_pendulum(user_param, state_discretization, solver_opts, original_sys);
elseif ( strcmp(original_sys.name, 'mult_noise' ) )
    [user_param, state_discretization] = define_roi_mult_noise(user_param, state_discretization, solver_opts, original_sys);
elseif ( strcmp(original_sys.name, 'unicycle2d_ltlf' ) )
    [user_param, state_discretization] = define_roi_unicycle2d_ltlf(user_param, state_discretization, solver_opts, original_sys);
elseif ( strcmp(original_sys.name, 'temp') || strcmp(original_sys.name, 'fm1') || strcmp(original_sys.name, 'fm2') )
    [user_param, state_discretization] = define_roi_temp4d_fcn(user_param, state_discretization, solver_opts, original_sys);
elseif ( strcmp(original_sys.name, 'temp') || strcmp(original_sys.name, 'lr') )
    [user_param, state_discretization] = define_roi_lr_fcn(user_param, state_discretization, solver_opts, original_sys);
end

end