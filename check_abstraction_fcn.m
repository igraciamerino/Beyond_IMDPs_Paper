function is_correct_yn = check_abstraction_fcn(solution, error_flags, solver_opts, user_param)

if ( strcmp( solver_opts.abstraction_choice, 'IMDP_naive') )
    is_correct_yn = check_abstraction_IMDP_naive_fcn(solution, error_flags, solver_opts, user_param);
elseif ( strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal') || strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs') || strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_l4dc2025') || strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs_l4dc2025') )
    if  ( solver_opts.group_terminal_states_together )
        is_correct_yn = check_abstraction_IMDP_fcn(solution, error_flags, solver_opts, user_param);
    end
elseif ( strcmp( solver_opts.abstraction_choice, '2layer') )
    is_correct_yn = check_abstraction_2layer_fcn(solution, error_flags, solver_opts, user_param);
elseif ( strcmp( solver_opts.abstraction_choice, '2layer_single_goal') || strcmp( solver_opts.abstraction_choice, '2layer_single_goal_and_obs') || strcmp( solver_opts.abstraction_choice, '2layer_single_goal_and_obs_l4dc2025') || strcmp( solver_opts.abstraction_choice, '2layer_single_goal_l4dc2025') )
    is_correct_yn = check_abstraction_2layer_single_goal_fcn(solution, error_flags, solver_opts, user_param);
elseif ( strcmp( solver_opts.abstraction_choice, '2layer_overlapping') )
    is_correct_yn = check_abstraction_2layer_overlapping_fcn(solution, error_flags, solver_opts, user_param);
elseif ( strcmp( solver_opts.abstraction_choice, 'multi_imdp') || strcmp( solver_opts.abstraction_choice, 'multi_imdp_single_goal_and_obs') )
    is_correct_yn = check_abstraction_multi_imdp_fcn(solution, error_flags, solver_opts, user_param);
elseif ( strcmp( solver_opts.abstraction_choice, 'mdpust_2layer_overlapping') )
    is_correct_yn = check_abstraction_mdpust_2layer_overlapping_fcn(solution, error_flags, solver_opts, user_param);
elseif ( strcmp( solver_opts.abstraction_choice, 'mdpist2') )
    is_correct_yn = check_abstraction_mdpist2_fcn(solution, error_flags, solver_opts, user_param);
elseif ( strcmp( solver_opts.abstraction_choice, 'mdpst') )
    is_correct_yn = check_abstraction_mdpst_fcn(solution, error_flags, solver_opts, user_param);
elseif ( strcmp( solver_opts.abstraction_choice, 'imdpst_ss') )
    is_correct_yn = check_abstraction_imdpst_ss_fcn(solution, error_flags, solver_opts, user_param);
elseif ( strcmp( solver_opts.abstraction_choice, 'imdpst_ss_coarse') )
    is_correct_yn = check_abstraction_imdpst_ss_coarse_fcn(solution, error_flags, solver_opts, user_param);
elseif ( strcmp( solver_opts.abstraction_choice, 'imdpst') )
    is_correct_yn = check_abstraction_imdpst_fcn(solution, error_flags, solver_opts, user_param);
elseif ( strcmp( solver_opts.abstraction_choice, 'imdpst_partition_W') )
    is_correct_yn = check_abstraction_imdpst_partition_W_fcn(solution, error_flags, solver_opts, user_param);
elseif ( strcmp( solver_opts.abstraction_choice, 'cme_umdp') )
    is_correct_yn = true;
end

end