function [state_discretization, user_param] = discretize_state_space(original_sys, user_param, solver_opts)

%if ( strcmp(original_sys.name, 'temp') )
    if ( strcmp( solver_opts.abstraction_choice, 'IMDP_naive') || strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal') || strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs') || strcmp( solver_opts.abstraction_choice, 'mdpst') || strcmp( solver_opts.abstraction_choice, 'mdpist2') || strcmp( solver_opts.abstraction_choice, 'mdpust_2layer') || strcmp( solver_opts.abstraction_choice, 'mdpust_2layer_overlapping') || strcmp( solver_opts.abstraction_choice, 'multi_imdp_single_goal_and_obs' ) || strcmp( solver_opts.abstraction_choice, 'multi_imdp' ) || strcmp(solver_opts.abstraction_choice, 'imdpst_partition_W') || strcmp(solver_opts.abstraction_choice, 'cme_umdp') || strcmp(solver_opts.abstraction_choice, 'me_umdp') )
        [state_discretization, user_param] = discretize_n_dim_state_space_IMDP(original_sys, user_param);
    elseif ( strcmp( solver_opts.abstraction_choice, '2layer') || strcmp( solver_opts.abstraction_choice, '2layer_single_goal_and_obs') || strcmp( solver_opts.abstraction_choice, '2layer_single_goal') || strcmp( solver_opts.abstraction_choice, '2layer_single_goal_l4dc2025') || strcmp( solver_opts.abstraction_choice, '2layer_single_goal_and_obs_l4dc2025' )  || strcmp( solver_opts.abstraction_choice, 'imdpst') )
        [state_discretization, user_param] = discretize_state_space_2layer_n_dim_fcn(original_sys, user_param);
    elseif ( strcmp(solver_opts.abstraction_choice, 'multi_imdp_dd') )
        [state_discretization, user_param] = discretize_state_space_overlapping_cluster(original_sys, user_param);
    end
    return;
%end

if ( original_sys.n == 2)

    if ( strcmp( solver_opts.abstraction_choice, 'IMDP_naive') || strcmp( solver_opts.abstraction_choice, 'IMDP_w_support') || strcmp( solver_opts.abstraction_choice, 'mdpst') || strcmp( solver_opts.abstraction_choice, 'imdpst') || strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal') || strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_l4dc2025') || strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs_l4dc2025') || strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs') || strcmp( solver_opts.abstraction_choice, 'mdpist2') || strcmp( solver_opts.abstraction_choice, 'mdpust_2layer') || strcmp( solver_opts.abstraction_choice, 'mdpust_2layer_overlapping') || strcmp( solver_opts.abstraction_choice, 'imdpst_ss') || strcmp( solver_opts.abstraction_choice, 'multi_imdp' ) || strcmp( solver_opts.abstraction_choice, 'multi_imdp_single_goal_and_obs' ) )
        [state_discretization, user_param] = discretize_state_space_IMDP(original_sys, user_param);
    elseif ( strcmp( solver_opts.abstraction_choice, '2layer') || strcmp( solver_opts.abstraction_choice, '2layer_single_goal_and_obs') || strcmp( solver_opts.abstraction_choice, '2layer_single_goal') || strcmp( solver_opts.abstraction_choice, '2layer_single_goal_l4dc2025') || strcmp( solver_opts.abstraction_choice, '2layer_single_goal_and_obs_l4dc2025' )  )
        [state_discretization, user_param] = discretize_state_space_2layer(original_sys, user_param);
    elseif ( strcmp( solver_opts.abstraction_choice, '2layer_overlapping') )
        [state_discretization, user_param] = discretize_state_space_overlapping_cluster(original_sys, user_param);
    elseif ( strcmp( solver_opts.abstraction_choice, 'imdpst_ss_coarse') )
        [state_discretization, user_param] = discretize_state_space_2layer(original_sys, user_param);
    end

elseif ( original_sys.n == 3)

    if ( strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs') || strcmp( solver_opts.abstraction_choice, 'IMDP_naive') || strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal') || strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs_l4dc2025') || strcmp( solver_opts.abstraction_choice, 'mdpst') || strcmp( solver_opts.abstraction_choice, 'mdpist2') || strcmp( solver_opts.abstraction_choice, 'mdpust_2layer') || strcmp( solver_opts.abstraction_choice, 'mdpust_2layer_overlapping') )
        [state_discretization, user_param] = discretize_3d_state_space_IMDP(original_sys, user_param);
    elseif ( strcmp( solver_opts.abstraction_choice, '2layer') || strcmp( solver_opts.abstraction_choice, '2layer_single_goal_and_obs') )
        [state_discretization, user_param] = discretize_state_space_unicycle3d_2layer(original_sys, user_param);
    elseif ( strcmp( solver_opts.abstraction_choice, '2layer_overlapping') )
        if (strcmp(original_sys.name, 'unicycle3d_nl_friction'))
            [state_discretization, user_param] = discretize_unicycle3d_state_space_overlapping_cluster(original_sys, user_param);
        else
            [state_discretization, user_param] = discretize_3d_state_space_overlapping_cluster(original_sys, user_param);
        end
    end

elseif ( strcmp(original_sys.name, 'unicycle4d_nl_friction') || strcmp(original_sys.name, 'temp') )
    if ( strcmp( solver_opts.abstraction_choice, 'IMDP_naive') || strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal') || strcmp( solver_opts.abstraction_choice, 'mdpst') || strcmp( solver_opts.abstraction_choice, 'mdpist2') || strcmp( solver_opts.abstraction_choice, 'mdpust_2layer') || strcmp( solver_opts.abstraction_choice, 'mdpust_2layer_overlapping') )
        [state_discretization, user_param] = discretize_4d_state_space_IMDP(original_sys, user_param);
    end
end

end

