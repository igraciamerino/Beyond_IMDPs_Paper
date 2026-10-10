function [disturbance_discretization, user_param] = discretize_disturbance_space(original_sys, user_param, solver_opts)

if (solver_opts.partition_W_yn)
    if ( strcmp( solver_opts.abstraction_choice, 'IMDP_naive') || strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal') || strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs') || strcmp( solver_opts.abstraction_choice, 'mdpst') || strcmp( solver_opts.abstraction_choice, 'mdpist2') || strcmp( solver_opts.abstraction_choice, 'mdpust_2layer') || strcmp( solver_opts.abstraction_choice, 'mdpust_2layer_overlapping') || strcmp( solver_opts.abstraction_choice, '2layer_single_goal') || strcmp( solver_opts.abstraction_choice, '2layer_single_goal_and_obs') || strcmp( solver_opts.abstraction_choice, 'multi_imdp_single_goal_and_obs' )  || strcmp( solver_opts.abstraction_choice, 'multi_imdp' ) || strcmp(solver_opts.abstraction_choice, 'imdpst_partition_W') )
        [disturbance_discretization, user_param] = discretize_disturbance_space_uniform_d_dim(original_sys, user_param);
    else
        disturbance_discretization = [];
    end
else
    disturbance_discretization = [];
end
return;

if ( original_sys.d == 2 )
    if ( strcmp( solver_opts.abstraction_choice, 'mdpist2') || ~original_sys.data_driven_yn )
        [disturbance_discretization, user_param] = discretize_disturbance_space_uniform_2layer(original_sys, user_param);
        user_param.Nc = user_param.Nxd*user_param.Nyd;
    elseif ( strcmp( solver_opts.abstraction_choice, 'mdpist2_voronoi') )
        [disturbance_discretization, user_param] = discretize_disturbance_space_voronoi(original_sys, user_param, solver_opts);
    elseif ( strcmp( solver_opts.abstraction_choice, 'mdpust_2layer') )
        [disturbance_discretization, user_param] = discretize_disturbance_space_uniform_2layer(original_sys, user_param);
    elseif ( strcmp( solver_opts.abstraction_choice, 'mdpust_2layer_overlapping') )
        [disturbance_discretization, user_param] = discretize_disturbance_space_uniform_2layer_overlapping(original_sys, user_param);
    else
        disturbance_discretization = [];
    end
elseif ( original_sys.d == 3 )
    if ( strcmp( solver_opts.abstraction_choice, 'mdpist2') || ~original_sys.data_driven_yn )
        [disturbance_discretization, user_param] = discretize_3d_disturbance_space_uniform(original_sys, user_param);
    end
elseif ( original_sys.d == 4 )
    if ( strcmp( solver_opts.abstraction_choice, 'mdpist2') || ~original_sys.data_driven_yn )
        [disturbance_discretization, user_param] = discretize_4d_disturbance_space_uniform(original_sys, user_param);
    end
end

end