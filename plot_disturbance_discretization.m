function [] = plot_disturbance_discretization(disturbance_discretization, user_param, original_sys, plot_grid, solver_opts)

%if ( strcmp(original_sys.name, 'temp') )
    if ( strcmp( solver_opts.abstraction_choice, 'IMDP_naive') || strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal') || strcmp( solver_opts.abstraction_choice, 'mdpst') || strcmp( solver_opts.abstraction_choice, 'mdpist2') || strcmp( solver_opts.abstraction_choice, 'mdpust_2layer') || strcmp( solver_opts.abstraction_choice, 'mdpust_2layer_overlapping') || strcmp( solver_opts.abstraction_choice, 'multi_imdp' ) || strcmp( solver_opts.abstraction_choice, '2layer_single_goal_and_obs') )
        plot_disturbance_discretization_uniform_d_dim(disturbance_discretization, user_param, plot_grid, original_sys);
    end
    return;
%end

if ( original_sys.d < 3 )
    if ( strcmp( solver_opts.abstraction_choice, 'mdpst') ||strcmp( solver_opts.abstraction_choice, 'mdpist2') || strcmp( solver_opts.abstraction_choice, 'mdpust_2layer_overlapping') )
        plot_disturbance_discretization_uniform(disturbance_discretization, plot_grid);
    elseif ( strcmp( solver_opts.abstraction_choice, 'mdpist2_voronoi') )
        plot_disturbance_discretization_voronoi(disturbance_discretization, plot_grid);
    end
end

end