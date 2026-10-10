function [samples, disturbance_discretization] = compute_weight_cells_fcn(samples, state_discretization, disturbance_discretization, original_sys, user_param, solver_opts)

if ( original_sys.data_driven_yn )
    if ( strcmp( solver_opts.abstraction_choice, 'mdpist2') || strcmp( solver_opts.abstraction_choice, 'mdpust_2layer') || strcmp( solver_opts.abstraction_choice, 'mdpust_2layer_overlapping') || strcmp( solver_opts.abstraction_choice, 'imdpst_partition_W') )
        [samples, disturbance_discretization] = compute_weight_emp_cells_uniform_fcn(samples, disturbance_discretization, original_sys, user_param);
    elseif ( strcmp( solver_opts.abstraction_choice, 'mdpist2_voronoi') )
        [samples, disturbance_discretization] = compute_weight_emp_cells_voronoi_fcn(samples, disturbance_discretization, original_sys, user_param);
    else
        disturbance_discretization = [];
    end

else
    if ( strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal' ) || strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs' ) || strcmp( solver_opts.abstraction_choice, 'mdpst') || strcmp(solver_opts.abstraction_choice, 'multi_imdp') || strcmp(solver_opts.abstraction_choice, 'multi_imdp_single_goal_and_obs') || strcmp( solver_opts.abstraction_choice, 'mdpist2') || strcmp( solver_opts.abstraction_choice, 'mdpust_2layer') || strcmp( solver_opts.abstraction_choice, 'mdpust_2layer_overlapping') || strcmp( solver_opts.abstraction_choice, '2layer_overlapping') || strcmp(solver_opts.abstraction_choice, '2layer_single_goal') || strcmp(solver_opts.abstraction_choice, '2layer_single_goal_and_obs')  )
        [samples, disturbance_discretization] = compute_weight_cells_gaussian_fcn(samples, disturbance_discretization, original_sys, user_param);
    end
end

% Plot clustered samples together with workspace discretization
n_samples_to_plot = 1000;
n_samples_to_plot = min(n_samples_to_plot, size(samples.Xi_hat, 2) );
figure(1)
if ( original_sys.n == 2 )
    x = [state_discretization.X_center{1}(1), state_discretization.X_center{2}(1)]';
    plot_samples_fcn(samples.Xi_hat(:, 1:n_samples_to_plot), x, original_sys.Act(:,1), original_sys, user_param);
elseif ( original_sys.n == 3 )
    x = [state_discretization.X_center{1}(1), state_discretization.X_center{2}(1),state_discretization.X_center{3}(1)]';
    plot_samples_fcn(samples.Xi_hat(:, 1:n_samples_to_plot), x, original_sys.Act(:,1), original_sys, user_param);
elseif ( original_sys.n == 4 )
    x = [state_discretization.X_center{1}(1), state_discretization.X_center{2}(1), state_discretization.X_center{3}(1), state_discretization.X_center{4}(1)]';
    plot_samples_fcn(samples.Xi_hat(:, 1:n_samples_to_plot), x, original_sys.Act(:,1), original_sys, user_param);
end
if ( ~isempty( disturbance_discretization ) )
    figure(2)
    if ( original_sys.d < 3 )
        plot(samples.Xi_hat(1, 1:n_samples_to_plot), samples.Xi_hat(2, 1:n_samples_to_plot), '.g', 'MarkerSize', 10)
        hold on
        plot(disturbance_discretization.Xd_center{1}, disturbance_discretization.Xd_center{2}, 'bo')
        samples.Xi_hat_c = zeros(original_sys.d, user_param.Nc + 1);
        for c_ind = 1:user_param.Nc
            samples.Xi_hat_c(:, c_ind) = [disturbance_discretization.Xd_center{1}(c_ind); disturbance_discretization.Xd_center{2}(c_ind)];
        end
    elseif ( original_sys.d == 3 )
        plot3(samples.Xi_hat(1, 1:n_samples_to_plot), samples.Xi_hat(2, 1:n_samples_to_plot), samples.Xi_hat(3, 1:n_samples_to_plot), '.g', 'MarkerSize', 10)
        hold on
        plot3(disturbance_discretization.Xd_center{1}, disturbance_discretization.Xd_center{2}, disturbance_discretization.Xd_center{3}, 'bo')
        for c_ind = 1:user_param.Nc
            samples.Xi_hat_c(:, c_ind) = [disturbance_discretization.Xd_center{1}(c_ind); disturbance_discretization.Xd_center{2}(c_ind); disturbance_discretization.Xd_center{3}(c_ind)];
        end
    elseif ( original_sys.d == 4 )
%         plot3(samples.Xi_hat(1, 1:n_samples_to_plot), samples.Xi_hat(2, 1:n_samples_to_plot), samples.Xi_hat(3, 1:n_samples_to_plot), '.g', 'MarkerSize', 10)
%         hold on
%         plot3(disturbance_discretization.Xd_center{1}, disturbance_discretization.Xd_center{2}, disturbance_discretization.Xd_center{3}, 'bo')
        for c_ind = 1:user_param.Nc
            samples.Xi_hat_c(:, c_ind) = [disturbance_discretization.Xd_center{1}(c_ind); disturbance_discretization.Xd_center{2}(c_ind); disturbance_discretization.Xd_center{3}(c_ind)];
        end
    end
end


end