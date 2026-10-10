function [samples, user_param, disturbance_discretization] = generate_and_cluster_samples_fcn(original_sys, state_discretization, disturbance_discretization, user_param, plotsamples, solver_opts, samples)

% Generates samples
if (isempty(samples))
    if ( ~strcmp(solver_opts.abstraction_choice, 'cme_umdp') )
        if ( strcmp( original_sys.Pw_class, 'gaussian') )
            [samples] = generate_samples_gaussian(original_sys, user_param, plotsamples);
        elseif ( strcmp( original_sys.Pw_class, 'heavy_tail') )
            [samples] = generate_samples_heavy_tail(original_sys, user_param, plotsamples);
            % elseif ( strcmp( original_sys.name, 'mult_noise') )
            %     [samples] = generate_samples_mult_noise(original_sys, user_param, plotsamples);
        end
    else
        if ( strcmp( original_sys.Pw_class, 'gaussian') )
            [samples] = generate_samples_cme_gaussian(original_sys, user_param, plotsamples);
        end
    end
end

% Cluster samples

if ( ~strcmp(solver_opts.abstraction_choice, 'cme_umdp') )

    if ( ~solver_opts.cluster_samples )
        % Find weights of each cell
        [samples, disturbance_discretization] = compute_weight_cells_fcn(samples, state_discretization, disturbance_discretization, original_sys, user_param, solver_opts);
    else
        [~,ia] = uniquetol(samples.Xi_hat', user_param.tol_cluster_samples, 'ByRows', true, ...
            'OutputAllIndices', true);

        user_param.Nc = length(ia);
        samples.Xi_hat_c = zeros(original_sys.d,user_param.Nc);
        samples.weight = zeros(1,user_param.Nc);
        samples.size_cluster = length(ia);
        for i = 1:user_param.Nc
            samples.Xi_hat_c(:,i) = mean(samples.Xi_hat(:,ia{i}),2);
            d_i = zeros(length(ia{i}),1);
            for j = 1:length(ia{i})
                d_i(j) = norm(samples.Xi_hat(:,ia{i}(j)) - samples.Xi_hat_c(:,i), "inf");%^user_param.p;
            end
            samples.size_cluster(i) = max(d_i);
            samples.weight(i) = length(ia{i});
        end
        samples.weight_norm = samples.weight/sum(samples.weight);
        user_param.N = size( samples.Xi_hat, 2 );

        if (solver_opts.cluster_samples)
            fprintf('N_cluster = %d. \n', user_param.Nc)
            if ( original_sys.d == 2 )
                figure
                hold on
                plot3(samples.Xi_hat_c(1,:), samples.Xi_hat_c(2,:), samples.size_cluster/sum(samples.size_cluster), '.g', 'MarkerSize', 10)
            elseif ( original_sys.d == 1 )
                figure
                hold on
                plot(samples.Xi_hat_c, samples.weight_norm, '.g', 'MarkerSize', 10)
            end
        else
            samples.Xi_hat_c = samples.Xi_hat;
            user_param.Nc = user_param.N;
        end

        % Plot clustered samples together with workspace discretization
        if (plotsamples == 1)
            figure(1)
            plot_samples_fcn(samples.Xi_hat_c, zeros(original_sys.n, 1), zeros(size(original_sys.Act,1),1), original_sys, user_param);
        end
    end
end

end

