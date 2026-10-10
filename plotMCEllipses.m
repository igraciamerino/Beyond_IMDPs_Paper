function P_reach_emp = plotMCEllipses(state_discretization, user_param, solver_opts, original_sys, Xi_hat, q0, nStdDevs, solution, variables_for_matrices, Act)

X_MC = cell(user_param.MC_runs, 1);
x0 = [state_discretization.X_center(q0)
    state_discretization.Y_center(q0)];
counter = 0;
for i = 1:user_param.MC_runs
    % Solve a Monte Carlo run starting from x0
    [x_MC, MC_flag] = MC_run_unicycle2D_nl_friction(x0, state_discretization, user_param,solution.sigma_PROD, original_sys, Xi_hat, Act, variables_for_matrices);
    if (strcmp(MC_flag,'reach') == 1)
        counter = counter + 1;
    end
    if ( size(x_MC, 2) < user_param.MC_timeout )
        lastState = x_MC(:,end);
        for k = 1:user_param.MC_timeout-size(x_MC, 2)
            x_MC = [x_MC lastState];
        end
    end
    X_MC{i} = x_MC;
end
P_reach_emp = counter/user_param.MC_runs;

mu = zeros(original_sys.n, user_param.MC_timeout);
for k = 1:user_param.MC_timeout
    data = zeros(original_sys.n, user_param.MC_runs);
    for i = 1:user_param.MC_runs
        data(:,i) = X_MC{i}(:,k);
    end
    mu(:,k) = mean(data,2)';
    plotStdEllipse(data, nStdDevs);
end

if (solver_opts.MC_plot)
    hold on;
    plot(mu(1,:), mu(2,:), 'b', 'Linewidth', 2)
end

end