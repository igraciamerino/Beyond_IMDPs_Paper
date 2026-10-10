function [user_param, state_discretization] = define_roi_pendulum(user_param, state_discretization, solver_opts, original_sys)

Nx = user_param.Nx;
Ny = user_param.Ny;
Ns = user_param.Ns;
X_center = state_discretization.X_center;
Y_center = state_discretization.Y_center;
plot_roi = user_param.plot_roi;
user_param.n_target_blocks = 2;

user_param.Ny_S_target = user_param.Nx_S_target; % Height of the tgt set (must be even number)

user_param.S_target1 = [];
% Non-wrapped portion of S_target
for j = 1:user_param.Ny_S_target
    for i = 1:user_param.Nx_S_target/2
        user_param.S_target1 = [user_param.S_target1 user_param.Nx*(user_param.Ny/2 - user_param.Nx_S_target/2) + i + (j-1)*user_param.Nx];
    end
end
user_param.S_target2 = user_param.S_target1+user_param.Nx-user_param.Nx_S_target/2; % Wrapped portion of S_target
user_param.S_target_blocks{1} = user_param.S_target1;
user_param.S_target_blocks{2} = user_param.S_target2;
user_param.S_target = [user_param.S_target1 user_param.S_target2];
user_param.S_obstacles = [];

user_param.S = 1:user_param.Ns + 1;
user_param.S_terminal = [user_param.S_target user_param.S_obstacles user_param.Ns + 1];
user_param.S_not_terminal = user_param.S(~ismember(user_param.S,user_param.S_terminal));

% Plot states

if (  plot_roi )
    figure(10)
    scatter(X_center(user_param.S_target), Y_center(user_param.S_target),40,'green')
    hold on
    scatter(X_center(user_param.S_obstacles), Y_center(user_param.S_obstacles),40,'black')
    xlim([0 1])
    ylim([0 1])
end

if ( solver_opts.group_terminal_states_together ) 
    [state_discretization, user_param] = account_for_single_goals_pendulum(state_discretization, user_param, original_sys);
end

end