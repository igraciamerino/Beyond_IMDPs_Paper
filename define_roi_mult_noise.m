function [user_param, state_discretization] = define_roi_mult_noise(user_param, state_discretization, solver_opts, original_sys)

Nx = user_param.Nx;
Ny = user_param.Ny;
Ns = user_param.Ns;
X_center = state_discretization.X_center;
Y_center = state_discretization.Y_center;
aX = original_sys.state_space_lower_left_corner;
bX = original_sys.state_space_upper_right_corner;
plot_roi = user_param.plot_roi;
user_param.n_target_blocks = 3;

user_param.Ny_S_target = user_param.Nx_S_target; % Height of the tgt set (must be even number)

% Define states of the abstraction

% Define S_target
user_param.move_S_target1 = 0; % Position of the target set

user_param.S_block = [];
for k = 1:user_param.Nx_S_target
    user_param.S_block = [user_param.S_block linspace((k-1)*user_param.Nx+1,(k-1)*user_param.Nx+user_param.Nx_S_target,user_param.Nx_S_target)];
end

% Define S_target 2
user_param.move_S_target2 = user_param.Nx/2; % Position of the target set
% Define S_target 3
user_param.move_S_target3 = user_param.Nx*user_param.Ny/2 + user_param.Nx/2; % Position of the target set

user_param.S_target_blocks{1} = user_param.S_block + user_param.move_S_target1;
user_param.S_target_blocks{2} = user_param.S_block + user_param.move_S_target2;
user_param.S_target_blocks{3} = user_param.S_block + user_param.move_S_target3;
user_param.S_target = [user_param.S_target_blocks{1}, user_param.S_target_blocks{2}, user_param.S_target_blocks{3}];

% Define obstacles

% Define S_obstacle 1
user_param.move_S_obstacle1 = user_param.Nx*user_param.Ny/4 + user_param.Nx/4; % Position of obstacle1
% Define S_target 2
user_param.move_S_obstacle2 = user_param.Nx*user_param.Ny*3/4 + user_param.Nx/4; % Position of obstacle2
% Define S_target 3
user_param.move_S_obstacle3 = user_param.Nx*user_param.Ny*3/4 + user_param.Nx*3/4; % Position ofobstacle3

user_param.S_obstacle1 = user_param.S_block + user_param.move_S_obstacle1;
user_param.S_obstacle2 = user_param.S_block + user_param.move_S_obstacle2;
user_param.S_obstacle3 = user_param.S_block + user_param.move_S_obstacle3;

user_param.S_obstacle_blocks = {user_param.S_obstacle1, user_param.S_obstacle2, user_param.S_obstacle3};
user_param.S_obstacle = [user_param.S_obstacle1, user_param.S_obstacle2, user_param.S_obstacle3];
user_param.n_obstacle_blocks = 3;

user_param.S = 1:user_param.Ns + 1;
user_param.S_terminal = [user_param.S_target user_param.S_obstacle user_param.Ns + 1];
user_param.S_not_terminal = user_param.S(~ismember(user_param.S,user_param.S_terminal));
% Plot states

if (  plot_roi )
    figure(10)
    scatter(X_center(user_param.S_target), Y_center(user_param.S_target),40,'green')
    hold on
    scatter(X_center(user_param.S_obstacle), Y_center(user_param.S_obstacle),40,'black')
    xlim([aX(1) bX(1)])
    ylim([aX(2) bX(2)])
end


if ( solver_opts.group_terminal_states_together ) 
    [state_discretization, user_param] = make_single_goals_fcn(state_discretization, user_param, original_sys);
end

if ( solver_opts.group_obs_states_together ) 
    [state_discretization, user_param] = make_single_obstacles_fcn(state_discretization, user_param, original_sys);
end

end