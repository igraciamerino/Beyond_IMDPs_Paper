function [user_param, state_discretization] = define_roi_unicycle2d_ltlf(user_param, state_discretization, solver_opts, original_sys)

Nx = user_param.Nx;
Ny = user_param.Ny;
Ns = user_param.Ns;
X_center = state_discretization.X_center;
Y_center = state_discretization.Y_center;
aX = original_sys.state_space_lower_left_corner;
bX = original_sys.state_space_upper_right_corner;
plot_roi = user_param.plot_roi;
user_param.n_target_blocks = 1;
user_param.n_obstacle_blocks = 1;

% Define S_target
user_param.Ny_S_target = user_param.Nx_S_target; % Height of the tgt set (must be even number)
user_param.move_S_target1 = 0; % Position of the target set

user_param.S_block = [];
for k = 1:user_param.Nx_S_target
    user_param.S_block = [user_param.S_block linspace((k-1)*user_param.Nx+1,(k-1)*user_param.Nx+user_param.Nx_S_target,user_param.Nx_S_target)];
end

user_param.Nx_S_target = ceil(user_param.Nx/5); % Width of target
user_param.Ny_S_target = ceil(user_param.Ny/5); % Height of target
user_param.move_S_target = 0; % Position of target

% Define water

user_param.S_water = [];
aux_S_water = [];

% Define water 1
user_param.Nx_S_water1 = ceil(user_param.Nx*2/5); % Width of water 1
user_param.Ny_S_water1 = ceil(user_param.Nx*2/5); % Height of water 1
user_param.move_S_water1 = user_param.Nx*(user_param.Ny - user_param.Ny_S_water1); % Position of water 1

% Define carpet

user_param.S_carpet = [];
aux_S_carpet = [];

% Define carpet 1
user_param.Nx_S_carpet1 = ceil(user_param.Nx/5); % Width of carpet 1
user_param.Ny_S_carpet1 = ceil(user_param.Nx/5); % Height of carpet 1
user_param.move_S_carpet1 = user_param.Nx - user_param.Nx_S_carpet1; % Position of carpet 1

% Define obstacles

user_param.S_obstacle = [];
aux_S_obstacles = [];

% Define obstacle 1
user_param.Nx_S_obstacle1 = floor(user_param.Nx*9/24); % Width of obstacle 1
user_param.Ny_S_obstacle1 = ceil(user_param.Nx/10); % Height of obstacle 1
user_param.move_S_obstacle1 = user_param.move_S_water1 + user_param.Nx_S_water1; % Position of obstacle 1

% Define obstacle 2
user_param.Nx_S_obstacle2 = ceil(user_param.Nx/7)-floor(1*user_param.Nx/60); % Width of obstacle 2
user_param.Ny_S_obstacle2 = ceil(user_param.Nx*2.5/10); % Height of obstacle 2
user_param.move_S_obstacle2 =  ceil(user_param.Nx/5*2)+floor(2*user_param.Nx/60); % Position of obstacle 2

% Define obstacle 3
user_param.Nx_S_obstacle3 = ceil(user_param.Nx/10); % Width of obstacle 3
user_param.Ny_S_obstacle3 = 4; % Height of obstacle 3
user_param.move_S_obstacle3 = user_param.move_S_obstacle1 + user_param.Nx*user_param.Ny_S_obstacle1 + ceil(user_param.Nx_S_obstacle1/2); % Position of obstacle 3

% 
% % Define obstacle 4
% user_param.Nx_S_obstacle4 = 2; % Width of obstacle 4
% user_param.Ny_S_obstacle4 = 2; % Height of obstacle 4
% user_param.move_S_obstacle4 = user_param.Nx*(user_param.Ny-2*user_param.Ny_S_obstacle4)+0.7*user_param.Ny; % Position of obstacle 4

[user_param.S_target] = define_obstacle(user_param.Nx_S_target,user_param.Ny_S_target,user_param.move_S_target,user_param.Nx,user_param.Ns);
user_param.S_target_blocks{1} = user_param.S_target;

if (isfield(user_param,'Nx_S_obstacle1'))
    [user_param.S_obstacle1] = define_obstacle(user_param.Nx_S_obstacle1,user_param.Ny_S_obstacle1,user_param.move_S_obstacle1,user_param.Nx,user_param.Ns);
    user_param.S_obstacle = user_param.S_obstacle1;
    user_param.S_obstacle_blocks{1} = user_param.S_obstacle1;
end

if (isfield(user_param,'Nx_S_obstacle2'))
    [user_param.S_obstacle2] = define_obstacle(user_param.Nx_S_obstacle2,user_param.Ny_S_obstacle2,user_param.move_S_obstacle2,user_param.Nx,user_param.Ns);
    user_param.S_obstacle = [user_param.S_obstacle, user_param.S_obstacle2];
    user_param.S_obstacle_blocks{2} = user_param.S_obstacle2;
end

if (isfield(user_param,'Nx_S_obstacle3'))
    [user_param.S_obstacle3] = define_obstacle(user_param.Nx_S_obstacle3,user_param.Ny_S_obstacle3,user_param.move_S_obstacle3,user_param.Nx,user_param.Ns);
    user_param.S_obstacle = [user_param.S_obstacle, user_param.S_obstacle3];
    user_param.S_obstacle_blocks{3} = user_param.S_obstacle3;
end

if (isfield(user_param,'Nx_S_obstacle4'))
    [user_param.S_obstacle4] = define_obstacle(user_param.Nx_S_obstacle4,user_param.Ny_S_obstacle4,user_param.move_S_obstacle4,user_param.Nx,user_param.Ns);
    user_param.S_obstacle = [user_param.S_obstacle, user_param.S_obstacle4];
    user_param.S_obstacle_blocks{4} = user_param.S_obstacle4;
end


if (isfield(user_param,'Nx_S_water1'))
    [user_param.S_water1] = define_obstacle(user_param.Nx_S_water1,user_param.Ny_S_water1,user_param.move_S_water1,user_param.Nx,user_param.Ns);
    user_param.S_water = user_param.S_water1;
end

if (isfield(user_param,'Nx_S_carpet1'))
    [user_param.S_carpet1] = define_obstacle(user_param.Nx_S_carpet1,user_param.Ny_S_carpet1,user_param.move_S_carpet1,user_param.Nx,user_param.Ns);
    user_param.S_carpet = user_param.S_carpet1;
end

% Define S_target 2
user_param.move_S_target2 = user_param.Nx/2; % Position of the target set
% Define S_target 3
user_param.move_S_target3 = user_param.Nx*user_param.Ny/2 + user_param.Nx/2; % Position of the target set

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