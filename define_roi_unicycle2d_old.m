function [user_param, state_discretization] = define_roi_unicycle2d_old(user_param, state_discretization, solver_opts, original_sys)

Nx = user_param.Nx{1};
Ny = user_param.Nx{2};
Deltax = state_discretization.Deltax{1};
Deltay = state_discretization.Deltax{2};
a_X = original_sys.state_space_lower_left_corner;
b_X = original_sys.state_space_upper_right_corner;
Ns = user_param.Ns;
X_center = state_discretization.X_center{1};
Y_center = state_discretization.X_center{2};
plot_roi = user_param.plot_roi;

move_S_target = [0; 0]; % Position of the target set
width_S_target = user_param.width_S_target;

a_S_target = a_X;
b_S_target = a_S_target + width_S_target;
[S_target, ~] = intersections_simplified_fcn(a_S_target, b_S_target, Deltax, Deltax, Nx, Nx, Ns, a_X, b_X, solver_opts.precision);
% Define states of the set to reach

S_target = S_target( S_target ~= Ns + 1 );
S_target = S_target';


% Define obstacle 1

Nx_S_obstacle1 = ceil(Nx/10); % Width of obstacle 1
Ny_S_obstacle1 = ceil(Nx/5); % Height of obstacle 1
move_S_obstacle1 = ceil(0.5*Nx); % Position of obstacle 1



% Define obstacle 2

Nx_S_obstacle2 = ceil(Nx/5); % Width of obstacle 2
Ny_S_obstacle2 = ceil(Nx/10); % Height of obstacle 2
move_S_obstacle2 = ceil(Nx/2)*Nx + ceil(0.5*Nx-ceil(Nx/5)); % Position of obstacle 2

% Define obstacle 3

Nx_S_obstacle3 = ceil(Nx/10); % Width of obstacle 3
Ny_S_obstacle3 = ceil(Nx/10); % Height of obstacle 3
move_S_obstacle3 = ceil( Nx*(Ny-Ny_S_obstacle3) )+ceil(Nx/5); % Position of obstacle 3

% Define obstacle 4

Nx_S_obstacle4 = ceil(Nx/10); % Width of obstacle 4
Ny_S_obstacle4 = ceil(Nx/10); % Height of obstacle 4
move_S_obstacle4 = ceil(0.8*Nx)*Ny + ceil(0.7*Ny); % Position of obstacle 4

% Make obstacle absorbing

if ( exist('Nx_S_obstacle1', 'var') )
    [S_obstacle1] = define_obstacle(Nx_S_obstacle1,Ny_S_obstacle1,move_S_obstacle1,Nx, Ns);
    S_obstacle = S_obstacle1;
end

if ( exist('Nx_S_obstacle2', 'var') )
    [S_obstacle2] = define_obstacle(Nx_S_obstacle2,Ny_S_obstacle2,move_S_obstacle2,Nx, Ns);
    S_obstacle = [S_obstacle, S_obstacle2];
end

if ( exist('Nx_S_obstacle3', 'var') )
    [S_obstacle3] = define_obstacle(Nx_S_obstacle3,Ny_S_obstacle3,move_S_obstacle3,Nx, Ns);
    S_obstacle = [S_obstacle, S_obstacle3];
end

if ( exist('Nx_S_obstacle4', 'var') )
    [S_obstacle4] = define_obstacle(Nx_S_obstacle4,Ny_S_obstacle4,move_S_obstacle4,Nx, Ns);
    S_obstacle = [S_obstacle, S_obstacle4];
end

% COMMENT THIS LINE!!
% S_obstacle = [];

S = 1:(Ns + 1);
S_sink = [S_obstacle Ns + 1];
S_not_sink = S(~ismember(S,S_sink));
S_terminal = [S_sink S_target];
S_not_terminal = S(~ismember(S, S_terminal));

% Plot states

if (  plot_roi )
    figure(10)
    scatter(X_center(S_target), Y_center(S_target),40,'green')
    hold on
    scatter(X_center(S_obstacle), Y_center(S_obstacle),40,'black')
    xlim([0 1])
    ylim([0 1])
end

user_param.S_target = S_target;
user_param.n_target_blocks = 1;
user_param.S_target_blocks{1} = user_param.S_target;
user_param.S_obstacle = S_obstacle;
user_param.S_obstacle_blocks = {S_obstacle1, S_obstacle2, S_obstacle3, S_obstacle4};
user_param.n_obstacle_blocks = 4;
user_param.S = S;
user_param.S_terminal = S_terminal;
user_param.S_not_terminal = S_not_terminal;

if ( solver_opts.group_terminal_states_together ) 
    [state_discretization, user_param] = make_single_goals_fcn(state_discretization, user_param, original_sys);
end

if ( solver_opts.group_obs_states_together ) 
    [state_discretization, user_param] = make_single_obstacles_fcn(state_discretization, user_param, original_sys);
end

end