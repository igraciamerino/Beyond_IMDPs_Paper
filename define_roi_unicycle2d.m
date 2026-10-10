function [user_param, state_discretization] = define_roi_unicycle2d(user_param, state_discretization, solver_opts, original_sys)

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
precision_roi = 0.001;
% Define states of the set to reach

a_S_target = a_X; % Position of the target set
Lx_S_target = user_param.width_S_target;
b_S_target = a_S_target + Lx_S_target;

[S_target, ~] = intersections_simplified_fcn(a_S_target, b_S_target, Deltax, Deltay, Nx, Ny, Ns, a_X, b_X, solver_opts.precision);
S_target = S_target( S_target ~= Ns + 1 );
S_target = S_target';


% Define obstacle 1

a_S_obs1 = [0.55; 0] + precision_roi; % Position of the target set
Lx_S_obs1 = 0.1 - precision_roi;
Ly_S_obs1 = 0.2 - precision_roi;
b_S_obs1 = a_S_obs1 + [Lx_S_obs1; Ly_S_obs1];

% Define obstacle 2

a_S_obs2 = [0.3; 0.5] + precision_roi; % Position of the target set
Lx_S_obs2 = 0.2 - precision_roi;
Ly_S_obs2 = 0.1 - precision_roi;
b_S_obs2 = a_S_obs2 + [Lx_S_obs2; Ly_S_obs2];

% Define obstacle 3

a_S_obs3 = [0.2; 0.9] + precision_roi; % Position of the target set
Lx_S_obs3 = 0.1 - precision_roi;
Ly_S_obs3 = 0.2 - precision_roi;
b_S_obs3 = a_S_obs3 + [Lx_S_obs3; Ly_S_obs3];

% Define obstacle 4

a_S_obs4 = [0.7; 0.8] + precision_roi; % Position of the target set
Lx_S_obs4 = 0.1 - precision_roi;
Ly_S_obs4 = 0.1 - precision_roi;
b_S_obs4 = a_S_obs4 + [Lx_S_obs4; Ly_S_obs4];

% Make obstacle absorbing

S_obstacle = [];
if ( exist('a_S_obs1', 'var') )
    [S_obstacle1, ~] = intersections_simplified_fcn(a_S_obs1, b_S_obs1, Deltax, Deltay, Nx, Ny, Ns, a_X, b_X, solver_opts.precision);
    S_obstacle1 = S_obstacle1( S_obstacle1 ~= Ns + 1 )';
    S_obstacle = [S_obstacle, S_obstacle1];
end

if ( exist('a_S_obs2', 'var') )
    [S_obstacle2, ~] = intersections_simplified_fcn(a_S_obs2, b_S_obs2, Deltax, Deltay, Nx, Ny, Ns, a_X, b_X, solver_opts.precision);
    S_obstacle2 = S_obstacle2( S_obstacle2 ~= Ns + 1 )';
    S_obstacle = [S_obstacle, S_obstacle2];
end

if ( exist('a_S_obs3', 'var') )
    [S_obstacle3, ~] = intersections_simplified_fcn(a_S_obs3, b_S_obs3, Deltax, Deltay, Nx, Ny, Ns, a_X, b_X, solver_opts.precision);
    S_obstacle3 = S_obstacle3( S_obstacle3 ~= Ns + 1 )';
    S_obstacle = [S_obstacle, S_obstacle3];
end

if ( exist('a_S_obs4', 'var') )
    [S_obstacle4, ~] = intersections_simplified_fcn(a_S_obs4, b_S_obs4, Deltax, Deltay, Nx, Ny, Ns, a_X, b_X, solver_opts.precision);
    S_obstacle4 = S_obstacle4( S_obstacle4 ~= Ns + 1 )';
    S_obstacle = [S_obstacle, S_obstacle4];
end

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
user_param.precision_roi = precision_roi;

if ( solver_opts.group_terminal_states_together ) 
    [state_discretization, user_param] = make_single_goals_fcn(state_discretization, user_param, original_sys);
end

if ( solver_opts.group_obs_states_together ) 
    [state_discretization, user_param] = make_single_obstacles_fcn(state_discretization, user_param, original_sys);
end

end