function [user_param, state_discretization] = define_roi_unicycle3d_nl_friction(user_param, state_discretization, solver_opts, original_sys)

Nx = user_param.Nx{1};
Ny = user_param.Nx{2};
Nz = user_param.Nx{3};
Ns = user_param.Ns;
X_center = state_discretization.X_center{1};
Y_center = state_discretization.X_center{2};
Z_center = state_discretization.X_center{3};
Deltax = state_discretization.Deltax{1};
Deltay = state_discretization.Deltax{2};
Deltaz = state_discretization.Deltax{3};
a_X = original_sys.state_space_lower_left_corner;
b_X = original_sys.state_space_upper_right_corner;
plot_roi = user_param.plot_roi;
precision_roi = 0.001;

% Define states of the set to reach

a_S_target = a_X  + 0.99*Deltaz/2*[0; 0; 1]; % Position of the target set
Lx_S_target = user_param.width_S_target;
b_S_target = a_S_target + Lx_S_target*[1; 1; 0] + Deltaz/2*[0; 0; 1];

[S_target, ~] = intersections_simplified_unicycle3d_fcn(a_S_target, b_S_target, Deltax, Deltay, Deltaz, Nx, Ny, Nz, Ns, a_X, b_X, solver_opts.precision);
S_target = S_target( S_target ~= Ns + 1 );
S_target = S_target';
user_param.a_S_target = a_S_target;

user_param.move_S_target = 0; % Position of the target set

user_param.S_target = [];
% Cylindrical extension along the dimension of th
for k = 1:Nz
    user_param.S_target = [user_param.S_target S_target+((k-1)*(Nx*Ny))];
end
S_target = user_param.S_target;


if ( isfield(user_param, 'width_S_obstacle') )
    a_S_obs1 = ((b_X + a_X)/2).*[1, 1, 0] + precision_roi; % Position of the target set
    a_S_obs1 = a_S_obs1';
    Lx_S_obs1 = user_param.width_S_obstacle - precision_roi;
    Ly_S_obs1 = user_param.width_S_obstacle - precision_roi;
    b_S_obs1 = a_S_obs1 + [Lx_S_obs1; Ly_S_obs1; precision_roi];
    
    [S_obs1, ~] = intersections_simplified_unicycle3d_fcn(a_S_obs1, b_S_obs1, Deltax, Deltay, Deltaz, Nx, Ny, Nz, Ns, a_X, b_X, solver_opts.precision);
    S_obs1 = S_obs1( S_obs1 ~= Ns + 1 );
    S_obs1 = S_obs1';
    move_S_obstacle = 0;
    user_param.move_S_obstacle = move_S_obstacle;
    % Cylindrical extension along the dimension of th
    S_obstacle = [];
    for k = 1:Nz
        S_obstacle = [S_obstacle S_obs1+((k-1)*(Nx*Ny))];
    end
    user_param.a_S_obstacle = a_S_obs1;

else
    S_obstacle = [];
    user_param.a_S_obstacle = [];
end

S = 1:(Ns + 1);
S_sink = [S_obstacle, Ns + 1];
S_not_sink = S(~ismember(S,S_sink));
S_terminal = [S_sink S_target];
S_not_terminal = S(~ismember(S, S_terminal));

% Plot states

if (  plot_roi )
    figure
    scatter3(X_center(S_target), Y_center(S_target), Z_center(S_target),40,'green')
    hold on
    scatter3(X_center(S_obstacle), Y_center(S_obstacle), Z_center(S_obstacle),40,'black')
    xlim([0 1])
    ylim([0 1])
    zlim([0 1])
end

user_param.S_target = S_target;
user_param.n_target_blocks = 1;
user_param.S_target_blocks{1} = S_target;
user_param.S_obstacles = S_obstacle;
user_param.S_obstacle_blocks = {S_obstacle};
user_param.n_obstacle_blocks = 1;
user_param.S = S;
user_param.S_terminal = S_terminal;
user_param.S_not_terminal = S_not_terminal;

if ( solver_opts.group_terminal_states_together ) 
    [state_discretization, user_param] = make_single_goals_3d_fcn(state_discretization, user_param, original_sys);
end

if ( solver_opts.group_obs_states_together ) 
    [state_discretization, user_param] = make_single_obstacles_3d_fcn(state_discretization, user_param, original_sys);
end

end