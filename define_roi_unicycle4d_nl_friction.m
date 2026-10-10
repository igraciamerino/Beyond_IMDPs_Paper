function [user_param, state_discretization] = define_roi_unicycle4d_nl_friction(user_param, state_discretization, solver_opts, original_sys)

Nx = user_param.Nx;
Ns = user_param.Ns;
X_center = state_discretization.X_center;
Nx_S_target = user_param.Nx_S_target;
plot_roi = user_param.plot_roi;

% Define states of the set to reach

user_param.move_S_target = 0; % Position of the target set

user_param.S_target = [];
aux_S_target = [];
for j = 1:Nx_S_target
    aux_S_target = [aux_S_target linspace((j-1)*Nx{1}+1,(j-1)*Nx{1}+Nx_S_target,Nx_S_target)];
end
% Cylindrical extension along the dimension of th
aux_S_target2 = [];
for k = 1:Nx{3}
    aux_S_target2 = [aux_S_target2 aux_S_target+((k-1)*(Nx{1}*Nx{2}))];
end
% Cylindrical extension along the dimension of v
for k = 1:Nx{4}
    user_param.S_target = [user_param.S_target aux_S_target2+((k-1)*(Nx{1}*Nx{2}*Nx{3}))];
end
S_target = user_param.S_target + user_param.move_S_target;

% Make obstacles absorbing (not yet)

if ( isfield(user_param, 'Nx_S_obstacle') )
    Nx_S_obstacle = user_param.Nx_S_obstacle;
    user_param.Ny_S_obstacle = Nx_S_obstacle;
    % Define obstacles
    Ny_S_obstacle = user_param.Ny_S_obstacle;
    move_S_obstacle = Nx{1}*floor(Nx{2}/2) + floor(Nx{1}/2)-1; % Position of obstacle_center
    user_param.move_S_obstacle = move_S_obstacle;
    % Cylindrical extension along the dimension of th
    aux_S_obstacle2 = [];
    aux_S_obstacles = define_obstacle(Nx_S_obstacle,Ny_S_obstacle,move_S_obstacle,Nx{1}, Ns);
    for k = 1:Nx{3}
        aux_S_obstacle2 = [aux_S_obstacle2 aux_S_obstacles+((k-1)*(Nx{1}*Nx{2}) + 1)];
    end
    % Cylindrical extension along the dimension of v
    S_obstacle = [];
    for k = 1:Nx{4}
        S_obstacle = [S_obstacle aux_S_obstacle2+((k-1)*(Nx{1}*Nx{2}*Nx{3}) + 1)];
    end

else
    S_obstacle = [];
end

S = 1:(Ns + 1);
S_sink = [S_obstacle, Ns + 1];
S_not_sink = S(~ismember(S,S_sink));
S_terminal = [S_sink S_target];
S_not_terminal = S(~ismember(S, S_terminal));

% Plot states

if (  plot_roi )
    figure
    scatter3(X_center{1}(S_target),X_center{2}(S_target),X_center{3}(S_target),40,'green')
    hold on
    scatter3(X_center{1}(S_obstacle),X_center{2}(S_obstacle),X_center{3}(S_obstacle),40,'black')
    xlim([0 1])
    ylim([0 1])
    zlim([0 1])
end

user_param.S_target = S_target;
user_param.n_target_blocks = 1;
user_param.S_target_blocks{1} = S_target;
user_param.S_obstacle = S_obstacle;
user_param.S_obstacle_blocks = {S_obstacle};
user_param.n_obstacle_blocks = 1;
user_param.S = S;
user_param.S_terminal = S_terminal;
user_param.S_not_terminal = S_not_terminal;

if ( solver_opts.group_terminal_states_together ) 
    [state_discretization, user_param] = make_single_goals_3d_fcn(state_discretization, user_param, original_sys);
end

if ( solver_opts.group_obs_states_together && isfield(user_param, 'Nx_S_obstacle') ) 
    [state_discretization, user_param] = make_single_obstacles_3d_fcn(state_discretization, user_param, original_sys);
end

end