clear
clc
close all

%% Original system description

original_sys.name = 'unicycle2d';
% original_sys.name = 'unicycle3d_nl_friction';
% original_sys.name = 'unicycle4d_nl_friction';
%original_sys.name = 'pendulum';
%original_sys.name = 'mult_noise';
% original_sys.name = 'unicycle2d_ltlf';
% original_sys.name = 'temp';
% original_sys.name = 'fm1';
% original_sys.name = 'fm2';
%original_sys.name = 'lr';

%solver_opts.abstraction_choice = 'IMDP_naive';
%solver_opts.abstraction_choice = 'IMDP_w_support_single_goal_l4dc2025';
% solver_opts.abstraction_choice = 'IMDP_w_support_single_goal_and_obs';
%solver_opts.abstraction_choice = 'IMDP_w_support_single_goal_and_obs_l4dc2025';
% solver_opts.abstraction_choice = '2layer';
% solver_opts.abstraction_choice = '2layer_single_goal_and_obs';
%solver_opts.abstraction_choice = '2layer_single_goal_and_obs_l4dc2025';
%solver_opts.abstraction_choice = '2layer_single_goal_l4dc2025';
%solver_opts.abstraction_choice = '2layer_overlapping';
% solver_opts.abstraction_choice = 'mdpist2';
% solver_opts.abstraction_choice = 'mdpist2_voronoi';
% solver_opts.abstraction_choice = 'mdpust_2layer';
%solver_opts.abstraction_choice = 'mdpust_2layer_overlapping';
%solver_opts.abstraction_choice = 'imdpst_ss';
%solver_opts.abstraction_choice = 'imdpst_ss_coarse';
% solver_opts.abstraction_choice = 'imdpst';
% solver_opts.abstraction_choice = 'multi_imdp';
% solver_opts.abstraction_choice = 'multi_imdp_single_goal_and_obs';
% solver_opts.abstraction_choice = 'imdpst_partition_W';
% solver_opts.abstraction_choice = 'cm_umdp';
% solver_opts.abstraction_choice = 'cme_umdp';
% solver_opts.abstraction_choice = 'multi_imdp_dd';

% Use these ones for the unicycle2D (and 3D) benchmarks
% solver_opts.abstraction_choice = 'multi_imdp_single_goal_and_obs';
% solver_opts.abstraction_choice = 'mdpst';
% solver_opts.abstraction_choice = 'IMDP_w_support_single_goal_and_obs';
solver_opts.abstraction_choice = '2layer_single_goal_and_obs';


% Use these ones for the temperature benchmark
% solver_opts.abstraction_choice = 'mdpst';
% solver_opts.abstraction_choice = 'IMDP_w_support_single_goal';
% solver_opts.abstraction_choice = '2layer_single_goal';
% solver_opts.abstraction_choice = 'multi_imdp_single_goal_and_obs';

if ( strcmp( solver_opts.abstraction_choice, 'IMDP_naive' ) )
    solver_opts.value_iteration_method = 'IVI_single_goal'; % Interval value iteration
elseif ( strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal' ) || strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_l4dc2025') || strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs_l4dc2025' )  || strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs')  )
    solver_opts.value_iteration_method = 'IVI_single_goal'; % Interval value iteration
elseif ( strcmp( solver_opts.abstraction_choice, '2layer' ) || strcmp( solver_opts.abstraction_choice, '2layer_single_goal' ) || strcmp( solver_opts.abstraction_choice, '2layer_single_goal_and_obs' ) || strcmp( solver_opts.abstraction_choice, '2layer_single_goal_l4dc2025' ) || strcmp( solver_opts.abstraction_choice, '2layer_single_goal_and_obs_l4dc2025' ) )
    solver_opts.value_iteration_method = 'RVI_2layer'; % Interval value iteration
elseif ( strcmp( solver_opts.abstraction_choice, '2layer_overlapping' ) )
    solver_opts.value_iteration_method = 'RVI_just_gurobi_overlapping_cluster';
elseif ( strcmp( solver_opts.abstraction_choice, 'multi_imdp' ) || strcmp( solver_opts.abstraction_choice, 'multi_imdp_single_goal_and_obs' ) )
    solver_opts.value_iteration_method = 'RVI_just_gurobi_multi_imdp_cluster';
elseif ( strcmp( solver_opts.abstraction_choice, 'imdpst' ) )
    solver_opts.value_iteration_method = 'RVI_imdpst'; %
elseif ( strcmp( solver_opts.abstraction_choice, 'mdpst' ) )
    solver_opts.value_iteration_method = 'RVI_mdpst'; %
elseif ( strcmp( solver_opts.abstraction_choice, 'imdpst_partition_W' ) )
    solver_opts.value_iteration_method='RVI_imdpst_partition_W';
elseif ( strcmp( solver_opts.abstraction_choice, 'cme_umdp' ) || strcmp( solver_opts.abstraction_choice, 'me_umdp' ) )
    solver_opts.value_iteration_method='RVI_cme_umdp';
elseif ( strcmp(solver_opts.abstraction_choice, 'multi_imdp_dd') )
    solver_opts.value_iteration_method = 'RVI_just_gurobi_multi_imdp_cluster';
end

%solver_opts.value_iteration_method = 'RVI_mdpist2_tailored'; % Interval value iteration
%solver_opts.value_iteration_method = 'RVI_mdpust_2layer_overlapping';
%solver_opts.value_iteration_method = 'RVI_mdpust_2layer_overlapping_cluster';
%solver_opts.value_iteration_method = 'RVI_imdpst_ss';
%solver_opts.value_iteration_method = 'RVI_imdpst_ss_coarse';

user_param.controller_given_yn = ~true;
solver_opts.clopper_pearson_yn = true;
solver_opts.compute_clopper_pearson_table_yn = true;
original_sys.data_driven_yn = ~true;
solver_opts.known_f_yn = true;
solver_opts.partition_W_yn = true;

% State space (\R^n)
if ( strcmp(original_sys.name, 'unicycle2d') && ~original_sys.data_driven_yn )
    original_sys.n = 2;
    % Define set X:
    original_sys.state_space_lower_left_corner = zeros(1, original_sys.n);
    original_sys.state_space_upper_right_corner = ones(1, original_sys.n);
    % Uncertainty space (\R^d)
    % Define set W:
    original_sys.d = 2;
    user_param.c = 1.4;
    if ( strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_l4dc2025' ) || strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs_l4dc2025' ) )
        user_param.epsilon_support = 0.001;
        user_param.n_transitions_est = 3e7;
    elseif ( strcmp( solver_opts.abstraction_choice, '2layer_single_goal_l4dc2025' ) )
        user_param.epsilon_support = 0.001;
        user_param.n_transitions_est = 3e7;
    end
    % Center of support
    original_sys.center_samples = [0, 0]';
    original_sys.disturbance_space_lower_left_corner = -user_param.c*ones(1, original_sys.d) + original_sys.center_samples';
    original_sys.disturbance_space_upper_right_corner = user_param.c*ones(1, original_sys.d) + original_sys.center_samples';
    original_sys.weight_samples = 2e-1;
    if ( original_sys.data_driven_yn )
        original_sys.Pw_class = 'heavy_tail';
    else
        original_sys.Pw_class = 'gaussian';
    end
    original_sys.rho = 3;
    % Drag coefficient
    original_sys.cd = 0.15; %6e-2; % 0.15;
    % Max (min) control input
    user_param.Umax_th = pi;
    user_param.Umax_vel = 0.1;
    % Time discretization
    user_param.Deltat = 1;
    % Number of actions
    user_param.Na_th = 8;
    solver_opts.learn_support = true;
elseif ( strcmp(original_sys.name, 'unicycle2d') && original_sys.data_driven_yn && strcmp(solver_opts.abstraction_choice, 'imdpst') )
    original_sys.n = 2;
    % Define set X:
    original_sys.state_space_lower_left_corner = zeros(1, original_sys.n);
    original_sys.state_space_upper_right_corner = ones(1, original_sys.n);
    % Uncertainty space (\R^d)
    % Define set W:
    original_sys.d = 2;
    % Center of support
    original_sys.center_samples = [0, 0]';
    original_sys.weight_samples = 2e-1;
    user_param.c = 1.4;%5*original_sys.weight_samples;
    if ( original_sys.data_driven_yn )
        original_sys.Pw_class = 'heavy_tail';
    else
        original_sys.Pw_class = 'gaussian';
    end
    original_sys.rho = 3;
    % Drag coefficient
    original_sys.cd = 6e-2; % 0.15;
    % Max (min) control input
    user_param.Umax_th = pi;
    user_param.Umax_vel = 0.1;
    % Time discretization
    user_param.Deltat = 1;
    % Number of actions
    user_param.Na_th = 8;
    solver_opts.learn_support = true;
elseif ( strcmp(original_sys.name, 'unicycle2d') && original_sys.data_driven_yn && ~strcmp(solver_opts.abstraction_choice, 'imdpst') )
    original_sys.n = 2;
    % Define set X:
    original_sys.state_space_lower_left_corner = zeros(1, original_sys.n);
    original_sys.state_space_upper_right_corner = ones(1, original_sys.n);
    % Uncertainty space (\R^d)
    % Define set W:
    original_sys.d = 2;
    % Center of support
    original_sys.center_samples = [0, 0]';
    original_sys.weight_samples = 2e-1;
    user_param.c = 5*original_sys.weight_samples;
    original_sys.disturbance_space_lower_left_corner = -user_param.c*ones(1, original_sys.d) + original_sys.center_samples';
    original_sys.disturbance_space_upper_right_corner = user_param.c*ones(1, original_sys.d) + original_sys.center_samples';
    if ( original_sys.data_driven_yn )
        original_sys.Pw_class = 'heavy_tail';
    else
        original_sys.Pw_class = 'gaussian';
    end
%     original_sys.rho = 3;
    % Drag coefficient
    original_sys.cd = 6e-2; % 0.15;
    % Max (min) control input
    user_param.Umax_th = pi;
    user_param.Umax_vel = 0.1;
    % Time discretization
    user_param.Deltat = 1;
    % Number of actions
    user_param.Na_th = 8;
    solver_opts.learn_support = true;
elseif ( strcmp(original_sys.name, 'unicycle3d_nl_friction') )
    original_sys.n = 3;
    % Define set X:
    original_sys.state_space_lower_left_corner = zeros(1, original_sys.n);
    original_sys.state_space_upper_right_corner = ones(1, original_sys.n);
    % Uncertainty space (\R^d)
    % Define set W:
    original_sys.d = 2;
    original_sys.weight_samples = 1/2; % 1/5; % 1/15
    user_param.c = original_sys.weight_samples/(1/5)*0.8;
    original_sys.center_samples = 0.4*[1, 0]';
    original_sys.disturbance_space_lower_left_corner = -user_param.c*ones(1, original_sys.d) + original_sys.center_samples';
    original_sys.disturbance_space_upper_right_corner = user_param.c*ones(1, original_sys.d) + original_sys.center_samples';
    original_sys.Pw_class = 'gaussian';
    original_sys.rho = 10;
    % Drag coefficient
    original_sys.cd = [0.1; 0.05];
    % Max (min) control input
    user_param.Umax_th = 2;
    user_param.Umax_vel = 0.3;
    user_param.Umin_vel = 0.7*user_param.Umax_vel; % 0.5*user_param.Umax_vel;
    % Time discretization
    user_param.Deltat = 0.5;
    % Number of actions
    user_param.Na_th = 5;
elseif ( strcmp(original_sys.name, 'unicycle4d_nl_friction') )
    original_sys.n = 4;
    % Define set X:
    original_sys.state_space_lower_left_corner = zeros(1, original_sys.n);
    original_sys.state_space_upper_right_corner = ones(1, original_sys.n);
    % Uncertainty space (\R^d)
    % Define set W:
    original_sys.d = 3;
    original_sys.weight_samples = 1/500; % 1/5; % 1/5; % 1/15
    user_param.c = original_sys.weight_samples/(1/5)*0.8;
    original_sys.center_samples = 0.4*[1, 0, 0]';
    original_sys.disturbance_space_lower_left_corner = -user_param.c*ones(1, original_sys.d) + original_sys.center_samples';
    original_sys.disturbance_space_upper_right_corner = user_param.c*ones(1, original_sys.d) + original_sys.center_samples';
    original_sys.Pw_class = 'gaussian';
    original_sys.rho = 10;
    % Drag coefficient
    original_sys.cd = [0.1; 0.05; 0.05];
    % Max (min) control input
    user_param.Umax = [2; 1]; % [\omega, a]
    % Time discretization
    user_param.Deltat = 0.5;
    % Number of actions
    user_param.N_yaw_rate = 3;
    user_param.N_lin_acc = 6;
    % Maximum linear speed
    user_param.vel_max = 0.5;
elseif ( strcmp(original_sys.name, 'pendulum') )
    original_sys.n = 3;
    % Define set X:
    original_sys.state_space_lower_left_corner = zeros(1, original_sys.n);
    original_sys.state_space_upper_right_corner = ones(1, original_sys.n);
    % Uncertainty space (\R^d)
    % Define set W:
    original_sys.d = 1;
    original_sys.center_samples = 0;
    user_param.c = 0.9;%0.8;
    original_sys.disturbance_space_lower_left_corner = -user_param.c*ones(1, original_sys.d) + original_sys.center_samples';
    original_sys.disturbance_space_upper_right_corner = user_param.c*ones(1, original_sys.d) + original_sys.center_samples';
    % Bounds on the support W\subset \R^m
    original_sys.rho = 3; % 3 S.D.
    % Length of the pendulum
    original_sys.l = 1;
    % Mass
    original_sys.M = 1;
    % Gravity
    original_sys.g = 1;
    % Drag coefficient
    original_sys.cd = 0.2;%0.3; % 2*0.1;
    original_sys.Pw_class = 'gaussian';
    % Weight on samples
    original_sys.weight_samples = 1/4.5;%1/5; % 1/4
    % Time discretization
    user_param.Deltat = 0.3; %0.25;
    % Max allowed velocity
    user_param.th_dot_max = 3;%3; %4;
    % Number of actions
    user_param.Na = 5;
    % Max control input
    user_param.Umax = 0.415;%0.8; % 0.415;
    if ( strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_l4dc2025' ) || strcmp( solver_opts.abstraction_choice, 'IMDP_w_support_single_goal_and_obs_l4dc2025' ) )
        user_param.epsilon_support = 0.001;
        user_param.n_transitions_est = 25000000;
    elseif ( strcmp( solver_opts.abstraction_choice, '2layer_single_goal_l4dc2025' ) )
        user_param.epsilon_support = 0.001;
        user_param.n_transitions_est = 25000000; % 17289408;
    end
    solver_opts.learn_support = true;
elseif ( strcmp(original_sys.name, 'mult_noise') )
    % State space (\R^n)
    original_sys.n = 2;
    % Define set X:
    original_sys.state_space_lower_left_corner = [0 0];
    original_sys.state_space_upper_right_corner = [2 2];
    % Uncertainty space (\R^d)
    original_sys.d = 2;
    % Weight on samples
    original_sys.weight_samples = 0.1;
    % Bounds on the support W\subset \R^m
    original_sys.rho = 0.1;
    % A matrix
    original_sys.A = [0.7 0.1; 0.1 0.8];
    original_sys.Pw_class = 'gaussian';
    % Weight on samples
    original_sys.center_samples = ones(original_sys.d, 1);
    solver_opts.learn_support = ~true;
    user_param.n_transitions_est = 6011087;
    user_param.epsilon_support = 0.001;
elseif ( strcmp(original_sys.name, 'unicycle2d_ltlf') )
    % State space (\R^n)
    original_sys.n = 3;
    % Define set X:
    original_sys.state_space_lower_left_corner = [0 0];
    original_sys.state_space_upper_right_corner = [1 1];
    % Uncertainty space (\R^d)
    original_sys.Pw_class = 'gaussian';
    original_sys.d = 1;
    % Weight on samples
    original_sys.weight_samples = 1/15;
    % Center of support
    original_sys.center_samples = 0.4;
    % Bounds on the support W\subset \R^m
    original_sys.rho = 1;
    % Drag coefficient
    original_sys.cd = 0.20;
    % Number of actions
    user_param.Na_th = 8;
    % Time discretization
    user_param.Deltat = 0.5;
    % Max (min) control input
    user_param.Umax_th = pi;
    user_param.Umax_vel = 0.3;
    solver_opts.learn_support = ~true;
    user_param.n_transitions_est = 8167938;
    user_param.epsilon_support = 0.001;
elseif ( strcmp(original_sys.name, 'temp') )
    % State space (\R^n)
    original_sys.n = 3;
    % Define set X:
    original_sys.state_space_lower_left_corner = 21 - 2.5*ones(1, original_sys.n); % [21-2.5 21-2.5 39.7 39.7] + 273;
    original_sys.state_space_upper_right_corner = 21 + 2.5*ones(1, original_sys.n); % [21+2.5 21+2.5 45.3 45.3] + 273;
    % Define set W:
    original_sys.Pw_class = 'gaussian';
    original_sys.d = original_sys.n;
    original_sys.center_samples = zeros(original_sys.d, 1);
    % Weight on samples
    original_sys.weight_samples = 1/150;
    user_param.c = original_sys.weight_samples/(1/150)*0.0295;
    original_sys.rho = 100;
    original_sys.disturbance_space_lower_left_corner = -user_param.c*ones(1, original_sys.d) + original_sys.center_samples';
    original_sys.disturbance_space_upper_right_corner = user_param.c*ones(1, original_sys.d) + original_sys.center_samples';
    % Coefficients
    original_sys.a_ij = 0.0625;
    original_sys.b_i = 0.0365;
    original_sys.c_i = 0.8;%1;%0.7;%0.6;
    original_sys.Ta = 6;

    % System Matrices                                                                      0 0 1 -1];
    for i = 1:original_sys.n
        for j = 1:original_sys.n
            if (abs(i-j) == 1)
                original_sys.A(i,j) = original_sys.a_ij;
            end
        end
    end
    for i = 1:original_sys.n
        original_sys.A(i,i) = original_sys.A(i,i) - sum(original_sys.A(i,:));
    end
    original_sys.A = original_sys.A + (1-original_sys.b_i)*eye(original_sys.n);
    original_sys.b = original_sys.b_i*original_sys.Ta*ones(original_sys.n, 1);
    original_sys.bu = original_sys.c_i*eye(original_sys.n);
elseif ( strcmp(original_sys.name, 'fm1') && ~strcmp(solver_opts.abstraction_choice, 'cme_umdp') )
    % State space (\R^n)
    original_sys.n = 1;
    % Define set X:
    original_sys.state_space_lower_left_corner = -2.5*ones(1, original_sys.n); % [21-2.5 21-2.5 39.7 39.7] + 273;
    original_sys.state_space_upper_right_corner = 2.5*ones(1, original_sys.n); % [21+2.5 21+2.5 45.3 45.3] + 273;
    % Define set W:
    original_sys.Pw_class = 'gaussian';
    original_sys.d = original_sys.n;
    original_sys.center_samples = zeros(original_sys.d, 1);
    % Weight on samples
    original_sys.weight_samples = 0.01;
    original_sys.cd = 1;
    user_param.c = 4*original_sys.weight_samples;
    original_sys.rho = 100;
%     original_sys.disturbance_space_lower_left_corner = -user_param.c*ones(1, original_sys.d) + original_sys.center_samples';
%     original_sys.disturbance_space_upper_right_corner = user_param.c*ones(1, original_sys.d) + original_sys.center_samples';
    % Non-determinism
    original_sys.d_nd_max = 0.05;
    original_sys.a_X0 = -0.5;
    original_sys.b_X0 = 0.5;
elseif ( strcmp(original_sys.name, 'fm1') && strcmp(solver_opts.abstraction_choice, 'cme_umdp') )
    % State space (\R^n)
    original_sys.n = 1;
    % Define set X:
    original_sys.state_space_lower_left_corner = -2.5*ones(1, original_sys.n); % [21-2.5 21-2.5 39.7 39.7] + 273;
    original_sys.state_space_upper_right_corner = 2.5*ones(1, original_sys.n); % [21+2.5 21+2.5 45.3 45.3] + 273;
    % Define set W:
    original_sys.Pw_class = 'gaussian';
    original_sys.d = original_sys.n;
    original_sys.center_samples = zeros(original_sys.d, 1);
    % Weight on samples
    original_sys.weight_samples = 0.01;
    original_sys.cd = 1;
    user_param.c = 4*original_sys.weight_samples;
    original_sys.rho = 100;
%     original_sys.disturbance_space_lower_left_corner = -user_param.c*ones(1, original_sys.d) + original_sys.center_samples';
%     original_sys.disturbance_space_upper_right_corner = user_param.c*ones(1, original_sys.d) + original_sys.center_samples';
    % Non-determinism
    original_sys.d_nd_max = 0.05;
    original_sys.a_X0 = -0.5;
    original_sys.b_X0 = 0.5;
    solver_opts.sigmaf = 1;
    solver_opts.sigmal = 10;
elseif ( strcmp(original_sys.name, 'fm2') )
    % State space (\R^n)
    original_sys.n = 2;
    % Define set X:
    original_sys.state_space_lower_left_corner = [-10, -10]; % [21-2.5 21-2.5 39.7 39.7] + 273;
    original_sys.state_space_upper_right_corner = [14, 10]; % [21+2.5 21+2.5 45.3 45.3] + 273;
    % Define set W:
    original_sys.Pw_class = 'gaussian';
    original_sys.d = original_sys.n;
    original_sys.center_samples = zeros(original_sys.d, 1);
    % Weight on samples
    original_sys.weight_samples = 0.1;
    original_sys.cd = 1;
    user_param.c = 4*original_sys.weight_samples;
    original_sys.rho = 100;
%     original_sys.disturbance_space_lower_left_corner = -user_param.c*ones(1, original_sys.d) + original_sys.center_samples';
%     original_sys.disturbance_space_upper_right_corner = user_param.c*ones(1, original_sys.d) + original_sys.center_samples';
    % System matrices
    original_sys.A = [1, 0.1; 0, 0.9];
    original_sys.a_X0 = [0.5, -0.5];
    original_sys.b_X0 = [1, 0.5];
elseif ( strcmp(original_sys.name, 'lr') )
    % State space (\R^n)
    original_sys.n = 1;
    % Define safe set
    original_sys.state_space_safe_set_lower_left_corner = 19;
    original_sys.state_space_safe_set_upper_right_corner = 22;
    solver_opts.offset_unsafe_set = 0.6;
    % Define set X:
    original_sys.state_space_lower_left_corner = original_sys.state_space_safe_set_lower_left_corner - solver_opts.offset_unsafe_set; % [21-2.5 21-2.5 39.7 39.7] + 273;
    original_sys.state_space_upper_right_corner = solver_opts.offset_unsafe_set + original_sys.state_space_safe_set_upper_right_corner; % [21+2.5 21+2.5 45.3 45.3] + 273;
    original_sys.state_space_middle = (original_sys.state_space_upper_right_corner + original_sys.state_space_lower_left_corner)/2;
    % Define set W:
    original_sys.Pw_class = 'gaussian';
    original_sys.d = original_sys.n;
    original_sys.center_samples = zeros(original_sys.d, 1);
    % Weight on samples
    original_sys.weight_samples = 0.2;
    original_sys.cd = 1;
    user_param.c = 4*original_sys.weight_samples;
    original_sys.rho = 100;
    % System matrices
    original_sys.h = 5/60; % hours
    original_sys.C = 2; % kWh/degC
    original_sys.R = 2; % degC/kW
    original_sys.P = 14; % kW
    original_sys.eta = 0.7;
    original_sys.theta = 32; % degC
    original_sys.alpha = exp(original_sys.h/(original_sys.C*original_sys.R));
    original_sys.A = original_sys.alpha;
    original_sys.bu = (1-original_sys.alpha)*(-original_sys.eta*original_sys.R*original_sys.P);
    original_sys.b = (1-original_sys.alpha)*original_sys.theta;
    solver_opts.sigmaf = 10;
    solver_opts.sigmal = 0.5;
    solver_opts.lambda = 1e-3;
    user_param.beta = 1e-9;
end

%% User parameters

% Number of states of the IMDP
user_param.Nx = {30,30};%, 30, 40, 12};
% Number of regions c in the partition of the disturbance
user_param.Nc = 250; % Used if voronoi
user_param.Nxd = {12,12,4,4};

% 1 - confidence
user_param.alpha = 1e-6;

% Parameters for learning the support
user_param.beta_support = 0.5*user_param.alpha;

%% Parameters for coarse discretization

% Number of aggregate regions
if ( strcmp( original_sys.name, 'temp' ) )
    user_param.coarse_Deltax_ratio = {2,2,2,2}; % Deltax_coarse/Deltax
else
    user_param.coarse_Deltax_ratio = {3, 3, 3}; % Deltax_coarse/Deltax
end

% Number of aggregate regions (for disturbance)
user_param.coarse_Deltaxd_ratio = {2, 2}; % Deltaxd_coarse/Deltaxd

%% Parameters for samples

% Number of samples
user_param.N = 1e4; % 

% Using clustering or not
solver_opts.cluster_samples = ~true;

%% Generate discretization

user_param.overlaping = true;

[state_discretization, user_param] = discretize_state_space(original_sys, user_param, solver_opts);
[disturbance_discretization, user_param] = discretize_disturbance_space(original_sys, user_param, solver_opts);

%% Additional parameters

solver_opts.precision = 1e-7; % Precision threshold for checking intersections

%% (Just for debugging purposes) Plot discretization

plot_grid = 1;
plot_discretization(state_discretization, original_sys, plot_grid, solver_opts, user_param)
plot_grid = 1;
plot_disturbance_discretization(disturbance_discretization, user_param, original_sys, plot_grid, solver_opts)

%% Discretize action set

plot_one_step_ahead_yn = 1;
%s = user_param.Nx*user_param.Ny/2 + user_param.Nx/2;
s = 1;
[original_sys, user_param, solution] = discretize_action_set_fcn(original_sys, user_param, state_discretization, plot_one_step_ahead_yn, s);

%% Generate samples

user_param.tol_cluster_samples = 0.05; % 0.008;
plot_samples_w_discretization = 1;
generate_samples_yn = true;
if (generate_samples_yn)
    samples = [];
end
[samples, user_param] = generate_and_cluster_samples_fcn(original_sys, state_discretization, disturbance_discretization,user_param, plot_samples_w_discretization, solver_opts, samples);

%% Compute Lipschitz constant w.r.t. w

original_sys = compute_lipschitz_constants_fcn(original_sys, user_param, state_discretization);

%% Define states of the abstraction

user_param.plot_roi = true;

if ( strcmp(original_sys.name, 'unicycle2d' ) )
    user_param.width_S_target = 0.99*1/5; % Width of the tgt set (tgt is a square)
elseif ( strcmp(original_sys.name, 'unicycle3d_nl_friction' ) )
    user_param.width_S_target = 0.99*2.666666666666667e-01; % ceil(user_param.Nx{1}/4);
    user_param.width_S_obstacle = 0.99*1.666666666666667e-01; % ceil(user_param.Nx{1}/7); % Width of obstacle for unicycle3d_nl_friction
elseif ( strcmp(original_sys.name, 'unicycle4d_nl_friction' ) )
    user_param.Nx_S_target = ceil(user_param.Nx{1}/4);
    %user_param.Nx_S_obstacle = ceil(user_param.Nx{1}/7); % Width of obstacle for unicycle3d_nl_friction
elseif ( strcmp(original_sys.name, 'pendulum' ) )
    user_param.Nx_S_target = ceil( user_param.Nx*40/200 ); % Length of the tgt set (must be even number)
elseif ( strcmp(original_sys.name, 'mult_noise' ) )
    user_param.Nx_S_target = ceil( (original_sys.state_space_upper_right_corner(1) - original_sys.state_space_lower_left_corner(1))/state_discretization.Deltax/4 ); % Length of the tgt set (must be even number)
elseif ( strcmp(original_sys.name, 'unicycle2d_ltlf' ) )
    user_param.Nx_S_target = ceil(user_param.Nx/5); % Width of target
end
[user_param, state_discretization, solver_opts] = define_roi(user_param, original_sys, state_discretization, solver_opts);


%% Obtain abstraction

[solution, error_flags] = obtain_abstraction_fcn(user_param, state_discretization, disturbance_discretization, original_sys, solver_opts, samples, solution);

% Check abstraction

is_correct_yn = check_abstraction_fcn(solution, error_flags, solver_opts, user_param);

%% Plot bounds for fixed (q, a) (Just for debugging purposes)

debug = ~true;

% Current state and action
s = 9;%ceil(0.8*user_param.Nx{4})*user_param.Nx{1}*user_param.Nx{2}*user_param.Nx{3}; % randsample(user_param.S_not_terminal, 1);
a = 8;
n_samples = 2;% user_param.Nc;

n_samples = min(n_samples, user_param.Nc);

% % % Check if q is terminal
% % if (any(ismember(user_param.Q_sink,q)))
% %     disp('q is a sink state. Choose another state')
% % end
if (debug)

    if ( strcmp(original_sys.name, 'unicycle3d_nl_friction') && ( strcmp(solver_opts.abstraction_choice, 'IMDP_naive_single_goal') || strcmp(solver_opts.abstraction_choice, 'IMDP_w_support_single_goal') || strcmp(solver_opts.abstraction_choice, '2layer_overlapping') || strcmp(solver_opts.abstraction_choice, '2layer') ) )
        % Check if q is terminal
        if (any(ismember(user_param.S_terminal,s)))
            disp('s is a terminal state. Choosing another state..')
            s = user_param.S_not_terminal(1);
        end

        % Take care of the unsafe state
        if (solution.Sp{s,a}(end) == user_param.Ns+1)
            % Plot bounds for current state and action
            plot_bounds_3D(solution.P_lower{s,a}(1:end-1),solution.P_upper{s,a}(1:end-1),solution.Sp{s,a}(1:end-1),state_discretization)
        else
            % Plot bounds for current state and action
            plot_bounds_3D(solution.P_lower{s,a},solution.P_upper{s,a},solution.Sp{s,a},state_discretization)
        end

        % Plot vertices of Post
        seed = RandStream('mlfg6331_64');
        indices_samples = datasample(seed,1:length(samples.weight_norm), min([n_samples, length(samples.weight)]),'Weights',samples.weight_norm);
        figure(1)
        hold on
        for w_ind = 1:min([n_samples, length(samples.weight)])
            u = original_sys.Act(:,a);
            w = samples.Xi_hat_c(:, indices_samples(w_ind) );%datasample(samples.Xi_hat_c,1,2);
            % Compute Post_q_a_w
            [a_Post, b_Post, error_flag_post] = compute_reach_simplified_unicycle3d_fcn(state_discretization.vertices_s{s},u,w,original_sys,user_param);
            a_Post = a_Post - original_sys.Lw_s_a{s,a}*samples.size_cluster( indices_samples(w_ind) );
            b_Post = b_Post + original_sys.Lw_s_a{s,a}*samples.size_cluster( indices_samples(w_ind) );
            R = [a_Post(1) a_Post(2) a_Post(3); b_Post(1) a_Post(2) a_Post(3); a_Post(1) b_Post(2) a_Post(3); b_Post(1) b_Post(2) a_Post(3); a_Post(1) a_Post(2) b_Post(3); b_Post(1) a_Post(2) b_Post(3); a_Post(1) b_Post(2) b_Post(3); b_Post(1) b_Post(2) b_Post(3)]';
            plot3(R(1, :), R(2, :), R(3, :),'color','g')
        end

        % Plot vertices of Reach_nd_q_u
        w = original_sys.center_samples; % Center of W
        [a_Post_w0, b_Post_w0, error_flag_post] = compute_reach_simplified_unicycle3d_fcn(state_discretization.vertices_s{s},u,w,original_sys,user_param);
        [a_Reach_nd_q_a, b_Reach_nd_q_a] = compute_Reach_nd_simplified_q_u(a_Post_w0, b_Post_w0, original_sys.Lw_s_a{s,a}, user_param.c);
        Reach_nd_q_a = [a_Reach_nd_q_a(1) a_Reach_nd_q_a(2) a_Reach_nd_q_a(3)
            a_Reach_nd_q_a(1) b_Reach_nd_q_a(2) a_Reach_nd_q_a(3)
            a_Reach_nd_q_a(1) a_Reach_nd_q_a(2) b_Reach_nd_q_a(3)
            a_Reach_nd_q_a(1) b_Reach_nd_q_a(2) b_Reach_nd_q_a(3)
            b_Reach_nd_q_a(1) a_Reach_nd_q_a(2) a_Reach_nd_q_a(3)
            b_Reach_nd_q_a(1) b_Reach_nd_q_a(2) a_Reach_nd_q_a(3)
            b_Reach_nd_q_a(1) a_Reach_nd_q_a(2) b_Reach_nd_q_a(3)
            b_Reach_nd_q_a(1) b_Reach_nd_q_a(2) b_Reach_nd_q_a(3)]';
        plot3(Reach_nd_q_a(1,:), Reach_nd_q_a(2,:), Reach_nd_q_a(3,:),'x-')

    elseif ( strcmp(original_sys.name, 'unicycle4d_nl_friction') && ( strcmp(solver_opts.abstraction_choice, 'mdpst') ||strcmp(solver_opts.abstraction_choice, 'IMDP_naive_single_goal') || strcmp(solver_opts.abstraction_choice, 'IMDP_w_support_single_goal') || strcmp(solver_opts.abstraction_choice, '2layer_overlapping') || strcmp(solver_opts.abstraction_choice, '2layer') ) )
        % Check if q is terminal
        if ( any( ismember(user_param.S_terminal, s) ) )
            disp('s is a terminal state. Choosing another state..')
            s = user_param.S_not_terminal(1);
        end

        ci = 1;
        % Take care of the unsafe state
        if (solution.Qp{s,a}.children{ci}(end) == user_param.Ns+1)
            q_s_a_c_3dProj = zeros(length(solution.Qp{s,a}.children{ci} ) - 1, 1);
        else
            q_s_a_c_3dProj = zeros(length(solution.Qp{s,a}.children{ci} ), 1);
        end
        for spi = 1:length(q_s_a_c_3dProj)
            sp = solution.Qp{s,a}.children{ci}(spi);
            q_s_a_c_3dProj(spi) = rem( sp, user_param.Nx{1}*user_param.Nx{2}*user_param.Nx{3} );
            if ( q_s_a_c_3dProj(spi) == 0 ) % End of previous block
                % Put it at end of current block
                q_s_a_c_3dProj(spi) = user_param.Nx{1}*user_param.Nx{2}*user_param.Nx{3};
            end
        end
        scatter3(state_discretization.X_center{1}(q_s_a_c_3dProj), state_discretization.X_center{2}(q_s_a_c_3dProj), state_discretization.X_center{3}(q_s_a_c_3dProj), 40,'filled')
        SProj = 1:user_param.Nx{1}*user_param.Nx{2}*user_param.Nx{3};
        P_s_a_Proj = zeros( size(SProj) );
        aux_xyth_plane = solution.Sp{s,a}
        if (solution.Sp{s,a}(end) == user_param.Ns+1)
            % Plot bounds for current state and action
            plot_bounds_3D(solution.P_lower{s,a}(1:end-1), solution.P_upper{s,a}(1:end-1), solution.Sp{s,a}(1:end-1), state_discretization)
        else
            % Plot bounds for current state and action
            plot_bounds_3D(solution.P_lower{s,a}, solution.P_upper{s,a}, solution.Sp{s,a}, state_discretization)
        end

        % Plot vertices of Post
        seed = RandStream('mlfg6331_64');
        indices_samples = datasample(seed,1:length(samples.weight_norm), min([n_samples, length(samples.weight)]),'Weights',samples.weight_norm);
        figure(1)
        hold on
        for w_ind = 1:min([n_samples, length(samples.weight)])
            u = original_sys.Act(:,a);
            w = samples.Xi_hat_c(:, indices_samples(w_ind) );%datasample(samples.Xi_hat_c,1,2);
            % Compute Post_q_a_w
            [a_Post, b_Post, error_flag_post] = compute_reach_simplified_unicycle3d_fcn(state_discretization.vertices_s{s},u,w,original_sys,user_param);
            a_Post = a_Post - original_sys.Lw_s_a{s,a}*samples.size_cluster( indices_samples(w_ind) );
            b_Post = b_Post + original_sys.Lw_s_a{s,a}*samples.size_cluster( indices_samples(w_ind) );
            R = [a_Post(1) a_Post(2) a_Post(3); b_Post(1) a_Post(2) a_Post(3); a_Post(1) b_Post(2) a_Post(3); b_Post(1) b_Post(2) a_Post(3); a_Post(1) a_Post(2) b_Post(3); b_Post(1) a_Post(2) b_Post(3); a_Post(1) b_Post(2) b_Post(3); b_Post(1) b_Post(2) b_Post(3)]';
            plot3(R(1, :), R(2, :), R(3, :),'color','g')
        end

        % Plot vertices of Reach_nd_q_u
        w = original_sys.center_samples; % Center of W
        [a_Post_w0, b_Post_w0, error_flag_post] = compute_reach_simplified_unicycle3d_fcn(state_discretization.vertices_s{s},u,w,original_sys,user_param);
        [a_Reach_nd_q_a, b_Reach_nd_q_a] = compute_Reach_nd_simplified_q_u(a_Post_w0, b_Post_w0, original_sys.Lw_s_a{s,a}, user_param.c);
        Reach_nd_q_a = [a_Reach_nd_q_a(1) a_Reach_nd_q_a(2) a_Reach_nd_q_a(3)
            a_Reach_nd_q_a(1) b_Reach_nd_q_a(2) a_Reach_nd_q_a(3)
            a_Reach_nd_q_a(1) a_Reach_nd_q_a(2) b_Reach_nd_q_a(3)
            a_Reach_nd_q_a(1) b_Reach_nd_q_a(2) b_Reach_nd_q_a(3)
            b_Reach_nd_q_a(1) a_Reach_nd_q_a(2) a_Reach_nd_q_a(3)
            b_Reach_nd_q_a(1) b_Reach_nd_q_a(2) a_Reach_nd_q_a(3)
            b_Reach_nd_q_a(1) a_Reach_nd_q_a(2) b_Reach_nd_q_a(3)
            b_Reach_nd_q_a(1) b_Reach_nd_q_a(2) b_Reach_nd_q_a(3)]';
        plot3(Reach_nd_q_a(1,:), Reach_nd_q_a(2,:), Reach_nd_q_a(3,:),'x-')


    elseif ( strcmp(original_sys.name, 'pendulum') )
        % Take care of the unsafe state
        if (solution.Sp{s,a}(end) == user_param.Ns+1)
            % Plot bounds for current state and action
            plot_bounds(solution.P_lower{s,a}(1:end-1),solution.P_upper{s,a}(1:end-1),solution.Sp{s,a}(1:end-1),state_discretization,user_param, original_sys, 0,1)
        else
            % Plot bounds for current state and action
            plot_bounds(solution.P_lower{s,a},solution.P_upper{s,a},solution.Sp{s,a},state_discretization,user_param, original_sys, 0,1)
        end
        % Plot vertices of Post for multiple realizations of the disturbance
        seed = RandStream('mlfg6331_64');
        indices_samples = datasample(1:user_param.Nc, n_samples);
        u = original_sys.Act(a);
        hold on
        for w_ind = 1:n_samples
            w = samples.Xi_hat_c(:, indices_samples(w_ind) );%datasample(samples.Xi_hat_c,1,2);
            % Compute Post_q_a_w
            [a_Post, b_Post, error_flag_post] = compute_reach_simplified_pendulum_fcn(state_discretization.vertices_s{s},u,w,original_sys,user_param);
            % Inflate post to account for clustering
            a_Post = a_Post - [0; 1]*original_sys.Lw_s_a{s,a}*samples.size_cluster( indices_samples(w_ind) );
            b_Post = b_Post + [0; 1]*original_sys.Lw_s_a{s,a}*samples.size_cluster( indices_samples(w_ind) );
            %     [Sp_intersects, error_flag_intersections] = intersections_simplified_pendulum_single_goal_fcn(a_Post, b_Post, state_discretization.Deltax, state_discretization.Deltay, state_discretization.Nx, state_discretization.Ny, user_param.Ns, original_sys.a_X, original_sys.b_X, solver_opts.precision, user_param);
            %Q_intersects
            figure(101)
            hold on
            R = [a_Post(1) a_Post(2); b_Post(1) a_Post(2); b_Post(1) b_Post(2); a_Post(1) b_Post(2); a_Post(1) a_Post(2)]';
            plot(R(1,:), R(2,:), 'g')
        end

        % Take care of the unsafe state
        if (solution.Sp{s,a}(end) == user_param.Ns+1)
            aux_Sp = solution.Sp{s,a}(1:end-1);
        else
            aux_Sp = solution.Sp{s,a};
        end
        % Plot Sp for current state and action
        plot_bounds(ones(size(aux_Sp)),ones(size(aux_Sp)),aux_Sp,state_discretization,user_param, original_sys, 1,0)
        % Plot Reach for qurrent state and action
        w = original_sys.center_samples;
        [a_Post, b_Post, error_flag_post] = compute_reach_simplified_pendulum_fcn(state_discretization.vertices_s{s},u,w,original_sys,user_param);
        a_Post = a_Post - [0; 1]*original_sys.Lw_s_a{s,a}*user_param.c;
        b_Post = b_Post + [0; 1]*original_sys.Lw_s_a{s,a}*user_param.c;
        figure(5)
        hold on
        R = [a_Post(1) a_Post(2); b_Post(1) a_Post(2); b_Post(1) b_Post(2); a_Post(1) b_Post(2); a_Post(1) a_Post(2)]';
        plot(R(1,:), R(2,:), 'b')
    elseif ( strcmp(original_sys.name, 'unicycle2d') )
        % Take care of the unsafe state
        if (solution.Sp{s,a}(end) == user_param.Ns+1)
            % Plot bounds for current state and action
            plot_bounds(solution.P_lower{s,a}(1:end-1),solution.P_upper{s,a}(1:end-1),solution.Sp{s,a}(1:end-1),state_discretization,user_param, original_sys, 0,1)
        else
            % Plot bounds for current state and action
            plot_bounds(solution.P_lower{s,a},solution.P_upper{s,a},solution.Sp{s,a},state_discretization,user_param, original_sys, 0,1)
        end

        % Plot vertices of Post for multiple realizations of the disturbance
        seed = RandStream('mlfg6331_64');
        %indices_samples = datasample(1:user_param.Nc, n_samples);
        u = original_sys.Act(a);
        hold on
        for w_ind = 1:n_samples
            %         w = samples.Xi_hat_c(:, indices_samples(w_ind) );%datasample(samples.Xi_hat_c,1,2);
            w = samples.Xi_hat_c(:, w_ind );%datasample(samples.Xi_hat_c,1,2);
            % Compute Post_q_a_w
            [a_reach, b_reach, error_flag_post] = compute_reach_simplified_fcn(state_discretization.vertices_s{s},u,w,original_sys,user_param);
            % Inflate post to account for clustering
            % Inflate post to account for clustering
            a_reach = a_reach - original_sys.Lw_s_a{s,a}*samples.size_cluster(w_ind);
            b_reach = b_reach + original_sys.Lw_s_a{s,a}*samples.size_cluster(w_ind);
            [Q_intersects, error_flag_intersections] = intersections_simplified_fcn(a_reach, b_reach, state_discretization.Deltax{1},  state_discretization.Deltax{2}, user_param.Nx{1}, user_param.Nx{2}, user_param.Ns, original_sys.state_space_lower_left_corner, original_sys.state_space_upper_right_corner, solver_opts.precision);
            %Q_intersects
            figure(1)
            hold on
            %plot(reach_q_a_w(1,:), reach_q_a_w(2,:),'color',[0.8 0.8 0.8])
            R = [a_reach(1) a_reach(2); b_reach(1) a_reach(2); b_reach(1) b_reach(2); a_reach(1) b_reach(2); a_reach(1) a_reach(2)]';
            plot(R(1,:), R(2,:), 'b')
        end

        %     % Take care of the unsafe state
        %     if (solution.Q_nd{q,a}(end) == user_param.Ns+1)
        %         aux_Q_nd = solution.Q_nd{q,a}(1:end-1);
        %     else
        %         aux_Q_nd = solution.Q_nd{q,a};
        %     end
        %     % Plot Q_nd for current state and action
        %     plot_bounds(ones(size(aux_Q_nd)),ones(size(aux_Q_nd)),aux_Q_nd,state_discretization,user_param, 1,0)
        %     % Plot Reach for qurrent state and action
        %     w = [0; 0];
        %     [Post_q_a_w, ~] = compute_post_q_u_w(state_discretization.vertices_q{q},u,w,original_sys,user_param);
        %     % Compute Reach_nd_q_a
        %     Reach_nd_q_a = compute_Reach_nd_q_u(Post_q_a_w, original_sys.Lw_s_a,user_param.c);
        %     figure(5)
        %     hold on
        %     plot(Reach_nd_q_a(1,:), Reach_nd_q_a(2,:),'b')

    elseif ( strcmp(original_sys.name, 'mult_noise') )
        % Take care of the unsafe state
        if (solution.Sp{s,a}(end) == user_param.Ns+1)
            % Plot bounds for current state and action
            plot_bounds(solution.P_lower{s,a}(1:end-1),solution.P_upper{s,a}(1:end-1),solution.Sp{s,a}(1:end-1),state_discretization,user_param, original_sys, 0,1)
        else
            % Plot bounds for current state and action
            plot_bounds(solution.P_lower{s,a},solution.P_upper{s,a},solution.Sp{s,a},state_discretization,user_param, original_sys, 0,1)
        end

        % Plot vertices of Post for multiple realizations of the disturbance
        seed = RandStream('mlfg6331_64');
        %indices_samples = datasample(1:user_param.Nc, n_samples);
        u = original_sys.Act(a);
        hold on
        for w_ind = 1:n_samples
            %         w = samples.Xi_hat_c(:, indices_samples(w_ind) );%datasample(samples.Xi_hat_c,1,2);
            w = samples.Xi_hat_c(:, w_ind );%datasample(samples.Xi_hat_c,1,2);
            % Compute Post_q_a_w
            [a_reach, b_reach, error_flag_post] = compute_reach_simplified_mult_noise_fcn(state_discretization.vertices_s{s},u,w,original_sys,user_param);
            % Inflate post to account for clustering
            % Inflate post to account for clustering
            a_reach = a_reach - original_sys.Lw_s_a{s,a}*samples.size_cluster(w_ind);
            b_reach = b_reach + original_sys.Lw_s_a{s,a}*samples.size_cluster(w_ind);
            [Q_intersects, error_flag_intersections] = intersections_simplified_fcn(a_reach, b_reach, state_discretization.Deltax,  state_discretization.Deltay, user_param.Nx, user_param.Ny, user_param.Ns, original_sys.state_space_lower_left_corner, original_sys.state_space_upper_right_corner, solver_opts.precision);
            %Q_intersects
            figure(1)
            hold on
            %plot(reach_q_a_w(1,:), reach_q_a_w(2,:),'color',[0.8 0.8 0.8])
            R = [a_reach(1) a_reach(2); b_reach(1) a_reach(2); b_reach(1) b_reach(2); a_reach(1) b_reach(2); a_reach(1) a_reach(2)]';
            plot(R(1,:), R(2,:), 'b')
        end
    end

end

%% Set this dummy variable to false (to compute variables for matrices)

solution.variables_for_matrices_computed = false;

% Set this dummy variable to false (start robust value iteration from the beginning)

solution.warm_start_RDP = false;

%% Solve the maximal reachability probability problem

% Tolerance ov VI (in case that you don't care about convergence, set it to zero)
if ( strcmp( original_sys.name, 'temp' ) )
    solver_opts.tol_VI = -1; % 1e-3;
    user_param.max_iter = 15; % 1000;
    solver_opts.n_states_per_group_RDP = 1;
elseif ( strcmp( original_sys.name, 'fm1' ) || strcmp( original_sys.name, 'fm2' ) || strcmp( original_sys.name, 'lr' ) )
    solver_opts.tol_VI = -1;
    user_param.max_iter = 14;
    solver_opts.n_states_per_group_RDP = 1;
else
    solver_opts.tol_VI = 1e-3;
    user_param.max_iter = 1000;
    solver_opts.n_states_per_group_RDP = 12;
end

% Old approaches (probably don't work)
% solver_opts.value_iteration_method = 'RVI_gurobi_overlapping'; % Interval value iteration
% solver_opts.value_iteration_method = 'RVI_tailored_overlapping'; % Interval value iteration
%solver_opts.value_iteration_method = 'RVI_tailored_overlapping2'; % Interval value iteration
% solver_opts.value_iteration_method = 'RVI_linprog'; % Interval value iteration
% solver_opts.value_iteration_method = 'RVI'; % Robust value iteration

solution = RDP(solution, user_param, solver_opts, original_sys, state_discretization);



%% Set P_upper to 1 everywhere that is not a sink
% 
% compute_P_reach_upper = true;
% 
% if (compute_P_reach_upper)
%     solution.P_reach_upper(user_param.Q_not_sink) = ones(length(user_param.Q_not_sink), 1);
%     solution.e_avg = mean(solution.P_reach_upper - solution.P_reach_lower);
% end


%% Plot results of maximal reachability probability

x3_plane = 5;
x4_plane = 5;

if ( strcmp(original_sys.name, 'temp') )
    plot_bounds_slice_n_dim(solution.P_reach_lower,solution.P_reach_upper,state_discretization,user_param, original_sys, x3_plane, x4_plane, 1,1);
else
    theta_plane = 1;

    if ( ( original_sys.n == 1 ) || ( original_sys.n == 2 ) )
        plot_bounds(solution.P_reach_lower(1:end-1),solution.P_reach_upper(1:end-1),user_param.S(1:end-1),state_discretization,user_param, original_sys, 1,1);
    elseif ( original_sys.n == 3 )
        is_greater_than_zero = (solution.P_reach_lower(1:end-1) > solver_opts.precision);
        plot_bounds_3D(solution.P_reach_lower(is_greater_than_zero),solution.P_reach_lower(is_greater_than_zero), user_param.S(is_greater_than_zero),state_discretization)
        if ( strcmp(original_sys.name, 'unicycle3d_nl_friction') )
            % Plot results of maximal reachability probability (2D plot for fixed theta)
            s_th_plane = (theta_plane-1)*user_param.Nx{1}*user_param.Nx{2}+1;
            fprintf('You are plotting for theta = %d. \n', 180/pi*(2*pi*state_discretization.X_center{3}(s_th_plane) - pi) );
            S_th_plane = s_th_plane:(s_th_plane+user_param.Nx{1}*user_param.Nx{2}-1);
            plot_bounds_slice(solution.P_reach_lower, solution.P_reach_upper, S_th_plane, state_discretization, user_param)
        end
    elseif ( ( original_sys.n == 4 ) && strcmp(original_sys.name, 'unicycle4d_nl_friction') )
        is_greater_than_zero = (solution.P_reach_lower(1:user_param.Nx{1}*user_param.Nx{2}*user_param.Nx{3}) > solver_opts.precision);
        plot_bounds_4D(solution.P_reach_lower(is_greater_than_zero), solution.P_reach_lower(is_greater_than_zero), user_param.S(is_greater_than_zero),state_discretization)
        %     if ( strcmp(original_sys.name, 'unicycle3d_nl_friction') )
        %         % Plot results of maximal reachability probability (2D plot for fixed theta)
        %         s_th_plane = (theta_plane-1)*user_param.Nx*user_param.Ny+1;
        %         fprintf('You are plotting for theta = %d. \n', 180/pi*(2*pi*state_discretization.Z_center(s_th_plane) - pi) );
        %         S_th_plane = s_th_plane:(s_th_plane+user_param.Nx*user_param.Nx-1);
        %         plot_bounds_slice(solution.P_reach_lower, solution.P_reach_upper, S_th_plane, state_discretization, user_param)
        %     end
    end
end

%% Plot results of maximal reachability probability (2D plot with max over theta)

if ( strcmp( original_sys.name, 'unicycle3d_nl_friction' ) )
    P_reach_lower_max_th = zeros( size(solution.P_reach_lower(S_th_plane)) );
    P_reach_upper_max_th = zeros( size(solution.P_reach_lower(S_th_plane)) );
    for theta_plane = 1:user_param.Nx{3}

        s_th_plane = (theta_plane-1)*user_param.Nx{1}*user_param.Nx{2}+1;

        S_th_plane = s_th_plane:(s_th_plane+user_param.Nx{1}*user_param.Nx{2}-1);

        P_reach_lower_max_th = max( [P_reach_lower_max_th, solution.P_reach_lower(S_th_plane)], [], 2);
        P_reach_upper_max_th = max( [P_reach_upper_max_th, solution.P_reach_upper(S_th_plane)], [], 2);

    end

    aux_S_th_plane = 1:1:length(S_th_plane);

    plot_bounds_slice(P_reach_lower_max_th, P_reach_upper_max_th,aux_S_th_plane , state_discretization, user_param)
elseif ( strcmp( original_sys.name, 'unicycle4d_nl_friction' ) )
    P_reach_lower_max_th_vel = zeros( user_param.Nx{1}*user_param.Nx{2}, 1 );
    P_reach_upper_max_th_vel = zeros( user_param.Nx{1}*user_param.Nx{2}, 1 );
    for v_block = 1:user_param.Nx{4}
        for theta_plane = 1:user_param.Nx{3}

            s_th_plane = (v_block - 1)*(theta_plane - 1)*user_param.Nx{1}*user_param.Nx{2} + 1;

            S_th_plane = s_th_plane:(s_th_plane + user_param.Nx{1}*user_param.Nx{2} - 1);

            P_reach_lower_max_th_vel = max( [P_reach_lower_max_th_vel, solution.P_reach_lower(S_th_plane)], [], 2);
            P_reach_upper_max_th_vel = max( [P_reach_upper_max_th_vel, solution.P_reach_upper(S_th_plane)], [], 2);

        end
    end

    aux_S_th_plane = 1:1:length(S_th_plane);

    plot_bounds_slice_4d(P_reach_lower_max_th_vel, P_reach_upper_max_th_vel,aux_S_th_plane , state_discretization, user_param)
end

%% Refine strategy

if ( isfield(solution, 'sigma_C') )
    solution = rmfield( solution, 'sigma_C' );
end
if ( ~strcmp( original_sys.name, 'temp' ) )
    solution.sigma_C = zeros(size(original_sys.Act,1), user_param.Ns);
    for s = user_param.S_not_terminal
        if ( ~user_param.controller_given_yn )
            solution.sigma_C(:, s) = original_sys.Act(:, solution.sigma(s));
        else
            solution.sigma_C(:, s) = original_sys.Act(:, solution.controller_given(s) );
        end
    end
else
    solution.sigma_C = zeros(size(original_sys.Act,1), user_param.max_iter, user_param.Ns);
    for i = 1:user_param.max_iter
        for s = user_param.S_not_terminal
            if ( ~user_param.controller_given_yn )
                solution.sigma_C(:, i, s) = original_sys.Act(:, solution.sigma(i, s));
            else
                solution.sigma_C(:, i, s) = original_sys.Act(:, solution.controller_given(1, s) );
            end
        end
    end
end


%% Validate results via Monte Carlo

user_param.MC_runs = 1000;
user_param.N_initial_states_MC_runs = 100;
user_param.MC_timeout = 100;
solver_opts.MC_plot = false;
solver_opts.reuse_x0_vect_yn = true;

variables_for_matrices = [];
if ( ~solver_opts.reuse_x0_vect_yn )
    x0_vect = [];
end
%user_param.s0_vect = [ceil(user_param.Nx{1}*(user_param.Nx{2}/2 - 5) - user_param.Nx*5/2 - 2) user_param.Ns - ceil(user_param.Nx*(user_param.Ny/2 - 5) - user_param.Nx*3/2 - 4)];
[solution, x0_vect] = simulate_traj_fcn(original_sys, state_discretization, user_param, solver_opts, solution, variables_for_matrices, x0_vect);

%% Plot Std ellipses

user_param.N_initial_states_MC_runs = 1;
user_param.MC_runs = 10;
user_param.MC_timeout = 100;
user_param.N_MC = 100; % Number of samples used for MC simulations
solver_opts.MC_plot = 1;
nStdDevs = 2;
% Initial state
q0_vect = datasample(user_param.Q_not_sink, user_param.N_initial_states_MC_runs);

[samples1] = generate_samples(original_sys,user_param, 0);
for i = 1:length(q0_vect)
    q0 = q0_vect(i);
    P_reach_emp = plotMCEllipses(state_discretization, user_param, solver_opts, original_sys, samples1.Xi_hat, q0, nStdDevs, solution, variables_for_matrices, original_sys.Act);
end

