function [original_sys, user_param, solution] = discretize_action_set_fcn(original_sys, user_param, state_discretization, plot_one_step_ahead_yn, s)

if ( strcmp(original_sys.name, 'unicycle3d_nl_friction') )
    original_sys.Act = [user_param.Umin_vel*ones(1,user_param.Na_th) user_param.Umax_vel*ones(1,user_param.Na_th)
        user_param.Umax_th*linspace(-1,1,user_param.Na_th) user_param.Umax_th*linspace(-1,1,user_param.Na_th)];
elseif ( strcmp(original_sys.name, 'unicycle2d') )
    original_sys.Act = user_param.Umax_th*linspace(-1,1,user_param.Na_th + 1);
    original_sys.Act = original_sys.Act(:,1:end-1);
elseif ( strcmp(original_sys.name, 'pendulum') )
    original_sys.Act = user_param.Umax*linspace(-1,1,user_param.Na);
elseif ( strcmp(original_sys.name, 'mult_noise') )
    original_sys.Act = 0;
elseif ( strcmp(original_sys.name, 'unicycle2d_ltlf') )
    if ( isfield(user_param, 'Umin_vel') )
        original_sys.Act = [user_param.Umin_vel*ones(1,user_param.Na_th) user_param.Umax_vel*ones(1,user_param.Na_th)
            user_param.Umax_th*linspace(-1,1,user_param.Na_th + 1) user_param.Umax_th*linspace(-1,1,user_param.Na_th + 1)];
    else
        original_sys.Act = user_param.Umax_th*linspace(-1,1,user_param.Na_th + 1);
    end
    original_sys.Act = original_sys.Act(:,1:end-1);
elseif ( strcmp(original_sys.name, 'unicycle4d_nl_friction') )
    aux_yaw = user_param.Umax(1)*linspace(-1, 1, user_param.N_yaw_rate);
    aux_acc = user_param.Umax(2)*linspace(-1, 1, user_param.N_lin_acc);
    [Aux_yaw, Aux_acc] = meshgrid(aux_yaw, aux_acc);
    original_sys.Act = [Aux_yaw(:), Aux_acc(:)]';
elseif ( strcmp(original_sys.name, 'temp') )
    original_sys.Act = ff2n(original_sys.n)';
elseif ( strcmp(original_sys.name, 'fm1') || strcmp(original_sys.name, 'fm2') || strcmp(original_sys.name, 'lr') )
    original_sys.Act = [0, 1];
end

if ( user_param.controller_given_yn )
    load('controller_table.mat', 'controller');
    solution.controller_given = controller;
    disp('Controller provided: verification...')
    user_param.Na = 1;
else
    disp('Controller not provided: synthesis...')
    solution.controller_given = [];
    user_param.Na = size(original_sys.Act, 2);
end


% (Just for debugging purposes) Plot one-step-ahead dynamics for several actions
if ( plot_one_step_ahead_yn )
    for i = 1:16
        s = ceil(1 + (user_param.Ns - 1).* rand(1));
        plot_one_step_ahead(s, state_discretization, original_sys.Act, user_param, original_sys, solution);
    end
end

end