function [x_MC, MC_flag] = MC_run_temp(sys, x0, state_discretization,user_param,sigma_C,original_sys,Xi_hat_new)
% Simulate system via Monte Carlo

Ns = user_param.Ns;
if ( isfield(user_param, 'S_target') )
    S_target = user_param.S_target;
else
    S_target = [];
end
if ( isfield(user_param, 'S_obstacle') )
    S_obstacle = user_param.S_obstacle;
elseif ( isfield(user_param, 'S_obstacles') )
    S_obstacle = user_param.S_obstacles;
else
    S_obstacle = [];
end
MC_timeout = user_param.MC_timeout;

x_MC = x0;
for t = 1:MC_timeout
    x = x_MC(:,end);
    s = J_function(x, state_discretization, user_param, original_sys);
    if ( any( ismember(S_target, s) ) && ~strcmp( original_sys.name, 'temp' ) ) % If state q \in Q_target
        MC_flag = 'reach';
        break;
    elseif ( (s == Ns + 1) || ( any(ismember(S_obstacle, s)) && ~strcmp( original_sys.name, 'temp' )  ) )
        MC_flag = 'no reach';
        break;
    elseif ( strcmp( original_sys.name, 'temp' ) && (t == user_param.max_iter + 1) )
        break;
    end
    if ( ~strcmp( original_sys.name, 'temp' ) )
        u = sigma_C(:, s);
    else
        i = user_param.max_iter - t + 1;
        u = sigma_C(:, i, s);
    end
    w = datasample(Xi_hat_new',1)';
    x_star = sys(x,u,w);
    if ( strcmp( original_sys.name, 'unicycle2D' ) )
        x_MC = [x_MC x_star];
    elseif ( strcmp( original_sys.name, 'unicycle3d_nl_friction' ) )
        if (x_star(3) < original_sys.state_space_lower_left_corner(3)) %Wraps on the left
            x_star(3) = x_star(3) + original_sys.state_space_upper_right_corner(3) - original_sys.state_space_lower_left_corner(3);
            x_MC = [x_MC x_star];
%             wraps = true;
        elseif (x_star(3) > original_sys.state_space_upper_right_corner(3)) %Wraps on the right
            x_star(3) = x_star(3) - original_sys.state_space_upper_right_corner(3) + original_sys.state_space_lower_left_corner(3);
            x_MC = [x_MC x_star];
%             wraps = true;
        else
            x_MC = [x_MC x_star];
        end
    elseif ( strcmp( original_sys.name, 'pendulum' ) )
        if (x_star(1) < original_sys.state_space_lower_left_corner(1)) %Wraps on the left
            x_star(1) = x_star(1) + original_sys.state_space_upper_right_corner(1) - original_sys.state_space_lower_left_corner(1);
            x_MC = [x_MC x_star];
            wraps = true;
        elseif (x_star(1) > original_sys.state_space_upper_right_corner(1)) %Wraps on the right
            x_star(1) = x_star(1) - original_sys.state_space_upper_right_corner(1) + original_sys.state_space_lower_left_corner(1);
            x_MC = [x_MC x_star];
            wraps = true;
        else
            x_MC = [x_MC x_star];
        end
    else
        x_MC = [x_MC x_star];
    end
end

if ( strcmp(original_sys.name, 'pendulum') )
    x_MC(1, :) = 2*pi*x_MC(1, :) - pi;
    x_MC(2, :) = 2*user_param.th_dot_max*x_MC(2, :) - user_param.th_dot_max;
elseif ( strcmp( original_sys.name, 'temp' ) )
    x_MC = x_MC(:, 1:user_param.max_iter);
    MC_flag = 'reach';
end

if (t == MC_timeout)
    MC_flag = 'no reach';
    disp('MC timeout...')
end

end
