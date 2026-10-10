function [x_MC, MC_flag, wraps] = MC_run(sys, x0, state_discretization,user_param,sigma_C,original_sys,Xi_hat_new)
% Simulate system via Monte Carlo

Ns = user_param.Ns;
S_target = user_param.S_target;
if ( isfield(user_param, 'S_obstacle') )
    S_obstacle = user_param.S_obstacle;
elseif ( isfield(user_param, 'S_obstacles') )
    S_obstacle = user_param.S_obstacles;
end
MC_timeout = user_param.MC_timeout;

x_MC = x0;
wraps = false;
for t = 1:MC_timeout
    x = x_MC(:,end);
    s = J_function(x, state_discretization, user_param, original_sys);
    if ( any( ismember(S_target, s) ) ) % If state q \in Q_target
        MC_flag = 'reach';
        break;
    elseif ( (s == Ns + 1) || any(ismember(S_obstacle, s)) )
        MC_flag = 'no reach';
        break;
    end
    u = sigma_C(:, s);
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
end

if (t == MC_timeout)
    MC_flag = 'no reach';
    disp('MC timeout...')
end

end
