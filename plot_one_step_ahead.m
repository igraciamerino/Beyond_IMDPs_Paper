function [] = plot_one_step_ahead(s,state_discretization,Act,user_param,original_sys, solution)
% Plots the deterministic part of the one-step-ahead dynamics

Na = user_param.Na;
name = original_sys.name;
figure(1)
for a = 1:Na
    w = zeros(original_sys.d, 1);
    if ( ~user_param.controller_given_yn )
        u = Act(:,a);
    else
        u = Act(:, solution.controller_given(s) );
    end
    if ( strcmp(name, 'unicycle2d') )
        X_center = state_discretization.X_center{1};
        Y_center = state_discretization.X_center{2};
        x = [X_center(s) Y_center(s)]';
        x_star_deterministic = unicycle2D(x, u, w, original_sys, user_param);
        plot([x(1) x_star_deterministic(1)], [x(2) x_star_deterministic(2)], 'rs-')
    elseif ( strcmp(name, 'unicycle3d_nl_friction') )
        X_center = state_discretization.X_center{1};
        Y_center = state_discretization.X_center{2};
        Z_center = state_discretization.X_center{3};
        x = [X_center(s) Y_center(s) Z_center(s)]';
        x_star_deterministic = unicycle3d_nl_friction(x, u, w, original_sys, user_param);
        plot3([x(1) x_star_deterministic(1)], [x(2) x_star_deterministic(2)], [x(3) x_star_deterministic(3)], 'rs-')
    elseif ( strcmp(name, 'unicycle4d_nl_friction') )
        X_center = state_discretization.X_center;
        x = [X_center{1}(s) X_center{2}(s) X_center{3}(s) X_center{4}(s)]';
        x_star_deterministic = unicycle4d_nl_friction(x, u, w, original_sys, user_param);
        plot3([x(1) x_star_deterministic(1)], [x(2) x_star_deterministic(2)], [x(3) x_star_deterministic(3)], 'rs-')
    elseif ( strcmp(name, 'pendulum') )
        X_center = state_discretization.X_center;
        Y_center = state_discretization.Y_center;
        x = [X_center(s) Y_center(s)]';
        x_star_deterministic = pendulum(x, u, w, original_sys, user_param);
        plot([x(1) x_star_deterministic(1)], [x(2) x_star_deterministic(2)], 'rs-')
    elseif ( strcmp(name, 'unicycle2d_ltlf') )
        X_center = state_discretization.X_center;
        Y_center = state_discretization.Y_center;
        x = [X_center(s) Y_center(s)]';
        x_star_deterministic = unicycle2D_ltlf(x, u, w, original_sys, user_param);
        plot([x(1) x_star_deterministic(1)], [x(2) x_star_deterministic(2)], 'rs-')
    elseif ( strcmp(name, 'temp') )
        X_center = state_discretization.X_center;
        if ( original_sys.n == 2 )
            x = [X_center{1}(s) X_center{2}(s)]';
            x_star_deterministic = temp_sys(x, u, w, original_sys, user_param);
            plot([x(1) x_star_deterministic(1)], [x(2) x_star_deterministic(2)], 'rs-')
        elseif ( original_sys.n == 3 )
            x = [X_center{1}(s) X_center{2}(s) X_center{3}(s)]';
            x_star_deterministic = temp_sys(x, u, w, original_sys, user_param);
            plot3([x(1) x_star_deterministic(1)], [x(2) x_star_deterministic(2)], [x(3) x_star_deterministic(3)], 'rs-')
        elseif ( original_sys.n == 4 )
            x = [X_center{1}(s) X_center{2}(s) X_center{3}(s) X_center{4}(s)]';
            x_star_deterministic = temp_sys(x, u, w, original_sys, user_param);
            plot([x(1) x_star_deterministic(1)], [x(2) x_star_deterministic(2)], 'rs-')
        end
    elseif ( strcmp(name, 'fm1') || strcmp(name, 'lr') )
        X_center = state_discretization.X_center{1};
        x = X_center(s);
        d_nd = 0;
        x_star_deterministic = fm1_sys(x, u, w, d_nd, original_sys, user_param);
        plot([x(1) x_star_deterministic(1)], [0 0], 'rs-')
    elseif ( strcmp(name, 'fm2') )
        X_center = state_discretization.X_center{1};
        Y_center = state_discretization.X_center{2};
        x = [X_center(s); Y_center(s)];
        x_star_deterministic = fm2_sys(x, u, w, original_sys, user_param);
        plot([x(1) x_star_deterministic(1)], [x(2) x_star_deterministic(2)], 'rs-')
    end
end

end

