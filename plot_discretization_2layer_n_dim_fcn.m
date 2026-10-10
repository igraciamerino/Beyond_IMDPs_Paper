function [] = plot_discretization_IMDP_n_dim_fcn(state_discretization, user_param, plot_grid, original_sys)

if ( ( original_sys.n == 2 ) || ( original_sys.n == 4 ) )
    Nx = user_param.Nx{1};
    Ny = user_param.Nx{2};
    x_grid = state_discretization.x_grid{1};
    y_grid = state_discretization.x_grid{2};
    X_center = state_discretization.X_center{1};
    Y_center = state_discretization.X_center{2};

    figure(1)
    plot(X_center, Y_center, 'Ob')
    hold on
    if (plot_grid)
        % Plot lines for fixed(x)
        for i = 1:(Nx+1)
            plot(x_grid(i)*ones(1,2), [y_grid(1) y_grid(end)],'k');
        end
        % Plot lines for fixed(y)
        for j = 1:(Ny+1)
            plot([x_grid(1) x_grid(end)], y_grid(j)*ones(1,2),'k');
        end
    end

    % Coarse

    Nx_coarse = user_param.Nx_coarse{1};
    Ny_coarse = user_param.Nx_coarse{2};
    x_grid_coarse = state_discretization.x_grid_coarse{1};
    y_grid_coarse = state_discretization.x_grid_coarse{2};
    X_center_coarse = state_discretization.X_center_coarse{1};
    Y_center_coarse = state_discretization.X_center_coarse{2};

    figure(1)
    plot(X_center_coarse, Y_center_coarse, 'Ob')
    hold on
    if (plot_grid)
        % Plot lines for fixed(x)
        for i = 1:(Nx_coarse+1)
            plot(x_grid_coarse(i)*ones(1,2), [y_grid_coarse(1) y_grid_coarse(end)],'k');
        end
        % Plot lines for fixed(y)
        for j = 1:(Ny_coarse+1)
            plot([x_grid_coarse(1) x_grid_coarse(end)], y_grid_coarse(j)*ones(1,2),'k');
        end
    end

elseif ( original_sys.n == 3 )
    Nx_coarse = user_param.Nx_coarse{1};
    Ny_coarse = user_param.Nx_coarse{2};
    Nz_coarse = user_param.Nx_coarse{3};
    x_grid_coarse = state_discretization.x_grid_coarse{1};
    y_grid_coarse = state_discretization.x_grid_coarse{2};
    z_grid_coarse = state_discretization.x_grid_coarse{3};
    X_center_coarse = state_discretization.X_center{1};
    Y_center_coarse = state_discretization.X_center_coarse{2};
    Z_center_coarse = state_discretization.X_center_coarse{3};

    figure(1)
    plot3(X_center_coarse, Y_center_coarse, Z_center_coarse, 'Ob')
    hold on
    if (plot_grid)

        % Plot lines for fixed(x,y)
        for i = 1:Nx_coarse+1
            for j = 1:Ny_coarse+1
                plot3(x_grid_coarse(i)*ones(1,2), y_grid_coarse(j)*ones(1,2), [z_grid_coarse(1) z_grid_coarse(end)],'k');
            end
        end
        % Plot lines for fixed(x,th)
        for i = 1:Nx_coarse+1
            for k = 1:Nz_coarse+1
                plot3(x_grid_coarse(i)*ones(1,2), [y_grid_coarse(1) y_grid_coarse(end)], z_grid(k)*ones(1,2),'k');
            end
        end
        % Plot lines for fixed(y,th)
        for j = 1:Ny_coarse+1
            for k = 1:Nz_coarse+1
                plot3([x_grid_coarse(1) x_grid_coarse(end)], y_grid_coarse(j)*ones(1,2), z_grid_coarse(k)*ones(1,2),'k');
            end
        end

    end

end

end
