function [] = plot_discretization_2layer(state_discretization, plot_grid)

if ( ~isfield(state_discretization, 'Z_center') )
    x_grid = state_discretization.x_grid;
    y_grid = state_discretization.y_grid;
    X_center = state_discretization.X_center;
    Y_center = state_discretization.Y_center;
    Nx = length(x_grid)-1;
    Ny = length(y_grid)-1;

    figure(1)
    plot(X_center, Y_center, 'Ob')
    hold on
    if (plot_grid)
        % Plot lines for fixed(x)
        for i = 1:Nx+1
            plot(x_grid(i)*ones(1,2), [y_grid(1) y_grid(end)],'k');
        end
        % Plot lines for fixed(y)
        for j = 1:Ny+1
            plot([x_grid(1) x_grid(end)], y_grid(j)*ones(1,2),'k');
        end
    end

else
    x_grid = state_discretization.x_grid;
    y_grid = state_discretization.y_grid;
    z_grid = state_discretization.z_grid;
    X_center = state_discretization.X_center;
    Y_center = state_discretization.Y_center;
    Z_center = state_discretization.Z_center;
    Nx = length(x_grid)-1;
    Ny = length(y_grid)-1;
    Nz = length(z_grid)-1;

    figure(1)
    plot3(X_center, Y_center, Z_center, 'Ob')
    hold on
    if (plot_grid)

        % Plot lines for fixed(x,y)
        for i = 1:Nx+1
            for j = 1:Ny+1
                plot3(x_grid(i)*ones(1,2), y_grid(j)*ones(1,2), [z_grid(1) z_grid(end)],'k');
            end
        end
        % Plot lines for fixed(x,th)
        for i = 1:Nx+1
            for k = 1:Nz+1
                plot3(x_grid(i)*ones(1,2), [y_grid(1) y_grid(end)], z_grid(k)*ones(1,2),'k');
            end
        end
        % Plot lines for fixed(y,th)
        for j = 1:Ny+1
            for k = 1:Nz+1
                plot3([x_grid(1) x_grid(end)], y_grid(j)*ones(1,2), z_grid(k)*ones(1,2),'k');
            end
        end

    end
end

% Coarse
if ( isfield( state_discretization, 'x_grid_coarse' ) )

    if ( ~isfield(state_discretization, 'Z_center') )
        x_grid = state_discretization.x_grid_coarse;
        y_grid = state_discretization.y_grid_coarse;
        X_center = state_discretization.X_center_coarse;
        Y_center = state_discretization.Y_center_coarse;
        Nx = length(x_grid)-1;
        Ny = length(y_grid)-1;

        figure(101)
        plot(X_center, Y_center, 'Ob')
        hold on
        if (plot_grid)
            % Plot lines for fixed(x)
            for i = 1:Nx+1
                plot(x_grid(i)*ones(1,2), [y_grid(1) y_grid(end)],'k');
            end
            % Plot lines for fixed(y)
            for j = 1:Ny+1
                plot([x_grid(1) x_grid(end)], y_grid(j)*ones(1,2),'k');
            end
        end
    else
    	x_grid_coarse = state_discretization.x_grid_coarse;
        y_grid_coarse = state_discretization.y_grid_coarse;
        z_grid_coarse = state_discretization.z_grid_coarse;
        X_center_coarse = state_discretization.X_center_coarse;
        Y_center_coarse = state_discretization.Y_center_coarse;
        Z_center_coarse = state_discretization.Z_center_coarse;
        Nx_coarse = length(x_grid_coarse)-1;
        Ny_coarse = length(y_grid_coarse)-1;
        Nz_coarse = length(z_grid_coarse)-1;

        figure(101)
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
                    plot3(x_grid_coarse(i)*ones(1,2), [y_grid_coarse(1) y_grid_coarse(end)], z_grid_coarse(k)*ones(1,2),'k');
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

end
