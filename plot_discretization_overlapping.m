function [] = plot_discretization_overlapping(state_discretization, original_sys, plot_grid)

if ( original_sys.n == 2)

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

    % Coarse

    % X_center = state_discretization.X_center_coarse;
    % Y_center = state_discretization.Y_center_coarse;
    %
    % figure(101)
    % plot(X_center, Y_center, 'Ob')
    % hold on
    % if (plot_grid)
    %     for q = 1:(Nx-2)*(Ny-2)
    %         vs = state_discretization.vertices_q_coarse{q};
    %         plot(vs(1, :), vs(2, :) ,'k');
    %     end
    % end

elseif ( original_sys.n == 3)

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

    % % Coarse
    %
%     X_center_coarse = state_discretization.X_center_coarse;
%     Y_center_coarse = state_discretization.Y_center_coarse;
%     Z_center_coarse = state_discretization.Z_center_coarse;
% 
%     figure(101)
%     plot3(X_center_coarse, Y_center_coarse, Z_center_coarse, 'Ob')
%     hold on

end


end