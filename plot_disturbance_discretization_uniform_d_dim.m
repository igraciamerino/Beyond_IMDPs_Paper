function [] = plot_disturbance_discretization_uniform_d_dim(disturbance_discretization, user_param, plot_grid, original_sys)

if ( ( original_sys.d == 2 ) || ( original_sys.d == 4 ) )
    Nxd = user_param.Nxd{1};
    Nyd = user_param.Nxd{2};
    xd_grid = disturbance_discretization.xd_grid{1};
    yd_grid = disturbance_discretization.xd_grid{2};
    Xd_center = disturbance_discretization.Xd_center{1};
    Yd_center = disturbance_discretization.Xd_center{2};

    figure(2)
    plot(Xd_center, Yd_center, 'Ob')
    hold on
    if (plot_grid)
        % Plot lines for fixed(x)
        for i = 1:(Nxd+1)
            plot(xd_grid(i)*ones(1,2), [yd_grid(1) yd_grid(end)],'k');
        end
        % Plot lines for fixed(y)
        for j = 1:(Nyd+1)
            plot([xd_grid(1) xd_grid(end)], yd_grid(j)*ones(1,2),'k');
        end
    end

elseif ( original_sys.d == 3 )
    Nxd = user_param.Nxd{1};
    Nyd = user_param.Nxd{2};
    Nzd = user_param.Nxd{3};
    xd_grid = disturbance_discretization.xd_grid{1};
    yd_grid = disturbance_discretization.xd_grid{2};
    zd_grid = disturbance_discretization.xd_grid{3};
    Xd_center = disturbance_discretization.Xd_center{1};
    Yd_center = disturbance_discretization.Xd_center{2};
    Zd_center = disturbance_discretization.Xd_center{3};

    figure(2)
    plot3(Xd_center, Yd_center, Zd_center, 'Ob')
    hold on
    if (plot_grid)

        % Plot lines for fixed(x,y)
        for i = 1:Nxd+1
            for j = 1:Nyd+1
                plot3(xd_grid(i)*ones(1,2), yd_grid(j)*ones(1,2), [zd_grid(1) zd_grid(end)],'k');
            end
        end
        % Plot lines for fixed(x,th)
        for i = 1:Nxd+1
            for k = 1:Nzd+1
                plot3(xd_grid(i)*ones(1,2), [yd_grid(1) yd_grid(end)], zd_grid(k)*ones(1,2),'k');
            end
        end
        % Plot lines for fixed(y,th)
        for j = 1:Nyd+1
            for k = 1:Nzd+1
                plot3([xd_grid(1) xd_grid(end)], yd_grid(j)*ones(1,2), zd_grid(k)*ones(1,2),'k');
            end
        end

    end

end
end
