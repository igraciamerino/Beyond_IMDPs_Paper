function [] = plot_disturbance_discretization_uniform(disturbance_discretization, plot_grid)

xd_grid = disturbance_discretization.xd_grid;
yd_grid = disturbance_discretization.yd_grid;
Xd_center = disturbance_discretization.Xd_center;
Yd_center = disturbance_discretization.Yd_center;
Nxd = length(xd_grid)-1;
Nyd = length(yd_grid)-1;

figure(2)
plot(Xd_center, Yd_center, 'Ob')
hold on
if (plot_grid)
    % Plot lines for fixed(x)
    for i = 1:Nxd+1
        plot(xd_grid(i)*ones(1,2), [yd_grid(1) yd_grid(end)],'k');
    end
    % Plot lines for fixed(y)
    for j = 1:Nyd+1
        plot([xd_grid(1) xd_grid(end)], yd_grid(j)*ones(1,2),'k');
    end
end

end
