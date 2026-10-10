function [] = plot_discretization_temp_fcn(state_discretization, user_param, plot_grid)

Nx = user_param.Nx{1};
Ny = user_param.Nx{2};
x_grid = state_discretization.X_grid{1}(1:(Nx+1)*(Ny+1));
y_grid = state_discretization.X_grid{2}(1:(Nx+1)*(Ny+1));
X_center = state_discretization.X_center{1}(1:Nx*Ny);
Y_center = state_discretization.X_center{2}(1:Ny*Ny);

figure(1)
plot(X_center, Y_center, 'Ob')
hold on
if (plot_grid)
    % Plot lines for fixed(x)
    for i = 1:(Nx+1)*(Ny+1)
        plot(x_grid(i)*ones(1,2), [y_grid(1) y_grid(end)],'k');
    end
    % Plot lines for fixed(y)
    for j = 1:(Nx+1)*(Ny+1)
        plot([x_grid(1) x_grid(end)], y_grid(j)*ones(1,2),'k');
    end
end

% % Coarse
% 
% X_center = state_discretization.T1_center_coarse;
% Y_center = state_discretization.T2_center_coarse;
% 
% figure(101)
% plot(X_center, Y_center, 'Ob')
% hold on
% if (plot_grid)
%     for q = 1:length(X_center)
%         vs = state_discretization.vertices_q_coarse{q};
%         plot(vs(1, :), vs(2, :) ,'k');
%     end
% end

end
