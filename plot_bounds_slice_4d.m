function [] = plot_bounds_slice_4d(P_lower,P_upper,support_qp,state_discretization,user_param)
%Plot bounds

% Nx = user_param.Nx;
% Ny = user_param.Ny;

X_center = state_discretization.X_center{1};
Y_center = state_discretization.X_center{2};

n = 500;
color1 = [0 0 0];%[0.6350 0.0780 0.1840]; % black
color2 = [0.7 0 0];%[0.9290 0.6940 0.1250]; % red
color3 = [0.6 0 0.6];%[0.9290 0.6940 0.1250]; % purple
color4 = [0.1 0.5 0.8];%[0.4660 0.6740 0.1880]; % blue
color5 = [1 1 1];%[0.4660 0.6740 0.1880]; % white

R = [linspace(color1(1),color2(1),n/5) linspace(color2(1),color3(1),n/5) linspace(color3(1),color4(1),n/5) linspace(color4(1),color5(1),n/5)];
G = [linspace(color1(2),color2(2),n/5) linspace(color2(2),color3(2),n/5) linspace(color3(2),color4(2),n/5) linspace(color4(2),color5(2),n/5)];
B = [linspace(color1(3),color2(3),n/5) linspace(color2(3),color3(3),n/5) linspace(color3(3),color4(3),n/5) linspace(color4(3),color5(3),n/5)];

f2 = figure(102);
imagesc(X_center(support_qp),Y_center(support_qp),reshape(P_lower(support_qp),[user_param.Nx{1},user_param.Nx{2}])')
set(gca,'YDir','normal')
colormap( [R(:), G(:), B(:)] );
set(gca,'FontSize',20)
colorbar
caxis([0 1]);
hold on
xlabel('$x_1$','Interpreter','latex')
ylabel('$x_2$','Interpreter','latex')
title('$\underline p(x)$','Interpreter','latex')
set(gca,'xtick',[0:0.2:1])

% a_goal = [state_discretization.X_center{1}(user_param.move_S_target+1)-state_discretization.Deltax{1}/2, state_discretization.Y_center(user_param.move_S_target+1)-state_discretization.Deltay/2];
% width_goal = [user_param.Nx_S_target*state_discretization.Deltax, user_param.Nx_S_target*state_discretization.Deltay];
% a_obstacle = [state_discretization.X_center(user_param.move_S_obstacle+2)-state_discretization.Deltax/2, state_discretization.Y_center(user_param.move_S_obstacle+2)-state_discretization.Deltay/2];
% width_obstacle = [user_param.Nx_S_obstacle*state_discretization.Deltax, user_param.Ny_S_obstacle*state_discretization.Deltay];
% 
% rectangle('position',[a_goal(1) a_goal(2) width_goal(1) width_goal(2)], 'FaceColor', [R(end), G(end), B(end)],'EdgeColor',[0 0 0] ,'LineWidth',2) % Goal
% rectangle('position',[a_obstacle(1) a_obstacle(2) width_obstacle(1) width_obstacle(2)], 'FaceColor', [R(1), G(1), B(1)],'EdgeColor', [0 0 0],'LineWidth',2) % Obstacle


f3 = figure(3);
imagesc(X_center(support_qp),Y_center(support_qp),reshape(P_upper(support_qp),[user_param.Nx{1},user_param.Nx{2}])')
set(gca,'YDir','normal')
colormap( [R(:), G(:), B(:)] );
set(gca,'FontSize',20)
colorbar
caxis([0 1]);
hold on
title('$$\overline{P}(q,a,\cdot)$$','interpreter','latex')

% rectangle('position',[a_goal(1) a_goal(2) width_goal(1) width_goal(2)], 'FaceColor', [R(end), G(end), B(end)],'EdgeColor',[0 0 0] ,'LineWidth',2) % Goal
% rectangle('position',[a_obstacle(1) a_obstacle(2) width_obstacle(1) width_obstacle(2)], 'FaceColor', [R(1), G(1), B(1)],'EdgeColor', [1 1 1],'LineWidth',2) % Obstacle

movegui(f2,'northwest');
movegui(f3,'northeast');
figure(2)
figure(3)

end

