function [] = plot_bounds(P_lower,P_upper,support_P,state_discretization,user_param, original_sys, left, right)
%Plot bounds

Nx = user_param.Nx{1};
X_center = state_discretization.X_center{1};
if (original_sys.n == 2)
    Ny = user_param.Nx{2};
    Y_center = state_discretization.X_center{2};
end

n = 1000;
color1 = [0 0 0];%[0.6350 0.0780 0.1840]; % black
color2 = [0.7 0 0];%[0.9290 0.6940 0.1250]; % red
color3 = [0.6 0 0.6];%[0.9290 0.6940 0.1250]; % purple
color4 = [0.1 0.5 0.8];%[0.4660 0.6740 0.1880]; % blue
color5 = [1 1 1];%[0.4660 0.6740 0.1880]; % white

R = [linspace(color1(1),color2(1),n/4) linspace(color2(1),color3(1),n/4) linspace(color3(1),color4(1),n/4) linspace(color4(1),color5(1),n/4)];
G = [linspace(color1(2),color2(2),n/4) linspace(color2(2),color3(2),n/4) linspace(color3(2),color4(2),n/4) linspace(color4(2),color5(2),n/4)];
B = [linspace(color1(3),color2(3),n/4) linspace(color2(3),color3(3),n/4) linspace(color3(3),color4(3),n/4) linspace(color4(3),color5(3),n/4)];

aux_P_lower = zeros(1,user_param.Ns);
aux_P_upper = zeros(1,user_param.Ns);
for sp_ind = 1:length(support_P)
    qp = support_P(sp_ind);
    aux_P_lower(qp) = P_lower(sp_ind);
    aux_P_upper(qp) = P_upper(sp_ind);
end

f5 = figure(5);
if (left)
    
    if ( strcmp(original_sys.name, 'pendulum') )
        imagesc(2*pi*X_center - pi, 2*user_param.th_dot_max*Y_center - user_param.th_dot_max, reshape(aux_P_lower,[Nx,Ny])')
        set(gca,'YDir','normal')
        colormap( [R(:), G(:), B(:)] );
        colorbar
        caxis([0 1]);
        set(gca,'FontSize',20)
        hold on
        title('$\underline p(x)$','Interpreter','latex')
        xlabel('$\theta$','Interpreter','latex')
        ylabel('$\dot\theta$','Interpreter','latex')
        set(get(gca,'YLabel'),'Rotation',0)
        set(gca,'FontSize',20)
        set(gca,'xtick',[-3:1:3])
        set(gca,'ytick',[-3:1:3])
        annotation('textbox', [0.11 0.525 0.05 0.05], 'String', "Goal", 'EdgeColor',"none",'FontSize',12)
        annotation('textbox', [0.73 0.525 0.05 0.05], 'String', "Goal", 'EdgeColor',"none",'FontSize',12)
        rectangle('position',[-pi 2*user_param.th_dot_max*0.4-user_param.th_dot_max 2*pi*0.1 2*user_param.th_dot_max*0.2], 'FaceColor', [R(end), G(end), B(end)],'EdgeColor','k','LineWidth',2)
        rectangle('position',[2*pi*0.9-pi 2*user_param.th_dot_max*0.4-user_param.th_dot_max 2*pi*0.1 2*user_param.th_dot_max*0.2], 'FaceColor', [R(end), G(end), B(end)],'EdgeColor','k','LineWidth',2)
    elseif ( strcmp(original_sys.name, 'unicycle2d_ltlf' ) )
        a_target = [state_discretization.X_center(user_param.move_S_target+1)-state_discretization.Deltax/2, state_discretization.Y_center(user_param.move_S_target+1)-state_discretization.Deltay/2];
        width_target = [user_param.Nx_S_target*state_discretization.Deltax, user_param.Ny_S_target*state_discretization.Deltay];

        a_water1 = [state_discretization.X_center(user_param.move_S_water1+1)-state_discretization.Deltax/2, state_discretization.Y_center(user_param.move_S_water1+1)-state_discretization.Deltay/2];
        width_water1 = [user_param.Nx_S_water1*state_discretization.Deltax, user_param.Ny_S_water1*state_discretization.Deltay];

        a_carpet1 = [state_discretization.X_center(user_param.move_S_carpet1+1)-state_discretization.Deltax/2, state_discretization.Y_center(user_param.move_S_carpet1+1)-state_discretization.Deltay/2];
        width_carpet1 = [user_param.Nx_S_carpet1*state_discretization.Deltax, user_param.Ny_S_carpet1*state_discretization.Deltay];

        a_obstacle1 = [state_discretization.X_center(user_param.move_S_obstacle1+1)-state_discretization.Deltax/2, state_discretization.Y_center(user_param.move_S_obstacle1+1)-state_discretization.Deltay/2];
        width_obstacle1 = [user_param.Nx_S_obstacle1*state_discretization.Deltax, user_param.Ny_S_obstacle1*state_discretization.Deltay];

        a_obstacle2 = [state_discretization.X_center(user_param.move_S_obstacle2+1)-state_discretization.Deltax/2, state_discretization.Y_center(user_param.move_S_obstacle2+1)-state_discretization.Deltay/2];
        width_obstacle2 = [user_param.Nx_S_obstacle2*state_discretization.Deltax, user_param.Ny_S_obstacle2*state_discretization.Deltay];

        a_obstacle3 = [state_discretization.X_center(user_param.move_S_obstacle3+1)-state_discretization.Deltax/2, state_discretization.Y_center(user_param.move_S_obstacle3+1)-state_discretization.Deltay/2];
        width_obstacle3 = [user_param.Nx_S_obstacle3*state_discretization.Deltax, user_param.Ny_S_obstacle3*state_discretization.Deltay];

        imagesc(X_center, Y_center, reshape(aux_P_lower,[Nx,Ny])')
        set(gca,'YDir','normal')
        colormap( [R(:), G(:), B(:)] );
        colorbar
        caxis([0 1]);
        set(gca,'FontSize',20)
        hold on
        xlabel('$x_1$','Interpreter','latex')
        ylabel('$x_2$','Interpreter','latex')
        title('$\underline p(x)$','Interpreter','latex')
        set(gca,'xtick',[0:0.2:1])
        set(get(gca,'YLabel'),'Rotation',0)
        set(gca,'FontSize',20)
        
                rectangle('position',[a_target width_target], 'EdgeColor',[0 1 0] ,'LineWidth',2) % Goal
        rectangle('position',[a_water1 width_water1], 'EdgeColor',[0 1 1] ,'LineWidth',2) % Water
        rectangle('position',[a_carpet1 width_carpet1], 'EdgeColor',[1 0 1] ,'LineWidth',2) % Carpet

%         rectangle('position',[a_obstacle1 width_obstacle1], 'EdgeColor', [0 0 0],'LineWidth',2) % Obstacle1
%         rectangle('position',[a_obstacle2 width_obstacle2], 'EdgeColor', [0 0 0],'LineWidth',2) % Obstacle2
%         rectangle('position',[a_obstacle3 width_obstacle3], 'EdgeColor', [0 0 0],'LineWidth',2) % Obstacle3
% 
%         rectangle('position',[a_target width_target], 'EdgeColor',[0 0 0] ,'LineWidth',2) % Goal
%         rectangle('position',[a_water1 width_water1], 'EdgeColor',[0 0 0] ,'LineWidth',2) % Water
%         rectangle('position',[a_carpet1 width_carpet1], 'EdgeColor',[0 0 0] ,'LineWidth',2) % Carpet
%
%         rectangle('position',[a_obstacle1 width_obstacle1], 'EdgeColor', [0 0 0],'LineWidth',2) % Obstacle1
%         rectangle('position',[a_obstacle2 width_obstacle2], 'EdgeColor', [0 0 0],'LineWidth',2) % Obstacle2
%         rectangle('position',[a_obstacle3 width_obstacle3], 'EdgeColor', [0 0 0],'LineWidth',2) % Obstacle3

    elseif ( strcmp(original_sys.name, 'unicycle2d' ) )
        [a_goal, b_goal, wx_goal, wy_goal] = state_block2rectangle_fcn(user_param.S_target, state_discretization.X_center{1}, state_discretization.X_center{2}, state_discretization.Deltax{1}, state_discretization.Deltax{2});
        [a_obs1, b_obs1, wx_obs1, wy_obs1] = state_block2rectangle_fcn(user_param.S_obstacle_blocks{1}, state_discretization.X_center{1}, state_discretization.X_center{2}, state_discretization.Deltax{1}, state_discretization.Deltax{2});
        [a_obs2, b_obs2, wx_obs2, wy_obs2] = state_block2rectangle_fcn(user_param.S_obstacle_blocks{2}, state_discretization.X_center{1}, state_discretization.X_center{2}, state_discretization.Deltax{1}, state_discretization.Deltax{2});
        [a_obs3, b_obs3, wx_obs3, wy_obs3] = state_block2rectangle_fcn(user_param.S_obstacle_blocks{3}, state_discretization.X_center{1}, state_discretization.X_center{2}, state_discretization.Deltax{1}, state_discretization.Deltax{2});
        [a_obs4, b_obs4, wx_obs4, wy_obs4] = state_block2rectangle_fcn(user_param.S_obstacle_blocks{4}, state_discretization.X_center{1}, state_discretization.X_center{2}, state_discretization.Deltax{1}, state_discretization.Deltax{2});
        
        imagesc(X_center, Y_center, reshape(aux_P_lower,[Nx,Ny])')
        set(gca,'YDir','normal')
        colormap( [R(:), G(:), B(:)] );
        colorbar
        caxis([0 1]);
        set(gca,'FontSize',20)
        hold on
        xlabel('$x_1$','Interpreter','latex')
        ylabel('$x_2$','Interpreter','latex')
        title('$\underline p(x)$','Interpreter','latex')
        set(gca,'xtick',[0:0.2:1])
        set(get(gca,'YLabel'),'Rotation',0)
        set(gca,'FontSize',20)

        rectangle('position',[a_goal' wx_goal wy_goal], 'EdgeColor', [0 0 0],'LineWidth',2) % Obstacle1
        rectangle('position',[a_obs1' wx_obs1 wy_obs1], 'EdgeColor', [0 0 0],'LineWidth',2) % Obstacle1
        rectangle('position',[a_obs2' wx_obs2 wy_obs2], 'EdgeColor', [0 0 0],'LineWidth',2) % Obstacle1
        rectangle('position',[a_obs3' wx_obs3 wy_obs3], 'EdgeColor', [0 0 0],'LineWidth',2) % Obstacle1
        rectangle('position',[a_obs4' wx_obs4 wy_obs4], 'EdgeColor', [0 0 0],'LineWidth',2) % Obstacle1
        annotation('textbox', [0.15 0.2 0.2 0.1], 'String', "Goal", 'EdgeColor',"none",'FontSize',12)
        annotation('textbox', [0.475 0.2 0.2 0.1], 'String', "Obs", 'EdgeColor',"none",'FontSize',12, Color=[1, 1, 1])
        annotation('textbox', [0.36 0.52 0.2 0.1], 'String', "Obs", 'EdgeColor',"none",'FontSize',12, Color=[1, 1, 1])
        annotation('textbox', [0.58 0.72 wx_obs3 wy_obs3], 'String', "Obs", 'EdgeColor',"none",'FontSize',12, Color=[1, 1, 1])
        annotation('textbox', [0.27, 0.75 wx_obs4 wy_obs4], 'String', "Obs", 'EdgeColor',"none",'FontSize',12, Color=[1, 1, 1])
        set(gca,'FontSize',20)
    elseif ( strcmp(original_sys.name, 'fm1' ) || strcmp(original_sys.name, 'lr' ) )
        imagesc(X_center, ones(size(X_center)), reshape(aux_P_lower,[Nx, 1])')
        set(gca,'YDir','normal')
        colormap( [R(:), G(:), B(:)] );
        colorbar
        caxis([0 1]);
        set(gca,'FontSize',20)
        hold on
        xlabel('$x_1$','Interpreter','latex')
        title('$\underline p(x)$','Interpreter','latex')
        set(gca,'xtick',[-2.5:0.5:2.5])
        set(get(gca,'YLabel'),'Rotation',0)
        set(gca,'FontSize',20)
elseif ( strcmp(original_sys.name, 'fm2' ) )
        imagesc(X_center, Y_center, reshape(aux_P_lower,[Nx, Ny])')
        set(gca,'YDir','normal')
        colormap( [R(:), G(:), B(:)] );
        colorbar
        caxis([0 1]);
        set(gca,'FontSize',20)
        hold on
        xlabel('$x_1$','Interpreter','latex')
        title('$\underline p(x)$','Interpreter','latex')
        set(gca,'xtick',-10:4:14)
        set(get(gca,'YLabel'),'Rotation',0)
        set(gca,'FontSize',20)
        rectangle('position',[0.5, -0.5, 0.5, 1], 'EdgeColor', [1 0.5 0],'LineWidth',2) % Initial set
        
    else
        imagesc(X_center, Y_center, reshape(aux_P_lower,[Nx,Ny])')
        set(gca,'YDir','normal')
        colormap( [R(:), G(:), B(:)] );
        colorbar
        caxis([0 1]);
        set(gca,'FontSize',20)
        hold on
        xlabel('$x_1$','Interpreter','latex')
        ylabel('$x_2$','Interpreter','latex')
        title('$\underline p(x)$','Interpreter','latex')
        set(gca,'xtick',[0:0.2:1])
        set(get(gca,'YLabel'),'Rotation',0)
        set(gca,'FontSize',20)
    
    end
end

f6 = figure(6);
if (right)
    if ( strcmp(original_sys.name, 'fm1' ) )
        imagesc(X_center, ones(size(X_center)), reshape(aux_P_upper,[Nx, 1])')
        set(gca,'YDir','normal')
        colormap( [R(:), G(:), B(:)] );
        colorbar
        caxis([0 1]);
        set(gca,'FontSize',20)
        hold on
        xlabel('$x_1$','Interpreter','latex')
        title('$\underline p(x)$','Interpreter','latex')
        set(gca,'xtick',[-2.5:0.5:2.5])
        set(get(gca,'YLabel'),'Rotation',0)
        set(gca,'FontSize',20)
    elseif ( strcmp(original_sys.name, 'fm2' ) )
        imagesc(X_center, Y_center, reshape(aux_P_upper,[Nx, Ny])')
        set(gca,'YDir','normal')
        colormap( [R(:), G(:), B(:)] );
        colorbar
        caxis([0 1]);
        set(gca,'FontSize',20)
        hold on
        xlabel('$x_1$','Interpreter','latex')
        title('$\underline p(x)$','Interpreter','latex')
        set(gca,'xtick',-10:4:14)
        set(get(gca,'YLabel'),'Rotation',0)
        set(gca,'FontSize',20)
        rectangle('position',[0.5, -0.5, 0.5, 1], 'EdgeColor', [1 0.5 0],'LineWidth',2)
    else
        imagesc(X_center,Y_center, reshape(aux_P_upper,[Nx,Ny])')
        set(gca,'YDir','normal')
        colormap( [R(:), G(:), B(:)] );
        set(gca,'FontSize',20)
        colorbar
        caxis([0 1]);
        hold on
        title('$\overline{P}(q,a,\cdot)$','interpreter','latex')

    end

    movegui(f5,'northwest');
    movegui(f6,'northeast');
    figure(5)
    figure(6)

end

