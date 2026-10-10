function [] = plot_bounds_slice_n_dim(P_lower,P_upper,state_discretization,user_param, original_sys, x3_plane, x4_plane, left, right)
%Plot bounds

n = 1000;
color1 = [0 0 0];%[0.6350 0.0780 0.1840]; % black
color2 = [0.7 0 0];%[0.9290 0.6940 0.1250]; % red
color3 = [0.6 0 0.6];%[0.9290 0.6940 0.1250]; % purple
color4 = [0.1 0.5 0.8];%[0.4660 0.6740 0.1880]; % blue
color5 = [1 1 1];%[0.4660 0.6740 0.1880]; % white

R = [linspace(color1(1),color2(1),n/4) linspace(color2(1),color3(1),n/4) linspace(color3(1),color4(1),n/4) linspace(color4(1),color5(1),n/4)];
G = [linspace(color1(2),color2(2),n/4) linspace(color2(2),color3(2),n/4) linspace(color3(2),color4(2),n/4) linspace(color4(2),color5(2),n/4)];
B = [linspace(color1(3),color2(3),n/4) linspace(color2(3),color3(3),n/4) linspace(color3(3),color4(3),n/4) linspace(color4(3),color5(3),n/4)];

Nx = user_param.Nx;
Ns = user_param.Ns;
X_center = state_discretization.X_center;

if ( original_sys.n == 2 )
    s_slice = 1;
elseif ( original_sys.n == 3 )
    s_slice = (x3_plane-1)*user_param.Nx{1}*user_param.Nx{2}+1;
elseif ( original_sys.n == 4 )
    s_slice = (x4_plane-1)*user_param.Nx{1}*user_param.Nx{2}*user_param.Nx{3} + (x3_plane-1)*user_param.Nx{1}*user_param.Nx{2}+1;
end
S_slice = s_slice:(s_slice+user_param.Nx{1}*user_param.Nx{2}-1);


f5 = figure(5);
if (left)
    imagesc(X_center{1}(Nx{1}*Nx{2}), X_center{2}(Nx{1}*Nx{2}), reshape(P_lower(S_slice),[Nx{1},Nx{2}])')
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

f6 = figure(6);
if (right)
    imagesc(X_center{1}(Nx{1}*Nx{2}), X_center{2}(Nx{1}*Nx{2}), reshape(P_upper(S_slice),[Nx{1},Nx{2}])')
    set(gca,'YDir','normal')
    colormap( [R(:), G(:), B(:)] );
    colorbar
    caxis([0 1]);
    set(gca,'FontSize',20)
    hold on
    xlabel('$x_1$','Interpreter','latex')
    ylabel('$x_2$','Interpreter','latex')
    title('$\overline p(x)$','Interpreter','latex')
    set(gca,'xtick',[0:0.2:1])
    set(get(gca,'YLabel'),'Rotation',0)
    set(gca,'FontSize',20)
end

movegui(f5,'northwest');
movegui(f6,'northeast');
figure(5)
figure(6)

end

