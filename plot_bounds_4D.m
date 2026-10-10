function [] = plot_bounds_4D(P_lower,P_upper,support_qp,state_discretization)
%Plot bounds 3D

X_center = state_discretization.X_center{1};
Y_center = state_discretization.X_center{2};
Z_center = state_discretization.X_center{3};

n = 500;
color1 = [0 0 0];%[0.6350 0.0780 0.1840]; % black
color2 = [0.7 0 0];%[0.9290 0.6940 0.1250]; % red
color3 = [0.6 0 0.6];%[0.9290 0.6940 0.1250]; % purple
color4 = [0.1 0.5 0.8];%[0.4660 0.6740 0.1880]; % blue
color5 = [1 1 1];%[0.4660 0.6740 0.1880]; % white

R = [linspace(color1(1),color2(1),n/5) linspace(color2(1),color3(1),n/5) linspace(color3(1),color4(1),n/5) linspace(color4(1),color5(1),n/5)];
G = [linspace(color1(2),color2(2),n/5) linspace(color2(2),color3(2),n/5) linspace(color3(2),color4(2),n/5) linspace(color4(2),color5(2),n/5)];
B = [linspace(color1(3),color2(3),n/5) linspace(color2(3),color3(3),n/5) linspace(color3(3),color4(3),n/5) linspace(color4(3),color5(3),n/5)];


% Don't plot lower bound
% figure
% scatter3(X_center(indices_lower_nonzero),Y_center(indices_lower_nonzero),Z_center(indices_lower_nonzero),40,P_upper(indices_lower_nonzero),'filled')
% cb = colorbar();
figure(3)
scatter3(X_center(support_qp),Y_center(support_qp),Z_center(support_qp),40,P_upper,'filled')
% colormap( [R(:), G(:), B(:)] );
set(gca,'FontSize',20)

clim([0, 1]);
end

