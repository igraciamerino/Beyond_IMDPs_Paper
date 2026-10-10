
myred = [204/255 0 102/255];
mypink = [204/255 0 204/255];
mypurple = [102/255 0/255 204/255];
myblue = [0 0 255/255];


%% Temperature

close all

dim = [2, 3, 4];

IMDP = [0.031, 0.0003, 0.0009, 0.000062532;
           0.066, 0.010, 0.027, 0.004;
           0.150, 0.740, 1.117, 0.200];
SMDP = [0.013, 0.0003, 0.0021, 0.000033546;
            0.015, 0.010, 0.099, 0.004251860;
            0.021, 0.584, 6.822, 0.544];
MIMDP = [0.016, 0.002, 0.091, 0.00013;
            0.039, 0.170, 1.958, 0.010;
            0.102, 39.257, 110.211, 0.828];
twolayer = [0.030, 0.001, 0.002, 0.000076;
                 0.065, 0.018, 0.053, 0.004;
                 0.150, 1.245, 2.043, 0.205];

% % n = 2
% 
% % 
% IMDP_n2 = [0.031, 0.0003, 0.0009, 0.000062532];
% MDPST_n2 = [0.013, 0.0003, 0.0021, 0.000033546];
% MIMDP_n2 = [0.016, 0.002, 0.091, 0.00013];
% IMDP2layer_n2 = [0.030, 0.001, 0.002, 0.000076];
% 
% % n = 3
% 
% IMDP_n3 = [0.066, 0.010, 0.027, 0.004];
% MDPST_n3 = [0.015, 0.010, 0.099, 0.004251860];
% MIMDP_n3 = [0.039, 0.170, 1.958, 0.010];
% IMDP2layer_n3 = [0.065, 0.018, 0.053, 0.004];
% 
% % n = 4
% 
% IMDP_n4 = [0.150, 0.740, 1.117, 0.200];
% MDPST_n4 = [0.021, 0.584, 6.822, 0.544];
% MIMDP_n4 = [0.102, 39.257, 110.211, 0.828];
% IMDP2layer_n4 = [0.150, 1.245, 2.043, 0.205];


% figure(301);
% semilogy(dim, [IMDP_n2(4), IMDP_n3(4), IMDP_n4(4)], LineWidth=2, Marker="+", Color=myred)
% hold on; grid
% semilogy(dim, [IMDP2layer_n2(4), IMDP2layer_n3(4), IMDP2layer_n4(4)], LineWidth=2, Marker="x", Color=mypink)
% semilogy(dim, [MIMDP_n2(4), MIMDP_n3(4), MIMDP_n4(4)], LineWidth=2, Marker="pentagram", Color=mypurple)
% semilogy(dim, [MDPST_n2(4), MDPST_n3(4), MDPST_n4(4)], LineWidth=2, Marker="diamond", Color=myblue)
% legend('IMDP', '2-IMDP', 'MI-MDP', 'SMDP')
% xlabel('n. Dimensions','interpreter','latex')
% ylabel('Memory (GB)','interpreter','latex')
% set(gca,'FontSize',20)
% yticks([1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 1])

figure(201); hold on; grid
    plot(dim, IMDP(:,1), LineWidth=2, Marker="+", Color = myred)
    plot(dim, twolayer(:,1), LineWidth=2, Marker="x", Color=mypink)
    plot(dim, MIMDP(:,1), LineWidth=2, Marker="pentagram", Color=mypurple)
    plot(dim, SMDP(:,1), LineWidth=2, Marker="diamond", Color = myblue)
    legend('IMDP', '2-IMDP', 'MI-MDP', 'SMDP')
    xlabel('n. Dimensions','fontsize',14,'interpreter','latex')
    ylabel('$e_{avg} := \mathrm{avg}( \overline p - \underline p )$','fontsize',14,'interpreter','latex')
    set(gca,'FontSize',20)

    figure(202); hold on; grid
    plot(dim, 60*IMDP(:,2), LineWidth=2, Marker="+", Color = myred)
    plot(dim, 60*twolayer(:,2), LineWidth=2, Marker="x", Color=mypink)
    plot(dim, 60*MIMDP(:,2), LineWidth=2, Marker="pentagram", Color=mypurple)
    plot(dim, 60*SMDP(:,2), LineWidth=2, Marker="diamond", Color = myblue)
    xlabel('n. Dimensions','fontsize',14,'interpreter','latex')
    ylabel('Abstraction Time (sec)','interpreter','latex')
    set(gca,'FontSize',20)
    set(gca, 'YScale', 'log')

    figure(203); hold on; grid
    plot(dim, 60*IMDP(:,3), LineWidth=2, Marker="+", Color = myred)
    plot(dim, 60*twolayer(:,3), LineWidth=2, Marker="x", Color=mypink)
    plot(dim, 60*MIMDP(:,3), LineWidth=2, Marker="pentagram", Color=mypurple)
    plot(dim, 60*SMDP(:,3), LineWidth=2, Marker="diamond", Color = myblue)
    xlabel('n. Dimensions','fontsize',14,'interpreter','latex')
    ylabel('Synthesis Time (sec)','interpreter','latex')
    set(gca,'FontSize',20)
    set(gca, 'YScale', 'log')
    yticks([1e-3, 1e-2, 1e-1, 1, 1e1, 1e2, 1e3, 1e4])
    
    figure(204); hold on; grid
    plot(dim, IMDP(:,4), LineWidth=2, Marker="+", Color = myred)
    plot(dim, twolayer(:,4), LineWidth=2, Marker="x", Color=mypink)
    plot(dim, MIMDP(:,4), LineWidth=2, Marker="pentagram", Color=mypurple)
    plot(dim, SMDP(:,4), LineWidth=2, Marker="diamond", Color = myblue)
    xlabel('n. Dimensions','fontsize',14,'interpreter','latex')
    ylabel('Memory (GB)','interpreter','latex')
    set(gca,'FontSize',20)
    set(gca, 'YScale', 'log')
    yticks([1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 1])

%% Unicycle3D

close all

cs = [6^2, 8^2, 10^2];

IMDP = [0.885, 2.800, 4.671, 0.625;
        0.710, 4.650, 4.769, 0.586;
        0.497, 6.571, 6.765, 0.554];
SMDP = [0.558, 1.325, 10.563, 0.486;
        0.367, 2.322, 14.228, 0.707;
        0.260, 3.450, 24.414, 0.962];
MIMDP = [0.740, 11.645, 347.826, 1.234;
         -1, 26.646, -1, 1.517;
         -1, 55.822, -1, 1.869];
twolayer = [0.884, 4.710, 15.005, 0.633;
            0.704, 7.625, 18.060, 0.594;
            0.490, 12.303, 22.283, 0.562];

figure(201); hold on; grid
    plot(cs, IMDP(:,1), LineWidth=2, Marker="+", Color = myred)
    plot(cs, twolayer(:,1), LineWidth=2, Marker="x", Color=mypink)
%     plot(cs(1), MIMDP(1,1), LineWidth=2, Marker="pentagram", Color=mypurple)
    plot(cs, SMDP(:,1), LineWidth=2, Marker="diamond", Color = myblue)
    legend('IMDP', '2-IMDP', 'MI-MDP', 'SMDP')
    xlabel('$|C|$','fontsize',14,'interpreter','latex')
    ylabel('$e_{avg} := \mathrm{avg}( \overline p - \underline p )$','fontsize',14,'interpreter','latex')
    set(gca,'FontSize',20)

    figure(202); hold on; grid
    plot(cs, 60*IMDP(:,2), LineWidth=2, Marker="+", Color = myred)
    plot(cs, 60*twolayer(:,2), LineWidth=2, Marker="x", Color=mypink)
    plot(cs, 60*MIMDP(:,2), LineWidth=2, Marker="pentagram", Color=mypurple)
    plot(cs, 60*SMDP(:,2), LineWidth=2, Marker="diamond", Color = myblue)
    xlabel('$|C|$','fontsize',14,'interpreter','latex')
    ylabel('Abstraction Time (sec)','interpreter','latex')
    set(gca,'FontSize',20)
    set(gca, 'YScale', 'log')

    figure(203); hold on; grid
    plot(cs, 60*IMDP(:,3), LineWidth=2, Marker="+", Color = myred)
    plot(cs, 60*twolayer(:,3), LineWidth=2, Marker="x", Color=mypink)
%     plot(cs(1), 60*MIMDP(1,3), LineWidth=2, Marker="pentagram", Color=mypurple)
    plot(cs, 60*SMDP(:,3), LineWidth=2, Marker="diamond", Color = myblue)
    xlabel('$|C|$','fontsize',14,'interpreter','latex')
    ylabel('Synthesis Time (sec)','interpreter','latex')
    set(gca,'FontSize',20)
    set(gca, 'YScale', 'log')
    yticks([1e2, 1e3])
    
    figure(204); hold on; grid
    plot(cs, IMDP(:,4), LineWidth=2, Marker="+", Color = myred)
    plot(cs, twolayer(:,4), LineWidth=2, Marker="x", Color=mypink)
    plot(cs, MIMDP(:,4), LineWidth=2, Marker="pentagram", Color=mypurple)
    plot(cs, SMDP(:,4), LineWidth=2, Marker="diamond", Color = myblue)
    xlabel('$|C|$','fontsize',14,'interpreter','latex')
    ylabel('Memory (GB)','interpreter','latex')
    set(gca,'FontSize',20)
    set(gca, 'YScale', 'log')
    yticks([1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 1])
