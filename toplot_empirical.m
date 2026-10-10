
close all

%% Unicycle 2D

% ns = [30^2, 42^2, 60^2, 75^2];
 
imdp = [];
twolayer = [];
mimdp = [];
smdp = [];

% figure(301);
% hold on; grid
% plot(ns, MDPST_emp_6, LineWidth=2, Marker="diamond", LineStyle="--")
% plot(ns, MDPST_lower_6, LineWidth=2, Marker="diamond", LineStyle="--")
% plot(ns, MDPST_upper_6, LineWidth=2, Marker="diamond", LineStyle="--")
% plot(ns, MDPST_emp_12, LineWidth=2, Marker="diamond")
% plot(ns, MDPST_lower_12, LineWidth=2, Marker="diamond")
% plot(ns, MDPST_upper_12, LineWidth=2, Marker="diamond")
% legend('$P^\kappa[\varphi], |C|=36$', '$\underline{p}, |C|=36$', '$\overline{p}, |C|=36$', '$P^\kappa[\varphi], |C|=144$', '$\underline{p}, |C|=144$', '$\overline{p}, |C|=144$','fontsize',10,'interpreter','latex')
% %     ylim([0.8, 1])
%     xlabel('$|S|$','fontsize',14,'interpreter','latex')
%     ylabel('Satisfaction Probability','interpreter','latex')
%     set(gca,'FontSize',20)
% %     yticks([1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 1])

%% Unicycle 3D

close all


myred = [204/255 0 102/255];
myblue = [0 0 255/255];
mypurple = [102/255 0/255 204/255];
mypink = [204/255 0 204/255];

ns = [25^3, 27^3, 30^3, 32^3, 35^3];

% n = 2

% 
MDPST_emp_6 = [0.736300, 0.77178, 0.881100, 0.86332, 0.834400];
MDPST_lower_6 = [0.677133, 0.731085, 0.727543, 0.735948, 0.760716];
MDPST_upper_6 = [0.918822, 0.910934, 0.901311, 0.902075, 0.900101];

MDPST_emp_12 = [0.891700, 0.857130, 0.844200, 0.8465 0.878910];
MDPST_lower_12 = [0.707260, 0.743766, 0.737118, 0.748579, 0.766424];
MDPST_upper_12 = [0.911481, 0.904887, 0.895656, 0.897810, 0.899282];

figure(301);
semilogx(ns, MDPST_emp_6, LineWidth=2, Marker="diamond", LineStyle="--", Color=mypurple)
hold on; grid
semilogx(ns, MDPST_lower_6, LineWidth=2, Marker="diamond", LineStyle="--", Color = myred)
semilogx(ns, MDPST_upper_6, LineWidth=2, Marker="diamond", LineStyle="--", Color = myblue)
semilogx(ns, MDPST_emp_12, LineWidth=2, Marker="diamond", Color = mypurple)
semilogx(ns, MDPST_lower_12, LineWidth=2, Marker="diamond", color = myred)
semilogx(ns, MDPST_upper_12, LineWidth=2, Marker="diamond", color = myblue)
legend('$P^\kappa[\varphi], |C|=36$', '$\underline{p}, |C|=36$', '$\overline{p}, |C|=36$', '$P^\kappa[\varphi], |C|=144$', '$\underline{p}, |C|=144$', '$\overline{p}, |C|=144$','fontsize',10,'interpreter','latex')
%     ylim([0.8, 1])
    xlabel('$|S|$','fontsize',14,'interpreter','latex')
    ylabel('Satisfaction Probability','interpreter','latex')
    set(gca,'FontSize',20)
%     yticks([1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 1])
