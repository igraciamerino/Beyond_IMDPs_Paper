clear
clc
close all

%% Case of d = 1

%% Size of the ambiguity set

% Fournier's
p = 1; % (Doesn't work for the 2-Wass distance)
d = 1;
m = inf;
beta = 10^(-9);
Ns = logspace(2,7,100000);
diam_inf = 1;

rho = 1/2*diam_inf;
C_stars = d^(p/m)*(0.73*log(Ns)+1);

if (p == 2) % Use 2-Wasserstein dist
    C_stars = d^(p/m)*(0.73*log(Ns)+1);
    epsilons = 2*rho*( C_stars.*Ns.^(-1/d) + sqrt(d)*(2*log(1/beta))^(1/(2*p))*Ns.^(-1/(2*p)) );
elseif (p == 1) % Use 1-Wasserstein dist (CHECK THIS !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1)
    C_stars = d^(p/m)*2.42;
    epsilons = 2*rho*( C_stars.*Ns.^(-1/2) + sqrt(d)*(2*log(1/beta))^(1/(2*p))*Ns.^(-1/(2*p)) );
end

figure
loglog(Ns,epsilons)
hold on
grid

%% Cluster samples

% Use 1-Wasserstein dist
N = 7886; % The corresponding to rho = 1, epsilon = 0.2;

tol = 0.4;%0.03;
% weight_samples = 0.005;
a = 1;
b = 1;
Xi = a/3*randn(N,2);

insideellipse = (Xi(:,1)-.0).^2/a^2+(Xi(:,2)-.0).^2/b^2<1;
Xi = Xi(insideellipse,:);

[~,ia] = uniquetol(Xi, tol, 'ByRows', true, ...
    'OutputAllIndices', true);

Xi_c = zeros(length(ia),2);
weight = zeros(length(ia),1);
sum_d_wrt_j = zeros(length(ia),1);
for i = 1:length(ia)
    Xi_c(i,:) = mean(Xi(ia{i},:),1);
    d_i = zeros(length(ia{i}),1);
    for j = 1: length(ia{i})
        d_i(j) = norm(Xi(ia{i}(j),:) - Xi_c(i,:),p)^p;
    end
    sum_d_wrt_j(i) = sum(d_i);
    weight(i) = length(ia{i});
end

weight_norm = weight/sum(weight);
W = 1/N*sum(sum_d_wrt_j);
W = W^(1/p)

% figure(1)
% plot(Xi(:,1),Xi(:,2),'.')
% hold on
% axis equal
% plot(Xi_c(1,:), Xi_c(2,:), '.r', 'MarkerSize', 10)

figure(2)
plot3(Xi_c(:,1),Xi_c(:,2),weight_norm,'o')

%% Compute exact Wasserstein distance between the two sets

% Wass_distance(Xi,Xi_c)

%% Case of d = 2

% Fournier's
p = 2;
d = 2;
m = 'inf';
beta = 10^(-9);
Ns = logspace(2,8,100000);
diam_inf = 2;

rho = 1/2*diam_inf;
if (strcmp('inf',m))
    C_stars = 0.73*log(Ns) + 1; % If we use the inf norm
else
    C_stars = d^(p/m)*( 0.73*log10(Ns) + 1 );
end
epsilons = 2*rho*( C_stars.*Ns.^(-1/d) + sqrt(d)*(2*log(1/beta))^(1/(2*p))*Ns.^(-1/(2*p)) );

%figure
loglog(Ns, epsilons)
hold on
grid

%% Cluster samples

% Use 2-Wasserstein dist
N = 138845; % The one corresponding to rho = 1, epsilon = 0.1

tol = 0.3;%0.03;
% weight_samples = 0.005;
a = 1;
b = 1;
Xi = a/3*randn(N,d);

insideellipse = (Xi(:,1)-.0).^2/a^2 + (Xi(:,2)-.0).^2/b^2 <1;
Xi = Xi(insideellipse,:);

[~,ia] = uniquetol(Xi, tol, 'ByRows', true, ...
    'OutputAllIndices', true);

Xi_c = zeros(length(ia),d);
weight = zeros(length(ia),1);
sum_d_wrt_j = zeros(length(ia),1);
for i = 1:length(ia)
    Xi_c(i,:) = mean(Xi(ia{i},:),1);
    d_i = zeros(length(ia{i}),1);
    for j = 1: length(ia{i})
        d_i(j) = norm(Xi(ia{i}(j),:) - Xi_c(i,:),p)^p;
    end
    sum_d_wrt_j(i) = sum(d_i);
    weight(i) = length(ia{i});
end

weight_norm = weight/sum(weight);
W = 1/N*sum(sum_d_wrt_j);
W = W^(1/p)

% figure(1)
% plot(Xi(:,1),Xi(:,2),'.')
% hold on
% axis equal
% plot(Xi_c(1,:), Xi_c(2,:), '.r', 'MarkerSize', 10)

figure(2)
plot3(Xi_c(:,1),Xi_c(:,2),weight_norm,'o')

%% Case of d = 3

% Fournier's
p = 1; % (Doesn't work for the 2-Wass distance)
d = 3;
m = 'inf';
beta = 10^(-9);
Ns = logspace(2,8,100000);
diam_inf = 2;

rho = 1/2*diam_inf;
if (strcmp('inf',m))
    C_stars = 3.72; % If we use the inf norm
else
    C_stars = d^(p/m)*3.72;
end
epsilons = 2*rho*( C_stars.*Ns.^(-1/d) + sqrt(d)*(2*log(1/beta))^(1/(2*p))*Ns.^(-1/(2*p)) );

figure
loglog(Ns, epsilons)
hold on
grid

%% Cluster samples

% Use 1-Wasserstein dist
% N = 1776000; % The corresponding to rho = 1, epsilon = 0.2;
% N = 13277700; % The corresponding to rho = 1, epsilon = 0.1; (after clustering (tol = 0.1) it is 0.2)
% N = 1812420; % The corresponding to rho = 1/2, epsilon = 0.1; (after clustering (tol = 0.1) it is 0.2)
N = 913734; % The corresponding to rho = 1, epsilon = 0.1 when using m = inf. (after clustering (tol = 0.1) it is 0.16)
    % Also... The corresponding to rho = 1, epsilon = 0.1 when using m = inf. (after clustering (tol = 0.2) it is 0.22)

tol = 0.2; % 0.03;
Xi_hat_aux = 1/3*randn(d,N);
a = 1;

% This is in case that I want to truncate the distribution at 3 std
ind_aux = false(1,N);
for w_ind = 1:N
    w = Xi_hat_aux(:, w_ind);
    if ( w(1)^2 + w(2)^2 + w(3)^2 < a^2 )
        ind_aux(w_ind) = true;
    end
end


% This is in case that I want to truncate the distribution on a hypercube
% ind_aux = false(1,N);
% for w_ind = 1:N
%     w = Xi_hat_aux(:, w_ind);
%     if ( norm( w, m) < rho )
%         ind_aux(w_ind) = true;
%     end
% end

Xi = Xi_hat_aux(:,ind_aux);

[~,ia] = uniquetol(Xi', tol, 'ByRows', true, ...
    'OutputAllIndices', true);

Nc = length(ia);
Xi_c = zeros(d,Nc);
weight = zeros(1,Nc);
sum_d_wrt_j = zeros(1,Nc);
for i = 1:Nc
    Xi_c(:,i) = mean(Xi(:,ia{i}),2);
    d_i = zeros(length(ia{i}),1);
    for j = 1:length(ia{i})
        d_i(j) = norm(Xi(:,ia{i}(j)) - Xi_c(:,i),m)^p;
    end
    sum_d_wrt_j(i) = sum(d_i);
    weight(i) = length(ia{i});
end

weight_norm = weight/sum(weight);
W = 1/N*sum(sum_d_wrt_j);
W = W^(1/p)

% figure(1)
% plot(Xi(:,1),Xi(:,2),'.')
% hold on
% axis equal
% plot(Xi_c(1,:), Xi_c(2,:), '.r', 'MarkerSize', 10)

figure(2)
plot3(Xi_c(1,:),Xi_c(2,:),weight_norm,'o')

%% Case of d = 2, p = 2

% Fournier's
p = 2;
d = 2;
m = 2;
beta = 10^(-9);
Ns = logspace(2,9,100000);
diam_inf = 2*(0.01 + 8*0.005); % TRuncate at 8 std

rho = 1/2*diam_inf;

gs = d^(p/m)*( ( 1.42 )*Ns.^(-1/4) ).^p;

epsilons = 2*rho*( gs.^(1/p) + sqrt(d)*(2*log(1/beta))^(1/(2*p))*Ns.^(-1/(2*p)) );

%figure
loglog(Ns, epsilons)
hold on
grid

%% Cluster samples

% Use 2-Wasserstein dist
N = 612700; % Truncate at 3 std
N = 2348410; % Truncate at 5 std
N = 1e7; % Truncate at 8 std

tol = 0.1;%0.03;
% weight_samples = 0.005;
a = 1;
b = 1;
gm = gmdistribution([0.01 0;-0.01 0],(0.005)^2*ones(1,2));
Xi = random(gm,N);

% Xi = [];
% for w_ind = 1:N
%     w = Xi_hat_aux(:, w_ind);
%     if ( (w(1)^2 < rho^2 ) && ( w(2)^2 < rho^2 ) )
%         Xi = [Xi w];
%     end
% end

[~,ia] = uniquetol(Xi, tol, 'ByRows', true, ...
    'OutputAllIndices', true);

Xi_c = zeros(length(ia),d);
weight = zeros(length(ia),1);
sum_d_wrt_j = zeros(length(ia),1);
for i = 1:length(ia)
    Xi_c(i,:) = mean(Xi(ia{i},:),1);
    d_i = zeros(length(ia{i}),1);
    for j = 1: length(ia{i})
        d_i(j) = norm(Xi(ia{i}(j),:) - Xi_c(i,:),p)^p;
    end
    sum_d_wrt_j(i) = sum(d_i);
    weight(i) = length(ia{i});
end

weight_norm = weight/sum(weight);
W = 1/N*sum(sum_d_wrt_j);
W = W^(1/p)

% figure(1)
% plot(Xi(:,1),Xi(:,2),'.')
% hold on
% axis equal
% plot(Xi_c(1,:), Xi_c(2,:), '.r', 'MarkerSize', 10)

figure(2)
plot3(Xi_c(:,1),Xi_c(:,2),weight_norm,'o')

%% TRy to do the same with d = 2, p = 1 and m = inf

% Fournier's
p = 1;
d = 2;
m = 'inf';
beta = 10^(-9);
Ns = logspace(2,9,100000);
diam_inf = 5e-2;

rho = 1/2*diam_inf;
if (strcmp('inf',m))
    C_stars = 3.72; % If we use the inf norm
else
    C_stars = d^(p/m)*3.72;
end
epsilons = 2*rho*( C_stars.*Ns.^(-1/d) + sqrt(d)*(2*log(1/beta))^(1/(2*p))*Ns.^(-1/(2*p)) );

%figure
loglog(Ns, epsilons)
hold on
grid

%% Cluster samples

% Use 2-Wasserstein dist
N = 602612; % The one corresponding to rho = 1, epsilon = 0.1

tol = 0.3;%0.03;
% weight_samples = 0.005;
a = 1;
b = 1;
gm = gmdistribution([0.01 0;-0.01 0],(0.005)^2*ones(1,2));
Xi = random(gm,N);

% Xi = [];
% for w_ind = 1:N
%     w = Xi_hat_aux(:, w_ind);
%     if ( (w(1)^2 < rho^2 ) && ( w(2)^2 < rho^2 ) )
%         Xi = [Xi w];
%     end
% end

[~,ia] = uniquetol(Xi, tol, 'ByRows', true, ...
    'OutputAllIndices', true);

Xi_c = zeros(length(ia),d);
weight = zeros(length(ia),1);
sum_d_wrt_j = zeros(length(ia),1);
for i = 1:length(ia)
    Xi_c(i,:) = mean(Xi(ia{i},:),1);
    d_i = zeros(length(ia{i}),1);
    for j = 1: length(ia{i})
        d_i(j) = norm(Xi(ia{i}(j),:) - Xi_c(i,:),p)^p;
    end
    sum_d_wrt_j(i) = sum(d_i);
    weight(i) = length(ia{i});
end

weight_norm = weight/sum(weight);
W = 1/N*sum(sum_d_wrt_j);
W = W^(1/p)

% figure(1)
% plot(Xi(:,1),Xi(:,2),'.')
% hold on
% axis equal
% plot(Xi_c(1,:), Xi_c(2,:), '.r', 'MarkerSize', 10)

figure(2)
plot3(Xi_c(:,1),Xi_c(:,2),weight_norm,'o')

%% From now on, comparison for different parameters

%% Case of d = 2, p = 1, m = inf, beta = 1e-1

p = 1;
d = 2;
m = inf;
beta = 10^(-1);
Ns = logspace(2,9,100000);
diam_inf = 2;

rho = 1/2*diam_inf;

gs = d^(p/m)*( 0.73*log(Ns) + 1 )./Ns.^(1/2);

epsilons = 2*rho*( gs.^(1/p) + sqrt(d)*(2*log(1/beta))^(1/(2*p))*Ns.^(-1/(2*p)));

%figure
loglog(Ns, epsilons, 'Color', [0 0.4470 0.7410],'LineWidth',3)
hold on
grid

% Case of d = 2, p = 1, m = inf, beta = 1e-9

beta = 10^(-9);

gs = d^(p/m)*( 0.73*log(Ns) + 1 )./Ns.^(1/2);

epsilons = 2*rho*( gs.^(1/p) + sqrt(d)*(2*log(1/beta))^(1/(2*p))*Ns.^(-1/(2*p)) );

%figure
loglog(Ns, epsilons, 'Color', [0 0.4470 0.7410],'LineStyle', '--','LineWidth',3)


% Case of d = 2, p = 1, m = 2, beta = 1e-1

p = 1;
d = 2;
m = 2;
beta = 10^(-1);
diam_inf = 2;

rho = 1/2*diam_inf;

gs = d^(p/m)*( 0.73*log(Ns) + 1 )./Ns.^(1/2);

epsilons = 2*rho*( gs.^(1/p) + sqrt(d)*(2*log(1/beta))^(1/(2*p))*Ns.^(-1/(2*p)) );

%figure
loglog(Ns, epsilons, 'Color',[0.3010 0.7450 0.9330],'LineWidth',3)


% Case of d = 2, p = 1, m = 2, beta = 1e-9

beta = 10^(-9);

gs = d^(p/m)*( 0.73*log(Ns) + 1 )./Ns.^(1/2);

epsilons = 2*rho*( gs.^(1/p) + sqrt(d)*(2*log(1/beta))^(1/(2*p))*Ns.^(-1/(2*p)) );

%figure
loglog(Ns, epsilons, 'Color',[0.3010 0.7450 0.9330],'LineStyle', '--','LineWidth',3)


% Case of d = 2, p = 2, m = inf, beta = 1e-1

p = 2;
d = 2;
m = inf;
beta = 10^(-1);
diam_inf = 2;

rho = 1/2*diam_inf;

gs = d^(p/m)*( ( 1.42 )*Ns.^(-1/4) ).^p;

epsilons = 2*rho*( gs.^(1/p) + sqrt(d)*(2*log(1/beta))^(1/(2*p))*Ns.^(-1/(2*p)) );

%figure
loglog(Ns, epsilons, 'Color',[0.8500 0.3250 0.0980],'LineWidth',3)

% Case of d = 2, p = 2, m = inf, beta = 1e-9

p = 2;
m = inf;
beta = 10^(-9);

gs = d^(p/m)*( ( 1.42 )*Ns.^(-1/4) ).^p;

epsilons = 2*rho*( gs.^(1/p) + sqrt(d)*(2*log(1/beta))^(1/(2*p))*Ns.^(-1/(2*p)) );

%figure
loglog(Ns, epsilons, 'Color',[0.8500 0.3250 0.0980],'LineStyle', '--','LineWidth',3)

% Case of d = 2, p = 2, m = 2, beta = 1e-1

p = 2;
m = 2;
beta = 10^(-1);

gs = d^(p/m)*( ( 1.42 )*Ns.^(-1/4) ).^p;

epsilons = 2*rho*( gs.^(1/p) + sqrt(d)*(2*log(1/beta))^(1/(2*p))*Ns.^(-1/(2*p)) );

%figure
loglog(Ns, epsilons, 'Color', [0.6350 0.0780 0.1840],'LineWidth',3)

% Case of d = 2, p = 2, m = 2, beta = 1e-9

p = 2;
m = 2;
beta = 10^(-9);

gs = d^(p/m)*( ( 1.42 )*Ns.^(-1/4) ).^p;

epsilons = 2*rho*( gs.^(1/p) + sqrt(d)*(2*log(1/beta))^(1/(2*p))*Ns.^(-1/(2*p)) );

%figure
loglog(Ns, epsilons, 'Color',[0.6350 0.0780 0.1840],'LineStyle', '--','LineWidth',3)

legend('$p = 1,\: m = \infty,\: \beta = 0.1$', '$p = 1,\: m = \infty,\: \beta = 10^{-9}$', '$p = 1,\: m = 2,\: \beta = 0.1$', '$p = 1,\: m = 2,\: \beta = 10^{-9}$', '$p = 2,\: m = \infty,\: \beta = 0.1$', '$p = 2,\: m = \infty,\: \beta = 10^{-9}$', '$p = 2,\: m = 2,\: \beta = 0.1$', '$p = 2,\: m = 2,\: \beta = 10^{-9}$', 'Interpreter','latex')
set(gca,'FontSize',18)

%% Case of d = 2, p = 2

% Fournier's
p = 2;
d = 2;
m = inf;
beta = 10^(-9);
Ns = logspace(2,9,100000);
diam_inf = 2;

rho = 1/2*diam_inf;

gs = d^(p/m)*( ( 1.42 )*Ns.^(-1/4) ).^p;

epsilons = 2*rho*( gs.^(1/p) + sqrt(d)*(2*log(1/beta))^(1/(2*p))*Ns.^(-1/(2*p)) );

%figure
loglog(Ns, epsilons)
hold on
grid

