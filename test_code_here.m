%% Test inhull
dim_Qn = 10000;
m = 4;
p = 2;
xyz = rand(m,p);
testpts = rand(dim_Qn,p)-.1;

tic
in1 = inhull(testpts,xyz);
toc

plot(xyz(:,1),xyz(:,2),'rs')
hold on
for i=1:dim_Qn
    if(in1(i)==1)
plot(testpts(i,1),testpts(i,2),'b.')
    else
plot(testpts(i,1),testpts(i,2),'g.')
    end
end

%% Test norm here
N = 150;
n_vert = 4;
dim = 2;
d = zeros(N,1);
Xi_hat = rand(dim,N);
vertices = rand(dim,n_vert);

prev = 0;
aux = zeros(n_vert,dim);
tic
for i=1:N
    for j=1:n_vert
        for k=1:dim
            aux(j,k) = abs(vertices(k,j)-Xi_hat(k,i));
        end
    end
    d(i) = max(reshape(aux,[n_vert*dim,1]));
end
toc

%% Test uniquetol

tol = 0.002;

aux_uniquetol = [lookup_table.Y_table' lookup_table.p_lower' lookup_table.p_upper'];
% lookup_table.Y_table = aux_uniquetol(:,1:2)';
% lookup_table.p_lower = aux_uniquetol(:,3)';

plot3(aux_uniquetol(:,1), aux_uniquetol(:,2), aux_uniquetol(:,4), '.')
view(-28,32)

DS = [1 1 Inf Inf];
[~,ia] = uniquetol(aux_uniquetol, tol, 'ByRows', true, ...
    'OutputAllIndices', true, 'DataScale', DS);

clusterA_uniquetol = zeros(length(ia),4);
for k = 1:length(ia)
    cluster_k = aux_uniquetol(ia{k},:);
    [~, index_p_min_cluster] = min(cluster_k(:,3));
    clusterA_uniquetol(k,3) = cluster_k(index_p_min_cluster);
    [~, index_p_max_cluster] = max(cluster_k(:,4));
    clusterA_uniquetol(k,4) = cluster_k(index_p_max_cluster);
    
    clusterA_uniquetol(k,1:2) = mean(aux_uniquetol(ia{k},1:2),1);
    clusterA_uniquetol(k,3) = min(aux_uniquetol(ia{k},3));
    clusterA_uniquetol(k,4) = max(aux_uniquetol(ia{k},4));
end

hold on
plot3(clusterA_uniquetol(:,1), clusterA_uniquetol(:,2), clusterA_uniquetol(:,4), '.r', 'MarkerSize', 15)

%% Test probability vector with three states

simplex = [1 0 0;0 1 0;0 0 1; 1 0 0]';
bounds = [0.1 0.3;0.1 0.3;0.5 0.7];
cube = [0.1 0.1 0.5;0.3 0.1 0.5;0.3 0.3 0.5;0.1 0.3 0.5;0.1 0.1 0.5;0.1 0.1 0.7;0.3 0.1 0.7;0.3 0.3 0.7;0.1 0.3 0.7;0.1 0.1 0.7]';
worst_cases = [0.1 0.3 0.6;0.3 0.2 0.5; 0.3 0.1 0.6; 0.2 0.1 0.7;0.1 0.3 0.6]';

N = 1000000;

Xi_hat = rand(3,N);

for i=1:N
    xi = Xi_hat (:,i);
    s = sum(xi);
    xi_scaled = xi/s;
    Xi_hat_scaled (:,i) = xi_scaled;
end

in1 = inhull(Xi_hat_scaled',cube');

plot3(cube(1,:),cube(2,:),cube(3,:),'ko-')
hold on
grid
plot3(Xi_hat_scaled(1,in1),Xi_hat_scaled(2,in1),Xi_hat_scaled(3,in1),'y.','Markersize',0.1)
plot3(worst_cases(1,:),worst_cases(2,:),worst_cases(3,:),'bo-','Linewidth',2)
plot3(0.2,0.2+0.1/1.1^2,0.6-0.1/1.1^2,'ro')
% xlim([0 1])
% ylim([0 1])
% zlim([0 1])

%% Test convex region

clear
close all
clc

N_samples = 5;
N_p = 1000000;
epsilon = sqrt(0.1*1^2);
emp_samples = [2 2;4 2;2 4]';
p_emp = [0.2 0.2 0.6]';
cube = [0.1 0.1 0.5;0.3 0.1 0.5;0.3 0.3 0.5;0.1 0.3 0.5;0.1 0.1 0.5;0.1 0.1 0.7;0.3 0.1 0.7;0.3 0.3 0.7;0.1 0.3 0.7;0.1 0.1 0.7]';

k = 1;
for j = 1:N_p
    Xi_hat = 6*rand(2,N_samples);
    wsd(j) = ws_distance(Xi_hat, emp_samples, 2);
    if (wsd(j)<=epsilon+0.01)
        for i = 1:N_samples
            xi = Xi_hat (:,i);
            if (xi(2)<3) % q1 or q2
                if (xi(1)<3) % q1
                    region(i) = 1;
                end
                if (xi(1)>=3) % q2
                    region(i) = 2;
                end
            else
                region(i) = 3;
            end
          
        end
    
        Nq1 = sum([region == 1]);
        Nq2 = sum([region == 2]);
        Nq3 = sum([region == 3]);
        
        p(:,k) = [Nq1 Nq2 Nq3]'/(Nq1+Nq2+Nq3);
        k = k+1;    
    else
    end
%         
%         counter = counter+1;
end


plot3(p(1,:),p(2,:),p(3,:),'.')
hold on
grid
plot3(p_emp(1),p_emp(2),p_emp(3),'mo')
plot3(cube(1,:),cube(2,:),cube(3,:),'ko-')

figure
xlim([0 6]);ylim([0 6])
hold on
grid
plot(emp_samples(1,:),emp_samples(2,:),'x')

%% Large LP

Nq = 40*40;
Na = 9;
s = 3;
a = 3;
dim_Qs = s*s;
dim_Qn = (s+2*a)*(s+2*a) + 1;


Nc = 2*dim_Qn + 1
Nc_eq = dim_Qs +1;
Nv = dim_Qn*dim_Qs + dim_Qs
A = rand(Nc,Nv);
b = randn(Nc,1);
Aeq = randn(Nc_eq,Nv);
beq = randn(Nc_eq,1);
lb = [];%zeros(n,1);
ub = [];%ones(n,1);
f = ones(Nv,1);

tic
x = linprog(f,A,b,Aeq,beq,lb,ub);
time = toc*Nq*Na/60;

fprintf('Time required for one iteration = %d min. \n',time)

%% Test interval value-iteration with linprog

f = solution.P_reach_lower(1:end-1);
lb = solution.P_lower(500,1,1:end-1);
ub = solution.P_upper(500,1,1:end-1);
Aeq = ones(1,user_param.Nq);
beq = 1;


tic
c2 = linprog(f,[],[],Aeq,beq,lb,ub);
time = toc*user_param.Nq*user_param.Na/60;

fprintf('Time required for one iteration = %d min. \n',time)

%% Test intersect

pgon = polyshape([0 0 1 3], [0 3 3 0]);
plot(pgon)
t = 0.05:0.5:2*pi;
x1 = cos(t);
y1 = sin(t);
x2 = 0.5*cos(t);
y2 = 0.5*sin(t);
pgon2 = polyshape({x1,x2},{y1,y2});
hold on; plot(pgon2)

tic
polyout = intersect(pgon,pgon2);
toc

plot(polyout)

%% Additional constraints in value iteration

q = floor(user_param.Nx/2)*(user_param.Nx-1);
a = 1;
epsilon = 0.03;

c_hat = 1/3*ones(3,1);
Qs(1) = 576;
Qs(2) = 615;
Qs(3) = 656;
s = length(Qs);
Nv = s*user_param.Nq;

Aeq_c_hat(1,:) = [ones(1,user_param.Nq) zeros(1,user_param.Nq) zeros(1,user_param.Nq)];
Aeq_c_hat(2,:) = [zeros(1,user_param.Nq) ones(1,user_param.Nq) zeros(1,user_param.Nq)];
Aeq_c_hat(3,:) = [zeros(1,user_param.Nq) zeros(1,user_param.Nq) ones(1,user_param.Nq)];
beq_c_hat = c_hat;

A_ub = zeros(user_param.Nq,Nv);
for i=1:user_param.Nq
    A_ub(i,i) = 1;
    A_ub(i,user_param.Nq+i) = 1;
    A_ub(i,2*user_param.Nq+i) = 1;
    
    b_ub(i,1) = solution.P_upper(q,a,i);
end

A_lb = zeros(user_param.Nq,Nv);
for i=1:user_param.Nq
    A_lb(i,i) = -1;
    A_lb(i,user_param.Nq+i) = -1;
    A_lb(i,2*user_param.Nq+i) = -1;
    
    b_lb(i,1) = -solution.P_lower(q,a,i);
end

dist = 2*norm(original_sys.state_space_upper_right_corner-original_sys.state_space_lower_left_corner,2)*ones(s,user_param.Nq);
for i = 1:s
    q_s = Qs(i);
    vertices_q_s = state_discretization.vertices_q{q_s};
    for j = 1:user_param.Nq
        % dist(i,j) = sqrt((state_discretization.X_center(Qs(i))-state_discretization.X_center(j))^2 + (state_discretization.Y_center(Qs(i))-state_discretization.Y_center(j))^2);
        vertices_q = state_discretization.vertices_q{j};
        for ii = 1:4
            for jj = 1:4
                dist(i,j) = min(dist(i,j), norm(vertices_q_s(:,ii)-vertices_q(:,jj),2));
            end
        end
    end
end

A_budget = [dist(1,:) dist(2,:) dist(3,:)];
b_budget = epsilon;

A = [A_ub; A_lb; A_budget];
b = [b_ub; b_lb; b_budget];

Aeq = Aeq_c_hat;
beq = beq_c_hat;

for i =1:user_param.Nq
    f_aux(i) = solution.P_reach_lower(i);
end

f = f_aux*A_ub;

lb = zeros(1,Nv);

tic
x = linprog(f,A,b,Aeq,beq,lb,[]);
time = toc*user_param.Nq*user_param.Na/60;

fprintf('Time required for one iteration = %d min. \n',time)

c = A_ub*x;

% Compare whith no extra constraints

f = solution.P_reach_lower(1:end-1);
lb = solution.P_lower(q,a,1:end-1);
ub = solution.P_upper(q,a,1:end-1);
Aeq = ones(1,user_param.Nq);
beq = 1;


tic
c2 = linprog(f,[],[],Aeq,beq,lb,ub);
time = toc*user_param.Nq*user_param.Na/60;

fprintf('Time required for one iteration = %d min. \n',time)

plot_bounds(c,c2,state_discretization,user_param)

%% LP reduction

Nq = 1600;
s = 9;
dist_from_samples = 4;
% n = (sqrt(s) + 2*dist_from_samples)^2
n = 1:1:Nq-1;

Nvar_original = Nq*s;
Nineq_original = Nq*(s+2)+1;
Neq_original = s+1;

Red_Nvar = (Nq - n - 1)*(s-1) - 1;
Red_Nineq = (Nq - n)*s - s;
Red_Neq = -1 + (n-n);

Nvar_new = Nvar_original - Red_Nvar;
Nineq_new = Nineq_original - Red_Nineq;
Neq_new = Neq_original - Red_Neq;

Fraction_red_Nvar = Red_Nvar/Nvar_original*100;
Fraction_red_Nineq = Red_Nineq/Nineq_original*100;
Fraction_red_Neq = Red_Neq/Neq_original*100;

figure(1)
hold on
grid
plot(n,Fraction_red_Nvar)
plot(n,Fraction_red_Nineq)
plot(n,Fraction_red_Neq)
legend('Variables','Inequality constraints','Equality constraints')
title('Effect of using DR constraints only on $$Q_n$$','interpreter','latex')
ylim([-15 100])
xlabel('$$|Q_n|$$','interpreter','latex')
ylabel('Reduction (%)')

set(gca,'FontSize',14)

%% LP reduction (New formulation without L constant)

Nq = 1600;
s = 9;
dist_from_samples = 4;
% n = (sqrt(s) + 2*dist_from_samples)^2
n = 1:1:Nq-1;

Nvar_original = Nq*s+Nq+s;
Nineq_original = 2*s + Nq*(s) + 1;
Neq_original = 1+Nq+s;

Nvar_new = (n+1)*s+Nq+s;
Nineq_new = 2*s + n*(s) + 1 +Nq-n;
Neq_new = 2+Nq+s + 0*n;

Red_Nvar = Nvar_new - Nvar_original;
Red_Nineq = Nineq_new - Nineq_original;
Red_Neq = Neq_new - Neq_original;

Fraction_red_Nvar = -Red_Nvar/Nvar_original*100;
Fraction_red_Nineq = -Red_Nineq/Nineq_original*100;
Fraction_red_Neq = -Red_Neq/Neq_original*100;

figure(1)
hold on
grid
plot(n,Fraction_red_Nvar)
plot(n,Fraction_red_Nineq)
plot(n,Fraction_red_Neq)
legend('Variables','Inequality constraints','Equality constraints')
title('Reduction in complexity by using set $$\widetilde\Gamma_q^a$$','interpreter','latex')
ylim([-15 100])
xlabel('$$|Q_n|$$','interpreter','latex')
ylabel('Reduction (%)')

set(gca,'FontSize',14)

%% LP reduction (New formulation with d_{is})

Nq = 1601;
s = 9;
dist_from_samples = 5;
% n = (sqrt(s) + 2*dist_from_samples)^2
n = 1:1:Nq-1;

Nvar_original = Nq*s;%Nq*s+Nq+s;
Nineq_original = 2*s + Nq*(s) + 1;
Neq_original = 1;%1+Nq+s;

Nvar_new = (n+1)*s;%(n+1)*s+s+Nq;
Nineq_new = (n+3)*s + 1;
Neq_new = 2+0*n;%2+n+s+0*n;

Red_Nvar = Nvar_new - Nvar_original;
Red_Nineq = Nineq_new - Nineq_original;
Red_Neq = Neq_new - Neq_original;

Fraction_red_Nvar = -Red_Nvar/Nvar_original*100;
Fraction_red_Nineq = -Red_Nineq/Nineq_original*100;
Fraction_red_Neq = -Red_Neq/Neq_original*100;

figure(1)
hold on
grid
plot(n,Fraction_red_Nvar)
plot(n,Fraction_red_Nineq)
plot(n,Fraction_red_Neq)
legend('Variables','Inequality constraints')
title('Reduction in complexity by using set $$\Gamma_{q,a}$$','interpreter','latex')
ylim([-15 100])
xlabel('$$|Q_n|$$','interpreter','latex')
ylabel('Reduction (%)')

set(gca,'FontSize',14)

%% Test affine under and ovarapproximators in 1D (sine)

q = 1;
c = 1;
f_is_sine = 1;

a_rectangle = state_discretization.vertices_q{q}(:,1);
b_rectangle = state_discretization.vertices_q{q}(:,7);
a = pi/2-0.2;%a_rectangle(2);
b = pi/2+0.1;%b_rectangle(2);

x = linspace(a_rectangle(3),b_rectangle(3),100000);
y = c*sin(x);

[m_under,m_over,n_under,n_over] = affine_under_and_overapproximate_sinusoidal(a_rectangle(3),b_rectangle(3),f_is_sine);

y_under = m_under*x + n_under;
y_over = m_over*x + n_over;

figure(1)
hold on
grid
plot(x,y)
plot(x,y_under)
plot(x,y_over)
legend('f','under','over')
% plot(a,c*sin(a),'s-')
% plot(b,c*sin(b),'s-')
% plot(a,c*sin(a),'s-')
% 
plot(a_rectangle(3),c*sin(a_rectangle(3)),'o')
plot(b_rectangle(3),c*sin(b_rectangle(3)),'x')

if (min(min(y-y_under)) < -1e-5)||(min(min(y_over-y)) < -1e-5)
    disp('Error')
end


%% Test affine under and ovarapproximators in 1D (cos)

q = 1;
c = 1;
f_is_sine = 0;

a_rectangle = state_discretization.vertices_q{q}(:,1);
b_rectangle = state_discretization.vertices_q{q}(:,7);
a = pi/2+0.1;%a_rectangle(2);
b = pi/2+0.6;%b_rectangle(2);

x = linspace(a_rectangle(3),b_rectangle(3),1000);
y = c*cos(x);

% [m_under,m_over,n_under,n_over] = affine_under_and_overapproximate_sinusoidal(a,b,f_is_sine);
[m_under,m_over,n_under,n_over] = affine_under_and_overapproximate_sinusoidal(a_rectangle(3),b_rectangle(3),f_is_sine);

y_under = m_under*x + n_under;
y_over = m_over*x + n_over;

figure(1)
hold on
grid
plot(x,y)
plot(x,y_under)
plot(x,y_over)
legend('f','under','over')
% plot(a,c*sin(a),'s-')
% plot(b,c*sin(b),'s-')
% plot(a,c*sin(a),'s-')
% 
plot(a_rectangle(3),c*cos(a_rectangle(3)),'o')
plot(b_rectangle(3),c*cos(b_rectangle(3)),'x')

if (min(min(y-y_under)) < -1e-5)||(min(min(y_over-y)) < -1e-5)
    disp('Error')
end


%% Test JacksonNL

a = 4;
a_rectangle = state_discretization.vertices_q{end}(:,1);
b_rectangle = state_discretization.vertices_q{end}(:,3);
% a = [0,0]';
% b = [1,1]';
[X,Y] = meshgrid(a_rectangle(1):0.001:b_rectangle(1),a_rectangle(2):0.001:b_rectangle(2));
X = X(:);
Y = Y(:);
x1 = zeros(2,length(X));
for k = 1:length(X)
    x = [X(k) Y(k)]';
    x1(:,k) = JacksonNL(x,a);
end
figure(1)
hold on
grid
plot(X,Y,'.')
plot(x1(1,:),x1(2,:),'.')

%% Test affine approximators in 3D

for q = 1:user_param.Nq
    a_rectangle = state_discretization.vertices_q{q}(:,1);
    b_rectangle = state_discretization.vertices_q{q}(:,3);
    
    [X,Y] = meshgrid(a_rectangle(1):0.03:b_rectangle(1),a_rectangle(2):0.03:b_rectangle(2));
    X = X(:);
    Y = Y(:);
    for a = 1:4
        f = zeros(2,length(X));
        f_under = zeros(2,length(X));
        f_over = zeros(2,length(X));
        [n_f1,d_under_f1,d_over_f1,n_f2,d_under_f2,d_over_f2] = affine_under_and_overapproximate_JacksonNL(a_rectangle,b_rectangle,a);
        for k = 1:length(X)
            x = [X(k) Y(k)]';
            xi = [0 0]';
            f(:,k) = JacksonNL(x,a,xi);
            [f_over(1,k),f_under(1,k)] = f_overapproximation(n_f1,d_over_f1,d_under_f1,x);
            [f_over(2,k),f_under(2,k)] = f_overapproximation(n_f2,d_over_f2,d_under_f2,x);
        end
        if (min(min(f-f_under)) < -1e-4)||(min(min(f_over-f)) < -1e-4)
            disp('Error')
            break
        else
            flag = 1;
            plot3(X,Y,f(1,:),'.')
            hold on
            plot3(X,Y,f_under(1,:),'s')
            plot3(X,Y,f_over(1,:),'o')
        end
    end
end
if (flag == 1)
    disp('Good bounds')
end
% plot3(X,Y,f(1,:),'.')
% hold on
% grid
% plot3(X,Y,f_under(1,:),'s')
% plot3(X,Y,f_over(1,:),'o')

%% Compute R_app_q_a

state_discretization.Reach_app_q_a = cell(user_param.Nq,user_param.Na);
flag = 1;
for q = 1:user_param.Nq
    a_rectangle = state_discretization.vertices_q{q}(:,1);
    b_rectangle = state_discretization.vertices_q{q}(:,3);
    [X,Y] = meshgrid(a_rectangle(1):0.01:b_rectangle(1),a_rectangle(2):0.01:b_rectangle(2));
    X = X(:);
    Y = Y(:);
    
    for a = 1:4
        state_discretization.Reach_app_q_a{q,a} = zeros(2,16);
        f = zeros(2,length(X));
        [n_f1,d_under_f1,d_over_f1,n_f2,d_under_f2,d_over_f2] = affine_under_and_overapproximate_JacksonNL(a_rectangle,b_rectangle,a);
        for k = 1:4
            v = state_discretization.vertices_q{q}(:,k);
            [f_over_1,f_under_1] = f_overapproximation(n_f1,d_over_f1,d_under_f1,v);
            [f_over_2,f_under_2] = f_overapproximation(n_f2,d_over_f2,d_under_f2,v);
            state_discretization.Reach_app_q_a{q,a}(:,4*(k-1)+1 : 4*k) = [f_under_1 f_over_1 f_over_1 f_under_1
                                                     f_under_2 f_under_2 f_over_2 f_over_2];
        end
        for k = 1:length(X)
            x = [X(k) Y(k)]';
            f(:,k) = JacksonNL(x,a);
        end
%                 figure(1)
%         hold on;grid
%         plot(state_discretization.Reach_app_q_a{q,a}(1,:),state_discretization.Reach_app_q_a{q,a}(2,:),'s')
%         plot(f(1,:),f(2,:),'.')
        in1 = inhull(f',state_discretization.Reach_app_q_a{q,a}',[],1e-5);
        if (min(in1) ~= 1)
            disp('Error generating R_app_q_a. Maybe increment tolerance')
            flag = 0;
            break
        end
    end
end
if (flag == 1)
    disp('Good approximation of the Reachable set')
end

%% Test RIVI


[V1_q,x] = RIVI_inner_minimization(V,A,b,Aeq,beq,f_aux_cost);