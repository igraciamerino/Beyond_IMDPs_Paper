p = 1;
d = 3;
m = 1;
beta = 10^(-9);
Ns = logspace(2,7,1000);
diam_inf = 0.025;
%% Dimitris' paper

C_star = sqrt(d)*2^((d-2)/(2*p))*(1/(1-2^(p-d/2)) + 1/(1-2^(-p)))^(1/p);
rho = 1/2*diam_inf;

epsilons = 2*rho*( C_star*Ns.^(-1/d) + sqrt(d)*(2*log(1/beta))^(1/(2*p))*Ns.^(-1/(2*p)) );

figure
loglog(Ns,epsilons)
hold on
grid

%% Dimitris' paper (more conservative formulas)

% C_STAR = (C_star^d)/(2*d^(d/2));
% c_STAR = 1/(2^d*d^(d/2));
% 
% epsilons = 2*rho*(log(C_STAR/beta)/c_STAR)^(1/d)*Ns.^(-1/d);
% 
% loglog(Ns,epsilons)
% hold on
% grid

%% Fournier's paper

if (d == 3)
    C_star = d^(p/m)*3.72;
elseif (d == 8)
    C_star = d^(p/m)*1.84;
end

epsilons = 2*rho*( C_star*Ns.^(-1/d) + sqrt(d)*(2*log(1/beta))^(1/(2*p))*Ns.^(-1/(2*p)) );
loglog(Ns,epsilons)
hold on
grid

%% d = 2 = 2*p;

% Dimitris'

d = 3;

C_star = sqrt(d)*2^((d-2)/(2*p))*(1/(1-2^(p-d/2)) + 1/(1-2^(-p)))^(1/p);
rho = 1/2*diam_inf;

epsilons = 2*rho*( C_star*Ns.^(-1/d) + sqrt(d)*(2*log(1/beta))^(1/(2*p))*Ns.^(-1/(2*p)) );

figure
loglog(Ns,epsilons)
hold on

% Fournier's

d = 2;

C_stars = d^(p/m)*(0.73*log(Ns)+1);

epsilons = 2*rho*( C_stars.*Ns.^(-1/d) + sqrt(d)*(2*log(1/beta))^(1/(2*p))*Ns.^(-1/(2*p)) );

loglog(Ns,epsilons)
hold on
grid


% In the case of m = p = 2, if T_p^(m) was used instead of W_p, the
% constant would be reduced by a factor of sqrt(2), which means N reduced
% by a factor of 2^(d/2) = 2 (in 2D)

