function xp = pendulum(x,u,w,original_sys,user_param)

Deltat = user_param.Deltat;
th_dot_max = user_param.th_dot_max;
M = original_sys.M;
l = original_sys.l;
cd = original_sys.cd;
g = original_sys.g;

th = 2*pi*x(1) - pi;
th_dot = 2*th_dot_max*x(2) - th_dot_max;

% Energy-based controller
% u = user_param.Umax*sign(th_dot);

th_p = th + Deltat*th_dot;
% Linear drag
% th_dot_p = th_dot + Deltat*( -cd/(M*L^2)*(L*th_dot - w*cos(th))*abs(L*th_dot - w*cos(th)) - g/(L^2)*sin(th) + u/(M*L^2));
% Quadratic drag
th_dot_p = th_dot + Deltat*( -cd/(M*l^2)*(l*th_dot - w*cos(th))*abs(l*th_dot - w*cos(th)) - g/l*sin(th) + u/(M*l^2) );

xp = zeros(2,1);
xp(1) = (th_p + pi)/(2*pi);
xp(2) = (th_dot_p + th_dot_max)/(2*th_dot_max);

end