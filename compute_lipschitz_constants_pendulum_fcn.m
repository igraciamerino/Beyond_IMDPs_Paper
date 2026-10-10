function original_sys = compute_lipschitz_constants_pendulum_fcn(original_sys, user_param, state_discretization)

%
Ns = user_param.Ns;
Na = user_param.Na;

Deltat = user_param.Deltat;
M = original_sys.M;
l = original_sys.l;
cd = original_sys.cd;
rho = original_sys.rho;
th_dot_max = user_param.th_dot_max;

Lw_s_a = cell(Ns,Na);
for s = 1:Ns
    q_lb = state_discretization.vertices_s{s}(:, 1);
    q_ub = state_discretization.vertices_s{s}(:, end);
    th_lb = 2*pi*q_lb(1) - pi;
    th_ub = 2*pi*q_ub(1) - pi;
    th_dot_lb = 2*th_dot_max*q_lb(2) - th_dot_max;
    th_dot_ub = 2*th_dot_max*q_ub(2) - th_dot_max;
    L_candidate = [];
    for a = 1:Na
        % 1) th_lb == -pi
        if ( ( th_lb == -pi ) || ( th_ub == pi ) || ( ( th_lb <= 0 ) && ( th_ub >= 0 ) ) )
            L_candidate = [L_candidate -2*(l*th_dot_lb - rho)];
            L_candidate = [L_candidate 2*(l*th_dot_ub + rho)];
        else % Check values at vertices
            L_candidate = [L_candidate 2*abs( rho*(cos(th_lb))^2 - l*th_dot_lb*cos(th_lb))];
            L_candidate = [L_candidate 2*abs( rho*(cos(th_ub))^2 - l*th_dot_lb*cos(th_ub))];
            L_candidate = [L_candidate 2*abs( rho*(cos(th_lb))^2 - l*th_dot_ub*cos(th_lb))];
            L_candidate = [L_candidate 2*abs( rho*(cos(th_ub))^2 - l*th_dot_ub*cos(th_ub))];
            L_candidate = [L_candidate 2*abs( -rho*(cos(th_lb))^2 - l*th_dot_lb*cos(th_lb))];
            L_candidate = [L_candidate 2*abs( -rho*(cos(th_ub))^2 - l*th_dot_lb*cos(th_ub))];
            L_candidate = [L_candidate 2*abs( -rho*(cos(th_lb))^2 - l*th_dot_ub*cos(th_lb))];
            L_candidate = [L_candidate 2*abs( -rho*(cos(th_ub))^2 - l*th_dot_ub*cos(th_ub))];
        end
        Lw_s_a{s,a} = 1/(2*th_dot_max)*Deltat*cd/(M*l^2)*max(L_candidate);
        if (Lw_s_a{s,a} > 1/(2*th_dot_max)*Deltat*cd/(M*l^2)*2*(rho + l*max(abs(th_dot_lb), abs(th_dot_ub))))
            disp('Error computing Lipschitz constants')
        end
    end
end
original_sys.Lw_s_a = Lw_s_a;


end