function original_sys = compute_lipschitz_constants_unicycle4d_nl_friction_fcn(original_sys, user_param, state_discretization)

%
Ns = user_param.Ns;
Na = user_param.Na;

Deltat = user_param.Deltat;
cd = original_sys.cd;

Lw_s_a = cell(user_param.Ns, user_param.Na);
for s = 1:Ns
    q_lb = state_discretization.vertices_s{s}(:,1);
    q_ub = state_discretization.vertices_s{s}(:,8);
    th_lb = 2*pi*q_lb(3) - pi;
    th_ub = 2*pi*q_ub(3) - pi;
    L_candidate = [];
    for a = 1:Na
        if ( ( th_lb == -pi ) || ( th_ub == pi ) )
            L_candidate = [L_candidate cd(1)];
        elseif ( ( th_lb < -pi/2 ) && ( th_ub > -pi/2 ) )
            L_candidate = [L_candidate cd(1)];
        elseif ( ( th_lb < 0 ) && ( th_ub > 0 ) )
            L_candidate = [L_candidate cd(1)];
        elseif ( ( th_lb < pi/2 ) && ( th_ub > pi/2 ) )
            L_candidate = [L_candidate cd(1)];
        else
            L_candidate = [L_candidate cd(1)*max(abs(cos(th_lb)), abs(cos(th_lb))) cd(1)*max(abs(sin(th_lb)), abs(sin(th_lb)))];
        end

        L_candidate = [L_candidate cd(2) cd(3)];

        Lw_s_a{s,a} = Deltat*max(L_candidate);
        if (Lw_s_a{s,a} > Deltat*max(cd) )
            disp('Error computing Lipschitz constants')
        end
    end
end

original_sys.Lw_s_a = Lw_s_a;
% original_sys.max_Lw_s_a = max_Lw_s_a;

end