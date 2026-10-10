function [L_q_a] = estimate_lipschitz_constants(original_sys,state_discretization, user_param, Act)
%
Nq = user_param.Nq;
Na = user_param.Na;
X_center = state_discretization.X_center;
Y_center = state_discretization.Y_center;
TH_center = state_discretization.TH_center;
Reach_app_q_a = original_sys.Reach_app_q_a;
delta_q = state_discretization.delta_q;
vel = original_sys.vel;

L_q_a = cell(Nq,Na);
for q = 1:Nq
    for a = 1:Na
        x = [X_center(q) Y_center(q) TH_center(q)]';
        u = Act(a);
        xi = [0 0 0]';
        x1 = unicycle3D(x,u,xi,vel);
        v = Reach_app_q_a{q,a};
        rad = 0;
        for k = 1:size(v,2)
            diff = x1 - v(:,k);
            rad = max(norm(diff,2),rad);
        end
        L_q_a{q,a} = rad/delta_q;
    end
end

end

