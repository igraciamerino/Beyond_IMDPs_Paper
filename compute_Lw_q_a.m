function Lw_q_a = compute_Lw_q_a(original_sys, user_param, Act)
%
Nq = user_param.Nq;
Na = user_param.Na;

Deltat = user_param.Deltat;
cd = original_sys.cd;

Lw_q_a = cell(Nq,Na);
for a = 1:Na
    u = Act(end, a);
    L_candidate = Deltat*cd*max( abs(cos(u)), abs(sin(u)) );
    for q = 1:Nq
        Lw_q_a{q,a} = L_candidate;
    end
end

end