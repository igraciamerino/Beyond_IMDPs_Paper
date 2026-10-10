function original_sys = compute_lipschitz_constants_mult_noise_fcn(original_sys, user_param, state_discretization)

%
Ns = user_param.Ns;
Na = user_param.Na;
A = original_sys.A;
vertices_s = state_discretization.vertices_s;

Lw_s_a = cell(Ns, Na);
for s = 1:Ns
    L_candidate = zeros(4,1);
    for k = 1:4
        v_k = vertices_s{s}(:,k);
        xp = abs(A*v_k);
        L_candidate(k) = max(xp);
    end
    Lw_s_a_aux = max(L_candidate);
    if (Lw_s_a_aux > norm(A*[2;2],'inf') )
        disp('Error computing Lipschitz constants')
    end
    for a = 1:Na
        Lw_s_a{s,a} = Lw_s_a_aux;
    end
end
original_sys.Lw_s_a = Lw_s_a;

end