function original_sys = compute_lipschitz_constants_unicycle2d_ltlf_fcn(original_sys, user_param, state_discretization)

Ns = user_param.Ns;
Na = user_param.Na;

disp('THIS IS NOT LTLF')
return;

Lw_s_a = cell(Ns,Na);
for s = 1:Ns
    vs = vertices_s{s};
    L_candidate = zeros(1, size(vs, 2));
    for i = 1:size(vs, 2)
        v = vs(:, i);
        L_candidate(i) = norm( original_sys.A*v , "inf");
    end
    for a = 1:Na
        Lw_s_a{s,a} = max( L_candidate );
    end
end

original_sys.Lw_s_a = Lw_s_a;

end