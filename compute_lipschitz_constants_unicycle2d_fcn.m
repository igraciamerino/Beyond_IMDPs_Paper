function original_sys = compute_lipschitz_constants_unicycle2d_fcn(original_sys, user_param)

original_sys.Lw_s_a = cell(user_param.Ns, user_param.Na);
max_Lw_s_a = 0;
for s = 1:user_param.Ns
    for a = 1:user_param.Na
        original_sys.Lw_s_a{s,a} = original_sys.cd;
        if ( original_sys.Lw_s_a{s,a} > max_Lw_s_a )
            max_Lw_s_a = original_sys.Lw_s_a{s,a};
        end
    end
end

original_sys.max_Lw_s_a = max_Lw_s_a;
original_sys.max_Lx_s_a = 1;

end