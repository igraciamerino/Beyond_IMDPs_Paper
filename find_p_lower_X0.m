function p_lower_X0 = find_p_lower_X0(p_lower, state_discretiation, original_sys, user_param, solver_opts)

% First, shrink X0 bc the intersection functions inflate it
a_X0 = original_sys.a_X0 + 2*solver_opts.precision;
b_X0 = original_sys.b_X0 - 2*solver_opts.precision;

if (strcmp(original_sys.name, 'fm1'))
    [S0, ~] = intersections_simplified_fb1_fcn(a_X0, b_X0, state_discretiation.Deltax, user_param.Nx, user_param.Ns, original_sys.state_space_lower_left_corner, original_sys.state_space_upper_right_corner, solver_opts.precision);
elseif (strcmp(original_sys.name, 'fm2'))
    [S0, ~] = intersections_simplified_temp_n_dim_fcn(a_X0, b_X0, state_discretiation.Deltax, user_param.Nx, user_param.Ns, original_sys.state_space_lower_left_corner, original_sys.state_space_upper_right_corner, solver_opts.precision, original_sys.n);
else
    S0 = [];
end
p_lower_X0 = min(p_lower(S0));

end