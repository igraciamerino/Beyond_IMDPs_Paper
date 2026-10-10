function original_sys = compute_lipschitz_constants_fcn(original_sys, user_param, state_discretization)

if ( strcmp( original_sys.name, 'unicycle2d') || strcmp( original_sys.name, 'fm2') )
    original_sys = compute_lipschitz_constants_unicycle2d_fcn(original_sys, user_param);
elseif ( strcmp( original_sys.name, 'unicycle3d_nl_friction') )
    original_sys = compute_lipschitz_constants_unicycle3d_nl_friction_fcn(original_sys, user_param, state_discretization);
elseif ( strcmp( original_sys.name, 'unicycle4d_nl_friction') )
    original_sys = compute_lipschitz_constants_unicycle4d_nl_friction_fcn(original_sys, user_param, state_discretization);
elseif ( strcmp( original_sys.name, 'pendulum') )
    original_sys = compute_lipschitz_constants_pendulum_fcn(original_sys, user_param, state_discretization);
elseif ( strcmp( original_sys.name, 'mult_noise') )
    original_sys = compute_lipschitz_constants_mult_noise_fcn(original_sys, user_param, state_discretization);
elseif ( strcmp( original_sys.name, 'unicycle2d_ltlf') )
    original_sys = compute_lipschitz_constants_unicycle2d_ltlf_fcn(original_sys, user_param, state_discretization);
elseif ( strcmp( original_sys.name, 'temp') )
    original_sys = compute_lipschitz_constants_temp_fcn(original_sys, user_param, state_discretization);
elseif (strcmp( original_sys.name, 'fm1'))
    original_sys = compute_lipschitz_constants_unicycle2d_fcn(original_sys, user_param);
elseif (strcmp( original_sys.name, 'lr'))
    original_sys = compute_lipschitz_constants_lr_fcn(original_sys, user_param);
end

end