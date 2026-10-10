function samples = generate_samples_MC_fcn(original_sys, disturbance_discretization, user_param, plotsamples, solver_opts)

user_param_dummy = user_param;
user_param_dummy.N = user_param.MC_timeout;
% Generates samples

if ( strcmp( original_sys.Pw_class, 'gaussian') )
    [samples] = generate_samples_gaussian(original_sys, user_param_dummy, plotsamples);
elseif ( strcmp( original_sys.Pw_class, 'heavy_tail') )
    [samples] = generate_samples_heavy_tail(original_sys, user_param_dummy, plotsamples);
% elseif ( strcmp( original_sys.name, 'mult_noise') )
%     [samples] = generate_samples_mult_noise(original_sys, user_param, plotsamples);
end

end

