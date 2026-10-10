function [] = plot_samples_fcn(Xi_hat,x,u,original_sys,user_param)

if ( strcmp(original_sys.name, 'unicycle2d') )
    plot_samples_unicycle2d_fcn(Xi_hat,x,u,original_sys,user_param);
elseif ( strcmp(original_sys.name, 'unicycle3d_nl_friction') )
    plot_samples_unicycle3d_nl_friction_fcn(Xi_hat,x,u,original_sys,user_param);
elseif ( strcmp(original_sys.name, 'pendulum') )
    plot_samples_pendulum_fcn(Xi_hat,x,u,original_sys,user_param);
elseif ( strcmp(original_sys.name, 'temp') )
    plot_samples_temp_fcn(Xi_hat,x,u,original_sys,user_param);
elseif ( strcmp(original_sys.name, 'fm1') )
    plot_samples_fm1_fcn(Xi_hat,x,u,original_sys,user_param);
elseif ( strcmp(original_sys.name, 'fm2') )
    plot_samples_fm2_fcn(Xi_hat,x,u,original_sys,user_param);
end

end