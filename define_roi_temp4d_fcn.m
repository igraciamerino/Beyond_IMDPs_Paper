function [user_param, state_discretization] = define_roi_temp4d_fcn(user_param, state_discretization, solver_opts, original_sys)

user_param.S = 1:user_param.Ns + 1;
user_param.S_terminal = user_param.Ns + 1;
user_param.S_not_terminal = user_param.S(~ismember(user_param.S,user_param.S_terminal));

end