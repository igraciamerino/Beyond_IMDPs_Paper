function [user_param, state_discretization] = define_roi_lr_fcn(user_param, state_discretization, solver_opts, original_sys)

aX = original_sys.state_space_safe_set_lower_left_corner;
bX = original_sys.state_space_safe_set_upper_right_corner;
is_terminal = false(1, user_param.Ns + 1);
for s = 1:user_param.Ns
    x = state_discretization.X_center{1}(s);
    if ( ( x < aX) || ( x > bX) )
        is_terminal(s) = true;
    end
end
is_terminal(end) = true;

user_param.S = 1:user_param.Ns + 1;
user_param.S_terminal = user_param.S(is_terminal);
user_param.S_not_terminal = user_param.S(~is_terminal);

end