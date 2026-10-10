function x1 = unicycle2D_ltlf(x,u,w,original_sys,user_param)

x1 = [0;0];

if(length(u) == 1) % Only control heading
    x1(1) = x(1) + user_param.Deltat*( user_param.Umax_vel - original_sys.cd*w )*cos(u(1));
    x1(2) = x(2) + user_param.Deltat*( user_param.Umax_vel - original_sys.cd*w )*sin(u(1));
else % Control heading and velocity
    x1(1) = x(1) + user_param.Deltat*( u(1) - original_sys.cd*w )*cos(u(2));
    x1(2) = x(2) + user_param.Deltat*( u(1) - original_sys.cd*w )*sin(u(2));
end

end