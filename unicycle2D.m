function x1 = unicycle2D(x,u,w,original_sys,user_param)

x1 = [0;0];

x1(1) = x(1) + user_param.Deltat*user_param.Umax_vel*cos(u(1)) + original_sys.cd*w(1);
x1(2) = x(2) + user_param.Deltat*user_param.Umax_vel*sin(u(1)) + original_sys.cd*w(2);

end