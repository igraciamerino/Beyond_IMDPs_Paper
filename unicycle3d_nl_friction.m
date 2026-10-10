function x1 = unicycle3d_nl_friction(x,u,w,original_sys,user_param)

th = 2*pi*x(3)-pi;

x1 =zeros(3,1);
x1(1) = x(1) + user_param.Deltat*( u(1) - original_sys.cd(1)*w(1) )*cos(th) ;
x1(2) = x(2) + user_param.Deltat*( u(1) - original_sys.cd(1)*w(1) )*sin(th);
x1(3) = x(3) + user_param.Deltat*( u(2)/(2*pi) + original_sys.cd(2)*w(2) );

end