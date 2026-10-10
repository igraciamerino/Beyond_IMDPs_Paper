function x1 = fm2_sys(x,u,w,original_sys,user_param)

x1 = original_sys.A*x + original_sys.cd*w;

end