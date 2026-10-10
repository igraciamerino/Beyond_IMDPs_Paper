function x1 = fm1_sys(x,u,w,d,original_sys,user_param)

x1 = x + d + original_sys.cd*w(1);

end