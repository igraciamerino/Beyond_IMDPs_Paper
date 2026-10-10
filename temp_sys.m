function x1 = temp_sys(x,u,w,original_sys,user_param)

    x1 = diag(1 + w)*original_sys.A*x + original_sys.b + original_sys.bu*u;

end