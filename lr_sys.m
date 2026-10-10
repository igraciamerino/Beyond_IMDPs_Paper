function x1 = lr_sys(x,u,w,original_sys)

% On-off controller
% if ( x > original_sys.state_space_middle )
%     u = 0;
% else
%     u = 1;
% end

x1 = original_sys.A*x + original_sys.bu*u + original_sys.b + w(1);

end