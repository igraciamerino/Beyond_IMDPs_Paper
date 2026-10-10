function [] = plot_samples_temp_fcn(Xi_hat,x,u,original_sys,user_param)

N = size(Xi_hat,2);
x_star = zeros(original_sys.n,N);

hold on
for i = 1:N
    w = Xi_hat(:,i);
    x_star(:,i) = temp_sys(x, u, w, original_sys, user_param);
    if ( ( original_sys.n == 2 ) || ( original_sys.n == 4 ) )
        plot(x_star(1,i), x_star(2,i), 'gx')
    elseif ( original_sys.n == 3 )
        plot3(x_star(1,i), x_star(2,i), x_star(3,i), 'gx')
    end
end

end