function [] = plot_samples_pendulum_fcn(Xi_hat,x,u,original_sys,user_param)

N = size(Xi_hat,2);
x_star = zeros(original_sys.n,N);

hold on
for i = 1:N
    w = Xi_hat(:,i);
    x_star(:,i) = pendulum(x, u, w, original_sys, user_param);
    plot(x_star(1,i), x_star(2,i), 'gx')
    % plot([x(1) x_star(1,i)],[x(2) x_star(2,i)], 'gx')
end

end