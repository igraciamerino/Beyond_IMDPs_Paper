function [samples, user_param] = generate_samples_heavy_tail(original_sys, user_param, plotsamples)
% Generates samples and over-approximating set

N = user_param.N;
% Xi_hat = zeros(N, original_sys.d)';
% 
% Xi_hat = zeros(2, N);
% rs = original_sys.weight_samples*abs(randn(1, N)).^(1.2);
% th = 2*pi*(rand(1, N) - 0.5);
% for w_ind = 1:N
%     Xi_hat(:, w_ind) = rs(w_ind)*[sin(th(w_ind)); cos(th(w_ind))];
% end


Xi_hat = original_sys.weight_samples*mvtrnd(eye(2),10, N)';
% This is in case that I want to truncate the distribution on a hypercube
% for w_ind = 1:N
%     w = Xi_hat_aux(:, w_ind);
%     if ( norm( w, inf) < rho )
%         Xi_hat = [Xi_hat w];
%     end
% end



%Compute convex hull of samples
% vertices_samples = convhull(Xi_hat');
% [samples.A,samples.b] = vert2con(Xi_hat');
% Visualize samples
% if (plotsamples == 1)
%     plot_samples_fcn(Xi_hat, [0.0 0.0]', 0, original_sys, user_param)
% %     plot(Xi_hat(1,vertices_samples),Xi_hat(2,vertices_samples))
% end
% [samples.center_ball_Xi_hat, samples.radius_ball_Xi_hat] = over_approximating_ball(Xi_hat);

samples.Xi_hat = Xi_hat;

end

