function [samples, user_param] = generate_samples_gaussian(original_sys, user_param, plotsamples)
% Generates samples and over-approximating set

N = user_param.N;
rho = original_sys.rho;
center_samples = original_sys.center_samples;
% If Gaussian mixture
%scaling_factor = original_sys.weight_samples*user_param.n_std_truncate+ 0.01;
%gm = gmdistribution( [0.01 0;-0.01 0]/scaling_factor, (original_sys.weight_samples/scaling_factor)^2*ones(1,2) );
%Xi_hat_aux = random(gm,N)';
% If Gaussian
Xi_hat_aux = original_sys.weight_samples*randn(N, original_sys.d)';

Xi_hat = [];
for w_ind = 1:N
    w = Xi_hat_aux(:, w_ind);
    if ( w'*w < (rho)^2 )
        Xi_hat = [Xi_hat, w];
    end
end
Xi_hat = Xi_hat + center_samples;

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

% Consider only the samples inside the bound
user_param.N = size(samples.Xi_hat, 2);


end

