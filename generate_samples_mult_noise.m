function [samples, user_param] = generate_samples_mult_noise(original_sys, user_param, plotsamples)

d = original_sys.d;
N = user_param.N;
rho = original_sys.rho;

Xi_hat_aux = original_sys.weight_samples*randn(d,N) + 1;
Xi_hat = [];

% This is in case that I want to truncate the distribution as in John's
% paper
for w_ind = 1:N
    w = Xi_hat_aux(:, w_ind);
    if ( norm(w - 1, "inf") < 0.1 ) 
        Xi_hat = [Xi_hat w];
    end
end

samples.Xi_hat = Xi_hat;

end

