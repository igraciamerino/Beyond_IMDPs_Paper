function [samples, user_param] = generate_samples_cme_gaussian(original_sys, user_param, plotsamples)
% Generates samples and over-approximating set

N = user_param.N;
rho = original_sys.rho;
center_samples = original_sys.center_samples;
aX = original_sys.state_space_lower_left_corner;
bX = original_sys.state_space_upper_right_corner;
n = original_sys.n;
d = original_sys.d;
Act = original_sys.Act;
Na = user_param.Na;
weight_samples = original_sys.weight_samples;

% Sample current states and disturbance
Wtrain = weight_samples*randn(N, d)';
Ytrain = (bX - aX).*rand(n, N) + aX; % Conditioning variable

Xtrain = cell(Na, 1); % Conditioned variable
for a = 1:Na
    Xtrain{a} = zeros(size(Ytrain));
    u = Act(:, a);
    for i = 1:N
        y = Ytrain(:, i);
        w = Wtrain(:, i);
        xp = lr_sys(y,u,w,original_sys);
        Xtrain{a}(:,i) = xp;
    end
end

samples.Ytrain = Ytrain;
samples.Wtrain = Wtrain;
samples.Xtrain = Xtrain;

end

