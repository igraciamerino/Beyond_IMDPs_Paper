% Parameters
n = 100;        % Number of data points
m = 5;         % Number of landmarks
sigma = 0.2;    % Gaussian bandwidth

% Create 1D grid
X = linspace(0, 1, n)';
Z = X(randperm(n, m));  % Random landmark subset

% Gaussian kernel function
gauss_kernel = @(a, b) exp(-pdist2(a, b).^2 / (2*sigma^2));

% Compute full kernel matrix
K = gauss_kernel(X, X);

% Compute Nyström approximation
Kxz = gauss_kernel(X, Z);         % n × m
Kzz = gauss_kernel(Z, Z);         % m × m
Kzz_inv = pinv(Kzz);              % Pseudoinverse (or regularized inverse)

K_nystrom = Kxz * Kzz_inv * Kxz';

% Check PSD of Nyström kernel
eigenvalues_nystrom = eig(K_nystrom)

% Check that K - K_nystrom is PSD
residual = K - K_nystrom;
min_eig_residual = min(eig(residual));  % Should be ≥ 0 or close (numerical errors)

fprintf('Minimum eigenvalue of K - K_nystrom: %.4e\n', min_eig_residual);
