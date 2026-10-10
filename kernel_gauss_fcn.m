function K = kernel_gauss_fcn(X, Y, sigmal, sigmaf)
% Syntax:
%   K = gaussian_kernel_cross(X, Z, sigma)
%
% Inputs:
%   X     - (n x m) data matrix
%   Z     - (n x p) data matrix
%   sigma - bandwidth of the Gaussian kernel
%
% Output:
%   K - (m x p) kernel matrix where K(i,j) = exp(-||X(i,:) - Z(j,:)||^2 / (2*sigma^2))

% Compute squared norms
X_norm = sum(X.^2, 1);   % (m x 1)
Y_norm = sum(Y.^2, 1);   % (p x 1)

% Compute squared distances
D = bsxfun(@plus, X_norm', Y_norm) - 2 * (X' * Y);  % (m x p)

% Compute kernel matrix
K = sigmaf^2*exp(-D / (2 * sigmal^2));
end