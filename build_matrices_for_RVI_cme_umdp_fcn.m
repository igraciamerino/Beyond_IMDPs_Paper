function [q, rhs] = build_matrices_for_RVI_cme_umdp_fcn(beta, Sp, x_center, Ns, sigmaf, sigmal, epsilon)

Q_2 = kernel_gauss_fcn(x_center, x_center(Sp), sigmal, sigmaf);
Q_3 = kernel_gauss_fcn(x_center(Sp), x_center(Sp), sigmal, sigmaf);

Q_3 = (Q_3 + Q_3')/2;

                           % Quad term in quad constraint
q = -2*Q_2*beta;                    % Linear term in quadratic constraint
% q = q';
r = beta'*Q_3*beta;                           % Constant offset in quadratic constraint
rhs = epsilon^2 - r;                         % Right-hand side of constraint

% Linear constraints
% sum(x) = 1
% Aeq = ones(1, Ns);
% beq = 1;

end

