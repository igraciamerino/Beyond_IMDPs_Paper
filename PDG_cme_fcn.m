function [p_opt, g_opt, g_hist, lambda_opt] = PDG_cme_fcn(c, Q, q, r, max_iters, step_size)
    % Inputs:
    %   c, q : n x 1 vectors
    %   Q    : n x n PSD matrix
    %   r    : scalar
    %   max_iters : number of gradient steps
    %   step_size : gradient ascent step size

    n = length(c);
    lambda = 1.0;     % initialize dual variable (positive scalar)
    g_hist  = zeros(max_iters, 1);
    for iter = 1:max_iters
        % Solve inner problem (primal p) for current lambda
        % p* = argmin_{p in simplex} c'p + lambda (p'Qp + q'p)
        H = 2*lambda*Q;  % factor of 2 for quadprog convention
        f = c + lambda*q;

        % Use MATLAB's quadprog to solve the inner problem
        % Minimize (1/2)*p'Hp + f'p subject to p in simplex
        Aeq = ones(1, n);
        beq = 1;
        lb = zeros(n, 1);
        [p_star, fval] = quadprog(H, f, [], [], Aeq, beq, lb, [], [], optimoptions('quadprog','Display','off'));
        g_hist(iter) = fval - lambda*r;
        % Compute constraint violation at p*
        constraint_value = p_star' * Q * p_star + q' * p_star;

        % Gradient of dual objective w.r.t lambda is constraint violation - r
        grad_lambda = constraint_value - r;

        % Gradient ascent step with projection onto lambda >= 0
        lambda = max(0, lambda + step_size * grad_lambda);
        if ( iter > 1)
            if ( abs(g_hist(iter) - g_hist(iter-1)) <= 1e-3 )
                break;
            end
        end
    end
    if ( iter == max_iters )
        disp('WARNING: PDG DID NOT CONVERGE...')
    end
    p_opt = p_star;
    g_opt = g_hist(iter);
    lambda_opt = lambda;
end