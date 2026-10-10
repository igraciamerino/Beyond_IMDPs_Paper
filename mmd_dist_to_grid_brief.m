function [dist, v_opt] = mmd_dist_to_grid_brief(u, xu, xv, sigmaf, sigmal, kv)

if ( nargin < 6)
    kv = kernel_gauss_fcn(xv', xv', sigmal, sigmaf);
end
ku = kernel_gauss_fcn(xu', xu', sigmal, sigmaf);
kuv = kernel_gauss_fcn(xu', xv', sigmal, sigmaf);
n = length(xv);

linprog_yn = false;
% Linprog
if ( linprog_yn)
    % QP: min 0.5*p'*H*p + f'*p
    H = 2 * kv;
    f = -2 * kuv' * u;

    Aeq = ones(1, n);
    beq = 1;
    lb = zeros(n, 1); % p >= 0

    % Solve QP
    opts = optimoptions('quadprog', 'Display', 'off');
    % p_opt = quadprog(H, f, [], [], Aeq, beq, lb, [], [], opts);
    [v_opt, dist, exitflag] = quadprog(H, f, [], [], Aeq, beq, lb, [], [], opts);

    if ( exitflag ~= 1)
        disp('ERROR: QUADPROG...')
    end

else

    params.outputflag = 0;
    params.Presolve = 1;
    model.Q = sparse(kv);               % Quadratic matrix in constraint
    model.obj = -2 * kuv' * u;
    model.A = sparse(ones(1, n));                % Linear constraints
    model.rhs = 1;
    model.sense = '>';
    result = gurobi(model, params);
    if ~strcmp(result.status, 'OPTIMAL')
        disp('ERROR: INNER MINIMIZATION FAILED...')
    end
    dist = result.objval;
    v_opt = result.x;
end

dist = sqrt( dist + u'*ku*u );

end