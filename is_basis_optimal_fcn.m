function result = is_basis_optimal_fcn(V, B_transp, N_transp, is_in_base, B_is_square_and_full_rank)

% WARNING: ASSUMES lb = 0

if ( ~B_is_square_and_full_rank )
    result = false;
    return;
end

% Resize V to take into account slack variables

V = [V; zeros(size(B_transp, 2), 1)];

% Reorder V

c_B = V(is_in_base);
c_N = V(~is_in_base);

% Obtain Lagrange multipliers

lambda = (B_transp)\c_B;
s_N = c_N - N_transp*lambda;

% Check s_N is non-negative

if ( any( s_N < 0 ) )
    result = false;
else
    result = true;
end

%change_in_optimizer

end