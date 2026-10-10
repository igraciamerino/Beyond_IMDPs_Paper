function [B_transp, N_transp, A1, is_in_base, B_is_square_and_full_rank] = compute_basis_stacked_fcn(A, b, gamma)

% WARNING: ASSUMES ROWS OF A ARE LI
% WARNING: ASSUMES lb = 0
 
tol = 1e-10;

% Define slack variables

slack = b - A*gamma;
is_tight = (slack < 0);
slack(is_tight) = 0;

A = [A eye(length(slack))];

x = [gamma; slack];

% Check A is full rank
if ( rank( full(A) ) ~= size(A, 1) )
    B_is_square_and_full_rank = false;
    is_in_base = [];
    B_transp = [];
    N_transp = [];
    A1 = [];
    return;
end

% Find indices of active constraints

is_in_base = (x > 0);

B = A(:, is_in_base);

%if ( size(B, 1) ~= size(B, 2) ) || ( rank( full(B) ) ~= size(B, 2) )
if ( norm( B\b - x( is_in_base) ) > tol )
    B_is_square_and_full_rank = false;
    B_transp = [];
    N_transp = [];
    A1 = [];
    return;
else
    B_is_square_and_full_rank = true;
end

N = A(:, ~is_in_base);
A1 = [B N];

B_transp = B';
N_transp = N';

end