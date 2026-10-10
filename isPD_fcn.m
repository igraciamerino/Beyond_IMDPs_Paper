function yn = isPD_fcn(A)
tol = 1e-9;
[n, m] = size(A);
yn = ( ~any(eig(A) < 0) && ~any(any(abs(A - A') > tol)) && (n == m) );
end