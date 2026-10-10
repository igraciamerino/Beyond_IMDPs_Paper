function [A,b] = matrices_polytope(a,b)
% Compute the matrices A and b that represent the polytope A<=b

% A = [-1 0;0 1;1 0;0 -1];
% b = [-x_lower_left y_upper_right x_upper_right -y_lower_left]';

% A = kron(eye(3),[-1;1]);
% b = [-a(1),b(1),-a(2),b(2),-a(3),b(3)]';

A = kron(eye(2),[-1;1]);
b = [-a(1),b(1),-a(2),b(2)]';

end