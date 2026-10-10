function [a, b, wx, wy] = state_block2rectangle_fcn(block, X_center, Y_center, Deltax, Deltay)

s1 = block(1);
s2 = block(end);

a = [X_center(s1); Y_center(s1)] - [Deltax/2; Deltay/2];
b = [X_center(s2); Y_center(s2)] + [Deltax/2; Deltay/2];

wx = b(1) - a(1);
wy = b(2) - a(2);

end