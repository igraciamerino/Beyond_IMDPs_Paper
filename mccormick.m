function [mz_under, mz_over, nz_under, nz_over] = mccormick(a,b)
% McCormick envelopes for z := f(x,y) = x*y
%Points a and b should define the rectangle [a b]\subseteq \R^2.

% w ≥ a(1) y + x a(2) − a(1) a(2);
% 
% w ≥ b(1) y + x b(2) − b(1) b(2);

mz_under = [a(2) a(1)
            b(2) b(1)];
nz_under = [-a(1)*a(2)
            -b(1)*b(2)];

mz_over = [a(2) b(1)
            b(2) a(1)];
nz_over = [-b(1)*a(2)
            -a(1)*b(2)];

% w ≤ b(1) y + x a(2) − b(1) a(2);
% 
% w ≤ x b(2) + a(1) y − a(1) b(2);

end

