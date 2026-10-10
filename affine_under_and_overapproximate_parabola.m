function [m_under,m_over,n_under,n_over] = affine_under_and_overapproximate_parabola(a,b)
%\underline()\overline f(x) = m*x+n
%Points a and b should define the interval [a b]\subseteq \R.

if (a ~= b)
    m_over = b + a;
    m_under = m_over;
    n_under = -m_under^2/4;
    n_over = a^2 - m_under*a;
else
    m_over = 0;
    m_under = 0;
    n_under = 0;
    n_over = a^2;
end

end

