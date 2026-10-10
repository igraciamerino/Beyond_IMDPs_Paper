function [m_under,m_over,n_under,n_over] = affine_under_and_overapproximate_sinusoidal(a,b,f_is_sine)
%Values of \underline()\overline f(x) = m*x+n at points a and b
%Points a and b should define the interval [a b]\subseteq [-\pi,\pi]. The function f should be a
%sine or a cosine

if (f_is_sine)
    if (a >= 0)
        m_under = (sin(b)-sin(a))/(b-a);
        m_over = m_under;
        n_under = sin(a) - m_under*a;
        x_over = acos(m_under);
        n_over = sin(x_over) - m_over*x_over;
    elseif (b <= 0)
        m_over = (sin(b)-sin(a))/(b-a);
        m_under = m_over;
        n_over = sin(a) - m_over*a;
        x_under = -acos(m_over);
        n_under = sin(x_under) - m_under*x_under;
    elseif (a < 0)&&(b > 0)
        m_under = cos(a);
        m_over = cos(b);
        n_under = sin(a) - m_under*a;
        n_over = sin(b) - m_over*b;
    end
        
elseif (~f_is_sine)
    if (b <= -pi/2)
        m_over = (cos(b)-cos(a))/(b-a);
        m_under = m_over;
        x_under = -pi - asin(-m_over);
        n_over = cos(a) - m_over*a;
        n_under = cos(x_under) - m_under*x_under;
    elseif (a < -pi/2)&&(b > -pi/2)
        m_under = -sin(a);
        m_over = -sin(b);
        n_under = cos(a) - m_under*a;
        n_over = cos(b) - m_over*b;
    elseif (a >= -pi/2)&&(b <= pi/2)
        m_under = (cos(b)-cos(a))/(b-a);
        m_over = m_under;
        x_over = - asin(m_under);
        n_over = cos(x_over) - m_under*x_over;
        n_under = cos(a) - m_under*a;
    elseif (a < pi/2)&&(b > pi/2)
        m_under = -sin(b);
        m_over = -sin(a);
        n_under = cos(b) - m_under*b;
        n_over = cos(a) - m_over*a;
    elseif (a >= pi/2)
        m_over = (cos(b)-cos(a))/(b-a);
        m_under = m_over;
        x_under = pi + asin(m_over);
        n_over = cos(a) - m_over*a;
        n_under = cos(x_under) - m_under*x_under;

    end
end

end