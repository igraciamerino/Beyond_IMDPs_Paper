function s = J_function(x,state_discretization,user_param, original_sys)
% Assign state q of the abstraction to state x of the original system iff x
% \in q

Ns = user_param.Ns;
aX = original_sys.state_space_lower_left_corner';
bX = original_sys.state_space_upper_right_corner';

is_unsafe = ( any(x <= aX) || any(x >= bX) ); % In case that Post_q_a_w intersects unsafe set

if ( is_unsafe )
    s = Ns + 1;
    return;
else
    n = original_sys.n;
    Nx = user_param.Nx;
    Deltax = state_discretization.Deltax;

    n_x = cell(n, 1);
    for di = 1:n
        n_x{di} = floor( ( x(di) - aX(di) )/Deltax{di} );
    end
    if ( n == 2 )
        s = n_x{2}*Nx{1} + n_x{1} + 1;
    elseif ( n == 3 )
        s =  n_x{3}*Nx{1}*Nx{2} + n_x{2}*Nx{1} + n_x{1} + 1;
    elseif ( n == 4 )
        s =  n_x{4}*Nx{1}*Nx{2}*Nx{3} + n_x{3}*Nx{1}*Nx{2} + n_x{2}*Nx{1} + n_x{1} + 1;
    end
end

end

