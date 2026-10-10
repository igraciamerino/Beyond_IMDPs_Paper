function [Q_intersects, error_flag] = intersections_simplified_temp_n_dim_fcn(a_Post, b_Post, Deltax, Nx, Ns, aX, bX, precision, n)

if ( n == 2)
    [Q_intersects, error_flag] = intersections_simplified_temp2d_fcn(a_Post, b_Post, Deltax, Nx, Ns, aX, bX, precision);
elseif ( n == 3)
    [Q_intersects, error_flag] = intersections_simplified_temp3d_fcn(a_Post, b_Post, Deltax, Nx, Ns, aX, bX, precision);
elseif ( n == 4)
    [Q_intersects, error_flag] = intersections_simplified_temp4d_fcn(a_Post, b_Post, Deltax, Nx, Ns, aX, bX, precision);
end


end