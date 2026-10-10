function [a_Post, b_Post, error_flag] = compute_reach_simplified_temp_n_dim_fcn(vertices, u, vertices_c, original_sys, user_param)

if ( original_sys.n == 2)
    [a_Post, b_Post, error_flag] = compute_reach_simplified_temp2d_fcn(vertices, u, vertices_c, original_sys, user_param);
elseif ( original_sys.n == 3)
    [a_Post, b_Post, error_flag] = compute_reach_simplified_temp3d_fcn(vertices, u, vertices_c, original_sys, user_param);
elseif ( original_sys.n == 4)
    [a_Post, b_Post, error_flag] = compute_reach_simplified_temp4d_fcn(vertices, u, vertices_c, original_sys, user_param);
end

end