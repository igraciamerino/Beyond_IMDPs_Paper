function [samples, disturbance_discretization] = compute_weight_emp_cells_uniform_fcn(samples, disturbance_discretization, original_sys, user_param)

samples.weight = zeros(1, user_param.Nc + 1);
samples.size_cluster = zeros(1, user_param.Nc + 1);
for w_i = 1:user_param.N
    w = samples.Xi_hat(:, w_i);
    if ( ~any( w < original_sys.disturbance_space_lower_left_corner' ) && ~any( w > original_sys.disturbance_space_upper_right_corner' ) )
        for c_i = 1:user_param.Nc
            a_c = disturbance_discretization.vertices_c{c_i}(:, 1);
            b_c = disturbance_discretization.vertices_c{c_i}(:, end);
            if ( ~any( w < a_c ) && ~any( w > b_c ) )
                samples.weight(c_i) = samples.weight(c_i) + 1;
                break;
            end
        end
    else
        samples.weight(end) = samples.weight(end) + 1;
    end
end
samples.weight_norm = samples.weight/user_param.N;

for c_i = 1:user_param.Nc
    samples.size_cluster(c_i) = disturbance_discretization.Deltaxd{1}/2;
end
c_i = user_param.Nc + 1;
samples.size_cluster(c_i) = 2*norm(original_sys.disturbance_space_upper_right_corner -  original_sys.disturbance_space_lower_left_corner, 2);

% If coarse disretization

if ( isfield(disturbance_discretization, 'vertices_c_coarse') )
    samples.weight_coarse = zeros(1, user_param.Nc_coarse + 1);
    samples.size_cluster_coarse = zeros(1, user_param.Nc_coarse + 1);
    for w_i = 1:user_param.N
        w = samples.Xi_hat(:, w_i);
        if ( ~any( w < original_sys.disturbance_space_lower_left_corner' ) && ~any( w > original_sys.disturbance_space_upper_right_corner' ) )
            for c_i_coarse = 1:user_param.Nc_coarse
                a_c_coarse = disturbance_discretization.vertices_c_coarse{c_i_coarse}(:, 1);
                b_c_coarse = disturbance_discretization.vertices_c_coarse{c_i_coarse}(:, end);
                if ( ~any( w < a_c_coarse ) && ~any( w > b_c_coarse ) )
                    samples.weight_coarse(c_i_coarse) = samples.weight_coarse(c_i_coarse) + 1;
                    % Don't break, since w might also be contained in other c
                end
            end
        else
            samples.weight_coarse(end) = samples.weight_coarse(end) + 1;
        end
    end

    for c_i_coarse = 1:user_param.Nc_coarse
        samples.size_cluster_coarse(c_i_coarse) = disturbance_discretization.Deltaxd_coarse/2;
    end
    c_i_coarse = user_param.Nc_coarse + 1;
    samples.size_cluster_coarse(c_i_coarse) = 2*norm(original_sys.disturbance_space_upper_right_corner -  original_sys.disturbance_space_lower_left_corner, 2);

end

end