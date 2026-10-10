function [samples, disturbance_discretization] = compute_weight_emp_cells_voronoi_fcn(samples, disturbance_discretization, original_sys, user_param)

samples.weight = zeros(1, user_param.Nc + 1);
samples.size_cluster = zeros(1, user_param.Nc + 1);
dist_w_c = zeros(user_param.N, user_param.Nc);
for w_i = 1:user_param.N
    w = samples.Xi_hat(:, w_i);
    if ( ~any( w < original_sys.disturbance_space_lower_left_corner' ) && ~any( w > original_sys.disturbance_space_upper_right_corner' ) )
        for c_i = 1:user_param.Nc
            center = [disturbance_discretization.Xd_center(c_i), disturbance_discretization.Yd_center(c_i)]';
            dist_w_c(w_i, c_i) = norm(w - center, "inf");
        end
        [~ , ind_c_min_dist] = min( dist_w_c(w_i, :) );
        samples.weight(ind_c_min_dist) = samples.weight(ind_c_min_dist) + 1;
    else
        samples.weight(end) = samples.weight(end) + 1;
    end
end

% Compute ength of voronoi cells
a_W = original_sys.disturbance_space_lower_left_corner;
b_W = original_sys.disturbance_space_upper_right_corner;
% [vs, cs, ~] = VoronoiLimit(disturbance_discretization.Xd_center,disturbance_discretization.Yd_center,'bs_ext',[a_W(1) a_W(2); b_W(1) a_W(2); b_W(1) b_W(2); a_W(1) b_W(2)], 'figure', 'off');
[vs, cs] = voronoin([disturbance_discretization.Xd_center', disturbance_discretization.Yd_center']);
group_cluster_into_unsafe = false(1, user_param.Nc);
for c_i = 1:user_param.Nc
    center = [disturbance_discretization.Xd_center(c_i), disturbance_discretization.Yd_center(c_i)];
    verts_c = vs(cs{c_i}, :);
    dist_aux = zeros(1, size( verts_c, 1 ));
    for r_i = 1:size(verts_c, 1)
        dist_aux(r_i) = norm( verts_c(r_i, :) - center, "inf" );
    end
    dist = max( dist_aux );
    %     dist = min([dist, 0.2]);
    samples.size_cluster(c_i) = dist;
    if ( dist > 0.25 )
        
%         disp('ERROR: UNBOUNDED CLUSTERS...')
        group_cluster_into_unsafe(c_i) = true;
%         samples.size_cluster(c_i) = max( [norm( original_sys.disturbance_space_lower_left_corner' - center, "inf" ), norm( original_sys.disturbance_space_upper_right_corner' - center, "inf" )] );
    end
end
samples.size_cluster(end) = 2*norm(original_sys.disturbance_space_upper_right_corner -  original_sys.disturbance_space_lower_left_corner, 2);
samples.size_cluster = [samples.size_cluster(~group_cluster_into_unsafe), samples.size_cluster(end)];
weight_unbounded = sum(samples.weight(group_cluster_into_unsafe)) + samples.weight(end);
samples.weight = [samples.weight(~group_cluster_into_unsafe), weight_unbounded];
samples.weight_norm = samples.weight/user_param.N;
user_param.Nc = length( samples.weight ) - 1; % -1 bc of unsafe
disturbance_discretization.Xd_center = disturbance_discretization.Xd_center(~group_cluster_into_unsafe);
disturbance_discretization.Yd_center = disturbance_discretization.Yd_center(~group_cluster_into_unsafe);

end