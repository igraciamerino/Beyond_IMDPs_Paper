function [samples, disturbance_discretization] = compute_weight_cells_gaussian_fcn(samples, disturbance_discretization, original_sys, user_param)

samples.weight_norm = ones(1, user_param.Nc + 1);
samples.size_cluster = zeros(1, user_param.Nc + 1);
for c_i = 1:user_param.Nc
    a_c = disturbance_discretization.vertices_c{c_i}(:, 1);
    b_c = disturbance_discretization.vertices_c{c_i}(:, end);
    for di = 1:original_sys.d
        mass_di = normcdf(b_c(di), original_sys.center_samples(di), original_sys.weight_samples) - normcdf(a_c(di), original_sys.center_samples(di), original_sys.weight_samples);
        samples.weight_norm(c_i) = samples.weight_norm(c_i)*mass_di;
    end
end
% Account for the rest of W
a_c = original_sys.disturbance_space_lower_left_corner;
b_c = original_sys.disturbance_space_upper_right_corner;
for di = 1:original_sys.d
    mass_di = normcdf(b_c(di), original_sys.center_samples(di), original_sys.weight_samples) - normcdf(a_c(di), original_sys.center_samples(di), original_sys.weight_samples);
    samples.weight_norm(end) = samples.weight_norm(end)*mass_di;
end
samples.weight_norm(end) = 1 - samples.weight_norm(end);

samples.weight = samples.weight_norm*user_param.N;

for c_i = 1:user_param.Nc
    if ( original_sys.d < 3 )
        samples.size_cluster(c_i) = max(disturbance_discretization.Deltaxd{1}, disturbance_discretization.Deltaxd{2})/2;
    elseif ( original_sys.d == 3 )
        samples.size_cluster(c_i) = max([disturbance_discretization.Deltaxd{1}, disturbance_discretization.Deltaxd{2}, disturbance_discretization.Deltaxd{3}])/2;
    elseif ( original_sys.d == 4 )
        samples.size_cluster(c_i) = max([disturbance_discretization.Deltaxd{1}, disturbance_discretization.Deltaxd{2}, disturbance_discretization.Deltaxd{3}, disturbance_discretization.Deltaxd{4}])/2;
    end
end
c_i = user_param.Nc + 1;
samples.size_cluster(c_i) = 2*norm(original_sys.disturbance_space_upper_right_corner -  original_sys.disturbance_space_lower_left_corner, 2);

% If coarse disretization

if ( isfield(disturbance_discretization, 'vertices_c_coarse') )
    disp('No functionality for coarse partition if P_w is known...')
end

end