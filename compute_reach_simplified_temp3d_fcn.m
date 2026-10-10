function [a_Post, b_Post, error_flag] = compute_reach_simplified_temp3d_fcn(vertices, u, vertices_c, original_sys, user_param)
% Computes vertices Post as a hyperrectangle given q (and vertices),
% u, w
% Uses interval arithmetic
% Assumes th \in [-pi, pi]

error_flag = false;

a_s = vertices(:, 1);
b_s = vertices(:, end);

a_w = vertices_c(:, 1);
b_w = vertices_c(:, end);

a_A_times_x = original_sys.A*a_s;
b_A_times_x = original_sys.A*b_s;
a_Post = zeros(original_sys.n, 1);
b_Post = zeros(original_sys.n, 1);
for di = 1:original_sys.n
    aux_Post = [(1 + a_w(di))*a_A_times_x(di), (1 + a_w(di))*b_A_times_x(di), (1 + b_w(di))*a_A_times_x(di), (1 + b_w(di))*b_A_times_x(di)];
    a_Post(di) = min(aux_Post);
    b_Post(di) = max(aux_Post);
end
a_Post = a_Post + original_sys.b + original_sys.bu*u;
b_Post = b_Post + original_sys.b + original_sys.bu*u;
% (Only for debugging purposes)
if ( ~true )
    fprintf('Debug is on for reach sets. This might slow down computation...\n\n')
    delta_debug = max(b_s - a_s)/10;
    precision = 1e-6;
    [X1, X2, X3] = ndgrid(a_s(1):delta_debug:b_s(1), a_s(2):delta_debug:b_s(2), a_s(3):delta_debug:b_s(3));
    X = [X1(:), X2(:), X3(:)];
    for k = 1:length(X)
        x = X(k, :)';
        w = (b_w + a_w)/2;
        x_star = temp_sys(x,u,w,original_sys,user_param);
        if ( any( x_star < a_Post - precision ) || any( x_star > b_Post + precision ) )
            disp('Error computing Post. Maybe increment tolerance')
            error_flag = true;
            return;
        end
    end
end

end