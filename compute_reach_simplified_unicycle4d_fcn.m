function [a_Post, b_Post, error_flag] = compute_reach_simplified_unicycle4d_fcn(vertices, u, vertices_c, original_sys, user_param)
% Computes vertices Post as a hyperrectangle given q (and vertices),
% u, w
% Uses interval arithmetic
% Assumes th \in [-pi, pi]

a_s = vertices(:, 1);
b_s = vertices(:, end);

a_w = vertices_c(:, 1);
b_w = vertices_c(:, end);

% De-normalize state
a_th = 2*pi*a_s(3) - pi;
b_th = 2*pi*b_s(3) - pi;
% a_vel = 2*user_param.vel_max*a_s(4) - user_param.vel_max;
% b_vel = 2*user_param.vel_max*b_s(4) - user_param.vel_max;
a_vel = user_param.vel_max*a_s(4);
b_vel = user_param.vel_max*b_s(4);

% Overapproximate sine and cosine functions
if ( b_th < pi/2 )
    a_sin_th = sin( a_th );
    b_sin_th = sin( b_th );
    a_cos_th = cos( b_th );
    b_cos_th = cos( a_th );
elseif  ( ( a_th > -pi/2 ) && ( b_th < 0 ) )
    a_sin_th = sin( a_th );
    b_sin_th = sin( b_th );
    a_cos_th = cos( a_th );
    b_cos_th = cos( b_th );
elseif  ( ( a_th > 0 ) && ( b_th < pi/2 ) )
    a_sin_th = sin( b_th );
    b_sin_th = sin( a_th );
    a_cos_th = cos( a_th );
    b_cos_th = cos( b_th );
elseif  ( a_th > pi/2 )
    a_sin_th = sin( b_th );
    b_sin_th = sin( a_th );
    a_cos_th = cos( b_th );
    b_cos_th = cos( a_th );
elseif ( ( a_th < -pi/2 ) && ( b_th > -pi/2 ) )
    a_sin_th = -1;
    b_sin_th = max( sin( a_th ), sin( b_th ) );
    a_cos_th = cos( a_th );
    b_cos_th = cos( b_th );
elseif ( ( a_th < 0 ) && ( b_th > 0 ) )
    a_sin_th = sin( a_th );
    b_sin_th = sin( b_th );
    a_cos_th = min( sin( a_th ), sin( b_th ) );
    b_cos_th = 1;
elseif ( ( a_th < pi/2 ) && ( b_th > pi/2 ) )
    a_sin_th = min( sin( a_th ), sin( b_th ) );
    b_sin_th = 1;
    a_cos_th = cos( b_th );
    b_cos_th = cos( a_th );
else
    a_sin_th = min( sin( a_th ), sin( b_th ) );
    b_sin_th = max( sin( a_th ), sin( b_th ) );
    a_cos_th = min( cos( a_th ), cos( b_th ) );
    b_cos_th = max( cos( a_th ), cos( b_th ) );
end
if ( ~exist('a_sin_th', 'var') )
    5
end
aux_vel_times_sin_th = [a_vel*a_sin_th, a_vel*b_sin_th, b_vel*a_sin_th, b_vel*b_sin_th];
a_vel_times_sin_th = min(aux_vel_times_sin_th);
b_vel_times_sin_th = max(aux_vel_times_sin_th);
aux_vel_times_cos_th = [a_vel*a_cos_th, a_vel*b_cos_th, b_vel*a_cos_th, b_vel*b_cos_th];
a_vel_times_cos_th = min(aux_vel_times_cos_th);
b_vel_times_cos_th = max(aux_vel_times_cos_th);

aux_w1_times_sin_th = [a_w(1)*a_sin_th, a_w(1)*b_sin_th, b_w(1)*a_sin_th, b_w(1)*b_sin_th];
a_w1_times_sin_th = min(aux_w1_times_sin_th);
b_w1_times_sin_th = max(aux_w1_times_sin_th);
aux_w1_times_cos_th = [a_w(1)*a_cos_th, a_w(1)*b_cos_th, b_w(1)*a_cos_th, b_w(1)*b_cos_th];
a_w1_times_cos_th = min(aux_w1_times_cos_th);
b_w1_times_cos_th = max(aux_w1_times_cos_th);

a_Post = a_s + user_param.Deltat*[(a_vel_times_cos_th - original_sys.cd(1)*b_w1_times_cos_th)
                                  ( a_vel_times_sin_th - original_sys.cd(1)*b_w1_times_sin_th )
                                  u(1)/(2*pi) + original_sys.cd(2)*a_w(2)
                                  u(2)/(user_param.vel_max) + original_sys.cd(3)*a_w(3)];
b_Post = b_s + user_param.Deltat*[(b_vel_times_cos_th - original_sys.cd(1)*a_w1_times_cos_th)
                                  ( b_vel_times_sin_th - original_sys.cd(1)*a_w1_times_sin_th )
                                  u(1)/(2*pi) + original_sys.cd(2)*b_w(2)
                                  u(2)/(user_param.vel_max) + original_sys.cd(3)*b_w(3)];

error_flag = false;

% (Only for debugging purposes)
if ( ~true )
    fprintf('Debug is on for reach sets. This might slow down computation...\n\n')
    delta_debug = max(b_s - a_s)/10;
    precision = 1e-6;
    [X1, X2, X3, X4] = ndgrid(a_s(1):delta_debug:b_s(1), a_s(2):delta_debug:b_s(2), a_s(3):delta_debug:b_s(3), a_s(4):delta_debug:b_s(4));
    X = [X1(:), X2(:), X3(:), X4(:)];
    for k = 1:length(X)
        x = X(k, :)';
        w = (b_w + a_w)/2;
        x_star = unicycle4d_nl_friction(x,u,w,original_sys,user_param);
        if ( any( x_star < a_Post - precision ) || any( x_star > b_Post + precision ) )
            disp('Error computing Post. Maybe increment tolerance')
            error_flag = true;
            return;
        end
    end
end

end