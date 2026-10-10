function [a_Post, b_Post, error_flag] = compute_reach_simplified_lipschitz_reach_fcn(vertices,u,w,original_sys,user_param, rad_s)
% Computes vertices of Post(q,u,w) given q (and vertices_q),
% u, w

Post_s_u_w = zeros(2,4);
s_center = vertices(:,end) - vertices(:,1);
xp = unicycle2D(s_center,u,w,original_sys,user_param);

% Compute limits of rectangle that contains Post_q_u_w
a_Post = xp - rad_s;
b_Post = xp + rad_s;
error_flag = false;

end