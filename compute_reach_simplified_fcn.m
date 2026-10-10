function [a_Post, b_Post, error_flag] = compute_reach_simplified_fcn(vertices,u,w,original_sys,user_param)
% Computes vertices of Post(q,u,w) given q (and vertices_q),
% u, w

Post_s_u_w = zeros(2,4);
for k = 1:4
    v = vertices(:,k);
    Post_s_u_w(:,k) = unicycle2D(v,u,w,original_sys,user_param);
end

% Compute limits of rectangle that contains Post_q_u_w
a_Post = min(Post_s_u_w,[],2);
b_Post = max(Post_s_u_w,[],2);

error_flag = false;

end