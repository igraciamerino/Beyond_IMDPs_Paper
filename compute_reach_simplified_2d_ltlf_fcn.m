function [a_Post, b_Post, error_flag] = compute_reach_simplified_2d_ltlf_fcn(vertices_q,u,w,original_sys,user_param)
% Computes vertices of Post(q,u,w) given q (and vertices_q),
% u, w

Post_q_u_w = zeros(2,4);
for k = 1:4
    v = vertices_q(:,k);
    Post_q_u_w(:,k) = unicycle2D_ltlf(v,u,w,original_sys,user_param);
end

% Compute limits of rectangle that contains Post_q_u_w
a_Post = min(Post_q_u_w,[],2);
b_Post = max(Post_q_u_w,[],2);

error_flag = false;

end