function [a_Post, b_Post, error_flag] = compute_reach_simplified_fm2_fcn(vertices,u,w,original_sys,user_param)
% Computes vertices of Post(q,u,w) given q (and vertices_q),
% u, w

Post_s_u_w = zeros(original_sys.n, 2*original_sys.n);
for k = 1:2^original_sys.n
    v = vertices(:,k);
    Post_s_u_w(:,k) = fm2_sys(v, u, w, original_sys, user_param);
end

% Compute limits of rectangle that contains Post_q_u_w
a_Post = min(Post_s_u_w,[],2) + original_sys.cd*w;
b_Post = max(Post_s_u_w,[],2) + original_sys.cd*w;

error_flag = false;

end