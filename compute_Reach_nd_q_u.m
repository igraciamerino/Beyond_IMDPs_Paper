function [Reach_nd_q_a] = compute_Reach_nd_q_u(Post_q_a_w0, L, rho)


Reach_nd_q_a = Post_q_a_w0  + L*rho*[-1 1 1 -1 -1; -1 -1 1 1 -1];

end