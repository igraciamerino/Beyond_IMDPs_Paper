function [a_Reach_nd_q_a, b_Reach_nd_q_a] = compute_Reach_nd_simplified_q_u(a_Post_w0, b_Post_w0, L, rho)

a_Reach_nd_q_a = a_Post_w0 - L*rho;
b_Reach_nd_q_a = b_Post_w0 + L*rho;

end