function [a_Post, b_Post, error_flag] = compute_reach_simplified_unicycle3d_fcn(vertices,u,w,original_sys,user_param)
% Computes vertices of Post(q,u,w) given q (and vertices),
% u, w

% Normalization transformation (x, y, th) |--> x (the state vector)
C = [1 0 0;
     0 1 0
     0 0 1/(2*pi)];
C_inv = [1 0 0;
     0 1 0
     0 0 2*pi];
d = [0 0 1/2]';

a_rectangle = vertices(:, 1);
b_rectangle = vertices(:, end);

% De-normalize state
th_a_rectangle = 2*pi*a_rectangle(3) - pi;
th_b_rectangle = 2*pi*b_rectangle(3) - pi;

% Overapproximate sine and cosine functions
[ms_under,ms_over,ns_under,ns_over] = affine_under_and_overapproximate_sinusoidal(th_a_rectangle,th_b_rectangle,1); % Overapproximate sine
[mc_under,mc_over,nc_under,nc_over] = affine_under_and_overapproximate_sinusoidal(th_a_rectangle,th_b_rectangle,0); % Overapproximate cosine

% Obtain un-normalized approximators
A_under = [1 0 user_param.Deltat*( u(1) - original_sys.cd(1)*w(1) )*mc_under
           0 1 user_param.Deltat*( u(1) - original_sys.cd(1)*w(1) )*ms_under
           0 0 1];
b_under = [user_param.Deltat*( u(1) - original_sys.cd(1)*w(1) )*nc_under
           user_param.Deltat*( u(1) - original_sys.cd(1)*w(1) )*ns_under
           user_param.Deltat*( u(2) + 2*pi*original_sys.cd(2)*w(2) )];

A_over = [1 0 user_param.Deltat*( u(1) - original_sys.cd(1)*w(1) )*mc_over
           0 1 user_param.Deltat*( u(1) - original_sys.cd(1)*w(1) )*ms_over
           0 0 1];
b_over = [user_param.Deltat*( u(1) - original_sys.cd(1)*w(1) )*nc_over
           user_param.Deltat*( u(1) - original_sys.cd(1)*w(1) )*ns_over
           user_param.Deltat*( u(2) + 2*pi*original_sys.cd(2)*w(2) )];

% Normalize affine approximators
b_under = C*b_under + d - (C*A_under*C_inv)*d;
b_over = C*b_over + d - (C*A_over*C_inv)*d;
A_under = C*A_under*C_inv;
A_over =  C*A_over*C_inv;

% Compute limits of rectangle that contains Post_q_u_w
a_Post = min(A_under*vertices + b_under,[],2);
b_Post = max(A_over*vertices + b_over,[],2);

error_flag = false;

end