function [epsilon_inflated_fine, epsilon_inflated_coarse, delta_fine, delta_coarse] = inflate_ball_table(state_discretization,user_param,original_sys,solver_opts)
% Compute the radius of the inflated Wasserstein ball and budget

Deltax = state_discretization.Deltax;
Deltay = state_discretization.Deltay;
over_approximation_distance_threshold = solver_opts.over_approximation_distance_threshold;
D1 = solver_opts.D1;
Nx_grid_q_star_fine = user_param.Nx_grid_q_star_fine;
Ny_grid_q_star_fine = user_param.Ny_grid_q_star_fine;
Nx_grid_q_star_coarse = user_param.Nx_grid_q_star_coarse;
Ny_grid_q_star_coarse = user_param.Ny_grid_q_star_coarse;
N = user_param.N;
epsilon = user_param.epsilon;

Deltax_subcells_fine = (Deltax + 2*over_approximation_distance_threshold)/Nx_grid_q_star_fine;
Deltay_subcells_fine = (Deltay +2*over_approximation_distance_threshold)/Ny_grid_q_star_fine;
Deltax_subcells_coarse = (Deltax + 2*D1)/Nx_grid_q_star_coarse;
Deltay_subcells_coarse = (Deltay +2*D1)/Ny_grid_q_star_coarse;
% Inflate Wasserstein ball
    delta_fine = sqrt((Deltax_subcells_fine/2)^2 + (Deltay_subcells_fine/2)^2);
    epsilon_inflated_fine = epsilon + delta_fine;
    delta_coarse = sqrt((Deltax_subcells_coarse/2)^2 + (Deltay_subcells_coarse/2)^2);
    epsilon_inflated_coarse = epsilon + delta_coarse;

total_budget = N*epsilon_inflated_fine;
% (Only for debugging purposes)
fprintf('Radius of the ambiguity ball inflated in the fine region of the lookup table = %.4f. \n', epsilon_inflated_fine)
fprintf('Total transport budget N*epsilon_inflated_fine = %.4f. \n', total_budget)

end
