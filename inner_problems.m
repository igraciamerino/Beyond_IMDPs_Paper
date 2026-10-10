function [p_lower,p_upper] = inner_problems(Xi_hat,epsilon,A,b)

% Compute p_lower

to_polytope = 0;
p_lower = 1 - DR_Uncertainty_Quantification(Xi_hat,epsilon,A,b,to_polytope);

% Compute p_upper

to_polytope = 1;
p_upper = DR_Uncertainty_Quantification(Xi_hat,epsilon,A,b,to_polytope);

end
