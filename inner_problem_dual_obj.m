function obj = inner_problem_dual_obj(epsilon_pow_p,P_lower_nom,P_upper_nom,s,mu,lambda,h)

Sum = 0;
for j = 1:s
    Sum = Sum + min(P_lower_nom(j)*(lambda + h(j)),P_upper_nom(j)*(lambda + h(j)));
end

obj = -lambda - mu*epsilon_pow_p + Sum;

%vectorized (apparently it's slower...)
% obj = -lambda - mu*epsilon^2 + sum(min(P_lower_nom.*(lambda + h),P_upper_nom.*(lambda + h)));

end