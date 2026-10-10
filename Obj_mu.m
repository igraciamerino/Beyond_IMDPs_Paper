function [V1] = Obj_mu(mu,V_lower,dist_sq,epsilon,P_lower_nom,P_upper_nom,s)

h = min(V_lower + mu*dist_sq);
objs = zeros(1,s);
for j = 1:s
    objs(j) = inner_problem_dual_obj(epsilon,P_lower_nom,P_upper_nom,s,mu,-h(j),h);
end
V1 = max(objs);

end