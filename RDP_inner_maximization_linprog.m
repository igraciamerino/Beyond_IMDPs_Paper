function [V1_q,x] = RDP_inner_maximization_linprog(V,A,b,Aeq,beq, lb, ub)
%Computes the inner minimization of DR-value iteration

options = optimoptions('linprog','Display','none');
[x, V1_q] = linprog(V, A, b, Aeq, beq, lb, ub, options);

end