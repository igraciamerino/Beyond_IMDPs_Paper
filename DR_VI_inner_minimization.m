function [V1_q,x] = DR_VI_inner_minimization(V,A,b,Aeq,beq,A_ub)
%Computes the inner minimization of DR-value iteration

Nv = size(A,2);

f = V'*A_ub;
lb = zeros(1,Nv);

[x,V1_q] = linprog(f,A,b,Aeq,beq,lb,[]);

end

