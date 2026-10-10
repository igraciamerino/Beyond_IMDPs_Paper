function [V1_q,x] = RIVI_inner_maximization(V,A,b,Aeq,beq,f_aux_cost)
%Computes the inner minimization of DR-value iteration

f = -f_aux_cost'*V;

options = optimoptions('linprog','Display','none');
[x,V1_q] = linprog(f,A,b,Aeq,beq,zeros(1,length(f)),[],options);
V1_q = -V1_q;
end