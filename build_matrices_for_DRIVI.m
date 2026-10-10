function [A,b,Aeq,beq,f_aux_cost] = build_matrices_for_DRIVI(epsilon,P_lower_nom,P_upper_nom,Qs,Nq,dist)

s = length(Qs);
Nv = (Nq+1)*s;

% \pi_{ij} \geq 0, for all i \in Q, j \in Qs
% A_lb_zero = zeros(Nv);
% for i = 1:Nv
%     A_lb_zero(i,i) = -1;
% end
% b_lb_zero = zeros(Nv,1);

% sum_{i\in Q} \pi_{ij} \leq P_upper_nom, for all j \in Q
A_ub_nom = zeros(s,Nv);
for i = 1:s
    A_ub_nom(i,(i-1)*(Nq+1)+1:i*(Nq+1)) = ones(1,Nq+1);
end
b_ub_nom = P_upper_nom;


% sum_{i \in Q} \pi_{ij} \geq P_lower_nom, for all j \in Qs
A_lb_nom = zeros(s,Nv);
for i = 1:s
    A_lb_nom(i,(i-1)*(Nq+1)+1:i*(Nq+1)) = -ones(1,Nq+1);
end
b_lb_nom = -P_lower_nom;


% sum_{i\in Q}sum_{j\in Qs}\pi_{ij} = 1 equality
Aeq = ones(1,Nv);
beq = 1;

% Budget inequality

A_budget = zeros(Nv,1);
for j = 1:s
    A_budget((j-1)*(Nq+1) +1 : j*(Nq+1),1) = dist(:,j).^2;
end
A_budget = A_budget';
b_budget = epsilon^2;

A = [A_lb_nom; A_ub_nom; A_budget];
b = [b_lb_nom; b_ub_nom; b_budget];
% % length_lb_zero = length(b_lb_zero);
% length_ub_nom = length(b_ub_nom);
% length_lb_nom = length(b_lb_nom);
% length_budget = 1;
% % % 
% length_ineq = length_ub_nom+length_lb_nom+length_budget;
% A = zeros(length_ineq,Nv);
% % A(1:length_lb_zero,:) = A_lb_zero;
% A(1:length_lb_nom,:) = A_lb_nom;
% A(length_lb_nom+1:length_lb_nom+length_ub_nom,:) = A_ub_nom;
% A(end,:) = A_budget;
% 
% b = zeros(length_ineq,1);
% % b(1:length_lb_zero,:) = b_lb_zero;
% b(1:length_lb_nom) = b_lb_nom;
% b(length_lb_nom+1:length_lb_nom+length_ub_nom) = b_ub_nom;
% b(end) = b_budget;

f_aux_cost = zeros(Nq+1,Nv);
for i = 1:Nq+1
    for j = 1:s
        f_aux_cost(i,i+(j-1)*(Nq+1)) = 1;
    end
end


end

