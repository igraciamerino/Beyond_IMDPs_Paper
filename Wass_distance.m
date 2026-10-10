function [W_distance] = Wass_distance(Xi_hat_1,Xi_hat_2)

Xi_hat_1 = Xi_hat_1';
Xi_hat_2 = Xi_hat_2';

N1 = size(Xi_hat_1,1);
N2 = size(Xi_hat_2,2);
Nv = N1*N2;

d_ij_2 = zeros(Nv,1);
for i = 1: N1
    xi_i = Xi_hat_1(:,i);
    for j = 1: N2
        xi_j = Xi_hat_2(:,j);
        d_ij_2(i,j) = (xi_i(1) - xi_j(1))^2 + (xi_i(2) - xi_j(2))^2;
    end
end

f = [];
for j = 1: N2
    for i = 1: N1
        f = [f; d_ij_2(i,j)];
    end
end

A_marg_j = zeros(N2,Nv);
for j = 1:N2
    A_marg_j(j,(j-1)*N1+1:j*N1) = ones(1,N1);
end
b_marg_j = 1/N2*ones(N2,1);

A_marg_i = kron(eye(N1),ones(1,N2));
b_marg_i = 1/N1*ones(N1,1);

Aeq = [A_marg_j;A_marg_i];
beq = [b_marg_j;b_marg_i];

%options = optimoptions('linprog','Display','none');
options = optimoptions('linprog','Algorithm','interior-point');
[pi,W_2] = linprog(f,[],[],Aeq,beq,zeros(Nv,1),[],options);

W_distance = sqrt(W_2);
end

