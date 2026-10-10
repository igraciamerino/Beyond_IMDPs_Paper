function [W_distance,lambda] = Wass_distance_dual(Xi_hat_1,Xi_hat_2)

N1 = size(Xi_hat_1,2);
N2 = size(Xi_hat_2,2);

d_ij_2 = zeros(N1,N2);
for i = 1: N1
    xi_i = Xi_hat_1(:,i);
    for j = 1: N2
        xi_j = Xi_hat_2(:,j);
        d_ij_2(i,j) = (xi_i(1) - xi_j(1))^2 + (xi_i(2) - xi_j(2))^2;
    end
end

W2 = @(x)(1/N2*sum(max(-d_ij_2-x)) + 1/N1*sum(x));
x0 = 0*ones(N1,1);

opts = optimoptions('fminunc','MaxFunctionEvaluations',10^6,'PlotFcns',@optimplotfval);%,'Algorithm','quasi-newton'
%opts = optimset('PlotFcns',@optimplotfval);
[lambda,fval,eflag,outputu] = fminunc(W2,x0,opts);
% lambda = ga(W2,length(x0));
% fval = W2(lambda);

W_distance = sqrt(-fval);
end

