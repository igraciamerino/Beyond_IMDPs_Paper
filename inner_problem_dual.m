% function [V1] = inner_problem_dual(epsilon,P_lower_nom,P_upper_nom,V_lower,s,dist_sq)
% 
% % % With grid search in mu
% % N_mu = 100;
% % 
% % mu_upper = max(V_lower)/0.025^2;
% % mus = linspace(0,mu_upper,N_mu);
% % objs = zeros(N_mu,s);
% % for mu_ind = 1:N_mu
% %     h = min(V_lower + mus(mu_ind)*dist_sq);
% %     % Search for the optimal value of lambda for a given mu
% %     for j = 1:s
% %         lambda = -h(j);
% %         objs(mu_ind,j) = inner_problem_dual_obj(epsilon,P_lower_nom,P_upper_nom,s,mus(mu_ind),lambda,h);
% %     end
% % end
% % 
% % V1 = max(objs, [], 'all');
% 
% % With bisection in mu
% max_iter_bisection = 1000;
% tol = 10e-6;
% mu_lower = 0;
% mu_upper = max(V_lower)/0.025^2;
% mu_current = mu_upper/2;
% objs = zeros(s,1);
% dist_sqOpt = zeros(s,1);
% for k = 1:max_iter_bisection
%     [h, l] = min(V_lower + mu_current*dist_sq);
%     % Search for the optimal value of lambda for a given mu
%     for j = 1:s
%         lambda = -h(j);
%         objs(j) = inner_problem_dual_obj(epsilon,P_lower_nom,P_upper_nom,s,mu_current,lambda,h);
%     end
%     [V1,ind_lambdaOpt_mu] = max(objs);
%     [k-1 V1]
%     lambdaOpt_mu = -h(ind_lambdaOpt_mu);
%     J1_ind = (lambdaOpt_mu + h > 0);
%     for j = 1:s
%         dist_sqOpt(j) = dist_sq(l(j),j);
%     end
%     supgradObj_mu_lambdaOpt = compute_supgrad(epsilon,P_lower_nom,P_upper_nom,dist_sqOpt,J1_ind,~J1_ind);
%     % Choose search direction
%     if (supgradObj_mu_lambdaOpt >0)
%         mu_lower = mu_current;
%     else
%         mu_upper = mu_current;
%     end
%     mu_current = (mu_upper + mu_lower)/2;
%     if (mu_upper - mu_current < tol)
%         break
%     end
% %     mu_plot(k) = mu_current;
% end
% 
% % plot(mu_plot)
% 
% if (k == max_iter_bisection)
%     disp('Timeout reached')
% end
% 
% 
% % Sort h
% % J1 = h(l);
% % for j = 1:s
% %     lambda = -h_ordered(j);
% %     objs(k,j) = inner_problem_dual_obj(epsilon,P_lower_nom,P_upper_nom,s,mus(k),lambda,h);
% %     if (j>1)&&(objs(k,j)<objs(k,j-1))
% %         break
% %     end
% % end
% 
% end

% With bisection in mu and stopping criterion related to the optimal value
function [V1] = inner_problem_dual(epsilon,P_lower_nom,P_upper_nom,V_lower,s,dist_sq)

max_iter_bisection = 1000;
tol = 10e-6;
mu_lower = 0;
mu_upper = max(V_lower)/0.025^2;
mu_current = mu_upper/2;
objs = zeros(s,1);
dist_sqOpt = zeros(s,1);
k = 1;
V1 = 0;
V1_prev = -1;
while ( (k < max_iter_bisection) && (abs(V1 - V1_prev) > tol) )
    V1_prev = V1;
    [h, l] = min(V_lower + mu_current*dist_sq);
    % Search for the optimal value of lambda for a given mu
    for j = 1:s
        lambda = -h(j);
        objs(j) = inner_problem_dual_obj(epsilon,P_lower_nom,P_upper_nom,s,mu_current,lambda,h);
    end
    [V1,ind_lambdaOpt_mu] = max(objs);
    
    lambdaOpt_mu = -h(ind_lambdaOpt_mu);
    J1_ind = (lambdaOpt_mu + h > 0);
    for j = 1:s
        dist_sqOpt(j) = dist_sq(l(j),j);
    end
    supgradObj_mu_lambdaOpt = compute_supgrad(epsilon,P_lower_nom,P_upper_nom,dist_sqOpt,J1_ind,~J1_ind);
    
%     disp(' k, mu, obj, supgr')
%     [k-1 mu_current V1 supgradObj_mu_lambdaOpt]
%     disp('h =')
%     h
%     disp("l_opt =")
%     l-1
    
    % Choose search direction
    if (supgradObj_mu_lambdaOpt >0)
        mu_lower = mu_current;
    else
        mu_upper = mu_current;
    end
    mu_current = (mu_upper + mu_lower)/2;

    k = k+1;
end

% plot(mu_plot)

if (k == max_iter_bisection)
    disp('Timeout reached')
end
% disp(k)

if (V1 <0)
    V1 = 0;
end

% Sort h
% J1 = h(l);
% for j = 1:s
%     lambda = -h_ordered(j);
%     objs(k,j) = inner_problem_dual_obj(epsilon,P_lower_nom,P_upper_nom,s,mus(k),lambda,h);
%     if (j>1)&&(objs(k,j)<objs(k,j-1))
%         break
%     end
% end

end