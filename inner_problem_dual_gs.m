function [V] = inner_problem_dual_gs(epsilon_pow_p,P_lower_nom,P_upper_nom,V_lower,s,dist_pow_p, d_min_nonzero_pow_p, lower_bound)

max_iter_gs = 1000;
tol = 10e-6;
gr = 0.61803;
mul = 0;
muu = 1/d_min_nonzero_pow_p;


% figure(1)
% hold on
% grid

% % Objective at mul
% h = min(V_lower + mul*dist_sq);
% for j = 1:s
%     objs(j) = inner_problem_dual_obj(epsilon,P_lower_nom,P_upper_nom,s,mul,-h(j),h);
% end
% Vl = max(objs);
% %plot(mul,Vl,'o')
% 
% % Objective at muu
% h = min(V_lower + muu*dist_sq);
% for j = 1:s
%     objs(j) = inner_problem_dual_obj(epsilon,P_lower_nom,P_upper_nom,s,muu,-h(j),h);
% end
% Vu = max(objs);
% %plot(muu,Vu,'o')


gap = muu - mul;
mu2 = mul + gr*gap;
mu1 = muu - gr*gap;
objs = zeros(s,1);

% Objective at mu1
h = min(V_lower + mu1*dist_pow_p);
for j = 1:s
    objs(j) = inner_problem_dual_obj(epsilon_pow_p,P_lower_nom,P_upper_nom,s,mu1,-h(j),h);
end
V1 = max(objs);
%plot(mu1,V1,'o')

% Objective at mu2
h = min(V_lower + mu2*dist_pow_p);
for j = 1:s
    objs(j) = inner_problem_dual_obj(epsilon_pow_p,P_lower_nom,P_upper_nom,s,mu2,-h(j),h);
end
V2 = max(objs);
%plot(mu1,V1,'o')

k = 1;
while (abs(V1 - V2) > tol) && (k < max_iter_gs)

    if (V1 > V2)
        muu= mu2;
        mu2 = mu1;
        V2 = V1;
        gap = muu - mul;
        mu1 = muu - gr*gap;
        % Objective at mu1
        h = min(V_lower + mu1*dist_pow_p);
        for j = 1:s
            objs(j) = inner_problem_dual_obj(epsilon_pow_p,P_lower_nom,P_upper_nom,s,mu1,-h(j),h);
        end
        V1 = max(objs);
        %plot(mu2,V2,'o')
    else
        mul = mu1;
        mu1 = mu2;
        V1 = V2;
        gap = muu - mul;
        mu2 = mul + gr*gap;
        % Objective at mu2
        h = min(V_lower + mu2*dist_pow_p);
        for j = 1:s
            objs(j) = inner_problem_dual_obj(epsilon_pow_p,P_lower_nom,P_upper_nom,s,mu2,-h(j),h);
        end
        V2 = max(objs);
        %plot(mu2,V2,'o')
    end
    k = k + 1;

end

% plot(mu_plot)
% if (mu1 > 41)
%     mu1
% end

if (k == max_iter_gs)
    disp('Timeout reached')
end
% disp(k)

V = (V1+V2)/2;

if ( (V <0) && (lower_bound) )
    V = 0;
elseif ( (V < -1) && (~lower_bound) )
    V = -1;
end

end