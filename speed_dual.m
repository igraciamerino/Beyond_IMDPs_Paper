tic
q = 169;
a = 5;
Qs = variables_for_matrices.Qs{q,a};
P_lower_nom = variables_for_matrices.P_lower_nom{q,a}';
P_upper_nom = variables_for_matrices.P_upper_nom{q,a}';

% CAREFUL WITH THIS ONE!!!! In dual the last column is set to the true
% distances
dist = variables_for_matrices.dist;
% %                 % Build the matrices involved in the LP
%                 [A,b,Aeq,beq,f_aux_cost] = build_matrices_for_DRIVI(epsilon,P_lower_nom,P_upper_nom,Qs,user_param.Nq,dist(:,Qs));
%                 [V1_lower_q_a(a),~] = RIVI_inner_minimization(V_lower,A,b,Aeq,beq,f_aux_cost);
                % Or use the dual
                [V1] = inner_problem_dual(epsilon,P_lower_nom,P_upper_nom,V_lower,length(Qs),dist(:,Qs).^2);
toc

%% Zero order (with fminbnd)

tic

q = 169;
a = 5;
Qs = variables_for_matrices.Qs{q,a};
s = variables_for_matrices.s{q,a};
P_lower_nom = variables_for_matrices.P_lower_nom{q,a}';
P_upper_nom = variables_for_matrices.P_upper_nom{q,a}';
dist = variables_for_matrices.dist;

mu_lower = 0;
mu_upper = max(V_lower)/0.025^2;

fun = @(mu)Obj_mu(mu,V_lower,dist(:,Qs).^2,epsilon,P_lower_nom,P_upper_nom,s);

[~,V1,exitflag,output] = fminbnd(fun,mu_lower,mu_upper);

toc

%% Zero order (just evaluations in a grid)

tic


q = 169;
a = 5;
Qs = variables_for_matrices.Qs{q,a};
s = variables_for_matrices.s{q,a};
P_lower_nom = variables_for_matrices.P_lower_nom{q,a}';
P_upper_nom = variables_for_matrices.P_upper_nom{q,a}';
dist = variables_for_matrices.dist;

mu_lower = 0;
mu_upper = max(V_lower)/0.025^2;
mus = linspace(mu_lower,mu_upper,100);

fun = @(mu)Obj_mu(mu,V_lower,dist(:,Qs).^2,epsilon,P_lower_nom,P_upper_nom,s);

[~,V1,exitflag,output] = fminbnd(fun,mu_lower,mu_upper);

toc

%% Zero order (golden section)

tic

q = 169;
a = 5;
Qs = variables_for_matrices.Qs{q,a};
s = variables_for_matrices.s{q,a};
P_lower_nom = variables_for_matrices.P_lower_nom{q,a}';
P_upper_nom = variables_for_matrices.P_upper_nom{q,a}';
dist = variables_for_matrices.dist;

[V1] = inner_problem_dual_gs(epsilon,P_lower_nom,P_upper_nom,V_lower,s,dist(:,Qs).^2);

toc

