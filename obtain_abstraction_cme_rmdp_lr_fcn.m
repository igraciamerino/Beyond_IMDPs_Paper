function [solution, error_flags] = obtain_abstraction_cme_rmdp_lr_fcn(user_param, state_discretization, disturbance_discretization, original_sys, solver_opts, samples, solution)

Act = original_sys.Act;
Ns = user_param.Ns;
Na = user_param.Na;
N = user_param.N;
n = original_sys.n;
c = user_param.c;
% d_nd_max = original_sys.d_nd_max;
Deltax = state_discretization.Deltax{1};
X_center = state_discretization.X_center{1};
Nx = user_param.Nx;
a_X = original_sys.state_space_lower_left_corner';
b_X = original_sys.state_space_upper_right_corner';
vertices_s = state_discretization.vertices_s;
% vertices_c = disturbance_discretization.vertices_c;
precision = solver_opts.precision;
Ytrain = samples.Ytrain;
Xtrain = samples.Xtrain;
% Wtrain = samples.Wtrain;
S_not_terminal = user_param.S_not_terminal;
alpha = user_param.alpha;
% compute_clopper_pearson_table_yn = solver_opts.compute_clopper_pearson_table_yn;
sigmaf = solver_opts.sigmaf;
sigmal = solver_opts.sigmal;
lambda = solver_opts.lambda;
Lx_s_a = original_sys.max_Lx_s_a;
Lk = sigmaf/sigmal;
solver_opt.Lk = Lk;

disp('Obtaining empirical CME...')

k_YY = [];
k_YY_reg_inv = [];
z = 0;
[Phi, beta,  Phi_z, beta_z, Phi_norm, k_YY, k_YY_reg_inv] = cme_fcn(Xtrain, Ytrain, z, sigmaf, sigmal, lambda, k_YY, k_YY_reg_inv, Na);

disp('Obtaining UMDP abstraction from CME...')

tic

% beta = cell(Ns+1, user_param.Na);
% Sp = cell(Ns+1, user_param.Na);
q = cell(Ns+1, user_param.Na);
r = cell(Ns+1, user_param.Na);
% x_supp_beta = cell(Ns+1, user_param.Na);
if (Ns + 1 < 65535)
    aux_S = uint16( 1:(Ns + 1) );
else
    aux_S = uint32( 1:(Ns + 1) );
end

bootstrap_yn = ~true;
if ( bootstrap_yn )
    disp('EStimating epsilon_learn via bootstrap...')
    Niter_boot = 1;
    for a = 1:Na
    Dtrain = [Xtrain{a}; Ytrain];
    phi_diff_norm = zeros(Niter_boot, 1);
    for i = 1:Niter_boot
        % Resample
        Dtraintilde = datasample(Dtrain, size(Dtrain, 2), 2, 'Replace', true);
        Xtraintilde = Dtraintilde(1:n, :);
        Ytraintilde = Dtraintilde(n+1:2*n, :);
        % Retrain
        phi_diff_norm(i) = cme_diff_norm_fcn(Xtrain{a}, Xtraintilde, Ytrain, Ytraintilde, sigmaf, sigmal, lambda);
        if (rem(i,10) == 0)
            disp(i)
        end
    end

    eps_boot(a) = quantile(phi_diff_norm, 0.99)
    end
end

Q1 = kernel_gauss_fcn(X_center', X_center', sigmal, sigmaf);
% Q1 = (Q1 + Q1')/2;
if (min(eig(Q1)) < 0)
    Q1 = Q1 + 1e-7*eye(size(Q1));
end
for a = 1:Na
    Q2{a} = kernel_gauss_fcn(X_center', Xtrain{a}, sigmal, sigmaf);
    Q3{a} = kernel_gauss_fcn(Xtrain{a}, Xtrain{a}, sigmal, sigmaf);
%     Q3{a} = (Q3{a} + Q3{a}')/2;
end
eps_diff = zeros(Ns + 1, Na);
eps_proj = zeros(Ns + 1, Na);
epsilon = zeros(Ns + 1, Na);

% Bound || p - \hat p || <= eps_discr
% Consider the case that x \in interval
% and \hat p is supported in just two points
dist_to_discrete = @(xu) mmd_dist_to_grid_superbrief(xu, X_center, sigmaf, sigmal);
lb = X_center(1);
ub = X_center(2);
[~, eps_discr] = golden_section_fcn(lb, ub, dist_to_discrete, 1e-3, 100);

ii = 1;
step = 1;
error_flag_computing_abstraction = false;
for s = S_not_terminal
    y = X_center(s);
    beta_y = beta(y);
%             if (s == 81)% && (a == 1)
%                 5
%             end
    for a = 1:user_param.Na
        if ( ~user_param.controller_given_yn )
            u = Act(:,a);
        else
            u = Act(:, solution.controller_given(s) );
        end
        % Project beta_y onto grid
        [dist_proj, beta_y_proj] = mmd_dist_to_grid_brief(beta_y, Xtrain{a}', X_center, sigmaf, sigmal, Q1);
        q{s,a} = -2*Q2{a}*beta_y;
        r{s,a} = beta_y'*Q3{a}*beta_y;
%         q{s,a} = -2*Q1*beta_y_proj;
%         r{s,a} = beta_y_proj'*Q1*beta_y_proj;
        % Bound || phi(x) - phi(z) || <= eps_diff
        lb = y - Deltax/2;
        ub = y + Deltax/2;
        beta_diff = @(z) beta(z) - beta(y);
        fobj = @(z) sqrt( beta_diff(z)'*Q3{a}*beta_diff(z) );
        [~, fmax] = golden_section_fcn(lb, ub, fobj, 1e-3, 100);
        eps_diff(s,a) = fmax;
        eps_proj(s,a) = dist_proj;
        epsilon(s,a) = fmax + eps_discr + dist_proj;
    end
    % Display progress
    [ii, step] = disp_progress_fcn(length( S_not_terminal ), ii, 20, step);
end
% epsilon = 0.1;
eps_learn = 0;%0.01*sqrt(log(1/user_param.beta)/user_param.N)*sigmaf/sigmal;
epsilon = epsilon + eps_learn;
solution.abstraction_time = toc/60;
disp('Abstraction complete')
disp(solution.abstraction_time)
% fprintf('eps_discr = %f \n', eps_discr)
% fprintf('eps_diff = %f \n', eps_diff)
% fprintf('eps_proj = %f \n', eps_proj)
epsilon_max = max(epsilon(:));
fprintf('epsilon_max = %f \n', epsilon_max)


solution.Q = Q1;
solution.q = q;
solution.r = r;
solution.epsilon = epsilon;
solution.eps_diff = eps_diff;
solution.eps_discr = eps_discr;
solution.eps_proj = eps_proj;
solution.eps_learn = eps_learn;

error_flags.error_flag_computing_abstraction = error_flag_computing_abstraction;

end