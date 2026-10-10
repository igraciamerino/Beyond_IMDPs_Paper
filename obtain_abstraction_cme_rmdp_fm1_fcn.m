function [solution, error_flags] = obtain_abstraction_cme_rmdp_fm1_fcn(user_param, state_discretization, disturbance_discretization, original_sys, solver_opts, samples, solution)

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
Xi_hat = samples.Xi_hat;
% Xi_hat_c = samples.Xi_hat_c;
% Lw_s_a = original_sys.Lw_s_a;
size_cluster = samples.size_cluster;
S_not_terminal = user_param.S_not_terminal;
alpha = user_param.alpha;
% compute_clopper_pearson_table_yn = solver_opts.compute_clopper_pearson_table_yn;
sigmaf = solver_opts.sigmaf;
sigmal = solver_opts.sigmal;

Lx_s_a = original_sys.max_Lx_s_a;
Lk = sqrt(2)*sigmaf/sigmal;
solver_opt.Lk = Lk;
disp('Obtaining IMDP abstraction (w learned support)...')

tic

beta = cell(Ns+1, user_param.Na);
Sp = cell(Ns+1, user_param.Na);
if (Ns + 1 < 65535)
    aux_S = uint16( 1:(Ns + 1) );
else
    aux_S = uint32( 1:(Ns + 1) );
end


ii = 1;
step = 1;
error_flag_computing_abstraction = false;
for s = S_not_terminal
    for a = 1:user_param.Na
        if ( ~user_param.controller_given_yn )
            u = Act(:,a);
        else
            u = Act(:, solution.controller_given(s) );
        end
        % Compute Reach_nd_s_a
        aux_beta = zeros(Ns + 1,1);
        for w_ind = 1:user_param.N
            w = samples.Xi_hat(:,w_ind);
            % Compute successor state
            x = X_center(s);
            xp = fm1_sys(x, u, w, 0, original_sys, user_param);
            sp = compute_sp_1d_fcn(xp, Deltax, Ns, a_X, b_X);
            aux_beta(sp) = aux_beta(sp) + 1;
            if ( sp == Ns + 1 )
                error_flag_computing_abstraction = true;
                disp('Problem found...')
            end
        end
        Sp{s,a} = user_param.S(aux_beta > 0);
        beta{s,a} = aux_beta(aux_beta > 0)/sum(aux_beta);
        if ( (length(Sp{s,a}) ~= length(beta{s,a}) ) )
            error_flag_computing_abstraction = true;
            disp('Problem found...')
        end
    end
    % Display progress
    [ii, step] = disp_progress_fcn(length( S_not_terminal ), ii, 20, step);
end


% Make Nq+1 absorbing
s = Ns + 1;
for a = 1:user_param.Na
    beta{s,a} = 1;
    solution.support_P{s,a} = s;
    Sp{s,a} = s;
end

beta_conf = alpha/(Ns*Na);
epsilon = Lk*Lx_s_a*Deltax/2 + sigmaf/sqrt(N) + sigmaf*sqrt(2*log(1/beta_conf)/N) + 2*sigmaf*sqrt( 2*( 1 - exp(-(Deltax/2/sigmal)^2) ) );


solution.abstraction_time = toc/60;
disp('Abstraction complete')
disp(solution.abstraction_time)

solution.Sp = Sp;
solution.beta = beta;
solution.epsilon = epsilon;
solution.beta_conf = beta_conf;

error_flags.error_flag_computing_abstraction = error_flag_computing_abstraction;

end