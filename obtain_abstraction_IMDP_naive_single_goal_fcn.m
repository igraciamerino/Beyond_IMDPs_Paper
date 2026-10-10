function [solution, error_flags] = obtain_abstraction_IMDP_naive_single_goal_fcn(user_param, state_discretization, original_sys, solver_opts, samples, solution)

Act = original_sys.Act;
Ns = user_param.Ns;
Na = user_param.Na;
N = user_param.N;
c = user_param.c;
Deltax = state_discretization.Deltax;
Deltay = state_discretization.Deltay;
Nx = user_param.Nx;
Ny = user_param.Ny;
a_X = original_sys.state_space_lower_left_corner;
b_X = original_sys.state_space_upper_right_corner;
vertices_s = state_discretization.vertices_s;
precision = solver_opts.precision;
Xi_hat = samples.Xi_hat;
Xi_hat_c = samples.Xi_hat_c;
Lw_s_a = original_sys.Lw_s_a;
size_cluster = samples.size_cluster;
S_not_terminal = user_param.S_not_terminal;
alpha = user_param.alpha;
disp('Obtaining naive IMDP abstraction...')

tic

Sp = cell(Ns+1, user_param.Na);
P_lower = cell(Ns+1, user_param.Na);
P_upper = cell(Ns+1, user_param.Na);

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
        aux_P_upper = zeros(Ns + 1,1);
        aux_P_lower = zeros(Ns + 1,1);
        Sp{s,a} = 1:Ns+1;
        Sp{s,a} = Sp{s,a}';
        for w_ind = 1:user_param.Nc
            w = samples.Xi_hat_c(:,w_ind);
            % Compute Post_q_a_w
            [a_Post, b_Post, error_flag_post] = compute_reach_simplified_fcn(state_discretization.vertices_s{s},u,w,original_sys,user_param);
            % Inflate post to account for clustering
            a_Post = a_Post - original_sys.Lw_s_a{s,a}*samples.size_cluster(w_ind);
            b_Post = b_Post + original_sys.Lw_s_a{s,a}*samples.size_cluster(w_ind);
            % Check intersection with all qp
            [Sp_intersects, error_flag_intersections] = intersections_simplified_single_goal_fcn(a_Post, b_Post, Deltax, Deltay, Nx, Ny, Ns, a_X, b_X, solver_opts.precision, user_param);
            % Check if post is subsumed by some qps
            [Sp_ind_subsumed, error_flag_subsumed] = subsumed_q_a_w_simplified(a_Post, b_Post, state_discretization.vertices_s, Sp_intersects, original_sys.state_space_lower_left_corner, original_sys.state_space_upper_right_corner, Ns, solver_opts.precision);
            for sp_ind = 1:length(Sp_intersects)
                sp = Sp_intersects(sp_ind);
                aux_P_upper(sp) = aux_P_upper(sp) + samples.weight_norm(w_ind);
                if (Sp_ind_subsumed(sp_ind))
                    aux_P_lower(sp) = aux_P_lower(sp) + samples.weight_norm(w_ind);
                end
            end
            if ( (error_flag_post == true) || (error_flag_intersections == true) || (error_flag_subsumed == true) )
                error_flag_computing_abstraction = true;
                disp('Problem found...')
            end
        end
        P_lower{s,a} = aux_P_lower;
        P_upper{s,a} = aux_P_upper;
        if ( length(P_upper{s,a}) ~= length(P_lower{s,a}) )
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
    P_lower{s,a} = 1;
    P_upper{s,a} = 1;
end

solution.abstraction_time = toc/60;
disp('Abstraction complete')
disp(solution.abstraction_time)

if ( original_sys.data_driven_yn )
    % Add epsilons
    n_bounds_per_sp = 2;
    if ( strcmp( solver_opts.abstraction_choice, '2layer') )
        n_bounds_per_qp = 2;
    else
        n_bounds_per_qp = 0;
        P_lower_coarse = [];
        P_upper_coarse = [];
        Qp = [];
    end
    [Sp, P_lower, P_upper, Qp, P_lower_coarse, P_upper_coarse, n_transitions, beta, beta_support, epsilon, epsilon_support] = add_epsilons_fcn(user_param, solver_opts, samples, Sp, P_lower, P_upper, Qp, P_lower_coarse, P_upper_coarse, n_bounds_per_sp, n_bounds_per_qp);
else
    beta = [];
    epsilon_support = [];
    n_transitions = [];
end

solution.P_lower = P_lower;
solution.P_upper = P_upper;
solution.Sp = Sp;
solution.n_transitions = n_transitions;
solution.beta = beta;
solution.epsilon_support = [];
solution.P_lower_coarse = [];
solution.P_upper_coarse = [];
solution.Qp = [];
solution.children = [];
solution.parents = [];

error_flag_intersections_nd = false;
error_flags.error_flag_intersections = error_flag_intersections;
error_flags.error_flag_intersections_nd = error_flag_intersections_nd;
error_flags.error_flag_post = error_flag_post;
error_flags.error_flag_subsumed = error_flag_subsumed;
error_flags.error_flag_computing_abstraction = error_flag_computing_abstraction;

end