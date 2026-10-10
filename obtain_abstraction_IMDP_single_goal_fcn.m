function [solution, error_flags] = obtain_abstraction_IMDP_single_goal_fcn(user_param, state_discretization, original_sys, solver_opts, samples, solution)

Act = original_sys.Act;
Ns = user_param.Ns;
Na = user_param.Na;
N = user_param.N;
c = user_param.c;
Deltax = state_discretization.Deltax{1};
Deltay = state_discretization.Deltax{2};
Nx = user_param.Nx{1};
Ny = user_param.Nx{2};
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
compute_clopper_pearson_table_yn = solver_opts.compute_clopper_pearson_table_yn;
disp('Obtaining IMDP abstraction (w learned support)...')

tic

P_lower = cell(Ns+1, user_param.Na);
P_upper = cell(Ns+1, user_param.Na);
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
        w = original_sys.center_samples; % Center of W
        [a_Post_w0, b_Post_w0, error_flag_post] = compute_reach_simplified_fcn(vertices_s{s},u,w,original_sys,user_param);
        % Inflate post to account support of P
        [a_Reach_nd_s_a, b_Reach_nd_s_a] = compute_Reach_nd_simplified_q_u(a_Post_w0, b_Post_w0, original_sys.Lw_s_a{s,a}, user_param.c);
        % Check intersection with all Qp
        [Sp_intersects, error_flag_intersections_nd] = intersections_simplified_single_goal_fcn(a_Reach_nd_s_a, b_Reach_nd_s_a, Deltax, Deltay, Nx, Ny, Ns, a_X, b_X, solver_opts.precision, user_param);
        Sp{s,a} = Sp_intersects;
        aux_P_upper = zeros(Ns + 1,1);
        aux_P_lower = zeros(Ns + 1,1);
        for w_ind = 1:size( Xi_hat_c, 2 )
            w = Xi_hat_c(:, w_ind);
            % Compute Post_q_a_w
            [a_Post, b_Post, error_flag_post] = compute_reach_simplified_fcn(state_discretization.vertices_s{s},u,w,original_sys,user_param);
            % Inflate post to account for clustering
            a_Post = a_Post - original_sys.Lw_s_a{s,a}*samples.size_cluster(w_ind);
            b_Post = b_Post + original_sys.Lw_s_a{s,a}*samples.size_cluster(w_ind);
            % Check intersection with all qp
            [Sp_intersects, error_flag_intersections] = intersections_simplified_single_goal_fcn(a_Post, b_Post, state_discretization.Deltax,  state_discretization.Deltay, user_param.Nx, user_param.Ny, Ns, a_X, b_X, solver_opts.precision, user_param);
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
        P_lower{s,a} = aux_P_lower(Sp{s,a});
        P_upper{s,a} = aux_P_upper(Sp{s,a});

        if ( (error_flag_intersections_nd == true) || (length(Sp{s,a}) ~= length(P_lower{s,a}) ) || (length(Sp{s,a}) ~= length(P_upper{s,a}) ) )
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
    solution.support_P{s,a} = s;
    Sp{s,a} = s;
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
solution.epsilon_support = epsilon_support;
solution.P_lower_coarse = [];
solution.P_upper_coarse = [];
solution.Qp = [];
solution.children = [];
solution.parents = [];

error_flags.error_flag_intersections = error_flag_intersections;
error_flags.error_flag_intersections_nd = error_flag_intersections_nd;
error_flags.error_flag_post = error_flag_post;
error_flags.error_flag_subsumed = error_flag_subsumed;
error_flags.error_flag_computing_abstraction = error_flag_computing_abstraction;

end