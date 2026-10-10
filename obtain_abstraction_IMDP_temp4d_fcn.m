function [solution, error_flags] = obtain_abstraction_IMDP_temp4d_fcn(user_param, state_discretization, disturbance_discretization, original_sys, solver_opts, samples, solution)

Act = original_sys.Act;
Ns = user_param.Ns;
Na = user_param.Na;
N = user_param.N;
n = original_sys.n;
c = user_param.c;
Deltax = state_discretization.Deltax;
Nx = user_param.Nx;
a_X = original_sys.state_space_lower_left_corner';
b_X = original_sys.state_space_upper_right_corner';
vertices_s = state_discretization.vertices_s;
vertices_c = disturbance_discretization.vertices_c;
precision = solver_opts.precision;
Xi_hat = samples.Xi_hat;
Xi_hat_c = samples.Xi_hat_c;
% Lw_s_a = original_sys.Lw_s_a;
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
        % Compute TP bounds
%         Qp{s,a}.children = cell(1, user_param.Nc + 1);
        aux_Sp = [];
        aux_P_upper = zeros(Ns + 1,1);
        aux_P_lower = zeros(Ns + 1,1);
        for c_ind = 1:user_param.Nc
            % Compute Reach_s_a_c
            [a_Reach, b_Reach, error_flag_post] = compute_reach_simplified_temp_n_dim_fcn(vertices_s{s},u,vertices_c{c_ind},original_sys,user_param);
            % Check intersection with all qp
            [Sp_intersects, error_flag_intersections] = intersections_simplified_temp_n_dim_fcn(a_Reach, b_Reach, Deltax, Nx, Ns, a_X, b_X, precision, n);
            aux_Sp = [aux_Sp; Sp_intersects];
            % Check if post is subsumed by some qps
            [Sp_ind_subsumed, error_flag_subsumed] = subsumed_q_a_w_simplified_4d_fcn(a_Reach, b_Reach, state_discretization.vertices_s, Sp_intersects, original_sys.state_space_lower_left_corner, original_sys.state_space_upper_right_corner, Ns, solver_opts.precision);
            for sp_ind = 1:length(Sp_intersects)
                sp = Sp_intersects(sp_ind);
                aux_P_upper(sp) = aux_P_upper(sp) + samples.weight_norm(c_ind);
                if (Sp_ind_subsumed(sp_ind))
                    aux_P_lower(sp) = aux_P_lower(sp) + samples.weight_norm(c_ind);
                end
            end
            if ( (error_flag_post == true) || (error_flag_intersections == true) || (error_flag_subsumed == true) )
                error_flag_computing_abstraction = true;
                disp('Problem found...')
            end
        end
        % Account for Nc+1 region of the disturbance partition
        aux_Sp = [aux_Sp; Ns + 1];
        Sp{s,a} = unique( aux_Sp );
        aux_P_upper(end) = aux_P_upper(end) + samples.weight_norm(end);
        P_lower{s,a} = aux_P_lower(Sp{s,a});
        P_upper{s,a} = aux_P_upper(Sp{s,a});
        error_flag_intersections_nd = false;
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