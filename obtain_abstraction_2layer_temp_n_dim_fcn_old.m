function [solution, error_flags] = obtain_abstraction_2layer_temp_n_dim_fcn(user_param, state_discretization, disturbance_discretization, original_sys, solver_opts, samples, solution)

Act = original_sys.Act;
Ns = user_param.Ns;
Nq = user_param.Nq;
Na = user_param.Na;
N = user_param.N;
n = original_sys.n;
c = user_param.c;
Deltax = state_discretization.Deltax;
Deltax_coarse = state_discretization.Deltax_coarse;
Nx = user_param.Nx;
Nx_coarse = user_param.Nx_coarse;
a_X = original_sys.state_space_lower_left_corner';
b_X = original_sys.state_space_upper_right_corner';
a_c0 = original_sys.disturbance_space_lower_left_corner';
b_c0 = original_sys.disturbance_space_upper_right_corner';
vertices_s = state_discretization.vertices_s;
vertices_q = state_discretization.vertices_q;
vertices_c = disturbance_discretization.vertices_c;
precision = solver_opts.precision;
Xi_hat = samples.Xi_hat;
Xi_hat_c = samples.Xi_hat_c;
% Lw_s_a = original_sys.Lw_s_a;
size_cluster = samples.size_cluster;
S_not_terminal = user_param.S_not_terminal;
alpha = user_param.alpha;
compute_clopper_pearson_table_yn = solver_opts.compute_clopper_pearson_table_yn;
vertices_c_0 = [a_c0, b_c0];

disp('Obtaining IMDP abstraction (w learned support)...')


tic

parents = cell(Ns+1, 1);
children = cell(Nq+1, 1);
P_lower_coarse = cell(Ns+1, user_param.Na);
P_upper_coarse = cell(Ns+1, user_param.Na);
Qp = cell(Ns+1, user_param.Na);
Qp_flag_removed = cell(Ns+1, user_param.Na);
if (Nq + 1 < 65535)
    aux_Q = uint16( 1:(Nq + 1) );
else
    aux_Q = uint32( 1:(Nq + 1) );
end


% Compute parents
parents{Ns+1} = Nq+1;
s_x_center = cell(1, n);
n_x = cell(1, n);
for s = 1:Ns
    for di = 1:n
        s_x_center{di} = state_discretization.X_center{di}(s);
        n_x{di} = floor( ( s_x_center{di} - a_X(di) )/state_discretization.Deltax_coarse{di} );
    end
    if ( n == 2 )
        parents{s} = n_x{2}*user_param.Nx_coarse{2} + n_x{1} + 1;
    elseif ( n == 3 )
        parents{s} =  n_x{3}*Nx_coarse{1}*Nx_coarse{2} + n_x{2}*Nx_coarse{1} + n_x{1} + 1;
    elseif ( n == 4 )
        parents{s} =  n_x{4}*Nx_coarse{1}*Nx_coarse{2}*Nx_coarse{3} + n_x{3}*Nx_coarse{1}*Nx_coarse{2} + n_x{2}*Nx_coarse{1} + n_x{1} + 1;
    end
end

% Compute children
for q = aux_Q
    for s = 1:Ns+1
        if ( parents{s} == q )
            children{q} = [children{q}; s];
        end
    end
end


% Obtain coarse abstraction

disp('First, coarse abstraction...')

error_flag_computing_abstraction = false;
step = 1;
ii = 1;
for s = S_not_terminal
    for a = 1:user_param.Na
        if ( ~user_param.controller_given_yn )
            u = Act(:,a);
        else
            u = Act(:, solution.controller_given(s) );
        end
        % Compute Reach_s_a_c
        [a_Reach, b_Reach, error_flag_post_nd_coarse] = compute_reach_simplified_temp_n_dim_fcn(vertices_s{s},u,vertices_c_0,original_sys,user_param);
        % Check intersection with all qp
        [Qp_intersects, error_flag_intersections_nd_coarse] = intersections_simplified_temp_n_dim_fcn(a_Reach, b_Reach, Deltax_coarse, Nx_coarse, Nq, a_X, b_X, precision, n);
        % Remove clusters qp that aren't fully contained in Reach_nd_s_a
        [Qp{s,a}, Qp_flag_removed{s,a}] = remove_qp_with_illegitimate_sp_fcn( Qp_intersects, vertices_q, a_Reach, b_Reach, Nq , solver_opts);
        % Compute TP bounds
        aux_P_upper_coarse = zeros(Ns + 1,1);
        aux_P_lower_coarse = zeros(Ns + 1,1);
        for c_ind = 1:user_param.Nc
            % Compute Reach_s_a_c
            [a_Reach, b_Reach, error_flag_post_coarse] = compute_reach_simplified_temp_n_dim_fcn(vertices_s{s},u,vertices_c{c_ind},original_sys,user_param);
            % Check intersection with all qp
            [Qp_intersects, error_flag_intersections_coarse] = intersections_simplified_temp_n_dim_fcn(a_Reach, b_Reach, Deltax_coarse, Nx_coarse, Nq, a_X, b_X, precision, n);
            % Check if post is subsumed by some qps
            [Qp_ind_subsumed, error_flag_subsumed_coarse] = subsumed_q_a_w_simplified_n_dim_fcn(a_Reach, b_Reach, state_discretization.vertices_q, Qp_intersects, a_X, b_X, Nq, solver_opts.precision);
            for qp_ind = 1:length(Qp_intersects)
                qp = Qp_intersects(qp_ind);
                aux_P_upper_coarse(qp) = aux_P_upper_coarse(qp) + samples.weight_norm(c_ind);
                if (Qp_ind_subsumed(qp_ind))
                    aux_P_lower_coarse(qp) = aux_P_lower_coarse(qp) + samples.weight_norm(c_ind);
                end
            end
            if ( (error_flag_post_coarse == true) || (error_flag_intersections_coarse == true) || (error_flag_post_nd_coarse == true) || (error_flag_intersections_nd_coarse == true) || (error_flag_subsumed_coarse == true) )
                error_flag_computing_abstraction = true;
                disp('Problem found...')
            end
        end
        P_lower_coarse{s,a} = aux_P_lower_coarse(Qp{s,a});
        P_upper_coarse{s,a} = aux_P_upper_coarse(Qp{s,a});
        if ( (error_flag_intersections_nd_coarse == true) || ( length(Qp{s,a}) ~= length(P_lower_coarse{s,a}) ) || (length(Qp{s,a}) ~= length(P_upper_coarse{s,a}) ) )
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
    P_lower_coarse{s,a} = 1;
    P_upper_coarse{s,a} = 1;
    Qp{s,a} = Nq + 1;
end

% Next, fine abstraction

disp('Next, fine abstraction...')

P_lower = cell(Ns+1, user_param.Na);
P_upper = cell(Ns+1, user_param.Na);
solution.support_P = cell(Ns+1, user_param.Na);
Sp = cell(Ns+1, user_param.Na);

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
        % Compute Reach_s_a_c
        [a_Reach, b_Reach, error_flag_post_nd] = compute_reach_simplified_temp_n_dim_fcn(vertices_s{s},u,vertices_c_0,original_sys,user_param);
        % Check intersection with all qp
        [Sp_intersects, error_flag_intersections_nd] = intersections_simplified_temp_n_dim_fcn(a_Reach, b_Reach, Deltax, Nx, Ns, a_X, b_X, precision, n);
        % Remove clusters qp that aren't fully contained in Reach_nd_s_a
        Sp{s,a} = Sp_intersects;
        aux_Sp = [];
        aux_P_upper = zeros(Ns + 1,1);
        aux_P_lower = zeros(Ns + 1,1);
        for c_ind = 1:user_param.Nc
            % Compute Reach_s_a_c
            [a_Reach, b_Reach, error_flag_post] = compute_reach_simplified_temp_n_dim_fcn(vertices_s{s},u,vertices_c{c_ind},original_sys,user_param);
            % Check intersection with all qp
            [Sp_intersects, error_flag_intersections] = intersections_simplified_temp_n_dim_fcn(a_Reach, b_Reach, Deltax, Nx, Ns, a_X, b_X, precision, n);
            % Check if post is subsumed by some qps
            [Sp_ind_subsumed, error_flag_subsumed] = subsumed_q_a_w_simplified_n_dim_fcn(a_Reach, b_Reach, state_discretization.vertices_s, Sp_intersects, a_X, b_X, Ns, solver_opts.precision);
            for sp_ind = 1:length(Sp_intersects)
                sp = Sp_intersects(sp_ind);
                aux_P_upper(sp) = aux_P_upper(sp) + samples.weight_norm(c_ind);
                if (Sp_ind_subsumed(sp_ind))
                    aux_P_lower(sp) = aux_P_lower(sp) + samples.weight_norm(c_ind);
                end
            end
            if ( (error_flag_post == true) || (error_flag_post_nd == true) || (error_flag_intersections_nd == true) || (error_flag_intersections == true) || (error_flag_subsumed == true) )
                error_flag_computing_abstraction = true;
                disp('Problem found...')
            end
        end
        % Account for Nc+1 region of the disturbance partition
        Sp{s,a} = [Sp{s,a}; Ns + 1];
        Sp{s,a} = unique( Sp{s,a} );
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

solution.Qp_flag_removed = Qp_flag_removed;
solution.P_lower = P_lower;
solution.P_upper = P_upper;
solution.Sp = Sp;
solution.n_transitions = n_transitions;
solution.beta = beta;
solution.epsilon_support = epsilon_support;
solution.P_lower_coarse = P_lower_coarse;
solution.P_upper_coarse = P_upper_coarse;
solution.Qp = Qp;
solution.children = children;
solution.parents = parents;

error_flags.error_flag_intersections = error_flag_intersections;
error_flags.error_flag_intersections_nd_coarse = error_flag_intersections_coarse;
error_flags.error_flag_intersections_nd = error_flag_intersections_nd;
error_flags.error_flag_intersections_nd_coarse = error_flag_intersections_nd_coarse;
error_flags.error_flag_post = error_flag_post;
error_flags.error_flag_subsumed = error_flag_subsumed;
error_flags.error_flag_subsumed_coarse = error_flag_subsumed_coarse;
error_flags.error_flag_computing_abstraction = error_flag_computing_abstraction;

end