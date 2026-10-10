function [solution, error_flags] = obtain_abstraction_pendulum_2layer_single_goal_l4dc2025_fcn_old(user_param, state_discretization, original_sys, solver_opts, samples, solution)

Act = original_sys.Act;
Ns = user_param.Ns;
Nq = user_param.Nq;
Na = user_param.Na;
N = user_param.N;
c = user_param.c;
Deltax = state_discretization.Deltax;
Deltay = state_discretization.Deltay;
Nx = user_param.Nx;
Ny = user_param.Ny;
Nx_coarse = user_param.Nx_coarse;
Ny_coarse = user_param.Ny_coarse;
a_X = original_sys.state_space_lower_left_corner;
b_X = original_sys.state_space_upper_right_corner;
vertices_s = state_discretization.vertices_s;
vertices_q = state_discretization.vertices_q;
precision = solver_opts.precision;
coarse_Deltax_ratio = user_param.coarse_Deltax_ratio;
coarse_Deltay_ratio = user_param.coarse_Deltay_ratio;
Xi_hat = samples.Xi_hat;
Xi_hat_c = samples.Xi_hat_c;
Lw_s_a = original_sys.Lw_s_a;
size_cluster = samples.size_cluster;
S_not_terminal = user_param.S_not_terminal;
alpha = user_param.alpha;
compute_clopper_pearson_table_yn = solver_opts.compute_clopper_pearson_table_yn;
clopper_pearson_yn = solver_opts.clopper_pearson_yn;
epsilon_support = user_param.epsilon_support;
n_transitions_est = user_param.n_transitions_est;
S_target = user_param.S_target;

tic

% First, estimate the support of w
if (solver_opts.learn_support)
    beta_support = alpha/2;
    user_param.N_support = ceil( log(1/beta_support)/log(1/(1-epsilon_support)) );
    if ( user_param.N_support > user_param.N )
        disp('Pick a smaller epsilon_support...')
        return
    end
    w_norm = zeros(1, user_param.N_support);
    for wi = 1:user_param.N_support
        w_norm(wi) = norm(samples.Xi_hat(:, wi), "inf");
    end
    solution.c = max( w_norm );
else
    solution.c = original_sys.rho;
    epsilon_support = 0;
end

fprintf('c_hat = %f \n', solution.c)

% Hoeffding's inequality
beta = 0.5*alpha/n_transitions_est;
epsilon = sqrt( log(1/beta)/(2*N) );

parents = cell(Ns+1, 1);
children = cell(Nq+1, 1);
P_lower_coarse = cell(Ns+1, user_param.Na);
P_upper_coarse = cell(Ns+1, user_param.Na);
Qp = cell(Ns+1, user_param.Na);
if (Nq + 1 < 65535)
    aux_Q = uint16( 1:(Nq + 1) );
else
    aux_Q = uint32( 1:(Nq + 1) );
end
error_flag_computing_abstraction = false;

% Compute parents
parents{Ns+1} = Nq+1;
for s = 1:Ns
    s_x_center = state_discretization.X_center(s);
    s_y_center = state_discretization.Y_center(s);
    n_x = floor( s_x_center/state_discretization.Deltax_coarse );
    n_y = floor( s_y_center/state_discretization.Deltay_coarse );
    parents{s} = n_y*user_param.Ny_coarse + n_x + 1;
end
n_children_per_parent = user_param.coarse_Deltax_ratio*user_param.coarse_Deltay_ratio;

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

step = 1;
ii = 1;
for s = S_not_terminal
    for a = 1:user_param.Na
        if ( ~user_param.controller_given_yn )
            u = Act(:,a);
        else
            u = Act(:, solution.controller_given(s) );
        end
        % Compute Reach_nd_s_a
        w = original_sys.center_samples; % Center of W
        [a_Post_w0, b_Post_w0, error_flag_post] = compute_reach_simplified_pendulum_fcn(vertices_s{s},u,w,original_sys,user_param);
        % Inflate post to account for clustering
        [a_Reach_nd_s_a, b_Reach_nd_s_a] = compute_Reach_nd_simplified_pendulum_q_u(a_Post_w0, b_Post_w0, original_sys.Lw_s_a{s,a}, user_param.c);
        % Check intersection with all Qp
        [Qp_intersects, error_flag_intersections_nd_coarse] = intersections_simplified_pendulum_fcn(a_Reach_nd_s_a, b_Reach_nd_s_a, state_discretization.Deltax_coarse,  state_discretization.Deltay_coarse, user_param.Nx_coarse, user_param.Ny_coarse, Nq, a_X, b_X, solver_opts.precision);
        % Remove clusters qp that aren't fully contained in Reach_nd_s_a
        % Remove clusters qp that aren't fully contained in Reach_nd_s_a
        [Qp{s,a}, Qp_flag_removed{s,a}] = remove_qp_with_illegitimate_sp_single_goal_fcn( Qp_intersects, vertices_q, a_Reach_nd_s_a, b_Reach_nd_s_a, Nq , solver_opts, children, S_target);
        if ( length( Qp{s,a} ) > 1 )
            5
        end
        % Compute TP bounds
        aux_P_upper_coarse = zeros(Ns + 1,1);
        aux_P_lower_coarse = zeros(Ns + 1,1);
        for w_ind = 1:user_param.Nc
            w = samples.Xi_hat_c(:, w_ind);
            % Compute Post_s_a_w
            [a_Post, b_Post, error_flag_post] = compute_reach_simplified_pendulum_fcn(state_discretization.vertices_s{s},u,w,original_sys,user_param);
            % Inflate post to account for clustering
            a_Post = a_Post - [0; 1]*original_sys.Lw_s_a{s,a}*samples.size_cluster(w_ind);
            b_Post = b_Post + [0; 1]*original_sys.Lw_s_a{s,a}*samples.size_cluster(w_ind);
            % Check intersection with all qp
            [Qp_intersects, error_flag_intersections_coarse] = intersections_simplified_pendulum_single_goal_fcn(a_Post, b_Post, state_discretization.Deltax_coarse,  state_discretization.Deltay_coarse, user_param.Nx_coarse, user_param.Ny_coarse, Nq, a_X, b_X, solver_opts.precision, user_param);
            % Check if post is subsumed by some qps
            [Q_ind_subsumed, error_flag_subsumed_coarse] = subsumed_q_a_w_simplified_pendulum_single_goal(a_Post, b_Post, state_discretization.vertices_q, Qp_intersects, original_sys.state_space_lower_left_corner, original_sys.state_space_upper_right_corner, Nq, solver_opts.precision, user_param);
            for qp_ind = 1:length(Qp_intersects)
                qp = Qp_intersects(qp_ind);
                if (any( ismember(Qp{s,a}, qp) ))
                    aux_P_upper_coarse(qp) = aux_P_upper_coarse(qp) + samples.weight_norm(w_ind);
                    if (Q_ind_subsumed(qp_ind))
                        aux_P_lower_coarse(qp) = aux_P_lower_coarse(qp) + samples.weight_norm(w_ind);
                    end
                end
            end
            if ( (error_flag_post == true) || (error_flag_intersections_coarse == true) || (error_flag_subsumed_coarse == true) )
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
    %     solution.support_P_coarse{s,a} = q;
    Qp{s,a} = Nq + 1;
end



% Obtain IMDP abstraction

disp('Second, fine abstraction...')

% solution.Post_q_a_w = cell(Ns, user_param.Na, user_param.Nc); % Maybe not needed (save memory)
P_lower = cell(Ns+1, user_param.Na);
P_upper = cell(Ns+1, user_param.Na);
solution.support_P = cell(Ns+1, user_param.Na);
Sp = cell(Ns+1, user_param.Na);

step = 1;
ii = 1;
for s = S_not_terminal
    for a = 1:user_param.Na
        if ( ~user_param.controller_given_yn )
            u = Act(:,a);
        else
            u = Act(:, solution.controller_given(s) );
        end
        % Compute Reach_nd_s_a
        w = original_sys.center_samples; % Center of W
        [a_Post_w0, b_Post_w0, error_flag_post] = compute_reach_simplified_pendulum_fcn(vertices_s{s},u,w,original_sys,user_param);
        % Inflate post to account for clustering
        [a_Reach_nd_s_a, b_Reach_nd_s_a] = compute_Reach_nd_simplified_pendulum_q_u(a_Post_w0, b_Post_w0, original_sys.Lw_s_a{s,a}, user_param.c);
        % Check intersection with all Qp
        [Sp_intersects, error_flag_intersections_nd_coarse] = intersections_simplified_pendulum_single_goal_fcn(a_Reach_nd_s_a, b_Reach_nd_s_a, state_discretization.Deltax,  state_discretization.Deltay, user_param.Nx, user_param.Ny, Ns, a_X, b_X, solver_opts.precision, user_param);
        Sp{s,a} = Sp_intersects;
        % Assign smaller probability yo every state in Sp
        aux_P_upper = zeros(Ns + 1,1);
        aux_P_lower = zeros(Ns + 1,1);
        for w_ind = 1:user_param.Nc
            w = samples.Xi_hat_c(:,w_ind);
            % Compute Post_q_a_w
            [a_Post, b_Post, error_flag_post] = compute_reach_simplified_pendulum_fcn(state_discretization.vertices_s{s},u,w,original_sys,user_param);
            % Inflate post to account for clustering
            a_Post = a_Post - [0; 1]*original_sys.Lw_s_a{s,a}*samples.size_cluster(w_ind);
            b_Post = b_Post + [0; 1]*original_sys.Lw_s_a{s,a}*samples.size_cluster(w_ind);
            % Check intersection with all qp
            [Sp_intersects, error_flag_intersections] = intersections_simplified_pendulum_single_goal_fcn(a_Post, b_Post, state_discretization.Deltax,  state_discretization.Deltay, user_param.Nx, user_param.Ny, Ns, a_X, b_X, solver_opts.precision, user_param);
            % Check if post is subsumed by some qps
            [Sp_ind_subsumed, error_flag_subsumed] = subsumed_q_a_w_simplified_pendulum_single_goal(a_Post, b_Post, state_discretization.vertices_s, Sp_intersects, original_sys.state_space_lower_left_corner, original_sys.state_space_upper_right_corner, Ns, solver_opts.precision, user_param);
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


% Add epsilons

% Compute epsilon
n_bounds_per_sp = 1;
n_bounds_per_qp = 2;

n_transitions_actual = 0;
for s = S_not_terminal
    if (isfield(user_param, 'Na'))
        for a = 1:Na
            n_transitions_actual = n_transitions_actual + n_bounds_per_sp*length( Sp{s,a} );
        end
    else
        n_transitions_actual = n_transitions_actual + n_bounds_per_sp*length( Sp{q} );
    end
end

for s = S_not_terminal
    if (isfield(user_param, 'Na'))
        for a = 1:Na
            n_transitions_actual = n_transitions_actual + n_bounds_per_qp*length( Qp{s,a} );
        end
    else
        n_transitions_actual = n_transitions_actual + n_bounds_per_qp*length( Qp{q} );
    end
end

fprintf('Actual number of transitions = %d. \n', n_transitions_actual)

if ( n_transitions_actual > n_transitions_est)
    disp('The estimated number of transitions is smaller than the actual number of transitions of the abstraction. Restart with a higher estimate...')
    return
end

for s = S_not_terminal
    for a = 1:Na
        P_lower{s,a} = max(P_lower{s,a} - epsilon, 0);
        P_upper{s,a} = min(P_upper{s,a} + epsilon, 1);
        P_lower_coarse{s,a} = max(P_lower_coarse{s,a} - epsilon, 0);
        P_upper_coarse{s,a} = min(P_upper_coarse{s,a} + epsilon, 1);
    end
end

% Account for epsilon_support (assuming beta_support = alpha/2)
for s = S_not_terminal
    for a = 1:Na
        if (Sp{s,a}(end) == Ns+1)
            P_upper{s,a}(end) = min(1, P_upper{s,a}(end) + user_param.epsilon_support);
        else
            Sp{s,a} = [Sp{s,a}; Ns+1];
            P_lower{s,a} = [P_lower{s,a}; 0];
            P_upper{s,a} = [P_upper{s,a}; user_param.epsilon_support];
        end
    end
end

for s = S_not_terminal
    for a = 1:Na
        if ( isempty( Qp{s,a} ) ) % Can happen bc I've discarded the qp that don't have all kids in Sp
            Qp{s,a} = Nq+1;
            P_lower_coarse{s,a} = 0;
            P_upper_coarse{s,a} = user_param.epsilon_support;
        else
            if (Qp{s,a}(end) == Nq + 1)
                Qp{s,a} = Qp{s,a};
                P_upper_coarse{s,a}(end) = min(1, P_upper_coarse{s,a}(end) + user_param.epsilon_support);
            else
                Qp{s,a} = [Qp{s,a}; Nq+1];
                P_lower_coarse{s,a} = [P_lower_coarse{s,a}; 0];
                P_upper_coarse{s,a} = [P_upper_coarse{s,a}; user_param.epsilon_support];
            end
        end
    end
end

solution.Qp_flag_removed = Qp_flag_removed;
solution.P_lower = P_lower;
solution.P_upper = P_upper;
solution.Sp = Sp;
solution.n_transitions_actual = n_transitions_actual;
solution.epsilon = epsilon;
solution.epsilon_support = epsilon_support;
solution.beta = beta;
solution.beta_support = beta_support;
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
