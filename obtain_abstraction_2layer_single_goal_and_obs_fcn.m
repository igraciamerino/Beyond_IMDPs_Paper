function [solution, error_flags] = obtain_abstraction_2layer_single_goal_and_obs_fcn(user_param, state_discretization, original_sys, solver_opts, samples, solution)

Act = original_sys.Act;
Ns = user_param.Ns;
Nq = user_param.Nq;
Na = user_param.Na;
N = user_param.N;
c = user_param.c;
Deltax = state_discretization.Deltax;
Deltax_coarse = state_discretization.Deltax_coarse;
Nx = user_param.Nx;
Nx_coarse = user_param.Nx_coarse;
X_center = state_discretization.X_center;
a_X = original_sys.state_space_lower_left_corner;
b_X = original_sys.state_space_upper_right_corner;
vertices_s = state_discretization.vertices_s;
vertices_q = state_discretization.vertices_q;
precision = solver_opts.precision;
coarse_Deltax_ratio = user_param.coarse_Deltax_ratio;
Xi_hat = samples.Xi_hat;
Xi_hat_c = samples.Xi_hat_c;
Lw_s_a = original_sys.Lw_s_a;
size_cluster = samples.size_cluster;
S_not_terminal = user_param.S_not_terminal;
alpha = user_param.alpha;
compute_clopper_pearson_table_yn = solver_opts.compute_clopper_pearson_table_yn;
S_target = user_param.S_target;
S_obstacle = user_param.S_obstacle;
s_target = user_param.s_target;
n_target_blocks = user_param.n_target_blocks;
s_target_blocks = user_param.s_target_blocks;

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
error_flag_computing_abstraction = false;
tic


% Compute parents
parents{Ns+1} = Nq+1;
for s = 1:Ns
    s_x_center = X_center{1}(s);
    s_y_center = X_center{2}(s);
    n_x = floor( s_x_center/Deltax_coarse{1} );
    n_y = floor( s_y_center/Deltax_coarse{2} );
    q = n_y*Nx_coarse{2} + n_x + 1;
    parents{s} = q;
end
n_children_per_parent = coarse_Deltax_ratio{1}*coarse_Deltax_ratio{2};

% Compute children
for q = aux_Q
    for s = 1:Ns+1
        if ( parents{s} == q )
            children{q} = [children{q}; s];
        end
    end
end

% Delete parents that intersect S_target
for q = 1:Nq
    children_q = children{q}';
    if ( any( ismember( children_q, S_target ) ) )
        for s = children_q
            parents{s} = [];
        end
    end
end

% % Assign q_target to s_target
% for block = 1:n_target_blocks
%     s = s_target_blocks{blocki};
%     s_x_center = state_discretization.X_center(s);
%     s_y_center = state_discretization.Y_center(s);
%     n_x = floor( s_x_center/state_discretization.Deltax_coarse );
%     n_y = floor( s_y_center/state_discretization.Deltay_coarse );
%     q = n_y*user_param.Ny_coarse + n_x + 1;
%     parents{s} = q;
% end

% % Recompute children
% for q = aux_Q
%     children{q} = [];
%     for s = 1:Ns+1
%         if ( parents{s} == q )
%             children{q} = [children{q}; s];
%         end
%     end
% end

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
        [a_Post_w0, b_Post_w0, error_flag_post] = compute_reach_simplified_fcn(vertices_s{s},u,w,original_sys,user_param);
        % Inflate post to account for clustering
        [a_Reach_nd_s_a, b_Reach_nd_s_a] = compute_Reach_nd_simplified_q_u(a_Post_w0, b_Post_w0, original_sys.Lw_s_a{s,a}, user_param.c);
%         if ((s == 9)&&(a == 3))
%             5
%         end
        % Check intersection with all Qp
        [Qp_intersects, error_flag_intersections_nd_coarse] = intersections_simplified_fcn(a_Reach_nd_s_a, b_Reach_nd_s_a, Deltax_coarse{1},  Deltax_coarse{2}, Nx_coarse{1}, Nx_coarse{2}, Nq, a_X, b_X, solver_opts.precision);
        % Remove clusters qp that aren't fully contained in Reach_nd_s_a
        [Qp{s,a}, Qp_flag_removed{s,a}] = remove_qp_with_illegitimate_sp_single_goal_and_obs_fcn( Qp_intersects, vertices_q, a_Reach_nd_s_a, b_Reach_nd_s_a, Nq , solver_opts, children, S_target, S_obstacle);
        % Compute TP bounds
        aux_P_upper_coarse = zeros(Ns + 1,1);
        aux_P_lower_coarse = zeros(Ns + 1,1);
        for c_ind = 1:user_param.Nc
            w = samples.Xi_hat_c(:,c_ind);
            % Compute Post_s_a_w
            [a_Post, b_Post, error_flag_post] = compute_reach_simplified_fcn(state_discretization.vertices_s{s},u,w,original_sys,user_param);
            % Inflate post to account for clustering
            a_Post = a_Post - original_sys.Lw_s_a{s,a}*samples.size_cluster(c_ind);
            b_Post = b_Post + original_sys.Lw_s_a{s,a}*samples.size_cluster(c_ind);
            % Check intersection with all qp
            [Qp_intersects, error_flag_intersections_coarse] = intersections_simplified_fcn(a_Post, b_Post, Deltax_coarse{1}, Deltax_coarse{2}, Nx_coarse{1}, Nx_coarse{2}, Nq, a_X, b_X, solver_opts.precision);
            % Check if post is subsumed by some qps
            [Q_ind_subsumed, error_flag_subsumed_coarse] = subsumed_q_a_w_simplified(a_Post, b_Post, state_discretization.vertices_q, Qp_intersects, original_sys.state_space_lower_left_corner, original_sys.state_space_upper_right_corner, Nq, solver_opts.precision);
            for qp_ind = 1:length(Qp_intersects)
                qp = Qp_intersects(qp_ind);
                if (any( ismember(Qp{s,a}, qp) ))
                    aux_P_upper_coarse(qp) = aux_P_upper_coarse(qp) + samples.weight_norm(c_ind);
                    if (Q_ind_subsumed(qp_ind))
                        aux_P_lower_coarse(qp) = aux_P_lower_coarse(qp) + samples.weight_norm(c_ind);
                    end
                end
            end
            if ( (error_flag_post == true) || (error_flag_intersections_coarse == true) || (error_flag_subsumed_coarse == true) )
                error_flag_computing_abstraction = true;
                disp('Problem found...')
            end
        end
        % Account for Nc + 1
        c_ind = user_param.Nc + 1;
        aux_P_upper_coarse(end) = aux_P_upper_coarse(end) + 1;
        Qp{s,a} = unique([Qp{s,a}; Nq + 1]);
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
        [a_Post_w0, b_Post_w0, error_flag_post] = compute_reach_simplified_fcn(vertices_s{s},u,w,original_sys,user_param);
        % Inflate post to account for clustering
        [a_Reach_nd_s_a, b_Reach_nd_s_a] = compute_Reach_nd_simplified_q_u(a_Post_w0, b_Post_w0, original_sys.Lw_s_a{s,a}, user_param.c);
        % Check intersection with all Qp
        [Sp_intersects, error_flag_intersections_nd_coarse] = intersections_simplified_single_goal_and_obs_fcn(a_Reach_nd_s_a, b_Reach_nd_s_a, Deltax{1},  Deltax{2}, Nx{1}, Nx{2}, Ns, a_X, b_X, solver_opts.precision, user_param);
        Sp{s,a} = Sp_intersects;
%         Sp{s,a} = 1:Ns+1;
        % Assign smaller probability yo every state in Sp
        aux_P_upper = zeros(Ns + 1,1);
        aux_P_lower = zeros(Ns + 1,1);
        for c_ind = 1:user_param.Nc
            w = samples.Xi_hat_c(:,c_ind);
            % Compute Post_q_a_w
            [a_Post, b_Post, error_flag_post] = compute_reach_simplified_fcn(state_discretization.vertices_s{s},u,w,original_sys,user_param);
            % Inflate post to account for clustering
            a_Post = a_Post - original_sys.Lw_s_a{s,a}*samples.size_cluster(c_ind);
            b_Post = b_Post + original_sys.Lw_s_a{s,a}*samples.size_cluster(c_ind);
            % Check intersection with all qp
            [Sp_intersects, error_flag_intersections] = intersections_simplified_single_goal_and_obs_fcn(a_Post, b_Post, Deltax{1}, Deltax{2}, Nx{1}, Nx{2}, Ns, a_X, b_X, solver_opts.precision, user_param);
            % Check if post is subsumed by some qps
            [Sp_ind_subsumed, error_flag_subsumed] = subsumed_q_a_w_simplified(a_Post, b_Post, state_discretization.vertices_s, Sp_intersects, a_X, b_X, Ns, solver_opts.precision);
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
        c_ind = user_param.Nc + 1;
        aux_P_upper(end) = aux_P_upper(end)  + samples.weight_norm(c_ind);
        Sp{s,a} = unique([Sp{s,a}; Ns + 1]);
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

if ( original_sys.data_driven_yn )
    % Add epsilons
    n_bounds_per_sp = 2;
    if ( strcmp( solver_opts.abstraction_choice, '2layer') || strcmp( solver_opts.abstraction_choice, '2layer_single_goal') )
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
