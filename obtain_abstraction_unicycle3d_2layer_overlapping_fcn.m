function [P_lower, P_upper, Sp, P_lower_coarse, P_upper_coarse, Qp, children, parents, n_transitions, beta, epsilon_support, error_flags] = obtain_abstraction_unicycle3d_2layer_overlapping_fcn(user_param, state_discretization, original_sys, solver_opts, samples, solution)

% Obtain UMDP abstraction

Act = original_sys.Act;
Ns = user_param.Ns;
Nq = user_param.Nq;
Na = user_param.Na;
N = user_param.N;
c = user_param.c;
Deltax = state_discretization.Deltax;
Deltay = state_discretization.Deltay;
Deltaz = state_discretization.Deltaz;
Nx = user_param.Nx;
Ny = user_param.Ny;
Nz = user_param.Nz;
Nx_coarse = user_param.Nx_coarse;
Ny_coarse = user_param.Ny_coarse;
Nz_coarse = user_param.Nz_coarse;
a_X = original_sys.state_space_lower_left_corner;
b_X = original_sys.state_space_upper_right_corner;
vertices_s = state_discretization.vertices_s;
vertices_q = state_discretization.vertices_q;
precision = solver_opts.precision;
coarse_Deltax_ratio = user_param.coarse_Deltax_ratio;
coarse_Deltay_ratio = user_param.coarse_Deltay_ratio;
coarse_Deltaz_ratio = user_param.coarse_Deltaz_ratio;
Xi_hat = samples.Xi_hat;
Xi_hat_c = samples.Xi_hat_c;
Lw_s_a = original_sys.Lw_s_a;
size_cluster = samples.size_cluster;
S_not_terminal = user_param.S_not_terminal;
alpha = user_param.alpha;
compute_clopper_pearson_table_yn = solver_opts.compute_clopper_pearson_table_yn;


disp('Obtaining UMDP abstraction: 2layer, overlapping...')

parents = cell(Ns+1, 1);
children = cell(Nq+1, 1);
P_lower = cell(Ns+1, user_param.Na);
P_lower_emp = cell(Ns+1, user_param.Na);
P_upper = cell(Ns+1, user_param.Na);
P_upper_emp = cell(Ns+1, user_param.Na);
Sp = cell(Ns+1, user_param.Na);
Sp_emp = cell(Ns+1, user_param.Na);
P_lower_coarse = cell(Ns+1, user_param.Na);
P_lower_coarse_emp = cell(Ns+1, user_param.Na);
P_upper_coarse = cell(Ns+1, user_param.Na);
P_upper_coarse_emp = cell(Ns+1, user_param.Na);
Qp = cell(Ns+1, user_param.Na);
Qp_emp = cell(Ns+1, user_param.Na);

bool_Qp_intersects = false(1, Nq + 1);
bool_Qp_subsumed = false(1, Nq + 1);
if (Ns + 1 < 65535)
    aux_Q_coarse = uint16( 1:(Nq + 1) );
else
    aux_Q_coarse = uint32( 1:(Nq + 1) );
end
error_flag_computing_abstraction = false;
tic

% % Compute parents
for q = 1:Nq
    a_q = vertices_q{q}(:, 1);
    b_q = vertices_q{q}(:, end);
    for s = 1:Ns
        s_center = [state_discretization.X_center(s); state_discretization.Y_center(s); state_discretization.Z_center(s)];
        if ( ~any( a_q > s_center ) && ~any( s_center > b_q ) )
            children{q} = [children{q}; s];
            parents{s} = [parents{s}; q];
        end
        % Wrapping dn
        if ( s_center(3) < a_X(3) + Deltaz*(coarse_Deltaz_ratio - 0.5) )
            s_center = s_center + Nz*Deltaz*[0 0 1]';
            if ( ~any( a_q > s_center ) && ~any( s_center > b_q ) )
                children{q} = [children{q}; s];
                parents{s} = [parents{s}; q];
            end
        end
        if ( length( children{q} ) == coarse_Deltax_ratio*coarse_Deltay_ratio*coarse_Deltaz_ratio )
            break;
        end
    end
end
q = Nq + 1;
s = Ns + 1;
children{q} = s;
parents{s} = q;

ii = 1;
step = 1;
for s = S_not_terminal
    for a = 1:Na

        %         if ( (q == 1) && (a == 6) )
        %             disp('asfasdas')
        %         end
        if ( ~user_param.controller_given_yn )
            u = Act(:,a);
        else
            u = Act(:, solution.controller_given(s) );
        end
        % Compute Sp
        w = original_sys.center_samples; % Center of W
        [a_Post_w0, b_Post_w0, error_flag_post] = compute_reach_simplified_unicycle3d_fcn(vertices_s{s},u,w,original_sys,user_param);
        [a_Reach_nd_q_a, b_Reach_nd_q_a] = compute_Reach_nd_simplified_q_u(a_Post_w0, b_Post_w0, Lw_s_a{s,a}, user_param.c);
        % Check intersection with all qp
        [Sp_intersects, error_flag_intersections_nd] = intersections_simplified_unicycle3d_fcn(a_Reach_nd_q_a, b_Reach_nd_q_a, Deltax,  Deltay, Deltaz, Nx, Ny, Nz, Ns, a_X, b_X, precision);
        Sp_emp{s,a} = Sp_intersects;
        [Qp_intersects, error_flag_intersections_nd_coarse] = compute_Qp_unicycle3d(a_Reach_nd_q_a, b_Reach_nd_q_a, Deltax, Deltay, Deltaz, Nx, Ny, Nz, Nq, Nx_coarse, Ny_coarse, Nz_coarse, precision, coarse_Deltax_ratio, coarse_Deltay_ratio, coarse_Deltaz_ratio, a_X, b_X);
        Qp_emp{s,a} = Qp_intersects;
        % Compute TP bounds
        aux_P_upper = zeros(Ns + 1,1);
        aux_P_lower_coarse = zeros(Nq + 1,1);
        aux_P_upper_coarse = zeros(Nq + 1,1);
        for w_ind = 1:user_param.Nc
            w = Xi_hat_c(:,w_ind);
            % Compute Post_q_a_w
            [a_Reach, b_Reach, error_flag_post] = compute_reach_simplified_unicycle3d_fcn(vertices_s{s},u,w,original_sys,user_param);
            % Inflate post to account for clustering
            a_Reach = a_Reach - Lw_s_a{s,a}*size_cluster(w_ind);
            b_Reach = b_Reach + Lw_s_a{s,a}*size_cluster(w_ind);
            % Check intersection with all qp
            [Sp_intersects, error_flag_intersections] = intersections_simplified_unicycle3d_fcn(a_Reach, b_Reach, Deltax,  Deltay, Deltaz, Nx, Ny, Nz, Ns, a_X, b_X, precision);
            % Check if post is subsumed by some qps
            [Qp_intersects, error_flag_intersections_coarse] = intersections_coarse_simplified_fcn(Sp_intersects', parents, bool_Qp_intersects, aux_Q_coarse, a_X, b_X, Ns, precision);
            [Qp_subsumed, error_flag_subsumed] = subsumed_q_a_w_coarse_simplified_unicycle3d(a_Reach, b_Reach, vertices_q, Qp_intersects, bool_Qp_subsumed, aux_Q_coarse, a_X, b_X, Nq, precision);
            for sp = Sp_intersects'
                aux_P_upper(sp) = aux_P_upper(sp) + samples.weight_norm(w_ind);
            end
            for qp = Qp_subsumed
                aux_P_lower_coarse(qp) = aux_P_lower_coarse(qp) + samples.weight_norm(w_ind);
            end
            for qp = Qp_intersects
                aux_P_upper_coarse(qp) = aux_P_upper_coarse(qp) + samples.weight_norm(w_ind);
            end
%             if ( (s == 35) && (a == 4) )
%                 disp('HEREEEEE')
%             end
            if ( (error_flag_post == true) || (error_flag_intersections == true) || (error_flag_subsumed == true) )
                error_flag_computing_abstraction = true;
                disp('Problem found...')
            end
        end
        P_lower_coarse_emp{s,a} = aux_P_lower_coarse(Qp_emp{s,a});
        P_upper_coarse_emp{s,a} = aux_P_upper_coarse(Qp_emp{s,a});
        P_upper_emp{s,a} = aux_P_upper(Sp_emp{s,a});
        P_lower_emp{s,a} = zeros(size(Sp_emp{s,a}));

        % DON'T ACCOUNT FOR eps_support heree

        if ( (error_flag_intersections_nd == true) || ( length(Qp_emp{s,a}) ~= length(P_lower_coarse_emp{s,a}) ) || ( length(Qp_emp{s,a}) ~= length(P_upper_coarse_emp{s,a}) ) || ( length(Sp_emp{s,a}) ~= length(P_upper_emp{s,a}) ) || ( length(Sp_emp{s,a}) ~= length(P_lower_emp{s,a}) ) )
            error_flag_computing_abstraction = true;
            disp('Problem found...')
        end
    end
    % Display progress
    [ii, step] = disp_progress_fcn(length( S_not_terminal ), ii, 10, step);
end

for s = 1:Ns+1
    for a = 1:Na
        P_lower_coarse{s,a} = zeros( size( P_lower_coarse_emp{s,a} ) );
        P_upper_coarse{s,a} = zeros( size( P_upper_coarse_emp{s,a} ) );
        P_lower{s,a} = zeros( size( P_lower_emp{s,a} ) );
        P_upper{s,a} = zeros( size( P_upper_emp{s,a} ) );
    end
end

% Make Ns+1 absorbing
s = Ns + 1;
for a = 1:Na
    P_lower_coarse_emp{s,a} = 1;
    P_upper_coarse_emp{s,a} = 1;
    P_lower_emp{s,a} = 1;
    P_upper_emp{s,a} = 1;
    Sp_emp{s,a} = Ns + 1;
    Sp{s,a} = Ns + 1;
    Qp{s,a} = Nq + 1;
    Qp_emp{s,a} = Nq + 1;
    P_lower_coarse{s,a} = 1;
    P_upper_coarse{s,a} = 1;
    P_lower{s,a} = 1;
    P_upper{s,a} = 1;
end

abstraction_time = toc/60;
disp('Abstraction complete')
disp(abstraction_time)


% Relax bounds by epsilon (Clopper - Pearson)
% Compute epsilon

n_bounds_per_sp = 1;
n_bounds_per_qp = 2;

n_transitions = 0;
for s = S_not_terminal
    for a = 1:Na
        n_transitions = n_transitions + n_bounds_per_sp*length( Sp_emp{s,a} );
    end
end

for s = S_not_terminal
    for a = 1:user_param.Na
        n_transitions = n_transitions + n_bounds_per_qp*length( Qp_emp{s,a} );
    end
end

beta = 0.5*alpha/n_transitions;
M = generate_clopper_pearson_table_fcn(user_param.N, beta, true, compute_clopper_pearson_table_yn, false);
for s = S_not_terminal
    for a = 1:Na
        n_successes_lower_coarse = round( N*P_lower_coarse_emp{s,a} );
        n_successes_upper_coarse = round( N*P_upper_coarse_emp{s,a} );
        n_successes_lower = round( N*P_lower_emp{s,a} );
        n_successes_upper = round( N*P_upper_emp{s,a} );
        for qp_coarse = 1:length(n_successes_lower_coarse)
            count_lower_coarse = n_successes_lower_coarse(qp_coarse) + 1;
            P_lower_coarse{s,a}(qp_coarse) = M(count_lower_coarse, 2);
        end
        for qp_coarse = 1:length(n_successes_upper_coarse)
            count_upper_coarse = n_successes_upper_coarse(qp_coarse) + 1;
            P_upper_coarse{s,a}(qp_coarse) = M(count_upper_coarse, 3);
        end
        for qp = 1:length(n_successes_lower)
            count_lower = n_successes_lower(qp) + 1;
            P_lower{s,a}(qp) = M(count_lower, 2);
        end
        for qp = 1:length(n_successes_upper)
            count_upper = n_successes_upper(qp) + 1;
            P_upper{s,a}(qp) = M(count_upper, 3);
        end
    end
end
        
% % JUST CHECKING. COMMENT THIS!!!!!!!!!!!1 AND MORE BELOWWWWW
% for s = S_not_terminal
%     for a = 1:Na
%         P_lower{s,a} = P_lower_emp{s,a};
%         P_upper{s,a} = P_upper_emp{s,a};
%         P_lower_coarse{s,a} = P_lower_coarse_emp{s,a};
%         P_upper_coarse{s,a} = P_upper_coarse_emp{s,a};
%     end
% end

% Account for epsilon_support (assuming beta_support = alpha/2)
beta_support = alpha/2;
count = 0;
for wi = 1:N
    if (norm(Xi_hat(:, wi) - original_sys.center_samples, "inf") < c)
        count = count+1;
    end
end
M_support = generate_clopper_pearson_table_fcn(N, beta_support, true, compute_clopper_pearson_table_yn, true);
epsilon_support = 1 - M_support(count+1, 2);
fprintf('epsilon_support = %f. \n', epsilon_support)

% % JUST CHECKING!!!!!! COMMENT THISSSSSS!!!!! AND MORE ABOVEEEEEEE
% epsilon_support = 0;

for s = S_not_terminal
    for a = 1:Na
        if (Sp_emp{s,a}(end) == Ns+1)
            Sp{s,a} = Sp_emp{s,a};
            P_upper{s,a}(end) = min(1, P_upper{s,a}(end) + epsilon_support);
        else
            Sp{s,a} = [Sp_emp{s,a}; Ns+1];
            P_lower{s,a} = [P_lower{s,a}; 0];
            P_upper{s,a} = [P_upper{s,a}; epsilon_support];
        end

    end
end

for s = S_not_terminal
    for a = 1:Na
        if (Qp_emp{s,a}(end) == Nq + 1)
            Qp{s,a} = Qp_emp{s,a};
            P_upper_coarse{s,a}(end) = min(1, P_upper_coarse{s,a}(end) + epsilon_support);
        else
            Qp{s,a} = [Qp_emp{s,a}; Nq+1];
            P_lower_coarse{s,a} = [P_lower_coarse{s,a}; 0];
            P_upper_coarse{s,a} = [P_upper_coarse{s,a}; epsilon_support];
        end
    end
end

error_flags.error_flag_intersections = error_flag_intersections;
error_flags.error_flag_intersections_nd_coarse = error_flag_intersections_coarse;
error_flags.error_flag_intersections_nd = error_flag_intersections_nd;
error_flags.error_flag_intersections_nd_coarse = error_flag_intersections_nd_coarse;
error_flags.error_flag_post = error_flag_post;
error_flags.error_flag_subsumed = error_flag_subsumed;
error_flags.error_flag_computing_abstraction = error_flag_computing_abstraction;

end