function [solution, error_flags] = obtain_abstraction_multi_imdp_uni3d_fcn(user_param, state_discretization, original_sys, solver_opts, samples, solution)

% Obtain UMDP abstraction

Act = original_sys.Act;
Ns = user_param.Ns;
Na = user_param.Na;
N = user_param.N;
Nc = user_param.Nc;
c = user_param.c;
Deltax = state_discretization.Deltax{1};
Deltay = state_discretization.Deltax{2};
Deltaz = state_discretization.Deltax{3};
Nx = user_param.Nx{1};
Ny = user_param.Nx{2};
Nz = user_param.Nx{3};
a_X = original_sys.state_space_lower_left_corner';
b_X = original_sys.state_space_upper_right_corner';
vertices_s = state_discretization.vertices_s;
precision = solver_opts.precision;
Xi_hat = samples.Xi_hat;
Xi_hat_c = samples.Xi_hat_c;
Lw_s_a = original_sys.Lw_s_a;
S_not_terminal = user_param.S_not_terminal;
alpha = user_param.alpha;
compute_clopper_pearson_table_yn = solver_opts.compute_clopper_pearson_table_yn;

disp('Obtaining UMDP abstraction: 2layer, overlapping...')

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

% bool_Qp_intersects = false(1, Nq + 1);
% bool_Qp_subsumed = false(1, Nq + 1);
% if (Ns + 1 < 65535)
%     aux_S = uint16( 1:(Ns + 1) );
%     aux_Q_coarse = uint16( 1:(Nq + 1) );
% else
%     aux_S = uint32( 1:(Ns + 1) );
%     aux_Q_coarse = uint32( 1:(Nq + 1) );
% end
error_flag_computing_abstraction = false;
tic

% First compute Qp

disp('Computing clusters...')

ii = 1;
step = 1;
for s = S_not_terminal
    for a = 1:Na
        if ( ~user_param.controller_given_yn )
            u = Act(:,a);
        else
            u = Act(:, solution.controller_given(s) );
        end
        % Compute TP bounds
        Qp{s,a}.children_q = cell(1, user_param.Nc + 1);
        aux_Sp = [];
        for c_ind = 1:user_param.Nc
            w = Xi_hat_c(:, c_ind);
            % Compute Post_q_a_w
            [a_Reach, b_Reach, error_flag_post] = compute_reach_simplified_unicycle3d_fcn(vertices_s{s},u,w,original_sys,user_param);
            % Inflate post to account for clustering
            a_Reach = a_Reach - Lw_s_a{s,a}*samples.size_cluster(c_ind);
            b_Reach = b_Reach + Lw_s_a{s,a}*samples.size_cluster(c_ind);
            % Obtain the (smallest) cluster q that contains Reach: Check intersection with all sp
            [Sp_intersects, error_flag_intersections] = intersections_simplified_unicycle3d_fcn(a_Reach, b_Reach, Deltax, Deltay, Deltaz, Nx, Ny, Nz, Ns, a_X, b_X, precision);
            if ( ( s == 4) && (a == 10) && ~ismember(Ns+1, Sp_intersects) )
                5
            end
            Qp{s,a}.children_q{c_ind} = Sp_intersects;
            aux_Sp = [aux_Sp; Sp_intersects];
            if ( (error_flag_post == true) || (error_flag_intersections == true) )
                error_flag_computing_abstraction = true;
                disp('Problem found...')
            end
        end
        % Account for the last c (corresponding to the rest of the space of the dist.)
        c_ind = user_param.Nc + 1;
        Sp_intersects = Ns + 1;
        Qp{s,a}.children_q{c_ind} = Sp_intersects;
        aux_Sp = [aux_Sp; Sp_intersects];
        Qp{s,a}.Nq = Nc + 1;
        Sp_emp{s,a} = unique( aux_Sp );
    end
    % Display progress
    [ii, step] = disp_progress_fcn(length( S_not_terminal ), ii, 20, step);
end

% Next compute parent-children relations

% disp('Computing parents-children...')

% for q = 1:Nq
%     a_q = vertices_q{q}(:, 1) + 2*precision;
%     b_q = vertices_q{q}(:, end) - 2*precision;
%     for s = 1:Ns
%         s_center = [state_discretization.X_center(s); state_discretization.Y_center(s)];
%         if ( ~any( a_q > s_center ) && ~any( s_center > b_q ) )
%             children{q} = [children{q}; s];
%             parents{s} = [parents{s}; q];
%         end
%     end
% end
% q = Nq + 1;
% s = Ns + 1;
% children{q} = s;
% parents{s} = q;

% Obtain abstraction

disp('Obtaining multi-IMDP abstraction...')

ii = 1;
step = 1;
for s = S_not_terminal
    for a = 1:Na
        if ( ~user_param.controller_given_yn )
            u = Act(:,a);
        else
            u = Act(:, solution.controller_given(s) );
        end
        aux_P_lower = zeros(Ns + 1,1);
        aux_P_upper = zeros(Ns + 1,1);
        aux_P_lower_coarse = zeros(Qp{s,a}.Nq, 1);
        aux_P_upper_coarse = zeros(Qp{s,a}.Nq, 1);
        for c_ind = 1:Nc
            w = Xi_hat_c(:,c_ind);
            % Compute Post_q_a_w
            [a_Reach, b_Reach, error_flag_post] = compute_reach_simplified_unicycle3d_fcn(vertices_s{s},u,w,original_sys,user_param);
            % Inflate post to account for clustering
            a_Reach = a_Reach - Lw_s_a{s,a}*samples.size_cluster(c_ind);
            b_Reach = b_Reach + Lw_s_a{s,a}*samples.size_cluster(c_ind);
            [Sp_intersects_small, error_flag_intersections] = intersections_simplified_unicycle3d_fcn(a_Reach, b_Reach, Deltax, Deltay, Deltaz, Nx, Ny, Nz, Ns, a_X, b_X, precision);
            % Check if post is subsumed by some qps
            [Sp_ind_subsumed, error_flag_subsumed] = subsumed_q_a_w_simplified_n_dim_fcn(Sp_emp{s,a}, Sp_intersects);
            for sp_ind = 1:length(Sp_intersects)
                sp = Sp_intersects(sp_ind);
                aux_P_upper(sp) = aux_P_upper(sp) + samples.weight_norm(c_ind);
            end
            for sp_ind = 1:length(Sp_intersects)
                if (Sp_ind_subsumed(sp_ind))
                    sp = Sp_intersects(sp_ind);
                    aux_P_lower(sp) = aux_P_lower(sp) + samples.weight_norm(c_ind);
                end
            end
            % Check if post is subsumed by some qps
            [Qp_intersects_bool, Qp_subsumed_bool, error_flag_intersections_nd_coarse] = compute_Qp_multi_imdp_fcn(Sp_intersects, Sp_intersects, Qp{s,a}.children_q, Qp{s,a}.Nq);
%             ind1 = (Sp_intersects_small == 1);
%             if ( any(ind1) &&  Sp_ind_subsumed(ind1) && ~Qp_subsumed_bool(89) )
%                 5
%             end
            aux_P_lower_coarse(Qp_subsumed_bool) = aux_P_lower_coarse(Qp_subsumed_bool) + samples.weight_norm(c_ind);
            aux_P_upper_coarse(Qp_intersects_bool) = aux_P_upper_coarse(Qp_intersects_bool) + samples.weight_norm(c_ind);
            if ( (error_flag_post == true) || (error_flag_intersections == true) || error_flag_subsumed || error_flag_intersections_nd_coarse )
                error_flag_computing_abstraction = true;
                disp('Problem found...')
            end
        end
        % Account for the last c (corresponding to the rest of the space of the dist.)
        c_ind = Nc + 1;
        aux_P_upper(end) = aux_P_upper(end) + samples.weight_norm(c_ind);
        %aux_P_lower_coarse(end) = aux_P_lower_coarse(end) + samples.weight_norm(c_ind);
        aux_P_upper_coarse = aux_P_upper_coarse + samples.weight_norm(c_ind);
        
        P_lower_coarse_emp{s,a} = aux_P_lower_coarse;
        P_upper_coarse_emp{s,a} = aux_P_upper_coarse;
        if ( any(P_lower_coarse_emp{s,a} > P_upper_coarse_emp{s,a} + precision ) )
            5
        end
        P_lower_emp{s,a} = aux_P_lower( Sp_emp{s,a} );
        P_upper_emp{s,a} = aux_P_upper( Sp_emp{s,a} );

        % DON'T ACCOUNT FOR eps_support heree

        if ( error_flag_computing_abstraction || ( length(P_lower_coarse_emp{s,a}) ~= Nc + 1 ) || ( length(P_upper_coarse_emp{s,a}) ~= Nc + 1 ) || ( length(Sp_emp{s,a}) ~= length(P_upper_emp{s,a}) ) || ( length(Sp_emp{s,a}) ~= length(P_lower_emp{s,a}) ) )
            error_flag_computing_abstraction = true;
            disp('Problem found...')
        end
    end
    % Display progress
    [ii, step] = disp_progress_fcn(length( S_not_terminal ), ii, 20, step);
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
    P_lower_coarse{s,a} = 1;
    P_upper_coarse{s,a} = 1;
    P_lower{s,a} = 1;
    P_upper{s,a} = 1;
end

if ( original_sys.data_driven_yn )
    % Relax bounds by epsilon (Clopper - Pearson)
    % Compute epsilon

    n_bounds_per_sp = 2;
    n_bounds_per_qp = 2;

    n_transitions_actual = 0;
    for s = S_not_terminal
        for a = 1:Na
            n_transitions_actual = n_transitions_actual + n_bounds_per_sp*length( Sp_emp{s,a} );
        end
    end

    for s = S_not_terminal
        for a = 1:user_param.Na
            n_transitions_actual = n_transitions_actual + n_bounds_per_qp*length( Qp_emp{s,a} );
        end
    end

    beta = 0.5*alpha/n_transitions_actual;
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

    % Account for epsilon_support (assuming beta_support = alpha/2)
    beta_support = alpha/2;
    count = 0;
    for wi = 1:N
        if (norm(Xi_hat(:, wi), "inf") < c)
            count = count+1;
        end
    end
    M_support = generate_clopper_pearson_table_fcn(user_param.N, beta_support, true, compute_clopper_pearson_table_yn, true);
    epsilon_support = 1 - M_support(count+1, 2);
    fprintf('epsilon_support = %f. \n', epsilon_support)

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
else
    for s = S_not_terminal
        for a = 1:Na
            Sp{s,a} = Sp_emp{s,a};
            P_lower{s,a} = P_lower_emp{s,a};
            P_upper{s,a} = P_upper_emp{s,a};
%             Qp{s,a} = Qp_emp{s,a};
            P_lower_coarse{s,a} = P_lower_coarse_emp{s,a};
            P_upper_coarse{s,a} = P_upper_coarse_emp{s,a};
        end
    end
    n_transitions_actual = [];
    beta = [];
    beta_support = [];
    epsilon = [];
    epsilon_support = [];
end

% Do this to save memory
clear P_lower_emp;
clear P_upper_emp;
clear P_lower_coarse_emp;
clear P_upper_coarse_emp;
clear Sp_emp;
clear Qp_emp;

abstraction_time = toc/60;
disp('Abstraction complete')
disp(abstraction_time)

solution.P_lower = P_lower;
solution.P_upper = P_upper;
solution.Sp = Sp;
solution.Qp = Qp;
solution.n_transitions_actual = n_transitions_actual;
solution.epsilon = epsilon;
solution.epsilon_support = epsilon_support;
solution.beta = beta;
solution.beta_support = beta_support;
solution.P_lower_coarse = P_lower_coarse;
solution.P_upper_coarse = P_upper_coarse;

error_flags.error_flag_intersections = error_flag_intersections;
error_flags.error_flag_intersections_nd_coarse = false;
error_flags.error_flag_intersections_nd = false;
error_flags.error_flag_intersections_nd_coarse = error_flag_intersections_nd_coarse;
error_flags.error_flag_post = error_flag_post;
error_flags.error_flag_subsumed = false;
error_flags.error_flag_computing_abstraction = error_flag_computing_abstraction;

end