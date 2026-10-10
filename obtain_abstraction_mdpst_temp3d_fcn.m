function [solution, error_flags] = obtain_abstraction_mdpst_temp3d_fcn(user_param, state_discretization, disturbance_discretization, original_sys, solver_opts, samples, solution)

% Obtain UMDP abstraction

Act = original_sys.Act;
Ns = user_param.Ns;
Na = user_param.Na;
N = user_param.N;
%c = user_param.c;
Deltax = state_discretization.Deltax;
Nx = user_param.Nx;
a_X = original_sys.state_space_lower_left_corner;
b_X = original_sys.state_space_upper_right_corner;
vertices_s = state_discretization.vertices_s;
vertices_c = disturbance_discretization.vertices_c;
precision = solver_opts.precision;
Xi_hat_c = samples.Xi_hat_c;
% Lw_s_a = original_sys.Lw_s_a;
S_not_terminal = user_param.S_not_terminal;
alpha = user_param.alpha;
Xd_center = disturbance_discretization.Xd_center;
disturbance_space_lower_left_corner = original_sys.disturbance_space_lower_left_corner;
disturbance_space_upper_right_corner = original_sys.disturbance_space_upper_right_corner;
compute_clopper_pearson_table_yn = solver_opts.compute_clopper_pearson_table_yn;

%epsilon_support = user_param.epsilon_support;


disp('Obtaining MDPIST2 abstraction...')

P_c_emp = cell(Ns+1, user_param.Na);
Sp = cell(Ns+1, user_param.Na);
Qp = cell(Ns+1, user_param.Na);
% Qp_emp = cell(Ns+1, user_param.Na);

error_flag_computing_abstraction = false;
tic

% % Compute parents
% parents{Ns+1} = Nq+1;
% for q = 1:Nq
%     a_q = vertices_q{q}(:, 1) + precision;
%     b_q = vertices_q{q}(:, end) - precision;
%     [Q_intersects, error_flag_intersections_nd] = intersections_simplified_fcn(a_q, b_q, Deltax,  Deltay, Nx, Ny, Ns, a_X, b_X, 0);
%     children{q} = Q_intersects;
%     for s = Q_intersects'
%         parents{s} = [parents{s}; q];
%     end
% end
% q = Nq + 1;
% s = Ns + 1;
% children{q} = s;
% parents{s} = q;

% First, estimate P_emp(c)

count_c = samples.weight;
delete('write_mdpst.txt')
%fileID = fopen('write_mdpst.txt','w','WriteMode','append');
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
        Qp{s,a}.children = cell(1, user_param.Nc + 1);
        aux_Sp = [];
        for c_ind = 1:user_param.Nc
            % Compute Reach_s_a_c
            [a_Reach, b_Reach, error_flag_post] = compute_reach_simplified_temp4d_fcn(vertices_s{s},u,vertices_c{c_ind},original_sys,user_param);
            % Obtain the (smallest) cluster q that contains Reach: Check intersection with all sp
            [Sp_intersects, error_flag_intersections] = intersections_simplified_temp4d_fcn(a_Reach, b_Reach, Deltax, Nx, Ns, a_X, b_X, precision);
            Qp{s,a}.children{c_ind} = Sp_intersects;
            %writematrix([s, a, c_ind, Sp_intersects'],'write_mdpst.txt','Delimiter',',', 'WriteMode', 'append');
            aux_Sp = [aux_Sp; Sp_intersects];
            if ( (error_flag_post == true) || (error_flag_intersections == true) )
                error_flag_computing_abstraction = true;
                disp('Problem found...')
            end
        end
        % Account for the last c (corresponding to the rest of the space of the dist.)
        c_ind = user_param.Nc + 1;
        Qp{s,a}.children{c_ind} = Ns + 1;
        %writematrix([s, a, c_ind, Ns+1],'write_mdpst.txt','Delimiter',',', 'WriteMode', 'append');
        aux_Sp = [aux_Sp; Ns + 1];
        Sp{s,a} = unique( aux_Sp );
        %P_c_emp{s,a} = count_c;
    end
    % Display progress
    [ii, step] = disp_progress_fcn(length( S_not_terminal ), ii, 20, step);
end

P_c = zeros( size( count_c ) );
fclose(fileID);

% Make Ns+1 absorbing
s = Ns + 1;
for a = 1:Na
    P_c_emp = 1;
    Qp{s,a}.children{1} = Ns + 1;
    P_c = 1;
end

abstraction_time = toc/60;
disp('Abstraction complete')
disp(abstraction_time)


% Relax bounds by epsilon (Clopper - Pearson)
% Compute epsilon

if ( original_sys.data_driven_yn )
    disp('P_w must be given...')
    return
else
    disp('P_w is given...')
    P_c = count_c/N;
    n_transitions = [];
    beta = [];
    epsilon_support = [];
end
% P_lower_c = count_c/N;
% P_upper_c = count_c/N;

% % Account for epsilon_support (assuming beta_support = alpha/2)
% beta_support = alpha/2;
% count = 0;
% for wi = 1:N
%     if (norm(Xi_hat(:, wi), "inf") < c)
%         count = count+1;
%     end
% end
% M_support = generate_clopper_pearson_table_fcn(N, beta_support, true);
% epsilon_support = 1 - M_support(count+1, 2);
% fprintf('epsilon_support = %f. \n', epsilon_support)
% 
% for s = S_not_terminal
%     for a = 1:Na
%         if (Sp_emp{s,a}(end) == Ns+1)
%             Sp{s,a} = Sp_emp{s,a};
%             P_upper{s,a}(end) = min(1, P_upper{s,a}(end) + epsilon_support);
%         else
%             Sp{s,a} = [Sp_emp{s,a}; Ns+1];
%             P_lower{s,a} = [P_lower{s,a}; 0];
%             P_upper{s,a} = [P_upper{s,a}; epsilon_support];
%         end
% 
%     end
% end
% 
% for s = S_not_terminal
%     for a = 1:Na
%         if (Qp_emp{s,a}(end) == Nq + 1)
%             Qp{s,a} = Qp_emp{s,a};
%             P_upper_c{s,a}(end) = min(1, P_upper_c{s,a}(end) + epsilon_support);
%         else
%             Qp{s,a} = [Qp_emp{s,a}; Nq+1];
%             P_lower_c{s,a} = [P_lower_c{s,a}; 0];
%             P_upper_c{s,a} = [P_upper_c{s,a}; epsilon_support];
%         end
%     end
% end

error_flags.error_flag_intersections = error_flag_intersections;
error_flags.error_flag_post = error_flag_post;
error_flags.error_flag_computing_abstraction = error_flag_computing_abstraction;

solution.P_c = P_c;
solution.Qp = Qp;
solution.n_transitions = n_transitions;
solution.beta = beta;
solution.epsilon_support = epsilon_support;


end