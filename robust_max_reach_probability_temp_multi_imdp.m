function [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_temp_multi_imdp(user_param, initial_condition, solution, solver_opts)
% Solve the maximal reachability probability problem
% Currently it only uses lower bound to compute the optimal strategy

max_iter = user_param.max_iter;
Ns = user_param.Ns;
Nc = user_param.Nc;
Na = user_param.Na;
tol_VI = solver_opts.tol_VI;
n_states_per_group_RDP = solver_opts.n_states_per_group_RDP;
S_not_terminal = user_param.S_not_terminal;

% Initialize interval value
V_lower = initial_condition.V_lower;
V_upper = initial_condition.V_upper;
sigma = initial_condition.sigma;
V1_lower = zeros(user_param.Ns+1,1);
V1_upper = zeros(user_param.Ns+1,1);
V1_lower_s_a = zeros(Na,1);

% If state s \in Q_target
for s = user_param.S_terminal
    V_lower(s) = 0;
    V1_lower(s) = 0;
    V_upper(s) = 0;
    V1_upper(s) = 0;
end


% Robust dynamic programming

gamma_prev = cell(Ns, user_param.Na);
model = cell(Ns, user_param.Na);
n_groups = ceil( length( S_not_terminal )/n_states_per_group_RDP );
model_cent = cell(length(user_param.S_not_terminal)*Na, 1);
params.outputflag = 0;
params.Presolve = 1;
% Iterate the lower bound
tic
groups = cell(n_groups, 1);
si = 1;
for groupi = 1:n_groups
    groups{groupi} = [];
    for counter_s_in_group = 1:n_states_per_group_RDP
        if ( si > length( S_not_terminal ) )
            break;
        end
        groups{groupi} = [groups{groupi}, S_not_terminal(si)];
        si = si + 1;
    end
end

for i = 1:max_iter
    for groupi = 1:n_groups
        % Non-terminal states inside group
        S_group_not_terminal = groups{groupi};
        if (i == 1) % Construct matrices
            A_block = [];
            b_block = [];
            obj_block = [];
            ind_interval = zeros(n_states_per_group_RDP*Na, 1);
            counter_s_a_in_group = 1;
            for s = S_group_not_terminal
                for a = 1:Na
                    %             s
                    %             a
                    %             if ((s == 8) && (a == 8))
                    %                 100
                    %             end
                    V_lower_proj_Sp = V_lower(solution.Sp{s,a});
                    [A_aux, b_aux, ~, ~] = build_matrices_for_RDP_multi_imdp_fcn(solution.P_lower_coarse{s,a}, solution.P_upper_coarse{s,a}, solution.Sp{s,a}, solution.Qp{s,a}.children_q);
%                     ind = find(abs(b_aux) > 0);
%                     A_aux = [sparse( A_aux(ind, :) ); sparse( eye(length(solution.P_upper{s,a})) )];
%                     b_aux = [b_aux(ind); solution.P_upper{s,a}];
                    A_aux = [sparse( A_aux ); sparse( -eye(length(solution.P_lower{s,a})) ); sparse( eye(length(solution.P_upper{s,a})) )];
                    b_aux = [b_aux; -solution.P_lower{s,a}; solution.P_upper{s,a}];
                    ind_interval(counter_s_a_in_group, 1) = length(obj_block) + 1;
                    A_block = blkdiag(A_block, A_aux );
                    b_block = [b_block; b_aux];
                    obj_block = [obj_block; V_lower_proj_Sp];
                    ind_interval(counter_s_a_in_group, 2) = length(obj_block);
                    counter_s_a_in_group = counter_s_a_in_group + 1;
                end
            end
            model_cent{groupi}.A = A_block;
            model_cent{groupi}.rhs = b_block;
            model_cent{groupi}.obj = obj_block;
        else % Modify only the objective
            obj_block = [];
            ind_interval = zeros(n_states_per_group_RDP*Na, 1);
            counter_s_a_in_group = 1;
            for s = S_group_not_terminal
                for a = 1:Na
                    %             s
                    %             a
                    %             if ((s == 8) && (a == 8))
                    %                 100
                    %             end
                    V_lower_proj_Sp = V_lower(solution.Sp{s,a});
                    ind_interval(counter_s_a_in_group, 1) = length(obj_block) + 1;
                    obj_block = [obj_block; V_lower_proj_Sp];
                    ind_interval(counter_s_a_in_group, 2) = length(obj_block);
                    counter_s_a_in_group = counter_s_a_in_group + 1;
                end
            end
            model_cent{groupi}.obj = obj_block;
        end
        % Clear block matrices
        A_block = [];
        b_block = [];
        obj_block = [];
        % Solve with gurobi
        result = gurobi(model_cent{groupi}, params);
        model_cent{groupi}.vbasis = result.vbasis;
        model_cent{groupi}.cbasis = result.cbasis;
        gamma_group = result.x';
        V_lower_group = model_cent{groupi}.obj;
        % Extract value for each (s , a) from result
        counter_s_a_in_group = 1;
        ind_interval = int64(ind_interval); % To avoid conversion errors
        for s = S_group_not_terminal
            V1_lower_s_a = zeros(Na, 1);
            for a = 1:Na
                if ( ~isinteger( ind_interval(counter_s_a_in_group, 1):ind_interval(counter_s_a_in_group, 2) ) )
                    5
                end
                gamma_s_a = gamma_group(ind_interval(counter_s_a_in_group, 1):ind_interval(counter_s_a_in_group, 2));
                V_lower_s_a = V_lower_group(ind_interval(counter_s_a_in_group, 1):ind_interval(counter_s_a_in_group, 2));
                V1_lower_s_a(a) = gamma_s_a*V_lower_s_a;
                counter_s_a_in_group = counter_s_a_in_group + 1;
            end
            % Maximize in a \in Act
            [V1_lower_max, sigma(i, s)] = max(V1_lower_s_a);
            V1_lower(s) = min(1, V1_lower_max);
        end
    end

    err = max(abs(V1_lower - V_lower));
    V_lower = V1_lower;

    toc
    fprintf('Iteration = %d . \n', i)
    fprintf('Diference between consecutive value functions = %f . \n', err)

    if ( (i == max_iter) || (err < tol_VI) )
        P_reach_lower = V_lower;
        break;
    end
    % Just for debugging purposes
    %if (i == 1)
    %    estimated_time = user_param.max_iter*toc/60;
    %    fprintf('Estimated value iteration time = %d min. \n', estimated_time)
    %end

end

% Or don't iterate
% P_reach_lower = V_lower;

% Iterate the upper bound
for i = 1:max_iter
    for groupi = 1:n_groups
        % Non-terminal states inside group
        S_group_not_terminal = groups{groupi};
        % Construct matrices at every iteration bc sigma changes
        model_cent{groupi} = [];
        A_block = [];
        b_block = [];
        obj_block = [];
        ind_interval = zeros(n_states_per_group_RDP, 1);
        counter_s_a_in_group = 1;
        for s = S_group_not_terminal
            a = sigma(i, s);
            %             s
            %             a
            %             if ((s == 8) && (a == 8))
            %                 100
            %             end
            V_upper_proj_Sp = V_upper(solution.Sp{s,a});
            [A_aux, b_aux, ~, ~] = build_matrices_for_RDP_multi_imdp_fcn(solution.P_lower_coarse{s,a}, solution.P_upper_coarse{s,a}, solution.Sp{s,a}, solution.Qp{s,a}.children_q);
            %                     ind = find(abs(b_aux) > 0);
            %                     A_aux = [sparse( A_aux(ind, :) ); sparse( eye(length(solution.P_upper{s,a})) )];
            %                     b_aux = [b_aux(ind); solution.P_upper{s,a}];
            % Flip inequality corresponding sum = 1
            A_aux(end, :) = -A_aux(end, :);
            b_aux(end) = -b_aux(end, :);
            A_aux = [sparse( A_aux ); sparse( -eye(length(solution.P_lower{s,a})) ); sparse( eye(length(solution.P_upper{s,a})) )];
            b_aux = [b_aux; -solution.P_lower{s,a}; solution.P_upper{s,a}];
            ind_interval(counter_s_a_in_group, 1) = length(obj_block) + 1;
            A_block = blkdiag(A_block, A_aux );
            b_block = [b_block; b_aux];
            obj_block = [obj_block; V_upper_proj_Sp];
            ind_interval(counter_s_a_in_group, 2) = length(obj_block);
            counter_s_a_in_group = counter_s_a_in_group + 1;
        end
        model_cent{groupi}.A = A_block;
        model_cent{groupi}.rhs = b_block;
        model_cent{groupi}.obj = -obj_block;
        % Clear block matrices
        A_block = [];
        b_block = [];
        obj_block = [];
        % Solve with gurobi
        result = gurobi(model_cent{groupi}, params);
        model_cent{groupi}.vbasis = result.vbasis;
        model_cent{groupi}.cbasis = result.cbasis;
        gamma_group = result.x';
        V_upper_group = model_cent{groupi}.obj;
        % Extract value for each (s , a) from result
        counter_s_a_in_group = 1;
        ind_interval = int64(ind_interval); % To avoid conversion errors
        for s = S_group_not_terminal
            gamma_s = gamma_group(ind_interval(counter_s_a_in_group, 1):ind_interval(counter_s_a_in_group, 2));
            V_upper_s = -V_upper_group(ind_interval(counter_s_a_in_group, 1):ind_interval(counter_s_a_in_group, 2));
            V1_upper(s) = gamma_s*V_upper_s;
            counter_s_a_in_group = counter_s_a_in_group + 1;
        end
    end

    V1_upper = min( V1_upper, 1 );
    err = max(abs(V1_upper - V_upper));
    V_upper = V1_upper;

    toc
    fprintf('Iteration = %d . \n', i)
    fprintf('Diference between consecutive value functions = %f . \n', err)

    if ( (i == max_iter) || (err < tol_VI) )
        P_reach_upper = V_upper;
        break;
    end
    % Just for debugging purposes
    %if (i == 1)
    %    estimated_time = user_param.max_iter*toc/60;
    %    fprintf('Estimated value iteration time = %d min. \n', estimated_time)
    %end

end
% Or don't iterate
% P_reach_upper = V_upper;

end

