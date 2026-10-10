function [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_overlapping_tailored(user_param, initial_condition, solution, state_discretization, tol_VI)
% Solve the maximal reachability probability problem
% Currently it only uses lower bound to compute the optimal strategy

tol = 1e-2;
max_iter = user_param.max_iter;
Nq = user_param.Nq;
Na = user_param.Na;
Deltax = state_discretization.Deltax;
Deltay = state_discretization.Deltay;

% Initialize interval value
V_lower = initial_condition.V_lower;
V_upper = initial_condition.V_upper;
sigma = initial_condition.sigma;
V1_lower = zeros(user_param.Nq+1,1);
V1_upper = zeros(user_param.Nq+1,1);
V1_lower_q_a = zeros(Na,1);

% If state q \in Q_target
for s = user_param.Q_target
    V_lower(s) = 1;
    V1_lower(s) = 1;
    V_upper(s) = 1;
    V1_upper(s) = 1;
end


if (Deltax ~= Deltay)
    disp('This only works for hypercubes...')
    return;
end

% Interval value iteration

% Iterate the lower bound
tic
for i = 1:max_iter
    % Non-terminal states
    for si = 1:length(user_param.Q_not_terminal)
        s = user_param.Q_not_terminal(si);
        % Obtain (infty norm-based) distance by multiplying the number
        % of cells by the corresponding Delta
        for a = 1:Na
%             s
%             a
            if ((s == 75) && (a == 8))
                100
            end
            Mass = 1;
            % First assign P_lower
            V_lower_proj_Q_nd = V_lower(solution.Q_nd{s,a});
            gamma = solution.P_lower{s,a}';
            gamma_coarse = zeros(1, length(solution.Q_nd_coarse{s,a}) );
            if (any(gamma))
                for spi_unordered = 1:length(solution.Q_nd{s,a})
                    sp = solution.Q_nd{s,a}(spi_unordered);
                    parents = solution.parents_coarse{sp}';
                    for parent = parents
                        qpi = find(solution.Q_nd_coarse{s,a} == parent);
                        if ( ~isempty( qpi ) ) % parent \in Q_nd_coarse
                            gamma_coarse(qpi) = gamma_coarse(qpi) + gamma(spi_unordered);
                            Mass = Mass - gamma(spi_unordered);
                        end
                    end
                end
            end
            % Second, assign P_lower_coarse
            [~, order_V_lower] = sortrows(V_lower_proj_Q_nd, 'ascend'); % Vector that maps the spi in variables_for-matrices to the spi in the ordered set
            for spi = order_V_lower'
                sp = solution.Q_nd{s,a}(spi);
                parents = solution.parents_coarse{sp}';
                for qp = parents
                    qpi = find(solution.Q_nd_coarse{s,a} == qp);
                    if ( ~isempty( qpi ) ) % parent \in Q_nd_coarse
                        mass = min(Mass, solution.P_lower_coarse{s,a}(qpi) - gamma_coarse(qpi)); % m is the mass to share between all children of parent
                        children_qp = solution.children{qp}';
                        % Sort children_qp according to their V_lower
                        V_lower_proj_children_qp = V_lower(children_qp);
                        [~, order_V_lower_proj_children_qp] = sortrows(V_lower_proj_children_qp, 'ascend');
                        children_qp_ordered = children_qp( order_V_lower_proj_children_qp );
                        for child_qp = children_qp_ordered
                            child_qp_i = find(solution.Q_nd{s,a} == child_qp);
                            if ( ~isempty( child_qp_i ) ) % childreni_qp \in Q_nd
                                g = min(mass, solution.P_upper{s,a}(child_qp_i) - gamma(child_qp_i) );
                                gamma(child_qp_i) = gamma(child_qp_i) + g;
                                gamma_coarse(qpi) = gamma_coarse(qpi) + g;
                                mass = mass - g;
                                Mass = Mass - g;
                                if (mass == 0)
                                    break;
                                end
                            end
                        end
                    end
                end
            end
            if ( (mass ~= 0) || ( any( gamma < solution.P_lower{s,a}' ) ) || ( any( gamma > solution.P_upper{s,a}' ) ) || ( any( gamma_coarse < solution.P_lower_coarse{s,a}' ) ) || ( any( gamma_coarse > solution.P_upper_coarse{s,a}' ) ) )
                disp('ERROR: Not all mass has been assigned or gamma/gamma_coarse exceed bounds...')
            end
%             if ( ( s == 479 ) && ( a == 4 ) )% && ( any( gamma < 0 ) ) )
%                 disp('asdasda')
%             end
            % Third, assign P_upper
            for spi = order_V_lower'
                sp = solution.Q_nd{s,a}(spi);
                parents_sp = solution.parents_coarse{sp}';
                aux_diff_P_upper_coarse = ones( size( parents_sp) );
                count = 1;
                for qp = parents_sp
                    qpi = find(solution.Q_nd_coarse{s,a} == qp);
                    if ( ~isempty(qpi) ) % The constraint corresponding to qp is present. Otherwise, do not consider it
                        aux_diff_P_upper_coarse(count) = solution.P_upper_coarse{s,a}(qpi) - gamma_coarse(qpi);
                    end
                    count = count + 1;
                end
                mass = min([Mass, aux_diff_P_upper_coarse, solution.P_upper{s,a}(spi) - gamma(spi)] );
                if ( mass < -tol )
                    disp('asdasd')
                end
                gamma(spi) = gamma(spi) + mass;
                for qp = parents_sp
                    qpi = find(solution.Q_nd_coarse{s,a} == qp);
                    if ( ~isempty(qpi) ) % The constraint corresponding to qp is present. Otherwise, do not consider it
                        gamma_coarse(qpi) = gamma_coarse(qpi) + mass;
                    end
                end
                Mass = Mass - mass;
                %end
                if (Mass == 0)
                    break;
                end
                if ( any( gamma < -tol ) || ( any( gamma_coarse < -tol ) ) )
                    disp('Error: not all mass assigned');
                end
            end
            if (Mass > 0)
                disp('Error: not all mass assigned');
            end
            V1_lower_q_a(a) = gamma*V_lower_proj_Q_nd;
%             if ( V1_lower_q_a(a) > 1 )
%                 disp('WEIRDDDDD...')
%             end
            if ( ( abs( sum( gamma ) - 1 ) > tol ) || any( gamma < -tol ) )
                disp('ERROR: gamma is weirdddd.....')
            end
%             if ((s == 17) && (a == 8))
%                 V1_lower_q_a(a)
%             end
        end
        % Maximize in a \in Act
        %             V_lower(q) = max(V_lower_q_a);
        [V1_lower_max, a_max] = max(V1_lower_q_a);
        V1_lower_max = min(1, V1_lower_max);
        if ( V1_lower_max > V_lower(s) )
            V1_lower(s) = V1_lower_max;
            sigma(s) = a_max;
        else
            V1_lower(s) = V_lower(s);
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
    % Non-terminal states
    for si = 1:length(user_param.Q_not_terminal)
        s = user_param.Q_not_terminal(si);
        a = sigma(s);
        P_M = zeros(length( solution.P_upper{s,a}), 1); % Assign lower bound to all
        S = sum(P_M); % Get sum of lower bounds for fixed q and a
        % Order interval values
        V_upper_proj_Q_nd = V_upper(solution.Q_nd{s,a});
        [~, order_V_upper] = sortrows(V_upper_proj_Q_nd, 'descend');
        for qp = 1:length(V_upper_proj_Q_nd)
            index = order_V_upper(qp);
            if (S + solution.P_upper{s,a}(index) -0 < 1)
                S = S + solution.P_upper{s,a}(index) - 0;
                P_M(index) = solution.P_upper{s,a}(index);
            else
                P_M(index) = P_M(index) + 1 - S;
                break;
            end
        end
        V1_upper_max = P_M'*V_upper_proj_Q_nd;
        if (V1_upper_max > V_upper(s))
            V1_upper(s) = V1_upper_max;
        else
            V1_upper(s) = V_upper(s);
        end
    end

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

