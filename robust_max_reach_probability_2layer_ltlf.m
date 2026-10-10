function [P_reach_lower, P_reach_upper, sigma] = robust_max_reach_probability_2layer_ltlf(user_param, initial_condition, variables_for_matrices, solver_opts)
% Solve the maximal reachability probability problem
% Currently it only uses lower bound to compute the optimal strategy

tol_VI = solver_opts.tol_VI;
max_iter = user_param.max_iter;
Ns = user_param.Ns;
Na = user_param.Na;

% Initialize interval value
V_lower = initial_condition.V_lower;
V_upper = initial_condition.V_upper;
sigma = initial_condition.sigma_PROD;
V1_lower = zeros(user_param.Ns+1,4);
V1_upper = zeros(user_param.Ns+1,4);
V_lower_proj_S = zeros(Ns+1,1);
V_upper_proj_S = zeros(Ns+1,1);
V1_lower_s_a = zeros(Na,1);

% If state z \in Z_acc
V_lower(:,2) = ones(user_param.Ns+1,1);
V1_lower(:,2) = ones(user_param.Ns+1,1);
V_upper(:,2) = ones(user_param.Ns+1,1);
V1_upper(:,2) = ones(user_param.Ns+1,1);

% Interval value iteration

% Iterate the lower bound
Z_not_sink = [1 3];
tic
for i = 1:max_iter
    % Non-terminal states
    for z = Z_not_sink
        for sp = 1:Ns + 1
            zp = variables_for_matrices.ZpDFA(z,sp);
            V_lower_proj_S(sp) = V_lower(sp,zp);
        end
        for si = 1:length(user_param.S_not_terminal)
            s = user_param.S_not_terminal(si);
            for a = 1:Na
                Mass = 1;
                % Order interval values
                V_lower_proj_Sp = V_lower_proj_S(variables_for_matrices.Sp{s,a});
                gamma = variables_for_matrices.P_lower{s,a}';
                gamma_coarse = zeros(1, length(variables_for_matrices.Qp{s,a}) );
                if (any(gamma))
                    for spi_unordered = 1:length(variables_for_matrices.Sp{s,a})
                        sp = variables_for_matrices.Sp{s,a}(spi_unordered);
                        qp = variables_for_matrices.parents{sp};
                        if ( ~isempty(qp) )
                            qpi = find(variables_for_matrices.Qp{s,a} == qp);
                            gamma_coarse(qpi) = gamma_coarse(qpi) + gamma(spi_unordered);
                        end
                        Mass = Mass - gamma(spi_unordered);
                    end
                end
                [~, order_V_lower_proj_Sp] = sortrows(V_lower_proj_Sp, 'ascend'); % Vector that maps the spi in variables_for_matrices to the spi in the ordered set
                for qpi = 1:length(variables_for_matrices.Qp{s,a})
                    mass = variables_for_matrices.P_lower_coarse{s,a}(qpi) - gamma_coarse(qpi);
                    if (mass > 0 )
                        qp = variables_for_matrices.Qp{s,a}(qpi);
                        if ( ~isempty(qp) )
                            children = cell2mat( variables_for_matrices.children(qp) );
                            [~, order_children] = sortrows(V_lower_proj_S(children), 'ascend');
                            children_ordered = children(order_children);
                            for sp = children_ordered'
                                spi = find(variables_for_matrices.Sp{s,a} == sp);
                                if (~isempty(spi))
                                    g = min(mass, variables_for_matrices.P_upper{s,a}(spi) - gamma(spi));
                                    gamma(spi) = gamma(spi) + g;
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
                    %                 if (mass ~= 0)
                    %                     disp('ERROR: Not all mass has been assigned...')
                    %                 end
                end
                for spi_unordered = 1:length(variables_for_matrices.Sp{s,a})
                    spi = order_V_lower_proj_Sp(spi_unordered);
                    sp = variables_for_matrices.Sp{s,a}(spi);
                    qp = variables_for_matrices.parents{sp};
                    if ( ~isempty(qp) )
                        qpi = find(variables_for_matrices.Qp{s,a} == qp);
                        mass = min([Mass, variables_for_matrices.P_upper_coarse{s,a}(qpi)-gamma_coarse(qpi), variables_for_matrices.P_upper{s,a}(spi)-gamma(spi)] );
                        gamma_coarse(qpi) = gamma_coarse(qpi) + mass;
                    else
                        mass = min([Mass, variables_for_matrices.P_upper{s,a}(spi)-gamma(spi)] );
                    end
                    gamma(spi) = gamma(spi) + mass;
                    Mass = Mass - mass;
                    if (Mass == 0)
                        break;
                    end
                end
                if (Mass > 0)
                    disp('Error: not all mass assigned');
                end
                V1_lower_s_a(a) = gamma*V_lower_proj_Sp;
            end
            % Maximize in a \in Act
            %             V_lower(q) = max(V_lower_q_a);
            [V1_lower(s,z), a_max] = max(V1_lower_s_a);
            if ( V1_lower(s,z) > V_lower(s,z) )
                sigma(s,z) = a_max;
            end
        end
    end

    err = max(abs(V1_lower(:) - V_lower(:)));
    V_lower = V1_lower;

    toc
    fprintf('Iteration = %d . \n', i)
    fprintf('Diference between consecutive value functions = %f . \n', err)

    if ( (i == max_iter) || (err < tol_VI) )
        P_reach_lower = V_lower;
        break;
    end
end
% Or don't iterate
% P_reach_lower = V_lower;

% Iterate the upper bound
for i = 1:max_iter
    % Non-terminal states
    for z = Z_not_sink
        for sp = 1:Ns + 1
            zp = variables_for_matrices.ZpDFA(z,sp);
            V_upper_proj_S(sp) = V_upper(sp,zp);
        end
        for si = 1:length(user_param.S_not_terminal)
            s = user_param.S_not_terminal(si);
            a = sigma(s,z);
            Mass = 1;
            % Order interval values
            V_upper_proj_Sp = V_upper_proj_S(variables_for_matrices.Sp{s,a});
            gamma = variables_for_matrices.P_lower{s,a}';
            gamma_coarse = zeros(1, length(variables_for_matrices.Qp{s,a}) );
            if (any(gamma))
                for spi_unordered = 1:length(variables_for_matrices.Sp{s,a})
                    sp = variables_for_matrices.Sp{s,a}(spi_unordered);
                    qp = variables_for_matrices.parents{sp};
                    if ( ~isempty(qp) )
                        qpi = find(variables_for_matrices.Qp{s,a} == qp);
                        gamma_coarse(qpi) = gamma_coarse(qpi) + gamma(spi_unordered);
                    end
                    Mass = Mass - gamma(spi_unordered);
                end
            end
            [~, order_V_upper] = sortrows(V_upper_proj_Sp, 'descend'); % Vector that maps the spi in variables_for-matrices to the spi in the ordered set
            for qpi = 1:length(variables_for_matrices.Qp{s,a})
                mass = variables_for_matrices.P_lower_coarse{s,a}(qpi) - gamma_coarse(qpi);
                if (mass > 0 )
                    qp = variables_for_matrices.Qp{s,a}(qpi);
                    if ( ~isempty(qp) )
                        children = cell2mat( variables_for_matrices.children(qp) );
                        [~, order_children] = sortrows(V_upper_proj_S(children), 'descend');
                        children_ordered = children(order_children);
                        for sp = children_ordered'
                            spi = find(variables_for_matrices.Sp{s,a} == sp);
                            if (~isempty(spi))
                                g = min(mass, variables_for_matrices.P_upper{s,a}(spi) - gamma(spi));
                                gamma(spi) = gamma(spi) + g;
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
                %                 if (mass ~= 0)
                %                     disp('ERROR: Not all mass has been assigned...')
                %                 end
            end
            for spi_unordered = 1:length(variables_for_matrices.Sp{s,a})
                spi = order_V_upper(spi_unordered);
                sp = variables_for_matrices.Sp{s,a}(spi);
                qp = variables_for_matrices.parents{sp};
                if ( ~isempty(qp) )
                    qpi = find(variables_for_matrices.Qp{s,a} == qp);
                    mass = min([Mass, variables_for_matrices.P_upper_coarse{s,a}(qpi)-gamma_coarse(qpi), variables_for_matrices.P_upper{s,a}(spi)-gamma(spi)] );
                    gamma_coarse(qpi) = gamma_coarse(qpi) + mass;
                else
                    mass = min([Mass, variables_for_matrices.P_upper{s,a}(spi)-gamma(spi)] );
                end
                gamma(spi) = gamma(spi) + mass;
                Mass = Mass - mass;
                if (Mass == 0)
                    break;
                end
            end
            if (Mass > 0)
                disp('Error: not all mass assigned');
            end
            V1_upper(s, z) = gamma*V_upper_proj_Sp;
        end
    end

    err = max(abs(V1_upper(:) - V_upper(:)));
    V_upper = V1_upper;

    toc
    fprintf('Iteration = %d . \n', i)
    fprintf('Diference between consecutive value functions = %f . \n', err)

    if ( (i == max_iter) || (err < tol_VI) )
        P_reach_upper = V_upper;
        break;
    end
end

% Or don't iterate
% P_reach_upper = V_upper;

end

