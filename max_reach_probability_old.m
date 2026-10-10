function [P_reach_lower, P_reach_upper, sigma, exceeded_tolerance] = max_reach_probability(P_lower, P_upper, user_param,initial_condition,solver_opts)
% Solve the maximal reachability probability problem
% Currently it only uses lower bound to compute the optimal strategy

max_iter = user_param.max_iter;
Nq = user_param.Nq;
Na = user_param.Na;
Q_target = user_param.Q_target;

% Initialize interval value
V_lower = initial_condition.V_lower;
V_upper = initial_condition.V_upper;
V1_lower = initial_condition.V_lower;
V1_upper = initial_condition.V_upper;
for q = 1:Nq
    if (max(q == Q_target) == 1) % If state q \in Q_target
        V_lower(q) = 1;
        V_upper(q) = 1;
        V1_lower(q) = 1;
        V1_upper(q) = 1;
    end
end

% Interval value iteration

% Iterate the lower bound
P_M = zeros(Nq,Na,Nq+1);
P_reach_lower = zeros(Nq+1,1);
sigma = zeros(Nq,1);
exceeded_tolerance = 0;
tic
for i = 1:max_iter
    rho = zeros(Nq,Na);
    for q = 1:Nq
        if (max(q == Q_target) == 0) % If state q \notin Q_target
            V_lower_q_a = zeros(Na,1);
            for a = 1:Na

                %%%%%%%%%%%%%%%
                
                
%                 f = V_lower(1:end-1);
% lb = P_lower(q,a,1:end);
% ub = P_upper(q,a,1:end);
% Aeq = ones(1,user_param.Nq);
% beq = 1;
% 
% tic
% [~,fval] = linprog(f,[],[],Aeq,beq,lb,ub);
% V_lower_q_a(a) = fval;
% 
% 
% time = toc*user_param.Nq*user_param.Na/60;

                %%%%%%%%%%%%%%%%
                
                for q1 = 1:Nq+1
                    P_M(q,a,q1) = P_lower(q,a,q1); % Assign lower bound to all
                end
                S = sum(P_lower(q,a,:)); % Get sum of lower bounds for fixed q and a
                % Order interval values
                [~,order_V_lower] = sortrows(V_lower,'ascend');
                for q1 = 1:Nq+1
                    index = order_V_lower(q1);
                    if (S + P_upper(q,a,index) - P_lower(q,a,index) < 1)
                        S = S + P_upper(q,a,index) - P_lower(q,a,index);
                        P_M(q,a,index) = P_upper(q,a,index);
                    else
                        P_M(q,a,index) = P_M(q,a,index) + 1 - S;
                        break
                    end
                end
                for q1 = 1:Nq % This loop is just a matrix multiplication
                    V_lower_q_a(a) = V_lower_q_a(a) + P_M(q,a,q1)*V_lower(q1);
                end
            end
            % Maximize in a \in Act
            V1_lower(q) = max(V_lower_q_a);
            if (i == max_iter)||(exceeded_tolerance == 1)
                P_reach_lower(q) = V1_lower(q);
                rho(q,:) = (V1_lower(q) == V_lower_q_a);
            end
        else
            P_reach_lower(q) = 1;
        end
    end
    if (exceeded_tolerance == 1)
       break 
    end
    difference_VI = max(V1_lower-V_lower);
    exceeded_tolerance = (difference_VI <= solver_opts.tol_VI);
    
    V_lower = V1_lower;
    % Just for debugging purposes
    %i
    %if (i == 1)
    %    estimated_time = user_param.max_iter*toc/60;
    %    fprintf('Estimated value iteration time = %d min. \n', estimated_time)
    %end
end

% Iterate the upper bound
P_M = zeros(Nq,Na,Nq+1);
P_reach_upper = zeros(Nq+1,1);
exceeded_tolerance = 0;
for i = 1:max_iter
    for q = 1:Nq
        if (max(q == Q_target) == 0) % If state q \notin Q_target
            V_upper_q_a = zeros(Na,1);
            for a = 1:Na
                if (rho(q,a) == 1)
                    for q1 = 1:Nq+1
                        P_M(q,a,q1) = P_lower(q,a,q1); % Assign lower bound to all
                    end
                    S = sum(P_lower(q,a,:)); % Get sum of lower bounds for fixed q and a
                    % Order interval values
                    [~,order_V_upper] = sortrows(V_upper,'descend');
                    for q1 = 1:Nq+1
                        index = order_V_upper(q1);
                        if (S + P_upper(q,a,index) - P_lower(q,a,index) < 1)
                            S = S + P_upper(q,a,index) - P_lower(q,a,index);
                            P_M(q,a,index) = P_upper(q,a,index);
                        else
                            P_M(q,a,index) = P_M(q,a,index) + 1 - S;
                            break
                        end
                    end
                    for q1 = 1:Nq % This loop is just a matrix multiplication
                        V_upper_q_a(a) = V_upper_q_a(a) + P_M(q,a,q1)*V_upper(q1);
                    end
                end
            end
            [V1_upper(q), index] = max(V_upper_q_a);
            if (i==max_iter)||(exceeded_tolerance == 1)
                P_reach_upper(q) = V1_upper(q);
                sigma(q) = index;
            end
        else
            % Assign default action for states q \in Q_target
            sigma(q) = floor(Na/2) + 1;
            P_reach_upper(q) = 1;
        end
    end
    if (exceeded_tolerance == 1)
       break 
    end
    difference_VI = max(V1_upper - V_upper);
    exceeded_tolerance = (difference_VI <= solver_opts.tol_VI);

    V_upper = V1_upper;
    % Just for debugging purposes
    %i
end

end

