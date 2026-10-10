function [variables_for_matrices] = obtain_variables_for_matrices_for_RVI_old(user_param, original_sys, state_discretization, support_P, Q_nd, P_upper,P_lower)

m = user_param.m;
Nth = user_param.Nth;
Deltax = state_discretization.Deltax;
Deltay = state_discretization.Deltay;
Deltath = state_discretization.Deltath;

variables_for_matrices.Qs = cell(user_param.Nq,user_param.Na);
variables_for_matrices.Q_nd = cell(user_param.Nq,user_param.Na);
variables_for_matrices.P_lower_nom = cell(user_param.Nq,user_param.Na);
variables_for_matrices.P_upper_nom = cell(user_param.Nq,user_param.Na);
variables_for_matrices.s = cell(user_param.Nq,user_param.Na);
variables_for_matrices.epsilon_robust = cell(user_param.Nq,user_param.Na);
variables_for_matrices.dist = cell(user_param.Nq,user_param.Na);
variables_for_matrices.ind_dist = cell(user_param.Nq,user_param.Na);

% New dist
for q = 1:user_param.Nq
    for a = 1:user_param.Na
%         dist = dist_max*ones(length(Q_nd{q,a}), length(support_P{q,a}));
        dist = uint8( zeros(length(Q_nd{q,a}), length(support_P{q,a})) );
        ind_dist = uint8( zeros(length(Q_nd{q,a}), length(support_P{q,a})) );
        for q1_index = 1:length(Q_nd{q,a})
            q1 = Q_nd{q,a}(q1_index);
            for q2_index = 1:length(support_P{q,a})
                q2 = support_P{q,a}(q2_index);
                if ( (q1 < user_param.Nq + 1) && (q2 < user_param.Nq + 1) )
                    if (m == inf)
                        center_q1 = [state_discretization.X_center(q1); state_discretization.Y_center(q1); state_discretization.TH_center(q1)];
                        center_q2 = [state_discretization.X_center(q2); state_discretization.Y_center(q2); state_discretization.TH_center(q2)];
                        diff = abs(center_q1 - center_q2);
                        diam_inf_q = [Deltax; Deltay; Deltath];
                        if ( any( diff - diam_inf_q > 1e-6 ) ) % With precision
                            [dist_not_wrapped, ind_not_wrapped] = max(diff - diam_inf_q);
                        else
                            dist(q1_index, q2_index) = uint8(0);
                            ind_dist(q1_index, q2_index) = uint8(1);
                            continue;
                        end
                        % Translate q1 up to account for wrpping up
                        center_q1 = center_q1 + Deltath*Nth*[0; 0; 1];
                        diff = abs(center_q1 - center_q2);
                        if ( any( diff - diam_inf_q > 1e-6 ) ) % With precision
                            [dist_wrapped_up, ind_wrapped_up] = max(diff - diam_inf_q);
                        else
                            dist(q1_index, q2_index) = uint8(0);
                            ind_dist(q1_index, q2_index) = uint8(1);
                            continue;
                        end
                        % Translate q1 down to account for wrapping dn
                        center_q1 = center_q1 - 2*Deltath*Nth*[0; 0; 1]; % x2
                        diff = abs(center_q1 - center_q2);
                        if ( any( diff - diam_inf_q > 1e-6 ) ) % With precision
                            [dist_wrapped_dn, ind_wrapped_dn] = max(diff - diam_inf_q);
                        else
                            dist(q1_index, q2_index) = uint8(0);
                            ind_dist(q1_index, q2_index) = uint8(1);
                            continue;
                        end
                        [dist_val, ind_wrapping] = min([dist_not_wrapped, dist_wrapped_up, dist_wrapped_dn]);
                        if ( ind_wrapping == 1)
                            ind_dist(q1_index, q2_index) = uint8(ind_not_wrapped);
                            dist(q1_index, q2_index) = uint8( dist_val/Deltax );
                        elseif ( ind_wrapping == 2)
                            ind_dist(q1_index, q2_index) = uint8(ind_wrapped_up);
                            dist(q1_index, q2_index) = uint8( dist_val/Deltay );
                        else
                            ind_dist(q1_index, q2_index) = uint8(ind_wrapped_dn);
                            dist(q1_index, q2_index) = uint8( dist_val/Deltath );
                        end
                    else
                        vertices_q1 = state_discretization.vertices_q{q1};
                        vertices_q2 = state_discretization.vertices_q{q2};
                        for k_q1 = 1:4
                            for k_q2 = 1:4
                                diff = vertices_q1(:,k_q1) - vertices_q2(:,k_q2);
                                dist(q1_index, q2_index) = min(dist(q1_index, q2_index), norm(diff,m));
                            end
                        end
                        % Translate q1 up to account for wrpping up
                        vertices_q1 = state_discretization.vertices_q{q1} + Deltath*Nth*[0; 0; 1];
                        for k_q1 = 1:4
                            for k_q2 = 1:4
                                diff = vertices_q1(:,k_q1) - vertices_q2(:,k_q2);
                                dist(q1_index, q2_index) = min(dist(q1_index, q2_index), norm(diff,m));
                            end
                        end
                        % Translate q1 down to account for wrapping dn
                        % left
                        vertices_q1 = state_discretization.vertices_q{q1} - Deltath*Nth*[0; 0; 1];
                        for k_q1 = 1:4
                            for k_q2 = 1:4
                                diff = vertices_q1(:,k_q1) - vertices_q2(:,k_q2);
                                dist(q1_index, q2_index) = min(dist(q1_index, q2_index), norm(diff,m));
                            end
                        end
                    end
                elseif (q2 == user_param.Nq + 1)
                    dist(q1_index, q2_index) = uint8(0);
                    ind_dist(q1_index, q2_index) = uint8(1);
                else
                    vertices_q2 = state_discretization.vertices_q{q2};
                    dist_val = uint8(100);
                    for k_q2 = 1:4
                        [dist_to_outside_of_X, ind_dist_aux] = calculate_distance_to_unsafe_with_index(original_sys.state_space_lower_left_corner, original_sys.state_space_upper_right_corner, vertices_q2(:,k_q2), Deltax, Deltay, Deltath);
                        if (dist_to_outside_of_X < dist_val)
                            dist_val = dist_to_outside_of_X;
                            ind_wrapping = ind_dist_aux; % I reuse ind_wrapping although its name doesn't make sense here
                        end
                    end
                    ind_dist(q1_index, q2_index) = ind_wrapping;
                    dist(q1_index, q2_index) = uint8(dist_val);
                end
            end
        end
        variables_for_matrices.dist{q,a} = dist;
        variables_for_matrices.ind_dist{q,a} = ind_dist;
    end
    % Display progress
    if (rem(q,1000) == 0)
        disp(q)
    end
end

% Compute variables for each q,a
for q = 1:user_param.Nq
    for a = 1:user_param.Na
        variables_for_matrices.Qs{q,a} = support_P{q,a};
        variables_for_matrices.Q_nd{q,a} = Q_nd{q,a};
        variables_for_matrices.P_lower_nom{q,a} = P_lower{q,a};
        variables_for_matrices.P_upper_nom{q,a} = P_upper{q,a};
        variables_for_matrices.s{q,a} = length(support_P{q,a});
        variables_for_matrices.epsilon_robust{q,a} = original_sys.Lw_q_a*user_param.epsilon;
    end
end

end