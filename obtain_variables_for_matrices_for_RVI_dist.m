function [variables_for_matrices] = obtain_variables_for_matrices_for_RVI_dist(user_param, original_sys, state_discretization, support_P, Q_nd, P_upper,P_lower)

% THIS ONLY WORKS FOR THE INFTY NORM AND HYPERCUBES
% CAN BE MODIFIED TO WORK FOR HYPERRECTANGLES

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
%         ind_dist = uint8( zeros(length(Q_nd{q,a}), length(support_P{q,a})) );
        if ( (Q_nd{q,a}(end) ~= user_param.Nq + 1) && (support_P{q,a}(end) ~= user_param.Nq + 1) )

            Qnd_not_unsafe = Q_nd{q,a};
            supp_P_not_unsafe = support_P{q,a};
            distX = uint8( abs(state_discretization.X_center(Qnd_not_unsafe)/state_discretization.Deltax - state_discretization.X_center(supp_P_not_unsafe)'/Deltax) - 1 );
            distY = uint8( abs(state_discretization.Y_center(Qnd_not_unsafe)/state_discretization.Deltay - state_discretization.Y_center(supp_P_not_unsafe)'/Deltay) - 1 );
            distTH = uint8( abs(state_discretization.TH_center(Qnd_not_unsafe)/state_discretization.Deltath - state_discretization.TH_center(supp_P_not_unsafe)'/Deltath) - 1 );

            % Translate q1 up to account for wrpping up
            distTH_wrapped_up = uint8( abs( state_discretization.TH_center(Qnd_not_unsafe)/Deltath + Nth - state_discretization.TH_center(supp_P_not_unsafe)'/Deltath) - 1 );
            
            % Translate q1 down to account for wrapping dn
            distTH_wrapped_dn = uint8( abs( state_discretization.TH_center(Qnd_not_unsafe)/Deltath -Nth - state_discretization.TH_center(supp_P_not_unsafe)'/Deltath) - 1 );

            dist_TH = min([distTH(:) distTH_wrapped_up(:) distTH_wrapped_dn(:)], [],2);
            dist = max([distX(:) distY(:) distTH(:)], [],2);
            dist = reshape(dist, length(Qnd_not_unsafe), length(supp_P_not_unsafe));

        elseif ( (Q_nd{q,a}(end) ~= user_param.Nq + 1) && (support_P{q,a}(end) == user_param.Nq + 1) )

            Qnd_not_unsafe = Q_nd{q,a};
            supp_P_not_unsafe = support_P{q,a}(1:end-1);
            distX = uint8( abs(state_discretization.X_center(Qnd_not_unsafe)/state_discretization.Deltax - state_discretization.X_center(supp_P_not_unsafe)'/Deltax) - 1 );
            distY = uint8( abs(state_discretization.Y_center(Qnd_not_unsafe)/state_discretization.Deltay - state_discretization.Y_center(supp_P_not_unsafe)'/Deltay) - 1 );
            distTH = uint8( abs(state_discretization.TH_center(Qnd_not_unsafe)/state_discretization.Deltath - state_discretization.TH_center(supp_P_not_unsafe)'/Deltath) - 1 );

            % Translate q1 up to account for wrpping up
            distTH_wrapped_up = uint8( abs( state_discretization.TH_center(Qnd_not_unsafe)/Deltath + Nth - state_discretization.TH_center(supp_P_not_unsafe)'/Deltath) - 1 );
            
            % Translate q1 down to account for wrapping dn
            distTH_wrapped_dn = uint8( abs( state_discretization.TH_center(Qnd_not_unsafe)/Deltath -Nth - state_discretization.TH_center(supp_P_not_unsafe)'/Deltath) - 1 );
            

            dist_TH = min([distTH(:) distTH_wrapped_up(:) distTH_wrapped_dn(:)], [],2);
            dist = max([distX(:) distY(:) distTH(:)], [],2);
            dist =  reshape(dist, length(Qnd_not_unsafe), length(supp_P_not_unsafe));

            % Add last column
            dist = [dist zeros(length(Qnd_not_unsafe), 1, 'uint8')];

        elseif ( (Q_nd{q,a}(end) == user_param.Nq + 1) && (support_P{q,a}(end) ~= user_param.Nq + 1) )

            Qnd_not_unsafe = Q_nd{q,a}(1:end-1);
            supp_P_not_unsafe = support_P{q,a};
            distX = uint8( abs(state_discretization.X_center(Qnd_not_unsafe)/state_discretization.Deltax - state_discretization.X_center(supp_P_not_unsafe)'/Deltax) - 1 );
            distY = uint8( abs(state_discretization.Y_center(Qnd_not_unsafe)/state_discretization.Deltay - state_discretization.Y_center(supp_P_not_unsafe)'/Deltay) - 1 );
            distTH = uint8( abs(state_discretization.TH_center(Qnd_not_unsafe)/state_discretization.Deltath - state_discretization.TH_center(supp_P_not_unsafe)'/Deltath) - 1 );

            % Translate q1 up to account for wrpping up
            distTH_wrapped_up = uint8( abs( state_discretization.TH_center(Qnd_not_unsafe)/Deltath + Nth - state_discretization.TH_center(supp_P_not_unsafe)'/Deltath) - 1 );
            
            % Translate q1 down to account for wrapping dn
            distTH_wrapped_dn = uint8( abs( state_discretization.TH_center(Qnd_not_unsafe)/Deltath -Nth - state_discretization.TH_center(supp_P_not_unsafe)'/Deltath) - 1 );
            

            dist_TH = min([distTH(:) distTH_wrapped_up(:) distTH_wrapped_dn(:)], [],2);
            dist = max([distX(:) distY(:) distTH(:)], [],2);
            dist =  reshape(dist, length(Qnd_not_unsafe), length(supp_P_not_unsafe));

            % Add last row
            dist_to_unsafe = zeros(1, length(supp_P_not_unsafe), 'uint8');
            for q_ind = 1:length(supp_P_not_unsafe)
                q_value = supp_P_not_unsafe(q_ind);
                x = [state_discretization.X_center(q_value); state_discretization.Y_center(q_value)];
                dist_to_unsafe(q_ind) = calculate_distance_to_unsafe_fast(original_sys.state_space_lower_left_corner, original_sys.state_space_upper_right_corner, x, Deltax, Deltay);
            end
            dist = [dist; dist_to_unsafe];

        elseif ( (Q_nd{q,a}(1) == user_param.Nq + 1) && (support_P{q,a}(1) == user_param.Nq + 1) )
            dist = uint8(0);
        elseif ( (Q_nd{q,a}(end) == user_param.Nq + 1) && (support_P{q,a}(end) == user_param.Nq + 1) )

            Qnd_not_unsafe = Q_nd{q,a}(1:end-1);
            supp_P_not_unsafe = support_P{q,a}(1:end-1);
            distX = uint8( abs(state_discretization.X_center(Qnd_not_unsafe)/state_discretization.Deltax - state_discretization.X_center(supp_P_not_unsafe)'/Deltax) - 1 );
            distY = uint8( abs(state_discretization.Y_center(Qnd_not_unsafe)/state_discretization.Deltay - state_discretization.Y_center(supp_P_not_unsafe)'/Deltay) - 1 );
            distTH = uint8( abs(state_discretization.TH_center(Qnd_not_unsafe)/state_discretization.Deltath - state_discretization.TH_center(supp_P_not_unsafe)'/Deltath) - 1 );

            % Translate q1 up to account for wrpping up
            distTH_wrapped_up = uint8( abs( state_discretization.TH_center(Qnd_not_unsafe)/Deltath + Nth - state_discretization.TH_center(supp_P_not_unsafe)'/Deltath) - 1 );
            
            % Translate q1 down to account for wrapping dn
            distTH_wrapped_dn = uint8( abs( state_discretization.TH_center(Qnd_not_unsafe)/Deltath -Nth - state_discretization.TH_center(supp_P_not_unsafe)'/Deltath) - 1 );
            
            dist_TH = min([distTH(:) distTH_wrapped_up(:) distTH_wrapped_dn(:)], [],2);
            dist = max([distX(:) distY(:) distTH(:)], [],2);
            dist =  reshape(dist, length(Qnd_not_unsafe), length(supp_P_not_unsafe));

            % Compute last row
            dist_to_unsafe = zeros(1, length(supp_P_not_unsafe), 'uint8');
            for q_ind = 1:length(supp_P_not_unsafe)
                q_value = supp_P_not_unsafe(q_ind);
                x = [state_discretization.X_center(q_value); state_discretization.Y_center(q_value)];
                dist_to_unsafe(q_ind) = calculate_distance_to_unsafe_fast(original_sys.state_space_lower_left_corner, original_sys.state_space_upper_right_corner, x, Deltax, Deltay);
            end
            % Add last row and column
            dist = [dist zeros(length(Qnd_not_unsafe), 1, 'uint8'); dist_to_unsafe uint8(0)];
        end
        variables_for_matrices.dist{q,a} = dist;
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