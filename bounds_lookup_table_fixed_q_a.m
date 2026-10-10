function [P_lower,P_upper,support_qp,flag] = bounds_lookup_table_fixed_q_a(q,a,state_discretization,original_sys,solver_opts,user_param,samples,Act,lookup_table,lookup_table_X)
% Compute bounds for transitions starting from q, under a

state_space_lower_left_corner = original_sys.state_space_lower_left_corner;
state_space_upper_right_corner = original_sys.state_space_upper_right_corner;

R_app_q_a = original_sys.Reach_app_q_a{q,a};
L_q_a = original_sys.L_q_a{q,a};
vel = original_sys.vel;
X_center = state_discretization.X_center;
Y_center = state_discretization.Y_center;
TH_center = state_discretization.TH_center;
Deltax = state_discretization.Deltax;
Deltay = state_discretization.Deltay;
Deltath = state_discretization.Deltath;
u = Act(a);

delta_q = state_discretization.delta_q;
epsilon = user_param.epsilon;
% A_T = samples.A;
% b_T = samples.b;
center_ball_Xi_hat = samples.center_ball_Xi_hat;
radius_ball_Xi_hat = samples.radius_ball_Xi_hat;
enable_over_approximation = solver_opts.enable_over_approximation;
over_approximation_distance_threshold = solver_opts.over_approximation_distance_threshold;
precision = solver_opts.precision;
Nq = user_param.Nq;
% m = original_sys.m;
% N = user_param.N;

Y_table = lookup_table.Y_table;
p_lower_table = lookup_table.p_lower;
p_upper_table = lookup_table.p_upper;
Y_table_X = lookup_table_X.Y_table;
p_lower_table_X = lookup_table_X.p_lower;
p_upper_table_X = lookup_table_X.p_upper;

P_lower = zeros(1,Nq+1);
P_upper = zeros(1,Nq+1);

% x_lower_left_q = X_center(q)-Deltax/2;
% y_lower_left_q = Y_center(q)-Deltay/2;

% % Define vertices of q (No need now)
% vertices_q = zeros (m,4);
% vertices_q(:,1) = [x_lower_left_q, y_lower_left_q]';
% vertices_q(:,2) = [x_lower_left_q + Deltax, y_lower_left_q]';
% vertices_q(:,3) = [x_lower_left_q + Deltax, y_lower_left_q + Deltay]';
% vertices_q(:,4) = [x_lower_left_q, y_lower_left_q + Deltay]';

% [A_q,b_q] = matrices_polytope(x_lower_left_q,y_lower_left_q,Deltax,Deltay);
% c = JacksonNL(zeros(m,1),a,zeros(m,1),Deltat);

counter = 1;
support_qp = zeros(1,Nq+1);
p_lower = 0;
p_upper = 0;
for q1 = 1:Nq
    a_rectangle_q1 = [0;0;0];
    b_rectangle_q1 = [0;0;0];
    a_rectangle_q1(1) = X_center(q1) - Deltax/2;
    a_rectangle_q1(2) = Y_center(q1) - Deltay/2;
    a_rectangle_q1(3) = TH_center(q1) - Deltath/2;
    b_rectangle_q1(1) = X_center(q1) + Deltax/2;
    b_rectangle_q1(2) = Y_center(q1) + Deltay/2;
    b_rectangle_q1(3) = TH_center(q1) + Deltath/2;
    [A_q1,b_q1] = matrices_polytope(a_rectangle_q1,b_rectangle_q1);

    % Compute minimum distance between rectangle q1 and polytope T
    % that contains the samples X_star_hat

    radius_ball_1 = L_q_a*delta_q + radius_ball_Xi_hat;
    x = [X_center(q) Y_center(q) TH_center(q)]';
    xi = [0 0 0]';
    center_ball_1 = unicycle3D(x,u,xi,vel) + center_ball_Xi_hat;
    D_min = sqrt(calculate_distance(A_q1, b_q1, center_ball_1, 1)) - radius_ball_1;
    
    if (D_min < 0)
        % Solve with lookup table (not wrapped)
        Post_q_a_moved = R_app_q_a - a_rectangle_q1;
        in1 = inhull(Y_table', Post_q_a_moved');
        % Check if there's grid points inside Post_q_a_moved
        if (any(in1))
            p_lower = min(p_lower_table(in1));
            p_upper = max(p_upper_table(in1));
        else
            disp('Error. No grid points found in Post_q_a_moved')
            flag = 'error';
        end
        
    else
        % Take into account wrappping of th
        th_min = min(R_app_q_a(3,:)) + center_ball_Xi_hat(3) - radius_ball_Xi_hat;
        th_max = max(R_app_q_a(3,:)) + center_ball_Xi_hat(3) + radius_ball_Xi_hat;
        D_min_wrapped = 0;
        if (th_min < 0)
            n_wraps = floor(th_min);
            center_ball_1_wrapped = center_ball_1 - [0 0 n_wraps]';
            D_min_wrapped = sqrt(calculate_distance(A_q1, b_q1, center_ball_1_wrapped, 1)) - radius_ball_1;
        elseif (th_max > 1)
            n_wraps = floor(th_max);
            center_ball_1_wrapped = center_ball_1 - [0 0 n_wraps]';
            D_min_wrapped = sqrt(calculate_distance(A_q1, b_q1, center_ball_1_wrapped, 1)) - radius_ball_1;
        end
        if (D_min_wrapped < 0)
            Post_q_a_moved = R_app_q_a - [0 0 n_wraps]' - a_rectangle_q1;
            in1 = inhull(Y_table', Post_q_a_moved');
            % Check if there's grid points inside Post_q_a_moved
            if (any(in1))
                p_lower = min(p_lower_table(in1));
                p_upper = max(p_upper_table(in1));
            else
                disp('Error. No grid points found in Post_q_a_moved')
                flag = 'error';
            end
        end
    end

    if (p_upper ~= 0)
        support_qp(counter) = q1;
        P_lower(counter) = p_lower;
        P_upper(counter) = p_upper;
        p_upper = 0;
        counter = counter + 1;
    end
        

end

% Compute probabilities of reaching q_u

q1 = Nq + 1;
a_rectangle_q1(1) = state_space_lower_left_corner(1);
a_rectangle_q1(2) = state_space_lower_left_corner(2);
a_rectangle_q1(3) = state_space_lower_left_corner(3);
b_rectangle_q1(1) = state_space_upper_right_corner(1);
b_rectangle_q1(2) = state_space_upper_right_corner(2);
b_rectangle_q1(3) = state_space_upper_right_corner(3);
[A_X,b_X] = matrices_polytope(a_rectangle_q1,b_rectangle_q1);

% Compute minimum distance between rectangle q1 and polytope T
% that contains the samples X_star_hat

x = [X_center(q) Y_center(q) TH_center(q)]';
xi = [0 0 0]';
x1 = unicycle3D(x,a,xi,vel);
center_ball_1 = x1(1:2) + center_ball_Xi_hat(1:2);
dist_to_X = sqrt(calculate_distance2D(A_X(1:4,1:2), b_X(1:4), center_ball_1, 1));
dist_to_outside_of_X = sqrt(calculate_distance2D(A_X(1:4,1:2), b_X(1:4), center_ball_1, 0));
if (dist_to_X - radius_ball_1 > 0) % I'm completely outside X
%     D_min = dist_to_X - radius_ball_1;
    P_upper_X = 0; % Upper bound on the probability of going to X
    P_lower_X = 0; % Lower bound on the probability of going to X
elseif (dist_to_outside_of_X - radius_ball_1 > 0) % I'm completely inside X
%     D_min = dist_to_outside_of_X - radius_ball_1;
    P_upper_X = 1; % Upper bound on the probability of going to X
    P_lower_X = 1; % Lower bound on the probability of going to X
else
    % Solve with lookup table
    R_app_q_a_proj = R_app_q_a(1:2,:);

    in1 = inhull(Y_table_X', R_app_q_a_proj');
    % Check if there's grid points inside Post_q_a
    if (any(in1))
        P_lower_X = min(p_lower_table_X(in1));
        P_upper_X = max(p_upper_table_X(in1));
    else
        disp('Error. No grid points found in Post_q_a')
        flag = 'error';
    end
end


p_lower = 1 - P_upper_X;
p_upper = 1 - P_lower_X;

if (p_upper ~= 0)
    support_qp(counter) = q1;
    P_lower(counter) = p_lower;
    P_upper(counter) = p_upper;
end

support_qp = nonzeros(support_qp);
length_support_q1 = length(support_qp);
P_lower = P_lower(1:length_support_q1);
P_upper = P_upper(1:length_support_q1);


% Check bounds of the abstraction
if (sum(P_lower(:)) > 1 + solver_opts.precision) || (sum(P_upper(:)) < 1 - solver_opts.precision)
    disp('Error found while constructing the abstraction')
    flag = 'error';
else
    flag = 'no error';
end

end