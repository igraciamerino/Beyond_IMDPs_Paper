function [P_lower,P_upper,Distance,flag] = bounds_lookup_table_fixed_q_a_old(q,a,state_discretization,original_sys,solver_opts,user_param,samples,Act,lookup_table,lookup_table_X)
% Compute bounds for transitions starting from q, under a

state_space_lower_left_corner = original_sys.state_space_lower_left_corner;
state_space_upper_right_corner = original_sys.state_space_upper_right_corner;
X_center = state_discretization.X_center;
Y_center = state_discretization.Y_center;
Deltax = state_discretization.Deltax;
Deltay = state_discretization.Deltay;
epsilon = user_param.epsilon;
center_ball_Xi_hat = samples.center_ball_Xi_hat;
radius_ball_Xi_hat = samples.radius_ball_Xi_hat;
enable_over_approximation = solver_opts.enable_over_approximation;
over_approximation_distance_threshold = solver_opts.over_approximation_distance_threshold;
precision = solver_opts.precision;
Nq = user_param.Nq;
m = original_sys.m;
vel = original_sys.vel;
L_q_a = original_sys.L_q_a{q,a};
delta_q = state_discretization.delta_q;

Y_table = lookup_table.Y_table;
p_lower_table = lookup_table.p_lower;
p_upper_table = lookup_table.p_upper;
Y_table_X = lookup_table_X.Y_table;
p_lower_table_X = lookup_table_X.p_lower;
p_upper_table_X = lookup_table_X.p_upper;

P_lower = zeros(1,Nq+1);
P_upper = zeros(1,Nq+1);
Distance = zeros(1,Nq);

x_lower_left_q = X_center(q)-Deltax/2;
y_lower_left_q = Y_center(q)-Deltay/2;

% Define vertices of q
vertices_q = zeros (m,4);
vertices_q(:,1) = [x_lower_left_q, y_lower_left_q]';
vertices_q(:,2) = [x_lower_left_q + Deltax, y_lower_left_q]';
vertices_q(:,3) = [x_lower_left_q + Deltax, y_lower_left_q + Deltay]';
vertices_q(:,4) = [x_lower_left_q, y_lower_left_q + Deltay]';

delta_ball_q = sqrt((Deltax/2)^2 + (Deltay/2)^2);

% [A_q,b_q] = matrices_polytope(x_lower_left_q,y_lower_left_q,x_upper_right_q,y_upper_right_q);
u = Act(a);
% c = dynamics_single_integrator(zeros(m,1),u,zeros(m,1),Deltat);
% Define set reached from q by a
Post_q_a = zeros(m,4);
xi = [0 0]';
for k = 1:size(Post_q_a,2)
    v = vertices_q(:,k);
    Post_q_a(:,k) = unicycle2D(v,u,xi,vel);
end

for q1 = 1:Nq
    x_lower_left_q1 = X_center(q1) - Deltax/2;
    y_lower_left_q1 = Y_center(q1) - Deltay/2;
    x_upper_right_q1 = X_center(q1) + Deltax/2;
    y_upper_right_q1 = Y_center(q1) + Deltay/2;
    [A_q1,b_q1] = matrices_polytope(x_lower_left_q1,y_lower_left_q1,x_upper_right_q1,y_upper_right_q1);
    
    % Compute minimum distance between rectangle q1 and polytope T
    % that contains the samples X_star_hat
    D_min_wrapped = 0;

    h_max = max(Reach)
    radius_ball_1 = L_q_a*delta_q + radius_ball_Xi_hat;
    center_ball_1 = unicycle3D([X_center(q) Y_center(q)]',u,xi,vel) + center_ball_Xi_hat;
    D_min = calculate_distance(A_q1, b_q1, center_ball_1, 1) - radius_ball_1;
    if (D_min < 0)
        D_min = 0;
    end

    if (D_min <= over_approximation_distance_threshold)||(D_min_wrapped <= over_approximation_distance_threshold)
        % Solve with lookup table
        Post_q_a_moved = Post_q_a - [x_lower_left_q1 y_lower_left_q1]';
        in1 = inhull(Y_table', Post_q_a_moved');
        % Check if there's grid points inside Post_q_a_moved
        if (isempty(in1))
            disp('Error. No grid points found in Post_q_a_moved')
        end
        P_lower(q1) = min(p_lower_table(in1));
        P_upper(q1) = max(p_upper_table(in1));
    end
    Distance(q1) = D_min;
end

% Compute probabilities of reaching q_u
q1 = Nq + 1;
[A_X,b_X] = matrices_polytope(state_space_lower_left_corner(1),state_space_lower_left_corner(2),state_space_upper_right_corner(1),state_space_upper_right_corner(2));
% Compute minimum distance between rectangle q1 and polytope T
% that contains the samples X_star_hat
D_min = 0;
P_lower_X = 0;

    if (D_min >= over_approximation_distance_threshold)
        % Use over-approximation
        if (D_min > precision)&&(epsilon/D_min <= 1)
            P_upper_X = (epsilon/D_min)^2;
        elseif (D_min <= precision)||(epsilon/D_min > 1)
            P_upper_X = 1;
        end
    end
if (enable_over_approximation == 1)
    radius_ball_1 = radius_ball_Xi_hat + delta_ball_q;
    center_ball_1 = unicycle2D([X_center(q) Y_center(q)]',u,xi,vel) + center_ball_Xi_hat;
    dist_to_X = calculate_distance(A_X, b_X, center_ball_1, 1);
    dist_to_outside_of_X = calculate_distance(A_X, b_X, center_ball_1, 0);
    if (dist_to_X - radius_ball_1 > 0)
        D_min = dist_to_X - radius_ball_1;
        if (D_min > over_approximation_distance_threshold)
            % Use over-approximation
            if (D_min > precision)&&(epsilon/D_min <= 1)
                P_upper_X = (epsilon/D_min)^2;
            elseif (D_min <= precision)||(epsilon/D_min > 1)
                P_upper_X = 1;
            end
        end
    elseif (dist_to_outside_of_X - radius_ball_1 > 0)
        D_min = dist_to_outside_of_X - radius_ball_1;
        P_upper_X = 1;
        if (D_min > over_approximation_distance_threshold)
            % Use over-approximation
            if (D_min > precision)&&(epsilon/D_min <= 1)
                P_lower_X = 1 - (epsilon/D_min)^2;
            elseif (D_min <= precision)||(epsilon/D_min > 1)
                P_lower_X = 0;
            end
        end
    else
        P_upper_X = 1;
    end
end
    
if (enable_over_approximation == 0)||(D_min <= over_approximation_distance_threshold)
    % Solve with lookup table
        in1 = inhull(Y_table_X', Post_q_a');
        % Check if there's grid points inside Post_q_a
        if (isempty(in1))
            disp('Error. No grid points found in Post_q_a')
        end
        P_lower_X = min(p_lower_table_X(in1));
        P_upper_X = max(p_upper_table_X(in1));
end
P_lower(q1) = 1 - P_upper_X;
P_upper(q1) = 1 - P_lower_X;

% Check bounds of the abstraction
if (sum(P_lower(:)) > 1 + solver_opts.precision) || (sum(P_upper(:)) < 1 - solver_opts.precision)
    disp('Error found while constructing the abstraction')
    flag = 'error';
else
    flag = 'no error';
end

end