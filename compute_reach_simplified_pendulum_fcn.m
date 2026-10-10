function [a_Post, b_Post, error_flag] = compute_reach_simplified_pendulum_fcn(vertices,u,w,original_sys,user_param)

% Computes vertices of Post(q,u,w) given q (and vertices),
% u, w

debug = 0;

% Normalization transformation (x,y) |--> x
C = [1/(2*pi) 0
    0 1/(2*user_param.th_dot_max)];
C_inv = [2*pi 0
    0 2*user_param.th_dot_max];
d = [1/2 1/2]';
error_flag = 0;

a_s = vertices(:, 1);
b_s = vertices(:, end);

% De-normalize state
x_a_rectangle = 2*pi*a_s(1) - pi;
x_b_rectangle = 2*pi*b_s(1) - pi;
y_a_rectangle = 2*user_param.th_dot_max*a_s(2) - user_param.th_dot_max;
y_b_rectangle = 2*user_param.th_dot_max*b_s(2) - user_param.th_dot_max;

% Overapproximate sine and cosine functions
[ms_under,ms_over,ns_under,ns_over] = affine_under_and_overapproximate_sinusoidal(x_a_rectangle,x_b_rectangle,1); % Overapproximate sine
[mc_under,mc_over,nc_under,nc_over] = affine_under_and_overapproximate_sinusoidal(x_a_rectangle,x_b_rectangle,0); % Overapproximate cosine

% Overapproximate parabola function for x and for y
[mp_x_under, mp_x_over, np_x_under, np_x_over] = affine_under_and_overapproximate_parabola(x_a_rectangle, x_b_rectangle);
[mp_y_under, mp_y_over, np_y_under, np_y_over] = affine_under_and_overapproximate_parabola(y_a_rectangle, y_b_rectangle);

% McCormick envelopes for x*y
[Az_under, Az_over, Bz_under, Bz_over] = mccormick([x_a_rectangle; y_a_rectangle], [x_b_rectangle; y_b_rectangle]);

% Some auxiliary matrices
H_under1 = [w^2*mc_over^2*mp_x_under + 2*w^2*mc_over*nc_over - 2*original_sys.l*w*mc_over*Az_over(1,1), original_sys.l^2*mp_y_under - 2*original_sys.l*w*mc_over*Az_over(1,2) - 2*original_sys.l*w*nc_over];
h_under1 = original_sys.l^2*np_y_under + w^2*mc_over^2*np_x_under + w^2*nc_over^2 - 2*original_sys.l*w*mc_over*Bz_over(1);
H_over1 = [w^2*mc_under^2*mp_x_over + 2*w^2*mc_under*nc_under - 2*original_sys.l*w*mc_under*Az_under(1,1), original_sys.l^2*mp_y_over - 2*original_sys.l*w*mc_under*Az_under(1,2) - 2*original_sys.l*w*nc_under];
h_over1 = original_sys.l^2*np_y_over + w^2*mc_under^2*np_x_over + w^2*nc_under^2 - 2*original_sys.l*w*mc_under*Bz_under(1);

H_under2 = [w^2*mc_over^2*mp_x_under + 2*w^2*mc_over*nc_over - 2*original_sys.l*w*mc_over*Az_over(2,1), original_sys.l^2*mp_y_under - 2*original_sys.l*w*mc_over*Az_over(2,2) - 2*original_sys.l*w*nc_over];
h_under2 = original_sys.l^2*np_y_under + w^2*mc_over^2*np_x_under + w^2*nc_over^2 - 2*original_sys.l*w*mc_over*Bz_over(2);
H_over2 = [w^2*mc_under^2*mp_x_over + 2*w^2*mc_under*nc_under - 2*original_sys.l*w*mc_under*Az_under(2,1), original_sys.l^2*mp_y_over - 2*original_sys.l*w*mc_under*Az_under(2,2) - 2*original_sys.l*w*nc_under];
h_over2 = original_sys.l^2*np_y_over + w^2*mc_under^2*np_x_over + w^2*nc_under^2 - 2*original_sys.l*w*mc_under*Bz_under(2);

% And don't forget about sign(l*y - w*cos(x))
cond1 = ( original_sys.l*y_a_rectangle >= w*cos(x_a_rectangle) );
cond2 = ( original_sys.l*y_a_rectangle >= w*cos(x_b_rectangle) );
cond3 = ( original_sys.l*y_b_rectangle >= w*cos(x_a_rectangle) );
cond4 = ( original_sys.l*y_b_rectangle >= w*cos(x_b_rectangle) );

if ( cond1 && cond2 && cond3 && cond4 ) % Positive sign
    % Do nothing
elseif ( ~cond1 && ~cond2 && ~cond3 && ~cond4 ) % Negative sign
    H_under1 = -H_under1;
    H_over1 = -H_over1;
    h_under1 = -h_under1;
    h_over1 =  -h_over1;

    h_under2 = -h_under2;
    h_over2 = -h_over2;
    H_under2 = -H_under2;
    H_over2 =  -H_over2;
elseif ( any([cond1 cond2 cond3 cond4]) && any(~[cond1 cond2 cond3 cond4]) ) % Negative sign only for lower bnd
    h_under1 = -h_under1;
    H_under1 = -H_under1;

    h_under2 = -h_under2;
    H_under2 = -H_under2;
end

% Obtain un-normalized approximators
A_under1 = zeros(2);
A_over1 = zeros(2);
A_under1(1,1:2) = [1 user_param.Deltat];
A_under1(2,1:2) = -user_param.Deltat*original_sys.cd/(original_sys.M*original_sys.l^2)*H_over1 - [user_param.Deltat*original_sys.g/original_sys.l*ms_over 0] + [0 1];
A_over1(1,1:2) = A_under1(1,1:2);
A_over1(2,1:2) = -user_param.Deltat*original_sys.cd/(original_sys.M*original_sys.l^2)*H_under1 - [user_param.Deltat*original_sys.g/original_sys.l*ms_under 0] + [0 1];

b_under1 = [0
    -user_param.Deltat*original_sys.cd/(original_sys.M*original_sys.l^2)*h_over1 - user_param.Deltat*original_sys.g/original_sys.l*ns_over + user_param.Deltat*u/(original_sys.M*original_sys.l^2) ];
b_over1 = [0
    -user_param.Deltat*original_sys.cd/(original_sys.M*original_sys.l^2)*h_under1 - user_param.Deltat*original_sys.g/original_sys.l*ns_under + user_param.Deltat*u/(original_sys.M*original_sys.l^2) ];

A_under2 = zeros(2);
A_over2 = zeros(2);
A_under2(1,1:2) = [1 user_param.Deltat];
A_under2(2,1:2) = -user_param.Deltat*original_sys.cd/(original_sys.M*original_sys.l^2)*H_over2 - [user_param.Deltat*original_sys.g/original_sys.l*ms_over 0] + [0 1];
A_over2(1,1:2) = A_under2(1,1:2);
A_over2(2,1:2) = -user_param.Deltat*original_sys.cd/(original_sys.M*original_sys.l^2)*H_under2 - [user_param.Deltat*original_sys.g/original_sys.l*ms_under 0] + [0 1];

b_under2 = [0
    -user_param.Deltat*original_sys.cd/(original_sys.M*original_sys.l^2)*h_over2 - user_param.Deltat*original_sys.g/original_sys.l*ns_over + user_param.Deltat*u/(original_sys.M*original_sys.l^2) ];
b_over2 = [0
    -user_param.Deltat*original_sys.cd/(original_sys.M*original_sys.l^2)*h_under2 - user_param.Deltat*original_sys.g/original_sys.l*ns_under + user_param.Deltat*u/(original_sys.M*original_sys.l^2) ];

% Normalize affine approximators
b_under1 = C*b_under1 + d - (C*A_under1*C_inv)*d;
b_over1 = C*b_over1 + d - (C*A_over1*C_inv)*d;
A_under1 = C*A_under1/C;
A_over1 =  C*A_over1/C;

b_under2 = C*b_under2 + d - (C*A_under2*C_inv)*d;
b_over2 = C*b_over2 + d - (C*A_over2*C_inv)*d;
A_under2 = C*A_under2*C_inv;
A_over2 =  C*A_over2*C_inv;

R = zeros(2,8);
counter = 1;
for k = 1:4
    v = vertices(:,k);
    f_under = A_under1*v + b_under1;
    f_over = A_over1*v + b_over1;
    Aux_app = [f_under f_over];
    for p1 = 1:1 % 1 because f_under(1) = f_over(1) for any v
        for p2 = 1:2
            R(:,counter) = [Aux_app(1,p1);Aux_app(2,p2)];
            counter = counter + 1;
        end
    end
end

% indices_hull = convhull(R1')';
% Post_q_u_w1 = R1(:,indices_hull);

R2 = zeros(2,8);
counter = 1;
for k = 1:4
    v = vertices(:,k);
    f_under = A_under1*v + b_under1;
    f_over = A_over2*v + b_over2;
    Aux_app = [f_under f_over];
    for p1 = 1:1 % 1 because f_under(1) = f_over(1) for any v
        for p2 = 1:2
            R2(:,counter) = [Aux_app(1,p1);Aux_app(2,p2)];
            counter = counter + 1;
        end
    end
end

% indices_hull = convhull(R2')';
% Post_q_u_w2 = R2(:,indices_hull);

R3 = zeros(2,8);
counter = 1;
for k = 1:4
    v = vertices(:,k);
    f_under = A_under2*v + b_under2;
    f_over = A_over1*v + b_over1;
    Aux_app = [f_under f_over];
    for p1 = 1:1 % 1 because f_under(1) = f_over(1) for any v
        for p2 = 1:2
            R3(:,counter) = [Aux_app(1,p1);Aux_app(2,p2)];
            counter = counter + 1;
        end
    end
end

% indices_hull = convhull(R3')';
% Post_q_u_w3 = R3(:,indices_hull);

R4 = zeros(2,8);
counter = 1;
for k = 1:4
    v = vertices(:,k);
    f_under = A_under2*v + b_under2;
    f_over = A_over2*v + b_over2;
    Aux_app = [f_under f_over];
    for p1 = 1:1 % 1 because f_under(1) = f_over(1) for any v
        for p2 = 1:2
            R4(:,counter) = [Aux_app(1,p1);Aux_app(2,p2)];
            counter = counter + 1;
        end
    end
end

% indices_hull = convhull(R4')';
% Post_q_u_w4 = R4(:,indices_hull);

R = [R R2 R3 R4];
a_Post = min(R,[],2);
b_Post = max(R,[],2);


% (Only for debugging purposes)
if (debug)
    fprintf('Debug is on for reach sets. This might slow down computation...\n\n')
    delta_debug = max(b_s - a_s)/10;
    R_rectangle = [a_Post(1) a_Post(2); b_Post(1) a_Post(2); b_Post(1) b_Post(2); a_Post(1) b_Post(2); a_Post(1) a_Post(2)]';
    [X,Y] = meshgrid(a_s(1):delta_debug:b_s(1), a_s(2):delta_debug:b_s(2));
    X = X(:);
    Y = Y(:);
    f = zeros(2,length(X));
    for k = 1:length(X)
        x = [X(k) Y(k)]';
        f(:,k) = pendulum(x,u,w,original_sys,user_param);
    end

    in1 = inhull(f',R_rectangle',[],1e-4);
    if (ismember(in1,0))
        disp('Error generating R_app_q_a. Maybe increment tolerance')
        error_flag = 1;
    end
end

if (error_flag == 1)
    disp('Error computing Post. Maybe increment tolerance')
end


end