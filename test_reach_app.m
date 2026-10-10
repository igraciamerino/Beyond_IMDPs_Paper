

%% Test Reach_app

%% Compute convex over-approximation of the reachable set for single (q,a)

q = 1;
a = 1;
flag = 1;

omega = Act(a);

original_sys.Reach_app_q_a = cell(user_param.Nq,user_param.Na);

a_rectangle = state_discretization.vertices_q{q}(:,1);
b_rectangle = state_discretization.vertices_q{q}(:,7);
[X,Y,TH] = meshgrid(a_rectangle(1):0.001:b_rectangle(1),a_rectangle(2):0.01:b_rectangle(2),a_rectangle(3):0.01:b_rectangle(3));
X = X(:);
Y = Y(:);
TH = TH(:);

original_sys.Reach_app_q_a{q,a} = zeros(3,64);
[ms_under,ms_over,ns_under,ns_over] = affine_under_and_overapproximate_sinusoidal(a_rectangle(3),b_rectangle(3),1); % Overapproximate sine
[mc_under,mc_over,nc_under,nc_over] = affine_under_and_overapproximate_sinusoidal(a_rectangle(3),b_rectangle(3),0); % Overapproximate cosine

A_under = [1 0 2*pi*user_param.Deltat*original_sys.vel*mc_under
           0 1 2*pi*user_param.Deltat*original_sys.vel*ms_under
           0 0 1];
b_under = [user_param.Deltat*original_sys.vel*nc_under-pi*user_param.Deltat*original_sys.vel*mc_under
           user_param.Deltat*original_sys.vel*ns_under-pi*user_param.Deltat*original_sys.vel*ms_under
           user_param.Deltat*omega/(2*pi)];

A_over = [1 0 2*pi*user_param.Deltat*original_sys.vel*mc_over
           0 1 2*pi*user_param.Deltat*original_sys.vel*ms_over
           0 0 1];
b_over = [user_param.Deltat*original_sys.vel*nc_over-pi*user_param.Deltat*original_sys.vel*mc_over
           user_param.Deltat*original_sys.vel*ns_over-pi*user_param.Deltat*original_sys.vel*ms_over
           user_param.Deltat*omega/(2*pi)];
counter = 1;
for k = 1:8
    v = state_discretization.vertices_q{q}(:,k);
    f_under = A_under*v + b_under;
    f_over = A_over*v + b_over;
    Aux_app = [f_under f_over];
    for p1 = 1:2
        for p2 = 1:2
            for p3 = 1:2
                original_sys.Reach_app_q_a{q,a}(:,counter) = [Aux_app(1,p1);Aux_app(2,p2);Aux_app(3,p3)];
                counter = counter + 1;
            end
        end
    end
end

f = zeros(3,length(X));
for k = 1:length(X)
    x = [X(k) Y(k) TH(k)]';
    f(:,k) = unicycle3D(x,omega,[0 0 0]',original_sys.vel);
end

in1 = inhull(f',original_sys.Reach_app_q_a{q,a}',[],1e-5);
if (min(in1) ~= 1)
    disp('Error generating R_app_q_a. Maybe increment tolerance')
end

% (Only for debugging purposes)
if (flag == 1)
    disp('Good approximation of the Reachable set')
end

%% Compute convex over-approximation of the reachable set forall (q,a)

original_sys.Reach_app_q_a = cell(user_param.Nq,user_param.Na);
flag = 1;
for q = 1:user_param.Nq
    a_rectangle = state_discretization.vertices_q{q}(:,1);
    b_rectangle = state_discretization.vertices_q{q}(:,7);
    [X,Y,TH] = meshgrid(a_rectangle(1):0.01:b_rectangle(1),a_rectangle(2):0.01:b_rectangle(2),a_rectangle(3):0.01:b_rectangle(3));
    X = X(:);
    Y = Y(:);
    TH = TH(:);

    for a = 1:4
        omega = Act(a);
        original_sys.Reach_app_q_a{q,a} = zeros(3,64);
        [ms_under,ms_over,ns_under,ns_over] = affine_under_and_overapproximate_sinusoidal(a_rectangle(3),b_rectangle(3),1); % Overapproximate sine
        [mc_under,mc_over,nc_under,nc_over] = affine_under_and_overapproximate_sinusoidal(a_rectangle(3),b_rectangle(3),0); % Overapproximate cosine

        A_under = [1 0 2*pi*user_param.Deltat*original_sys.vel*mc_under
            0 1 2*pi*user_param.Deltat*original_sys.vel*ms_under
            0 0 1];
        b_under = [user_param.Deltat*original_sys.vel*nc_under-pi*user_param.Deltat*original_sys.vel*mc_under
            user_param.Deltat*original_sys.vel*ns_under-pi*user_param.Deltat*original_sys.vel*ms_under
            user_param.Deltat*omega/(2*pi)];

        A_over = [1 0 2*pi*user_param.Deltat*original_sys.vel*mc_over
            0 1 2*pi*user_param.Deltat*original_sys.vel*ms_over
            0 0 1];
        b_over = [user_param.Deltat*original_sys.vel*nc_over-pi*user_param.Deltat*original_sys.vel*mc_over
            user_param.Deltat*original_sys.vel*ns_over-pi*user_param.Deltat*original_sys.vel*ms_over
            user_param.Deltat*omega/(2*pi)];
        counter = 1;
        for k = 1:8
            v = state_discretization.vertices_q{q}(:,k);
            f_under = A_under*v + b_under;
            f_over = A_over*v + b_over;
            Aux_app = [f_under f_over];
            for p1 = 1:2
                for p2 = 1:2
                    for p3 = 1:2
                        original_sys.Reach_app_q_a{q,a}(:,counter) = [Aux_app(1,p1);Aux_app(2,p2);Aux_app(3,p3)];
                        counter = counter + 1;
                    end
                end
            end
        end

        f = zeros(3,length(X));
        for k = 1:length(X)
            x = [X(k) Y(k) TH(k)]';
            f(:,k) = unicycle3D(x,omega,[0 0 0]',original_sys.vel);
        end


        in1 = inhull(f',original_sys.Reach_app_q_a{q,a}',[],1e-5);
        if (min(in1) ~= 1)
            disp('Error generating R_app_q_a. Maybe increment tolerance')
            flag = 0;
            break
        end
    end
    if (flag == 0)
        break
    end
end

% (Only for debugging purposes)
% if (flag == 1)
%     disp('Good approximation of the Reachable set')
% end