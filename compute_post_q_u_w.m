function [Post_q_u_w, flag] = compute_post_q_u_w(vertices_q,u,w,original_sys,user_param)
% Computes vertices of Post(q,u,w) given q (and vertices_q),
% u, w

debug = ~true;

flag = 0;
a_rectangle = vertices_q(:,1);
b_rectangle = vertices_q(:,4);
vertices_ordered = vertices_q(:,[1 3 4 2]);

Post_q_u_w = zeros(2,5);
for k = 1:4
    v = vertices_ordered(:,k);
    Post_q_u_w(:,k) = unicycle2D(v,u,w,original_sys,user_param);
end
v = vertices_ordered(:,1);
Post_q_u_w(:,end) = unicycle2D(v,u,w,original_sys,user_param);

% (Only for debugging purposes)
if (debug)
    [X, Y] = meshgrid(a_rectangle(1):0.005:b_rectangle(1), a_rectangle(2):0.005:b_rectangle(2));
    X = X(:);
    Y = Y(:);
    f = zeros(2,length(X));
    for k = 1:length(X)
        x = [X(k) Y(k)]';
        f(:,k) = unicycle2D(x,u,w,original_sys,user_param);
    end

    in1 = inhull(f',Post_q_u_w',[],1e-4);
    if (ismember(in1,0))
        disp('Error generating R_app_q_a. Maybe increment tolerance')
        flag = 1;
    end
    figure
    plot(f(1,:),f(2,:),'o')
    hold on
    plot(Post_q_u_w(1,:),Post_q_u_w(2,:),'o-')
end

if (flag == 1)
    disp('Error computing Post. Maybe increment tolerance')
end

end