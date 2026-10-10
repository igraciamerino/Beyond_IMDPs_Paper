function intersects = check_intersection(Post_q_a_w, a, b)

% ADD FUNCTIONALITY FOR Post inside qp (if diam_inf(Post) bigger than diam_inf(qp), skip)
a_Post = min(Post_q_a_w,[],2); % For now w.o. precision
b_Post = max(Post_q_a_w,[],2);
if (max(b_Post - a_Post) > max(b - a)) % Post_q_u_w bigger than qp ([a b])

    % Check intersections with qp
    intersects = false;
    for k = 1:length(Post_q_a_w(1,:))-1
        u = Post_q_a_w(:,k);
        v = Post_q_a_w(:,k+1);
        if (v(1) ~= u(1)) % General case (u, v not on a vertical line)
            m = (v(2) - u(2))/(v(1) - u(1));
            n = u(2) - m*u(1);
            % Intersection with 1 (th_dot = a(2))
            x_intersection = (a(2) - n)/m;
            if ( (a(1) <= x_intersection) && (b(1) >= x_intersection) && ((u(2)-a(2))*(v(2)-a(2)) <= 0) )
                intersects = true;
                break
            end
            % Intersection with 3 (th_dot = b(2))
            x_intersection = (b(2) - n)/m;
            if ( (a(1) <= x_intersection) && (b(1) >= x_intersection) && ((u(2)-b(2))*(v(2)-b(2)) <= 0) )
                intersects = true;
                break
            end
            % Intersection with 4 (th = a(1))
            y_intersection = m*a(1) + n;
            if ( (a(2) <= y_intersection) && (b(2) >= y_intersection) && ((u(1)-a(1))*(v(1)-a(1)) <= 0) )
                intersects = true;
                break
            end
            % Intersection with 2 (th = b(1))
            y_intersection = m*b(1) + n;
            if ( (a(2) <= y_intersection) && (b(2) >= y_intersection) && ((u(1)-b(1))*(v(1)-b(1)) <= 0) )
                intersects = true;
                break
            end
        else % u, v are on a vertical line
            % Intersection with 1 (th_dot = a(2))
            x_intersection = v(1);
            if ( (a(1) <= x_intersection) && (b(1) >= x_intersection) && ((u(2)-a(2))*(v(2)-a(2)) <= 0) )
                intersects = true;
                break
            end
            % Intersection with 3 (th_dot = b(2))
            x_intersection = v(1);
            if ( (a(1) <= x_intersection) && (b(1) >= x_intersection) && ((u(2)-b(2))*(v(2)-b(2)) <= 0) )
                intersects = true;
                break
            end
            % No logic for intersection with 2 or 4 bc if that was the case, it
            % would have already been detected
        end
    end

    % Logic for the case that Post_q_a_w contains [a b]
    in1 = inhull([a(1) a(2); b(1) a(2); b(1) b(2); a(1) b(2)], Post_q_a_w');
    if (~any(ismember(in1,0))) % [a b] is inside
        intersects = true;
    end

else
    disp('You should be accounting for Post_q_u_w \subset [a b]. Add logic for subsumed here')

end