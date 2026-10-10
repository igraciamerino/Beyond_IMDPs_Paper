function intersects = check_intersection_pol_cube(Post_q_a_w, a, b)

heuristic = true;
intersects = false;

a_Post = min(Post_q_a_w,[],2); % For now w.o. precision
b_Post = max(Post_q_a_w,[],2);
if (max(b_Post - a_Post) > max(b - a)) % Post_q_u_w bigger than qp ([a b])

    % Check intersections with qp

    % First, heuristic: check if any point of Post_q_a_w is inside qp
    if (heuristic)
        [A_qp,b_qp] = matrices_polytope(a,b);
        for k = 1:length(Post_q_a_w(1,:))
            u = Post_q_a_w(:,k);
            if ( ~any(A_qp*u > b_qp) ) % Vertex inside qp
                intersects = true;
                return;
            end
        end
    end
    % 1.5: check if any vertex of qp is inside Post_q_a_w
    in1 = inhull([a(1) a(2) a(3); b(1) a(2) a(3); b(1) b(2) a(3); a(1) b(2) a(3); a(1) a(2) b(3); b(1) a(2) b(3); b(1) b(2) b(3); a(1) b(2) b(3)], Post_q_a_w');
    if (~any(ismember(in1,0))) % [a b] is inside
        intersects = true;
        return;
    end

    % Second, intersections between lines of polytope and faces of qp
    indices_triang = convhull(Post_q_a_w');
    for ind_triang = 1:size(indices_triang,1)
        triang = Post_q_a_w(:,indices_triang(ind_triang,:));
        triang_ext = [triang triang(:,1)];
        for edge_triang = 1:3
            u = triang_ext(:, edge_triang);
            v = triang_ext(:, edge_triang + 1);

            % Facet x = a(1);
            if ( (u(1) - a(1))*(v(1) - a(1)) <= 0 ) % u, v are on different sides of the plane x = a(1). Else, can't intersect
                % Compute intersection between line and plane
                l = u-v;
                normal = [1; 0; 0];
                l_dot_n = l'*normal;
                if (l_dot_n ~= 0) % Line is not contained in plane
                    d = (a - u)'*normal/l_dot_n;
                    p = u + d*l;
                    if ( (p(2) >= a(2)) && (p(2) <= b(2)) && (p(3) >= a(3)) && (p(3) <= b(3)) ) % p is within limits of facet
                        intersects = true;
                        return;
                    end
                else % Line is contained in plane
                    % Check if u \in facet
                    if ( (u(2) >= a(2)) && (u(2) <= b(2)) && (u(3) >= a(3)) && (u(3) <= b(3)) ) % u is within limits of facet
                        intersects = true;
                        return;
                    end
                    % Check if v \in facet
                    if ( (v(2) >= a(2)) && (v(2) <= b(2)) && (v(3) >= a(3)) && (v(3) <= b(3)) ) % v is within limits of facet
                        intersects = true;
                        return;
                    end
                end
            end

            % Facet x = b(1);
            if ( (u(1) - b(1))*(v(1) - b(1)) <= 0 ) % u, v are on different sides of the plane x = b(1). Else, can't intersect
                % Compute intersection between line and plane
                l = u-v;
                normal = [1; 0; 0];
                l_dot_n = l'*normal;
                if (l_dot_n ~= 0) % Line is not contained in plane
                    d = (b - u)'*normal/l_dot_n;
                    p = u + d*l;
                    if ( (p(2) >= a(2)) && (p(2) <= b(2)) && (p(3) >= a(3)) && (p(3) <= b(3)) ) % p is within limits of facet
                        intersects = true;
                        return;
                    end
                else % Line is contained in plane
                    % Check if u \in facet
                    if ( (u(2) >= a(2)) && (u(2) <= b(2)) && (u(3) >= a(3)) && (u(3) <= b(3)) ) % u is within limits of facet
                        intersects = true;
                        return;
                    end
                    % Check if v \in facet
                    if ( (v(2) >= a(2)) && (v(2) <= b(2)) && (v(3) >= a(3)) && (v(3) <= b(3)) ) % v is within limits of facet
                        intersects = true;
                        return;
                    end
                end
            end

            % Facet y = a(2);
            if ( (u(2) - a(2))*(v(2) - a(2)) <= 0 ) % u, v are on different sides of the plane x = a(2). Else, can't intersect
                % Compute intersection between line and plane
                l = u-v;
                normal = [0; 1; 0];
                l_dot_n = l'*normal;
                if (l_dot_n ~= 0) % Line is not contained in plane
                    d = (a - u)'*normal/l_dot_n;
                    p = u + d*l;
                    if ( (p(1) >= a(1)) && (p(1) <= b(1)) && (p(3) >= a(3)) && (p(3) <= b(3)) ) % p is within limits of facet
                        intersects = true;
                        return;
                    end
                else % Line is contained in plane
                    % Check if u \in facet
                    if ( (u(1) >= a(1)) && (u(1) <= b(1)) && (u(3) >= a(3)) && (u(3) <= b(3)) ) % u is within limits of facet
                        intersects = true;
                        return;
                    end
                    % Check if v \in facet
                    if ( (v(1) >= a(1)) && (v(1) <= b(1)) && (v(3) >= a(3)) && (v(3) <= b(3)) ) % v is within limits of facet
                        intersects = true;
                        return;
                    end
                end
            end

            % Facet y = b(2);
            if ( (u(2) - b(2))*(v(2) - b(2)) <= 0 ) % u, v are on different sides of the plane x = b(1). Else, can't intersect
                % Compute intersection between line and plane
                l = u-v;
                normal = [0; 1; 0];
                l_dot_n = l'*normal;
                if (l_dot_n ~= 0) % Line is not contained in plane
                    d = (b - u)'*normal/l_dot_n;
                    p = u + d*l;
                    if ( (p(1) >= a(1)) && (p(1) <= b(1)) && (p(3) >= a(3)) && (p(3) <= b(3)) ) % p is within limits of facet
                        intersects = true;
                        return;
                    end
                else % Line is contained in plane
                    % Check if u \in facet
                    if ( (u(1) >= a(1)) && (u(1) <= b(1)) && (u(3) >= a(3)) && (u(3) <= b(3)) ) % u is within limits of facet
                        intersects = true;
                        return;
                    end
                    % Check if v \in facet
                    if ( (v(1) >= a(1)) && (v(1) <= b(1)) && (v(3) >= a(3)) && (v(3) <= b(3)) ) % v is within limits of facet
                        intersects = true;
                        return;
                    end
                end
            end

            % Facet th = a(3);
            if ( (u(3) - a(3))*(v(3) - a(3)) <= 0 ) % u, v are on different sides of the plane x = a(2). Else, can't intersect
                % Compute intersection between line and plane
                l = u-v;
                normal = [0; 0; 1];
                l_dot_n = l'*normal;
                if (l_dot_n ~= 0) % Line is not contained in plane
                    d = (a - u)'*normal/l_dot_n;
                    p = u + d*l;
                    if ( (p(1) >= a(1)) && (p(1) <= b(1)) && (p(2) >= a(2)) && (p(2) <= b(2)) ) % p is within limits of facet
                        intersects = true;
                        return;
                    end
                else % Line is contained in plane
                    % Check if u \in facet
                    if ( (u(1) >= a(1)) && (u(1) <= b(1)) && (u(2) >= a(2)) && (u(2) <= b(2)) ) % u is within limits of facet
                        intersects = true;
                        return;
                    end
                    % Check if v \in facet
                    if ( (v(1) >= a(1)) && (v(1) <= b(1)) && (v(2) >= a(2)) && (v(2) <= b(2)) ) % v is within limits of facet
                        intersects = true;
                        return;
                    end
                end
            end

            % Facet th = b(3);
            if ( (u(3) - b(3))*(v(3) - b(3)) <= 0 ) % u, v are on different sides of the plane x = b(1). Else, can't intersect
                % Compute intersection between line and plane
                l = u-v;
                normal = [0; 0; 1];
                l_dot_n = l'*normal;
                if (l_dot_n ~= 0) % Line is not contained in plane
                    d = (b - u)'*normal/l_dot_n;
                    p = u + d*l;
                    if ( (p(1) >= a(1)) && (p(1) <= b(1)) && (p(2) >= a(2)) && (p(2) <= b(2)) ) % p is within limits of facet
                        intersects = true;
                        return;
                    end
                else % Line is contained in plane
                    % Check if u \in facet
                    if ( (u(1) >= a(1)) && (u(1) <= b(1)) && (u(2) >= a(2)) && (u(2) <= b(2)) ) % u is within limits of facet
                        intersects = true;
                        return;
                    end
                    % Check if v \in facet
                    if ( (v(1) >= a(1)) && (v(1) <= b(1)) && (v(2) >= a(2)) && (v(2) <= b(2)) ) % v is within limits of facet
                        intersects = true;
                        return;
                    end
                end
            end

        end
        
    end

    % Third, intersections between each edge of qp and each triangle of the
    % convex hull of the polytope (normal'*[x; y; th] + d = 0)

    for ind_triang = 1:size(indices_triang,1)
        triang = Post_q_a_w(:,indices_triang(ind_triang,:));
        v1 = triang(:,1);
        v2 = triang(:,2);
        v3 = triang(:,3);
        l1 = v2 - v1;
        l2 = v3 - v1;
        normal = [l1(2)*l2(3) - l1(3)*l2(2); l1(1)*l2(3) - l1(3)*l2(1); l1(1)*l2(2) - l1(2)*l2(1)]; % Basically cross(l1,l2)
        d = -normal'*v1;
        
        % Intersection with (x,y) = (0,0)
        u = [0; 0; 0];
        v = [0; 0; 1];
        if ( (normal'*u + d)*(normal'*v + d) <= 0 ) % u, v are on different sides of the plane defined by the triangle (or at least one is contained). Else, can't intersect
            % Compute intersection between line and plane
            if ( normal(3) ~= 0 ) % Line is not contained in plane (and plane is not vertical)
                p = [0; 0; -d/normal(3)];
                if ( (p-u)'*(p-v) <= 0 ) % p is on the segment of qp
                    if (is_point_on_triangle_fcn(v1, v2, v3, l1, l2, p)) % p is inside the triangle
                        intersects = true;
                        return;
                    end
                end
            elseif ( is_point_on_triangle_fcn(v1, v2, v3, l1, l2, u) || is_point_on_triangle_fcn(v1, v2, v3, l1, l2, v) ) % Line is contained in plane and u or v are on the triangle (only need to consider this case because the previous functionaity would have dettected an intersection)
                intersects = true;
                return;
            end
        end

        % Intersection with (x,y) = (1,0)
        u = [1; 0; 0];
        v = [1; 0; 1];
        if ( (normal'*u + d)*(normal'*v + d) <= 0 ) % u, v are on different sides of the plane defined by the triangle (or at least one is contained). Else, can't intersect
            % Compute intersection between line and plane
            if ( normal(3) ~= 0 ) % Line is not contained in plane (and plane is not vertical)
                p = [1; 0; -(d + normal(1))/normal(3)];
                if ( (p-u)'*(p-v) <= 0 ) % p is on the segment of qp
                    if (is_point_on_triangle_fcn(v1, v2, v3, l1, l2, p)) % p is inside the triangle
                        intersects = true;
                        return;
                    end
                end
            elseif ( is_point_on_triangle_fcn(v1, v2, v3, l1, l2, u) || is_point_on_triangle_fcn(v1, v2, v3, l1, l2, v) ) % Line is contained in plane and u or v are on the triangle (only need to consider this case because the previous functionaity would have dettected an intersection)
                intersects = true;
                return;
            end
        end

        % Intersection with (x,y) = (0,1)
        u = [0; 1; 0];
        v = [0; 1; 1];
        if ( (normal'*u + d)*(normal'*v + d) <= 0 ) % u, v are on different sides of the plane defined by the triangle (or at least one is contained). Else, can't intersect
            % Compute intersection between line and plane
            if ( normal(3) ~= 0 ) % Line is not contained in plane (and plane is not vertical)
                p = [0; 1; -(d + normal(2))/normal(3)];
                if ( (p-u)'*(p-v) <= 0 ) % p is on the segment of qp
                    if (is_point_on_triangle_fcn(v1, v2, v3, l1, l2, p)) % p is inside the triangle
                        intersects = true;
                        return;
                    end
                end
            elseif ( is_point_on_triangle_fcn(v1, v2, v3, l1, l2, u) || is_point_on_triangle_fcn(v1, v2, v3, l1, l2, v) ) % Line is contained in plane and u or v are on the triangle (only need to consider this case because the previous functionaity would have dettected an intersection)
                intersects = true;
                return;
            end
        end

        % Intersection with (x,y) = (1,1)
        u = [1; 1; 0];
        v = [1; 1; 1];
        if ( (normal'*u + d)*(normal'*v + d) <= 0 ) % u, v are on different sides of the plane defined by the triangle (or at least one is contained). Else, can't intersect
            % Compute intersection between line and plane
            if ( normal(3) ~= 0 ) % Line is not contained in plane (and plane is not vertical)
                p = [1; 1; -(d + normal(1) + normal(2))/normal(3)];
                if ( (p-u)'*(p-v) <= 0 ) % p is on the segment of qp
                    if (is_point_on_triangle_fcn(v1, v2, v3, l1, l2, p)) % p is inside the triangle
                        intersects = true;
                        return;
                    end
                end
            elseif ( is_point_on_triangle_fcn(v1, v2, v3, l1, l2, u) || is_point_on_triangle_fcn(v1, v2, v3, l1, l2, v) ) % Line is contained in plane and u or v are on the triangle (only need to consider this case because the previous functionaity would have dettected an intersection)
                intersects = true;
                return;
            end
        end

        % Intersection with (x,th) = (0,0)
        u = [0; 0; 0];
        v = [0; 1; 0];
        if ( (normal'*u + d)*(normal'*v + d) <= 0 ) % u, v are on different sides of the plane defined by the triangle (or at least one is contained). Else, can't intersect
            % Compute intersection between line and plane
            if ( normal(2) ~= 0 ) % Line is not contained in plane (and plane is not vertical)
                p = [0; -d/normal(2); 0];
                if ( (p-u)'*(p-v) <= 0 ) % p is on the segment of qp
                    if (is_point_on_triangle_fcn(v1, v2, v3, l1, l2, p)) % p is inside the triangle
                        intersects = true;
                        return;
                    end
                end
            elseif ( is_point_on_triangle_fcn(v1, v2, v3, l1, l2, u) || is_point_on_triangle_fcn(v1, v2, v3, l1, l2, v) ) % Line is contained in plane and u or v are on the triangle (only need to consider this case because the previous functionaity would have dettected an intersection)
                intersects = true;
                return;
            end
        end

        % Intersection with (x,th) = (1,0)
        u = [1; 0; 0];
        v = [1; 1; 0];
        if ( (normal'*u + d)*(normal'*v + d) <= 0 ) % u, v are on different sides of the plane defined by the triangle (or at least one is contained). Else, can't intersect
            % Compute intersection between line and plane
            if ( normal(2) ~= 0 ) % Line is not contained in plane (and plane is not vertical)
                p = [1; -(d + normal(1))/normal(2); 0];
                if ( (p-u)'*(p-v) <= 0 ) % p is on the segment of qp
                    if (is_point_on_triangle_fcn(v1, v2, v3, l1, l2, p)) % p is inside the triangle
                        intersects = true;
                        return;
                    end
                end
            elseif ( is_point_on_triangle_fcn(v1, v2, v3, l1, l2, u) || is_point_on_triangle_fcn(v1, v2, v3, l1, l2, v) ) % Line is contained in plane and u or v are on the triangle (only need to consider this case because the previous functionaity would have dettected an intersection)
                intersects = true;
                return;
            end
        end

        % Intersection with (x,th) = (0,1)
        u = [0; 0; 1];
        v = [0; 1; 1];
        if ( (normal'*u + d)*(normal'*v + d) <= 0 ) % u, v are on different sides of the plane defined by the triangle (or at least one is contained). Else, can't intersect
            % Compute intersection between line and plane
            if ( normal(2) ~= 0 ) % Line is not contained in plane (and plane is not vertical)
                p = [0; 1; -(d + normal(3))/normal(2)];
                if ( (p-u)'*(p-v) <= 0 ) % p is on the segment of qp
                    if (is_point_on_triangle_fcn(v1, v2, v3, l1, l2, p)) % p is inside the triangle
                        intersects = true;
                        return;
                    end
                end
            elseif ( is_point_on_triangle_fcn(v1, v2, v3, l1, l2, u) || is_point_on_triangle_fcn(v1, v2, v3, l1, l2, v) ) % Line is contained in plane and u or v are on the triangle (only need to consider this case because the previous functionaity would have dettected an intersection)
                intersects = true;
                return;
            end
        end

        % Intersection with (x,th) = (1,1)
        u = [1; 0; 1];
        v = [1; 1; 1];
        if ( (normal'*u + d)*(normal'*v + d) <= 0 ) % u, v are on different sides of the plane defined by the triangle (or at least one is contained). Else, can't intersect
            % Compute intersection between line and plane
            if ( normal(2) ~= 0 ) % Line is not contained in plane (and plane is not vertical)
                p = [1; 1; -(d + normal(1) + normal(3))/normal(2)];
                if ( (p-u)'*(p-v) <= 0 ) % p is on the segment of qp
                    if (is_point_on_triangle_fcn(v1, v2, v3, l1, l2, p)) % p is inside the triangle
                        intersects = true;
                        return;
                    end
                end
            elseif ( is_point_on_triangle_fcn(v1, v2, v3, l1, l2, u) || is_point_on_triangle_fcn(v1, v2, v3, l1, l2, v) ) % Line is contained in plane and u or v are on the triangle (only need to consider this case because the previous functionaity would have dettected an intersection)
                intersects = true;
                return;
            end
        end

        % Intersection with (y,th) = (0,0)
        u = [0; 0; 0];
        v = [1; 0; 0];
        if ( (normal'*u + d)*(normal'*v + d) <= 0 ) % u, v are on different sides of the plane defined by the triangle (or at least one is contained). Else, can't intersect
            % Compute intersection between line and plane
            if ( normal(1) ~= 0 ) % Line is not contained in plane (and plane is not vertical)
                p = [0; -d/normal(1); 0];
                if ( (p-u)'*(p-v) <= 0 ) % p is on the segment of qp
                    if (is_point_on_triangle_fcn(v1, v2, v3, l1, l2, p)) % p is inside the triangle
                        intersects = true;
                        return;
                    end
                end
            elseif ( is_point_on_triangle_fcn(v1, v2, v3, l1, l2, u) || is_point_on_triangle_fcn(v1, v2, v3, l1, l2, v) ) % Line is contained in plane and u or v are on the triangle (only need to consider this case because the previous functionaity would have dettected an intersection)
                intersects = true;
                return;
            end
        end

        % Intersection with (y,th) = (1,0)
        u = [0; 1; 0];
        v = [1; 1; 0];
        if ( (normal'*u + d)*(normal'*v + d) <= 0 ) % u, v are on different sides of the plane defined by the triangle (or at least one is contained). Else, can't intersect
            % Compute intersection between line and plane
            if ( normal(1) ~= 0 ) % Line is not contained in plane (and plane is not vertical)
                p = [1; -(d + normal(2))/normal(1); 0];
                if ( (p-u)'*(p-v) <= 0 ) % p is on the segment of qp
                    if (is_point_on_triangle_fcn(v1, v2, v3, l1, l2, p)) % p is inside the triangle
                        intersects = true;
                        return;
                    end
                end
            elseif ( is_point_on_triangle_fcn(v1, v2, v3, l1, l2, u) || is_point_on_triangle_fcn(v1, v2, v3, l1, l2, v) ) % Line is contained in plane and u or v are on the triangle (only need to consider this case because the previous functionaity would have dettected an intersection)
                intersects = true;
                return;
            end
        end

        % Intersection with (y,th) = (0,1)
        u = [0; 0; 1];
        v = [1; 0; 1];
        if ( (normal'*u + d)*(normal'*v + d) <= 0 ) % u, v are on different sides of the plane defined by the triangle (or at least one is contained). Else, can't intersect
            % Compute intersection between line and plane
            if ( normal(1) ~= 0 ) % Line is not contained in plane (and plane is not vertical)
                p = [0; 1; -(d + normal(3))/normal(1)];
                if ( (p-u)'*(p-v) <= 0 ) % p is on the segment of qp
                    if (is_point_on_triangle_fcn(v1, v2, v3, l1, l2, p)) % p is inside the triangle
                        intersects = true;
                        return;
                    end
                end
            elseif ( is_point_on_triangle_fcn(v1, v2, v3, l1, l2, u) || is_point_on_triangle_fcn(v1, v2, v3, l1, l2, v) ) % Line is contained in plane and u or v are on the triangle (only need to consider this case because the previous functionaity would have dettected an intersection)
                intersects = true;
                return;
            end
        end

        % Intersection with (y,th) = (1,1)
        u = [0; 1; 1];
        v = [1; 1; 1];
        if ( (normal'*u + d)*(normal'*v + d) <= 0 ) % u, v are on different sides of the plane defined by the triangle (or at least one is contained). Else, can't intersect
            % Compute intersection between line and plane
            if ( normal(1) ~= 0 ) % Line is not contained in plane (and plane is not vertical)
                p = [1; 1; -(d + normal(2) + normal(3))/normal(1)];
                if ( (p-u)'*(p-v) <= 0 ) % p is on the segment of qp
                    if (is_point_on_triangle_fcn(v1, v2, v3, l1, l2, p)) % p is inside the triangle
                        intersects = true;
                        return;
                    end
                end
            elseif ( is_point_on_triangle_fcn(v1, v2, v3, l1, l2, u) || is_point_on_triangle_fcn(v1, v2, v3, l1, l2, v) ) % Line is contained in plane and u or v are on the triangle (only need to consider this case because the previous functionaity would have dettected an intersection)
                intersects = true;
                return;
            end
        end

    end

else
    disp('You should be accounting for Post_q_u_w \subset [a b]. Add logic for subsumed here')
end