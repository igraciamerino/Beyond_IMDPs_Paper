function [dist] = calculate_distance2D(A_mat, b_mat, x, to_polytope)

% Euclidean distance (SQUARED!!!!!!!!!!)

aux_is_inside_polytope = (A_mat*x <= b_mat);

is_inside_polytope = ~ismember(0,aux_is_inside_polytope);

if (to_polytope == 1) && (is_inside_polytope == 1)
    
    dist = 0;
    
elseif (to_polytope == 1) && (is_inside_polytope == 0)

    Nxi = length(x);
    % Solve with if's (for rectangles in |R^2)

    ineq_x1 = aux_is_inside_polytope(1);
    ineq_x2 = aux_is_inside_polytope(2);
    ineq_y1 = aux_is_inside_polytope(3);
    ineq_y2 = aux_is_inside_polytope(4);
    ineq_th1 = 1;
    ineq_th2 = 1;

    a = -[b_mat(1),b_mat(3)];
    b = [b_mat(2),b_mat(4)];

    if (ineq_x1)
        if (ineq_x2) % Inside the range in x
            if (ineq_y1)
                if (ineq_y2) % Inside the range in y
                    if (ineq_th1) % Z15. Inside the range in th
                        %display('Z15')
                        dist = ( x(3) - b(3) )^2; % DIST TO PLANE TH2
                    else % Z13.
                        %display('Z13')
                        dist = ( a(3) - x(3) )^2; % DIST TO PLANE TH1
                    end
                else % To the right of plane Y2
                    if (ineq_th1)
                        if (ineq_th2) % Z17. Inside the range in th
                        %display('Z17')
                            dist = ( x(2) - b(2) )^2; % DIST TO PLANE Y2
                        else % Z18. To the right of plane TH2
                        %display('Z18')
                            dist = (x(2) - b(2))^2 + (x(3) - b(3))^2; % DIST TO LINE (4-8)
                        end
                    else % Z17. To the left of plane TH1
                        %display('Z17')
                        dist = (x(2) - b(2))^2 + (x(3) - a(3))^2; %DIST TO LINE (3-7)
                    end
                end
            else % To the left of plane Y1
                if (ineq_th1) % tO THE RIGHT OF PLANE TH1
                    if (ineq_th2) % Z11. Inside the range in th
                        dist = ( a(2) - x(2) )^2; % DIST TO PLANE Y1
                        %display('Z11')
                    else % Z12. To the left of plane TH2
                        %display('Z12')
                        dist = (x(2) - a(2))^2 + (x(3) - b(3))^2; % DIST TO LINE (2-6)
                    end
                else % Z10. To the left of plane TH1
                        %display('Z10')
                    dist = (x(2) - a(2))^2 + (x(3) - a(3))^2; % DIST TO LINE (1-5)
                end
            end

        else % To the right of the plane X2
            if (ineq_y1)
                if (ineq_y2) % Inside the range in y
                    if (ineq_th1)
                        if (ineq_th2) % Z23. Inside the range in th
                        %display('Z23')
                            dist = ( b(1) - x(1) )^2; % DIST TO PLANE X2
                        else % Z24. To the right of plane th2
                        %display('Z24')
                            dist = (x(1) - b(1))^2 + (x(3) - b(3))^2; % DIST TO LINE (6-8)
                        end
                    else % Z22.
                        %display('Z22')
                        dist = (x(1) - b(1))^2 + (x(3) - a(3))^2; % DIST TO LINE (5,7)
                    end
                else % To the right of plane Y2
                    if (ineq_th1)
                        if (ineq_th2) % Z26. Inside the range in th
                        %display('Z26')
                            dist = (x(1) - b(1))^2 + (x(2) - b(2))^2; % DIST TO LINE (7-8)
                        else % Z27. To the right of plane TH2
                        % display('Z27')
                            dist = (x(1) - b(1))^2 + (x(2) - b(2))^2 + (x(3) - b(3))^2; %DIST TO VERT 8
                        end
                    else % Z25. To the left of plane TH1
                        %display('Z25')
                        dist = (x(1) - b(1))^2 + (x(2) - b(2))^2 + (x(3) - a(3))^2; %DIST TO VERT 7
                    end
                end
            else % To the left of plane Y1
                if (ineq_th1) % tO THE RIGHT OF PLANE TH1
                    if (ineq_th2) % Z20. Inside the range in th
                        %display('Z20')
                        dist = (x(1) - b(1))^2 + (x(2) - a(2))^2; % DIST TO  LINE (5-6)
                    else % Z12. To the left of plane TH2
                        %display('Z12')
                        dist = (x(1) - b(1))^2 + (x(2) - a(2))^2 + (x(3) - b(3))^2; % DIST TO vert 6
                    end
                else % Z19. To the left of plane TH1
                        %display('Z19')
                    dist = (x(1) - b(1))^2 + (x(2) - a(2))^2 + (x(3) - a(3))^2; % DIST TO VERT 5
                end
            end
        end
    else
        if (ineq_y1)
            if (ineq_y2) % Inside the range in y
                if (ineq_th1)
                    if (ineq_th2) % Z5. Inside the range in th
                        %display('Z5')
                        dist = ( a(1) - x(1) )^2; % DIST TO PLANE X1
                    else % Z6. To the right of plane th2
                        %display('Z6')
                        dist = (x(1) - a(1))^2 + (x(3) - b(3))^2; % DIST TO LINE (2-4)
                    end
                else % Z4.
                        %display('Z4')
                    dist = (x(1) - a(1))^2 + (x(3) - a(3))^2; % DIST TO LINE (1-3)
                end
            else % To the right of plane Y2
                if (ineq_th1)
                    if (ineq_th2) % Z8. Inside the range in th
                        %display('Z8')
                        dist = (x(1) - a(1))^2 + (x(2) - b(2))^2; % DIST TO LINE (3-4)
                    else % Z9. To the right of plane TH2
                        %display('Z9')
                        dist = (x(1) - a(1))^2 + (x(2) - b(2))^2 + (x(3) - b(3))^2; %DIST TO VERT 4
                    end
                else % Z7. To the left of plane TH1
                        %display('Z7')
                    dist = (x(1) - a(1))^2 + (x(2) - b(2))^2 + (x(3) - a(3))^2; %DIST TO VERT 3
                end
            end
        else % To the left of plane Y1
            if (ineq_th1) % tO THE RIGHT OF PLANE TH1
                if (ineq_th2) % Z2. Inside the range in th
                        %display('Z2')
                    dist = (x(1) - a(1))^2 + (x(2) - a(2))^2; % DIST TO  LINE (1-2)
                else % Z3. To the left of plane TH2
                        %display('Z3')
                    dist = (x(1) - a(1))^2 + (x(2) - a(2))^2 + (x(3) - b(3))^2; % DIST TO vert 2
                end
            else % Z1. To the left of plane TH1
                        %display('Z1')
                dist = (x(1) - a(1))^2 + (x(2) - a(2))^2 + (x(3) - a(3))^2; % DIST TO VERT 1
            end
        end
    end





    
%     if ((A*xi <= b) == [0 1 1 1]')
%         
%         dist = (dot(A(1,:),xi)-b(1))/(norm(A(1,:),2));
%         
%     elseif ((A*xi <= b) == [1 0 1 1]')
%         
%         dist = (dot(A(2,:),xi)-b(2))/(norm(A(2,:),2));
%         
%     elseif ((A*xi <= b) == [1 1 0 1]')
%         
%         dist = (dot(A(3,:),xi)-b(3))/(norm(A(3,:),2));
%         
%     elseif ((A*xi <= b) == [1 1 1 0]')
%         
%         dist = (dot(A(4,:),xi)-b(4))/(norm(A(4,:),2));
%         
%     elseif ((A*xi <= b) == [0 0 1 1]')
%         
%         dist1  = (dot(A(1,:),xi)-b(1))/(norm(A(1,:),2));
%         dist2  = (dot(A(2,:),xi)-b(2))/(norm(A(2,:),2));
%         dist = sqrt(dist1^2 + dist2^2);
%         
%     elseif ((A*xi <= b) == [0 1 1 0]')
%         
%         dist1  = (dot(A(1,:),xi)-b(1))/(norm(A(1,:),2));
%         dist4  = (dot(A(4,:),xi)-b(4))/(norm(A(4,:),2));
%         dist = sqrt(dist1^2 + dist4^2);
%         
%     elseif ((A*xi <= b) == [1 0 0 1]')
%         
%         dist2  = (dot(A(2,:),xi)-b(2))/(norm(A(2,:),2));
%         dist3  = (dot(A(3,:),xi)-b(3))/(norm(A(3,:),2));
%         dist = sqrt(dist2^2 + dist3^2);
%         
%     elseif ((A*xi <= b) == [1 1 0 0]')
%         
%         dist3  = (dot(A(3,:),xi)-b(3))/(norm(A(3,:),2));
%         dist4  = (dot(A(4,:),xi)-b(4))/(norm(A(4,:),2));
%         dist = sqrt(dist3^2 + dist4^2);
%         
%     end
        
    % Solve with convex program
%     H = eye(Nxi);
%     f = -xi;
%     options = optimoptions('quadprog','Display','off');
%     [~,fval] = quadprog(H,f,A,b,[],[],[],[],[],options);
%     dist = sqrt(2*fval + xi'*xi);
    
elseif (to_polytope == 0) && (is_inside_polytope == 0)
    
    dist = 0;
    
elseif (to_polytope == 0) && (is_inside_polytope == 1)
    
    % Solve with if's (for rectangles in |R^3)
    
%     dist = zeros(6,1);
%     dist(1)  = ( dot(A_mat(1,:),x)-b_mat(1) )/(norm(A_mat(1,:),2));
%     dist(2)  = ( dot(A_mat(2,:),x)-b_mat(2) )/(norm(A_mat(2,:),2));
%     dist(3)  = ( dot(A_mat(3,:),x)-b_mat(3) )/(norm(A_mat(3,:),2));
%     dist(4)  = ( dot(A_mat(4,:),x)-b_mat(4) )/(norm(A_mat(4,:),2));
%     dist(5)  = ( dot(A_mat(5,:),x)-b_mat(5) )/(norm(A_mat(5,:),2));
%     dist(6)  = ( dot(A_mat(6,:),x)-b_mat(6) )/(norm(A_mat(6,:),2));
%     dist = min(dist.^2);

    a = -[b_mat(1),b_mat(3)];
    b = [b_mat(2),b_mat(4)];
    dist(1)  = x(1) - a(1);
    dist(2)  = x(1) - b(1);
    dist(3)  = x(2) - a(2);
    dist(4)  = x(2) - b(2);
%     dist(5)  = x(3) - a(3);
%     dist(6)  = x(3) - b(3);
    dist = min(dist.^2);
    
    % Solve with convex program
%     Nconstraints = length(b);
%     Nxi = length(xi);
%     H = eye(Nxi);
%     f = -xi;
%     fval = zeros(1,Nconstraints);
%     options = optimoptions('quadprog','Display','off');
%     for k=1:Nconstraints
%         [~,fval(k)] = quadprog(H,f,-A(k,:),-b(k),[],[],[],[],[],options);
%     end
%     dist = sqrt(2*min(fval) + xi'*xi);
    
end

end