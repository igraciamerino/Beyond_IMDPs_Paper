function [Y_table,p_lower,p_upper] = obtain_lookup_table(Xi_hat,a,b,Nx_grid,Ny_grid,Nth_grid,D1,D2,epsilon_inflated)
% This function computes a lookup table with the values of the solution of
% the DR-uncertainty quantification problems for each point in a grid
% surrounding the rectangle q' (= [a b]) + D1 excluding
% the points in the rectangle [a b] + D2, and the coordinates of each point in
% that grid

% Obtain grid

[A_q1,b_q1] = matrices_polytope(a,b);
% [A_q1,b_q1] = vert2con(Xi_hat');

x_lower_left = a - D1;
x_upper_right = b + D1;

Deltax_grid = (x_upper_right(1) - x_lower_left(1))/Nx_grid;
Deltay_grid = (x_upper_right(2) - x_lower_left(2))/Ny_grid;
Deltath_grid = (x_upper_right(3) - x_lower_left(3))/Nth_grid;

x_grid_center = zeros(Nx_grid,1);
y_grid_center = zeros(Ny_grid,1);
th_grid_center = zeros(Nth_grid,1);
for i =1:Nx_grid
    x_grid_center(i) = x_lower_left(1) + Deltax_grid*(i-1/2);
end
for j =1:Ny_grid
    y_grid_center(j) = x_lower_left(2) + Deltay_grid*(j-1/2);
end
for k =1:Ny_grid
    th_grid_center(k) = x_lower_left(3) + Deltath_grid*(k-1/2);
end

[X_grid_center,Y_grid_center,TH_grid_center] = meshgrid(x_grid_center,y_grid_center,th_grid_center);
X_grid_center = X_grid_center(:);
Y_grid_center = Y_grid_center(:);
TH_grid_center = TH_grid_center(:);

% Substract hole
if (isempty(D2))
    N_grid = Nx_grid*Ny_grid*Nth_grid;
else
    %Compute vertices of hole
    vertices_hole = zeros(3,8);
    Aux_vertices_hole = [a+D2 b-D2];
    counter = 1;
    for p1 = 1:2
        for p2 = 1:2
            for p3 = 1:2
                vertices_hole(:,counter) = [Aux_vertices_hole(1,p1);Aux_vertices_hole(2,p2);Aux_vertices_hole(3,p3)];
                counter = counter + 1;
            end
        end
    end
    in1 = inhull([X_grid_center Y_grid_center TH_grid_center],vertices_hole');
    X_grid_center = X_grid_center(~in1);
    Y_grid_center = Y_grid_center(~in1);
    TH_grid_center = TH_grid_center(~in1);
    N_grid = sum(~in1);
end

% Solve problems for each point in the grid
Y_table = zeros(3,N_grid);
p_lower = zeros(1,N_grid);
p_upper = zeros(1,N_grid);
for k = 1:N_grid
    y = [X_grid_center(k) Y_grid_center(k) TH_grid_center(k)]';
    Y_table(:,k) = y;
    % Generate samples of x_star
    Xi_hat_plus_x = Xi_hat + y;
    [p_lower(k),p_upper(k)] = inner_problems(Xi_hat_plus_x,epsilon_inflated,A_q1,b_q1);

    % (Only for debugging purposes)
%     if (k == 10)
%         tic
%     end
%     if (k == 20)
%         fprintf('Estimated time required to obtain the lookup table = %d min. \n', toc*Nx_grid*Ny_grid/10/60)
%     end
end

end

