function [state_discretization, user_param] = discretize_state_space_overlapping_cluster(original_sys, user_param)
% Takes a rectangle in |R^n and discretizes it into Nx*Ny rectangles.

user_param.Ns = user_param.Nx{1}*user_param.Nx{2};

% First fine discretization

state_space_lower_left_corner = original_sys.state_space_lower_left_corner;
state_space_upper_right_corner = original_sys.state_space_upper_right_corner;
Nx = user_param.Nx{1};
Ny = user_param.Nx{2};

x_grid = linspace(state_space_lower_left_corner(1), state_space_upper_right_corner(1), Nx+1);
y_grid = linspace(state_space_lower_left_corner(2), state_space_upper_right_corner(2), Ny+1);
Deltax = (state_space_upper_right_corner(1) - state_space_lower_left_corner(1))/Nx;
Deltay = (state_space_upper_right_corner(2) - state_space_lower_left_corner(2))/Ny;
[X_grid, Y_grid] = meshgrid(x_grid, y_grid);
X_grid = X_grid(:);
Y_grid = Y_grid(:);

X_center = zeros(Nx,Ny);
Y_center = zeros(Nx,Ny);

for i = 1:Nx
    for j = 1:Ny
        X_center(i,j) = (x_grid(i+1) + x_grid(i))/2;
        Y_center(i,j) = (y_grid(j+1) + y_grid(j))/2;
    end
end

X_center = X_center(:);
Y_center = Y_center(:);

state_discretization.X_grid{1} = X_grid;
state_discretization.X_grid{2} = Y_grid;

state_discretization.x_grid{1} = x_grid;
state_discretization.x_grid{2} = y_grid;

state_discretization.X_center{1} = X_center;
state_discretization.X_center{2} = Y_center;

state_discretization.Deltax{1} = Deltax;
state_discretization.Deltax{2} = Deltay;

state_discretization.vertices_s = cell(Nx*Ny,1);

for q = 1:Nx*Ny
    state_discretization.vertices_s{q} = zeros(2,4);
    Aux_vert = [X_center(q)-Deltax/2 X_center(q)+Deltax/2
                Y_center(q)-Deltay/2 Y_center(q)+Deltay/2];
    counter = 1;
    for px = 1:2
        for py = 1:2
            state_discretization.vertices_s{q}(:,counter) = [Aux_vert(1,px); Aux_vert(2,py)];
            counter = counter + 1;
        end
    end
end

% Next coarse discretization

state_space_lower_left_corner = original_sys.state_space_lower_left_corner;
state_space_upper_right_corner = original_sys.state_space_upper_right_corner;
user_param.Nx_coarse{1} = user_param.Nx{1} - user_param.coarse_Deltax_ratio{1} + 1;
user_param.Nx_coarse{2} = user_param.Nx{2} - user_param.coarse_Deltax_ratio{2} + 1;
user_param.Nq = user_param.Nx_coarse{1}*user_param.Nx_coarse{2};

Deltax = (state_space_upper_right_corner(1) - state_space_lower_left_corner(1))/Nx;
Deltay = (state_space_upper_right_corner(2) - state_space_lower_left_corner(2))/Ny;


Nx_coarse = user_param.Nx_coarse{1};
Ny_coarse = user_param.Nx_coarse{2};

X_center = zeros(Nx_coarse, Ny_coarse);
Y_center = zeros(Nx_coarse, Ny_coarse);
aux_center_coarse_x = user_param.coarse_Deltax_ratio{1}/2 - 1;
aux_cnter_coarse_y = user_param.coarse_Deltax_ratio{2}/2 - 1;
for i = 1:Nx_coarse
    for j = 1:Ny_coarse
        X_center(i,j) = (i + aux_center_coarse_x)*Deltax;
        Y_center(i,j) = (j + aux_cnter_coarse_y)*Deltay;
    end
end

X_center = X_center(:);
Y_center = Y_center(:);

state_discretization.X_center_coarse{1} = X_center;
state_discretization.X_center_coarse{2} = Y_center;

state_discretization.Deltax_coarse{1} = user_param.coarse_Deltax_ratio{1}*Deltax;
state_discretization.Deltax_coarse{2} = user_param.coarse_Deltax_ratio{2}*Deltay;

state_discretization.vertices_q = cell((Nx_coarse)*(Ny_coarse), 1);

for q = 1:(Nx_coarse)*(Ny_coarse)
    state_discretization.vertices_q{q} = zeros(2,4);
    Aux_vert = [X_center(q)-state_discretization.Deltax_coarse{1}/2 X_center(q)+state_discretization.Deltax_coarse{1}/2
                Y_center(q)-state_discretization.Deltax_coarse{2}/2 Y_center(q)+state_discretization.Deltax_coarse{2}/2];
    counter = 1;
    for px = 1:2
        for py = 1:2
            state_discretization.vertices_q{q}(:,counter) = [Aux_vert(1,px); Aux_vert(2,py)];
            counter = counter + 1;
        end
    end
end

end

