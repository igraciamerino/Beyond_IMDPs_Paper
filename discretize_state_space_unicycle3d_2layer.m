function [state_discretization, user_param] = discretize_state_space_unicycle3d_2layer(original_sys, user_param)
% Takes a rectangle in |R^n and discretizes it into Nx*Ny rectangles.

user_param.Ns = user_param.Nx*user_param.Ny*user_param.Nz;

state_space_lower_left_corner = original_sys.state_space_lower_left_corner;
state_space_upper_right_corner = original_sys.state_space_upper_right_corner;
Nx = user_param.Nx;
Ny = user_param.Ny;
Nz = user_param.Nz;

x_grid = linspace(state_space_lower_left_corner(1), state_space_upper_right_corner(1), Nx+1);
y_grid = linspace(state_space_lower_left_corner(2), state_space_upper_right_corner(2), Ny+1);
z_grid = linspace(state_space_lower_left_corner(3), state_space_upper_right_corner(3), Nz+1);
Deltax = (state_space_upper_right_corner(1) - state_space_lower_left_corner(1))/Nx;
Deltay = (state_space_upper_right_corner(2) - state_space_lower_left_corner(2))/Ny;
Deltaz = (state_space_upper_right_corner(3) - state_space_lower_left_corner(3))/Nz;
[X_grid, Y_grid, Z_grid] = meshgrid(x_grid, y_grid, z_grid);
X_grid = X_grid(:);
Y_grid = Y_grid(:);
Z_grid = Z_grid(:);

X_center = zeros(Nx,Ny,Nz);
Y_center = zeros(Nx,Ny,Nz);
Z_center = zeros(Nx,Ny,Nz);

for i = 1:Nx
    for j = 1:Ny
        for k = 1:Nz
            X_center(i,j,k) = (x_grid(i+1) + x_grid(i))/2;
            Y_center(i,j,k) = (y_grid(j+1) + y_grid(j))/2;
            Z_center(i,j,k) = (z_grid(k+1) + z_grid(k))/2;
        end
    end
end

X_center = X_center(:);
Y_center = Y_center(:);
Z_center = Z_center(:);

state_discretization.X_grid = X_grid;
state_discretization.Y_grid = Y_grid;
state_discretization.Z_grid = Z_grid;

state_discretization.x_grid = x_grid;
state_discretization.y_grid = y_grid;
state_discretization.z_grid = z_grid;

state_discretization.X_center = X_center;
state_discretization.Y_center = Y_center;
state_discretization.Z_center = Z_center;

state_discretization.Deltax = Deltax;
state_discretization.Deltay = Deltay;
state_discretization.Deltaz = Deltaz;

state_discretization.vertices_s = cell(Nx*Ny*Nz,1);

for q = 1:Nx*Ny*Nz
    state_discretization.vertices_s{q} = zeros(3,8);
    Aux_vert = [X_center(q)-Deltax/2 X_center(q)+Deltax/2
                Y_center(q)-Deltay/2 Y_center(q)+Deltay/2
                Z_center(q)-Deltaz/2 Z_center(q)+Deltaz/2];
    counter = 1;
    for px = 1:2
        for py = 1:2
            for pth = 1:2
                state_discretization.vertices_s{q}(:,counter) = [Aux_vert(1,px); Aux_vert(2,py); Aux_vert(3,pth)];
                counter = counter + 1;
            end
        end
    end
end

% Coarse

user_param.Nx_coarse = user_param.Nx/user_param.coarse_Deltax_ratio;
user_param.Ny_coarse = user_param.Ny/user_param.coarse_Deltay_ratio;
user_param.Nz_coarse = user_param.Nz/user_param.coarse_Deltaz_ratio;
user_param.Nq = user_param.Nx_coarse*user_param.Ny_coarse*user_param.Nz_coarse;

Nx = user_param.Nx_coarse;
Ny = user_param.Ny_coarse;
Nz = user_param.Nz_coarse;

x_grid = linspace(state_space_lower_left_corner(1), state_space_upper_right_corner(1), Nx+1);
y_grid = linspace(state_space_lower_left_corner(2), state_space_upper_right_corner(2), Ny+1);
z_grid = linspace(state_space_lower_left_corner(3), state_space_upper_right_corner(3), Nz+1);
Deltax = (state_space_upper_right_corner(1) - state_space_lower_left_corner(1))/Nx;
Deltay = (state_space_upper_right_corner(2) - state_space_lower_left_corner(2))/Ny;
Deltaz = (state_space_upper_right_corner(3) - state_space_lower_left_corner(3))/Nz;
[X_grid, Y_grid, Z_grid] = meshgrid(x_grid, y_grid, z_grid);
X_grid = X_grid(:);
Y_grid = Y_grid(:);
Z_grid = Z_grid(:);

X_center = zeros(Nx,Ny,Nz);
Y_center = zeros(Nx,Ny,Nz);
Z_center = zeros(Nx,Ny,Nz);

for i = 1:Nx
    for j = 1:Ny
        for k = 1:Nz
            X_center(i,j,k) = (x_grid(i+1) + x_grid(i))/2;
            Y_center(i,j,k) = (y_grid(j+1) + y_grid(j))/2;
            Z_center(i,j,k) = (z_grid(k+1) + z_grid(k))/2;
        end
    end
end

X_center = X_center(:);
Y_center = Y_center(:);
Z_center = Z_center(:);

state_discretization.X_grid_coarse = X_grid;
state_discretization.Y_grid_coarse = Y_grid;
state_discretization.Z_grid_coarse = Z_grid;

state_discretization.x_grid_coarse = x_grid;
state_discretization.y_grid_coarse = y_grid;
state_discretization.z_grid_coarse = z_grid;

state_discretization.X_center_coarse = X_center;
state_discretization.Y_center_coarse = Y_center;
state_discretization.Z_center_coarse = Z_center;

state_discretization.Deltax_coarse = Deltax;
state_discretization.Deltay_coarse = Deltay;
state_discretization.Deltaz_coarse = Deltaz;

state_discretization.vertices_q = cell(Nx*Ny*Nz,1);

for q = 1:Nx*Ny*Nz
    state_discretization.vertices_q{q} = zeros(3,8);
    Aux_vert = [X_center(q)-Deltax/2 X_center(q)+Deltax/2
                Y_center(q)-Deltay/2 Y_center(q)+Deltay/2
                Z_center(q)-Deltaz/2 Z_center(q)+Deltaz/2];
    counter = 1;
    for px = 1:2
        for py = 1:2
            for pth = 1:2
                state_discretization.vertices_q{q}(:,counter) = [Aux_vert(1,px); Aux_vert(2,py); Aux_vert(3,pth)];
                counter = counter + 1;
            end
        end
    end
end

end

