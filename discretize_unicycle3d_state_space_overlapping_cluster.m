function [state_discretization, user_param] = discretize_unicycle3d_state_space_overlapping_cluster(original_sys, user_param)
% Takes a rectangle in |R^n and discretizes it into Nx*Ny*Nz rectangles.

user_param.Ns = user_param.Nx*user_param.Ny*user_param.Nz;

% First fine discretization

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

X_center = zeros(Nx,Ny, Nz);
Y_center = zeros(Nx,Ny, Nz);
Z_center = zeros(Nx,Ny, Nz);

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

state_discretization.vertices_s = cell(Nx*Ny*Nz, 1);

for s = 1:Nx*Ny*Nz
    state_discretization.vertices_s{s} = zeros(original_sys.n, 2^original_sys.n);
    Aux_vert = [X_center(s)-Deltax/2 X_center(s)+Deltax/2
                Y_center(s)-Deltay/2 Y_center(s)+Deltay/2
                Z_center(s)-Deltaz/2 Z_center(s)+Deltaz/2];
    counter = 1;
    for px = 1:2
        for py = 1:2
            for pz = 1:2
                state_discretization.vertices_s{s}(:,counter) = [Aux_vert(1,px); Aux_vert(2,py); Aux_vert(3,pz)];
                counter = counter + 1;
            end
        end
    end
end

% Next coarse discretization

state_space_lower_left_corner = original_sys.state_space_lower_left_corner;
state_space_upper_right_corner = original_sys.state_space_upper_right_corner;
Nx = user_param.Nx;
Ny = user_param.Ny;
Nz = user_param.Nz;

Deltax = (state_space_upper_right_corner(1) - state_space_lower_left_corner(1))/Nx;
Deltay = (state_space_upper_right_corner(2) - state_space_lower_left_corner(2))/Ny;
Deltaz = (state_space_upper_right_corner(3) - state_space_lower_left_corner(3))/Nz;

user_param.Nx_coarse = user_param.Nx - user_param.coarse_Deltax_ratio + 1;
user_param.Ny_coarse = user_param.Ny - user_param.coarse_Deltay_ratio + 1;
user_param.Nz_coarse = user_param.Nz; % ZIS ONE WRAPS
user_param.Nq = user_param.Nx_coarse*user_param.Ny_coarse*user_param.Nz_coarse;

Nx_coarse = user_param.Nx_coarse;
Ny_coarse = user_param.Ny_coarse;
Nz_coarse = user_param.Nz_coarse;

X_center = zeros(Nx_coarse, Ny_coarse, Nz_coarse);
Y_center = zeros(Nx_coarse, Ny_coarse, Nz_coarse);
Z_center = zeros(Nx_coarse, Ny_coarse, Nz_coarse);
aux_center_coarse_x = user_param.coarse_Deltax_ratio/2 - 1;
aux_cnter_coarse_y = user_param.coarse_Deltay_ratio/2 - 1;
aux_cnter_coarse_z = user_param.coarse_Deltaz_ratio/2 - 1;
for i = 1:Nx_coarse
    for j = 1:Ny_coarse
        for k = 1:Nz_coarse
            X_center(i,j, k) = (i + aux_center_coarse_x)*Deltax;
            Y_center(i,j, k) = (j + aux_cnter_coarse_y)*Deltay;
            Z_center(i,j, k) = (k + aux_cnter_coarse_z)*Deltaz;
        end
    end
end

X_center = X_center(:);
Y_center = Y_center(:);
Z_center = Z_center(:);

state_discretization.X_center_coarse = X_center;
state_discretization.Y_center_coarse = Y_center;
state_discretization.Z_center_coarse = Z_center;

Deltax_coarse = user_param.coarse_Deltax_ratio*Deltax;
Deltay_coarse = user_param.coarse_Deltay_ratio*Deltay;
Deltaz_coarse = user_param.coarse_Deltaz_ratio*Deltaz;

state_discretization.Deltax_coarse = Deltax_coarse;
state_discretization.Deltay_coarse = Deltay_coarse;
state_discretization.Deltaz_coarse = Deltaz_coarse;

state_discretization.vertices_q = cell((Nx_coarse)*(Ny_coarse)*(Nz_coarse), 1);


for q = 1:(Nx_coarse)*(Ny_coarse)*(Nz_coarse)
    state_discretization.vertices_q{q} = zeros(original_sys.n, 2^original_sys.n);
    Aux_vert = [X_center(q)-Deltax_coarse/2 X_center(q)+Deltax_coarse/2
                Y_center(q)-Deltay_coarse/2 Y_center(q)+Deltay_coarse/2
                Z_center(q)-Deltaz_coarse/2 Z_center(q)+Deltaz_coarse/2];
    counter = 1;
    for px = 1:2
        for py = 1:2
            for pz = 1:2
                state_discretization.vertices_q{q}(:,counter) = [Aux_vert(1,px); Aux_vert(2,py); Aux_vert(3,pz)];
                counter = counter + 1;
            end
        end
    end
end

end

