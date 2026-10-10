function [disturbance_discretization, user_param] = discretize_disturbance_space_uniform_2layer(original_sys, user_param)
% Takes a rectangle in |R^d and discretizes it into Nxd*Nyd rectangles.

% First fine discretization

disturbance_space_lower_left_corner = original_sys.disturbance_space_lower_left_corner;
disturbance_space_upper_right_corner = original_sys.disturbance_space_upper_right_corner;
Nxd = user_param.Nxd;
Nyd = user_param.Nyd;

xd_grid = linspace(disturbance_space_lower_left_corner(1), disturbance_space_upper_right_corner(1), Nxd+1);
yd_grid = linspace(disturbance_space_lower_left_corner(2), disturbance_space_upper_right_corner(2), Nyd+1);
Deltaxd = (disturbance_space_upper_right_corner(1) - disturbance_space_lower_left_corner(1))/Nxd;
Deltayd = (disturbance_space_upper_right_corner(2) - disturbance_space_lower_left_corner(2))/Nyd;
[Xd_grid, Yd_grid] = meshgrid(xd_grid, yd_grid);
Xd_grid = Xd_grid(:);
Yd_grid = Yd_grid(:);

Xd_center = zeros(Nxd,Nyd);
Yd_center = zeros(Nxd,Nyd);

for i = 1:Nxd
    for j = 1:Nyd
        Xd_center(i,j) = (xd_grid(i+1) + xd_grid(i))/2;
        Yd_center(i,j) = (yd_grid(j+1) + yd_grid(j))/2;
    end
end

Xd_center = Xd_center(:);
Yd_center = Yd_center(:);

disturbance_discretization.Xd_grid = Xd_grid;
disturbance_discretization.Yd_grid = Yd_grid;

disturbance_discretization.xd_grid = xd_grid;
disturbance_discretization.yd_grid = yd_grid;

disturbance_discretization.Xd_center = Xd_center;
disturbance_discretization.Yd_center = Yd_center;

disturbance_discretization.Deltaxd = Deltaxd;
disturbance_discretization.Deltayd = Deltayd;

disturbance_discretization.vertices_c = cell(Nxd*Nyd,1);

for c_ind = 1:Nxd*Nyd
    disturbance_discretization.vertices_c{c_ind} = zeros(2,4);
    Aux_vert = [Xd_center(c_ind)-Deltaxd/2 Xd_center(c_ind)+Deltaxd/2
                Yd_center(c_ind)-Deltayd/2 Yd_center(c_ind)+Deltayd/2];
    counter = 1;
    for px = 1:2
        for py = 1:2
            disturbance_discretization.vertices_c{c_ind}(:,counter) = [Aux_vert(1,px); Aux_vert(2,py)];
            counter = counter + 1;
        end
    end
end

user_param.Nc = user_param.Nxd*user_param.Nyd;

% Next, coarse discretization

user_param.Nxd_coarse = user_param.Nxd/user_param.coarse_Deltaxd_ratio;
user_param.Nyd_coarse = user_param.Nyd/user_param.coarse_Deltayd_ratio;
user_param.Nc_coarse = user_param.Nxd_coarse*user_param.Nyd_coarse;


Nxd = user_param.Nxd_coarse;
Nyd = user_param.Nyd_coarse;

xd_grid = linspace(disturbance_space_lower_left_corner(1), disturbance_space_upper_right_corner(1), Nxd+1);
yd_grid = linspace(disturbance_space_lower_left_corner(2), disturbance_space_upper_right_corner(2), Nyd+1);
Deltaxd = (disturbance_space_upper_right_corner(1) - disturbance_space_lower_left_corner(1))/Nxd;
Deltayd = (disturbance_space_upper_right_corner(2) - disturbance_space_lower_left_corner(2))/Nyd;
[Xd_grid, Yd_grid] = meshgrid(xd_grid, yd_grid);
Xd_grid = Xd_grid(:);
Yd_grid = Yd_grid(:);

Xd_center = zeros(Nxd,Nyd);
Yd_center = zeros(Nxd,Nyd);

for i = 1:Nxd
    for j = 1:Nyd
        Xd_center(i,j) = (xd_grid(i+1) + xd_grid(i))/2;
        Yd_center(i,j) = (yd_grid(j+1) + yd_grid(j))/2;
    end
end

Xd_center = Xd_center(:);
Yd_center = Yd_center(:);

disturbance_discretization.Xd_grid_coarse = Xd_grid;
disturbance_discretization.Yd_grid_coarse = Yd_grid;

disturbance_discretization.xd_grid_coarse = xd_grid;
disturbance_discretization.yd_grid_coarse = yd_grid;

disturbance_discretization.Xd_center_coarse = Xd_center;
disturbance_discretization.Yd_center_coarse = Yd_center;

disturbance_discretization.Deltaxd_coarse = Deltaxd;
disturbance_discretization.Deltayd_coarse = Deltayd;

disturbance_discretization.vertices_c_coarse = cell(Nxd*Nyd,1);

for c_coarse_i = 1:Nxd*Nyd
    disturbance_discretization.vertices_c_coarse{c_coarse_i} = zeros(2,4);
    Aux_vert = [Xd_center(c_coarse_i)-Deltaxd/2 Xd_center(c_coarse_i)+Deltaxd/2
                Yd_center(c_coarse_i)-Deltayd/2 Yd_center(c_coarse_i)+Deltayd/2];
    counter = 1;
    for px = 1:2
        for py = 1:2
            disturbance_discretization.vertices_c_coarse{c_coarse_i}(:,counter) = [Aux_vert(1,px); Aux_vert(2,py)];
            counter = counter + 1;
        end
    end
end


end

