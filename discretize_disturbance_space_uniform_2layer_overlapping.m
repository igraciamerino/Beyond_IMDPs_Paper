function [disturbance_discretization, user_param] = discretize_disturbance_space_uniform_2layer_overlapping(original_sys, user_param)
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

X_center = zeros(Nxd,Nyd);
Y_center = zeros(Nxd,Nyd);

for i = 1:Nxd
    for j = 1:Nyd
        X_center(i,j) = (xd_grid(i+1) + xd_grid(i))/2;
        Y_center(i,j) = (yd_grid(j+1) + yd_grid(j))/2;
    end
end

X_center = X_center(:);
Y_center = Y_center(:);

disturbance_discretization.Xd_grid = Xd_grid;
disturbance_discretization.Yd_grid = Yd_grid;

disturbance_discretization.xd_grid = xd_grid;
disturbance_discretization.yd_grid = yd_grid;

disturbance_discretization.Xd_center = X_center;
disturbance_discretization.Yd_center = Y_center;

disturbance_discretization.Deltaxd = Deltaxd;
disturbance_discretization.Deltayd = Deltayd;

disturbance_discretization.vertices_c = cell(Nxd*Nyd,1);

for c_ind = 1:Nxd*Nyd
    disturbance_discretization.vertices_c{c_ind} = zeros(2,4);
    Aux_vert = [X_center(c_ind)-Deltaxd/2 X_center(c_ind)+Deltaxd/2
                Y_center(c_ind)-Deltayd/2 Y_center(c_ind)+Deltayd/2];
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

user_param.Nxd_coarse = user_param.Nxd - user_param.coarse_Deltaxd_ratio + 1;
user_param.Nyd_coarse = user_param.Nyd - user_param.coarse_Deltayd_ratio + 1;

user_param.Nc_coarse = user_param.Nxd_coarse*user_param.Nyd_coarse;

Nxd_coarse = user_param.Nxd_coarse;
Nyd_coarse = user_param.Nyd_coarse;

X_center = zeros(Nxd_coarse, Nyd_coarse);
Y_center = zeros(Nxd_coarse, Nyd_coarse);
aux_center_coarse_xd = user_param.coarse_Deltaxd_ratio/2 - 1;
aux_center_coarse_yd = user_param.coarse_Deltayd_ratio/2 - 1;
for i = 1:Nxd_coarse
    for j = 1:Nyd_coarse
        X_center(i,j) = (i + aux_center_coarse_xd)*Deltaxd + disturbance_space_lower_left_corner(1);
        Y_center(i,j) = (j + aux_center_coarse_yd)*Deltayd + disturbance_space_lower_left_corner(2);
    end
end

X_center = X_center(:);
Y_center = Y_center(:);

disturbance_discretization.Xd_center_coarse = X_center;
disturbance_discretization.Yd_center_coarse = Y_center;

disturbance_discretization.Deltaxd_coarse = user_param.coarse_Deltaxd_ratio*Deltaxd;
disturbance_discretization.Deltayd_coarse = user_param.coarse_Deltayd_ratio*Deltayd;

disturbance_discretization.vertices_c_coarse = cell((Nxd_coarse)*(Nyd_coarse), 1);

for c_coarse_ind = 1:(Nxd_coarse)*(Nyd_coarse)
    disturbance_discretization.vertices_c_coarse{c_coarse_ind} = zeros(2,4);
    Aux_vert = [X_center(c_coarse_ind)-disturbance_discretization.Deltaxd_coarse/2 X_center(c_coarse_ind)+disturbance_discretization.Deltaxd_coarse/2
                Y_center(c_coarse_ind)-disturbance_discretization.Deltayd_coarse/2 Y_center(c_coarse_ind)+disturbance_discretization.Deltayd_coarse/2];
    counter = 1;
    for px = 1:2
        for py = 1:2
            disturbance_discretization.vertices_c_coarse{c_coarse_ind}(:,counter) = [Aux_vert(1,px); Aux_vert(2,py)];
            counter = counter + 1;
        end
    end
end

end

