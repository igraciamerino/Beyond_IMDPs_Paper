function [disturbance_discretization, user_param] = discretize_4d_disturbance_space_uniform(original_sys, user_param)
% Takes a rectangle in |R^d and discretizes it into Nxd*Nyd rectangles.

d = original_sys.d;
disturbance_space_lower_left_corner = original_sys.disturbance_space_lower_left_corner;
disturbance_space_upper_right_corner = original_sys.disturbance_space_upper_right_corner;
Nxd = user_param.Nxd;

Nc = 1;
for di = 1:d
    xd_grid{di} = linspace(disturbance_space_lower_left_corner(di), disturbance_space_upper_right_corner(di), Nxd{di}+1);
    Deltaxd{di} = (disturbance_space_upper_right_corner(di) - disturbance_space_lower_left_corner(di))/Nxd{di};
    Nc = Nc*Nxd{di};
end
[Xd_grid{1}, Xd_grid{2}, Xd_grid{3}, Xd_grid{4}] = ndgrid(xd_grid{1}, xd_grid{2}, xd_grid{3}, xd_grid{4});
Xd_grid{1} = Xd_grid{1}(:);
Xd_grid{2} = Xd_grid{2}(:);
Xd_grid{3} = Xd_grid{3}(:);
Xd_grid{4} = Xd_grid{4}(:);

for di = 1:d
    Xd_center{di} = zeros(Nxd{1}, Nxd{2}, Nxd{3}, Nxd{4});
end

for i = 1:Nxd{1}
    for j = 1:Nxd{2}
        for k = 1:Nxd{3}
            for l = 1:Nxd{4}
                Xd_center{1}(i,j,k,l) = (xd_grid{1}(i+1) + xd_grid{1}(i))/2;
                Xd_center{2}(i,j,k,l) = (xd_grid{2}(j+1) + xd_grid{2}(j))/2;
                Xd_center{3}(i,j,k,l) = (xd_grid{3}(k+1) + xd_grid{3}(k))/2;
                Xd_center{4}(i,j,k,l) = (xd_grid{4}(l+1) + xd_grid{4}(l))/2;
            end
        end
    end
end

Xd_center{1} = Xd_center{1}(:);
Xd_center{2} = Xd_center{2}(:);
Xd_center{3} = Xd_center{3}(:);
Xd_center{4} = Xd_center{4}(:);
for di = 1:d
    disturbance_discretization.Xd_grid{di} = Xd_grid{di};
    disturbance_discretization.xd_grid{di} = xd_grid{di};
    disturbance_discretization.Xd_center{di} = Xd_center{di};
    disturbance_discretization.Deltaxd{di} = Deltaxd{di};
end

disturbance_discretization.vertices_c = cell(Nc, 1);

for ci = 1:Nc
    disturbance_discretization.vertices_c{ci} = zeros(original_sys.d, 2^original_sys.d);
    Aux_vert = [Xd_center{1}(ci)-Deltaxd{1}/2 Xd_center{1}(ci)+Deltaxd{1}/2
                Xd_center{2}(ci)-Deltaxd{2}/2 Xd_center{2}(ci)+Deltaxd{2}/2
                Xd_center{3}(ci)-Deltaxd{3}/2 Xd_center{3}(ci)+Deltaxd{3}/2
                Xd_center{4}(ci)-Deltaxd{4}/2 Xd_center{4}(ci)+Deltaxd{4}/2];
    counter = 1;
    for p1 = 1:2
        for p2 = 1:2
            for p3 = 1:2
                for p4 = 1:2
                    disturbance_discretization.vertices_c{ci}(:,counter) = [Aux_vert(1,p1); Aux_vert(2,p2); Aux_vert(3,p3); Aux_vert(4,p4)];
                    counter = counter + 1;
                end
            end
        end
    end
end

user_param.Nc = Nc;

end

