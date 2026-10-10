function [state_discretization, user_param] = discretize_state_space_ndim_2layer(original_sys, user_param)

n = original_sys.n;
if ( n > 4 )
    disp('ERROR: dimension of the state space must be n <= 4')
end

% First fine discretization

state_space_lower_left_corner = original_sys.state_space_lower_left_corner;
state_space_upper_right_corner = original_sys.state_space_upper_right_corner;
Nx = user_param.Nx;

Ns = 1;
for dimi = 1:n
    Ns = Ns*Nx(dimi);
end

x_grid = cell(n, 1);
Deltax = zeros(n, 1);
for i = 1:n
    Nxi = Nx(i);
    x_grid{i} = linspace(state_space_lower_left_corner(i), state_space_upper_right_corner(i), Nxi+1);
    Deltax(i) = (state_space_upper_right_corner(i) - state_space_lower_left_corner(i))/Nxi;
end

x_center = cell(n, 1);
for i = 1:n
    x_center{i} = zeros( Nx );
end

if ( n == 2 )
    for i = 1:Nx(1)
        for j = 1:Nx(2)
            x_center{1}(i,j) = (x_grid{1}(i+1) + x_grid{1}(i))/2;
            x_center{2}(i,j) = (x_grid{2}(j+1) + x_grid{2}(j))/2;
        end
    end
elseif ( n == 3 )
    for i = 1:Nx(1)
        for j = 1:Nx(2)
            for k = 1:Nx(3)
                x_center{1}(i,j,k) = (x_grid{1}(i+1) + x_grid{1}(i))/2;
                x_center{2}(i,j,k) = (x_grid{2}(j+1) + x_grid{2}(j))/2;
                x_center{3}(i,j,k) = (x_grid{3}(k+1) + x_grid{3}(k))/2;
            end
        end
    end
elseif ( n == 4 )
    for i = 1:Nx(1)
        for j = 1:Nx(2)
            for k = 1:Nx(3)
                for l = 1:Nx(4)
                    x_center{1}(i,j,k,l) = (x_grid{1}(i+1) + x_grid{1}(i))/2;
                    x_center{2}(i,j,k,l) = (x_grid{2}(j+1) + x_grid{2}(j))/2;
                    x_center{3}(i,j,k,l) = (x_grid{3}(k+1) + x_grid{3}(k))/2;
                    x_center{4}(i,j,k,l) = (x_grid{4}(l+1) + x_grid{4}(l))/2;
                end
            end
        end
    end
end

X_center = [];
for dimi = 1:n
    X_center = [X_center, x_center{dimi}(:)];
end

state_discretization.x_grid = x_grid;
state_discretization.X_grid = X_center;

state_discretization.Deltax = Deltax;

state_discretization.vertices_s = cell(Ns, 1);

% Only store 2 vertices
for s = 1:Ns
    a_s = 0;
    b_s = 0;
    for dimi = 1:n
        a_s = [a_s; X_center_coarse{dimi}(s) - Deltax_coarse(dimi)/2];
        b_s = [b_s; X_center_coarse{dimi}(s) + Deltax_coarse(dimi)/2];
    end
    state_discretization.vertices_s{s} = [a_s, b_s];
end


% Next coarse discretization

Nq = 1;
Nx_coarse = zeros(n, 1);
for dimi = 1:n
    Nx_coarse(dimi) = Nx(dimi)/user_param.coarse_Deltax_ratio(dimi);
    Nq = Nq*Nx_coarse(dimi);
end

x_grid_coarse = cell(n, 1);
Deltax_coarse = zeros(n, 1);
for i = 1:n
    Nx_coarsei = Nx_coarse(i);
    x_grid_coarse{i} = linspace(state_space_lower_left_corner(i), state_space_upper_right_corner(i), Nx_coarsei+1);
    Deltax_coarse(i) = (state_space_upper_right_corner(i) - state_space_lower_left_corner(i))/Nx_coarsei;
end

x_center_coarse = cell(n, 1);
for i = 1:n
    x_center_coarse{i} = zeros( Nx_coarse );
end

if ( n == 2 )
    for i = 1:Nx_coarse(1)
        for j = 1:Nx_coarse(2)
            x_center_coarse{1}(i,j) = (x_grid_coarse{1}(i+1) + x_grid_coarse{1}(i))/2;
            x_center_coarse{2}(i,j) = (x_grid_coarse{2}(j+1) + x_grid_coarse{2}(j))/2;
        end
    end
elseif ( n == 3 )
    for i = 1:Nx_coarse(1)
        for j = 1:Nx_coarse(2)
            for k = 1:Nx_coarse(3)
                x_center_coarse{1}(i,j,k) = (x_grid_coarse{1}(i+1) + x_grid_coarse{1}(i))/2;
                x_center_coarse{2}(i,j,k) = (x_grid_coarse{2}(j+1) + x_grid_coarse{2}(j))/2;
                x_center_coarse{3}(i,j,k) = (x_grid_coarse{3}(k+1) + x_grid_coarse{3}(k))/2;
            end
        end
    end
elseif ( n == 4 )
    for i = 1:Nx_coarse(1)
        for j = 1:Nx_coarse(2)
            for k = 1:Nx_coarse(3)
                for l = 1:Nx_coarse(4)
                    x_center_coarse{1}(i,j,k,l) = (x_grid_coarse{1}(i+1) + x_grid_coarse{1}(i))/2;
                    x_center_coarse{2}(i,j,k,l) = (x_grid_coarse{2}(j+1) + x_grid_coarse{2}(j))/2;
                    x_center_coarse{3}(i,j,k,l) = (x_grid_coarse{3}(k+1) + x_grid_coarse{3}(k))/2;
                    x_center_coarse{4}(i,j,k,l) = (x_grid_coarse{4}(l+1) + x_grid_coarse{4}(l))/2;
                end
            end
        end
    end
end

X_center_coarse = [];
for dimi = 1:n
    X_center_coarse = [X_center_coarse, x_center_coarse{dimi}(:)];
end

state_discretization.x_grid_coarse = x_grid_coarse;
state_discretization.X_grid_coarse = X_center_coarse;

state_discretization.Deltax_coarse = Deltax_coarse;

state_discretization.vertices_q = cell(Nq, 1);

% Only store 2 vertices
for q = 1:Nq
    a_q = 0;
    b_q = 0;
    for dimi = 1:n
        a_q = [a_q; X_center_coarse{dimi}(q) - Deltax_coarse(dimi)/2];
        b_q = [b_q; X_center_coarse{dimi}(q) + Deltax_coarse(dimi)/2];
    end
    state_discretization.vertices_q{q} = [a_q, b_q];
end


end

