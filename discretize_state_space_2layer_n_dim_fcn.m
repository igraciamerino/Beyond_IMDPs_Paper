function [state_discretization, user_param] = discretize_state_space_2layer_n_dim_fcn(original_sys, user_param)

n = original_sys.n;
Nx = user_param.Nx;

user_param.Ns = 1;
for di = 1:n
    user_param.Ns = user_param.Ns*Nx{di};
end

state_space_lower_left_corner = original_sys.state_space_lower_left_corner;
state_space_upper_right_corner = original_sys.state_space_upper_right_corner;

for di = 1:n
    x_grid{di} = linspace(state_space_lower_left_corner(di), state_space_upper_right_corner(di), Nx{di}+1);
    Deltax{di} = (state_space_upper_right_corner(di) - state_space_lower_left_corner(di))/Nx{di};
end

if ( n == 2)
    [X_grid{1}, X_grid{2}] = ndgrid(x_grid{1}, x_grid{2});
    X_grid{1} = X_grid{1}(:);
    X_grid{2} = X_grid{2}(:);

elseif ( n == 3)
    [X_grid{1}, X_grid{2}, X_grid{3}] = ndgrid(x_grid{1}, x_grid{2}, x_grid{3});
    X_grid{1} = X_grid{1}(:);
    X_grid{2} = X_grid{2}(:);
    X_grid{3} = X_grid{3}(:);
elseif ( n == 4)
    [X_grid{1}, X_grid{2}, X_grid{3}, X_grid{4}] = ndgrid(x_grid{1}, x_grid{2}, x_grid{3}, x_grid{4});
    X_grid{1} = X_grid{1}(:);
    X_grid{2} = X_grid{2}(:);
    X_grid{3} = X_grid{3}(:);
    X_grid{4} = X_grid{4}(:);
end

Nx_aux = [];
for di = 1:n
    Nx_aux = [Nx_aux Nx{di}];
end

for di = 1:n
    X_center{di} = zeros(Nx_aux);
end

if ( n == 2)
    for i = 1:Nx{1}
        for j = 1:Nx{2}
            X_center{1}(i,j) = (x_grid{1}(i+1) + x_grid{1}(i))/2;
            X_center{2}(i,j) = (x_grid{2}(j+1) + x_grid{2}(j))/2;
        end
    end
    X_center{1} = X_center{1}(:);
    X_center{2} = X_center{2}(:);
elseif ( n == 3)
    for i = 1:Nx{1}
        for j = 1:Nx{2}
            for k = 1:Nx{3}
                X_center{1}(i,j,k) = (x_grid{1}(i+1) + x_grid{1}(i))/2;
                X_center{2}(i,j,k) = (x_grid{2}(j+1) + x_grid{2}(j))/2;
                X_center{3}(i,j,k) = (x_grid{3}(k+1) + x_grid{3}(k))/2;

            end
        end
    end
    X_center{1} = X_center{1}(:);
    X_center{2} = X_center{2}(:);
    X_center{3} = X_center{3}(:);
elseif ( n == 4)
    for i = 1:Nx{1}
        for j = 1:Nx{2}
            for k = 1:Nx{3}
                for l = 1:Nx{4}
                    X_center{1}(i,j,k,l) = (x_grid{1}(i+1) + x_grid{1}(i))/2;
                    X_center{2}(i,j,k,l) = (x_grid{2}(j+1) + x_grid{2}(j))/2;
                    X_center{3}(i,j,k,l) = (x_grid{3}(k+1) + x_grid{3}(k))/2;
                    X_center{4}(i,j,k,l) = (x_grid{4}(l+1) + x_grid{4}(l))/2;
                end
            end
        end
    end
    X_center{1} = X_center{1}(:);
    X_center{2} = X_center{2}(:);
    X_center{3} = X_center{3}(:);
    X_center{4} = X_center{4}(:);
end


for di = 1:n
    state_discretization.X_grid{di} = X_grid{di};
    state_discretization.x_grid{di} = x_grid{di};
    state_discretization.X_center{di} = X_center{di};
    state_discretization.Deltax{di} = Deltax{di};
end

state_discretization.vertices_s = cell(user_param.Ns, 1);

if ( n == 2)
    for s = 1:user_param.Ns
        state_discretization.vertices_s{s} = zeros(original_sys.n, 2^original_sys.n);
        Aux_vert = [X_center{1}(s)-Deltax{1}/2 X_center{1}(s)+Deltax{1}/2
            X_center{2}(s)-Deltax{2}/2 X_center{2}(s)+Deltax{2}/2];
        counter = 1;
        for p1 = 1:2
            for p2 = 1:2
                state_discretization.vertices_s{s}(:,counter) = [Aux_vert(1,p1); Aux_vert(2,p2)];
                counter = counter + 1;
            end
        end
    end
elseif ( n == 3)
    for s = 1:user_param.Ns
        state_discretization.vertices_s{s} = zeros(original_sys.n, 2^original_sys.n);
        Aux_vert = [X_center{1}(s)-Deltax{1}/2 X_center{1}(s)+Deltax{1}/2
            X_center{2}(s)-Deltax{2}/2 X_center{2}(s)+Deltax{2}/2
            X_center{3}(s)-Deltax{3}/2 X_center{3}(s)+Deltax{3}/2];
        counter = 1;
        for p1 = 1:2
            for p2 = 1:2
                for p3 = 1:2
                    state_discretization.vertices_s{s}(:,counter) = [Aux_vert(1,p1); Aux_vert(2,p2); Aux_vert(3,p3)];
                    counter = counter + 1;
                end
            end
        end
    end
elseif ( n == 4)
    for s = 1:user_param.Ns
        state_discretization.vertices_s{s} = zeros(original_sys.n, 2^original_sys.n);
        Aux_vert = [X_center{1}(s)-Deltax{1}/2 X_center{1}(s)+Deltax{1}/2
            X_center{2}(s)-Deltax{2}/2 X_center{2}(s)+Deltax{2}/2
            X_center{3}(s)-Deltax{3}/2 X_center{3}(s)+Deltax{3}/2
            X_center{4}(s)-Deltax{4}/2 X_center{4}(s)+Deltax{4}/2];
        counter = 1;
        for p1 = 1:2
            for p2 = 1:2
                for p3 = 1:2
                    for p4 = 1:2
                        state_discretization.vertices_s{s}(:,counter) = [Aux_vert(1,p1); Aux_vert(2,p2); Aux_vert(3,p3); Aux_vert(4,p4)];
                        counter = counter + 1;
                    end
                end
            end
        end
    end
end

% Second, coarse discretization

Nx_coarse = cell(1, n);
user_param.Nq = 1;
for di = 1:n
    Nx_coarse{di} = Nx{di}/user_param.coarse_Deltax_ratio{di};
    user_param.Nq = user_param.Nq*Nx_coarse{di};
end

user_param.Nx_coarse = Nx_coarse;

state_space_lower_left_corner = original_sys.state_space_lower_left_corner;
state_space_upper_right_corner = original_sys.state_space_upper_right_corner;

for di = 1:n
    x_grid_coarse{di} = linspace(state_space_lower_left_corner(di), state_space_upper_right_corner(di), Nx_coarse{di}+1);
    Deltax_coarse{di} = (state_space_upper_right_corner(di) - state_space_lower_left_corner(di))/Nx_coarse{di};
end

if ( n == 2)
    [X_grid_coarse{1}, X_grid_coarse{2}] = ndgrid(x_grid_coarse{1}, x_grid_coarse{2});
    X_grid_coarse{1} = X_grid_coarse{1}(:);
    X_grid_coarse{2} = X_grid_coarse{2}(:);

elseif ( n == 3)
    [X_grid_coarse{1}, X_grid_coarse{2}, X_grid_coarse{3}] = ndgrid(x_grid_coarse{1}, x_grid_coarse{2}, x_grid_coarse{3});
    X_grid_coarse{1} = X_grid_coarse{1}(:);
    X_grid_coarse{2} = X_grid_coarse{2}(:);
    X_grid_coarse{3} = X_grid_coarse{3}(:);
elseif ( n == 4)
    [X_grid_coarse{1}, X_grid_coarse{2}, X_grid_coarse{3}, X_grid_coarse{4}] = ndgrid(x_grid_coarse{1}, x_grid_coarse{2}, x_grid_coarse{3}, x_grid_coarse{4});
    X_grid_coarse{1} = X_grid_coarse{1}(:);
    X_grid_coarse{2} = X_grid_coarse{2}(:);
    X_grid_coarse{3} = X_grid_coarse{3}(:);
    X_grid_coarse{4} = X_grid_coarse{4}(:);
end

Nx_coarse_aux = [];
for di = 1:n
    Nx_coarse_aux = [Nx_coarse_aux Nx_coarse{di}];
end

for di = 1:n
    X_center_coarse{di} = zeros(Nx_coarse_aux);
end

if ( n == 2)
    for i = 1:Nx_coarse{1}
        for j = 1:Nx_coarse{2}
            X_center_coarse{1}(i,j) = (x_grid_coarse{1}(i+1) + x_grid_coarse{1}(i))/2;
            X_center_coarse{2}(i,j) = (x_grid_coarse{2}(j+1) + x_grid_coarse{2}(j))/2;
        end
    end
    X_center_coarse{1} = X_center_coarse{1}(:);
    X_center_coarse{2} = X_center_coarse{2}(:);
elseif ( n == 3)
    for i = 1:Nx_coarse{1}
        for j = 1:Nx_coarse{2}
            for k = 1:Nx_coarse{3}
                X_center_coarse{1}(i,j,k) = (x_grid_coarse{1}(i+1) + x_grid_coarse{1}(i))/2;
                X_center_coarse{2}(i,j,k) = (x_grid_coarse{2}(j+1) + x_grid_coarse{2}(j))/2;
                X_center_coarse{3}(i,j,k) = (x_grid_coarse{3}(k+1) + x_grid_coarse{3}(k))/2;
            end
        end
    end
    X_center_coarse{1} = X_center_coarse{1}(:);
    X_center_coarse{2} = X_center_coarse{2}(:);
    X_center_coarse{3} = X_center_coarse{3}(:);
elseif ( n == 4)
    for i = 1:Nx_coarse{1}
        for j = 1:Nx_coarse{2}
            for k = 1:Nx_coarse{3}
                for l = 1:Nx_coarse{4}
                    X_center_coarse{1}(i,j,k,l) = (x_grid_coarse{1}(i+1) + x_grid_coarse{1}(i))/2;
                    X_center_coarse{2}(i,j,k,l) = (x_grid_coarse{2}(j+1) + x_grid_coarse{2}(j))/2;
                    X_center_coarse{3}(i,j,k,l) = (x_grid_coarse{3}(k+1) + x_grid_coarse{3}(k))/2;
                    X_center_coarse{4}(i,j,k,l) = (x_grid_coarse{4}(l+1) + x_grid_coarse{4}(l))/2;
                end
            end
        end
    end
    X_center_coarse{1} = X_center_coarse{1}(:);
    X_center_coarse{2} = X_center_coarse{2}(:);
    X_center_coarse{3} = X_center_coarse{3}(:);
    X_center_coarse{4} = X_center_coarse{4}(:);
end


for di = 1:n
    state_discretization.X_grid_coarse{di} = X_grid_coarse{di};
    state_discretization.x_grid_coarse{di} = x_grid_coarse{di};
    state_discretization.X_center_coarse{di} = X_center_coarse{di};
    state_discretization.Deltax_coarse{di} = Deltax_coarse{di};
end

state_discretization.vertices_q = cell(user_param.Nq, 1);

if ( n == 2)
    for q = 1:user_param.Nq
        state_discretization.vertices_q{q} = zeros(original_sys.n, 2^original_sys.n);
        Aux_vert = [X_center_coarse{1}(q)-Deltax_coarse{1}/2 X_center_coarse{1}(q)+Deltax_coarse{1}/2
            X_center_coarse{2}(q)-Deltax_coarse{2}/2 X_center_coarse{2}(q)+Deltax_coarse{2}/2];
        counter = 1;
        for p1 = 1:2
            for p2 = 1:2
                state_discretization.vertices_q{q}(:,counter) = [Aux_vert(1,p1); Aux_vert(2,p2)];
                counter = counter + 1;
            end
        end
    end
elseif ( n == 3)
    for q = 1:user_param.Nq
        state_discretization.vertices_q{q} = zeros(original_sys.n, 2^original_sys.n);
        Aux_vert = [X_center_coarse{1}(q)-Deltax_coarse{1}/2 X_center_coarse{1}(q)+Deltax_coarse{1}/2
            X_center_coarse{2}(q)-Deltax_coarse{2}/2 X_center_coarse{2}(q)+Deltax_coarse{2}/2
            X_center_coarse{3}(q)-Deltax_coarse{3}/2 X_center_coarse{3}(q)+Deltax_coarse{3}/2];
        counter = 1;
        for p1 = 1:2
            for p2 = 1:2
                for p3 = 1:2
                    state_discretization.vertices_q{q}(:,counter) = [Aux_vert(1,p1); Aux_vert(2,p2); Aux_vert(3,p3)];
                    counter = counter + 1;
                end
            end
        end
    end
elseif ( n == 4)
    for q = 1:user_param.Nq
        state_discretization.vertices_q{q} = zeros(original_sys.n, 2^original_sys.n);
        Aux_vert = [X_center_coarse{1}(q)-Deltax_coarse{1}/2 X_center_coarse{1}(q)+Deltax_coarse{1}/2
            X_center_coarse{2}(q)-Deltax_coarse{2}/2 X_center_coarse{2}(q)+Deltax_coarse{2}/2
            X_center_coarse{3}(q)-Deltax_coarse{3}/2 X_center_coarse{3}(q)+Deltax_coarse{3}/2
            X_center_coarse{4}(q)-Deltax_coarse{4}/2 X_center_coarse{4}(q)+Deltax_coarse{4}/2];
        counter = 1;
        for p1 = 1:2
            for p2 = 1:2
                for p3 = 1:2
                    for p4 = 1:2
                        state_discretization.vertices_q{q}(:,counter) = [Aux_vert(1,p1); Aux_vert(2,p2); Aux_vert(3,p3); Aux_vert(4,p4)];
                        counter = counter + 1;
                    end
                end
            end
        end
    end
end

end

