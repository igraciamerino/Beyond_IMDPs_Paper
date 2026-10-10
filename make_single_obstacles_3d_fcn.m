function [state_discretization, user_param] = make_single_obstacles_3d_fcn(state_discretization, user_param, original_sys)
% Make goal a single state and assign it to the first S_obstacle
% ASSUMES S_goal IS A SINGLE CONNEX SET

vertices_s = state_discretization.vertices_s;
n = original_sys.n;
n_obstacle_blocks = user_param.n_obstacle_blocks;
S_obstacle_blocks = user_param.S_obstacle_blocks;

user_param.s_obstacle = [];
for blocki = 1:n_obstacle_blocks
    S_obstacle_block = S_obstacle_blocks{blocki};
    s_obstacle_block = S_obstacle_block(1);
    n_obstacle_block = length( S_obstacle_block );
    aux_a = zeros( n, n_obstacle_block);
    aux_b = aux_a;
    ii = uint16(1);
    for s = S_obstacle_block
        Vs = vertices_s{s};
        a_v = Vs( :, 1 );
        b_v = Vs( :, end );
        aux_a(:, ii) = a_v;
        aux_b(:, ii) = b_v;
        ii = ii + 1;
    end
    a_obstacle = min( aux_a, [], 2 );
    b_obstacle = max( aux_b, [], 2 );

    % Combine to get vertices
    vertices_s_obstacle = zeros(n, ceil(2^n) );
    Aux_vert = [a_obstacle b_obstacle];
    ii = 1;
    for px = 1:2
        for py = 1:2
            for pz = 1:2
                vertices_s_obstacle(:, ii) = [Aux_vert(1, px); Aux_vert(2, py); Aux_vert(3, pz)];
                ii = ii + 1;
            end
        end
    end
    vertices_s{s_obstacle_block} = vertices_s_obstacle;

    % Erase vertices of obstacle states (except for s_obstacle)
    for s = S_obstacle_block
        if ( s ~= s_obstacle_block )
            vertices_s{s} = [];
        end
    end

    user_param.s_obstacle_blocks{blocki} = s_obstacle_block;
    user_param.s_obstacle = [user_param.s_obstacle, s_obstacle_block];
end


state_discretization.vertices_s = vertices_s;

end