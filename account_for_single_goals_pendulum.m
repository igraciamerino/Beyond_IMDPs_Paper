function [state_discretization, user_param] = account_for_single_goals_pendulum(state_discretization, user_param, original_sys)
% Make goal a single state and assign it to the first S_target
% ASSUMES S_goal IS A SINGLE CONNEX SET

vertices_s = state_discretization.vertices_s;
n = original_sys.n;
Nx_S_target = user_param.Nx_S_target;
Deltax = state_discretization.Deltax;
n_target_blocks = user_param.n_target_blocks;
S_target_blocks = user_param.S_target_blocks;
S_target = user_param.S_target;

user_param.s_target = [];
for blocki = 1:n_target_blocks
    S_target_block = S_target_blocks{blocki};
    s_target_block = S_target_block(1);
    n_target_block = length( S_target_block );
    aux_a = zeros( n, n_target_block);
    aux_b = aux_a;
    ii = uint16(1);
    for s = S_target_block
        Vs = vertices_s{s};
        a_v = Vs( :, 1 );
        b_v = Vs( :, end );
        aux_a(:, ii) = a_v;
        aux_b(:, ii) = b_v;
        ii = ii + 1;
    end
    a_target = min( aux_a, [], 2 );
    b_target = max( aux_b, [], 2 );
    % Modify (a_target, b_target) to account for wrapping
    if ( blocki == 1 )
        a_target(1) = -b_target(1);
    elseif ( blocki == 2 )
        b_target(1) = 2*b_target(1)-a_target(1);
    end
    % Combine to get vertices
    vertices_s_target = zeros(n, ceil(2^n) );
    Aux_vert = [a_target b_target];
    ii = 1;
    for px = 1:2
        for py = 1:2
            vertices_s_target(:, ii) = [Aux_vert(1, px); Aux_vert(2, py)];
            ii = ii + 1;
        end
    end
    vertices_s{s_target_block} = vertices_s_target;

    % Erase vertices of target states (except for s_target)
    for s = S_target_block
        if ( s ~= s_target_block )
            vertices_s{s} = [];
        end
    end

    user_param.s_target_blocks{blocki} = s_target_block;
    user_param.s_target = [user_param.s_target, s_target_block];
end


state_discretization.vertices_s = vertices_s;

end