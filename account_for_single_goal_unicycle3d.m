function [state_discretization, user_param] = account_for_single_goal_unicycle3d(state_discretization, user_param, original_sys)
% Make goal a single state and assign it to the first S_target
% ASSUMES S_goal IS A SINGLE CONNEX SET

S_target = user_param.S_target;
vertices_s = state_discretization.vertices_s;
n = original_sys.n;

s_target = S_target(1);
% Delete vertices
n_target = length( S_target );
aux_a = zeros( n, n_target);
aux_b = aux_a;
ii = uint16(1);
for s = S_target
    Vs = vertices_s{s};
    a_v = Vs( :, 1 );
    b_v = Vs( :, end );
    aux_a(:, ii) = a_v;
    aux_b(:, ii) = b_v;
    ii = ii + 1;
end
a_target = min( aux_a, [], 2 );
b_target = max( aux_b, [], 2 );

% Erase vertices of target states (except for s_target)
for s = S_target
    if ( s ~= s_target )
        vertices_s{s} = [];
    end
end

% Combine to get vertices
vertices_s_target = zeros(n, ceil(2^n) );
Aux_vert = [a_target b_target];
ii = 1;
for px = 1:2
    for py = 1:2
        for pz = 1:2
            vertices_s_target(:, ii) = [Aux_vert(1, px); Aux_vert(2, py); Aux_vert(3, pz)];
            ii = ii + 1;
        end
    end
end

vertices_s{s_target} = vertices_s_target;
state_discretization.vertices_s = vertices_s;
user_param.s_target = s_target;
% user_param = rmfield( user_param, 'S_target' );
% user_param.S_target
end