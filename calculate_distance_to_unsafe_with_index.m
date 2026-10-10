function [dist, ind_dist] = calculate_distance_to_unsafe_with_index(a_X, b_X, x, Deltax, Deltay, Deltath)

% infty-distance
dist = zeros(4,1);
dist(1)  = ( x(1) - a_X(1) )/Deltax;
dist(2)  = ( x(1) - b_X(1) )/Deltax;
dist(3)  = ( x(2) - a_X(2) )/Deltay;
dist(4)  = ( x(2) - b_X(2) )/Deltay;

[dist, aux_ind] = min(abs(dist));
dist = uint8(dist);

if ( (aux_ind == 1) || (aux_ind == 2) )
    ind_dist = uint8(1);
else
    ind_dist = uint8(2);
end

end