function [dist] = calculate_distance_to_unsafe(a_X, b_X, x)

% infty-distance
dist = zeros(4,1);
dist(1)  = x(1) - a_X(1);
dist(2)  = x(1) - b_X(1);
dist(3)  = x(2) - a_X(2);
dist(4)  = x(2) - b_X(2);

dist = min(abs(dist));

end