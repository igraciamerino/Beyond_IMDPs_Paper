function [DR_UQ] = DR_Uncertainty_Quantification2D(Xi_hat,epsilon,A,b,to_polytope)

% CAREFUL!!!!!! I'VE TAKEN OUT THE SQUARES IN THE DISTANCE BC THE DISTANCE
% FUNCTION ALREADY GIVES YOU THAT DISTANCE

N = size(Xi_hat,2);
dist = zeros(1,N);
for i=1:N
    xi = Xi_hat(:,i);
    dist(i) = calculate_distance2D(A, b, xi, to_polytope);
end

dist = sort(dist);

sum = 0;
for i = 1:N
   
    sum = sum + dist(i);
    if (sum>N*epsilon^2)
        sum = sum - dist(i);
        i = i-1;
        break
    end

end

j_star = i;

if (j_star == N)
    DR_UQ = 1;
elseif (j_star < N)
   p_star = (N*epsilon^2 - sum)/dist(j_star+1);
   DR_UQ = (j_star + p_star)/N;
end


end

