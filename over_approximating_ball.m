function [center_ball_Xi_hat, radius_ball_Xi_hat] = over_approximating_ball(Xi_hat)

N = length(Xi_hat(1,:));
center_ball_Xi_hat = mean(Xi_hat')';
% center_ball_Xi_hat = [0;0;0];
previous_radius = 0;
for i = 1:N
    current_sample = Xi_hat(:,i);
    current_radius = norm(current_sample - center_ball_Xi_hat,2);
    if (current_radius > previous_radius)
        previous_radius = current_radius;
    end
end

radius_ball_Xi_hat = previous_radius;

end

