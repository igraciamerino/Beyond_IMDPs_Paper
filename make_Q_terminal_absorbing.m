function [Q_obstacle,P_lower_with_obstacles,P_upper_with_obstacles] = make_Q_terminal_absorbing(Q_terminal,P_lower_with_obstacles,P_upper_with_obstacles)

% NOT FINISHED BECAUSE I THINK I DON'T NEED THIS!!!!!

Na = length(P_lower_with_obstacles(1,:,1));

% Make obstacle absorbing
for q = Q_terminal
    for a = 1:Na
        for q1 = 1:Nq+1
            P_lower_with_obstacles(q_obstacle_aux,a,q1) = 0;
            P_upper_with_obstacles(q_obstacle_aux,a,q1) = 0;
        end
        P_lower_with_obstacles(q_obstacle_aux,a,q_obstacle_aux) = 1;
        P_upper_with_obstacles(q_obstacle_aux,a,q_obstacle_aux) = 1;
    end
end
end

