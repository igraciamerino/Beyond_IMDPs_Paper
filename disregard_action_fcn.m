function yn = disregard_action_fcn(V_k, gamma_prev, V_k_s)
% Checks if the optimal action for the previous iteration is non-optimal
% WARNING: IF THE ANSWER IS NO,THE ACTION CAN STILL BE NON OPTIMAL

if (gamma_prev'*V_k < V_k_s)
    yn = true;
else
    yn = false;
end


end