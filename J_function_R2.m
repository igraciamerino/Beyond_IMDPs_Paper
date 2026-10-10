function s = J_function_R2(x,state_discretization,user_param)
% Assign state q of the abstraction to state x of the original system iff x
% \in q

X_center = state_discretization.X_center;
Y_center = state_discretization.Y_center;
Deltax = state_discretization.Deltax;
Deltay = state_discretization.Deltay;
Ns = user_param.Ns;
for s = 1:Ns
    x_lower_left_s = X_center(s)-Deltax/2;
    y_lower_left_s = Y_center(s)-Deltay/2;
    x_upper_right_s = X_center(s)+Deltax/2;
    y_upper_right_s = Y_center(s)+Deltay/2;
    if ( ( x_lower_left_s <= x(1) ) && ( x_upper_right_s >= x(1) ) &&( y_lower_left_s <= x(2) ) && ( y_upper_right_s >= x(2) ) )
        return;
    end
end

if (s == Ns)
    s = Ns + 1;
end

end

