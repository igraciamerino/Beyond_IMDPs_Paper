function s = J_function_R2C1(x,state_discretization,user_param)
% Assign state q of the abstraction to state x of the original system iff x
% \in q

X_center = state_discretization.X_center;
Y_center = state_discretization.Y_center;
Deltax = state_discretization.Deltax;
Deltay = state_discretization.Deltay;
if ( original_sys.n == 3 )
    Z_center = state_discretization.Z_center;
    Deltaz = state_discretization.Deltaz;
end
Ns = user_param.Ns;
for s = 1:Ns
    a_s(1) = X_center(s)-Deltax/2;
    a_s(2) = Y_center(s)-Deltay/2;
    b_s(1) = X_center(s)+Deltax/2;
    b_s(2) = Y_center(s)+Deltay/2;
    if ( original_sys.n == 2 )
        if ( ~any( a_s >= x ) && ~any( b_s <= x ) )
            return;
        end
    elseif ( original_sys.n == 3 )
        a_s(3) = Z_center(s)-Deltaz/2;
        b_s(3) = Z_center(s)+Deltaz/2;
        if ( ~any( a_s >= x ) && ~any( b_s <= x ) )
            return;
        end
    end   
end

if ( s == Ns )
    s = Ns + 1;
end

end

