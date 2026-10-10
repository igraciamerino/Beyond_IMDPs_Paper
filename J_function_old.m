function s = J_function(x,state_discretization,user_param)
% Assign state q of the abstraction to state x of the original system iff x
% \in q

X_center = state_discretization.X_center{1};
Y_center = state_discretization.X_center{2};
Deltax = state_discretization.Deltax{1};
Deltay = state_discretization.Deltax{2};
if ( length(x) == 3 )
    Z_center = state_discretization.X_center{3};
    Deltaz = state_discretization.Deltax{3};
elseif ( length(x) == 4 )
    Z_center = state_discretization.X_center{3};
    Deltaz = state_discretization.Deltax{3};
    U_center = state_discretization.X_center{4};
    Deltau = state_discretization.Deltax{4};
end
Ns = user_param.Ns;
a_s = x;
b_s = x;
for s = 1:Ns
    a_s(1) = X_center(s)-Deltax/2;
    a_s(2) = Y_center(s)-Deltay/2;
    b_s(1) = X_center(s)+Deltax/2;
    b_s(2) = Y_center(s)+Deltay/2;
    if ( length(x) == 2 )
        if ( ~any( a_s >= x ) && ~any( b_s <= x ) )
            return;
        end
    elseif ( length(x) == 3 )
        a_s(3) = Z_center(s)-Deltaz/2;
        b_s(3) = Z_center(s)+Deltaz/2;
        if ( ~any( a_s >= x ) && ~any( b_s <= x ) )
            return;
        end
    elseif ( length(x) == 4 )
        a_s(3) = Z_center(s)-Deltaz/2;
        b_s(3) = Z_center(s)+Deltaz/2;
        a_s(4) = U_center(s)-Deltau/2;
        b_s(4) = U_center(s)+Deltau/2;
        if ( ~any( a_s >= x ) && ~any( b_s <= x ) )
            return;
        end
    end   
end

if ( s == Ns )
    s = Ns + 1;
end

end

