function [n_f1,d_under_f1,d_over_f1,n_f2,d_under_f2,d_over_f2] = affine_under_and_overapproximate_JacksonNL(a_rectangle,b_rectangle,a,vel)

if (a == 1) % Mode 1
    
    % x1(1) = x(1) + 0.2*sin(x(2)) + 0.5;
    % x1(2) = x(2) + 0.4*cos(x(1));
    
    %Calculations for f1
    u1 = [1 0 1]';
    amplitude = 0.2;
    f_is_sine = 1;
    [m,x_under,x_over] = affine_under_and_overapproximate_sinusoidal(a_rectangle(2),b_rectangle(2),amplitude,f_is_sine);
    u2 = [0 1 m]';
    n_f1 = cross(u1,u2);
    v_under = [0, x_under, amplitude*sin(x_under) + vel]';
    v_over = [0, x_over, amplitude*sin(x_over) + vel]';
    d_under_f1 = n_f1'*v_under;
    d_over_f1 = n_f1'*v_over;
    
    %Calculations for f2
    
    u1 = [0 1 1]';
    amplitude = 0.4;
    f_is_sine = 0;
    [m,x_under,x_over] = affine_under_and_overapproximate_sinusoidal(a_rectangle(1),b_rectangle(1),amplitude,f_is_sine);
    u2 = [1 0 m]';
    n_f2 = cross(u1,u2);
    v_under = [x_under, 0, amplitude*cos(x_under)]';
    v_over = [x_over, 0, amplitude*cos(x_over)]';
    d_under_f2 = n_f2'*v_under;
    d_over_f2 = n_f2'*v_over;
   
elseif (a == 2) % Mode 2
%     x1(1) = x(1) + 0.2*sin(x(2)) - 0.5;
%     x1(2) = x(2) + 0.4*cos(x(1));

    
    %Calculations for f1
    u1 = [1 0 1]';
    amplitude = 0.2;
    f_is_sine = 1;
    [m,x_under,x_over] = affine_under_and_overapproximate_sinusoidal(a_rectangle(2),b_rectangle(2),amplitude,f_is_sine);
    u2 = [0 1 m]';
    n_f1 = cross(u1,u2);
    v_under = [0, x_under, amplitude*sin(x_under) - vel]';
    v_over = [0, x_over, amplitude*sin(x_over) - vel]';
    d_under_f1 = n_f1'*v_under;
    d_over_f1 = n_f1'*v_over;
    
    %Calculations for f2
    
    u1 = [0 1 1]';
    amplitude = 0.4;
    f_is_sine = 0;
    [m,x_under,x_over] = affine_under_and_overapproximate_sinusoidal(a_rectangle(1),b_rectangle(1),amplitude,f_is_sine);
    u2 = [1 0 m]';
    n_f2 = cross(u1,u2);
    v_under = [x_under, 0, amplitude*cos(x_under)]';
    v_over = [x_over, 0, amplitude*cos(x_over)]';
    d_under_f2 = n_f2'*v_under;
    d_over_f2 = n_f2'*v_over;
    
elseif (a == 3) % Mode 3
%     x1(1) = x(1) + 0.4*cos(x(2));
%     x1(2) = x(2) + 0.2*sin(x(1)) + 0.5;

    
    %Calculations for f1
    u1 = [1 0 1]';
    amplitude = 0.4;
    f_is_sine = 0;
    [m,x_under,x_over] = affine_under_and_overapproximate_sinusoidal(a_rectangle(2),b_rectangle(2),amplitude,f_is_sine);
    u2 = [0 1 m]';
    n_f1 = cross(u1,u2);
    v_under = [0, x_under, amplitude*cos(x_under)]';
    v_over = [0, x_over, amplitude*cos(x_over)]';
    d_under_f1 = n_f1'*v_under;
    d_over_f1 = n_f1'*v_over;
    
    %Calculations for f2
    
    u1 = [0 1 1]';
    amplitude = 0.2;
    f_is_sine = 1;
    [m,x_under,x_over] = affine_under_and_overapproximate_sinusoidal(a_rectangle(1),b_rectangle(1),amplitude,f_is_sine);
    u2 = [1 0 m]';
    n_f2 = cross(u1,u2);
    v_under = [x_under, 0, amplitude*sin(x_under) + vel]';
    v_over = [x_over, 0, amplitude*sin(x_over) + vel]';
    d_under_f2 = n_f2'*v_under;
    d_over_f2 = n_f2'*v_over;
    
elseif (a == 4) % Mode 4
%     x1(1) = x(1) + 0.4*cos(x(2));
%     x1(2) = x(2) + 0.2*sin(x(1)) - 0.5;

    %Calculations for f1
    u1 = [1 0 1]';
    amplitude = 0.4;
    f_is_sine = 0;
    [m,x_under,x_over] = affine_under_and_overapproximate_sinusoidal(a_rectangle(2),b_rectangle(2),amplitude,f_is_sine);
    u2 = [0 1 m]';
    n_f1 = cross(u1,u2);
    v_under = [0, x_under, amplitude*cos(x_under)]';
    v_over = [0, x_over, amplitude*cos(x_over)]';
    d_under_f1 = n_f1'*v_under;
    d_over_f1 = n_f1'*v_over;
    
    %Calculations for f2
    
    u1 = [0 1 1]';
    amplitude = 0.2;
    f_is_sine = 1;
    [m,x_under,x_over] = affine_under_and_overapproximate_sinusoidal(a_rectangle(1),b_rectangle(1),amplitude,f_is_sine);
    u2 = [1 0 m]';
    n_f2 = cross(u1,u2);
    v_under = [x_under, 0, amplitude*sin(x_under) - vel]';
    v_over = [x_over, 0, amplitude*sin(x_over) - vel]';
    d_under_f2 = n_f2'*v_under;
    d_over_f2 = n_f2'*v_over;
end
end

