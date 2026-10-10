
clear
clc
close all

%%

A = [0.5 0.2; 0 0.5]';
b = [0.7,1.2]';
b_w = [2.4, 0]';

s1 = zonotope( interval( [0, -2]', [1, -1]' ) );

c1 = zonotope( interval([-1]', [-0.6]') );
c2 = zonotope( interval([-0.6]', [-0.2]') );
c3 = zonotope( interval([-0.2]', [0.2]') );
c4 = zonotope( interval([0.2]', [0.6]') );
c5 = zonotope( interval([0.6]', [1]') );

R1 = plus( mtimes(A, s1), mtimes(b_w, c1) ) + b;
R2 = plus( mtimes(A, s1), mtimes(b_w, c2) ) + b;
R3 = plus( mtimes(A, s1), mtimes(b_w, c3) ) + b;
R4 = plus( mtimes(A, s1), mtimes(b_w, c4) ) + b;
R5 = plus( mtimes(A, s1), mtimes(b_w, c5) ) + b;

figure(1)
hold on
plot(R1)
plot(R2)
plot(R3)
plot(R4)
plot(R5)
grid
ylim([0,1])
xlim([-2,4])


