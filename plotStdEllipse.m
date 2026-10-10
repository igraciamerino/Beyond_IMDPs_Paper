function plotStdEllipse(data, nStdDevs)
% Data must be 2D

mu = mean(data, 2);
Std = nStdDevs^2*cov(data');
Std_inv = inv(Std);
hold on
% f1 = @(x,y) [x y]*(Std\[x; y]) - 1;
f1 = @(x,y) Std_inv(1,1)*(x - mu(1)).^2 + 2*Std_inv(1,2)*(x - mu(1)).*(y - mu(2)) + Std_inv(2,2)*(y - mu(2)).^2 - 1;

fimplicit(f1, 'b');

end