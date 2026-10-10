function [Q_obstacle] = define_obstacle(Nx_Q_obstacle,Ny_Q_obstacle,move_Q_obstacle,Nx,Nq)

Q_obstacle = [];
for k = 1:Ny_Q_obstacle
    Q_obstacle = [Q_obstacle linspace((k-1)*Nx+1,(k-1)*Nx+Nx_Q_obstacle,Nx_Q_obstacle)];
end
Q_obstacle = Q_obstacle + move_Q_obstacle;

end