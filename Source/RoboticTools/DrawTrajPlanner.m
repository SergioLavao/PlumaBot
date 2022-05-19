function Traj_Draw = DrawTrajPlanner(VelMax,Ts)

% drawPoints = [ 0.00 0.200 0.000 ; 
%                 0.1 0.1 0.000 ; 
%                -0.1 0.1 0.000 ; 
%                0.00 0.200 0.000 ;];

drawPoints = [ -0.00 0.200 0.000 ; 
0.05 0.145 0.000 ; 
0.08 0.083 0.000 ; 
0.08 0.050 0.000 ; 
0.07 0.072 0.000 ; 
0.03 0.072 0.000 ; 
0.01 0.100 0.000 ; 
0.04 0.130 0.000 ; 
0.06 0.131 0.000 ; 
0.09 0.100 0.000 ; 
0.07 0.072 0.000 ; 
0.03 0.071 0.000 ; 
0.02 0.052 0.000 ; 
0.02 0.084 0.000 ; 
0.05 0.145 0.000 ;]

[n,j]=size(drawPoints);

Traj_Draw(1,:) = zeros(1,10);
NormalizedVelocity(1) = 0; 

 for i = 1 : n - 1

     Traject = drawPoints(i:i+1,:);
     Traj_Draw = [ Traj_Draw; Traj_Planner(1,Traject,VelMax,0.4,Ts)];

 end

 [PT , Nan] = size(Traj_Draw)
 

for i = 1: PT
    NormalizedVelocity(i) = norm(Traj_Draw(i,4:6));
end

figure()

plot(NormalizedVelocity(:))
title('Trajectory Velocity Profile (Magnitude)')
xlabel('Sample[n]') 
ylabel('Velocity[m/s]') 
hold on
grid on

figure()

end