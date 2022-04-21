function Traj_Draw = DrawTrajPlanner

drawPoint = [0.059 0.059 0;
              0.068 0.075 0;
              0.068 0.049 0;
              0.047 0.000 0;
              0.025 0.049 0;
              0.025 0.074 0;
              0.034 0.059 0;
              0.059 0.059 0;
              0.081 0.036 0;
              0.052 0.012 0;
              0.041 0.012 0;
              0.013 0.036 0;
              0.034 0.059 0];

[n,j]=size(drawPoint);

Traj_Draw(1,:) = zeros(1,10);

 for i = 1 : n - 1

     Traject = drawPoint(i:i+1,:);
     Traj_Draw = [ Traj_Draw; Traj_Planner(1,Traject,0.01,0.2,0.05)];

 end

 [PT , Nan] = size(Traj_Draw)
 
 plot3( Traj_Draw(:,1), Traj_Draw(:,2), Traj_Draw(:,3), '-o' );
 hold on
 grid on

 plot3( drawPoint(:,1), drawPoint(:,2), drawPoint(:,2).*0, '*' );
 hold on
 grid on

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