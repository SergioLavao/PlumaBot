clear all

[ROBOT, PARAM] = puma_param();

line = [0.5 -0.1 0.2; 0.5 0.3 0.2];%Taller %line2 = [0.5 0.1 0.2; 0.6 0.4 -0.2];
circle = [0.5 0.1 0.2; 0.7 0 0.2; 0.5 -0.1 0.2; 0.3 0 0.2; 0.5 0.1 0.2];

TrajCubic = Traj_Planner(2, circle, [1 2 3 4 5],[0 0 0 ; 0 0 0],0.1);
TrajPrism = Traj_Planner(1,line,1,0.2,0.05);

T = eye(4);
T(1:3,4) = [0.6 -0.3 0.1];
q = ROBOT.ikine( T )

Traj_Test( ROBOT, TrajCubic(:,1:3), [0 0 0 0 0 0],'Prismatic Spline Trajectory')

[PT, axis] = size(TrajCubic)

for i = 1: PT
    NormalizedVelocity(i) = norm(TrajCubic(i,4:6));
end

figure()
plot(NormalizedVelocity(:))
title('Trajectory Velocity Profile (Magnitude)')
xlabel('Sample[n]') 
ylabel('Velocity[m/s]') 
hold on
grid on