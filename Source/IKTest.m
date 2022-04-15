clc
clear all

[ROBOT, PARAM] = puma_param();

line = [0.5 -0.1 0.2; 0.5 0.3 0.2];%Taller
circle = [0.5 0.1 0.2; 0.7 0 0.2; 0.5 -0.1 0.2; 0.3 0 0.2; 0.5 0.1 0.2];

TrajCubic = Traj_Planner(2, circle, [1 2 3 4 5],[0 0 0 ; 0 0 0],0.1);
TrajPrism = Traj_Planner(1,line,1,0.2,0.05);

T = eye(4);
T(1:3,4) = [0.6 -0.3 0.1];
q = ROBOT.ikine( T )

CurTraj = TrajPrism;

linetest = [0.5 0.1 0.2; 0.5 0.5 -0.4];%------------UNCOMMENT THIS FOR TEST MULTIPLE TRAJECTORIES!
TrajLine = Traj_Planner(1,linetest,0.3,0.2,0.1);%------------UNCOMMENT THIS FOR TEST MULTIPLE TRAJECTORIES!
%OPTIONAL: MULTIPLE TRAJECTORIES
CurTraj = [ TrajCubic ; TrajLine ];%------------COMMENT THIS TO DISABLE MULTIPLE TRAJECTORIES!

Traj_Test( ROBOT, CurTraj(:,1:3), [0 0 0 0 0 0], 'Total Trajectory')

[PT, axis] = size(CurTraj)

for i = 1: PT
    NormalizedVelocity(i) = norm(CurTraj(i,4:6));
end

figure()
plot(NormalizedVelocity(:))
title('Trajectory Velocity Profile (Magnitude)')
xlabel('Sample[n]') 
ylabel('Velocity[m/s]') 
hold on
grid on
