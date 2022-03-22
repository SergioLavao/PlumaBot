clear all
[ROBOT, PARAM] = puma_param();

line = [0.5 0.1 0.2; 0.6 0.4 -0.2];
circle = [0.5 0.1 0.2; 0.7 0 0.2; 0.5 -0.1 0.2; 0.3 0 0.2; 0.5 0.1 0.2];

TrajCubic = Traj_Planner(2, circle, [1 2 3 4 5],[0 0 0 ; 0 0 0],0.1);
TrajPrism = Traj_Planner(1,line,1,0.2,0.05);

TrajTotal = [TrajCubic;TrajPrism]

[PT, axis] = size(TrajPrism);
[PTCUBIC, axis] = size(TrajCubic);
[PTTotal, axis] = size(TrajTotal);

T = eye(4);

q_traj(1,:) = [0 0 0 0 0 0];
IdealTraj(:,:,1) = zeros(4);
q_trajRobot(1,:) = [0 0 0 0 0 0];

q = q_trajRobot(1,:);

for i = 1 : PTTotal

    T(1:3,4) = TrajTotal(i,:);
    
    %q_trajRobot( i + 1 ,:) = ROBOT.ikine( T );
    q_trajRobot( i + 1 , :) = InverseKinematics( ROBOT , T ,  q_trajRobot( i ,:) );
end

plot3d( ROBOT, q_trajRobot);
hold on
TrajCubic = Traj_Planner(2, circle, [1 2 3 4 5],[0 0 0 ; 0 0 0],0.1);
hold on
TrajPrism = Traj_Planner(1,line,1,0.2,0.05);
hold on
