clc
clear all

[ROBOT, PARAM] = pluma_param();

line = [0.14 0.1 -0.007; 0.071 0.11 -0.007];%Taller
line2 = [0.071 0.11 -0.007; 0.14 0.09 -0.007];%Taller
Traj = Traj_Planner(1,line,0.01,0.4,0.05);
Traj = [Traj ; Traj_Planner(1,line2,0.01,0.4,0.05)];
Traj(1,:) = [];

[PT, axis] = size(Traj)

T = eye(4);

q_trajRobot = [0 0;
               0.2 0;
               0.3 0;
               0.5 0;
               0.6 0;
               0.8 0;
               1 0;
               1.2 0;
               1.5 0;];

ROBOT.n

q_final(1,:) = [0 3*pi/4];
err = 1;

for i = 1 : PT

    T(1:3,4) = Traj(i,1:3)';
    
    q_final( i + 1 , :) = InverseKinematics( ROBOT , T , q_final(i,:) );
    TFK = ForwardKinematics( ROBOT , q_final( i + 1, :) );
    TTest(i,:) = TFK(1:3,4)';

end

% ROBOT.plot3d([0 3*pi/4], 'alpha', 0);
% ROBOT.teach(q_final);         
% hold on

plot3(Traj(:,1), Traj(:,2),Traj(:,3), '-*')
hold on
plot3(TTest(:,1), TTest(:,2),TTest(:,3), '-+')
grid on
