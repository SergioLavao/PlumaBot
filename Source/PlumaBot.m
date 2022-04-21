clc
clear all

[ROBOT, PARAM] = pluma_param();

%line = [0.14 0.1 -0.007; 0.071 0.11 -0.007];%Taller
%line2 = [0.071 0.11 -0.007; 0.14 0.09 -0.007];%Taller
% Traj = Traj_Planner(1,line,0.01,0.4,0.1);
% Traj = [Traj ; Traj_Planner(1,line2,0.01,0.4,0.1)];
% Traj(1,:) = [];

Traj = DrawTrajPlanner();

[PT, axis] = size(Traj)

T = eye(4);

ROBOT.n

q_final(1,:) = [0 0];

for i = 1 : PT

    Traj(i,2) = Traj(i,2) + 0.05;
    T(1:3,4) = Traj(i,1:3)';
    
    q_final( i + 1 , :) = InverseKinematics( ROBOT , T , q_final(i,:) );
    TFK = ForwardKinematics( ROBOT , q_final( i + 1, :) );
    TTest(i,:) = TFK(1:3,4)';

end


plot3(Traj(:,1), Traj(:,2),Traj(:,3), '-*')
hold on
plot3(TTest(:,1), TTest(:,2),Traj(:,3), '-+')
grid on
hold on

figure

ROBOT.plot3d(q_final, 'path', 'C:\Users\sergi\Desktop\SergioLavao\PlumaBot\Source\Model', 'workspace',[-0.25 0.25 -0.25 0.25 -0.05 0.1]);
hold on