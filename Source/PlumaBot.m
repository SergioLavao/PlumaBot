clear all;
clc

[ROBOT, PARAM] = pluma_param();

Traj = DrawTrajPlanner(0.05,0.05);

Traj(1,:) = []
[PT, axis] = size(Traj);

T = eye(4);

q(1,:) = [0 0];
qd(1,:) = [0 0];
qdd(1,:) = [0 0];

for i = 1 : PT
% 
    T(1:3,4) = Traj(i,1:3)';
    
    q( i + 1 , :) = ikine(ROBOT, T,  'q0', q( 1 , :), 'mask',[1, 1, 0, 0, 0, 0]);
    qd(i + 1,:) = q(i + 1,:) - q(i,:);%pinv( J_d(1:3,:) ) * Traj(i,4:6)'; %Velocidad articular

    TFK_Toolbox = ForwardKinematics( ROBOT , q( i + 1, :) );

    Traj_FK(i,:) = TFK_Toolbox(1:3,4)';

end

plot3(Traj_FK(:,1), Traj_FK(:,2),Traj_FK(:,3), '-*')
hold on

% figure
% ROBOT.plot3d(q, 'path', 'C:\Users\sergi\Desktop\SergioLavao\PlumaBot\Source\Model', 'workspace',[-0.25 0.25 -0.25 0.25 -0.05 0.1]);
% ROBOT.T()
% hold on