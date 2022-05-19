Grav = zeros(6,1)
Fext = zeros(6,1)

% line = [0.2 0 0;0.0 0.2 0];%Taller

% Traj = Traj_Planner(1,line,0.01,0.2,0.05);

% Draw = DrawTrajPlanner();
% 
% Draw(:,2) = Draw(:,2) + 0.05;
% 
% Traj = [Traj_Planner(1,line,0.01,0.2,0.05); Draw]

[PT, axis] = size(Traj)

T = eye(4);

lc_1 = 0.1;
lc_2 = 0.1;

L1=Link([  0     0     lc_1        0       0  ],'standard');
L2=Link([  0     0     lc_2        0       0  ],'standard');  

ROBOT = SerialLink([L1 L2]);
ROBOT.name='PLUMA';

ROBOT.n;

        %m      Sx      Sy  Sz   Ixx      Iyy      Izz      Jm	     G        B          Tc+       Tc- 
PARAM = [0.05   2.837    0   0    0.1      0        0        0        0          0        0       0;
         0.025  2.82    0   0    0.1      0        0        0        0          0        0      0];  

ROBOT.n

% q(1,:) = [0 0];
% qd(1,:) = [0 0];
% qdd(1,:) = [0 0];

for i = 1 : PT

%     T(1:3,4) = Traj(i,1:3)';
%     
%     q( i + 1 , :) = ikine(ROBOT, T,  'q0', q( i , :), 'mask',[1, 1, 0, 0, 0, 0]);
% 
%     T = ForwardKinematics( ROBOT, q(i + 1,:));
%     
%     POS_FK(i,:) = T(1:3,4)'; 
% 
%     J_d = jacobe( ROBOT, q( i ,:) );
% 
%     qd(i + 1,:) = q(i + 1,:) - q(i,:);%pinv( J_d(1:3,:) ) * Traj(i,4:6)'; %Velocidad articular
% 
%      qdd(i,:) = qd(i + 1,:) - qd(i,:); %Aceleración articular

    Tau(i,:) = ID( ROBOT, PARAM, q( i ,:), qd(i,:), qdd(i,:), Fext, Grav, '0' )

end

m_1 = PARAM(1,1);
m_2 = m_1;

Torque_Time = 0:Traj(2,10):(PT - 1)*Traj(2,10);
 
% qd(1,:) = [];

qd1_Sim = [Torque_Time' qd(:,1)];
qd2_Sim = [Torque_Time' qd(:,2)];

Tau1_Sim = [Torque_Time' Tau(:,1)];
Tau2_Sim = [Torque_Time' Tau(:,2)];

sim('Euler_Lagrange2DoF',(PT - 1)*Traj(2,10))