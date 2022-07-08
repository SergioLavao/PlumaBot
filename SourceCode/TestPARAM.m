Grav = zeros(6,1)
Fext = zeros(6,1)

T = eye(4);

lc_1 = 0.1;
lc_2 = 0.1;

L1=Link([  0     0     lc_1        0       0  ],'standard');
L2=Link([  0     0     lc_2        0       0  ],'standard');  

ROBOT = SerialLink([L1 L2]);
ROBOT.name='PLUMA';

ROBOT.n;

        %m  Sx   Sy    Sz   Ixx      Iyy      Izz      Jm	     G        B          Tc+       Tc- 
PARAM = [2  1.4    0   0    0.1          0        0        0        0          0        0       0;
         1  1.43    0   0    0.1      0        0        0        0          0        0      0];  

ROBOT.n

for i = 1 : PT

    Tau(i,:) = ID( ROBOT, PARAM, q( i ,:), qd(i,:), qdd(i,:), Fext, Grav, '0' )

end

m_1 = PARAM(1,1);
m_2 = m_1;

Torque_Time = 0:Traj(2,10):(PT - 1)*Traj(2,10);

qd(1,:) = [];

qd1_Sim = [Torque_Time' qd(:,1)];
qd2_Sim = [Torque_Time' qd(:,2)];

Tau1_Sim = [Torque_Time' Tau(:,1)];
Tau2_Sim = [Torque_Time' Tau(:,2)];