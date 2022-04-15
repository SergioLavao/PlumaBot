function [PLUMA, PARAM] = pluma_param()

L1=Link([  0     0     0.1        0       0  ],'standard');
L2=Link([  0     -0.007     0.1   0       0  ],'standard');  

line = [0.1 0.1 0; 0.2 0 0];%Taller
Traj = Traj_Planner(1,line,0.01,0.2,0.05);

PLUMA = SerialLink([L1 L2]);
PLUMA.name='PLUMA';

T = eye(4);
[PT, axis] = size(Traj)

q_trajRobot = [0 0;
               0.2 0;
               0.3 0;
               0.5 0;
               0.6 0;
               0.8 0;
               1 0;
               1.2 0;
               1.5 0;];

q_final(1,:) = zeros(1,10);

PLUMA.n

for i = 1 : 2
    
    T = ForwardKinematics( PLUMA, q_trajRobot(i,:) );
    
    TTest(i,:) = T(1:3,4)';
   q(i) = PLUMA.ikinem( T );
   %q_final( i + 1 , :) = InverseKinematics( PLUMA , T , q_final(i,:) );

end

plot3(TTest(:,1), TTest(:,2),TTest(:,3), '-*')

hold on
grid on

PARAM = [2.5   0       0     0       0       0.35     0     0.0002   -62.61    0.00148    0.4       -0.43;
         17.4  0.3638  0.006  0.2275 0.13    0.524    0.539   0.0002    107.81   0.000817   0.13      -0.07;
         4.8   0.0203 -0.0141 0.070  0.066   0.086    0.0125  0.0002    53.71    0.00138    0.13      -0.1;
         0.82  0       0.019  0      1.8e-3  1.3e-3   1.8e-3  3e-005    76.0364  7e-005     0.0112    -0.0169; 
         0.34  0       0      0      0.3e-3  0.4e-3   0.3e-3  3e-005    71.923   9e-005     0.00926    0.0145;
         0.09  0       0      0.032  0.15e-3 0.15e-3  0.04e-3 3e-005    76.686   4e-005     0.00396    0.0105];
 
              
 
end    