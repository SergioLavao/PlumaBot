
%Esta funcion contiene los par?metros cinem?ticos y din?micos del robot
%manipulador PUMA560 de n=6 grados de libertad rotacionales

%Copyright: Julian Colorado.


function [PUMA, PARAM] = puma_param()

%matriz de par?metros LINK:

%-Contiene la matriz DH con sus 4 par?metros geom?tricos: alpha[rad],a[m],theta[rad],d[m].
%sigma=0 (articulaci?n Rotacional), Sigma=1 (articulaci?n prism?tica)

 
 %LINK     theta   d     a       alpha  sigma
  L1=Link([  0     0     0.1        0       0  ],'standard');
  L2=Link([  0     -0.007     0.1   0       0  ],'standard');  

 %Ensamble del robot PUMA usando la funcion robot() del toolbox de robotica
 %de Peter Corke
 PUMA = SerialLink([L1 L2]);
 PUMA.name='PUMA';
 

%Visualizacion del robot.
q = [0 0];        %postura de cada articulaci?n [rad]

ForwardKinematics( PUMA, q , 2)

%[q,qd,qdd] = joint_traj_puma560();

% 
PUMA.plot3d(q , 'alpha', 0);
PUMA.teach(q);         

         %m     Sx      Sy     Sz    Ixx      Iyy      Izz      Jm	     G        B          Tc+       Tc- 
 PARAM = [2.5     0       0     0      0       0.35     0     0.0002   -62.61    0.00148    0.4       -0.43;
         17.4  0.3638  0.006  0.2275 0.13    0.524    0.539   0.0002    107.81   0.000817   0.13      -0.07;
         4.8   0.0203 -0.0141 0.070  0.066   0.086    0.0125  0.0002    53.71    0.00138    0.13      -0.1;
         0.82  0       0.019  0      1.8e-3  1.3e-3   1.8e-3  3e-005    76.0364  7e-005     0.0112    -0.0169; 
         0.34  0       0      0      0.3e-3  0.4e-3   0.3e-3  3e-005    71.923   9e-005     0.00926    0.0145;
         0.09  0       0      0.032  0.15e-3 0.15e-3  0.04e-3 3e-005    76.686   4e-005     0.00396    0.0105];
 
              
 
end
    