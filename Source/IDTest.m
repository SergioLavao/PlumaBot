[ROBOT, PARAM] = puma_param();
[q,qd,qdd] = joint_traj_puma560();

Grav = zeros(6,1)
Fext = zeros(6,1)

q_points = size(qdd)
%line = [0.14 0.1 -0.007; 0.071 0.11 -0.007];%Taller
%line2 = [0.071 0.11 -0.007; 0.14 0.09 -0.007];%Taller
% Traj = Traj_Planner(1,line,0.01,0.4,0.1);

for i = 1: q_points

    TauID(i,:) = InverseDynamics( ROBOT, PARAM, q(i,:), qd(i,:), qdd(i,:), Fext, Grav )
%     InverseDynamicsOG = ID( ROBOT, PARAM, q(i,:), qd(i,:), qdd(i,:), Fext, Grav,0 )

end
