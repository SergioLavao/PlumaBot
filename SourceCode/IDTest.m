[ROBOT, PARAM] = puma_param();
[q,qd,qdd] = joint_traj_puma560();

Grav = zeros(6,1)
Fext = zeros(6,1)

q_points = size(qdd)

T_Traj = [0 0 0];%IK Without control, e = 0

for i = 1: q_points

%     TauID(i,:) = InverseDynamics( ROBOT, PARAM, q(i,:), qd(i,:), qdd(i,:), Fext, Grav )
    TauID(i,:) = ID( ROBOT, PARAM, q(i,:), qd(i,:), qdd(i,:), Fext, Grav, 0 )

%     T_Temp = ForwardKinematics( ROBOT, q(i,:) );%Optional, used for error measurement
%     T_Traj = [ T_Traj ; transpose(T_Temp(1:3,4)) ]%Optional, used for error measurement

end

figure
plot(TauID(:,:))
hold on 
grid on 

T_Traj(1,:) = [];
figure
plot3( T_Traj(:,1), T_Traj(:,2), T_Traj(:,3), '*' );
title('Trajectory')
xlabel('x_{axis}[m]') 
ylabel('y_{axis}[m]')
zlabel('z_{axis}[m]') 
hold on
grid on
