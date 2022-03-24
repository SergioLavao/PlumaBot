function Traj_Test( Robot, Trajectory, q_ini, alias )

[PT, axis] = size(Trajectory);
T = eye(4);

q_trajRobot(1,:) = q_ini;
q_traOptjRobot( 1 ,:) = q_ini;

T_OptTraj = [0 0 0];%IK With control System
T_Traj = [0 0 0];%IK Without control, e = 0

for i = 1 : PT

    T(1:3,4) = Trajectory(i,:);
    
    q_traOptjRobot( i + 1 ,:) = Robot.ikine( T );%Optional, used for error measurement
    T_Temp = ForwardKinematics( Robot, q_traOptjRobot(i + 1 ,:) );%Optional, used for error measurement
    T_OptTraj = [ T_OptTraj ; transpose(T_Temp(1:3,4)) ];%Optional, used for error measurement

    q_trajRobot( i + 1 , :) = InverseKinematics( Robot , T , q_trajRobot(i,:) );

    T_Temp = ForwardKinematics( Robot, q_trajRobot(i + 1 ,:) );%Optional, used for error measurement
    T_Traj = [ T_Traj ; transpose(T_Temp(1:3,4)) ];%Optional, used for error measurement

    RMSE(i) = sqrt(sum((T_OptTraj(i,:) - T_Traj(i,:)) .^ 2)/3);%Optional, used for error measurement 
    
end

plot3d( Robot, q_trajRobot);
hold all

figure()

plot3( Trajectory(:,1), Trajectory(:,2), Trajectory(:,3), 'o' );
hold on
grid on

T_OptTraj(1,:) = [];
plot3( T_OptTraj(:,1), T_OptTraj(:,2), T_OptTraj(:,3), '-' );
hold on
grid on

T_Traj(1,:) = [];
plot3( T_Traj(:,1), T_Traj(:,2), T_Traj(:,3), '*' );
title(alias)
xlabel('x_{axis}[m]') 
ylabel('y_{axis}[m]')
zlabel('z_{axis}[m]') 
xlim([-0 1])
ylim([-0.5 0.5])
zlim([-0 0.5])
hold on
grid on

figure()
plot(RMSE(:))
%xlim([0 53])
ylim([0 0.5])
title('Root mean square error based on samples')
xlabel('Sample[n]') 
ylabel('RMSE[m]') 
hold on
grid on

end