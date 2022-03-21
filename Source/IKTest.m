[ROBOT, PARAM] = puma_param();

circle =  [0.45 -0.11 0.43; 0.45 0.5 0.43]; %punto inicial-final
TrajCubic = Traj_Planner(2, circle, [1 2 3 4 5],[0 0 0 ; 0 0 0],0.1);

line =  [0.45 -0.11 0.43; 0.45 0.5 0.43]; %punto inicial-final
TrajPrism = Traj_Planner(1,line,1,0.2,0.05);

[PT, axis] = size(TrajPrism);
[PTCUBIC, axis] = size(TrajCubic);

T = eye(4);

q_traj(1,:) = [0 0 0 0 0 0];
IdealTraj(:,:,1) = zeros(4);
q_trajRobot(1,:) = [0 0 0 0 0 0];

for i = 1 : PT

    T(1:3,4) = TrajCubic(i,:);
    
    q_trajRobot( i + 1 ,:) = ROBOT.ikine( T );
    %q_trajRobot( i + 1 ,:) = InverseKinematics( ROBOT , T ,  q_trajRobot( i ,:) );

end

plot3d( ROBOT, q_trajRobot);
hold on
TrajCubic = Traj_Planner(2, circle, [1 2 3 4 5],[0 0 0 ; 0 0 0],0.1);
hold on
TrajPrism = Traj_Planner(1,line,1,0.2,0.05);
hold on
