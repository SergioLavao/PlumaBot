[ROBOT, PARAM] = puma_param();
[q,qd,qdd] = joint_traj_puma560();
[pt, n] = size(q);

tarj_points = zeros( n , 3 );
tarj_points_ik = zeros( n , 3 );
q_ik(1,:) = [0 0 0 0 0 0];

for j = 1: pt - 2

    T_x = ForwardKinematics( ROBOT, q(j, :) );
    tarj_points( j , : ) = transpose( T_x( 1:3 , 4 ) );

    q_ik(j + 1,:) = InverseKinematics( ROBOT, T_x, q_ik(j,:))

    T_ik = ForwardKinematics( ROBOT, q_ik(j, :) );
    tarj_points_ik( j , : ) = transpose( T_ik( 1:3 , 4 ) );

end

R = corrcoef( tarj_points , tarj_points_ik )

plot3( tarj_points( : , 1 ), tarj_points( : , 2 ), tarj_points( : , 3 ), '-o' )
hold on
grid on
plot3( tarj_points_ik( : , 1 ), tarj_points_ik( : , 2 ), tarj_points_ik( : , 3 ), '-*' )
title('Trajectory points')
xlabel('x_{axis}[m]') 
ylabel('y_{axis}[m]')
zlabel('z_{axis}[m]') 
