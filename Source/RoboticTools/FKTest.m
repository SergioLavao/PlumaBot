[ROBOT, PARAM] = puma_param();
[q,qd,qdd] = joint_traj_puma560();
[pt, n] = size(q);

tarj_points = zeros( n , 3 );

for j = 1: pt

    T_x = ForwardKinematics( ROBOT, q(j, :) );
    tarj_points( j , : ) = transpose( T_x( 1:3 , 4 ) );

end

plot3d( ROBOT, q )

plot3( tarj_points( : , 1 ), tarj_points( : , 2 ), tarj_points( : , 3 ), '-o' )
hold on