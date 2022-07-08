[ROBOT, PARAM] = puma_param();
[q,qd,qdd] = joint_traj_puma560();
[pt, n] = size(q);

tarj_points = zeros( n , 3 );

for j = 1: pt

    T_x = ForwardKinematics( ROBOT, q(j, :) );
    tarj_points( j , : ) = transpose( T_x( 1:3 , 4 ) );

end

teach( ROBOT, q(1,:) )
