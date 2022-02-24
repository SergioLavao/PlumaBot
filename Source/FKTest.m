
[ROBOT, PARAM] = puma_param();
[q,qd,qdd] = joint_traj_puma560();
[pt, n] = size(q);

for j = 1: pt
    T_x = ForwardKinematics( ROBOT, q(j, :) );
    disp(T_x)
end

plot3d(ROBOT, q)