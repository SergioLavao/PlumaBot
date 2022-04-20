clear all
[ROBOT, PARAM] = puma_param();

q_traj(1,:) = [0 0 0 0 0 0];
q = [0 0 0 0 0 0];

n = ROBOT.n;

for j = n : -1 : 1

    ROBOTIK.a = ROBOT.a(j:6);
    ROBOTIK.d = ROBOT.d(j:6);
    ROBOTIK.theta = ROBOT.theta(j:6);
    ROBOTIK.alpha = ROBOT.alpha(j:6);
    ROBOTIK.offset = ROBOT.offset(j:6);
    ROBOTIK.n = 7 - j;

    T = ForwardKinematics( ROBOTIK , q(j:6) );
  
    n = T(1:3,1);
    o = T(1:3,2);
    a = T(1:3,3);
    p = T(1:3,4);

    J (1,j) = -n(1)*p(2)+n(2)*p(1);
    J (2,j) = -o(1)*p(2)+o(2)*p(1);
    J (3,j) = -a(1)*p(2)+a(2)*p(1);

    J (4,j) = n(3);
    J (5,j) = o(3);
    J (6,j) = a(3);

end

J = J(1:3,:)