function J = Jacobian( Robot,q )
    
    n = Robot.n;
    
    for j = n : -1 : 1
    
        RobotIK.a = Robot.a(j:6);
        RobotIK.d = Robot.d(j:6);
        RobotIK.theta = Robot.theta(j:6);
        RobotIK.alpha = Robot.alpha(j:6);
        RobotIK.offset = Robot.offset(j:6);
        RobotIK.n = 7 - j;
    
        T = ForwardKinematics( RobotIK , q(j:6) );
      
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
    
    J = J(1:3,:);

end