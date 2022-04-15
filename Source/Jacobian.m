function J = Jacobian( Robot,q )
    
    n = Robot.n;
    
    for j = n : -1 : 1
        
        m = 6

        RobotIK.a = Robot.a(j:m);
        RobotIK.d = Robot.d(j:m);
        RobotIK.theta = Robot.theta(j:m);
        RobotIK.alpha = Robot.alpha(j:m);
        RobotIK.offset = Robot.offset(j:m);
        RobotIK.n = 7 - j;
    
        T = ForwardKinematics( RobotIK , q(j:m) );
      
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