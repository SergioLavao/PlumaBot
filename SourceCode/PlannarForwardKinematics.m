function X_Y = PlannarForwardKinematics( ROBOT, q )

    T = ForwardKinematics( ROBOT, q );
    
    X_Y = T(1:2,4);
    
end