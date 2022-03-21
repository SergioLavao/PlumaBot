function q = InverseKinematics( Robot, T, q_past )
   
    J_d = Jacobian( Robot, q_past );

    T_Past = ForwardKinematics( Robot, q_past );
    T_delta = T(1:3,4) - T_Past(1:3,4);
        
    q = ( pinv( J_d(1:3,:) ) * T_delta ) + transpose( q_past );% [ n x 1 ]; 

end