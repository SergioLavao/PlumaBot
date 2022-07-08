function Tau = InverseDynamics( ROBOT, PARAM, q, qd, qdd, Fext, grav )

    n = ROBOT.n;

    for i = 1 : n
    
        [r p] = HomogeneousMatrix( ROBOT , i, q(i) );
        
        R = [r zeros(3,3);zeros(3,3) r];
        P = [eye(3) skew(p);zeros(3,3) eye(3)];
        H = [0 0 1-ROBOT.offset(i) 0 0 ROBOT.offset(i)]';

        if(i == 1)
            V(:,i) = R'*P*zeros(6,1) + H*qd(i); % [ 6x6 ] + [ 1x6 ]
        else
            V(:,i) = R'*P*V(:, i - 1 ) + H*qd(i); % [ 6x6 ] + [ 1x6 ]
        end

        W_actual = [skew(V(1:3,i)) zeros(3,3); zeros(3,3) skew(V(1:3,i))];

        if(i == 1)
            W_past = zeros(6,6);
        else
            W_past = [skew(V(1:3,i-1)) zeros(3,3); zeros(3,3) skew(V(1:3,i-1))];
        end

        Hd = W_actual*H;
        RPd = W_past * ( R'*P );

        if(i == 1)
            Vd(:,i) = R'*P*grav + H*qdd(i) + Hd*qd(i);
        else
            Vd(:,i) = R'*P*Vd(:,i-1) + RPd*V(:,i-1) + H*qdd(i) + Hd*qd(i);
        end
        
    end

    %Calculo fuerzas 
    for i = n: -1 : 1
    
        [r p] = HomogeneousMatrix( ROBOT , i, q(i) );
        R = [r zeros(3,3);zeros(3,3) r];
        P = [eye(3) skew(p);zeros(3,3) eye(3)];

        J_cm = [PARAM(i,5) 0 0;0 PARAM(i,6) 0;0 0 PARAM(i,7)];
        I_cm=[J_cm zeros(3,3);zeros(3,3) PARAM(i,1)*eye(3,3)];
        S = [eye(3,3) skew(PARAM(i,2:4));zeros(3,3) eye(3,3)];

        I = S*I_cm*S';

        W = [ skew(V(1:3,i)) zeros(3,3); zeros(3,3) skew(V(1:3,i))];

        Id = W*I;

        Sd = W*S;

        if(i == n)
            F(:,i) = P'*R*Fext + I*Vd(:,i) + ( I*Sd + Id )*V(:,i);  
        else
            F(:,i) = P'*R*F(:,i+1) + I*Vd(:,i) + ( I*Sd + Id )*V(:,i);
        end        
    
        Tau(i) = H'*F(:,i);

    end    
    
end