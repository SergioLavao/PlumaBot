function Robot_Traj = Traj_Planner( Type, Traj, Input3, Input4, Ts)
   
    if Type == 1
       
        AP = Input4;
        VelMax = Input3;
       
        T = norm(Traj(2, :) - Traj(1, :)) / (VelMax * (1 - AP));
               
        n = T / Ts;
        tao = T * AP;

        k = round( AP * n );

        a = VelMax / tao;

        P_tao = ((0.5)*a*tao*tao) .* (( Traj(2,:) - Traj(1,:) ) / norm(Traj(2,:) - Traj(1,:))) + Traj(1,:);
        P_Ttao =  ( (VelMax*(T-(2*tao))* (Traj(2,:) - Traj(1,:))) / norm( Traj(2,:) - Traj(1,:) ) ) + P_tao;

        %Tramo 1
        t = [ 0 : Ts : tao];
        for i = 1: k
            POS(i,:) = 1/(norm(P_tao-Traj(1,:))) * (0.5*a*t(i)*t(i)) * (P_tao - Traj(1,:)) + Traj(1,:);
        end

        %Tramo 2
        t2 = [ tao : Ts : T - tao] - tao;
        k2 = round(n - 2*k);
        for i = 1: k2
             POS(i + k,:) = ( (VelMax*t2(i)* (P_Ttao - P_tao) )/ norm(P_Ttao - P_tao) ) + P_tao;
        end

        %Tramo 3
        for i = 1: k
            POS(i + k + k2,:) = ( 1/norm(Traj(2,:) - P_Ttao) ) * ( ((-0.5*a*t(i)*t(i)) + VelMax * t(i)) * (Traj(2,:) - P_Ttao) ) + P_Ttao;
        end
               
        plot3( POS(:,1), POS(:,2), POS(:,3), 'o' )
        hold on
        grid on
   
    end
   
    if Type == 2
       
        Velocities = Input4;
        times  = Input3;
        Ts = 0.01;

        n = size(Traj); n = n(1);

        A = eye(n)

        for i=1:n-1
            h(i) = times( i + 1 ) - times(i);
        end

        A(1,1) = 2*h(1); A(1,2) = h(1);
        for i=2:n-1
            A(i,i) = 2*( h(i) + h(i-1) );
            A(i, i-1) = h(i-1);
            A(i, i+1) = h(i);

            CoefsFunc(i,1) = ((3/h(i))*(Traj(i+1,1)-Traj(i,1))) - ((3/h(i-1))*(Traj(i,1)-Traj(i-1,1)));
            CoefsFunc(i,2) = ((3/h(i))*(Traj(i+1,2)-Traj(i,2))) - ((3/h(i-1))*(Traj(i,2)-Traj(i-1,2)));
            CoefsFunc(i,3) = ((3/h(i))*(Traj(i+1,3)-Traj(i,3))) - ((3/h(i-1))*(Traj(i,3)-Traj(i-1,3)));

        end
        A(n,n) = 2*h(n-1); A(n,n-1) = h(n-1);

        for i=1:3
            CoefsFunc(1,i) = ((3/h(1))*(Traj(2,i) - Traj(1,i))) - 3*Velocities(1,i);
            CoefsFunc(n,i) = -((3/h(n-1))*(Traj(n,i) - Traj(n-1,i))) + 3*Velocities(2,i);
        end

        b = inv(A)*CoefsFunc;
        
        
        S = [0 0 0];

        for i=1:n-1

            pt = h(i)/Ts;
            t = [0:Ts:h(i)];

            for j=1:3

                a(i,j) = (b(i+1,j) - b(i,j))/(3*h(i));
                c(i,j) = ((1/h(i))*(Traj(i+1,j) - Traj(i,j))) - (h(i)*(2*b(i,j) + b(i+1,j))/3);

                for k=1:pt
                    CUR_POS(k,j) = a(i,j)*t(k)^3 + b(i,j)*t(k)^2 + c(i,j)*t(k) + Traj(i,j);
                end

            end
            S = [S; CUR_POS];
            plot3( S(:,1), S(:,2), S(:,3), 'o' );
            hold on
            grid on
           
        end
        S(1,:) = [];
        POS = S
    end

    Robot_Traj = POS;
end