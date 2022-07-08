function tau = ID(ROBOT,PARAM,q,qd,qdd,grav,Fext,friction)

V0 = zeros(6,1);        %velocidad de la base = 0[m/s]
dV0 = grav;          %aceleracion de la base = grav 
n = ROBOT.n;            %Grados de libertad

DH(:,1) = ROBOT.alpha';
DH(:,2) = ROBOT.a';
DH(:,3) = q(1,1:n)';
DH(:,4) = ROBOT.d';
for u=1:n
   DH(u,5) = ROBOT.links(u).isprismatic;  %sigma
end
[pt,n]=size(q);

%Ciclo para cada punto de trayectoria
for k=1:pt
%**********************************************************************
% A. Ciclo para calcular V y dV desde i=1 hasta n 
%**********************************************************************
  for i=1:n   
    %**********************************************************************
    % Calculo de la cinematica directa para hallar r y p.
    %**********************************************************************   
    if(i==1) % La relacion entre los frame 1 y 0 (base) es:
      r=eye(3,3);	    %matriz basica de rotacion r = identidad (I)
      p=zeros(3,1);     %vector de translacion p=[0 0 0]'
    else	    % Para el resto de frames:   
      [r, p] = homogeneus_matrix(DH(i-1,:), q(k,i-1));
    end
    
    %**********************************************************************
    % Matrices espaciales 6x6 Proyeccion (H), Orientacion (R), translacion (P)
    %**********************************************************************
    H = [0; 0; 1-DH(i,5); 0; 0; DH(i,5)];   %vector H de proyeccion sobre eje de movimiento (6x1)
    R = [r zeros(3,3);zeros(3,3) r];        %Matriz espacial de orientacion R (6x6)
    Skew_p=[0 -p(3,1) p(2,1); p(3,1) 0 -p(1,1); -p(2,1) p(1,1) 0];  %Matriz skew_simetrica de p
    P=[eye(3,3) Skew_p;zeros(3,3) eye(3,3)]; %Matriz espacial de traslacion P (6x6)
    %**********************************************************************
    % 1. CALCULO DE LA EC.DE VELOCIDAD ESPACIAL
    % V[i] = R'*P*V[i-1] + H*qd[i];
    %**********************************************************************
    if(i==1) 
        V(:,i) = R'*P*V0 + H*qd(k,i);         %Efecto de la base: V0 y dV0
    else
        V(:,i) = R'*P*V(:,i-1) + H*qd(k,i);   %Velocidad espacial para el resto de cuerpos. 
    end %CHECK
    %**********************************************************************
    %Calculo de terminos que componen la ec. de aceleracion
    % dV[i] = d[R'*P]*V[i-1] + R'*P*dV[i-1]+ H*qdd[i] + dH*qd[i];
    %**********************************************************************
    % Para hallar la derivada de R,P y H, es necesario hallar el
    % operador 6-dimensional(SKW_va)compuesto por la skew-simetrica del vector de
    % velocidad en i-i. As? mismo la derivada de H requiere el calculo de skew-simetrica del vector de
    % velocidad en i (SKW_v)
    if i==1
       Skew_va = [0 -V0(3) V0(2);V0(3) 0 -V0(1);-V0(2) V0(1) 0]; %skew-simetrica de V[i-1]; efecto base
    else
       Skew_va = [0 -V(3,i-1) V(2,i-1);V(3,i-1) 0 -V(1,i-1);-V(2,i-1) V(1,i-1) 0]; %skew-simetrica de V[i-1]
    end    
    SKW_va = [Skew_va zeros(3,3);zeros(3,3) Skew_va];   %Termino espacial para derivadas dependientes de V[i-1]
    Skew_v = [0 -V(3,i) V(2,i);V(3,i) 0 -V(1,i);-V(2,i) V(1,i) 0]; %skew-simetrica de V[i]
    SKW_v = [Skew_v zeros(3,3);zeros(3,3) Skew_v];   %Termino espacial para derivadas dependientes de V[i]
    %******************************************************************************************************
    % 2. CALCULO DE LA EC.DE ACELERACION ESPACIAL
    %**********************************************************************
    if(i==1)
        dV(:,i) = SKW_va*(R'*P)*V0 + R'*P*dV0 + H*qdd(k,i) + SKW_v*H*qd(k,i); %Efecto de la base: V0 y dV0
    else
        dV(:,i) = SKW_va*(R'*P)*V(:,i-1) + R'*P*dV(:,i-1) + H*qdd(k,i) + SKW_v*H*qd(k,i); %Aceleracion espacial para el resto de cuerpos. 
    end  
 end     %cierra ciclo propagaci?n hacia adelante
 
%**********************************************************************
% B. Ciclo para calcular F (Fuerzas espaciales) desde i=n hasta 1 (recurrencia hacia atr?s) 
% F = P'*R*F[i+1] + I[i]*dV[i] + [dI[i] - I[i]*S'[i]]*V[i]
%**********************************************************************
for i=n:-1:1   
    %**********************************************************************
    % Calculo de la cinematica directa para hallar r y p (en propagacion hacia atras)
    %********************************************************************** 
    [r, p] = homogeneus_matrix(DH(i,:), q(k,i));    
    %**********************************************************************
    % Matrices espaciales 6x6 para Orientacion (R), translacion (P) y (S)
    %**********************************************************************
    R = [r zeros(3,3);zeros(3,3) r];        %Matriz espacial de orientacion R (6x6)
    Skew_p = [0 -p(3,1) p(2,1); p(3,1) 0 -p(1,1); -p(2,1) p(1,1) 0];  %Matriz skew_simetrica de p
    P=[eye(3,3) Skew_p;zeros(3,3) eye(3,3)]; %Matriz espacial de traslacion P (6x6)
    %Vector de translacion espacial(S[i]) entre el frame i y el centro de masa SM 
    s = PARAM(i,2:4);   %Distancia al centro de masa (s) medido en el marco de referencia i-1.
    skew_s = [0 -s(3) s(2);s(3) 0 -s(1);-s(2) s(1) 0]; %skew-simetrica del vector s
    S=[eye(3,3) skew_s;zeros(3,3) eye(3,3)]; %Operador espacial de translacion S
    %**********************************************************************
    % Operador espacial de inercia I[i]
    %**********************************************************************
    Ticm=[PARAM(i,5) 0 0;0 PARAM(i,6) 0;0 0 PARAM(i,7)];    %tensor de inercia del cuerpo i con respecto CM [Ixx, Iyy, Izz]
    Icm=[Ticm zeros(3,3);zeros(3,3) PARAM(i,1)*eye(3,3)];   %Operador espacial de inercia Icm en el centro de masa
    I = S*Icm*S'; %aplicando ejes paralelos: se halla el Operador espacial de inercia con respecto al frame i
    %**********************************************************************
    % Derivadas temporales de los operadores I y S: dI, dS
    %**********************************************************************
    Skew_v = [0 -V(3,i) V(2,i);V(3,i) 0 -V(1,i);-V(2,i) V(1,i) 0]; %skew-simetrica de V[i]
    SKW_v = [Skew_v zeros(3,3);zeros(3,3) Skew_v];   %Termino espacial para derivadas dependientes de V[i]
    dI = SKW_v*I; % derivada de I con respecto al tiempo
    dS = SKW_v*S; % derivada de S con respecto al tiempo
    %******************************************************************************************************
    % 2. CALCULO DE LA EC.DE FUERZA ESPACIAL
    %**********************************************************************
    if(i==n)
        F(:,i) = P'*R*Fext + I*dV(:,i) + (I*dS + dI)*V(:,i);  %efecto de la fuerza externa
    else
        F(:,i) = P'*R*F(:,i+1) + I*dV(:,i) + (I*dS + dI)*V(:,i);%Fuerza espacial para el resto de cuerpos.
    end 
    
    tau(k,i)  = H'*F(:,i);  %Torque efectivo de cada articulaci?n.
    
    
    % Add motor related friction components (B, Tc+,Tc-), viscours and colombic
        % Consider motor reduction and motor rotor inertia
        % Jm,G,B,Tc+,Tc-

        if (friction == 'f')
                tau(k,i) = tau(k,i) + PARAM(i,8)*qdd(k,i)*PARAM(i,9)^2; % gear reduction
                
                if(qd(k,i)<0)
                    mfriction = PARAM(i,10)*qd(k,i) + PARAM(i,12);
                end
                
                if(qd(k,i)>0)
                    mfriction = PARAM(i,10)*qd(k,i) + PARAM(i,11);
                end
                
                if(qd(k,i)==0)
                    mfriction = PARAM(i,11);
                end
                    
                tau(k,i) = tau(k,i) + mfriction; % compute motor referred friction
                
        end
    

  end %cierra ciclo propagaci?n hacia atr?s
 
end %cierra ciclo trayectoria
end


