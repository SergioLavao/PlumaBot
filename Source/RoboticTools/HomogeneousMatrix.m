%Copyright: Julian Colorado.

function [r, p] = HomogeneousMatrix(ROBOT , joint , q )

   alfa = ROBOT.alpha(joint);
   a = ROBOT.a(joint);
   sigma = ROBOT.offset(joint); 
    
   if(sigma==0)  %Evalua si la articulacion es rotacional o prismatica
        teta=q;  %rotacional
        d=ROBOT.d(joint);  
        %Halla transformacion homogenea
        Rot=[cos(teta)  -cos(alfa)*sin(teta)  sin(alfa)*sin(teta)    a*cos(teta);
            sin(teta)    cos(alfa)*cos(teta)   -sin(alfa)*cos(teta)  a*sin(teta);
            0               sin(alfa)                cos(alfa)            d;
            0               0                           0                 1 ];
      else
        d=q;     
        teta=ROBOT.theta(joint);  

        Rot=[cos(teta)  -cos(alfa)*sin(teta)  sin(alfa)*sin(teta)         0;
            sin(teta)    cos(alfa)*cos(teta)   -sin(alfa)*cos(teta)       0;
            0               sin(alfa)                cos(alfa)            d;
            0               0                           0                 1 ];
   end

   r = Rot(1:3,1:3);
   p = Rot(1:3,4);
    
end
     