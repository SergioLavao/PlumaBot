%Calculo de la matriz de transformaci?n Homogenea Rot de 4x4 para un robot articulado de
%n grados de libertad

%Entradas:
%DH = matriz Denavit-Hartenberg
%q     = Matriz de posiciones articulares seg?n trayectorias (1xn)

%Salidas:
%r = matriz basica de rotacion de 3x3
%p = vector de posicion de 3x1.

%Copyright: Julian Colorado.

function [r, p]=homogeneus_matrix(DH,q)

   alfa = DH(1,1);
   a = DH(1,2);
   sigma = DH(1,5); 
    
   if(sigma==0)  %Evalua si la articulacion es rotacional o prismatica
        teta=q;  %rotacional
        d=DH(1,4);  
        %Halla transformacion homogenea
        Rot=[cos(teta)  -cos(alfa)*sin(teta)  sin(alfa)*sin(teta)    a*cos(teta);
            sin(teta)    cos(alfa)*cos(teta)   -sin(alfa)*cos(teta)  a*sin(teta);
            0               sin(alfa)                cos(alfa)            d;
            0               0                           0                 1 ];
      else
        d=q;     %prismatica
        teta=DH(1,3);  
        %Halla transformacion homogenea
        Rot=[cos(teta)  -cos(alfa)*sin(teta)  sin(alfa)*sin(teta)         0;
            sin(teta)    cos(alfa)*cos(teta)   -sin(alfa)*cos(teta)       0;
            0               sin(alfa)                cos(alfa)            d;
            0               0                           0                 1 ];
   end
   % Se Extrae r y p de Rot
   r = Rot(1:3,1:3);
   p = Rot(1:3,4);
    
end
     