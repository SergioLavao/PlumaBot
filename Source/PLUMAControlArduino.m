Ts = 0.01;

qd(2,:) = [0 0];

Max_Vel_1 = max(qd(:,1)) 
Max_Vel_2 = max(qd(:,2)) 

if Max_Vel_1 >= Max_Vel_2
    Max_Vel = Max_Vel_1;
else
    Max_Vel = Max_Vel_2;
end

VelMotor = 150;

qd1_normalized = (-qd(:,1) / Max_Vel) * VelMotor;
qd2_normalized = (qd(:,2) / Max_Vel)  * VelMotor;

max( qd1_normalized )
max( qd2_normalized )

serialportlist("available")
arduino= serialport('COM3',9600);

[PT, axis] = size(qd)

for i = 1: PT 
    
    if(abs(qd1_normalized(i)) == 10)
       qd1_normalized(i) = qd1_normalized(i) - 1; 
    end

    if(abs(qd2_normalized(i)) == 10)
       qd2_normalized(i) = qd2_normalized(i) - 1; 
    end
 
    fwrite(arduino, round(qd1_normalized(i)), 'int16'); 
     
%     pause(0.001)

    fwrite(arduino, round(qd2_normalized(i)), 'int16'); 

%     pause(0.001)

    fwrite(arduino, 10, 'int8'); 
    
    disp([round(qd1_normalized(i)) round(qd2_normalized(i)) i])

    pause(Ts)

end

fwrite(arduino, 0, 'int16'); 
    
fwrite(arduino, 0, 'int16'); 

fwrite(arduino, 10, 'int8'); 


delete(arduino)