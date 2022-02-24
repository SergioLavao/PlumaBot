function T = ForwardKinematics( Robot, q)
    
    n = size(q);
    T = eye(4);
    
    for i = 1: 6

        alpha = Robot.alpha(i);
        a = Robot.a(i);

        Ca = cos(alpha);
        Sa = sin(alpha);
    
        if Robot.offset(i) == 0
            
            tetha = q(i);
            d = Robot.d(i);
            
            Ct = cos(tetha);
            St = sin(tetha);
            
            A = [ Ct  -Ca*St Sa*St a*Ct;
                  St  Ca*Ct -Sa*Ct a*St;
                  0   Sa     Ca       d;
                  0    0      0       1];

        else

            tetha = Robot.tetha(i);
            d = q(i);

            Ct = cos(tetha);
            St = sin(tetha);

            A = [ Ct  -Ca*St Sa*St 0;
                  St  Ca*Ct -Sa*Ct 0;
                  0   Sa     Ca    d;
                  0    0      0    1];

        end
        T = T*A;
    end
end