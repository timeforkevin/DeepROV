function [ y ] = IMU_model( x, x_1, dt, rov )
%Measurement Model for the IMU
%   Detailed explanation goes here
    y = zeros(9, 1);
    
    % Inertial Frame to Body Frame
    rol = x(4);
    pit = x(5);
    yaw = x(6);
    R = eul2rotm([yaw, pit, rol]);
    
    % Body Frame Rates
    v_body =   [  R*x(7:9);   R*x(10:12)];
    v_body_1 = [R*x_1(7:9); R*x_1(10:12)];
    
    % Body Frame Acceleration
    a_body = (v_body - v_body_1)/dt;
    
    % IMU offset error vector
    p = v_body(4); % Roll Rate
    q = v_body(5); % Pitch Rate
    r = v_body(6); % Yaw Rate
    dp = a_body(4);
    dq = a_body(5);
    dr = a_body(6);

    OE = [(q^2+r^2), -(p*q-dr), -(p*r+dq);
         -(p*q+dr), (p^2+r^2), -(q*r-dp);
         -(p*r-dq), -(p*r+dp), (p^2+q^2);];
    
    % Body Frame Gravity vector
    g_body = R*[0;0;9.81;];
    
    % Body Frame Magnetic vector
    mag_body = R*rov.IMU.decl;
    
    y(1:3) = g_body + a_body(1:3) + OE*rov.IMU.off;
    y(4:6) = v_body(4:6);
    y(7:9) = mag_body;
    
    Ey = rov.IMU.REy*sqrt(rov.IMU.Rey)*randn(9,1); %noise
    y = y+Ey;
end

