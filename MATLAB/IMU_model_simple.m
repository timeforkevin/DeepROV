function [ y ] = IMU_model_simple( x, x_1, dt, decl )
%Measurement Model for the IMU
%   Detailed explanation goes here


    y = zeros(9, 1);
    
    % Inertial Frame to Body Frame
    rol = x(4);
    pit = x(5);
    yaw = x(6);
    R = R3D(yaw, pit, rol);
    
    % Body Frame Rates
    v_body =   [  R*x(7:9);   R*x(10:12)];
    v_body_1 = [R*x_1(7:9); R*x_1(10:12)];
    
    % Body Frame Acceleration
    a_body = (v_body - v_body_1)/dt;
    
    % Body Frame Gravity vector
    g_body = R*[0;0;9.81;];
    
    % Body Frame Magnetic vector
    mag_body = R*decl;
    
    y(1:3) = g_body + a_body(1:3);
    y(4:6) = v_body(4:6);
    y(7:9) = mag_body;
end

