function [ y ] = IMU_model( x, u, dt, rov )
%Measurement Model for the IMU
%   Detailed explanation goes here
    y = zeros(9, 1);
    
    % Inertial Frame to Body Frame
    rol = x(4);
    pit = x(5);
    yaw = x(6);
    R = eul2rotm([yaw, pit, rol]);
    
    % Body Frame Rates
    v_body = [R*x(7:9); R*x(10:12)];
    
    % IMU offset error vector
    p = v_body(4); % Roll Rate
    q = v_body(5); % Pitch Rate
    r = v_body(6); % Yaw Rate
    dp = (rov.W(2)*(u(5)-u(3)) - abs(v_body(4))*v_body(4)*rov.B(4))/rov.J(1);
    dq = (rov.L(2)*u(4)-rov.L(1)*(u(3)+u(5)) - abs(v_body(5))*v_body(5)*rov.B(5))/rov.J(2); %neglect off axis effect on bottom
    dr = (rov.W(1)*(u(2)-u(1)) - abs(v_body(6))*v_body(6)*rov.B(6))/rov.J(3);

    OE = [(q^2+r^2), -(p*q-dr), -(p*r+dq);
         -(p*q+dr), (p^2+r^2), -(q*r-dp);
         -(p*r-dq), -(p*r+dp), (p^2+q^2);];
    
    % Body Frame Gravity vector
    g_body = R*[0;0;9.81;];
    y(1:3) = g_body + OE*rov.IMU;
    y(4:6) = v_body(4:6);
    

end

