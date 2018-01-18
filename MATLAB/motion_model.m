function [ x_next ] = motion_model( x, u, dt, rov )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    % Inertial Frame to Body Frame
    rol = x(4);
    pit = x(5);
    yaw = x(6);
    R = eul2rotm([yaw, pit, rol]);
    
    %% Constant Velocity Pose updates
    x_next = zeros(12,1);
    Ex = 0;%rov.REx*sqrt(rov.Rex)*randn(12,1); %noise
    % Position
    x_next(1:3) = x(1:3) + x(7:9)*dt;
    % Orientation
    x_next(4:6) = x(4:6) + x(10:12)*dt;

    
    v_body = [R*x(7:9); R*x(10:12)];
    % Velocity
    x_next(7:9) = x(7:9)+ R'*[(u(1)+u(2) - abs(v_body(1))*v_body(1)*rov.B(1));
                              -abs(v_body(2))*v_body(2)*rov.B(2);
                              (u(3)+u(4)+u(5) - abs(v_body(3))*v_body(3)*rov.B(3));]/rov.M*dt;

    % Angular Velocity
    x_next(10:12) = x(10:12)+ R'*[(rov.W(2)*(u(5)-u(3)) - abs(v_body(4))*v_body(4)*rov.B(4))/rov.J(1);
                                  (rov.L(2)*u(4)-rov.L(1)*(u(3)+u(5)) - abs(v_body(5))*v_body(5)*rov.B(5))/rov.J(2); %neglect off axis effect on bottom
                                  (rov.W(1)*(u(2)-u(1)) - abs(v_body(6))*v_body(6)*rov.B(6))/rov.J(3)]*dt; %neglect off axis effects for sides
    
    x_next(4:6) = wrapToPi(x_next(4:6));
    x_next(10:12) = wrapToPi(x_next(10:12));
    x_next = x_next + Ex;
end