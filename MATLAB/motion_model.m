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
    Ex = rov.REx*sqrt(rov.Rex)*randn(12,1);
    % Position
    x_next(1:3) = x(1:3) + x(7:9)*dt;
    % Orientation
    x_next(4:6) = x(4:6) + x(10:12)*dt;

    % Velocity
    x_next(7:9) = R'*[(u(1)+u(2) - abs(x(7))*x(7)*rov.B(1))/rov.M;
                     0
                     (u(3)+u(4)+u(5) - abs(x(9))*x(9)*rov.B(3))/rov.M;]*dt;

    % Angular Velocity
    x_next(10:12) = R'*[(rov.W(2)*(u(5)-u(3)) - abs(x(10))*x(10)*rov.B(4))/rov.J(1);
                       (rov.L(2)*u(4)-rov.L(1)*(u(3)+u(5)) - abs(x(11))*x(11)*rov.B(5))/rov.J(2); %neglect off axis effect on bottom
                       (rov.W(1)*(u(2)-u(1)) - abs(x(12))*x(12)*rov.B(6))/rov.J(3);]*dt; %neglect off axis effects for sides
    x_next = x_next + Ex;
end

