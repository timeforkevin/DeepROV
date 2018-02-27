function [ x_next ] = motion_model2( x, u, dt, rov, noise)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    
    %% Constant Velocity Pose updates
    x_next = x;
    Ex = rov.REx*sqrt(rov.Rex)*randn(12,1); %noise
    % Position
    x_next(1:3) = x(1:3) + x(7:9)*dt;
    % Orientation
    x_next(4:6) = x(4:6) + x(10:12)*dt;

    % Inertial Frame to Body Frame
    r = x_next(4);
    p = x_next(5);
    y = x_next(6);
    R = R3D(y, p, r);
    Rz = [cos(y), -sin(y), 0;
          sin(y),  cos(y), 0;
               0,       0, 1;];

    % Body Frame Velocities
    v_body = [Rz*x(7:9); Rz*x(10:12)];
    % Angular Velocity
    x_next(10:12) = x(10:12)+ Rz'*[(rov.W(2)*(u(5)-u(3)) - abs(v_body(4))*v_body(4)*rov.B(4))/rov.J(1);
                                   (rov.L(2)*u(4)-rov.L(1)*(u(3)+u(5)) - abs(v_body(5))*v_body(5)*rov.B(5))/rov.J(2); %neglect off axis effect on bottom
                                   (rov.W(1)*(u(2)-u(1)) - abs(v_body(6))*v_body(6)*rov.B(6))/rov.J(3)]*dt; %neglect off axis effects for sides

    % Body Frame Acceleration due to rotation rate
    % Velocity
    x_next(7:9) = x(7:9)+ R'*[(u(1)+u(2) - abs(v_body(1))*v_body(1)*rov.B(1));
                              -abs(v_body(2))*v_body(2)*rov.B(2);
                              (u(3)+u(4)+u(5) - abs(v_body(3))*v_body(3)*rov.B(3));]/rov.M*dt;
    x_next(4:6) = wrapToPi(x_next(4:6));
    x_next(10:12) = wrapToPi(x_next(10:12));
    if (noise)
        x_next = x_next + Ex;
    end
end