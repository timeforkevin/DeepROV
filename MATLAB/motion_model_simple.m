function [ x_next ] = motion_model_simple( x, u, dt, rov )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    
    J = rov.J;
    M = rov.M;
    W = rov.W;
    L = rov.L;
    
    %% Constant Velocity Pose updates
    x_next = x;
    % Position
    x_next(1:3) = x(1:3) + x(7:9)*dt;
    % Orientation
    x_next(4:6) = x(4:6) + x(10:12)*dt;

    % Inertial Frame to Body Frame
    r = x_next(4);
    p = x_next(5);
    y = x_next(6);
    R = R3D(y, p, r);
    
    % Angular Velocity
    x_next(10:12) = x(10:12)+ R'*[(W(2)*(u(5)-u(3)))/J(1);
                                  (L(2)*u(4)-L(1)*(u(3)+u(5)))/J(2); %neglect off axis effect on bottom
                                  (W(1)*(u(2)-u(1)))/J(3)]*dt; %neglect off axis effects for sides

    % Body Frame Acceleration due to rotation rate
    % Velocity
    x_next(7:9) = x(7:9)+ R'*[(u(1)+u(2));
                              0;
                              (u(3)+u(4)+u(5));]/M*dt;
end