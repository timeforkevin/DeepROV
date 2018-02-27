function [ A, B ] = motion_model_simple( dt, rov )
%Simplified Model of Attitude + Height
%   Detailed explanation goes here
    % States: Z Height, Roll, Pitch, Yaw
    b = rov.B;
    j = rov.J;
    m = rov.M;
    w = rov.W;
    L = rov.L;
    
    A = diag([1 1 1 1 1 1 1 1]);
%     A = diag([1 1 1 1 1 1 1 1]);
    A(1:4,5:8) = diag([dt dt dt dt]);
    
    B = zeros(8,5);
    B(5:8,:) = [0, 0, 1/m, 1/m, 1/m;
                0, 0, -w(2)/j(1), 0, w(2)/j(1);
                0, 0, -L(1)/j(2), L(2)/j(2), -L(1)/j(2);
                -w(1)/j(3), w(1)/j(3), 0, 0, 0;]/dt;
end