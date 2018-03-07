function [ mu_next, cov_next ] = kalman_filter( mu, cov, y, C, Q_est, R_est, dt )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    
    
    M = 25.6;       % [lb]
    Jxx = 746.77;   % [lb*in^2]
    Jyy = 1401.13;  % [lb*in^2]
    Jzz = 1320.75;  % [lb*in^2]
    cx = 100;
    cxx = 100;
    cyy = 100;
    czz = 100;

    A = diag([1 1 1 1 1 1 1 1]);
    A(1:4,5:8) = diag([dt dt dt dt]);
    A(5,5) = 1 - cx/M*dt;
    A(6,6) = 1 - cxx/Jxx*dt;
    A(7,7) = 1 - cyy/Jyy*dt;
    A(8,8) = 1 - czz/Jzz*dt;

    mu_p = A*mu;
    cov_p = A*cov*A' + Q_est;
    Kt = cov_p*C'/(C*cov_p*C'+R_est);
    I = (y - C*mu_p);
    mu_next = mu_p + Kt*I;
    cov_next = (eye(8)-Kt*C)*cov_p;

end

