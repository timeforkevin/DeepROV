function [ mu_next, cov_next ] = kalman_filter( mu, cov, y, C, Q_est, R_est )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    mu_p = mu;
    cov_p = cov + Q_est;
    Kt = cov_p*C'/(C*cov_p*C'+R_est);
    I = (y - C*mu_p);
    mu_next = mu_p + Kt*I;
    cov_next = (eye(4)-Kt*C)*cov_p;

end

