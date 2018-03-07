clear all;

W1 = 6; % [in] distance of 'handling' motors from center
W2 = 6; % [in] distance of 'drive' motors from center
L1 = 8; % [in] distance of front handling motors from center
L2 = 8; % [in] distance of rear hanling motor from center

M = 25.6; % [lb]

J = [746.77,  -63.19,   -4.42;
     -63.19, 1401.13,    1.87;
      -4.42,    1.87, 1320.75]; % [lb*in^2]
Jxx = J(1,1);
Jyy = J(2,2);
Jzz = J(3,3);
  
  
%% Controller
dt = 0.05;                    % [s]
T100_THRUST = 5;              % [lbf]
g = 32.174;                   % [ft/s^2]
T100_THRUST = T100_THRUST*g;  % [lb*ft/s^2]
T100_THRUST = T100_THRUST*12; % [lb*in/s^2]
pf = 1.35;

cx = 100;
cxx = 100;
cyy = 100;
czz = 100;

% States: Z [in], Roll [rad], Pitch [rad], Yaw [rad]

Ad = diag([1 1 1 1]);
Bd = zeros(4, 5);

Bd(1,[3,4,5]) = T100_THRUST/M*dt*dt;
Bd(2,3) =      -T100_THRUST*W2/Jxx*dt*dt;
Bd(2,5) =       T100_THRUST*W2/Jxx*dt*dt;
Bd(3,[3,5]) =  -T100_THRUST*L1/Jyy*dt*dt;
Bd(3,4) =       T100_THRUST*L2/Jyy*dt*dt;
Bd(4,1) =      -T100_THRUST*W1/Jzz*dt*dt;
Bd(4,2) =       T100_THRUST*W1/Jzz*dt*dt;

Qd = diag([1 1 1 1]);
Rd = diag([10 10 10 10 10]);

Kd = dlqr(Ad,Bd,Qd,Rd);


A = diag([1 1 1 1 1 1 1 1]);
A(1:4,5:8) = diag([dt dt dt dt]);
A(5,5) = 1 - cx/M*dt;
A(6,6) = 1 - cxx/Jxx*dt;
A(7,7) = 1 - cyy/Jyy*dt;
A(8,8) = 1 - czz/Jzz*dt;

B = zeros(8, 5);
B(5,[3,4,5]) = T100_THRUST/M*dt*dt;
B(6,3) =      -T100_THRUST*W2/Jxx*dt*dt;
B(6,5) =       T100_THRUST*W2/Jxx*dt*dt;
B(7,[3,5]) =  -T100_THRUST*L1/Jyy*dt*dt;
B(7,4) =       T100_THRUST*L2/Jyy*dt*dt;
B(8,1) =      -T100_THRUST*W1/Jzz*dt*dt;
B(8,2) =       T100_THRUST*W1/Jzz*dt*dt;

C = [diag([1 1 1 1]), diag([0 0 0 0])];

Q = diag([1 1 1 1 10 10 10 10]);
R = diag([10 10 10 10 10]);

K = dlqr(A,B,Q,R);

t = 0:dt:100;
u = zeros(5, length(t));
x = zeros(8, length(t));
y = zeros(4, length(t));
mu = zeros(8, length(t));
cov = zeros(8, 8, length(t));

% Covariance
Q = diag([0.1, 0.008, 0.008, 0.008,...
          0.01, 0.001, 0.001, 0.001].^2);
[REx, Rex] = eig(Q);
R = diag([1, 0.008, 0.008, 0.008].^2);
[REy, Rey] = eig(R);
Q_est = diag([0.1, 0.08, 0.08, 0.08,...
              1, 0.05, 0.05, 0.05].^2);
% Q_est = 10*Q_est;
R_est = 2*R;


x(:,1) = [0; 0; 0; 0; 0; 0; 0; 0;];
mu_tar  = [36; 0; 0; 0; 0; 0; 0; 0;];
mu_thresh = [2; 5*pi/180; 5*pi/180; 5*pi/180;
             0.4; 1*pi/180; 1*pi/180; 1*pi/180;];

u_min = [-1;-1;-1;-1;-1;];
u_max = [ 1; 1; 1; 1; 1;];
u_delay = round(0.33/dt);

for k = 1:length(t)-u_delay
    Ex = REx*sqrt(Rex)*randn(8,1);
    x(:,k+1) = A*x(:,k) + B*u(:,k) + Ex;
    Ey = REy*sqrt(Rey)*randn(4,1);
    y(:,k) = C*x(:,k) + Ey;
    
    % Estimation
    
    [mu_next, cov_next] = kalman_filter(mu(:,k), cov(:,:,k), y(:,k), C, Q_est, R_est, dt);
    mu(:,k+1) = mu_next;
    cov(:,:,k+1) = cov_next;
    
%     mu_tar(1) = mu(1,k+1);
    dmu = mu_tar - mu(:,k+1);
    dmu(abs(dmu) < mu_thresh) = 0;
    u_next = K*dmu;
%     u_next = Kd*dmu(1:4);
    u_next = round(u_next,2);
    
    if (u_next(4) > u_max(4) || u_next(4) < u_min(4))
        u_next(3) = u_next(3)/abs(u_next(4));
        u_next(5) = u_next(5)/abs(u_next(4));
    end
    u_next([3,5]) = u_next([3,5])/pf;
    u_next = max(u_min,min(u_max,u_next));
    u(:,k+u_delay) = u_next;
end

figure(1)
subplot(2, 1, 1)
plot(t,x(1,:)/12,t,x(2:4,:),t,x(5,:)/12,t,x(6:8,:))
% plot(t,mu(1,:)/12,t,mu(2:4,:),t,mu(5,:)/12,t,mu(6:8,:))
% plot(t,mu(1,:),t,mu(5,:))
% ylim([-pi/2 pi/2]);
legend('z', 'r', 'p', 'y', 'zd', 'rd', 'pd', 'yd');
subplot(2, 1, 2)
plot(t,u(:,:))
ylim([-1 1]);
legend('1', '2', '3', '4', '5');

figure(2)
plot(t,x(5,:),t,mu(5,:))

