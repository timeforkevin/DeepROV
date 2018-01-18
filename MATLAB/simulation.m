clc;
close all;

%% Vehicle Parameters
m_body = 21.10095;  % mass of pipe body with domes (from solidworks) [kg]
m_frame = 15.04264; % mass of fram (from solidworks) [kg]
m_elec = 5;         % mass of electrical components [kg]

D = 0.18542;   % pipe outer diameter [m]
L = 1.16;      % sub body length [m]
W = 0.52635;% frame width [m] 

sH = 0.6;   % height of side plates [m]
sL = 1.27;  % length of side plates [m]
sT = 0.01;  % thickness/width of side plates [m]

bL = 0.96689;   % length of bottom plate [m]
bH = 0.5;       % height of bottom plate [m]
bT = 0.00635;   % thickness of bottom plate [m]

A_bp = 0.002029;    % area of bottom plate [m^2]
A_sp = 0.00456;     % area of side plates [m^2]

rho_acryl = 1190;   % density of acrylic [kg/m^3]

w1 = W/2;
w2 = w1-sT;
l1 = L/2;
l2 = L/2;

%% Performance Requirements
roll_time = 1; % settling time for rolling
roll_OS = 0.015; % max overshoot for rolling

%% Enviornmental Parameters
mu_H2O = 1.8*10^-3;     % kinematic viscotisy of water [Ns^2]
rho_H2O = 1000;         % denisty of water [kg/m^3]

C_D_cyl_x = 0.82;  % drag coefficient for long cylinder's face(wikipedia)
C_D_cyl_z = 1.17;  % drag coefficient for long cylinder's side(wikipedia)
C_D_cube = 1.05;    % drag coefficient for a cube (wikipedia)
C_D_plate = 1.28;   % drag coefficient for a plate (nasa) 

Keff = 0.5;     %eff. of elec to mech power in motor+prop []

%% Paramter Calculations
M = m_body+m_frame+m_elec; % total mass of sub

%assuming thin pipe and solid sides
Jx = 35.44794;  % (from solidworks) [kg m^2]
Jy = 38.01554;  % (from solidworks) [kg m^2]
Jz = 55.75488;  % (from solidworks) [kg m^2]


%Dampers
bx = 74;   %based off 74N of resistance at 1 m/s (from CFD)
by = 0;
bz = 0;
bphi = 0;  %assume water friction is negligable (~5mN)
btheta = 0;
bpsi = 0;


%% State Space Model

lin_drag = @(u,CD,A) 0.5*rho_H2O*CD*A*u^2; % [N]
rot_drag = @(u,CD,h,L) (rho_H2O*h*CD*(L^4)*(u^2))/64; %[Nm]

f = cell(12,1); %creates 12 cells for x_dot = f(...) functions

%% Inputs and Testing    
dt = 0.1;
t = 0:dt:1000; % simulation time/period in seconds
u = zeros(5, length(t));
u(1, 1:10) = 1;
u(2, 1:10) = 3;
u(3, :) = 0;
u(5, :) = 0;
u(4, :) = 0;
x_init = zeros(12,1);

x = [x_init zeros(length(x_init),length(t)-1)];

b_x = 74/(4/3.6)^2;
b_y = b_x;
b_z = 57.4/(1/3.6)^2;
b_rol = b_x;
b_pit = b_x;
b_yaw = b_x;

%% Set-up

% Positions of any features relative to centre mass
feat_off = [ 0,  0, l1,-l2, l1;
           w1,-w1, w2,  0,-w2;
            0,  0,  0,  0,  0;];
% Order of which the features are connected by line
draw_order = [1,4,2,5,3,1];

% Disturbance Covariance5
Q = diag([0.001, 0.001, 0.001,     ... Position Disturbance
          0.002, 0.002, 0.002,     ... Orientation Disturbance
          0.001, 0.001, 0.001,     ... Velocity Disturbance
          0.002, 0.002, 0.002].^2);... Ang. Velocity Disturbance
[REx, Rex] = eig(Q);
rov = struct('J',[Jx,Jy,Jz],...
             'B',[b_x,b_y,b_z,b_rol,b_pit,b_yaw],...
             'M',M,...
             'W',[w1,w2],...
             'L',[l1,l2],...
             'REx',REx,...
             'Rex',Rex);
         
%% Simulate
for k = 1:length(t)-1
    % Inertial Frame to Body Frame
    rol = x(4,k);
    pit = x(5,k);
    yaw = x(6,k);
    R = eul2rotm([yaw, pit, rol]);
    
    % Inertial frame Feature positions
    x_feat = R'*feat_off;
    x_feat(1,:) = x_feat(1,:) + x(1,k);
    x_feat(2,:) = x_feat(2,:) + x(2,k);
    x_feat(3,:) = x_feat(3,:) + x(3,k);
    
    % Simulated Motion Model Update
    x(:,k+1) = motion_model(x(:,k),u(:,k),dt,rov);
    
    % Plot
    if (mod(k, 100) == 0)
        figure(1)
        clf
        hold on
        plot3(x(1,1:k+1),x(2,1:k+1),x(3,1:k+1), '--r',...
              x_feat(1,draw_order),x_feat(2,draw_order),x_feat(3,draw_order),'-k')
        plot3(x(1,k+1),x(2,k+1),x(3,k+1),'xr', ...
              x_feat(1,:),x_feat(2,:),x_feat(3,:),'og');
        view(3)
        axis equal
        xlabel('x')
        ylabel('y')
        zlabel('z')
        drawnow;
    end
end

%% Controllers
