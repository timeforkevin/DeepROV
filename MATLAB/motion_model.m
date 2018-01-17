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
% m_fuselag = rho_acryl*sH*sW*sL;
% m_frame = 3*m_side;
% M = 36.14359; % [kg] from solidworks
M = m_body+m_frame+m_elec; % total mass of sub

%assuming thin pipe and solid sides
Jx = 35.44794;  % (from solidworks) [kg m^2]
Jy = 38.01554;  % (from solidworks) [kg m^2]
Jz = 55.75488;  % (from solidworks) [kg m^2]
Inert = diag([Jx, Jy, Jz]);


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
T = 0.1;
t = 0:T:1000; % simulation time/period in seconds
u = zeros(5, length(t));
u(1, :) = 1;
u(2, :) = 1;
u(3, :) = 10;
u(5, :) = -10;
u(4, :) = 0;
x_init = zeros(12,1);

x = [x_init zeros(length(x_init),length(t)-1)];

mot_off = [0;w1;0;
           0;-w1;0;
           -l1;w2;0;
           l2;0;0;
           -l1;-w2;0;];

x_mot = [mot_off zeros(3*5, length(t)-1)];

b_x = 74/(4/3.6)^2;
b_y = b_x;
b_z = 57.4/(1/3.6)^2;
b_rol = b_x;
b_pit = b_x;
b_yaw = b_x;
% b_x = 0;
% b_y = 0;
% b_z = 0;
% b_rol = 0;
% b_pit = 0;
% b_yaw = 0;



for k = 1:length(t)-1
    rol = x(4,k);
    pit = x(5,k);
    yaw = x(6,k);
%     d_rol = x(10,k);
%     d_pit = x(11,k);
%     d_yaw = x(12,k);
%     
%     dd_rol = (w2*(u(5,k)-u(3,k)) - abs(x(10,k))*x(10,k)*b_rol)/Jx;
%     dd_pit = (l2*u(4,k)-l1*(u(3,k)+u(5,k)) - abs(x(11,k))*x(11,k)*b_pit)/Jy; %neglect off axis effect on bottom
%     dd_yaw = (w1*(u(2,k)-u(1,k)) - abs(x(12,k))*x(12,k)*b_yaw)/Jz; %neglect off axis effects for sides
    
    % Inertial Frame to Body Frame
    R = eul2rotm([yaw, pit, rol]);
    x(1:3,k+1) = x(1:3,k) + x(7:9,k)*T;
    x(4:6,k+1) = x(4:6,k) + x(10:12,k)*T;
    
    x(7:9,k+1) = R'*[(u(1,k) + u(2,k) - abs(x(7,k))*x(7,k)*b_x)/M;
                     0
                     (u(3,k) + u(4,k) +u(5,k) - abs(x(9,k))*x(9,k)*b_z)/M;]*T;
    x(10:12,k+1) = R'*[(w2*(u(5,k)-u(3,k)) - abs(x(10,k))*x(10,k)*b_rol)/Jx;
                       (l2*u(4,k)-l1*(u(3,k)+u(5,k)) - abs(x(11,k))*x(11,k)*b_pit)/Jy; %neglect off axis effect on bottom
                       (w1*(u(2,k)-u(1,k)) - abs(x(12,k))*x(12,k)*b_yaw)/Jz;]*T; %neglect off axis effects for sides

    x_mot(1:3,k+1) = x(1:3,k) + R'*mot_off(1:3);
    x_mot(4:6,k+1) = x(1:3,k) + R'*mot_off(4:6);
    x_mot(7:9,k+1) = x(1:3,k) + R'*mot_off(7:9);
    x_mot(10:12,k+1) = x(1:3,k) + R'*mot_off(10:12);
    x_mot(13:15,k+1) = x(1:3,k) + R'*mot_off(13:15);

    if (mod(k, 100) == 0)
        figure(1)
        clf
        hold on
        plot3(x(1,1:k+1),x(2,1:k+1),x(3,1:k+1), '-r',...
              x_mot(1,1:k+1),x_mot(2,1:k+1),x_mot(3,1:k+1), '-g',...
              x_mot(4,1:k+1),x_mot(5,1:k+1),x_mot(6,1:k+1), '-b',...
              x_mot(7,1:k+1),x_mot(8,1:k+1),x_mot(9,1:k+1), '-c',...
              x_mot(10,1:k+1),x_mot(11,1:k+1),x_mot(12,1:k+1), '-m',...
              x_mot(13,1:k+1),x_mot(14,1:k+1),x_mot(15,1:k+1), '-k');
        plot3(x(1,k+1),x(2,k+1),x(3,k+1),'xr', ...
              x_mot(1,k+1),x_mot(2,k+1),x_mot(3,k+1),'og', ...
              x_mot(4,k+1),x_mot(5,k+1),x_mot(6,k+1),'ob', ...
              x_mot(7,k+1),x_mot(8,k+1),x_mot(9,k+1),'oc', ...
              x_mot(10,k+1),x_mot(11,k+1),x_mot(12,k+1),'om', ...
              x_mot(13,k+1),x_mot(14,k+1),x_mot(15,k+1),'ok');
        view(3)
        axis equal
        xlabel('x')
        ylabel('y')
        zlabel('z')
        drawnow;                        
    end
end

% figure (1)
% clf
% hold on
% plot3(x(1,:),x(2,:),x(3,:))
% plot3(x(1,1),x(2,1),x(3,1),'x')

% plot(x(1,:))
% figure (2)
% plot(x(2,:))
% figure (3)
% plot(x(3,:))
% figure (4)
% plot(x(4,:))
% figure (5)
% plot(x(5,:))
% figure (6)
% plot(x(6,:))
% figure (7)
% plot(x(7,:))
% figure (8)
% plot(x(8,:))
% figure (9)
% plot(x(9,:))
% figure (10)
% plot(x(10,:))
% figure (11)
% plot(x(11,:))
% figure (12)
% plot(x(12,:))

%% Controllers
