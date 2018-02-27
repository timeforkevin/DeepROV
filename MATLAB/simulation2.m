%% Vehicle Parameters
m_body = 21.10095;  % mass of pipe body with domes (from solidworks) [kg]
m_frame = 15.04264; % mass of fram (from solidworks) [kg]
m_elec = 5;         % mass of electrical components [kg]

L = 1.16;      % sub body length [m]
W = 0.52635;% frame width [m] 

w1 = W/2;
w2 = w1-sT;
l1 = L/2;
l2 = L/2;

%% Paramter Calculations
% m_fuselag = rho_acryl*sH*sW*sL;
% m_frame = 3*m_side;
% M = 36.14359; % [kg] from solidworks
M = m_body+m_frame+m_elec; % total mass of sub

%assuming thin pipe and solid sides
Jx = 35.44794;  % (from solidworks) [kg m^2]
Jy = 38.01554;  % (from solidworks) [kg m^2]
Jz = 55.75488;  % (from solidworks) [kg m^2]

%% Inputs and Testing    
dt = 0.1;
t = 0:dt:1000; % simulation time/period in seconds
u = zeros(5, length(t));
u(1, :) = 1;
u(2, :) = 1;
u(3, :) = 10;
u(5, :) = -10;
u(4, :) = 0;
x_init = zeros(14,1);

x = [x_init zeros(length(x_init),length(t)-1)];

% Damping effects
% TODO: Verify
b_x = 74/(4/3.6)^2;
b_y = b_x;
b_z = 57.4/(1/3.6)^2;
b_rol = b_x;
b_pit = b_x;
b_yaw = b_x;

% Disturbance Covariance
Q = diag([0.001, 0.001, 0.001,        ... Position Disturbance
          0.002, 0.002, 0.002, 0.002, ... Orientation Disturbance
          0.000, 0.000, 0.000,        ... Velocity Disturbance
          0.000, 0.000, 0.000, 0.000].^2);... Ang. Velocity Disturbance
[REx, Rex] = eig(Q);

% IMU Position Offset from CG
imu_off = [0.25; 0; 0.05];

rov = struct('J',[Jx,Jy,Jz],...
             'B',[b_x,b_y,b_z,b_rol,b_pit,b_yaw],...
             'M',M,...
             'W',[w1,w2],...
             'L',[l1,l2],...
             'REx',REx,...
             'Rex',Rex,...
             'IMU',imu_off);

% Positions of any features relative to centre mass
feat_off = [ 0,  0, l1,-l2, l1;
            w1,-w1, w2,  0,-w2;
             0,  0,  0,  0,  0;];
% Order of which the features are connected by line
draw_order = [1,4,2,5,3,1];

for k = 1:length(t)-1
    %% Inertial frame Feature positions
    x_feat = quatrotate(x(4:7,k)',feat_off')';
    x_feat(1,:) = x_feat(1,:) + x(1,k);
    x_feat(2,:) = x_feat(2,:) + x(2,k);
    x_feat(3,:) = x_feat(3,:) + x(3,k);
    
    %% Simulated Motion Model Update
    x(:,k+1) = motion_model(x(:,k),u,dt,rov);
    
    %% Plot
    if (mod(k, 100) == 0)
        figure(1)
        clf
        hold on
        plot3(x(1,1:k+1),x(2,1:k+1),x(3,1:k+1), '-r',...
              x_feat(1,draw_order),x_feat(2,draw_order),x_feat(3,draw_order),'-r')
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
