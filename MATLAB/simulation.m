clc;
clear all;

%% Vehicle Parameters
m_body = 21.10095;  % mass of pipe body with domes (from solidworks) [kg]
m_frame = 15.04264; % mass of fram (from solidworks) [kg]
m_elec = 5;         % mass of electrical components [kg]

D = 0.18542;   % pipe outer diameter [m]
L = 0.75;      % sub body length [m]
W = 0.35;% frame width [m] 

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
by = 74;
bz = 74;
bphi = 74;  %assume water friction is negligable (~5mN)
btheta = 74;
bpsi = 74;


%% State Space Model

lin_drag = @(u,CD,A) 0.5*rho_H2O*CD*A*u^2; % [N]
rot_drag = @(u,CD,h,L) (rho_H2O*h*CD*(L^4)*(u^2))/64; %[Nm]

f = cell(12,1); %creates 12 cells for x_dot = f(...) functions

%% Inputs and Testing    
dt = 0.1;
t = 0:dt:1000; % simulation time/period in seconds
u = zeros(5, length(t));
x = zeros(12, length(t));
e = zeros(3, length(t));

b_x = 74/(4/3.6)^2;
b_y = 10*b_x;
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

% Disturbance Covariance
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
waypoints = [  0,  1,   2,   3,   4,   5,   6,   7,   8;
              -1, -1,  -1,-1.5,-2.5,-3.5,-4.0,-4.0,-4.0;
               1,  1, 1.5, 1.5,   2,   2,   2,   2,   2;];
way_idx = 1;
%% Simulate
for k = 1:length(t)-1
    % Inertial Frame to Body Frame
    rol = x(4,k);
    pit = x(5,k);
    yaw = x(6,k);
    R = R3D(yaw, pit, rol);
    
    % Inertial frame Feature positions
    x_feat = R'*feat_off;
    x_feat(1,:) = x_feat(1,:) + x(1,k);
    x_feat(2,:) = x_feat(2,:) + x(2,k);
    x_feat(3,:) = x_feat(3,:) + x(3,k);
    
    % Simulated Motion Model Update
    x(:,k+1) = motion_model(x(:,k),u(:,k),dt,rov,true);
    
    
%% Outer Loop Non Linear Steering Controller
    K_STEER = 10;
    K_SOFT = 10;
    K_VEL = 1;
    tar_vel = 0.1;

    prev_point = waypoints(1:2, way_idx)';
    next_point = waypoints(1:2, way_idx+1)';
    curr_point = x(1:2, k)';
    traj_angle = atan2(next_point(2)-prev_point(2), next_point(1)-prev_point(1));
    e_h = wrapToPi(x(6, k) - traj_angle);
    [e_ct, completion] = distanceToLineSegment(prev_point, next_point, curr_point);
    
    
    e(1, k) = e_ct; % Record Cross Track Error XY

    
    steer = wrapToPi(e_h/2 + atan2(K_STEER*e_ct, K_SOFT-tar_vel));
    
%% Inner Loop Controller
    % Inputs: Z height, Roll, Pitch
    % Controls: 3 Motors
    delta = 0.001;
    
    ui_bar = zeros(5, 1);
    x_bar = zeros(12, 1);
    x_bar(1:2) = waypoints(1:2, way_idx);
    x_bar(3) = waypoints(3, way_idx) +...
               (completion+0.2)*(waypoints(3, way_idx+1)-waypoints(3, way_idx));
    x_bar(6) = steer;

    % Record Cross Track Error Z
    e(2, k) = x(3, k) - (waypoints(3, way_idx) +...
              completion*(waypoints(3, way_idx+1)-waypoints(3, way_idx)));

    % Linearization
    Ai = zeros(8, 8);
    Bi = zeros(8, 3);
    f_bar = motion_model(x(:,k),ui_bar,dt,rov,false);
    states_i = [3 4 5 6 9 10 11 12];
    for i = 1:8
        dx = zeros(12, 1); dx(states_i(i)) = delta;
        df_dxi = (f_bar-motion_model(x(:,k)+dx,ui_bar,dt,rov,false))/delta;
        df_dxi([4:6, 10:12]) = wrapToPi(df_dxi([4:6, 10:12]));
        Ai(:, i) = df_dxi(states_i);
    end
    motors_i = [1 2 3 4 5];
    for i = 1:5
        du = zeros(5, 1); du(motors_i(i)) = delta;
        df_dui = (f_bar-motion_model(x(:,k),ui_bar+du,dt,rov,false))/delta;
        df_dui([4:6, 10:12]) = wrapToPi(df_dui([4:6, 10:12]));
        Bi(:, i) = df_dui(states_i);
    end
    % Weights
    Qi = diag([1 1 1 100 0.1 0.1 0.1 0.001]);
    Ri = diag([0.01 0.01 0.1 0.1 0.1]);
    Kinner = dlqr(Ai,Bi,Qi,Ri);
    
    % Determine Inputs
    dx = x_bar-x(:,k);
    dx([4:6, 10:12]) = wrapToPi(dx([4:6, 10:12]));
    dx = dx(states_i);
    cur_vel = norm(x(7:8,k));
    u(motors_i, k+1) = Kinner*(dx);
    u(1:2,k+1) = u(1:2,k+1) - mean(u(1:2,k+1)) + K_VEL*(tar_vel-cur_vel);

    
    %% Update Waypoint
    if (completion > 1)
        way_idx = way_idx + 1;
        if (way_idx == length(waypoints))
            break;
        end
    end

%% Plot
    if (mod(k, 10) == 0)
        figure(1)
        clf
        subplot(4, 1, 1:2)
        hold on
        plot3(x(1,1:k+1),x(2,1:k+1),x(3,1:k+1), '--r',...
              x_feat(1,draw_order),x_feat(2,draw_order),x_feat(3,draw_order),'-k')
        plot3(x(1,k+1),x(2,k+1),x(3,k+1),'xr', ...
              x_feat(1,:),x_feat(2,:),x_feat(3,:),'og');
        plot3(waypoints(1, :),...
              waypoints(2, :),...
              waypoints(3, :), '-b');
        
        
        projections = repmat(x(1:3,k+1), 1, 3);
        projections(2, 1) = 0;
        projections(3, 3) = 0;
        plot3(projections(1, :),projections(2, :),projections(3, :),'-r');
        
        plot3(projections(1, 1),projections(2, 1),projections(3, 1),'xr');
        plot3(waypoints(1, :),...
              waypoints(2, :),...
              0*waypoints(3, :), '-c');
        plot3(projections(1, 3),projections(2, 3),projections(3, 3),'xr');
        plot3(waypoints(1, :),...
              0*waypoints(2, :),...
              waypoints(3, :), '-c');
        grid on;
        title('Simulation')
        view(3)
        axis equal
        xlabel('x')
        ylabel('y')
        zlabel('z')
        
        subplot(4, 1, 3)
        plot(e(1,1:k))
%         plot(x(1,k+1),x(2,k+1),'xr');
%         plot(waypoints(1, :),...
%              waypoints(2, :),...
%              '-b');
        grid on;
        title('Cross Track Error XY Plane')
        subplot(4, 1, 4)
        plot(e(2,1:k))
%         plot(x(1,k+1),x(3,k+1),'xr');
%         plot(waypoints(1, :),...
%              waypoints(3, :), '-b');
        grid on;
        title('Cross Track Error XZ Height')
        drawnow;
    end
end

%% Controllers