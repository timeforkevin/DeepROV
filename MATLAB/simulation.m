clc;
clear all;

%% Vehicle Parameters

w1 = 0.19; % distance of 'handling' motors from center
w2 = w1; % distance of 'drive' motors from center
l1 = 0.29; % distance of front handling motors from center
l2 = 0.29; % distance of rear hanling motor from center

m_body = 5.289;  % mass of pipe body with domes (from solidworks) [kg]
m_frame = 3; % mass of fram (from solidworks) [kg]
m_elec = 5;         % mass of electrical components [kg]

M = m_body+m_frame+m_elec; % total mass of sub

%assuming thin pipe and solid sides
Jx = 35.44794;  % (from solidworks) [kg m^2]
Jy = 38.01554;  % (from solidworks) [kg m^2]
Jz = 55.75488;  % (from solidworks) [kg m^2]

b_x = 74/(4/3.6)^2;
b_y = 10*b_x;
b_z = 57.4/(1/3.6)^2;
b_rol = b_x;
b_pit = b_x;
b_yaw = b_x;

offset_IMU = [0.01; 0; -0.015];
R_IMU = diag([0.001, 0.001, 0.001,          ... Noise on Accelerometer
              0.001, 0.001, 0.001,          ... Noise on Gyroscope
              0.001, 0.001, 0.001].^2);     ... Noise on Magnetometer
declination = R3D( -10*pi/180, 0, 0 )*[1;0;0];
[REy_IMU, Rey_IMU] = eig(R_IMU);
IMU = struct('off',offset_IMU,...
             'decl',declination,...
             'REy',REy_IMU,...
             'Rey',Rey_IMU);

%% Video Writing
vidwriter = VideoWriter('video.avi');
open(vidwriter);

%% Inputs, Estimation and Testing    
dt = 0.1;
t = 0:dt:1000; % simulation time/period in seconds
u = zeros(5, length(t));
x = zeros(12, length(t));
x(2:3,1) = [-2;1];
y_IMU = zeros(9, length(t));
mu = zeros(12, length(t));
cov = zeros(12, 12, length(t));

e = zeros(3, length(t));


%% Set-up

% Positions of any features (motors) relative to centre mass
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
Q = 4*Q;
[REx, Rex] = eig(Q);
rov_sim = struct('J',[Jx,Jy,Jz],...
             'B',[b_x,b_y,b_z,b_rol,b_pit,b_yaw],...
             'M',M,...
             'W',[w1,w2],...
             'L',[l1,l2],...
             'REx',REx,...
             'Rex',Rex,...
             'IMU',IMU);
[As, Bs] = motion_model_simple(dt, rov_sim);
rov_est = rov_sim;
rov_est.W = 2*rov_sim.W;
rov_est.L = 3*rov_sim.L;
[Ae, Be] = motion_model_simple(dt, rov_est);
waypoints = [  0,  1,   2,   4,   6,   8,  10,  12,  14;
              -2, -2,  -1,  -3,  -1,  -3,  -1,  -3,  -1;
               1,  1, 1.5, 0.5,   2, 2.5,   2,   2,   2;];
% waypoints = [  0, 14;
%               -2, -1;
%                1,  2;];

way_idx = 1;
cont_way_idx = 1;
for k = 1:length(t)-1

%% Simulation
    
    % Simulated Motion Model Update
    x(:,k+1) = motion_model2(x(:,k),u(:,k),dt,rov_sim,true);
    % Simulated IMU Sensor
%     y_IMU(:,k+1) = IMU_model(x(:,k+1), x(:,k), dt, rov_sim);
    
    
%% EKF Estimation
%     mu_p = motion_model(mu(:,k),u(:,k),dt,rov_sim,false);
%     
%     
%     % Linearize Sensor Model about mu
%     delta = 0.0001;
%     H_IMU = zeros(9, 12);
%     y_bar = IMU_model_simple(mu_p, mu(:,k), dt, declination);
%     measurements = 1:9;
%     observable_states = [4,5,6];
%     for i = 1:length(observable_states)
%         dmu = zeros(12, 1); dmu(observable_states(i)) = delta;
%         dy_dmu = -(y_bar-IMU_model_simple(mu_p+dmu, mu(:,k), dt, declination))/delta;
%         H_IMU(:, observable_states(i)) = dy_dmu(measurements);
%     end
%     
%     cov_p = Ai*cov(:,:,k)*Ai' + Q;
%     
%     Kt = cov_p*H_IMU'/(H_IMU*cov_p*H_IMU'+R_IMU);
%     y_IMU_p = IMU_model_simple(mu_p, mu(:,k), dt, declination);
%     I = (y_IMU(:,k+1) - y_IMU_p);
%     mu(:,k+1) = mu_p + Kt*I;
%     cov(:,:,k+1) = (eye(12)-Kt*H_IMU)*cov_p;

%% Outer Loop Non Linear Steering Controller
    K_STEER = 10;
    K_SOFT = 10;
    K_VEL = 1;
    tar_vel = 0.1;

    prev_point = waypoints(1:2, way_idx)';
    next_point = waypoints(1:2, way_idx+1)';
    curr_point = x(1:2, k)';
    [e_ct, completion] = distanceToLineSegment(prev_point, next_point, curr_point);
    e(1, k) = e_ct; % Record Cross Track Error XY
    
    prev_point = waypoints(1:2, cont_way_idx)';
    next_point = waypoints(1:2, cont_way_idx+1)';
    curr_point = x(1:2, k)';
    [e_ct, cont_completion] = distanceToLineSegment(prev_point, next_point, curr_point);
    traj_angle = atan2(next_point(2)-prev_point(2), next_point(1)-prev_point(1));
    e_h = wrapToPi(x(6, k) - traj_angle);
    
    steer = wrapToPi(e_h/2 + atan2(K_STEER*e_ct, K_SOFT-tar_vel));
    
%% Inner Loop Controller
    % Inputs: Z height, Roll, Pitch
    % Controls: 3 Motors
    
    ui_bar = zeros(5, 1);
    x_bar = zeros(12, 1);
    x_bar(1:2) = waypoints(1:2, way_idx);
    x_bar(3) = waypoints(3, way_idx) +...
               (cont_completion+0.1)*(waypoints(3, way_idx+1)-waypoints(3, way_idx));
    x_bar(6) = steer;

    % Record Cross Track Error Z
    e(2, k) = x(3, k) - (waypoints(3, way_idx) +...
              cont_completion*(waypoints(3, way_idx+1)-waypoints(3, way_idx)));

    % Select rows and columns of G for inner LQR
    states_i = [3 4 5 6 9 10 11 12];
    % Weights
    Qi = diag([1 100 100 100 0.1 1 1 1]);
    Ri = diag([0.01 0.01 0.1 0.1 0.1]);
    Kinner = dlqr(Ae,Be,Qi,Ri);
    
    % Determine Inputs
    dmu = x_bar-x(:,k);
    dmu([4:6, 10:12]) = wrapToPi(dmu([4:6, 10:12]));
    dmu = dmu(states_i);
    cur_vel = norm(x(7:8,k));
    u(:, k+1) = Kinner*(dmu);
    u(1:2,k+1) = u(1:2,k+1) - mean(u(1:2,k+1)) + K_VEL*(tar_vel-cur_vel);

    %% Update Waypoint
    if (cont_completion > 1 && cont_way_idx ~= length(waypoints))
        cont_way_idx = cont_way_idx + 1;
    end
    if (completion > 1)
        way_idx = way_idx + 1;
        if (way_idx == length(waypoints))
            break;
        end
    end

%% Plot
    if (mod(k, 100) == 0)
        % Inertial Frame to Body Frame
        R = R3D(x(6,k), x(5,k), x(4,k));

        % Inertial frame Feature positions
        x_feat = R'*feat_off;
        x_feat(1,:) = x_feat(1,:) + x(1,k);
        x_feat(2,:) = x_feat(2,:) + x(2,k);
        x_feat(3,:) = x_feat(3,:) + x(3,k);
        
        % Inertial Frame to Body Frame
        R = R3D(mu(6,k), mu(5,k), mu(4,k));

        % Inertial frame Feature positions
        mu_feat = R'*feat_off;
        mu_feat(1,:) = mu_feat(1,:) + mu(1,k);
        mu_feat(2,:) = mu_feat(2,:) + mu(2,k);
        mu_feat(3,:) = mu_feat(3,:) + mu(3,k);
        
        figure(1)
        clf
        subplot(2, 2, [1 3])
        hold on
        plot3(x(1,1:k+1),x(2,1:k+1),x(3,1:k+1), '--r',...
              x_feat(1,draw_order),x_feat(2,draw_order),x_feat(3,draw_order),'-k')
        plot3(x(1,k+1),x(2,k+1),x(3,k+1),'xr', ...
              x_feat(1,:),x_feat(2,:),x_feat(3,:),'og');
        plot3(mu(1,1:k+1),mu(2,1:k+1),mu(3,1:k+1), '--g',...
              mu_feat(1,draw_order),mu_feat(2,draw_order),mu_feat(3,draw_order),'-g')
        plot3(mu(1,k+1),mu(2,k+1),mu(3,k+1),'xg', ...
              mu_feat(1,:),mu_feat(2,:),mu_feat(3,:),'og');
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
        
        subplot(2, 2, 2)
        if (k > 500)
            plot(e(1,k-500:k))
        else
            plot(e(1,1:k))
        end
            
        grid on;
        title('Cross Track Error XY Plane')
        subplot(2, 2, 4)
        if (k > 500)
            plot(e(2,k-500:k))
        else
            plot(e(2,1:k))
        end
        grid on;
        title('Cross Track Error XZ Height')
        drawnow;
        f = getframe(1);
        writeVideo(vidwriter, f);
    end
end

%% Video Writing
close(vidwriter);

%% Controllers
