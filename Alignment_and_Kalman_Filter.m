
clc;
close all;

load('D:\M.M.L.E\Autonomous Vehicle Engineering-Federico Uni\2nd Semester\Sensor Data Fusion\Project\Matlab\SDF_Normalized_Data.mat');

dt = Dynamic_IMU(2, 1) - Dynamic_IMU(1, 1);

total_steps = length(Dynamic_IMU);
total_gnss_readings = size(Dynamic_GNSS, 1);
gnss_idx = 1;

x = zeros(4,1);
P = eye(4)*(0.1);

Q = [0.001 0 0 0;
     0 0.001 0 0;
     0 0 acc_var(1) 0;
     0 0 0 gyro_var(3)];

EKF_trajectory = zeros(total_steps, 2);


for i = 1:total_steps

    accel_forward = Dynamic_IMU(i,2);
    yaw_rate = Dynamic_IMU(i,7);

    x(4) = x(4) + (yaw_rate * dt);
    x(1) = x(1) + (x(3) * cos(x(4)) * dt);
    x(2) = x(2) + (x(3) * sin(x(4)) * dt);
    x(3) = x(3) + (accel_forward * dt);

    F = [1, 0,  cos(x(4))*dt, -x(3)*sin(x(4))*dt;
         0, 1,  sin(x(4))*dt,  x(3)*cos(x(4))*dt;
         0, 0,  1,             0;
         0, 0,  0,             1];

    P = F * P * F' + Q;

    if gnss_idx <= total_gnss_readings && Dynamic_IMU(i,1) >= Dynamic_GNSS(gnss_idx, 1)

        Z = [Dynamic_GNSS(gnss_idx, 2); 
             Dynamic_GNSS(gnss_idx, 3)];

        R = diag([Dynamic_GNSS(gnss_idx, 4), Dynamic_GNSS(gnss_idx, 5)]);

        H = [1, 0, 0, 0; 
             0, 1, 0, 0];

        S = H * P * H' + R;

        K = P * H' / S;
        y = Z - (H * x);
        x = x + (K * y);
        P = (eye(4) - (K * H)) * P;
        gnss_idx = gnss_idx + 1;
    end

    EKF_trajectory(i, 1) = x(1);
    EKF_trajectory(i, 2) = x(2);
end



% Plot the estimated trajectory
figure('Name', 'EKF Sensor Fusion Trajectory');
hold on;
grid on;

plot(Dynamic_GNSS(:,2), Dynamic_GNSS(:,3), 'r.', 'MarkerSize', 15, 'DisplayName', 'Raw GNSS (1 Hz)');


plot(EKF_trajectory(:,1), EKF_trajectory(:,2), 'b-', 'LineWidth', 2.5, 'DisplayName', 'EKF Trajectory (200 Hz)');

title('Autonomous Vehicle 2D Trajectory: IMU + GNSS Fusion');
xlabel('East (meters)');
ylabel('North (meters)');
legend('Location', 'best');

   


% --- PHASE 4: RMSE ERROR CALCULATION ---
% We must find the EKF positions that perfectly match the GNSS timestamps
error_array = zeros(size(Dynamic_GNSS, 1), 1);
t_imu = Dynamic_IMU(:,1);

for i = 1:size(Dynamic_GNSS, 1)
    [~, idx] = min(abs(t_imu - Dynamic_GNSS(i, 1)));
    
    diff_east = EKF_trajectory(idx, 1) - Dynamic_GNSS(i, 2);
    diff_north = EKF_trajectory(idx, 2) - Dynamic_GNSS(i, 3);
    
    error_array(i) = sqrt(diff_east^2 + diff_north^2);
end

RMSE = sqrt(mean(error_array.^2));
fprintf('The Root Mean Square Error (RMSE) of the fusion is: %.3f meters\n', RMSE);
% --- PHASE 4: COORDINATE TRANSFORMATION (NED TO LLA) ---
% Convert the entire EKF trajectory back to degrees for satellite mapping
lat0 = GNSS_numeric(1, 6); % Pulls original Latitude (Raw DD)
lon0 = GNSS_numeric(1, 7); % Pulls original Longitude (Raw DD)

EKF_LLA = ned2lla([EKF_trajectory(:,2), EKF_trajectory(:,1), zeros(length(EKF_trajectory),1)], [lat0, lon0, 0], 'flat');

figure('Name', 'Real-World Satellite Map');
geoplot(EKF_LLA(:,1), EKF_LLA(:,2), 'b', 'LineWidth', 2); 
hold on;
geoplot(GNSS_numeric(:,6), GNSS_numeric(:,7), 'r.', 'MarkerSize', 15); % Plots original coordinates
title('Sensor Fusion Trajectory on Map');
legend('EKF Path', 'Raw GNSS');