clc;
clear all;
close all;

port_name = "COM5";
baud_rate = 921600;
gath_sec = 120*5;

s = serialport(port_name, baud_rate);
flush(s);

IMU_data = zeros(100000, 7);
GNSS_data = strings(500, 1);

logging_started = false;
imu_row = 1;
gnss_row = 1;

disp('Waiting for GNSS lock to start recording...');

while 1 
     % Check if our 120 seconds are up
     if logging_started && toc >= gath_sec
         break;
     end
     
     % Read the entire incoming line of text at once
     line = strtrim(readline(s));
     
     % --- GNSS DATA PARSING ---
     if startsWith(line, "$PSTMPV")
         if ~logging_started
             logging_started = true; 
             disp('Recording started! Do not move the board for 60 seconds.');
             tic;
         end
         GNSS_data(gnss_row, 1) = line;
         gnss_row = gnss_row + 1;
     
     % --- IMU DATA PARSING ---
     elseif startsWith(line, "IMU,")
         if ~logging_started
             continue;
         end
         
         % Split the text line by commas
         parts = split(line, ",");
         
         % Verify we received all 8 pieces (IMU, time, ax, ay, az, gx, gy, gz)
         if length(parts) == 8
             t  = str2double(parts(2));
             ax = str2double(parts(3));
             ay = str2double(parts(4));
             az = str2double(parts(5));
             gx = str2double(parts(6));
             gy = str2double(parts(7));
             gz = str2double(parts(8));
             
             % Save to our array
             IMU_data(imu_row,:) = [t, ax, ay, az, gx, gy, gz];
             imu_row = imu_row + 1;
         end
     end
end

% Trim the empty zeros off the bottom of the arrays
IMU_data = IMU_data(1:imu_row-1, :);
GNSS_data = GNSS_data(1:gnss_row-1, :);

disp('Data Collection Complete!');
save('SDF_Raw_Data.mat', 'IMU_data', 'GNSS_data');