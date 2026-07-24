
clc;
close all;

load('SDF_Raw_Data.mat');

mg_to_mps2 = 0.00981;
dps_to_rad = pi / 180;

IMU_data(:, 2:4) = IMU_data(:, 2:4) * mg_to_mps2;
IMU_data(:, 5:7) = IMU_data(:, 5:7) * dps_to_rad;

start_time = IMU_data(1,1);
IMU_data(:,1) = IMU_data(:,1) - start_time;

static_IMU = IMU_data(IMU_data(:,1)<=60,:);
Dynamic_IMU = IMU_data(IMU_data(:,1)>60,:);

bias_gyro_x = mean(static_IMU(:,5));
bias_gyro_y = mean(static_IMU(:,6));
bias_gyro_z = mean(static_IMU(:,7));

gyro_bias = [bias_gyro_x,bias_gyro_y,bias_gyro_z];

Dynamic_IMU(:, 5:7) = Dynamic_IMU(:,5:7) - gyro_bias;

var_gyro_x = var(static_IMU(:,5));
var_gyro_y = var(static_IMU(:,6));
var_gyro_z = var(static_IMU(:,7));

gyro_var = [var_gyro_x,var_gyro_y,var_gyro_z];


mean_acc_x = mean(static_IMU(:,2));
mean_acc_y = mean(static_IMU(:,3));
mean_acc_z = mean(static_IMU(:,4));

acc_bias = [mean_acc_x,mean_acc_y,mean_acc_z];

Dynamic_IMU(:, 2:4) = Dynamic_IMU(:,2:4) - acc_bias;

var_acc_x = var(static_IMU(:,2));
var_acc_y = var(static_IMU(:,3));
var_acc_z = var(static_IMU(:,4));

acc_var = [var_acc_x,var_acc_y,var_acc_z];


%%%%%GNSS SECTION
num_gnss_rows = length(GNSS_data);
GNSS_numeric = zeros(num_gnss_rows, 7); % Expanded to 7 columns to save original Lat/Lon

for i = 1:num_gnss_rows
    % Fix the string array indexing
    line_str = char(GNSS_data(i)); 
    pieces = strsplit(line_str, ',');
    
    % 1. Time Synchronization
    raw_time = str2double(pieces{2});
    hh = floor(raw_time / 10000);
    rem = raw_time - (hh * 10000);
    mm = floor(rem / 100);
    ss = rem - (mm * 100);
    time_val = (hh * 3600) + (mm * 60) + ss; 
    
    % 2. Latitude
    raw_lat = str2double(pieces{3});
    ns_hemi = pieces{4};
    lat_deg = floor(raw_lat / 100);
    lat_min = raw_lat - (lat_deg * 100);
    pure_lat = lat_deg + (lat_min / 60);
    if strcmp(ns_hemi, 'S')
        pure_lat = -pure_lat;
    end
    
    % 3. Longitude
    raw_lon = str2double(pieces{5});
    ew_hemi = pieces{6};
    lon_deg = floor(raw_lon / 100);
    lon_min = raw_lon - (lon_deg * 100);
    pure_lon = lon_deg + (lon_min / 60);
    if strcmp(ew_hemi, 'W')
        pure_lon = -pure_lon;
    end
    
    % Extract Errors
    lat_err = str2double(pieces{12});
    long_err = str2double(pieces{13});
    
    % The Zero Check 
    if raw_lat == 0 && raw_lon == 0
        GNSS_numeric(i,:) = NaN; % Mark bad data so it gets removed
    else
        % Save: [Time, East(temp), North(temp), err_lat, err_lon, pure_lat, pure_lon]
        GNSS_numeric(i,:) = [time_val, pure_lat, pure_lon, lat_err, long_err, pure_lat, pure_lon];
    end
end

%% Removing Nan
GNSS_numeric(any(isnan(GNSS_numeric), 2), :) = [];

%%% Zeroing the base timeline
gnss_start_time = GNSS_numeric(1, 1);
GNSS_numeric(:, 1) = GNSS_numeric(:, 1) - gnss_start_time;

% Convert LLA to NED (Meters)
lat0 = GNSS_numeric(1, 2);
lon0 = GNSS_numeric(1, 3);
h0   = 0;
lat_path = GNSS_numeric(:, 2);
lon_path = GNSS_numeric(:, 3);
h_path   = zeros(length(lat_path), 1);
wgs84 = wgs84Ellipsoid('meter');
[xEast, yNorth, zUp] = geodetic2enu(lat_path, lon_path, h_path, lat0, lon0, h0, wgs84);

% Overwrite 2 and 3 with Meters, but 6 and 7 still hold your Decimal Degrees!
GNSS_numeric(:, 2) = xEast;
GNSS_numeric(:, 3) = yNorth;

% Separate the Dynamic GNSS data
Dynamic_GNSS = GNSS_numeric(GNSS_numeric(:, 1) > 60, :);

% Save all variables
save('SDF_Normalized_Data.mat', 'Dynamic_IMU', 'static_IMU', 'acc_var', 'gyro_var', 'GNSS_numeric', 'Dynamic_GNSS');