# Autonomous Vehicle Sensor Data Fusion (EKF)

This repository contains the embedded C firmware and MATLAB algorithms developed for my Master's capstone project in Autonomous Vehicle Engineering at the University of Naples Federico II.

## 🎯 Project Objective
To estimate a highly accurate 2D vehicle trajectory by fusing high-speed data from an IMU with low-speed positional data from a GNSS module, mitigating inherent sensor drift.

## ⚙️ System Architecture

### 1. Embedded Hardware (STM32 & IKS02A1)
*   **Microcontroller:** STM32 architecture programmed in C.
*   **Sensors:** IKS02A1 Expansion Board (Accelerometer & Gyroscope) and external GNSS module.
*   **Multi-Rate Acquisition:** Handled asynchronous data streams by combining non-blocking `HAL_GetTick()` polling for the IMU (208 Hz ODR) and hardware interrupts for incoming GNSS sentences (1 Hz).
*   **Data Formatting:** Processed raw sensor floats into structured, comma-separated ASCII strings for reliable debugging and real-time transmission.
*   **Transmission:** Utilized high-speed UART (921600 baud) to safely transmit the continuous data streams to the PC.

### 2. PC Processing (MATLAB)
*   **Data Parsing:** Real-time serial reading and string parsing of both custom IMU packets and standard NMEA (`$PSTMPV`) sentences.
*   **Coordinate Transformation:** Converted raw GNSS LLA (Latitude, Longitude, Altitude) coordinates into a local NED (North-East-Down) Cartesian frame for accurate physical modeling.
*   **Extended Kalman Filter (EKF):** Dynamically fuses the multi-rate sensor data. The algorithm performs continuous dead-reckoning between the sparse 1 Hz GPS updates, successfully mitigating the gyroscope and accelerometer drift.
*   **Visualization:** Calculates the trajectory RMSE error and plots the raw GNSS data against the smoothed EKF trajectory on both local Cartesian graphs and real-world satellite maps.

## 📊 Results
The integration of the EKF effectively smoothed the noisy GPS data and corrected the inherent drift of the IMU sensors, resulting in a reliable trajectory estimation under dynamic movement.
