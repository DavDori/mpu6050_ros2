
# MPU6050 Node for ROS2
This repository provides a ROS2 package that interfaces with an MPU6050 sensor over I2C.
On node startup, the sensor is calibrated automatically — make sure the sensor is placed flat with the Z-axis pointing upward and remains still during calibration.
Calibration can be disabled via a parameter.
The node publishes an IMU message (sensor_msgs/msg/Imu), with the quaternion currently set to the unit quaternion the acceleration in `m/s²` and the angular velocity in `rad/s`.

## Dependencies
-  libi2c-dev

## Setup
The number of iterations for calibration can be set up in `include/mpu6050driver/mpu6050sensor.h`.
Following other parameters and default values are listed here and can be changed in `config/mpu6050.yaml`.

| Parameter         | Default Value | Unit     |
|-------------------|---------------|----------|
| `calibrate`         | true           | -        |
| `gyro_range`        | 0             | -        |
| `accel_range`       | 0             | -        |
| `dlpf_bandwidth`    | 2             | -        |
| `gyro_x_offset`     | 0.0           | [deg/s]  |
| `gyro_y_offset`     | 0.0           | [deg/s]  |
| `gyro_z_offset`     | 0.0           | [deg/s]  |
| `accel_x_offset`    | 0.0           | [m/s²]   |
| `accel_y_offset`    | 0.0           | [m/s²]   |
| `accel_z_offset`    | 0.0           | [m/s²]   |
| `frequency`         | 100           | [Hz]     |


Build the package in your workspace:

    colcon build --packages-select mpu6050

Source setup.bash in your workspace:

    . install/setup.bash
    
Launch it:

    ros2 launch mpu6050 driver.launch.py

