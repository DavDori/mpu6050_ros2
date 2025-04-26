#include <chrono>
#include <memory>
#include "mpu6050/mpu6050sensor.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#define DEG2RAD(x) ((x) * M_PI / 180.0)
using namespace std::chrono_literals;

class MPU6050Driver : public rclcpp::Node {
 public:
  
MPU6050Driver() : Node("mpu6050node"), 
  mpu6050_{std::make_unique<MPU6050Sensor>()}
{
  // Declare parameters
  declareParameters();
  printParameters();
  // Set parameters to the sensor
  setupSensor();
  if (this->get_parameter("calibrate").as_bool()) 
  {
    RCLCPP_INFO(this->get_logger(), "Calibrating...");
    mpu6050_->calibrate();
  }
  mpu6050_->printOffsets();
  // Create publisher
  publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
  double frequency_hz = static_cast<double>(this->get_parameter("frequency").as_int());
  auto period_ms = std::chrono::milliseconds(static_cast<int>(1000.0 / frequency_hz));
  timer_ = this->create_wall_timer(
    period_ms,
    std::bind(&MPU6050Driver::publishIMU, this)
  );
}

private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  std::unique_ptr<MPU6050Sensor> mpu6050_;
  size_t count_;
  rclcpp::TimerBase::SharedPtr timer_;

  void publishIMU()
  {
    auto message = sensor_msgs::msg::Imu();
    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = "imu_link";
    message.linear_acceleration.x = mpu6050_->getAccelerationX();
    message.linear_acceleration.y = mpu6050_->getAccelerationY();
    message.linear_acceleration.z = mpu6050_->getAccelerationZ();
    message.angular_velocity.x = DEG2RAD(mpu6050_->getAngularVelocityXdegps());
    message.angular_velocity.y = DEG2RAD(mpu6050_->getAngularVelocityYdegps());
    message.angular_velocity.z = DEG2RAD(mpu6050_->getAngularVelocityZdegps());
    // Invalidate quaternion
    message.orientation.x = 0;
    message.orientation.y = 0;
    message.orientation.z = 0;
    message.orientation.w = 1;
    publisher_->publish(message);
  }


  void setupSensor()
  {
    // Set the sensor configuration
    mpu6050_->setGyroscopeRange(
      static_cast<MPU6050Sensor::GyroRange>(this->get_parameter("gyro_range").as_int()));
    mpu6050_->setAccelerometerRange(
      static_cast<MPU6050Sensor::AccelRange>(this->get_parameter("accel_range").as_int()));
    mpu6050_->setDlpfBandwidth(
      static_cast<MPU6050Sensor::DlpfBandwidth>(this->get_parameter("dlpf_bandwidth").as_int()));
    mpu6050_->setGyroscopeOffset(this->get_parameter("gyro_x_offset").as_double(),
                              this->get_parameter("gyro_y_offset").as_double(),
                              this->get_parameter("gyro_z_offset").as_double());
    mpu6050_->setAccelerometerOffset(this->get_parameter("accel_x_offset").as_double(),
                                  this->get_parameter("accel_y_offset").as_double(),
                                  this->get_parameter("accel_z_offset").as_double());
  }
  void declareParameters()
  {
    this->declare_parameter<bool>("calibrate", true);
    this->declare_parameter<int>("gyro_range", MPU6050Sensor::GyroRange::GYR_250_DEG_S);
    this->declare_parameter<int>("accel_range", MPU6050Sensor::AccelRange::ACC_2_G);
    this->declare_parameter<int>("dlpf_bandwidth", MPU6050Sensor::DlpfBandwidth::DLPF_260_HZ);
    this->declare_parameter<double>("gyro_x_offset", 0.0);
    this->declare_parameter<double>("gyro_y_offset", 0.0);
    this->declare_parameter<double>("gyro_z_offset", 0.0);
    this->declare_parameter<double>("accel_x_offset", 0.0);
    this->declare_parameter<double>("accel_y_offset", 0.0);
    this->declare_parameter<double>("accel_z_offset", 0.0);
    this->declare_parameter<int>("frequency", 100);
  }

  void printParameters()
  {
    bool calibrate = this->get_parameter("calibrate").as_bool();
    int gyro_range = this->get_parameter("gyro_range").as_int();
    int accel_range = this->get_parameter("accel_range").as_int();
    int dlpf_bandwidth = this->get_parameter("dlpf_bandwidth").as_int();
    double gyro_x_offset = this->get_parameter("gyro_x_offset").as_double();
    double gyro_y_offset = this->get_parameter("gyro_y_offset").as_double();
    double gyro_z_offset = this->get_parameter("gyro_z_offset").as_double();
    double accel_x_offset = this->get_parameter("accel_x_offset").as_double();
    double accel_y_offset = this->get_parameter("accel_y_offset").as_double();
    double accel_z_offset = this->get_parameter("accel_z_offset").as_double();
    int frequency = this->get_parameter("frequency").as_int();

    RCLCPP_INFO(this->get_logger(), "Parameters:");
    RCLCPP_INFO(this->get_logger(), "  calibrate: %s", calibrate ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  gyro_range: %d", gyro_range);
    RCLCPP_INFO(this->get_logger(), "  accel_range: %d", accel_range);
    RCLCPP_INFO(this->get_logger(), "  dlpf_bandwidth: %d", dlpf_bandwidth);
    RCLCPP_INFO(this->get_logger(), "  gyro offset: %.6f, %.6f, %.6f", gyro_x_offset, gyro_y_offset, gyro_z_offset);
    RCLCPP_INFO(this->get_logger(), "  accel offset: %.6f, %.6f, %.6f", accel_x_offset, accel_y_offset, accel_z_offset);
    RCLCPP_INFO(this->get_logger(), "  frequency: %d", frequency);
  }
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MPU6050Driver>());
  rclcpp::shutdown();
  return 0;
}