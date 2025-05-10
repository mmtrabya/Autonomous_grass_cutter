#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <chrono>
#include <cmath>

// MPU6050 Constants
#define MPU6050_ADDRESS 0x68
#define PWR_MGMT_1      0x6B
#define SMPLRT_DIV      0x19
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C
#define ACCEL_XOUT_H    0x3B
#define ACCEL_YOUT_H    0x3D
#define ACCEL_ZOUT_H    0x3F
#define GYRO_XOUT_H     0x43
#define GYRO_YOUT_H     0x45
#define GYRO_ZOUT_H     0x47

using namespace std::chrono_literals;

class SensorDriver : public rclcpp::Node {
public:
  SensorDriver() : Node("sensor_driver") {
    // Parameters
    declare_parameter("publish_rate", 30.0);
    declare_parameter("imu_frame_id", "imu_link");

    imu_frame_id_ = get_parameter("imu_frame_id").as_string();
    double publish_rate = get_parameter("publish_rate").as_double();

    // Publishers
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);

    // Initialize hardware
    if (!init_mpu6050()) {
      RCLCPP_ERROR(get_logger(), "Hardware initialization failed!");
      rclcpp::shutdown();
      return;
    }

    // Initialize time
    last_time_ = this->now();

    // Main timer
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0/publish_rate),
      std::bind(&SensorDriver::publish_data, this)
    );

    RCLCPP_INFO(get_logger(), "IMU sensor driver initialized successfully!");
  }

private:
  bool init_mpu6050() {
    fd_ = wiringPiI2CSetup(MPU6050_ADDRESS);
    if (fd_ == -1) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize MPU6050");
      return false;
    }
    
    wiringPiI2CWriteReg8(fd_, PWR_MGMT_1, 0x00);  // Wake up
    wiringPiI2CWriteReg8(fd_, SMPLRT_DIV, 0x07);   // 125Hz sample rate
    wiringPiI2CWriteReg8(fd_, ACCEL_CONFIG, 0x08); // ±4g
    wiringPiI2CWriteReg8(fd_, GYRO_CONFIG, 0x08);  // ±500°/s
    
    // Verify communication
    int test_reg = wiringPiI2CReadReg8(fd_, PWR_MGMT_1);
    if (test_reg < 0) {
      RCLCPP_ERROR(get_logger(), "Communication with MPU6050 failed");
      return false;
    }
    
    // Initialize IMU orientation (assume we start level)
    yaw_ = 0.0;
    
    RCLCPP_INFO(get_logger(), "MPU6050 initialized successfully");
    return true;
  }

  int16_t read_word_2c(int addr) {
    uint16_t high = wiringPiI2CReadReg8(fd_, addr);
    uint16_t low = wiringPiI2CReadReg8(fd_, addr + 1);
    uint16_t value = (high << 8) | low;
    
    // Convert to signed 16-bit integer
    return static_cast<int16_t>(value);
  }

  // Convert euler angles to quaternion
  geometry_msgs::msg::Quaternion euler_to_quaternion(double roll, double pitch, double yaw) {
    geometry_msgs::msg::Quaternion q;
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;

    return q;
  }

  void publish_data() {
    auto now = this->now();
    auto dt = (now - last_time_).seconds();
    last_time_ = now;
    
    // Skip first iteration (dt = 0)
    if (dt < 0.0001) return;
    
    // --------------------------
    // Read and publish IMU data
    // --------------------------
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = now;
    imu_msg.header.frame_id = imu_frame_id_;
    
    // Read accelerometer (convert to m/s^2)
    double accel_scale = 9.81 / 8192.0;  // For ±4g range
    imu_msg.linear_acceleration.x = read_word_2c(ACCEL_XOUT_H) * accel_scale;
    imu_msg.linear_acceleration.y = read_word_2c(ACCEL_YOUT_H) * accel_scale;
    imu_msg.linear_acceleration.z = read_word_2c(ACCEL_ZOUT_H) * accel_scale;
    
    // Read gyroscope (convert to rad/s)
    double gyro_scale = (M_PI/180.0) / 65.5;  // For ±500°/s range
    double gyro_x = read_word_2c(GYRO_XOUT_H) * gyro_scale;
    double gyro_y = read_word_2c(GYRO_YOUT_H) * gyro_scale;
    double gyro_z = read_word_2c(GYRO_ZOUT_H) * gyro_scale;
    
    imu_msg.angular_velocity.x = gyro_x;
    imu_msg.angular_velocity.y = gyro_y;
    imu_msg.angular_velocity.z = gyro_z;
    
    // Update yaw estimate with gyro integration
    yaw_ += gyro_z * dt;
    
    // Convert to quaternion (assume roll and pitch are 0 for ground robot)
    imu_msg.orientation = euler_to_quaternion(0.0, 0.0, yaw_);
    
    // Set covariance (tune these values based on your IMU performance)
    for (int i = 0; i < 9; i++) {
      imu_msg.orientation_covariance[i] = 0.0;
      imu_msg.angular_velocity_covariance[i] = 0.0;
      imu_msg.linear_acceleration_covariance[i] = 0.0;
    }
    
    // Diagonal elements (variances)
    imu_msg.orientation_covariance[0] = 0.01;  // roll
    imu_msg.orientation_covariance[4] = 0.01;  // pitch
    imu_msg.orientation_covariance[8] = 0.005; // yaw
    
    imu_msg.angular_velocity_covariance[0] = 0.0001;  // roll rate
    imu_msg.angular_velocity_covariance[4] = 0.0001;  // pitch rate
    imu_msg.angular_velocity_covariance[8] = 0.0001;  // yaw rate
    
    imu_msg.linear_acceleration_covariance[0] = 0.01;  // x accel
    imu_msg.linear_acceleration_covariance[4] = 0.01;  // y accel
    imu_msg.linear_acceleration_covariance[8] = 0.01;  // z accel
    
    imu_pub_->publish(imu_msg);
  }

  // Hardware variables
  int fd_;
  std::string imu_frame_id_;
  
  // State variables
  double yaw_;           // IMU yaw estimate
  rclcpp::Time last_time_;
  
  // ROS components
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorDriver>());
  rclcpp::shutdown();
  return 0;
}