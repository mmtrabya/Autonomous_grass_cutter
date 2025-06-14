#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <cmath>

class FakeImuNode : public rclcpp::Node {
public:
  FakeImuNode() : Node("fake_imu") {
    declare_parameter("publish_rate", 30.0);
    declare_parameter("imu_frame_id", "imu_link");

    publish_rate_ = get_parameter("publish_rate").as_double();
    imu_frame_id_ = get_parameter("imu_frame_id").as_string();

    yaw_ = 0.0;

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_),
      std::bind(&FakeImuNode::publish_fake_imu, this)
    );

    RCLCPP_INFO(get_logger(), "Fake IMU Node started");
  }

private:
  void publish_fake_imu() {
    double dt = 1.0 / publish_rate_;

    // Simulate slow yaw rotation around Z axis
    yaw_ += 0.1 * dt;  // 0.1 rad/s angular velocity
    if (yaw_ > M_PI) yaw_ -= 2.0 * M_PI;
    if (yaw_ < -M_PI) yaw_ += 2.0 * M_PI;

    auto now = this->now();

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = now;
    imu_msg.header.frame_id = imu_frame_id_;

    imu_msg.orientation = euler_to_quaternion(0.0, 0.0, yaw_);

    // Angular velocity (around Z axis only)
    imu_msg.angular_velocity.x = 0.0;
    imu_msg.angular_velocity.y = 0.0;
    imu_msg.angular_velocity.z = 0.1;

    // Linear acceleration (gravity on z-axis)
    imu_msg.linear_acceleration.x = 0.0;
    imu_msg.linear_acceleration.y = 0.0;
    imu_msg.linear_acceleration.z = 9.81;

    // Zero covariance for simplicity, set diagonals for typical noise
    for (int i = 0; i < 9; i++) {
      imu_msg.orientation_covariance[i] = 0.0;
      imu_msg.angular_velocity_covariance[i] = 0.0;
      imu_msg.linear_acceleration_covariance[i] = 0.0;
    }
    imu_msg.orientation_covariance[0] = 0.01;
    imu_msg.orientation_covariance[4] = 0.01;
    imu_msg.orientation_covariance[8] = 0.005;

    imu_msg.angular_velocity_covariance[0] = 0.0001;
    imu_msg.angular_velocity_covariance[4] = 0.0001;
    imu_msg.angular_velocity_covariance[8] = 0.0001;

    imu_msg.linear_acceleration_covariance[0] = 0.01;
    imu_msg.linear_acceleration_covariance[4] = 0.01;
    imu_msg.linear_acceleration_covariance[8] = 0.01;

    imu_pub_->publish(imu_msg);
  }

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

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double publish_rate_;
  std::string imu_frame_id_;
  double yaw_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeImuNode>());
  rclcpp::shutdown();
  return 0;
}
