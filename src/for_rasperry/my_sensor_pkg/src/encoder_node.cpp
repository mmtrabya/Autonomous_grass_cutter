#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <cmath>

class FakeEncoderOdometryNode : public rclcpp::Node {
public:
  FakeEncoderOdometryNode() : Node("fake_encoder_odometry") {
    // Declare parameters
    declare_parameter("wheel_radius", 0.09);
    declare_parameter("wheel_base", 0.45);
    declare_parameter("encoder_resolution", 4096);
    declare_parameter("publish_rate", 10.0);
    declare_parameter("odom_frame_id", "odom");
    declare_parameter("base_frame_id", "base_link");

    wheel_radius_ = get_parameter("wheel_radius").as_double();
    wheel_base_ = get_parameter("wheel_base").as_double();
    encoder_resolution_ = get_parameter("encoder_resolution").as_int();
    odom_frame_id_ = get_parameter("odom_frame_id").as_string();
    base_frame_id_ = get_parameter("base_frame_id").as_string();
    publish_rate_ = get_parameter("publish_rate").as_double();

    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("wheel_odom", 10);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_),
      std::bind(&FakeEncoderOdometryNode::publish_fake_odometry, this)
    );

    RCLCPP_INFO(get_logger(), "Fake Encoder Odometry Node started");
  }

private:
  void publish_fake_odometry() {
    // Simulate robot moving forward and turning slightly
    double dt = 1.0 / publish_rate_;
    
    double linear_velocity = 0.2;  // m/s
    double angular_velocity = 0.1; // rad/s

    x_ += linear_velocity * std::cos(theta_) * dt;
    y_ += linear_velocity * std::sin(theta_) * dt;
    theta_ += angular_velocity * dt;

    // normalize theta
    if (theta_ > M_PI) theta_ -= 2.0 * M_PI;
    if (theta_ < -M_PI) theta_ += 2.0 * M_PI;

    auto now = this->now();

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = now;
    msg.header.frame_id = odom_frame_id_;
    msg.child_frame_id = base_frame_id_;

    msg.pose.pose.position.x = x_;
    msg.pose.pose.position.y = y_;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation = euler_to_quaternion(0.0, 0.0, theta_);

    msg.twist.twist.linear.x = linear_velocity;
    msg.twist.twist.angular.z = angular_velocity;

    // covariance zero for simplicity
    std::fill(std::begin(msg.pose.covariance), std::end(msg.pose.covariance), 0.0);
    std::fill(std::begin(msg.twist.covariance), std::end(msg.twist.covariance), 0.0);

    // some small variances, tuned for example
    msg.pose.covariance[0] = 0.01;   // x pos var
    msg.pose.covariance[7] = 0.01;   // y pos var
    msg.pose.covariance[35] = 0.01;  // yaw var

    msg.twist.covariance[0] = 0.01;  // x vel var
    msg.twist.covariance[35] = 0.03; // yaw rate var

    odom_pub_->publish(msg);
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

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double wheel_radius_;
  double wheel_base_;
  int encoder_resolution_;
  double publish_rate_;
  std::string odom_frame_id_;
  std::string base_frame_id_;

  // Pose state:
  double x_, y_, theta_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeEncoderOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
