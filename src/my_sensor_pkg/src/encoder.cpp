#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <wiringPi.h>
#include <cmath>

// Encoder Pins
#define ENCODER_LEFT_A  13
#define ENCODER_LEFT_B  19
#define ENCODER_RIGHT_A 5
#define ENCODER_RIGHT_B 6

// Global variables for encoder counts
volatile int g_left_ticks = 0;
volatile int g_right_ticks = 0;

// ISR functions for encoder interrupts
void encoder_left_isr(void) {
    bool a = digitalRead(ENCODER_LEFT_A);
    bool b = digitalRead(ENCODER_LEFT_B);
    g_left_ticks += (a == b) ? 1 : -1;
}

void encoder_right_isr(void) {
    bool a = digitalRead(ENCODER_RIGHT_A);
    bool b = digitalRead(ENCODER_RIGHT_B);
    g_right_ticks += (a != b) ? 1 : -1;
}

class EncoderOdometryNode : public rclcpp::Node {
public:
  EncoderOdometryNode() : Node("encoder_odometry") {
    // Parameters
    declare_parameter("wheel_radius", 0.09);  // meters
    declare_parameter("wheel_base", 0.2);     // meters
    declare_parameter("encoder_resolution", 4096);  // ticks per revolution
    declare_parameter("publish_rate", 10.0);  // Hz
    declare_parameter("odom_frame_id", "odom");
    declare_parameter("base_frame_id", "base_link");
    
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    wheel_base_ = get_parameter("wheel_base").as_double();
    encoder_resolution_ = get_parameter("encoder_resolution").as_int();
    odom_frame_id_ = get_parameter("odom_frame_id").as_string();
    base_frame_id_ = get_parameter("base_frame_id").as_string();
    double publish_rate = get_parameter("publish_rate").as_double();
    
    // Publisher
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("wheel_odom", 10);
    
    // Initialize hardware
    if (!init_encoders()) {
      RCLCPP_ERROR(get_logger(), "Encoder initialization failed!");
      rclcpp::shutdown();
      return;
    }
    
    // Initialize time, encoder values, and odometry state
    last_time_ = this->now();
    last_left_ticks_ = 0;
    last_right_ticks_ = 0;
    
    // Position and orientation
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
    
    // Velocity
    linear_velocity_ = 0.0;
    angular_velocity_ = 0.0;
    
    // Main timer
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0/publish_rate),
      std::bind(&EncoderOdometryNode::update_and_publish_odometry, this)
    );
    
    RCLCPP_INFO(get_logger(), "Encoder odometry node initialized successfully!");
    RCLCPP_INFO(get_logger(), "Parameters - wheel_radius: %.3f m, wheel_base: %.3f m, encoder_resolution: %d ticks",
                wheel_radius_, wheel_base_, encoder_resolution_);
  }

private:
  bool init_encoders() {
    try {
      if (wiringPiSetupGpio() == -1) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize WiringPi GPIO");
        return false;
      }
      
      // Setup encoder pins
      pinMode(ENCODER_LEFT_A, INPUT); pullUpDnControl(ENCODER_LEFT_A, PUD_UP);
      pinMode(ENCODER_LEFT_B, INPUT); pullUpDnControl(ENCODER_LEFT_B, PUD_UP);
      pinMode(ENCODER_RIGHT_A, INPUT); pullUpDnControl(ENCODER_RIGHT_A, PUD_UP);
      pinMode(ENCODER_RIGHT_B, INPUT); pullUpDnControl(ENCODER_RIGHT_B, PUD_UP);
      
      // Read initial values
      RCLCPP_INFO(get_logger(), "Initial encoder readings - Left A: %d, B: %d, Right A: %d, B: %d",
                  digitalRead(ENCODER_LEFT_A), digitalRead(ENCODER_LEFT_B),
                  digitalRead(ENCODER_RIGHT_A), digitalRead(ENCODER_RIGHT_B));
      
      // Setup interrupt service routines
      if (wiringPiISR(ENCODER_LEFT_A, INT_EDGE_BOTH, &encoder_left_isr) < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to set up left encoder ISR");
        return false;
      }
      
      if (wiringPiISR(ENCODER_RIGHT_A, INT_EDGE_BOTH, &encoder_right_isr) < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to set up right encoder ISR");
        return false;
      }
      
      RCLCPP_INFO(get_logger(), "Encoders initialized successfully");
      return true;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize encoders: %s", e.what());
      return false;
    }
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

  void update_and_publish_odometry() {
    auto now = this->now();
    auto dt = (now - last_time_).seconds();
    last_time_ = now;
    
    // Skip first iteration (dt = 0)
    if (dt < 0.0001) return;
    
    // Read current encoder values from global variables
    int current_left_ticks = g_left_ticks;
    int current_right_ticks = g_right_ticks;
    
    int ticks_diff_left = current_left_ticks - last_left_ticks_;
    int ticks_diff_right = current_right_ticks - last_right_ticks_;
    
    // Update stored values
    last_left_ticks_ = current_left_ticks;
    last_right_ticks_ = current_right_ticks;
    
    // Calculate wheel distances and velocities
    double dist_per_tick = (2.0 * M_PI * wheel_radius_) / encoder_resolution_;
    
    double left_dist = ticks_diff_left * dist_per_tick;
    double right_dist = ticks_diff_right * dist_per_tick;
    
    double left_vel = left_dist / dt;
    double right_vel = right_dist / dt;
    
    // Calculate robot motion (differential drive kinematics)
    linear_velocity_ = (left_vel + right_vel) / 2.0;
    angular_velocity_ = (right_vel - left_vel) / wheel_base_;
    
    // Update position estimate (integrate velocities)
    double delta_x = linear_velocity_ * cos(theta_) * dt;
    double delta_y = linear_velocity_ * sin(theta_) * dt;
    double delta_theta = angular_velocity_ * dt;
    
    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_theta;
    
    // Normalize theta to [-π, π]
    while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
    while (theta_ < -M_PI) theta_ += 2.0 * M_PI;
    
    // Log significant movements
    if (std::abs(delta_x) > 0.001 || std::abs(delta_y) > 0.001 || std::abs(delta_theta) > 0.001) {
      RCLCPP_INFO(get_logger(), "Position: (%.3f, %.3f, %.3f), Velocity: (%.3f, %.3f)",
                 x_, y_, theta_, linear_velocity_, angular_velocity_);
    }
    
    // Create and publish odometry message
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = base_frame_id_;
    
    // Set position
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    
    // Set orientation (assuming ground robot, so roll and pitch are 0)
    odom_msg.pose.pose.orientation = euler_to_quaternion(0.0, 0.0, theta_);
    
    // Set velocities
    odom_msg.twist.twist.linear.x = linear_velocity_;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_velocity_;
    
    // Set covariance (critical for EKF)
    // Position covariance
    for (int i = 0; i < 36; i++) {
      odom_msg.pose.covariance[i] = 0.0;
      odom_msg.twist.covariance[i] = 0.0;
    }
    
    // Diagonal elements (variances) - tune these based on your robot
    odom_msg.pose.covariance[0] = 0.01;   // x position variance
    odom_msg.pose.covariance[7] = 0.01;   // y position variance
    odom_msg.pose.covariance[35] = 0.01;  // yaw variance
    
    odom_msg.twist.covariance[0] = 0.01;  // x velocity variance
    odom_msg.twist.covariance[35] = 0.03; // yaw rate variance
    
    odom_pub_->publish(odom_msg);
  }

  // Hardware variables
  int last_left_ticks_, last_right_ticks_;
  double wheel_radius_, wheel_base_;
  int encoder_resolution_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  
  // State variables
  double x_, y_, theta_;             // Position and orientation
  double linear_velocity_, angular_velocity_; // Current velocities
  rclcpp::Time last_time_;
  
  // ROS components
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EncoderOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
