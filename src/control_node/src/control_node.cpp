#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <serial/serial.h>
#include <cmath>
#include <chrono>
#include <string>
#include <algorithm>

using namespace std::chrono_literals;

class ControlNode : public rclcpp::Node
{
public:
    VelocityControlNode()
        : Node("control_node"),
          target_linear_velocity_(0.0),
          target_angular_velocity_(0.0),
          current_linear_velocity_(0.0),
          current_angular_velocity_(0.0),
          previous_linear_error_(0.0),
          previous_angular_error_(0.0),
          integral_linear_error_(0.0),
          integral_angular_error_(0.0),
          wheel_base_(0.3) // Adjust based on your robot
    {
        RCLCPP_INFO(this->get_logger(), "ControlNode started.");

        // Initialize PID gains (adjust as needed)
        Kp_linear_ = 30.0;
        Ki_linear_ = 0.0;
        Kd_linear_ = 2.0;

        Kp_angular_ = 20.0;
        Ki_angular_ = 0.0;
        Kd_angular_ = 1.5;

        // Subscribe to /cmd_vel
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&VelocityControlNode::cmd_vel_callback, this, std::placeholders::_1));

        // Subscribe to wheel odometry
        wheel_odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "wheel_odom", 10,
            std::bind(&VelocityControlNode::wheel_odom_callback, this, std::placeholders::_1));

        // Timer to update control loop
        timer_ = this->create_wall_timer(
            100ms, std::bind(&VelocityControlNode::update_control_loop, this));

        // Initialize serial port
        try {
            serial_port_.setPort("/dev/ttyUSB0");  // Adjust for your system
            serial_port_.setBaudrate(9600);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            serial_port_.setTimeout(to);
            serial_port_.open();
            if (serial_port_.isOpen()) {
                RCLCPP_INFO(this->get_logger(), "Serial port opened successfully.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
            }
        } catch (serial::IOException &e) {
            RCLCPP_ERROR(this->get_logger(), "Serial IOException: %s", e.what());
        }
    }

private:
    // Callbacks
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        target_linear_velocity_ = msg->linear.x;
        target_angular_velocity_ = msg->angular.z;
    }

    void wheel_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_linear_velocity_ = msg->twist.twist.linear.x;
        current_angular_velocity_ = msg->twist.twist.angular.z;
    }

    void update_control_loop()
    {
        double dt = 0.1; // seconds (100ms)

        // Calculate linear PID control
        double linear_error = target_linear_velocity_ - current_linear_velocity_;
        integral_linear_error_ += linear_error * dt;
        double derivative_linear_error = (linear_error - previous_linear_error_) / dt;
        double linear_output = Kp_linear_ * linear_error +
                               Ki_linear_ * integral_linear_error_ +
                               Kd_linear_ * derivative_linear_error;

        previous_linear_error_ = linear_error;

        // Calculate angular PID control
        double angular_error = target_angular_velocity_ - current_angular_velocity_;
        integral_angular_error_ += angular_error * dt;
        double derivative_angular_error = (angular_error - previous_angular_error_) / dt;
        double angular_output = Kp_angular_ * angular_error +
                                Ki_angular_ * integral_angular_error_ +
                                Kd_angular_ * derivative_angular_error;

        previous_angular_error_ = angular_error;

        // Convert linear and angular to wheel velocities
        double left_pwm = linear_output - (angular_output * wheel_base_ / 2.0);
        double right_pwm = linear_output + (angular_output * wheel_base_ / 2.0);

        // Clip PWM values
        int left_pwm_int = std::clamp(static_cast<int>(left_pwm), -255, 255);
        int right_pwm_int = std::clamp(static_cast<int>(right_pwm), -255, 255);

        // Format PWM command as a string to send over serial (e.g., "L100R100\n")
        std::string command = "L" + std::to_string(left_pwm_int) +
                              "R" + std::to_string(right_pwm_int) + "\n";

        // Send command
        if (serial_port_.isOpen()) {
            serial_port_.write(command);
        }
    }

    // Subscriptions and Timer
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Serial communication
    serial::Serial serial_port_;

    // PID Variables
    double Kp_linear_, Ki_linear_, Kd_linear_;
    double Kp_angular_, Ki_angular_, Kd_angular_;
    double previous_linear_error_, previous_angular_error_;
    double integral_linear_error_, integral_angular_error_;

    // Target and current velocities
    double target_linear_velocity_;
    double target_angular_velocity_;
    double current_linear_velocity_;
    double current_angular_velocity_;

    double wheel_base_;
};

// Main function
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}

