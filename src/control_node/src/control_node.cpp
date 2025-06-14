#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <chrono>
#include <string>
#include <algorithm>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

using namespace std::chrono_literals;
using namespace std::chrono;

class ControlNode : public rclcpp::Node
{
public:
    ControlNode()
        : Node("control_node"),
          target_linear_velocity_(0.0),
          target_angular_velocity_(0.0),
          current_linear_velocity_(0.0),
          current_angular_velocity_(0.0),
          previous_linear_error_(0.0),
          previous_angular_error_(0.0),
          integral_linear_error_(0.0),
          integral_angular_error_(0.0),
          wheel_base_(0.3),
          last_left_pwm_(0),
          last_right_pwm_(0),
          last_send_time_(steady_clock::now())
    {
        RCLCPP_INFO(this->get_logger(), "ControlNode started.");

        // PID gains
        Kp_linear_ = 30.0;
        Ki_linear_ = 0.0;
        Kd_linear_ = 2.0;

        Kp_angular_ = 20.0;
        Ki_angular_ = 0.0;
        Kd_angular_ = 1.5;

        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&ControlNode::cmd_vel_callback, this, std::placeholders::_1));

        wheel_odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "wheel_odom", 10, std::bind(&ControlNode::wheel_odom_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(100ms, std::bind(&ControlNode::update_control_loop, this));

        // POSIX serial setup
        serial_fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", strerror(errno));
        } else {
            struct termios tty;
            if (tcgetattr(serial_fd_, &tty) != 0) {
                RCLCPP_ERROR(this->get_logger(), "tcgetattr failed: %s", strerror(errno));
                close(serial_fd_);
                serial_fd_ = -1;
            } else {
                cfsetospeed(&tty, B9600);
                cfsetispeed(&tty, B9600);
                tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
                tty.c_iflag &= ~IGNBRK;
                tty.c_lflag = 0;
                tty.c_oflag = 0;
                tty.c_cc[VMIN] = 0;
                tty.c_cc[VTIME] = 10;
                tty.c_iflag &= ~(IXON | IXOFF | IXANY);
                tty.c_cflag |= (CLOCAL | CREAD);
                tty.c_cflag &= ~(PARENB | PARODD);
                tty.c_cflag &= ~CSTOPB;
                tty.c_cflag &= ~CRTSCTS;
                tcsetattr(serial_fd_, TCSANOW, &tty);
                RCLCPP_INFO(this->get_logger(), "Serial port opened successfully.");
            }
        }
    }

private:
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
        double dt = 0.1;

        double linear_error = target_linear_velocity_ - current_linear_velocity_;
        integral_linear_error_ += linear_error * dt;
        double derivative_linear_error = (linear_error - previous_linear_error_) / dt;
        double linear_output = Kp_linear_ * linear_error +
                               Ki_linear_ * integral_linear_error_ +
                               Kd_linear_ * derivative_linear_error;
        previous_linear_error_ = linear_error;

        double angular_error = target_angular_velocity_ - current_angular_velocity_;
        integral_angular_error_ += angular_error * dt;
        double derivative_angular_error = (angular_error - previous_angular_error_) / dt;
        double angular_output = Kp_angular_ * angular_error +
                                Ki_angular_ * integral_angular_error_ +
                                Kd_angular_ * derivative_angular_error;
        previous_angular_error_ = angular_error;

        double left_pwm = linear_output - (angular_output * wheel_base_ / 2.0);
        double right_pwm = linear_output + (angular_output * wheel_base_ / 2.0);

        int left_pwm_int = std::clamp(static_cast<int>(left_pwm), -255, 255);
        int right_pwm_int = std::clamp(static_cast<int>(right_pwm), -255, 255);

        auto now = steady_clock::now();
        auto elapsed = duration_cast<milliseconds>(now - last_send_time_);

        const int threshold = 5;             // Minimum change to trigger new command
        const int cooldown_ms = 300;         // Min time between serial sends

        if ((std::abs(left_pwm_int - last_left_pwm_) >= threshold ||
             std::abs(right_pwm_int - last_right_pwm_) >= threshold) &&
            elapsed.count() >= cooldown_ms)
        {
            std::string command = "L" + std::to_string(left_pwm_int) +
                                  "R" + std::to_string(right_pwm_int) + "\n";

            if (serial_fd_ >= 0) {
                write(serial_fd_, command.c_str(), command.length());
            }

            last_send_time_ = now;
            last_left_pwm_ = left_pwm_int;
            last_right_pwm_ = right_pwm_int;
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    int serial_fd_;

    double Kp_linear_, Ki_linear_, Kd_linear_;
    double Kp_angular_, Ki_angular_, Kd_angular_;
    double previous_linear_error_, previous_angular_error_;
    double integral_linear_error_, integral_angular_error_;
    double target_linear_velocity_;
    double target_angular_velocity_;
    double current_linear_velocity_;
    double current_angular_velocity_;
    double wheel_base_;

    int last_left_pwm_;
    int last_right_pwm_;
    std::chrono::steady_clock::time_point last_send_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
