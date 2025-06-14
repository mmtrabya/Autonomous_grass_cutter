#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

// PID controller
class PID {
public:
    PID(double p, double i, double d)
        : Kp(p), Ki(i), Kd(d), prev_error(0), integral(0) {}

    double compute(double setpoint, double current_value) {
        double error = setpoint - current_value;
        integral += error;
        double derivative = error - prev_error;
        double output = Kp * error + Ki * integral + Kd * derivative;
        prev_error = error;
        return output;
    }

private:
    double Kp, Ki, Kd;
    double prev_error;
    double integral;
};

// Main node
class VelocityControlNode : public rclcpp::Node {
public:
    VelocityControlNode()
        : Node("velocity_control_node"),
          pid_left(1.0, 0.1, 0.05),
          pid_right(1.0, 0.1, 0.05),
          serial_fd(-1) {

        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&VelocityControlNode::cmd_vel_callback, this, std::placeholders::_1));

        wheel_odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/wheel_odom", 10, std::bind(&VelocityControlNode::wheel_odom_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&VelocityControlNode::check_serial_connection, this));

        initializeSerial();
    }

    ~VelocityControlNode() {
        if (serial_fd != -1) {
            close(serial_fd);
        }
    }

private:
    void wheel_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double linear = msg->twist.twist.linear.x;
        double angular = msg->twist.twist.angular.z;

        // Convert linear and angular velocities to left/right wheel velocities
        double wheel_base = 0.5; // adjust as needed
        left_encoder_velocity = linear - (angular * wheel_base / 2.0);
        right_encoder_velocity = linear + (angular * wheel_base / 2.0);
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (serial_fd == -1) {
            RCLCPP_ERROR(this->get_logger(), "Serial port not available");
            return;
        }

        double linear = msg->linear.x;
        double angular = msg->angular.z;
        double wheel_base = 0.5; // adjust for your robot

        double desired_left = linear - (angular * wheel_base / 2.0);
        double desired_right = linear + (angular * wheel_base / 2.0);

        double control_left = pid_left.compute(desired_left, left_encoder_velocity);
        double control_right = pid_right.compute(desired_right, right_encoder_velocity);

        char buffer[50];
        int len = snprintf(buffer, sizeof(buffer), "L%.2fR%.2f\n", control_left, control_right);
        if (write(serial_fd, buffer, len) != len) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write to serial port: %s", strerror(errno));
        }
    }

    void initializeSerial() {
        serial_fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
        if (serial_fd < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", strerror(errno));
            return;
        }

        struct termios tty{};
        memset(&tty, 0, sizeof(tty));

        cfsetospeed(&tty, B9600);
        cfsetispeed(&tty, B9600);
        tty.c_cflag = CS8 | CREAD | CLOCAL;
        tty.c_iflag = IGNPAR;
        tty.c_oflag = 0;
        tty.c_lflag = 0;

        if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr: %s", strerror(errno));
            close(serial_fd);
            serial_fd = -1;
        } else {
            usleep(2000000);
            RCLCPP_INFO(this->get_logger(), "Serial port initialized successfully");
        }
    }

    void check_serial_connection() {
        if (serial_fd == -1) {
            RCLCPP_INFO(this->get_logger(), "Attempting to reconnect to serial port...");
            initializeSerial();
        }
    }

    // Subscribers and timer
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    // PID
    PID pid_left;
    PID pid_right;

    // Serial
    int serial_fd;

    // Encoder velocities from odometry
    double left_encoder_velocity = 0.0;
    double right_encoder_velocity = 0.0;
};

// Main
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityControlNode>());
    rclcpp::shutdown();
    return 0;
}

