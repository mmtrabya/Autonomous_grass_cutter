#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"  // NEW: Added for publishing serial output
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <chrono>    // NEW: Added for delay functionality
#include <thread>    // NEW: Added for sleep functionality

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
          serial_fd(-1),
          serial_delay_ms_(500) {   // NEW: Added configurable delay variable (default 500ms = 0.5 seconds)

        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&VelocityControlNode::cmd_vel_callback, this, std::placeholders::_1));

        wheel_odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/wheel_odom", 10, std::bind(&VelocityControlNode::wheel_odom_callback, this, std::placeholders::_1));

        // NEW: Create publisher for serial output messages
        serial_output_publisher_ = this->create_publisher<std_msgs::msg::String>("/serial_output", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&VelocityControlNode::check_serial_connection, this));

        // NEW: Declare ROS2 parameter for configurable serial delay (default 500ms = 0.5 seconds)
        this->declare_parameter("serial_delay_ms", 500);
        serial_delay_ms_ = this->get_parameter("serial_delay_ms").as_int();

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
        
        // MODIFIED: Enhanced serial write with better error checking
        ssize_t bytes_written = write(serial_fd, buffer, len);
        if (bytes_written != len) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write to serial port: %s", strerror(errno));
        } else {
            // NEW: Publish the serial output to a ROS topic
            auto serial_msg = std_msgs::msg::String();
            serial_msg.data = std::string(buffer);
            serial_output_publisher_->publish(serial_msg);
            
            // NEW: Optional debug logging to see what's being sent
            // (Use RCLCPP_DEBUG to avoid log spam - only shows with debug level)
            RCLCPP_DEBUG(this->get_logger(), "Sent: %s (bytes: %ld)", buffer, bytes_written);
            
            // NEW: Add configurable delay after successful serial transmission
            // This prevents overwhelming the Arduino and ensures reliable communication
            std::this_thread::sleep_for(std::chrono::milliseconds(serial_delay_ms_));
            
            // ALTERNATIVE DELAY METHODS (comment out the above line and use one of these):
            // usleep(serial_delay_ms_ * 1000);  // Convert ms to microseconds
            // std::this_thread::sleep_for(std::chrono::microseconds(serial_delay_ms_ * 1000));
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
            // MODIFIED: Added delay info to initialization log (500ms = 0.5 seconds)
            RCLCPP_INFO(this->get_logger(), "Serial port initialized successfully with %dms (%.1fs) delay", serial_delay_ms_, serial_delay_ms_/1000.0);
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
    
    // NEW: Publisher for serial output
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serial_output_publisher_;

    // PID
    PID pid_left;
    PID pid_right;

    // Serial
    int serial_fd;
    int serial_delay_ms_;    // NEW: Configurable delay in milliseconds (default 500ms = 0.5 seconds)

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

/*
=== SUMMARY OF CHANGES FOR SERIAL DELAY ===

1. ADDED INCLUDES:
   - #include <chrono>    // For time-based delays
   - #include <thread>    // For sleep functions
   - #include "std_msgs/msg/string.hpp"  // For publishing serial output

2. NEW MEMBER VARIABLE:
   - int serial_delay_ms_;  // Configurable delay in milliseconds (default: 500ms = 0.5 seconds)

3. NEW ROS2 PARAMETER:
   - "serial_delay_ms" parameter allows runtime configuration of delay
   - Can be set via command line: --ros-args -p serial_delay_ms:=100

4. ENHANCED SERIAL TRANSMISSION:
   - Better error checking with ssize_t bytes_written
   - Debug logging to see transmitted messages
   - Configurable delay after successful transmission

5. DELAY IMPLEMENTATION:
   - std::this_thread::sleep_for(std::chrono::milliseconds(serial_delay_ms_))
   - Only delays after successful write operations
   - Prevents overwhelming Arduino with rapid commands

6. NEW PUBLISHER:
   - "/serial_output" topic publishes std_msgs/String messages
   - Contains exact serial command sent to Arduino
   - Published only on successful transmission

=== SERIAL MESSAGE FORMAT ===
Your messages are sent as: "L<left_control>R<right_control>\n"
Examples:
- "L1.25R1.30\n" - Left: 1.25, Right: 1.30
- "L-0.50R0.75\n" - Left: -0.50, Right: 0.75

=== USAGE EXAMPLES ===
# Default 500ms delay (0.5 seconds):
ros2 run your_package velocity_control_node

# Custom 1000ms delay (1 second):
ros2 run your_package velocity_control_node --ros-args -p serial_delay_ms:=1000

# Faster 250ms delay (0.25 seconds):
ros2 run your_package velocity_control_node --ros-args -p serial_delay_ms:=250

=== MONITORING SERIAL OUTPUT ===
# Listen to what's being sent to Arduino:
ros2 topic echo /serial_output

# Monitor in real-time:
ros2 topic hz /serial_output
*/
