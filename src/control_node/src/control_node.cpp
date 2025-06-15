#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sstream>
#include <chrono>
#include <thread>

class ControlNode : public rclcpp::Node {
public:
    ControlNode() : Node("control_node") {
        // Set your Arduino serial port
        serial_fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Serial port opened.");
        }
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&ControlNode::cmdVelCallback, this, std::placeholders::_1));
    }

    ~ControlNode() {
        if (serial_fd_ != -1) {
            close(serial_fd_);
        }
    }

private:
    int serial_fd_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double linear = msg->linear.x;
        double angular = msg->angular.z;
        
        // Calculate wheel speeds
        int left_speed = static_cast<int>((linear - angular) * 100);
        int right_speed = static_cast<int>((linear + angular) * 100);
        
        // Create command string
        std::stringstream ss;
        ss << "M:" << left_speed << ":" << right_speed << "\n";
        std::string command = ss.str();
        
        // Send command via serial
        if (serial_fd_ != -1) {
            ssize_t bytes_written = write(serial_fd_, command.c_str(), command.size());
            
            // Log the sent message
            RCLCPP_INFO(this->get_logger(), "Sent: %s (bytes: %ld)", 
                       command.c_str(), bytes_written);
            
            // Add delay after sending (50ms = 50000 microseconds)
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            
            // Alternative delay methods:
            // usleep(50000);  // 50ms in microseconds
            // std::this_thread::sleep_for(std::chrono::microseconds(50000));
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
