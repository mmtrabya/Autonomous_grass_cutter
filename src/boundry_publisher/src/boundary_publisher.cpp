// src/boundary_publisher_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

class BoundaryPublisher : public rclcpp::Node {
public:
    BoundaryPublisher()
    : Node("boundary_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("boundary_points", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&BoundaryPublisher::publishNextPoint, this)
        );
        //call path
        loadCoordinates("/home/kemo/Azza_ws/src/boundry_publisher/src/coordinates.csv");
    }

private:
    void loadCoordinates(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", filepath.c_str());
            return;
        }

        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string lat, lon;
            if (std::getline(ss, lat, ',') && std::getline(ss, lon, ',')) {
                geometry_msgs::msg::Point point;
                point.x = std::stod(lat);  // latitude
                point.y = std::stod(lon);  // longitude
                point.z = 0.0;
                coordinates_.push_back(point);
            }
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %zu coordinates from CSV", coordinates_.size());
    }

    void publishNextPoint() {
        if (index_ < coordinates_.size()) {
            publisher_->publish(coordinates_[index_]);
            ++index_;
        } else {
            RCLCPP_INFO(this->get_logger(), "All boundary points published.");
            rclcpp::shutdown();
        }
    }

    std::vector<geometry_msgs::msg::Point> coordinates_;
    size_t index_ = 0;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BoundaryPublisher>());
    rclcpp::shutdown();
    return 0;
}
