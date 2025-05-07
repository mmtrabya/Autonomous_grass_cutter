#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

class UltrasonicSubscriber : public rclcpp::Node {
public:
    UltrasonicSubscriber() : Node("ultrasonic_listener") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Range>(
            "ultrasonic_range", 10, 
            std::bind(&UltrasonicSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const sensor_msgs::msg::Range::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Distance: %f meters", msg->range);
    }

    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UltrasonicSubscriber>());
    rclcpp::shutdown();
    return 0;
}
