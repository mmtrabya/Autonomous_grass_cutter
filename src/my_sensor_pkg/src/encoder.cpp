#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
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

class EncoderTester : public rclcpp::Node {
public:
  EncoderTester() : Node("encoder_tester") {
    // Parameters
    declare_parameter("publish_rate", 10.0);
    double publish_rate = get_parameter("publish_rate").as_double();
    
    // Publisher
    encoder_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("encoder_ticks", 10);
    
    // Initialize hardware
    if (!init_encoders()) {
      RCLCPP_ERROR(get_logger(), "Encoder initialization failed!");
      rclcpp::shutdown();
      return;
    }
    
    // Initialize encoder values
    last_left_ticks_ = 0;
    last_right_ticks_ = 0;
    
    // Main timer
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0/publish_rate),
      std::bind(&EncoderTester::publish_encoder_data, this)
    );
    
    RCLCPP_INFO(get_logger(), "Encoder tester initialized successfully!");
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

  void publish_encoder_data() {
    // Read current encoder values from global variables
    int current_left_ticks = g_left_ticks;
    int current_right_ticks = g_right_ticks;
    
    int ticks_diff_left = current_left_ticks - last_left_ticks_;
    int ticks_diff_right = current_right_ticks - last_right_ticks_;
    
    // Update stored values
    last_left_ticks_ = current_left_ticks;
    last_right_ticks_ = current_right_ticks;
    
    // Create message
    auto msg = std_msgs::msg::Int32MultiArray();
    msg.data.resize(4);
    msg.data[0] = current_left_ticks;    // Left total
    msg.data[1] = current_right_ticks;   // Right total
    msg.data[2] = ticks_diff_left;       // Left delta
    msg.data[3] = ticks_diff_right;      // Right delta
    
    // Publish message
    encoder_pub_->publish(msg);
    
    // Log encoder ticks if they've changed
    if (ticks_diff_left != 0 || ticks_diff_right != 0) {
      RCLCPP_INFO(get_logger(), "Encoder ticks - Left: %d (%d diff), Right: %d (%d diff)",
                 current_left_ticks, ticks_diff_left, current_right_ticks, ticks_diff_right);
    }
  }

  // Hardware variables
  int last_left_ticks_, last_right_ticks_;
  
  // ROS components
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EncoderTester>());
  rclcpp::shutdown();
  return 0;
}
