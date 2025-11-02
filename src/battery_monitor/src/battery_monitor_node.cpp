#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <fstream>
#include <string>

class BatteryMonitorNode : public rclcpp::Node {
public:
  BatteryMonitorNode()
  : Node("battery_monitor_node"), battery_level_(30.0f)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float32>("/battery/level", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&BatteryMonitorNode::updateBatteryLevel, this));

    RCLCPP_INFO(this->get_logger(), "Battery monitor started.");
  }

private:
  void updateBatteryLevel() {
    std_msgs::msg::Float32 msg;

    // --- Option 1: Read from simulated CSV file ---
    // If you have a real CSV file with battery data (like /home/raquel/battery.csv),
    // uncomment and adapt the code below:
    /*
    std::ifstream file("/home/raquel/battery.csv");
    if (file.is_open()) {
      float value;
      if (file >> value) {
        battery_level_ = value;
      }
      file.close();
    }
    */

    // --- Option 2: Simulate gradual discharge (default for testing) ---
    battery_level_ -= 1.0f;
    if (battery_level_ < 0.0f) {
      battery_level_ = 30.0f;  // reset for continuous testing
    }

    msg.data = battery_level_;
    publisher_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Battery level: %.2f%%", battery_level_);
  }

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  float battery_level_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatteryMonitorNode>());
  rclcpp::shutdown();
  return 0;
}

