#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <cstdlib>
#include <memory>
#include <string>

class CoffeeOrderHandler : public rclcpp::Node {
public:
  CoffeeOrderHandler() : Node("coffee_order_handler") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/coffee_orders", 10,
        std::bind(&CoffeeOrderHandler::order_callback, this,
                  std::placeholders::_1));

    feedback_publisher_ =
        this->create_publisher<std_msgs::msg::String>("/robot_feedback", 10);
  }

private:
  void order_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::string drink_name = msg->data;
    std::string launch_command = "ros2 launch robot_trajectory_manipulation ";

    // Determine which launch file to use based on the drink
    if (drink_name == "Classic Espresso") {
      launch_command += "hole_placement_1.launch.py";
    } else if (drink_name == "Caramel Macchiato") {
      launch_command += "hole_placement_2.launch.py";
    } else if (drink_name == "Matcha Latte") {
      launch_command += "hole_placement_3.launch.py";
    } else if (drink_name == "Vanilla Frappuccino") {
      launch_command += "hole_placement_4.launch.py";
    } else {
      auto feedback = std::make_unique<std_msgs::msg::String>();
      feedback->data = "Error: Unknown drink order";
      feedback_publisher_->publish(*feedback);
      return;
    }

    // Execute the launch file
    int result = system(launch_command.c_str());

    auto feedback = std::make_unique<std_msgs::msg::String>();
    if (result == 0) {
      feedback->data = "Order completed successfully";
    } else {
      feedback->data = "Error processing order";
    }
    feedback_publisher_->publish(*feedback);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CoffeeOrderHandler>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}