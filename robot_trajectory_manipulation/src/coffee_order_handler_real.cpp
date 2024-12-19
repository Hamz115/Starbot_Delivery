#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <cstdlib>
#include <memory>
#include <string>

class CoffeeOrderHandlerReal : public rclcpp::Node {
public:
  CoffeeOrderHandlerReal() : Node("coffee_order_handler_real") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/coffee_orders", 10,
        std::bind(&CoffeeOrderHandlerReal::order_callback, this,
                  std::placeholders::_1));

    feedback_publisher_ =
        this->create_publisher<std_msgs::msg::String>("/robot_feedback", 10);
    invoice_publisher_ =
        this->create_publisher<std_msgs::msg::String>("/order_invoice", 10);
  }

private:
  void order_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::string drink_name = msg->data;
    std::string launch_command = "ros2 launch robot_trajectory_manipulation ";
    std::string price;

    // Handle Cold Drinks
    if (drink_name == "Iced Coconut Caramel Latte") {
      launch_command += "hole_placement_real_1.launch.py";
      price = "$5.99";
    } else if (drink_name == "Iced Lavender Latte") {
      launch_command += "hole_placement_real_2.launch.py";
      price = "$6.49";
    } else if (drink_name == "Iced Mocha Mint") {
      launch_command += "hole_placement_real_3.launch.py";
      price = "$5.99";
    } else if (drink_name == "Iced Honey Almond Latte") {
      launch_command += "hole_placement_real_4.launch.py";
      price = "$6.49";
    } else {
      auto feedback = std::make_unique<std_msgs::msg::String>();
      feedback->data = "Error: Unknown drink order";
      feedback_publisher_->publish(*feedback);
      return;
    }

    // Execute the launch file
    int result = system(launch_command.c_str());

    auto feedback = std::make_unique<std_msgs::msg::String>();
    auto invoice = std::make_unique<std_msgs::msg::String>();

    if (result == 0) {
      feedback->data = "Order completed successfully";

      // Publish invoice details
      invoice->data =
          "Drink: " + drink_name + "\nPrice: " + price + "\nStatus: Delivered";
      invoice_publisher_->publish(*invoice);

    } else {
      feedback->data = "Error processing order";

      // Publish error invoice
      invoice->data = "Drink: " + drink_name + "\nPrice: " + price +
                      "\nStatus: Error processing order";
      invoice_publisher_->publish(*invoice);
    }

    feedback_publisher_->publish(*feedback);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr invoice_publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CoffeeOrderHandlerReal>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}