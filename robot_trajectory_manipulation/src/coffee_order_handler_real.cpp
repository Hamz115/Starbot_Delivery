#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>
#include <string>
#include <cstdlib>

class CoffeeOrderHandlerReal : public rclcpp::Node
{
public:
    CoffeeOrderHandlerReal() : Node("coffee_order_handler_real")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/coffee_orders", 10,
            std::bind(&CoffeeOrderHandlerReal::order_callback, this, std::placeholders::_1));

        feedback_publisher_ = this->create_publisher<std_msgs::msg::String>("/robot_feedback", 10);
    }

private:
    void order_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string order = msg->data;
        
        // Parse order type
        size_t delimiter_pos = order.find("|");
        std::string order_name = order.substr(0, delimiter_pos);
        std::string order_type = order.substr(delimiter_pos + 1);

        if (order_type == "real")
        {
            
            int result = system("ros2 launch robot_trajectory_manipulation planning_scene_real.launch.py");
            
            auto feedback = std::make_unique<std_msgs::msg::String>();
            if (result == 0) {
                feedback->data = "Order completed successfully";
            } else {
                feedback->data = "Error processing order";
            }
            feedback_publisher_->publish(*feedback);
        }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CoffeeOrderHandlerReal>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}