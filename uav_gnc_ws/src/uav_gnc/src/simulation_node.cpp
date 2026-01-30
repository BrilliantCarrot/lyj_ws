#include <chrono>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("dummy_node");
  // basic pub test
  auto pub = node->create_publisher<std_msgs::msg::String>("/dummy/chatter", 10);
  // basic sub test
  auto sub = node->create_subscription<std_msgs::msg::String>(
  "/dummy/in", 10,
  [node](const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(node->get_logger(), "received: %s", msg->data.c_str());
  });

  auto timer = node->create_wall_timer(500ms, [node, pub]() {
    std_msgs::msg::String msg;
    msg.data = "hello from dummy_node";
    pub->publish(msg);
    // RCLCPP_INFO(node->get_logger(), "publish: %s", msg.data.c_str());
  });

  RCLCPP_INFO(node->get_logger(), "dummy_node started");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}