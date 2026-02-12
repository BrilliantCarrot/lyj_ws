#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

// /sim/odom(ground truth)을 구독
// 그대로(혹은 약간의 노이즈를 섞어서) /nav/odom으로 다시 publish
// 그리고 control_node는 이제 /sim/odom 대신 /nav/odom만 보게 바꿈
class NavigationNode : public rclcpp::Node
{
public:
  NavigationNode() : Node("navigation_node")
  {
    // 입력/출력 토픽 파라미터로도 바꿀 수 있게(실무감)
    in_topic_  = this->declare_parameter<std::string>("in_topic",  "/sim/odom");
    out_topic_ = this->declare_parameter<std::string>("out_topic", "/nav/odom");

    pub_ = this->create_publisher<nav_msgs::msg::Odometry>(out_topic_, 10);

    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      in_topic_, 10,
      std::bind(&NavigationNode::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "navigation_node started (in: %s -> out: %s)",
                in_topic_.c_str(), out_topic_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // 더미 EKF: 그냥 통과
    nav_msgs::msg::Odometry out = *msg;

    // frame_id는 그대로 유지해도 되고, 표준적으로는 nav가 쓰는 frame으로 통일해도 됨
    // out.header.frame_id = "world";
    // out.child_frame_id = "base_link";

    pub_->publish(out);
  }

private:
  std::string in_topic_;
  std::string out_topic_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationNode>());
  rclcpp::shutdown();
  return 0;
}