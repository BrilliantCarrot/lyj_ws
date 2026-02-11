#include <chrono>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
// ROS2/ament에서는 “패키지의 include 디렉토리”가 include 경로로 잡히기 때문에
// 헤더는 include/를 빼고 패키지 경로 기준으로만 씀.
#include <uav_gnc/sixdof.h>


using namespace std::chrono_literals;

class SimNode : public rclcpp::Node
{
public:
  SimNode() : Node("sim_node")
  {
    // ===== parameters (yaml로 업데이트 계속 필요) =====
    dt_ = this->declare_parameter<double>("dt", 0.01);

    params_.mass = this->declare_parameter<double>("mass", 2.0);
    params_.inertia.x = this->declare_parameter<double>("Ix", 0.02);
    params_.inertia.y = this->declare_parameter<double>("Iy", 0.02);
    params_.inertia.z = this->declare_parameter<double>("Iz", 0.04);
    params_.g = this->declare_parameter<double>("g", 9.80665);

    params_.use_drag = this->declare_parameter<bool>("use_drag", true);
    params_.k1 = this->declare_parameter<double>("k1", 0.15);
    params_.k2 = this->declare_parameter<double>("k2", 0.02);

    // 초기 상태 (나중에 파라미터로 빼기 가능)
    state_.p = {0.0, 0.0, 0.0};
    state_.v = {0.0, 0.0, 0.0};
    state_.q = {1.0, 0.0, 0.0, 0.0};   // body->world
    state_.w = {0.0, 0.0, 0.0};        // body frame angular rate

    // 기본 입력 0
    input_.thrust_body = {0.0, 0.0, 0.0};
    input_.moment_body = {0.0, 0.0, 0.0};

    // ===== pub/sub =====
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/sim/odom", 10);

    wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/control/wrench", 10,
      std::bind(&SimNode::wrenchCallback, this, std::placeholders::_1)
    );

    // ===== timer =====
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt_),
      std::bind(&SimNode::onTimer, this)
    );

    RCLCPP_INFO(this->get_logger(), "sim_node started (dt=%.4f)", dt_);
  }

private:
  void wrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    // 여기서는 /control/wrench가 "body frame 기준 힘/토크"라고 가정한다.
    // (네 sixdof도 thrust_body/moment_body를 body frame으로 받도록 설계되어 있음)
    std::lock_guard<std::mutex> lock(mtx_);
    input_.thrust_body = { msg->wrench.force.x,  msg->wrench.force.y,  msg->wrench.force.z };
    input_.moment_body = { msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z };
  }

  void onTimer()
  {
    // 적분(상태 업데이트)
    Input u_copy;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      u_copy = input_;
    }

    state_ = rk4_step(state_, u_copy, params_, dt_);

    // Odometry publish
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->now();
    odom.header.frame_id = "world";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = state_.p.x;
    odom.pose.pose.position.y = state_.p.y;
    odom.pose.pose.position.z = state_.p.z;

    odom.pose.pose.orientation.w = state_.q.w;
    odom.pose.pose.orientation.x = state_.q.x;
    odom.pose.pose.orientation.y = state_.q.y;
    odom.pose.pose.orientation.z = state_.q.z;

    // 주의: state_.v는 world frame 속도, state_.w는 body frame 각속도
    // 4주차에서는 일단 그대로 넣고, 나중에 frame 정교화(또는 tf)로 다듬으면 된다.
    odom.twist.twist.linear.x = state_.v.x;
    odom.twist.twist.linear.y = state_.v.y;
    odom.twist.twist.linear.z = state_.v.z;

    odom.twist.twist.angular.x = state_.w.x;
    odom.twist.twist.angular.y = state_.w.y;
    odom.twist.twist.angular.z = state_.w.z;

    odom_pub_->publish(odom);
  }

private:
  double dt_{0.01};

  Params params_;
  State  state_;
  Input  input_;

  std::mutex mtx_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimNode>());
  rclcpp::shutdown();
  return 0;
}

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<rclcpp::Node>("dummy_node");
//   // basic pub test
//   auto pub = node->create_publisher<std_msgs::msg::String>("/dummy/chatter", 10);
//   // basic sub test
//   auto sub = node->create_subscription<std_msgs::msg::String>(
//   "/dummy/in", 10,
//   [node](const std_msgs::msg::String::SharedPtr msg) {
//     RCLCPP_INFO(node->get_logger(), "received: %s", msg->data.c_str());
//   });

//   auto timer = node->create_wall_timer(500ms, [node, pub]() {
//     std_msgs::msg::String msg;
//     msg.data = "hello from dummy_node";
//     pub->publish(msg);
//     // RCLCPP_INFO(node->get_logger(), "publish: %s", msg.data.c_str());
//   });

//   RCLCPP_INFO(node->get_logger(), "dummy_node started");
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }