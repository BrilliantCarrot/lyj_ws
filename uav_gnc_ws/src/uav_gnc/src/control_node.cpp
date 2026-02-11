#include <chrono>
#include <mutex>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <uav_gnc/sixdof.h>
#include <uav_gnc/controller.h>

using namespace std::chrono_literals;

class ControlNode : public rclcpp::Node
{
public:
  ControlNode() : Node("control_node")
  {
    dt_ = this->declare_parameter<double>("dt", 0.01);

    // simulation_node와 params/gains를 맞추는 게 중요함
    params_.mass = this->declare_parameter<double>("mass", 2.0);
    params_.inertia.x = this->declare_parameter<double>("Ix", 0.02);
    params_.inertia.y = this->declare_parameter<double>("Iy", 0.02);
    params_.inertia.z = this->declare_parameter<double>("Iz", 0.04);
    params_.g = this->declare_parameter<double>("g", 9.80665);
    params_.use_drag = this->declare_parameter<bool>("use_drag", true);
    params_.k1 = this->declare_parameter<double>("k1", 0.15);
    params_.k2 = this->declare_parameter<double>("k2", 0.02);

    // gains 값 확인해서 yaml 파라미터로 빼기 (일단 default 그대로)
    gains_.kp_pos_xy = this->declare_parameter<double>("kp_pos_xy", gains_.kp_pos_xy);
    gains_.kp_pos_z  = this->declare_parameter<double>("kp_pos_z",  gains_.kp_pos_z);
    gains_.kp_vel_xy = this->declare_parameter<double>("kp_vel_xy", gains_.kp_vel_xy);
    gains_.kp_vel_z  = this->declare_parameter<double>("kp_vel_z",  gains_.kp_vel_z);

    gains_.kp_att_rp  = this->declare_parameter<double>("kp_att_rp",  gains_.kp_att_rp);
    gains_.kd_att_rp  = this->declare_parameter<double>("kd_att_rp",  gains_.kd_att_rp);
    gains_.kp_att_yaw = this->declare_parameter<double>("kp_att_yaw", gains_.kp_att_yaw);
    gains_.kd_att_yaw = this->declare_parameter<double>("kd_att_yaw", gains_.kd_att_yaw);

    gains_.max_tilt_deg = this->declare_parameter<double>("max_tilt_deg", gains_.max_tilt_deg);

    gains_.max_vxy_cmd  = this->declare_parameter<double>("max_vxy_cmd",  gains_.max_vxy_cmd);
    gains_.max_axy_cmd  = this->declare_parameter<double>("max_axy_cmd",  gains_.max_axy_cmd);
    gains_.max_vz_cmd   = this->declare_parameter<double>("max_vz_cmd",   gains_.max_vz_cmd);
    gains_.max_az_cmd   = this->declare_parameter<double>("max_az_cmd",   gains_.max_az_cmd);

    gains_.thrust_min   = this->declare_parameter<double>("thrust_min",   gains_.thrust_min);
    gains_.thrust_max   = this->declare_parameter<double>("thrust_max",   gains_.thrust_max);
    gains_.moment_max_rp = this->declare_parameter<double>("moment_max_rp", gains_.moment_max_rp);
    gains_.moment_max_y  = this->declare_parameter<double>("moment_max_y",  gains_.moment_max_y);

    wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/control/wrench", 10);

    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/nav/odom"); // nav/odom 기본
    // nav_node가 없을 때엔 터미널에서 ros2 run uav_gnc control_node --ros-args -p odom_topic:=/sim/odom 입력
    // 즉, 실행할 때 파라미터가 필요
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

    sp_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/guidance/setpoint", 10,
      std::bind(&ControlNode::setpointCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt_),
      std::bind(&ControlNode::onTimer, this));

    RCLCPP_INFO(this->get_logger(), "control_node started (dt=%.4f)", dt_);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);

    state_.p = { msg->pose.pose.position.x,
                 msg->pose.pose.position.y,
                 msg->pose.pose.position.z };

    state_.v = { msg->twist.twist.linear.x,
                 msg->twist.twist.linear.y,
                 msg->twist.twist.linear.z };

    state_.q = { msg->pose.pose.orientation.w,
                 msg->pose.pose.orientation.x,
                 msg->pose.pose.orientation.y,
                 msg->pose.pose.orientation.z };

    state_.w = { msg->twist.twist.angular.x,
                 msg->twist.twist.angular.y,
                 msg->twist.twist.angular.z };

    has_state_ = true;
  }

  void setpointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);

    ref_.p_ref = { msg->pose.position.x,
                   msg->pose.position.y,
                   msg->pose.position.z };

    // quaternion -> yaw
    const auto &q = msg->pose.orientation;
    ref_.yaw_ref = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    has_ref_ = true;
  }

  void onTimer()
  {
    State s;
    Ref ref;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (!has_state_ || !has_ref_) return;
      s = state_;
      ref = ref_;
    }

    // 네 controller는 함수형
    const Input u = controller_update(s, ref, params_, gains_);

    geometry_msgs::msg::WrenchStamped wrench;
    wrench.header.stamp = this->now();
    wrench.header.frame_id = "base_link"; // sim_node는 body frame 입력이라고 가정 중

    wrench.wrench.force.x  = u.thrust_body.x;
    wrench.wrench.force.y  = u.thrust_body.y;
    wrench.wrench.force.z  = u.thrust_body.z;

    wrench.wrench.torque.x = u.moment_body.x;
    wrench.wrench.torque.y = u.moment_body.y;
    wrench.wrench.torque.z = u.moment_body.z;

    wrench_pub_->publish(wrench);
  }

private:
  double dt_{0.01};

  Params params_;
  ControllerGains gains_;

  State state_;
  Ref ref_;
  bool has_state_{false};
  bool has_ref_{false};

  std::mutex mtx_;
  std::string odom_topic_; // odom 구독 토픽 이름

  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sp_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}