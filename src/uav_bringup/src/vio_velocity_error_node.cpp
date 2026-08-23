#include <cmath>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class VioVelocityErrorNode : public rclcpp::Node
{
public:
  VioVelocityErrorNode() : Node("vio_velocity_error_node")
  {
    reference_topic_ = declare_parameter<std::string>("reference_topic", "/nav/odom");
    vio_topic_ = declare_parameter<std::string>("vio_topic", "/vio_aligned/odom");
    max_pair_age_s_ = declare_parameter<double>("max_pair_age_s", 0.05);
    use_header_stamp_pairing_ = declare_parameter<bool>("use_header_stamp_pairing", false);
    min_samples_ = declare_parameter<int>("min_samples", 20);

    reference_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      reference_topic_, rclcpp::SensorDataQoS(),
      std::bind(&VioVelocityErrorNode::referenceCallback, this, std::placeholders::_1));
    vio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      vio_topic_, rclcpp::SensorDataQoS(),
      std::bind(&VioVelocityErrorNode::vioCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "[VIO Velocity Error] reference=%s vio=%s header_pairing=%s max_pair_age=%.3fs",
      reference_topic_.c_str(), vio_topic_.c_str(),
      use_header_stamp_pairing_ ? "true" : "false", max_pair_age_s_);
  }

private:
  void referenceCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_reference_ = *msg;
    has_reference_ = true;
  }

  void vioCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!has_reference_) {
      return;
    }

    if (use_header_stamp_pairing_) {
      const rclcpp::Time t_ref(latest_reference_.header.stamp);
      const rclcpp::Time t_vio(msg->header.stamp);
      if (std::abs((t_ref - t_vio).seconds()) > max_pair_age_s_) {
        return;
      }
    }

    const double ex = msg->twist.twist.linear.x - latest_reference_.twist.twist.linear.x;
    const double ey = msg->twist.twist.linear.y - latest_reference_.twist.twist.linear.y;
    const double ez = msg->twist.twist.linear.z - latest_reference_.twist.twist.linear.z;
    const double err = std::sqrt(ex * ex + ey * ey + ez * ez);

    sum_sq_x_ += ex * ex;
    sum_sq_y_ += ey * ey;
    sum_sq_z_ += ez * ez;
    sum_sq_ += err * err;
    ++samples_;

    if (samples_ < static_cast<uint64_t>(min_samples_)) {
      return;
    }

    const rclcpp::Time now = get_clock()->now();
    if ((now - last_log_time_).seconds() < 1.0) {
      return;
    }
    last_log_time_ = now;

    const double n = static_cast<double>(samples_);
    RCLCPP_INFO(
      get_logger(),
      "[VIO Velocity Error] samples=%lu err=%.3fm/s rmse=%.3fm/s axis_rmse=(%.3f, %.3f, %.3f) "
      "ref_v=(%.2f, %.2f, %.2f) vio_v=(%.2f, %.2f, %.2f)",
      samples_, err, std::sqrt(sum_sq_ / n),
      std::sqrt(sum_sq_x_ / n), std::sqrt(sum_sq_y_ / n), std::sqrt(sum_sq_z_ / n),
      latest_reference_.twist.twist.linear.x,
      latest_reference_.twist.twist.linear.y,
      latest_reference_.twist.twist.linear.z,
      msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.linear.z);
  }

  std::string reference_topic_;
  std::string vio_topic_;
  double max_pair_age_s_{0.05};
  bool use_header_stamp_pairing_{false};
  int min_samples_{20};

  bool has_reference_{false};
  uint64_t samples_{0};
  double sum_sq_x_{0.0};
  double sum_sq_y_{0.0};
  double sum_sq_z_{0.0};
  double sum_sq_{0.0};
  rclcpp::Time last_log_time_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Odometry latest_reference_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr reference_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vio_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VioVelocityErrorNode>());
  rclcpp::shutdown();
  return 0;
}
