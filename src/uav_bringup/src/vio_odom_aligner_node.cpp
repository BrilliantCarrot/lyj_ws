#include <cmath>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"

class VioOdomAlignerNode : public rclcpp::Node
{
public:
  VioOdomAlignerNode() : Node("vio_odom_aligner_node")
  {
    reference_topic_ = declare_parameter<std::string>("reference_topic", "/nav/odom");
    vio_topic_ = declare_parameter<std::string>("vio_topic", "/vio/odom");
    output_topic_ = declare_parameter<std::string>("output_topic", "/vio_aligned/odom");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "world");
    output_child_frame_id_ = declare_parameter<std::string>("output_child_frame_id", "imu");
    calibration_samples_ = declare_parameter<int>("calibration_samples", 200);
    min_motion_m_ = declare_parameter<double>("min_motion_m", 0.25);
    align_z_offset_ = declare_parameter<bool>("align_z_offset", true);
    rotate_orientation_yaw_ = declare_parameter<bool>("rotate_orientation_yaw", true);
    use_fixed_alignment_ = declare_parameter<bool>("use_fixed_alignment", false);
    fixed_yaw_deg_ = declare_parameter<double>("fixed_yaw_deg", 0.0);
    fixed_offset_x_ = declare_parameter<double>("fixed_offset_x", 0.0);
    fixed_offset_y_ = declare_parameter<double>("fixed_offset_y", 0.0);
    fixed_offset_z_ = declare_parameter<double>("fixed_offset_z", 0.0);

    if (use_fixed_alignment_) {
      yaw_align_rad_ = fixed_yaw_deg_ * M_PI / 180.0;
      offset_x_ = fixed_offset_x_;
      offset_y_ = fixed_offset_y_;
      offset_z_ = fixed_offset_z_;
      calibrated_ = true;
    }

    reference_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      reference_topic_, rclcpp::SensorDataQoS(),
      std::bind(&VioOdomAlignerNode::referenceCallback, this, std::placeholders::_1));
    vio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      vio_topic_, rclcpp::SensorDataQoS(),
      std::bind(&VioOdomAlignerNode::vioCallback, this, std::placeholders::_1));
    aligned_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, 10);

    RCLCPP_INFO(
      get_logger(),
      "[VIO Odom Aligner] reference=%s vio=%s output=%s calibration_samples=%d min_motion=%.2fm fixed=%s yaw=%.2fdeg offset=(%.3f, %.3f, %.3f)",
      reference_topic_.c_str(), vio_topic_.c_str(), output_topic_.c_str(),
      calibration_samples_, min_motion_m_,
      use_fixed_alignment_ ? "true" : "false",
      yaw_align_rad_ * 180.0 / M_PI, offset_x_, offset_y_, offset_z_);
  }

private:
  struct Vec3
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };

  static double norm2d(const Vec3 & v)
  {
    return std::sqrt(v.x * v.x + v.y * v.y);
  }

  static Vec3 positionOf(const nav_msgs::msg::Odometry & msg)
  {
    return {
      msg.pose.pose.position.x,
      msg.pose.pose.position.y,
      msg.pose.pose.position.z
    };
  }

  static Vec3 subtract(const Vec3 & a, const Vec3 & b)
  {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
  }

  Vec3 rotateYaw(const Vec3 & p) const
  {
    const double c = std::cos(yaw_align_rad_);
    const double s = std::sin(yaw_align_rad_);
    return {
      c * p.x - s * p.y,
      s * p.x + c * p.y,
      p.z
    };
  }

  void referenceCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_reference_ = *msg;
    has_reference_ = true;
  }

  void vioCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (use_fixed_alignment_) {
      publishAligned(*msg);
      return;
    }

    if (!has_reference_) {
      return;
    }

    const Vec3 ref_p = positionOf(latest_reference_);
    const Vec3 vio_p = positionOf(*msg);

    if (!has_initial_pair_) {
      ref0_ = ref_p;
      vio0_ = vio_p;
      has_initial_pair_ = true;
      RCLCPP_INFO(
        get_logger(),
        "[VIO Odom Aligner] initial reference=(%.3f, %.3f, %.3f) vio=(%.3f, %.3f, %.3f)",
        ref0_.x, ref0_.y, ref0_.z, vio0_.x, vio0_.y, vio0_.z);
      return;
    }

    if (!calibrated_) {
      accumulateCalibration(ref_p, vio_p);
      return;
    }

    publishAligned(*msg);
  }

  void accumulateCalibration(const Vec3 & ref_p, const Vec3 & vio_p)
  {
    const Vec3 dr = subtract(ref_p, ref0_);
    const Vec3 dv = subtract(vio_p, vio0_);

    if (norm2d(dr) < min_motion_m_ || norm2d(dv) < min_motion_m_) {
      return;
    }

    dot_sum_ += dv.x * dr.x + dv.y * dr.y;
    cross_sum_ += dv.x * dr.y - dv.y * dr.x;
    ++calibration_count_;

    if (calibration_count_ < calibration_samples_) {
      return;
    }

    yaw_align_rad_ = std::atan2(cross_sum_, dot_sum_);
    const Vec3 rotated_vio0 = rotateYaw(vio0_);
    offset_x_ = ref0_.x - rotated_vio0.x;
    offset_y_ = ref0_.y - rotated_vio0.y;
    offset_z_ = align_z_offset_ ? ref0_.z - rotated_vio0.z : 0.0;
    calibrated_ = true;

    RCLCPP_INFO(
      get_logger(),
      "[VIO Odom Aligner] calibrated yaw=%.2fdeg offset=(%.3f, %.3f, %.3f) samples=%lu",
      yaw_align_rad_ * 180.0 / M_PI, offset_x_, offset_y_, offset_z_,
      calibration_count_);
  }

  void publishAligned(const nav_msgs::msg::Odometry & vio_msg)
  {
    nav_msgs::msg::Odometry out = vio_msg;
    out.header.frame_id = output_frame_id_;
    out.child_frame_id = output_child_frame_id_;

    const Vec3 vio_p = positionOf(vio_msg);
    const Vec3 rotated_p = rotateYaw(vio_p);
    out.pose.pose.position.x = rotated_p.x + offset_x_;
    out.pose.pose.position.y = rotated_p.y + offset_y_;
    out.pose.pose.position.z = rotated_p.z + offset_z_;

    const Vec3 v{
      vio_msg.twist.twist.linear.x,
      vio_msg.twist.twist.linear.y,
      vio_msg.twist.twist.linear.z
    };
    const Vec3 rotated_v = rotateYaw(v);
    out.twist.twist.linear.x = rotated_v.x;
    out.twist.twist.linear.y = rotated_v.y;
    out.twist.twist.linear.z = rotated_v.z;

    if (rotate_orientation_yaw_) {
      tf2::Quaternion q_in(
        vio_msg.pose.pose.orientation.x,
        vio_msg.pose.pose.orientation.y,
        vio_msg.pose.pose.orientation.z,
        vio_msg.pose.pose.orientation.w);
      tf2::Quaternion q_yaw;
      q_yaw.setRPY(0.0, 0.0, yaw_align_rad_);
      tf2::Quaternion q_out = q_yaw * q_in;
      q_out.normalize();
      out.pose.pose.orientation.x = q_out.x();
      out.pose.pose.orientation.y = q_out.y();
      out.pose.pose.orientation.z = q_out.z();
      out.pose.pose.orientation.w = q_out.w();
    }

    aligned_pub_->publish(out);
  }

  std::string reference_topic_;
  std::string vio_topic_;
  std::string output_topic_;
  std::string output_frame_id_;
  std::string output_child_frame_id_;
  int calibration_samples_{200};
  double min_motion_m_{0.25};
  bool align_z_offset_{true};
  bool rotate_orientation_yaw_{true};
  bool use_fixed_alignment_{false};
  double fixed_yaw_deg_{0.0};
  double fixed_offset_x_{0.0};
  double fixed_offset_y_{0.0};
  double fixed_offset_z_{0.0};

  bool has_reference_{false};
  bool has_initial_pair_{false};
  bool calibrated_{false};
  uint64_t calibration_count_{0};
  double dot_sum_{0.0};
  double cross_sum_{0.0};
  double yaw_align_rad_{0.0};
  double offset_x_{0.0};
  double offset_y_{0.0};
  double offset_z_{0.0};
  Vec3 ref0_;
  Vec3 vio0_;

  nav_msgs::msg::Odometry latest_reference_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr reference_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vio_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr aligned_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VioOdomAlignerNode>());
  rclcpp::shutdown();
  return 0;
}
