#include <cmath>
#include <chrono>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "uav_gnc/trajectory.h"

using namespace std::chrono_literals;

class GuidanceNode : public rclcpp::Node
{
public:
  GuidanceNode() : Node("guidance_node")
  {
    use_nav_odom_ = this->declare_parameter<bool>("use_nav_odom", true);
    setpoint_topic_ = this->declare_parameter<std::string>("setpoint_topic", "/guidance/setpoint");
    nav_odom_topic_ = this->declare_parameter<std::string>("nav_odom_topic", "/nav/odom");
    sim_odom_topic_ = this->declare_parameter<std::string>("sim_odom_topic", "/sim/odom");
    rate_hz_ = this->declare_parameter<double>("rate_hz", 20.0);
    z_ref_ = this->declare_parameter<double>("z_ref", 1.0);
    
    // 모드 4가지: "lookahead", "min_jerk", "min_snap", "multi_snap"
    guidance_mode_ = this->declare_parameter<std::string>("guidance_mode", "lookahead");

    wp_x_ = this->declare_parameter<std::vector<double>>("waypoints_x", std::vector<double>{0.0});
    wp_y_ = this->declare_parameter<std::vector<double>>("waypoints_y", std::vector<double>{0.0});
    
    accept_radius_ = this->declare_parameter<double>("accept_radius", 0.5);
    lookahead_dist_ = this->declare_parameter<double>("lookahead_dist", 1.5);
    avg_speed_ = this->declare_parameter<double>("avg_speed", 1.0); 
    hold_last_ = this->declare_parameter<bool>("hold_last", true);

    std::string odom_topic = use_nav_odom_ ? nav_odom_topic_ : sim_odom_topic_;
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10, std::bind(&GuidanceNode::odomCallback, this, std::placeholders::_1));
    setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(setpoint_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&GuidanceNode::onTimer, this));

    RCLCPP_INFO(this->get_logger(), "Guidance Mode: [%s]", guidance_mode_.c_str());
  }

private:
  static geometry_msgs::msg::Quaternion yawToQuat(double yaw) {
    geometry_msgs::msg::Quaternion q;
    q.w = std::cos(yaw * 0.5); q.x = 0.0; q.y = 0.0; q.z = std::sin(yaw * 0.5);
    return q;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_z_ = msg->pose.pose.position.z;
    last_frame_id_ = msg->header.frame_id;
    
    if (!have_odom_) {
      have_odom_ = true;
      if (guidance_mode_ == "min_jerk" || guidance_mode_ == "min_snap") {
        generateTrajectoryForCurrentSegment();
      } else if (guidance_mode_ == "multi_snap") {
        generateMultiSegmentTrajectory(); // 다중 구간 궤적 단 한 번 생성!
      }
    }
  }

  // [기존] 단일 구간 궤적 생성
  void generateTrajectoryForCurrentSegment() {
    if (wp_index_ >= wp_x_.size() - 1) return;
    double p0_x = wp_index_ == 0 ? current_x_ : wp_x_[wp_index_];
    double p0_y = wp_index_ == 0 ? current_y_ : wp_y_[wp_index_];
    double pf_x = wp_x_[wp_index_ + 1], pf_y = wp_y_[wp_index_ + 1];

    double dist = std::hypot(pf_x - p0_x, pf_y - p0_y);
    current_segment_T_ = std::max(0.1, dist / avg_speed_);

    if (guidance_mode_ == "min_jerk") {
      jerk_x_.generate(p0_x, 0.0, 0.0, pf_x, 0.0, 0.0, current_segment_T_);
      jerk_y_.generate(p0_y, 0.0, 0.0, pf_y, 0.0, 0.0, current_segment_T_);
    } else {
      snap_x_.generate(p0_x, 0.0, 0.0, 0.0, pf_x, 0.0, 0.0, 0.0, current_segment_T_);
      snap_y_.generate(p0_y, 0.0, 0.0, 0.0, pf_y, 0.0, 0.0, 0.0, current_segment_T_);
    }
    segment_start_time_ = this->now();
    is_trajectory_active_ = true;
  }

  // [신규] 전체 다중 구간 궤적 한 번에 생성!
  void generateMultiSegmentTrajectory() {
    std::vector<double> full_wp_x, full_wp_y, times;
    
    // 현재 위치를 출발점으로 설정
    full_wp_x.push_back(current_x_);
    full_wp_y.push_back(current_y_);

    // 중복되거나 너무 가까운 Waypoint 필터링하며 시간 계산
    for (size_t i = 0; i < wp_x_.size(); ++i) {
        double dx = wp_x_[i] - full_wp_x.back();
        double dy = wp_y_[i] - full_wp_y.back();
        double dist = std::hypot(dx, dy);
        
        if (dist < 0.1) continue; // 너무 가까운 점 무시

        full_wp_x.push_back(wp_x_[i]);
        full_wp_y.push_back(wp_y_[i]);
        times.push_back(dist / avg_speed_);
    }

    if (times.empty()) return; // 갈 곳이 없음

    multi_snap_x_.generate(full_wp_x, times);
    multi_snap_y_.generate(full_wp_y, times);

    segment_start_time_ = this->now();
    is_trajectory_active_ = true;
    total_multi_T_ = multi_snap_x_.getTotalTime();
    
    RCLCPP_INFO(this->get_logger(), "Generated MULTI-SEGMENT Trajectory! Total Time: %.2f sec", total_multi_T_);
  }

  void onTimer() {
    if (!have_odom_) return;

    geometry_msgs::msg::PoseStamped sp;
    sp.header.stamp = this->now();
    sp.header.frame_id = last_frame_id_.empty() ? "world" : last_frame_id_;
    sp.pose.position.z = z_ref_;

    // ==========================================
    // 1. Lookahead 모드
    // ==========================================
    if (guidance_mode_ == "lookahead") {
      const size_t N = wp_x_.size();
      if (wp_index_ >= N) wp_index_ = N - 1;
      double dx = wp_x_[wp_index_] - current_x_, dy = wp_y_[wp_index_] - current_y_;
      double dist = std::hypot(dx, dy);

      if (dist < accept_radius_) {
        if (wp_index_ + 1 < N) wp_index_++;
        else if (!hold_last_) return;
      }
      double spx = (dist < 1e-6) ? wp_x_[wp_index_] : current_x_ + (dx / dist) * std::min(lookahead_dist_, dist);
      double spy = (dist < 1e-6) ? wp_y_[wp_index_] : current_y_ + (dy / dist) * std::min(lookahead_dist_, dist);

      sp.pose.position.x = spx; sp.pose.position.y = spy;
      sp.pose.orientation = yawToQuat(std::atan2(spy - current_y_, spx - current_x_));
      setpoint_pub_->publish(sp);
    } 
    // ==========================================
    // 2. Single-segment 모드
    // ==========================================
    else if (guidance_mode_ == "min_jerk" || guidance_mode_ == "min_snap") {
      if (is_trajectory_active_) {
        double t = (this->now() - segment_start_time_).seconds();
        double spx, spy, vx, vy;

        if (guidance_mode_ == "min_jerk") {
          spx = jerk_x_.getPosition(t); spy = jerk_y_.getPosition(t);
          vx = jerk_x_.getVelocity(t); vy = jerk_y_.getVelocity(t);
        } else {
          spx = snap_x_.getPosition(t); spy = snap_y_.getPosition(t);
          vx = snap_x_.getVelocity(t); vy = snap_y_.getVelocity(t);
        }
        sp.pose.position.x = spx; sp.pose.position.y = spy;
        current_yaw_ = (std::hypot(vx, vy) > 0.05) ? std::atan2(vy, vx) : current_yaw_;
        sp.pose.orientation = yawToQuat(current_yaw_);

        if (t >= current_segment_T_) {
          wp_index_++;
          if (wp_index_ < wp_x_.size() - 1) generateTrajectoryForCurrentSegment();
          else is_trajectory_active_ = false; 
        }
      } else {
        if (!hold_last_) return;
        sp.pose.position.x = wp_x_.back(); sp.pose.position.y = wp_y_.back();
        sp.pose.orientation = yawToQuat(current_yaw_);
      }
      setpoint_pub_->publish(sp);
    }
    // ==========================================
    // 3. Multi-segment 모드 (NEW!)
    // ==========================================
    else if (guidance_mode_ == "multi_snap") {
      if (is_trajectory_active_) {
        double t = (this->now() - segment_start_time_).seconds();
        
        sp.pose.position.x = multi_snap_x_.getPosition(t);
        sp.pose.position.y = multi_snap_y_.getPosition(t);

        double vx = multi_snap_x_.getVelocity(t);
        double vy = multi_snap_y_.getVelocity(t);
        current_yaw_ = (std::hypot(vx, vy) > 0.05) ? std::atan2(vy, vx) : current_yaw_;
        sp.pose.orientation = yawToQuat(current_yaw_);

        if (t >= total_multi_T_) is_trajectory_active_ = false;
      } else {
        if (!hold_last_) return;
        sp.pose.position.x = wp_x_.back(); sp.pose.position.y = wp_y_.back();
        sp.pose.orientation = yawToQuat(current_yaw_);
      }
      setpoint_pub_->publish(sp);
    }
  }

private:
  std::string guidance_mode_{"multi_snap"};
  bool use_nav_odom_{true}, hold_last_{true}, have_odom_{false};
  std::string setpoint_topic_, nav_odom_topic_, sim_odom_topic_, last_frame_id_;
  double rate_hz_{20.0}, z_ref_{1.0}, accept_radius_{0.5}, lookahead_dist_{1.5}, avg_speed_{1.0};
  double current_x_{0.0}, current_y_{0.0}, current_z_{0.0}, current_yaw_{0.0};
  size_t wp_index_{0};
  std::vector<double> wp_x_, wp_y_;

  MinJerkTrajectory jerk_x_, jerk_y_;
  MinSnapTrajectory snap_x_, snap_y_;
  MultiMinSnapTrajectory multi_snap_x_, multi_snap_y_; // 다중 구간 클래스
  
  rclcpp::Time segment_start_time_;
  double current_segment_T_{0.0}, total_multi_T_{0.0};
  bool is_trajectory_active_{false};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GuidanceNode>());
  rclcpp::shutdown();
  return 0;
}