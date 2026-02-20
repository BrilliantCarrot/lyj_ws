#include <cmath>
#include <chrono>
#include <vector>
#include <string>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "uav_gnc/trajectory.h"

using namespace std::chrono_literals;

// 지금 시각 t에서 목표 위치(그리고 필요하면 목표 yaw)를 만들어서 /guidance/setpoint로 계속 publish하는 노드
class GuidanceNode : public rclcpp::Node
{
public:
  GuidanceNode() : Node("guidance_node")
  {
    // 공통 파라미터
    use_nav_odom_   = this->declare_parameter<bool>("use_nav_odom", true);
    setpoint_topic_ = this->declare_parameter<std::string>("setpoint_topic", "/guidance/setpoint");
    nav_odom_topic_ = this->declare_parameter<std::string>("nav_odom_topic", "/nav/odom");
    sim_odom_topic_ = this->declare_parameter<std::string>("sim_odom_topic", "/sim/odom");

    rate_hz_ = this->declare_parameter<double>("rate_hz", 20.0);
    z_ref_   = this->declare_parameter<double>("z_ref", 1.0);

    // waypoint 파라미터(기존 방식도 유지 + trajectory에서도 사용)
    wp_x_ = this->declare_parameter<std::vector<double>>("waypoints_x", std::vector<double>{0.0});
    wp_y_ = this->declare_parameter<std::vector<double>>("waypoints_y", std::vector<double>{0.0});

    accept_radius_  = this->declare_parameter<double>("accept_radius", 0.5);
    lookahead_dist_ = this->declare_parameter<double>("lookahead_dist", 1.5);
    hold_last_      = this->declare_parameter<bool>("hold_last", true);


    // ===== 7주차: trajectory 모드 파라미터 =====
    use_trajectory_    = this->declare_parameter<bool>("use_trajectory", true);
    traj_max_speed_    = this->declare_parameter<double>("traj_max_speed", 1.5);   // [m/s]
    traj_seg_dt_min_   = this->declare_parameter<double>("traj_seg_dt_min", 0.8);  // [s]
    traj_loop_         = this->declare_parameter<bool>("traj_loop", false);
    traj_lookahead_t_  = this->declare_parameter<double>("traj_lookahead_t", 0.5); // [s] yaw/방향 안정화용
    traj_start_delay_  = this->declare_parameter<double>("traj_start_delay", 0.0); // [s]

    if (wp_x_.size() != wp_y_.size() || wp_x_.empty()) {
      RCLCPP_FATAL(this->get_logger(), "Waypoints invalid: size mismatch or empty.");
      throw std::runtime_error("Invalid waypoints");
    }

    // 1차원 배열을 좌표 구조체로 변환
    // for (size_t i = 0; i + 2 < wp_raw.size(); i += 3) {
    //   waypoints_.push_back({wp_raw[i], wp_raw[i+1], wp_raw[i+2]});
    // }
    // 2. Pub/Sub 설정
    std::string odom_topic = use_nav_odom_ ? nav_odom_topic_ : sim_odom_topic_;
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      std::bind(&GuidanceNode::odomCallback, this, std::placeholders::_1));
    // odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //   odom_topic, 10, std::bind(&GuidanceNode::odomCallback, this, std::placeholders::_1));
    setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(setpoint_topic_, 10);
    // 3. 타이머 설정
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz_)); // 주기 계산
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&GuidanceNode::onTimer, this));
    // 일정 시간(period)마다 특정 함수(onTimer)를 자동으로 실행하도록 하는 타이머를 설정
    RCLCPP_INFO(this->get_logger(),
                "GuidanceNode started. odom=%s, setpoint=%s, use_trajectory=%s",
                odom_topic.c_str(), setpoint_topic_.c_str(),
                use_trajectory_ ? "true" : "false");

    // 원형 궤적 파라미터
    // cx_ = this->declare_parameter<double>("center_x", 0.0);
    // cy_ = this->declare_parameter<double>("center_y", 0.0);
    // z_  = this->declare_parameter<double>("z", 1.0);
    // r_  = this->declare_parameter<double>("radius", 3.0);
    // // 각속도(rad/s). 예: 0.2면 천천히 돈다.
    // omega_ = this->declare_parameter<double>("omega", 0.2);
    // // 시작 각도(rad)
    // theta0_ = this->declare_parameter<double>("theta0", 0.0);
    // start_time_ = this->now();
    // RCLCPP_INFO(this->get_logger(),
    //             "guidance_node started (rate=%.1f Hz, r=%.2f, omega=%.3f, z=%.2f)",
    //             rate_hz_, r_, omega_, z_);
  }

private:
  struct Point { double x, y, z; };

  static geometry_msgs::msg::Quaternion yawToQuat(double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    // roll=pitch=0 가정
    q.w = std::cos(yaw * 0.5);
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    return q;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // curr_pos_ = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
    // has_odom_ = true;
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_z_ = msg->pose.pose.position.z;
    have_odom_ = true;
    last_frame_id_ = msg->header.frame_id;

    // trajectory는 odom 들어온 다음에 한 번만 build (frame_id, 시작 시각 안정화)
    if (use_trajectory_ && !traj_built_) {
      buildTrajectoryOnce();
    }
  }

  void buildTrajectoryOnce()
  {
    // [DEBUG] 7주차: 파라미터가 제대로 들어왔는지 확인 로그
    RCLCPP_WARN(this->get_logger(),
      "[DEBUG] waypoints size: x=%zu, y=%zu, z_ref=%.2f", 
      wp_x_.size(), wp_y_.size(), z_ref_);
    std::vector<uav_gnc::Waypoint> wps;
    wps.reserve(wp_x_.size());
    // [DEBUG] 너무 큰 값(파라미터 깨짐) 방어
    if (wp_x_.size() > 10000) {
      throw std::runtime_error("waypoints_x is unexpectedly huge. Check YAML indentation/type.");
    }
    for (size_t i = 0; i < wp_x_.size(); ++i) {
      uav_gnc::Waypoint w;
      w.x = wp_x_[i];
      w.y = wp_y_[i];
      w.z = z_ref_;   // 현재는 z는 고정(원하면 7주차 후반에 wp_z 추가 가능)
      wps.push_back(w);
    }

    try {
      traj_.buildAutoTime(wps, traj_max_speed_, traj_seg_dt_min_);
      traj_total_T_ = traj_.totalTime();
      traj_start_time_ = this->now();
      traj_built_ = true;

      RCLCPP_INFO(this->get_logger(),
                  "Trajectory built: segments=%zu, total_T=%.3f sec (max_speed=%.2f, seg_dt_min=%.2f)",
                  (wps.size() >= 2 ? (wps.size()-1) : 0), traj_total_T_, traj_max_speed_, traj_seg_dt_min_);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to build trajectory: %s", e.what());
      traj_built_ = false;
    }
  }

void onTimer()
  {
    if (!have_odom_) return;

    geometry_msgs::msg::PoseStamped sp;
    sp.header.stamp = this->now();
    sp.header.frame_id = last_frame_id_.empty() ? "world" : last_frame_id_;

    if (use_trajectory_) {
      if (!traj_built_) {
        // odom이 왔는데도 build 실패했으면, 안전하게 기존 방식으로 폴백
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Trajectory not built yet. Fallback to waypoint mode.");
        publishWaypointLookahead(sp);
        return;
      }

      // 경과 시간
      double t = (this->now() - traj_start_time_).seconds() - traj_start_delay_;
      if (t < 0.0) t = 0.0;

      if (traj_total_T_ <= 1e-6) {
        publishWaypointLookahead(sp);
        return;
      }

      if (t > traj_total_T_) {
        if (traj_loop_) {
          t = std::fmod(t, traj_total_T_);
        } else {
          if (!hold_last_) {
            return; // 마지막에서 종료
          }
          t = traj_total_T_;
        }
      }

      // 현재 참조 샘플
      const auto ref = traj_.sample(t);

      // yaw는 velocity 기반이 제일 안정적. (속도가 너무 작으면 lookahead 샘플로 보완)
      double yaw = 0.0;
      const double vnorm = std::hypot(ref.vx, ref.vy);
      if (vnorm > 1e-3) {
        yaw = std::atan2(ref.vy, ref.vx);
      } else {
        const auto ref2 = traj_.sample(std::min(t + std::max(0.0, traj_lookahead_t_), traj_total_T_));
        yaw = std::atan2(ref2.y - ref.y, ref2.x - ref.x);
      }

      sp.pose.position.x = ref.x;
      sp.pose.position.y = ref.y;
      sp.pose.position.z = ref.z;
      sp.pose.orientation = yawToQuat(yaw);

      setpoint_pub_->publish(sp);
      return;
    }

    // 기존: waypoint look-ahead
    publishWaypointLookahead(sp);
  }

  void publishWaypointLookahead(geometry_msgs::msg::PoseStamped &sp)
  {
    const size_t N = wp_x_.size();
    if (wp_index_ >= N) wp_index_ = N - 1;

    double wx = wp_x_[wp_index_];
    double wy = wp_y_[wp_index_];

    // waypoint 도달 체크
    double dx = wx - current_x_;
    double dy = wy - current_y_;
    double dist = std::hypot(dx, dy);

    if (dist < accept_radius_) {
      if (wp_index_ + 1 < N) {
        wp_index_++;
        wx = wp_x_[wp_index_];
        wy = wp_y_[wp_index_];
        dx = wx - current_x_;
        dy = wy - current_y_;
        dist = std::hypot(dx, dy);
      } else {
        if (!hold_last_) return;
      }
    }

    // look-ahead setpoint 계산
    double spx, spy;
    if (dist < 1e-6) {
      spx = wx;
      spy = wy;
    } else {
      double L = std::min(lookahead_dist_, dist);
      spx = current_x_ + (dx / dist) * L;
      spy = current_y_ + (dy / dist) * L;
    }

    // yaw: 현재 위치 -> setpoint 방향
    double yaw = std::atan2(spy - current_y_, spx - current_x_);

    sp.pose.position.x = spx;
    sp.pose.position.y = spy;
    sp.pose.position.z = z_ref_;
    sp.pose.orientation = yawToQuat(yaw);

    setpoint_pub_->publish(sp);
  }

private:
  // params
  bool use_nav_odom_{true};
  std::string setpoint_topic_;
  std::string nav_odom_topic_;
  std::string sim_odom_topic_;
  double rate_hz_{20.0};
  double z_ref_{1.0};

  std::vector<double> wp_x_;
  std::vector<double> wp_y_;
  double accept_radius_{0.5};
  double lookahead_dist_{1.5};
  bool hold_last_{true};

  // 7주차 trajectory params
  bool use_trajectory_{true};
  double traj_max_speed_{1.5};
  double traj_seg_dt_min_{0.8};
  bool traj_loop_{false};
  double traj_lookahead_t_{0.5};
  double traj_start_delay_{0.0};

  // state
  bool have_odom_{false};
  double current_x_{0.0}, current_y_{0.0}, current_z_{0.0};
  size_t wp_index_{0};
  std::string last_frame_id_;

  // trajectory state
  uav_gnc::PiecewiseQuinticTrajectory traj_;
  bool traj_built_{false};
  rclcpp::Time traj_start_time_;
  double traj_total_T_{0.0};

  // rclcpp
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

  int main(int argc, char **argv)
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GuidanceNode>());
    rclcpp::shutdown();
    return 0;
  }