#include <cmath>
#include <chrono>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

// 지금 시각 t에서 목표 위치(그리고 필요하면 목표 yaw)를 만들어서 /guidance/setpoint로 계속 publish하는 노드
class GuidanceNode : public rclcpp::Node
{
public:
  GuidanceNode() : Node("guidance_node")
  {
    // 1. 6주차를 위해 업데이트된 파라미터 선언
    use_nav_odom_ = this->declare_parameter<bool>("use_nav_odom", true);
    setpoint_topic_ = this->declare_parameter<std::string>("setpoint_topic", "/guidance/setpoint");
    nav_odom_topic_ = this->declare_parameter<std::string>("nav_odom_topic", "/nav/odom");
    sim_odom_topic_ = this->declare_parameter<std::string>("sim_odom_topic", "/sim/odom");
    // this->declare_parameter<double>("acceptance_radius", 1.0);
    // this->declare_parameter<std::vector<double>>("waypoints", std::vector<double>{});

    // 파라미터 읽기
    rate_hz_ = this->declare_parameter<double>("rate_hz", 20.0);
    z_ref_ = this->declare_parameter<double>("z_ref", 1.0);
    // R_acc_ = this->get_parameter<double>("accept_radius");
    // std::vector<double> wp_raw = this->get_parameter("waypoints_x").as_double_array();
    wp_x_ = this->declare_parameter<std::vector<double>>("waypoints_x", std::vector<double>{0.0});
    wp_y_ = this->declare_parameter<std::vector<double>>("waypoints_y", std::vector<double>{0.0});

    accept_radius_ = this->declare_parameter<double>("accept_radius", 0.5);
    lookahead_dist_ = this->declare_parameter<double>("lookahead_dist", 1.5);
    hold_last_ = this->declare_parameter<bool>("hold_last", true);

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
    RCLCPP_INFO(this->get_logger(), "GuidanceNode started. odom=%s, setpoint=%s",
                odom_topic.c_str(), setpoint_topic_.c_str());

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
  }

  void onTimer()
  {
    // odom이 아직 안 들어왔거으면 대기
    if (!have_odom_) return;
    // 현재 목표 waypoint
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
        // 마지막 waypoint
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

    geometry_msgs::msg::PoseStamped sp;
    sp.header.stamp = this->now();
    sp.header.frame_id = last_frame_id_.empty() ? "world" : last_frame_id_;
    sp.pose.position.x = spx;
    sp.pose.position.y = spy;
    sp.pose.position.z = z_ref_;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    // sp.pose.orientation = tf2::toMsg(q);
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

  // state
  bool have_odom_{false};
  double current_x_{0.0}, current_y_{0.0}, current_z_{0.0};
  size_t wp_index_{0};
  std::string last_frame_id_;

  // rclcpp::Time start_time_;
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