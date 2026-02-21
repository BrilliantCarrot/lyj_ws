#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <deque>

class PathVizNode : public rclcpp::Node
{
public:
  PathVizNode() : Node("path_viz_node")
  {
    input_odom_topic_  = this->declare_parameter<std::string>("input_odom_topic", "/nav/odom");
    output_path_topic_ = this->declare_parameter<std::string>("output_path_topic", "/nav/path");
    output_marker_topic_ = this->declare_parameter<std::string>("output_marker_topic", "/nav/marker");

    frame_id_ = this->declare_parameter<std::string>("frame_id", "world");
    history_size_ = this->declare_parameter<int>("history_size", 2000);
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 10.0);
    // use_smoothing_   = this->declare_parameter<bool>("use_smoothing", true);
    // smooth_window_   = this->declare_parameter<int>("smooth_window", 5);      // 이동평균 창 크기
    // min_point_dist_  = this->declare_parameter<double>("min_point_dist", 0.10); // [m] 이보다 가까우면 path에 점 추가 안 함

    path_.header.frame_id = frame_id_;

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(output_path_topic_, 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(output_marker_topic_, 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      input_odom_topic_, 50,
      std::bind(&PathVizNode::odomCallback, this, std::placeholders::_1));

    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PathVizNode::onTimer, this));

    RCLCPP_INFO(this->get_logger(),
      "PathVizNode: in=%s, path=%s, marker=%s",
      input_odom_topic_.c_str(), output_path_topic_.c_str(), output_marker_topic_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // frame_id는 실제 odom header 기준으로 자동 갱신해주는게 실사용에 편함
    if (!msg->header.frame_id.empty()) {
      frame_id_ = msg->header.frame_id;
      path_.header.frame_id = frame_id_;
    }

    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = msg->header.stamp;
    ps.header.frame_id = frame_id_;
    ps.pose = msg->pose.pose;

    // history 제한
    path_.poses.push_back(ps);
    // // 1) 너무 촘촘히 찍히면 지그재그가 더 심해 보이므로, 최소 거리 이하이면 skip
    // if (!path_.poses.empty()) {
    //   const auto &last = path_.poses.back().pose.position;
    //   const double dx = ps.pose.position.x - last.x;
    //   const double dy = ps.pose.position.y - last.y;
    //   const double dz = ps.pose.position.z - last.z;
    //   const double d = std::sqrt(dx*dx + dy*dy + dz*dz);
    //   if (min_point_dist_ > 0.0 && d < min_point_dist_) {
    //     last_pose_ = ps;
    //     have_pose_ = true;
    //     return;
    //   }
    // }

    // if (use_smoothing_) {
    //   // 2) 최근 N개 위치를 이동평균 내서 path에 넣기 (orientation은 최신값 유지)
    //   pos_buf_x_.push_back(ps.pose.position.x);
    //   pos_buf_y_.push_back(ps.pose.position.y);
    //   pos_buf_z_.push_back(ps.pose.position.z);

    //   const int W = std::max(1, smooth_window_);
    //   while (static_cast<int>(pos_buf_x_.size()) > W) {
    //     pos_buf_x_.pop_front();
    //     pos_buf_y_.pop_front();
    //     pos_buf_z_.pop_front();
    //   }

    //   double sx = 0.0, sy = 0.0, sz = 0.0;
    //   for (size_t i = 0; i < pos_buf_x_.size(); ++i) {
    //     sx += pos_buf_x_[i];
    //     sy += pos_buf_y_[i];
    //     sz += pos_buf_z_[i];
    //   }
    //   const double inv = 1.0 / std::max<size_t>(1, pos_buf_x_.size());
    //   ps.pose.position.x = sx * inv;
    //   ps.pose.position.y = sy * inv;
    //   ps.pose.position.z = sz * inv;
    // }

    // // history 제한
    // path_.poses.push_back(ps);
    // if (history_size_ > 0 && static_cast<int>(path_.poses.size()) > history_size_) {
    //   path_.poses.erase(path_.poses.begin(), path_.poses.begin() + (path_.poses.size() - history_size_));
    // }
    if (history_size_ > 0 && static_cast<int>(path_.poses.size()) > history_size_) {
      path_.poses.erase(path_.poses.begin(), path_.poses.begin() + (path_.poses.size() - history_size_));
    }

    last_pose_ = ps;
    have_pose_ = true;
  }

  void onTimer()
  {
    if (!have_pose_) return;

    // Path publish
    path_.header.stamp = this->now();
    path_pub_->publish(path_);

    // Current position as a point (SPHERE marker)
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id_;
    m.header.stamp = this->now();
    m.ns = this->get_name();
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.pose = last_pose_.pose;

    // 점 크기 (너무 크면 지저분해져서 작게)
    m.scale.x = 0.15;
    m.scale.y = 0.15;
    m.scale.z = 0.15;

    // 색은 RViz에서 구분이 편하게 (sim=빨강, nav=초록 같은 걸로 나중에 조절 가능)
    // 여기서는 기본 흰색으로 두고, RViz Marker에서 색상 override도 가능해.
    m.color.a = 1.0;
    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.b = 1.0;

    marker_pub_->publish(m);
  }

private:
  std::string input_odom_topic_;
  std::string output_path_topic_;
  std::string output_marker_topic_;
  std::string frame_id_;

  int history_size_{2000};
  double publish_rate_hz_{10.0};

  // bool use_smoothing_{true};
  // int smooth_window_{5};
  // double min_point_dist_{0.10};

  // std::deque<double> pos_buf_x_;
  // std::deque<double> pos_buf_y_;
  // std::deque<double> pos_buf_z_;

  nav_msgs::msg::Path path_;
  geometry_msgs::msg::PoseStamped last_pose_;
  bool have_pose_{false};

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathVizNode>());
  rclcpp::shutdown();
  return 0;
}
