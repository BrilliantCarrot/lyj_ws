#include <cmath>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace std::chrono_literals;

class GuidanceNode : public rclcpp::Node
{
public:
  GuidanceNode() : Node("guidance_node")
  {
    // publish 주기
    rate_hz_ = this->declare_parameter<double>("rate_hz", 20.0);

    // 원형 궤적 파라미터
    cx_ = this->declare_parameter<double>("center_x", 0.0);
    cy_ = this->declare_parameter<double>("center_y", 0.0);
    z_  = this->declare_parameter<double>("z", 1.0);
    r_  = this->declare_parameter<double>("radius", 3.0);

    // 각속도(rad/s). 예: 0.2면 천천히 돈다.
    omega_ = this->declare_parameter<double>("omega", 0.2);

    // 시작 각도(rad)
    theta0_ = this->declare_parameter<double>("theta0", 0.0);

    pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/guidance/setpoint", 10);

    start_time_ = this->now();

    const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    timer_ = this->create_wall_timer(period, std::bind(&GuidanceNode::onTimer, this));

    RCLCPP_INFO(this->get_logger(),
                "guidance_node started (rate=%.1f Hz, r=%.2f, omega=%.3f, z=%.2f)",
                rate_hz_, r_, omega_, z_);
  }

private:
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

  void onTimer()
  {
    const double t = (this->now() - start_time_).seconds();
    const double theta = theta0_ + omega_ * t;

    const double x = cx_ + r_ * std::cos(theta);
    const double y = cy_ + r_ * std::sin(theta);

    // yaw를 궤적의 진행 방향을 바라보게 하고 싶으면, 접선 방향으로 잡으면 됨
    // 원 궤적의 접선 방향 yaw = theta + pi/2
    const double yaw = theta + M_PI_2;

    geometry_msgs::msg::PoseStamped sp;
    sp.header.stamp = this->now();
    sp.header.frame_id = "world";   // sim_node의 odom frame_id랑 통일

    sp.pose.position.x = x;
    sp.pose.position.y = y;
    sp.pose.position.z = z_;

    sp.pose.orientation = yawToQuat(yaw);

    pub_->publish(sp);
  }

private:
  double rate_hz_{20.0};
  double cx_{0.0}, cy_{0.0}, z_{1.0}, r_{3.0}, omega_{0.2}, theta0_{0.0};

  rclcpp::Time start_time_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

  int main(int argc, char **argv)
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GuidanceNode>());
    rclcpp::shutdown();
    return 0;
  }