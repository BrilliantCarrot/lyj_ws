#include <mutex>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "uav_gnc/ekf.h" // 우리가 만든 EKF 헤더

using namespace std::chrono_literals;

class NavigationNode : public rclcpp::Node
{
public:
  NavigationNode() : Node("navigation_node")
  {
    // 파라미터 선언
    imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/sim/imu");
    gps_topic_ = this->declare_parameter<std::string>("gps_topic", "/sim/gps/pos");
    out_topic_ = this->declare_parameter<std::string>("out_topic", "/nav/odom");

    // EKF 초기화 (일단 0,0,0에서 시작한다고 가정)
    // 실제로는 첫 GPS 수신 시 초기화하는 것이 좋음
    ekf_.init(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());

    // Publisher
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(out_topic_, 10);

    // Subscriber 1: IMU (Prediction 단계 - High Frequency)
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 10,
      std::bind(&NavigationNode::imuCallback, this, std::placeholders::_1));

    // Subscriber 2: GPS (Update 단계 - Low Frequency)
    gps_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      gps_topic_, 10,
      std::bind(&NavigationNode::gpsCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Navigation Node Started with EKF.");
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // 1. dt 계산
    rclcpp::Time current_time = msg->header.stamp;
    if (last_time_.nanoseconds() == 0) {
      last_time_ = current_time;
      return;
    }
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    // dt가 너무 크거나 작으면 예외 처리 (시뮬레이션 리셋 등 상황 대비)
    if (dt <= 0.0 || dt > 1.0) return;

    // 2. IMU 데이터 추출 (ROS msg -> Eigen)
    Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    Eigen::Vector3d gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

    // [추가] 제어기로 넘겨주기 위해 Gyro 데이터 저장 (Bias 보정은 EKF에서 꺼내와도 되지만, 일단 Raw값도 OK)
    // 더 정교하게 하려면: current_gyro_ = gyro - ekf_.getGyroBias();
    current_gyro_ = gyro; 
    // 3. EKF 예측 (Prediction)
    ekf_.predict(acc, gyro, dt);

    // 4. 추정된 상태 Publish (/nav/odom)
    publishOdometry(msg->header.stamp);
  }

  void gpsCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    // 1. GPS 데이터 추출
    Eigen::Vector3d meas_pos(msg->point.x, msg->point.y, msg->point.z);

    // 2. EKF 보정 (Update)
    ekf_.update_gps(meas_pos);
    
    // 로그를 가끔 찍어서 동작 확인
    // RCLCPP_INFO(this->get_logger(), "GPS Update: %.2f, %.2f, %.2f", meas_pos.x(), meas_pos.y(), meas_pos.z());
  }

  void publishOdometry(const rclcpp::Time& stamp)
  {
    // 1. EKF에서 현재 추정된 상태 가져오기
    Eigen::Vector3d p = ekf_.getPosition();
    Eigen::Vector3d v = ekf_.getVelocity();
    Eigen::Quaterniond q = ekf_.getAttitude();

    // ==========================================
    // [추가된 안전장치] NaN 체크 (가장 중요!)
    // ==========================================
    // 위치(p), 속도(v), 자세(q) 중 하나라도 숫자가 아니면(NaN) 절대 보내지 않음
    bool is_p_nan = std::isnan(p.x()) || std::isnan(p.y()) || std::isnan(p.z());
    bool is_v_nan = std::isnan(v.x()) || std::isnan(v.y()) || std::isnan(v.z());
    bool is_q_nan = std::isnan(q.w()) || std::isnan(q.x()) || std::isnan(q.y()) || std::isnan(q.z());

    if (is_p_nan || is_v_nan || is_q_nan) {
        // (선택) 로그를 너무 많이 찍으면 렉 걸리니까, 1초에 한 번만 경고 출력
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                             "EKF Output is NaN! Skipping publish to protect controller.");
        return; // 여기서 함수 강제 종료! (쓰레기 값 전송 차단)
    }

    // ==========================================
    // [추가된 안전장치] 폭주(Divergence) 체크
    // ==========================================
    // 위치가 10km 이상 튀면 필터가 발산한 것으로 간주하고 차단
    if (std::abs(p.x()) > 10000.0 || std::abs(p.y()) > 10000.0 || std::abs(p.z()) > 10000.0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                             "EKF Diverged (Too large position)! Skipping publish.");
        return;
    }

    // ==========================================
    // 2. 데이터가 정상이면 메시지 생성 및 발행
    // ==========================================
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "world";
    odom.child_frame_id = "base_link";

    // Position
    odom.pose.pose.position.x = p.x();
    odom.pose.pose.position.y = p.y();
    odom.pose.pose.position.z = p.z();

    // Orientation
    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();

    // Velocity (World frame)
    odom.twist.twist.linear.x = v.x();
    odom.twist.twist.linear.y = v.y();
    odom.twist.twist.linear.z = v.z();

    // Angular Velocity는 EKF 상태에 없다면 IMU 값을 그대로 쓰거나 비워둠 (여기선 생략)
    // [추가] 제어기의 D-gain을 위해 각속도(Angular Velocity) 필수!!
    odom.twist.twist.angular.x = current_gyro_.x();
    odom.twist.twist.angular.y = current_gyro_.y();
    odom.twist.twist.angular.z = current_gyro_.z();
    
    odom_pub_->publish(odom);
  }

private:
  EKF ekf_;
  rclcpp::Time last_time_{0};

  // [추가] 각속도 전달을 위한 변수
  Eigen::Vector3d current_gyro_{0.0, 0.0, 0.0}; 

  std::string imu_topic_;
  std::string gps_topic_;
  std::string out_topic_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr gps_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationNode>());
  rclcpp::shutdown();
  return 0;
}