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


// 1. sim_node가 /control/wrench를 subscribe 해서 “가장 최근 wrench(힘/토크)”를 저장
// 2. 타이머(dt)마다 sixdof를 한 스텝 적분
// 3. 결과 상태를 /sim/odom으로 publish
class SimNode : public rclcpp::Node
{
public:
  SimNode() : Node("sim_node") // SimNode 노드 생성자, 노드 이름 "sim_node"로 초기화
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
    // "/sim/odom" 토픽으로 Odometry 메시지를 발행하는 큐 사이즈 10의 퍼블리셔 생성
    wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/control/wrench", 10,
      std::bind(&SimNode::wrenchCallback, this, std::placeholders::_1)
    );
    // create_subscription<메시지타입>(토픽이름, QoS, 콜백함수)
    // "/control/wrench" 토픽을 구독하여 힘/토크(WrenchStamped) 데이터를 수신하고,
    // 메시지가 도착하면 클래스 내 wrenchCallback 멤버 함수를 호출하도록 설정, 큐 사이즈는 10으로 설정
    // &SimNode::wrenchCallback → 멤버 함수 주소
    // this → 이 객체의 인스턴스
    // _1 → 첫 번째 인자를 그대로 전달
    // ===== timer =====
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt_),
      std::bind(&SimNode::onTimer, this)
    );
    // “dt_ 초마다 onTimer() 함수를 실행하는 타이머를 만듦
    RCLCPP_INFO(this->get_logger(), "simulation_node started (dt=%.4f)", dt_);
  }

private:
  void wrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    // control_node가 /control/wrench에 publish한 힘/토크를
    // sim_node가 받아서 sixdof에 넣을 **Input 구조체(input_)**에 저장하는 역할
    // SharedPtr는 ROS2에서 메시지를 효율적으로 전달하기 위한 스마트 포인터로, 메시지 데이터를 복사하지 않고 참조를 공유할 수 있게 해준다. 
    // 따라서 콜백 함수에서 메시지 데이터를 안전하게 읽을 수 있다.
    // 여기서는 /control/wrench가 "body frame 기준 힘/토크"라고 가정
    // (sixdof도 thrust_body/moment_body를 body frame으로 받도록 설계되어 있음)
    std::lock_guard<std::mutex> lock(mtx_);
    // mutex로 input_에 대한 동시 접근을 방지하여 쓰레드 안전하게 업데이트, RAII 스타일로 lock 관리
    input_.thrust_body = { msg->wrench.force.x,  msg->wrench.force.y,  msg->wrench.force.z };
    input_.moment_body = { msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z };
  }

  void onTimer()
  {
    // 적분(상태 업데이트)
    Input u_copy;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      // mutex로 input_에 대한 동시 접근을 방지하여 쓰레드 안전하게 읽기, RAII 스타일로 lock 관리
      u_copy = input_; // 복사본을 만들어서 뒷부분 적분에 사용
    }

    state_ = rk4_step(state_, u_copy, params_, dt_);

    // Odometry publish
    nav_msgs::msg::Odometry odom; // 드론/로봇의 상태를 나타내는 표준 메시지 타입
    odom.header.stamp = this->now(); // 메시지가 생성된 시간, stamp를 통해 time synchronization 가능
    odom.header.frame_id = "world"; // world 좌표계 기준
    odom.child_frame_id = "base_link"; // 드론/로봇 본체 프레임

    odom.pose.pose.position.x = state_.p.x;
    odom.pose.pose.position.y = state_.p.y;
    odom.pose.pose.position.z = state_.p.z;

    odom.pose.pose.orientation.w = state_.q.w;
    odom.pose.pose.orientation.x = state_.q.x;
    odom.pose.pose.orientation.y = state_.q.y;
    odom.pose.pose.orientation.z = state_.q.z;

    // 여기서 state_.v는 world frame 속도, state_.w는 body frame 각속도
    // 일단은 그대로 넣고, 나중에 frame 정교화(또는 tf)로 다듬기로
    odom.twist.twist.linear.x = state_.v.x;
    odom.twist.twist.linear.y = state_.v.y;
    odom.twist.twist.linear.z = state_.v.z;

    odom.twist.twist.angular.x = state_.w.x;
    odom.twist.twist.angular.y = state_.w.y;
    odom.twist.twist.angular.z = state_.w.z;

    odom_pub_->publish(odom); // 퍼블리셔로 메시지 발행하여 ROS 네트워크에 전파
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