#include <cctype>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <inspection_robot_interfaces/srv/set_control_mode.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
using namespace std::chrono_literals;
class VelocityManager : public rclcpp::Node {
  using Clock = std::chrono::steady_clock;
  struct Source {
    geometry_msgs::msg::Twist msg;
    Clock::time_point t{};
    bool have{false};
  };

 public:
  VelocityManager() : Node("inspection_robot_velocity_manager") {
    timeout_ = std::chrono::milliseconds(declare_parameter<int>("source_timeout_ms", 500));
    mode_ = declare_parameter<std::string>("initial_mode", "STOP");
    pub_ =
        create_publisher<geometry_msgs::msg::Twist>("/control/cmd_vel", rclcpp::QoS(1).reliable());
    manual_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel/manual", 1, [this](geometry_msgs::msg::Twist::SharedPtr m) {
          update(manual_s_, *m);
        });
    nav_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel/nav", 1, [this](geometry_msgs::msg::Twist::SharedPtr m) {
          update(nav_s_, *m);
        });
    follow_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel/follow", 1, [this](geometry_msgs::msg::Twist::SharedPtr m) {
          update(follow_s_, *m);
        });
    srv_ = create_service<inspection_robot_interfaces::srv::SetControlMode>(
        "/control/set_mode",
        [this](const std::shared_ptr<inspection_robot_interfaces::srv::SetControlMode::Request> q,
               std::shared_ptr<inspection_robot_interfaces::srv::SetControlMode::Response> r) {
          setMode(q, r);
        });
    timer_ = create_wall_timer(20ms, [this]() {
      tick();
    });
  }

 private:
  void update(Source& s, const geometry_msgs::msg::Twist& msg) {
    std::lock_guard<std::mutex> l(mu_);
    s.msg = msg;
    s.t = Clock::now();
    s.have = true;
  }
  void setMode(const std::shared_ptr<inspection_robot_interfaces::srv::SetControlMode::Request> q,
               std::shared_ptr<inspection_robot_interfaces::srv::SetControlMode::Response> r) {
    std::string m = q->mode;
    for (char& c : m)
      c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    if (m != "STOP" && m != "MANUAL" && m != "NAVIGATION" && m != "FOLLOW") {
      r->success = false;
      r->active_mode = mode_;
      r->message = "valid modes: STOP MANUAL NAVIGATION FOLLOW";
      return;
    }
    std::lock_guard<std::mutex> l(mu_);
    mode_ = m;
    if (mode_ == "MANUAL")
      manual_s_.have = false;
    else if (mode_ == "NAVIGATION")
      nav_s_.have = false;
    else if (mode_ == "FOLLOW")
      follow_s_.have = false;
    r->success = true;
    r->active_mode = mode_;
    r->message = "mode updated; a fresh source command is required";
  }
  bool fresh(const Source& s, Clock::time_point n) const {
    return s.have && (n - s.t) <= timeout_;
  }
  static bool finite(const geometry_msgs::msg::Twist& m) {
    return std::isfinite(m.linear.x) && std::isfinite(m.linear.y) && std::isfinite(m.angular.z);
  }
  void tick() {
    geometry_msgs::msg::Twist out;
    auto now = Clock::now();
    std::lock_guard<std::mutex> l(mu_);
    Source* s = nullptr;
    if (mode_ == "MANUAL")
      s = &manual_s_;
    else if (mode_ == "NAVIGATION")
      s = &nav_s_;
    else if (mode_ == "FOLLOW")
      s = &follow_s_;
    if (s && fresh(*s, now) && finite(s->msg))
      out = s->msg;
    pub_->publish(out);
  }
  std::mutex mu_;
  std::string mode_;
  std::chrono::milliseconds timeout_{500};
  Source manual_s_, nav_s_, follow_s_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_, nav_, follow_;
  rclcpp::Service<inspection_robot_interfaces::srv::SetControlMode>::SharedPtr srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityManager>());
  rclcpp::shutdown();
  return 0;
}
