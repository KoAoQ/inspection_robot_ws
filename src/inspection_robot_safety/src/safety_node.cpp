#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <inspection_robot_interfaces/msg/safety_status.hpp>
#include <limits>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
using namespace std::chrono_literals;
class SafetyNode : public rclcpp::Node {
  using Clock = std::chrono::steady_clock;
  struct RangeState {
    double value{std::numeric_limits<double>::quiet_NaN()};
    double min_range{0.0};
    double max_range{0.0};
    Clock::time_point t{};
    bool have{false};
  };

 public:
  SafetyNode() : Node("inspection_robot_safety") {
    hard_ = declare_parameter<double>("ultrasonic.hard_stop_distance", 0.15);
    slow_ = declare_parameter<double>("ultrasonic.slow_distance", 0.40);
    stale_ = std::chrono::milliseconds(declare_parameter<int>("ultrasonic.stale_timeout_ms", 300));
    cmd_stale_ = std::chrono::milliseconds(declare_parameter<int>("command.stale_timeout_ms", 500));
    fail_closed_ = declare_parameter<bool>("ultrasonic.fail_closed", true);
    use_laser_ = declare_parameter<bool>("laser.enabled", false);
    laser_hard_ = declare_parameter<double>("laser.hard_stop_distance", 0.18);
    laser_stale_ = std::chrono::milliseconds(declare_parameter<int>("laser.stale_timeout_ms", 300));
    laser_half_angle_ = declare_parameter<double>("laser.front_half_angle_rad", 0.52);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/control/cmd_vel",
        rclcpp::QoS(1).reliable(),
        [this](geometry_msgs::msg::Twist::SharedPtr m) {
          std::lock_guard<std::mutex> l(mu_);
          cmd_ = *m;
          cmd_t_ = Clock::now();
          have_cmd_ = true;
        });
    estop_sub_ = create_subscription<std_msgs::msg::Bool>("/base/emergency_stop",
                                                          rclcpp::QoS(1).reliable(),
                                                          [this](std_msgs::msg::Bool::SharedPtr m) {
                                                            std::lock_guard<std::mutex> l(mu_);
                                                            estop_ = m->data;
                                                          });
    for (std::size_t i = 0; i < 4; ++i) {
      range_subs_[i] = create_subscription<sensor_msgs::msg::Range>(
          "/ultrasonic/" + std::string(1, static_cast<char>('a' + i)),
          rclcpp::SensorDataQoS(),
          [this, i](sensor_msgs::msg::Range::SharedPtr m) {
            std::lock_guard<std::mutex> l(mu_);
            ranges_[i].value = m->range;
            ranges_[i].min_range = m->min_range;
            ranges_[i].max_range = m->max_range;
            ranges_[i].t = Clock::now();
            ranges_[i].have = true;
          });
    }
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::SharedPtr m) {
          onScan(*m);
        });
    pub_ = create_publisher<geometry_msgs::msg::Twist>("/base/safe_cmd_vel",
                                                       rclcpp::QoS(1).reliable());
    status_pub_ =
        create_publisher<inspection_robot_interfaces::msg::SafetyStatus>("/safety/status", 10);
    timer_ = create_wall_timer(20ms, [this]() {
      tick();
    });
  }

 private:
  void onScan(const sensor_msgs::msg::LaserScan& m) {
    std::lock_guard<std::mutex> l(mu_);
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < m.ranges.size(); ++i) {
      double a = m.angle_min + static_cast<double>(i) * m.angle_increment;
      if (std::abs(a) > laser_half_angle_)
        continue;
      double r = m.ranges[i];
      if (std::isfinite(r) && r >= m.range_min && r <= m.range_max)
        best = std::min(best, r);
    }
    laser_min_ = best;
    laser_t_ = Clock::now();
    have_laser_ = true;
  }
  static bool finiteCmd(const geometry_msgs::msg::Twist& m) {
    return std::isfinite(m.linear.x) && std::isfinite(m.linear.y) && std::isfinite(m.angular.z);
  }
  void tick() {
    const auto steady_now = Clock::now();
    geometry_msgs::msg::Twist out;
    inspection_robot_interfaces::msg::SafetyStatus st;
    st.header.stamp = this->now();
    std::lock_guard<std::mutex> l(mu_);
    bool cmd_ok = have_cmd_ && (steady_now - cmd_t_) <= cmd_stale_ && finiteCmd(cmd_);
    double nearest = std::numeric_limits<double>::infinity();
    bool ultra_ok = true;
    for (const auto& r : ranges_) {
      bool fresh = r.have && (steady_now - r.t) <= stale_;
      bool no_hit = std::isinf(r.value) && r.value > 0.0;
      bool measured = std::isfinite(r.value) && r.value >= r.min_range && r.value <= r.max_range;
      bool valid = fresh && (no_hit || measured);
      ultra_ok = ultra_ok && valid;
      if (measured)
        nearest = std::min(nearest, r.value);
    }
    bool laser_ok = !use_laser_ || (have_laser_ && (steady_now - laser_t_) <= laser_stale_);
    bool obstacle =
        (std::isfinite(nearest) && nearest <= hard_) ||
        (use_laser_ && laser_ok && std::isfinite(laser_min_) && laser_min_ <= laser_hard_);
    bool healthy = cmd_ok && !estop_ && (!fail_closed_ || ultra_ok) && laser_ok && !obstacle;
    if (healthy) {
      out = cmd_;
      if (out.linear.x > 0.0 && std::isfinite(nearest) && nearest < slow_ && nearest > hard_) {
        double scale = (nearest - hard_) / (slow_ - hard_);
        out.linear.x *= std::clamp(scale, 0.0, 1.0);
      }
    }
    pub_->publish(out);
    st.healthy = healthy;
    st.emergency_stop = estop_;
    st.ultrasonic_healthy = ultra_ok;
    st.laser_healthy = laser_ok;
    st.obstacle_stop = obstacle;
    st.command_valid = cmd_ok;
    st.nearest_ultrasonic_m = static_cast<float>(nearest);
    st.nearest_laser_m = static_cast<float>(laser_min_);
    if (estop_)
      st.reason = "emergency stop";
    else if (!cmd_ok)
      st.reason = "command stale or invalid";
    else if (fail_closed_ && !ultra_ok)
      st.reason = "front ultrasonic stale/invalid";
    else if (!laser_ok)
      st.reason = "laser stale";
    else if (obstacle)
      st.reason = "hard-stop obstacle";
    else
      st.reason = "healthy";
    status_pub_->publish(st);
  }
  std::mutex mu_;
  double hard_{.15}, slow_{.4}, laser_hard_{.18}, laser_half_angle_{.52};
  std::chrono::milliseconds stale_{300}, cmd_stale_{500}, laser_stale_{300};
  bool fail_closed_{true}, use_laser_{false}, estop_{false}, have_cmd_{false}, have_laser_{false};
  geometry_msgs::msg::Twist cmd_;
  Clock::time_point cmd_t_{}, laser_t_{};
  double laser_min_{std::numeric_limits<double>::infinity()};
  std::array<RangeState, 4> ranges_{};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  std::array<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr, 4> range_subs_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Publisher<inspection_robot_interfaces::msg::SafetyStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyNode>());
  rclcpp::shutdown();
  return 0;
}
