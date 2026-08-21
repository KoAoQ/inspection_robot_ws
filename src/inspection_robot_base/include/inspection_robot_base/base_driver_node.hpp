#pragma once
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <inspection_robot_interfaces/msg/base_status.hpp>
#include <inspection_robot_interfaces/msg/charging_state.hpp>
#include "inspection_robot_base/base_io_worker.hpp"
#include "inspection_robot_base/control_state.hpp"
#include "inspection_robot_base/hardware_state.hpp"
namespace inspection_robot_base {
class BaseDriverNode : public rclcpp::Node {
public:
  explicit BaseDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~BaseDriverNode() override;
private:
  void onVelocity(const geometry_msgs::msg::Twist::SharedPtr msg);
  void onEmergencyStop(const std_msgs::msg::Bool::SharedPtr msg);
  void onReset(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void onRecharge(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void onMcuSafety(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void onLight(const std_msgs::msg::ColorRGBA::SharedPtr msg);
  void publishState();
  void publishMotion(const HardwareSnapshot & snapshot);
  void publishUltrasonic(const HardwareSnapshot & snapshot);
  void publishCharging(const HardwareSnapshot & snapshot);
  void publishStatus(const HardwareSnapshot & snapshot);
  std::shared_ptr<SharedControlState> control_;
  std::shared_ptr<HardwareStateStore> hardware_;
  std::unique_ptr<BaseIoWorker> worker_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr light_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr recharge_srv_, mcu_safety_srv_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  std::array<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr, 6> range_pubs_;
  rclcpp::Publisher<inspection_robot_interfaces::msg::ChargingState>::SharedPtr charging_pub_;
  rclcpp::Publisher<inspection_robot_interfaces::msg::BaseStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::uint64_t published_motion_seq_{0}, published_ultrasonic_seq_{0}, published_charging_seq_{0};
  std::string odom_frame_, base_frame_, imu_frame_;
  std::array<std::string, 6> ultrasonic_frames_{};
  double ultrasonic_fov_{0.52}, ultrasonic_min_{0.02}, ultrasonic_max_{5.3};
  bool allow_mcu_safety_disable_{false};
};
}
