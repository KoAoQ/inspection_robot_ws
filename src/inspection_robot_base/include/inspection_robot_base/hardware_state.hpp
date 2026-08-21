#pragma once
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include "inspection_robot_base/base_fault_manager.hpp"
#include "inspection_robot_base/frame_types.hpp"
#include "inspection_robot_base/odometry_integrator.hpp"
namespace inspection_robot_base {
struct HardwareSnapshot {
  MotionFeedback motion{}; UltrasonicFeedback ultrasonic{}; ChargingFeedback charging{}; OdometryState odometry{};
  bool have_motion{false}, have_ultrasonic{false}, have_charging{false};
  std::uint64_t motion_seq{0}, ultrasonic_seq{0}, charging_seq{0};
  std::chrono::steady_clock::time_point motion_time{}, ultrasonic_time{}, charging_time{};
  RouterStats router_stats{};
  FaultSnapshot fault{};
  std::string serial_error;
  bool mcu_safety_requested{false};
};
class HardwareStateStore {
public:
  HardwareSnapshot snapshot() const;
  void update(const HardwareSnapshot & value);
private:
  mutable std::mutex mutex_;
  HardwareSnapshot value_;
};
}
