#pragma once
#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>

#include "inspection_robot_base/frame_types.hpp"
namespace inspection_robot_base {
enum class DeviceRequestType : std::uint8_t { kRecharge, kMcuSafety, kLight };
struct DeviceRequest {
  DeviceRequestType type{DeviceRequestType::kRecharge};
  bool enabled{false};
  std::uint8_t r{0}, g{0}, b{0};
};
struct ControlSnapshot {
  VelocityCommand command{};
  std::uint64_t command_seq{0};
  bool have_command{false};
  bool emergency_stop{false};
  std::uint64_t reset_seq{0};
};
class SharedControlState {
 public:
  void submitVelocity(const VelocityCommand& command);
  void setEmergencyStop(bool active);
  void requestReset();
  void enqueueDeviceRequest(const DeviceRequest& request);
  ControlSnapshot snapshot() const;
  std::deque<DeviceRequest> drainDeviceRequests();
  void invalidateCommand();

 private:
  mutable std::mutex mutex_;
  ControlSnapshot state_;
  std::deque<DeviceRequest> device_requests_;
};
}  // namespace inspection_robot_base
