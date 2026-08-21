#include "inspection_robot_base/control_state.hpp"
namespace inspection_robot_base {
void SharedControlState::submitVelocity(const VelocityCommand& c) {
  std::lock_guard<std::mutex> l(mutex_);
  state_.command = c;
  state_.have_command = true;
  ++state_.command_seq;
}
void SharedControlState::setEmergencyStop(bool a) {
  std::lock_guard<std::mutex> l(mutex_);
  state_.emergency_stop = a;
}
void SharedControlState::requestReset() {
  std::lock_guard<std::mutex> l(mutex_);
  ++state_.reset_seq;
}
void SharedControlState::enqueueDeviceRequest(const DeviceRequest& r) {
  std::lock_guard<std::mutex> l(mutex_);
  device_requests_.push_back(r);
}
ControlSnapshot SharedControlState::snapshot() const {
  std::lock_guard<std::mutex> l(mutex_);
  return state_;
}
std::deque<DeviceRequest> SharedControlState::drainDeviceRequests() {
  std::lock_guard<std::mutex> l(mutex_);
  auto out = std::move(device_requests_);
  device_requests_.clear();
  return out;
}
void SharedControlState::invalidateCommand() {
  std::lock_guard<std::mutex> l(mutex_);
  state_.have_command = false;
  state_.command = {};
  ++state_.command_seq;
}
}  // namespace inspection_robot_base
