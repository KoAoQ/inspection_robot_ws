#include "inspection_robot_base/hardware_state.hpp"
namespace inspection_robot_base {
HardwareSnapshot HardwareStateStore::snapshot() const {
  std::lock_guard<std::mutex> l(mutex_);
  return value_;
}
void HardwareStateStore::update(const HardwareSnapshot& v) {
  std::lock_guard<std::mutex> l(mutex_);
  value_ = v;
}
}  // namespace inspection_robot_base
