#pragma once
#include <optional>

#include "inspection_robot_base/frame_types.hpp"
namespace inspection_robot_base {
class UltrasonicProtocol {
 public:
  static std::optional<UltrasonicFeedback> decode(const ValidatedFrame& frame);
};
}  // namespace inspection_robot_base
