#pragma once
#include <optional>
#include "inspection_robot_base/frame_types.hpp"
namespace inspection_robot_base {
class ChargingProtocol {
public:
  static std::optional<ChargingFeedback> decode(const ValidatedFrame & frame);
};
}
