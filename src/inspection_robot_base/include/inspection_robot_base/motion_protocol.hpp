#pragma once
#include <array>
#include <cstdint>
#include <optional>

#include "inspection_robot_base/frame_types.hpp"
namespace inspection_robot_base {
struct MotionCommandContext {
  std::uint8_t recharge_flag{0};
  // This is the reserved motion-frame byte observed in the legacy protocol.
  // It is intentionally NOT assumed to be equivalent to the 0xB0/0xB1 safety configuration command.
  std::uint8_t reserved_byte{0};
};
class MotionProtocol {
 public:
  static constexpr std::size_t kTxFrameSize = 11;
  using TxFrame = std::array<std::uint8_t, kTxFrameSize>;
  static std::optional<MotionFeedback> decode(const ValidatedFrame& frame);
  static std::optional<TxFrame> encodeVelocity(double vx,
                                               double vy,
                                               double wz,
                                               MotionCommandContext context = {});
};
}  // namespace inspection_robot_base
