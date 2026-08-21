#include "inspection_robot_base/ultrasonic_protocol.hpp"
namespace inspection_robot_base {
namespace {
std::int16_t i16(std::uint8_t h, std::uint8_t l) {
  return static_cast<std::int16_t>((static_cast<std::uint16_t>(h) << 8U) | l);
}
}  // namespace
std::optional<UltrasonicFeedback> UltrasonicProtocol::decode(const ValidatedFrame& f) {
  if (f.type() != FrameType::kUltrasonic || f.bytes().size() != 19)
    return std::nullopt;
  UltrasonicFeedback o;
  for (std::size_t i = 0; i < 6; ++i) {
    auto j = 1 + i * 2;
    o.ranges_m[i] = i16(f.bytes()[j], f.bytes()[j + 1]) / 1000.0;
  }
  return o;
}
}  // namespace inspection_robot_base
