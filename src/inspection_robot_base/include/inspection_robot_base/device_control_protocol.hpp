#pragma once
#include <array>
#include <cstdint>
namespace inspection_robot_base {
class DeviceControlProtocol {
public:
  static constexpr std::size_t kFrameSize = 11;
  using Frame = std::array<std::uint8_t, kFrameSize>;
  static Frame setRechargeEnabled(bool enabled);
  static Frame setMcuSafetyEnabled(bool enabled);
  static Frame setLight(bool enabled, std::uint8_t r, std::uint8_t g, std::uint8_t b);
private:
  static Frame finalize(Frame frame);
};
}
