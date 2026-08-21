#pragma once
#include <array>
#include <cstdint>
#include <vector>
namespace inspection_robot_base {
enum class FrameType : std::uint8_t { kMotionFeedback, kUltrasonic, kCharging };
class ValidatedFrame {
 public:
  FrameType type() const {
    return type_;
  }
  const std::vector<std::uint8_t>& bytes() const {
    return bytes_;
  }

 private:
  friend class SerialFrameRouter;
  ValidatedFrame(FrameType type, std::vector<std::uint8_t> bytes)
      : type_(type), bytes_(std::move(bytes)) {
  }
  FrameType type_;
  std::vector<std::uint8_t> bytes_;
};
struct RouterStats {
  std::uint64_t accepted_motion{0}, accepted_ultrasonic{0}, accepted_charging{0};
  std::uint64_t checksum_errors{0}, framing_errors{0}, discarded_bytes{0};
};
struct MotionFeedback {
  std::uint8_t stop_flag{0};
  double vx{0.0}, vy{0.0}, wz{0.0};
  double accel_x{0.0}, accel_y{0.0}, accel_z{0.0};
  double gyro_x{0.0}, gyro_y{0.0}, gyro_z{0.0};
  double battery_voltage{0.0};
};
struct UltrasonicFeedback {
  std::array<double, 6> ranges_m{};
};
struct ChargingFeedback {
  double current_a{0.0};
  std::uint8_t infrared_state{0};
  bool charging{false};
  bool charge_mode_set{false};
};
struct VelocityCommand {
  double vx{0.0}, vy{0.0}, wz{0.0};
};
}  // namespace inspection_robot_base
