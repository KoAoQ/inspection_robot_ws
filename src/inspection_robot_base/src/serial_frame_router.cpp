#include "inspection_robot_base/serial_frame_router.hpp"
namespace inspection_robot_base {
namespace {
constexpr std::uint8_t kMotionHeader = 0x7B, kMotionTail = 0x7D, kUltrasonicHeader = 0xFA,
                       kUltrasonicTail = 0xFC, kChargingHeader = 0x7C, kChargingTail = 0x7F;
constexpr std::size_t kMotionSize = 24, kUltrasonicSize = 19, kChargingSize = 8;
}  // namespace
void SerialFrameRouter::pushBytes(const std::uint8_t* data, std::size_t size) {
  for (std::size_t i = 0; i < size; ++i)
    buffer_.push_back(data[i]);
}
void SerialFrameRouter::clear() {
  buffer_.clear();
}
std::optional<SerialFrameRouter::Descriptor> SerialFrameRouter::descriptorFor(std::uint8_t h) {
  switch (h) {
    case kMotionHeader:
      return Descriptor{FrameType::kMotionFeedback, kMotionSize, kMotionTail};
    case kUltrasonicHeader:
      return Descriptor{FrameType::kUltrasonic, kUltrasonicSize, kUltrasonicTail};
    case kChargingHeader:
      return Descriptor{FrameType::kCharging, kChargingSize, kChargingTail};
    default:
      return std::nullopt;
  }
}
bool SerialFrameRouter::checksumValid(const std::deque<std::uint8_t>& b, std::size_t n) {
  std::uint8_t x = 0;
  for (std::size_t i = 0; i < n - 2; ++i)
    x ^= b[i];
  return x == b[n - 2];
}
std::optional<ValidatedFrame> SerialFrameRouter::popNext() {
  while (!buffer_.empty()) {
    auto d = descriptorFor(buffer_.front());
    if (!d) {
      buffer_.pop_front();
      ++stats_.discarded_bytes;
      continue;
    }
    if (buffer_.size() < d->size)
      return std::nullopt;
    if (buffer_[d->size - 1] != d->tail) {
      buffer_.pop_front();
      ++stats_.framing_errors;
      ++stats_.discarded_bytes;
      continue;
    }
    if (!checksumValid(buffer_, d->size)) {
      buffer_.pop_front();
      ++stats_.checksum_errors;
      ++stats_.discarded_bytes;
      continue;
    }
    std::vector<std::uint8_t> bytes;
    bytes.reserve(d->size);
    for (std::size_t i = 0; i < d->size; ++i) {
      bytes.push_back(buffer_.front());
      buffer_.pop_front();
    }
    switch (d->type) {
      case FrameType::kMotionFeedback:
        ++stats_.accepted_motion;
        break;
      case FrameType::kUltrasonic:
        ++stats_.accepted_ultrasonic;
        break;
      case FrameType::kCharging:
        ++stats_.accepted_charging;
        break;
    }
    return ValidatedFrame(d->type, std::move(bytes));
  }
  return std::nullopt;
}
}  // namespace inspection_robot_base
