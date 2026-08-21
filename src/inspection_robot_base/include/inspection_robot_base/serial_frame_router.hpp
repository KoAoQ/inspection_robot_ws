#pragma once
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>

#include "inspection_robot_base/frame_types.hpp"
namespace inspection_robot_base {
class SerialFrameRouter {
 public:
  void pushBytes(const std::uint8_t* data, std::size_t size);
  std::optional<ValidatedFrame> popNext();
  void clear();
  RouterStats stats() const {
    return stats_;
  }

 private:
  struct Descriptor {
    FrameType type;
    std::size_t size;
    std::uint8_t tail;
  };
  static std::optional<Descriptor> descriptorFor(std::uint8_t header);
  static bool checksumValid(const std::deque<std::uint8_t>& buffer, std::size_t frame_size);
  std::deque<std::uint8_t> buffer_;
  RouterStats stats_;
};
}  // namespace inspection_robot_base
