#pragma once
#include <cstddef>
#include <cstdint>
namespace inspection_robot_base {
inline std::uint8_t xorBcc(const std::uint8_t* data, std::size_t count) {
  std::uint8_t value = 0;
  for (std::size_t i = 0; i < count; ++i)
    value ^= data[i];
  return value;
}
}  // namespace inspection_robot_base
