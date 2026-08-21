#pragma once
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>
namespace inspection_robot_base {
class IByteTransport {
public:
  virtual ~IByteTransport() = default;
  virtual bool open() = 0;
  virtual void close() = 0;
  virtual bool isOpen() const = 0;
  virtual bool flushInput() = 0;
  virtual bool readAvailable(std::vector<std::uint8_t> & out, std::size_t max_bytes) = 0;
  virtual bool write(const std::uint8_t * data, std::size_t size) = 0;
  virtual std::string lastError() const = 0;
};
}
