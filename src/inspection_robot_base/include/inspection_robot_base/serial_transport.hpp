#pragma once
#include <serial/serial.h>

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "inspection_robot_base/byte_transport.hpp"
namespace inspection_robot_base {
struct SerialConfig {
  std::string port{"/dev/inspection_robot_controller"};
  std::uint32_t baud_rate{115200};
  std::uint32_t timeout_ms{20};
};
class SerialTransport final : public IByteTransport {
 public:
  explicit SerialTransport(SerialConfig config);
  bool open() override;
  void close() override;
  bool isOpen() const override;
  bool flushInput() override;
  bool readAvailable(std::vector<std::uint8_t>& out, std::size_t max_bytes) override;
  bool write(const std::uint8_t* data, std::size_t size) override;
  std::string lastError() const override;

 private:
  void setError(const std::string& value);
  SerialConfig config_;
  mutable std::mutex error_mutex_;
  std::string last_error_;
  serial::Serial serial_;
};
}  // namespace inspection_robot_base
