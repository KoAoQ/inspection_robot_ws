#include "inspection_robot_base/device_control_protocol.hpp"
#include "inspection_robot_base/checksum.hpp"
namespace inspection_robot_base {
DeviceControlProtocol::Frame DeviceControlProtocol::finalize(Frame f){f[0]=0x7B;f[9]=xorBcc(f.data(),9);f[10]=0x7D;return f;}
DeviceControlProtocol::Frame DeviceControlProtocol::setRechargeEnabled(bool e){Frame f{};f[1]=1;f[2]=e?0xA1:0xA0;return finalize(f);} 
DeviceControlProtocol::Frame DeviceControlProtocol::setMcuSafetyEnabled(bool e){Frame f{};f[1]=0;f[2]=e?0xB1:0xB0;return finalize(f);} 
DeviceControlProtocol::Frame DeviceControlProtocol::setLight(bool e,std::uint8_t r,std::uint8_t g,std::uint8_t b){Frame f{};f[1]=0x04;f[2]=e?1:0;f[3]=r;f[4]=g;f[5]=b;return finalize(f);} }
