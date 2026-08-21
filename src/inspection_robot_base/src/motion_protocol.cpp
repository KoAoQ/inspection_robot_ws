#include "inspection_robot_base/motion_protocol.hpp"
#include <cmath>
#include <limits>
#include "inspection_robot_base/checksum.hpp"
namespace inspection_robot_base { namespace {
std::int16_t i16(std::uint8_t h,std::uint8_t l){return static_cast<std::int16_t>((static_cast<std::uint16_t>(h)<<8U)|l);} std::uint16_t u16(std::uint8_t h,std::uint8_t l){return static_cast<std::uint16_t>((static_cast<std::uint16_t>(h)<<8U)|l);} 
bool milli(double v,std::int16_t& o){if(!std::isfinite(v))return false;double s=std::round(v*1000.0);if(s<std::numeric_limits<std::int16_t>::min()||s>std::numeric_limits<std::int16_t>::max())return false;o=static_cast<std::int16_t>(s);return true;}
void put(MotionProtocol::TxFrame& f,std::size_t i,std::int16_t v){auto r=static_cast<std::uint16_t>(v);f[i]=static_cast<std::uint8_t>((r>>8U)&0xFFU);f[i+1]=static_cast<std::uint8_t>(r&0xFFU);} }
std::optional<MotionFeedback> MotionProtocol::decode(const ValidatedFrame& f){const auto& b=f.bytes();if(f.type()!=FrameType::kMotionFeedback||b.size()!=24)return std::nullopt;MotionFeedback o;o.stop_flag=b[1];o.vx=i16(b[2],b[3])/1000.0;o.vy=i16(b[4],b[5])/1000.0;o.wz=i16(b[6],b[7])/1000.0;o.accel_x=i16(b[8],b[9])/1671.84;o.accel_y=i16(b[10],b[11])/1671.84;o.accel_z=i16(b[12],b[13])/1671.84;o.gyro_x=i16(b[14],b[15])*0.00026644;o.gyro_y=i16(b[16],b[17])*0.00026644;o.gyro_z=i16(b[18],b[19])*0.00026644;o.battery_voltage=u16(b[20],b[21])/1000.0;return o;}
std::optional<MotionProtocol::TxFrame> MotionProtocol::encodeVelocity(double vx,double vy,double wz,MotionCommandContext c){std::int16_t x=0,y=0,z=0;if(!milli(vx,x)||!milli(vy,y)||!milli(wz,z))return std::nullopt;TxFrame f{};f[0]=0x7B;f[1]=c.recharge_flag;f[2]=c.reserved_byte;put(f,3,x);put(f,5,y);put(f,7,z);f[9]=xorBcc(f.data(),9);f[10]=0x7D;return f;}
}
