#include <gtest/gtest.h>
#include <array>
#include <vector>
#include "inspection_robot_base/checksum.hpp"
#include "inspection_robot_base/motion_protocol.hpp"
#include "inspection_robot_base/serial_frame_router.hpp"
using namespace inspection_robot_base;
namespace { std::vector<std::uint8_t> make(std::uint8_t h,std::size_t n,std::uint8_t tail){std::vector<std::uint8_t> f(n,0);f[0]=h;f[n-2]=xorBcc(f.data(),n-2);f[n-1]=tail;return f;} }
TEST(Router, RecoversFromGarbageAndRoutesMixedFrames){SerialFrameRouter r;auto m=make(0x7B,24,0x7D);auto u=make(0xFA,19,0xFC);auto c=make(0x7C,8,0x7F);std::vector<std::uint8_t> stream={0x11,0x22,0x33};stream.insert(stream.end(),m.begin(),m.end());stream.insert(stream.end(),u.begin(),u.end());stream.insert(stream.end(),c.begin(),c.end());r.pushBytes(stream.data(),stream.size());auto a=r.popNext();ASSERT_TRUE(a);EXPECT_EQ(a->type(),FrameType::kMotionFeedback);auto b=r.popNext();ASSERT_TRUE(b);EXPECT_EQ(b->type(),FrameType::kUltrasonic);auto d=r.popNext();ASSERT_TRUE(d);EXPECT_EQ(d->type(),FrameType::kCharging);}
TEST(Router, BadChecksumDropsOneByteAndFindsNextFrame){SerialFrameRouter r;auto bad=make(0x7B,24,0x7D);bad[5]^=0x7F;auto good=make(0x7B,24,0x7D);bad.insert(bad.end(),good.begin(),good.end());r.pushBytes(bad.data(),bad.size());auto f=r.popNext();ASSERT_TRUE(f);EXPECT_EQ(f->type(),FrameType::kMotionFeedback);EXPECT_GE(r.stats().checksum_errors,1u);}
TEST(MotionProtocol, RejectsNonFinite){auto f=MotionProtocol::encodeVelocity(std::numeric_limits<double>::quiet_NaN(),0,0);EXPECT_FALSE(f);}
