#pragma once
namespace inspection_robot_base {
struct OdometryState {
  double x{0.0}, y{0.0}, yaw{0.0};
};
struct OdometryScale {
  double x{1.0}, y{1.0}, yaw_positive{1.0}, yaw_negative{1.0};
};
class OdometryIntegrator {
 public:
  explicit OdometryIntegrator(OdometryScale scale = {}) : scale_(scale) {
  }
  const OdometryState& update(double vx, double vy, double wz, double dt_sec);
  const OdometryState& state() const {
    return state_;
  }
  void reset();

 private:
  OdometryScale scale_;
  OdometryState state_;
};
}  // namespace inspection_robot_base
