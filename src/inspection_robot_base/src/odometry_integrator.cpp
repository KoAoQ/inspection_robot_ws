#include "inspection_robot_base/odometry_integrator.hpp"
#include <cmath>
namespace inspection_robot_base { const OdometryState& OdometryIntegrator::update(double vx,double vy,double wz,double dt){if(dt<=0.0||dt>1.0)return state_;vx*=scale_.x;vy*=scale_.y;wz*=wz>=0.0?scale_.yaw_positive:scale_.yaw_negative;state_.x+=(vx*std::cos(state_.yaw)-vy*std::sin(state_.yaw))*dt;state_.y+=(vx*std::sin(state_.yaw)+vy*std::cos(state_.yaw))*dt;state_.yaw+=wz*dt;state_.yaw=std::atan2(std::sin(state_.yaw),std::cos(state_.yaw));return state_;} void OdometryIntegrator::reset(){state_={};} }
