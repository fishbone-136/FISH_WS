#ifndef FAST_LIO_STATIONARY_GUARD_HPP_
#define FAST_LIO_STATIONARY_GUARD_HPP_

#include <array>
#include <cmath>
#include <limits>

namespace fast_lio_guard
{
// Independent command + leg-state evidence. Missing/stale evidence never
// means stationary, and a nonzero command releases the constraint at once.
class StationaryGuard
{
public:
  void command(double now, double vx, double vy, double wz)
  {
    command_time_ = now;
    zero_ = std::isfinite(vx) && std::isfinite(vy) && std::isfinite(wz) &&
      std::abs(vx) <= 1.e-6 && std::abs(vy) <= 1.e-6 && std::abs(wz) <= 1.e-6;
    if (!zero_) {reset();}
  }

  void feedback(double now, const std::array<double, 3> &position,
    const std::array<double, 3> &rpy, double speed, double yaw_rate)
  {
    feedback_time_ = now;
    stopped_ = std::isfinite(speed) && speed < .025 &&
      std::isfinite(yaw_rate) && std::abs(yaw_rate) < .06;
    for (int i = 0; i < 3; ++i) {
      stopped_ = stopped_ && std::isfinite(position[i]) && std::isfinite(rpy[i]);
    }
    if (!stopped_) {reset(); return;}
    if (!reference_) {
      position_ = position;
      rpy_ = rpy;
      reference_ = true;
    }
    for (int i = 0; i < 3; ++i) {
      if (std::abs(position[i] - position_[i]) > .02 ||
        std::abs(std::remainder(rpy[i] - rpy_[i], 2. * 3.141592653589793)) > .035)
      {
        reset();
        return;
      }
    }
  }

  bool stationary(double now, bool imu_quiet)
  {
    if (!zero_ || !stopped_ || !reference_ || !imu_quiet ||
      now < command_time_ || now < feedback_time_ ||
      now - command_time_ > .25 || now - feedback_time_ > .25)
    {
      reset();
      return false;
    }
    if (quiet_since_ < 0.) {quiet_since_ = now;}
    return now - quiet_since_ >= .8;
  }

private:
  void reset() {quiet_since_ = -1.; reference_ = false;}
  double command_time_ = -std::numeric_limits<double>::infinity();
  double feedback_time_ = -std::numeric_limits<double>::infinity();
  double quiet_since_ = -1.;
  bool zero_ = false, stopped_ = false, reference_ = false;
  std::array<double, 3> position_{}, rpy_{};
};
}  // namespace fast_lio_guard
#endif
