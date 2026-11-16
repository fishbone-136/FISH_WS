#ifndef FAST_LIO_FRONTEND_GUARD_HPP_
#define FAST_LIO_FRONTEND_GUARD_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace fast_lio_guard
{
// HAP already supplies per-point time. A short frame must never borrow the
// mean duration: doing so consumes IMU samples belonging to the next frame.
inline double scan_duration(double point_duration, double mean_duration, bool timed_hap)
{
  if (timed_hap) {return point_duration;}
  return point_duration < 0.5 * mean_duration ? mean_duration : point_duration;
}

// Keep the newest complete scan after a slow mapping callback. Only cloud
// frames are skipped: the synchronizer must retain every intervening IMU.
template<typename ScanEnd>
inline std::size_t latest_covered_scan(
  std::size_t count, double imu_end, ScanEnd scan_end)
{
  std::size_t selected = 0;
  for (std::size_t i = 1; i < count; ++i) {
    const double end = scan_end(i);
    if (!std::isfinite(end) || end > imu_end) {break;}
    selected = i;
  }
  return selected;
}

struct ImuSample
{
  double stamp;
  std::array<double, 6> values;
};

inline const char * input_error(
  double begin, double end, double now, const std::vector<ImuSample> & imu,
  double previous_imu, double max_age, double max_gap, double max_scan_duration)
{
  if (!std::isfinite(begin) || !std::isfinite(end) || end <= begin ||
    end - begin > max_scan_duration)
  {
    return "invalid LiDAR scan duration";
  }
  if (now - end > max_age || end - now > 0.1) {
    return "stale or future LiDAR timestamp";
  }
  if (imu.empty()) {return "missing IMU coverage";}
  if (imu.front().stamp > begin + max_gap || end - imu.back().stamp > max_gap) {
    return "IMU does not cover LiDAR scan";
  }
  for (const auto & sample : imu) {
    if (!std::isfinite(sample.stamp)) {return "invalid IMU timestamp";}
    for (double value : sample.values) {
      if (!std::isfinite(value)) {return "non-finite IMU measurement";}
    }
    if (previous_imu > 0.0 &&
      (sample.stamp <= previous_imu || sample.stamp - previous_imu > max_gap))
    {
      return "IMU timestamp discontinuity";
    }
    previous_imu = sample.stamp;
  }
  return nullptr;
}

inline bool reliable_match(
  int matched, int total, double residual, int minimum, double min_ratio, double max_residual)
{
  return total > 0 && matched >= minimum &&
         static_cast<double>(matched) / total >= min_ratio &&
         std::isfinite(residual) && residual <= max_residual;
}
}  // namespace fast_lio_guard
#endif  // FAST_LIO_FRONTEND_GUARD_HPP_
