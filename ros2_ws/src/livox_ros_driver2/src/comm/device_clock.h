// SPDX-License-Identifier: MIT
#ifndef LIVOX_DEVICE_CLOCK_H_
#define LIVOX_DEVICE_CLOCK_H_

#include <algorithm>
#include <cstdint>

namespace livox_ros {
// Both IMU and LiDAR use the same boot clock. Estimate its epoch during the
// first second, then freeze it so delivery/scheduling jitter never changes
// sampling intervals. Never clamp duplicates, gaps or clock regressions.
class DeviceClock {
 public:
  uint64_t Map(uint64_t device_ns, uint64_t receipt_ns, uint64_t steady_ns) {
    const int64_t candidate = static_cast<int64_t>(receipt_ns) -
        static_cast<int64_t>(device_ns);
    if (!initialized_) {
      initialized_ = true;
      started_ = steady_ns;
      offset_ = candidate;
    } else if (steady_ns - started_ < 1000000000ULL) {
      offset_ = std::min(offset_, candidate);
    }
    return static_cast<uint64_t>(static_cast<int64_t>(device_ns) + offset_);
  }

 private:
  bool initialized_{false};
  uint64_t started_{0};
  int64_t offset_{0};
};
}  // namespace livox_ros
#endif
