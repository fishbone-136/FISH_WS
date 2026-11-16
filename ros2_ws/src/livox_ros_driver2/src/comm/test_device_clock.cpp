// SPDX-License-Identifier: MIT
#include <cassert>
#include <iostream>
#include "device_clock.h"

int main() {
  using livox_ros::DeviceClock;
  constexpr uint64_t epoch = 1700000000000000000ULL;
  constexpr uint64_t boot = 30000000000ULL;
  DeviceClock clock;
  // Startup backlog must not set a permanent future epoch.
  clock.Map(boot, epoch + 400000000ULL, 0);
  clock.Map(boot + 500000000ULL, epoch + 500000000ULL, 100000000ULL);
  auto first = clock.Map(boot + 2000000000ULL, epoch + 2050000000ULL, 2000000000ULL);
  auto second = clock.Map(boot + 2005000000ULL, epoch + 2200000000ULL, 2150000000ULL);
  assert(first == epoch + 2000000000ULL);
  assert(second - first == 5000000ULL);  // 150 ms delivery jitter, 5 ms sampling.
  // Duplicate, lost and out-of-order samples remain visible to FAST-LIO.
  assert(clock.Map(boot + 2005000000ULL, epoch + 2300000000ULL, 2200000000ULL) == second);
  auto gap = clock.Map(boot + 2070000000ULL, epoch + 2400000000ULL, 2300000000ULL);
  assert(gap - second == 65000000ULL);
  assert(clock.Map(boot + 2060000000ULL, epoch + 2500000000ULL, 2400000000ULL) < gap);
  DeviceClock other;
  assert(other.Map(1000000000ULL, epoch, 0) == epoch);
  assert(other.Map(1005000000ULL, epoch + 15000000ULL, 2000000000ULL) == epoch + 5000000ULL);
  std::cout << "PASS shared device sampling intervals, startup calibration, jitter, "
               "duplicates, real gaps, regressions and independent devices\n";
}
