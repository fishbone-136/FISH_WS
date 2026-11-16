#include <limits>
#include <vector>
#include "gtest/gtest.h"
#include "frontend_guard.hpp"

namespace
{
std::vector<fast_lio_guard::ImuSample> healthy()
{
  std::vector<fast_lio_guard::ImuSample> samples;
  for (int i = 0; i < 20; ++i) {
    samples.push_back({10.0 + i * 0.005, {0.0, 0.0, 1.0, 0.0, 0.0, 0.0}});
  }
  return samples;
}
const char * check(const std::vector<fast_lio_guard::ImuSample> & samples, double now = 10.14)
{
  return fast_lio_guard::input_error(10.0, 10.1, now, samples, 9.995, 0.3, 0.05, 0.15);
}
}

TEST(InputGuard, AcceptsFreshContinuousDeviceTime)
{
  EXPECT_EQ(check(healthy()), nullptr);
}

TEST(ScanTiming, TimedHapShortFrameDoesNotConsumeNextFramesImu)
{
  const double previous_begin = 10.;
  const double next_begin = 10.01;
  const double end = previous_begin + fast_lio_guard::scan_duration(.009, .083, true);
  EXPECT_LT(end, next_begin);
  EXPECT_GT(previous_begin + fast_lio_guard::scan_duration(.009, .083, false), next_begin);
  EXPECT_DOUBLE_EQ(fast_lio_guard::scan_duration(.066, .083, true), .066);
}
TEST(InputGuard, RejectsStartupBacklogAndFutureData)
{
  EXPECT_NE(check(healthy(), 13.1), nullptr);
  EXPECT_NE(check(healthy(), 9.8), nullptr);
}
TEST(InputGuard, RejectsMissingImuAndInvalidValues)
{
  EXPECT_NE(check({}), nullptr);
  auto samples = healthy();
  samples[5].values[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_NE(check(samples), nullptr);
}
TEST(InputGuard, RejectsGapsAndOutOfOrderSamples)
{
  auto samples = healthy();
  samples.erase(samples.begin() + 3, samples.begin() + 17);
  EXPECT_NE(check(samples), nullptr);
  samples = healthy();
  samples[10].stamp = samples[9].stamp;
  EXPECT_NE(check(samples), nullptr);
}
TEST(InputGuard, RequiresBothEndsOfScanToBeCovered)
{
  auto samples = healthy();
  samples.erase(samples.begin(), samples.begin() + 12);
  EXPECT_NE(check(samples), nullptr);
  samples = healthy();
  samples.resize(5);
  EXPECT_NE(check(samples), nullptr);
}

TEST(ScanBacklog, SelectsNewestCompleteScanAfterTreeBuild)
{
  const std::vector<double> ends{10.10, 10.17, 10.24, 10.31, 10.38, 10.45};
  auto end = [&](std::size_t i) {return ends[i];};
  EXPECT_EQ(fast_lio_guard::latest_covered_scan(ends.size(), 10.46, end), 5u);
  EXPECT_EQ(fast_lio_guard::latest_covered_scan(ends.size(), 10.40, end), 4u);
  EXPECT_EQ(fast_lio_guard::latest_covered_scan(ends.size(), 10.12, end), 0u);
  EXPECT_EQ(fast_lio_guard::latest_covered_scan(1, 10.46, end), 0u);
}

TEST(ScanBacklog, PreservedImuCoversSkippedFramesAndStillRejectsActualLoss)
{
  std::vector<fast_lio_guard::ImuSample> samples;
  for (int i = 0; i < 90; ++i) {
    samples.push_back({10.0 + i * 0.005, {0., 0., 1., 0., 0., 0.}});
  }
  EXPECT_EQ(fast_lio_guard::input_error(
      10.38, 10.45, 10.48, samples, 9.995, .3, .05, .15), nullptr);
  samples.erase(samples.begin() + 5, samples.begin() + 70);
  EXPECT_NE(fast_lio_guard::input_error(
      10.38, 10.45, 10.48, samples, 9.995, .3, .05, .15), nullptr);
}
TEST(MatchGuard, RejectsWeakOrNonfiniteGeometryWithoutRelaxingGoodScans)
{
  using fast_lio_guard::reliable_match;
  EXPECT_TRUE(reliable_match(700, 1000, 0.025, 50, 0.05, 0.15));
  EXPECT_FALSE(reliable_match(1, 1000, 0.001, 50, 0.05, 0.15));
  EXPECT_FALSE(reliable_match(50, 2000, 0.001, 50, 0.05, 0.15));
  EXPECT_FALSE(reliable_match(700, 1000, 0.4, 50, 0.05, 0.15));
  EXPECT_FALSE(reliable_match(700, 1000, std::numeric_limits<double>::quiet_NaN(),
      50, 0.05, 0.15));
}
