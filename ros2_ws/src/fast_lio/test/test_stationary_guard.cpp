#include "gtest/gtest.h"
#include "stationary_guard.hpp"

namespace
{
void idle(fast_lio_guard::StationaryGuard &g, double t)
{
  g.command(t, 0., 0., 0.);
  g.feedback(t, {0., 0., .3}, {0., 0., 0.}, 0., 0.);
}
bool settled(fast_lio_guard::StationaryGuard &g)
{
  for (int i = 0; i <= 20; ++i) {
    idle(g, i * .05);
    g.stationary(i * .05, true);
  }
  return g.stationary(1., true);
}
}

TEST(StationaryGuard, RequiresSustainedIndependentEvidence)
{
  fast_lio_guard::StationaryGuard g;
  EXPECT_FALSE(g.stationary(0., true));
  idle(g, 0.);
  EXPECT_FALSE(g.stationary(0., true));
  EXPECT_TRUE(settled(g));
}

TEST(StationaryGuard, ReleasesOnCommandFeedbackLossAndImuMotion)
{
  fast_lio_guard::StationaryGuard g;
  ASSERT_TRUE(settled(g));
  g.command(1.01, .001, 0., 0.);
  EXPECT_FALSE(g.stationary(1.01, true));
  fast_lio_guard::StationaryGuard h;
  ASSERT_TRUE(settled(h));
  EXPECT_FALSE(h.stationary(1.3, true));
  fast_lio_guard::StationaryGuard k;
  ASSERT_TRUE(settled(k));
  EXPECT_FALSE(k.stationary(1., false));
  idle(k, 1.1);
  EXPECT_FALSE(k.stationary(1.1, true));
}

TEST(StationaryGuard, LegDisplacementAndRotationInvalidateAnchor)
{
  fast_lio_guard::StationaryGuard g;
  ASSERT_TRUE(settled(g));
  g.feedback(1.01, {.03, 0., .3}, {0., 0., 0.}, 0., 0.);
  EXPECT_FALSE(g.stationary(1.01, true));
  fast_lio_guard::StationaryGuard h;
  ASSERT_TRUE(settled(h));
  h.feedback(1.01, {0., 0., .3}, {0., .04, 0.}, 0., 0.);
  EXPECT_FALSE(h.stationary(1.01, true));
}

TEST(StationaryGuard, RejectsNonfiniteAndMovingFeedback)
{
  fast_lio_guard::StationaryGuard g;
  idle(g, 0.);
  g.feedback(.1, {0., 0., .3}, {0., 0., 0.}, .1, 0.);
  EXPECT_FALSE(g.stationary(.1, true));
  g.command(.2, std::numeric_limits<double>::quiet_NaN(), 0., 0.);
  EXPECT_FALSE(g.stationary(.2, true));
}
