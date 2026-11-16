#include "gtest/gtest.h"
#include <Eigen/Eigenvalues>
#include "use-ikfom.hpp"
#include "stationary_update.hpp"

namespace
{
void no_lidar(state_ikfom &, esekfom::dyn_share_datastruct<double> &) {}
}

TEST(StationaryUpdate, CorrectsStateAndCovarianceInsteadOfClampingOdometry)
{
  esekfom::esekf<state_ikfom, 12, input_ikfom> filter;
  double limits[23];
  std::fill_n(limits, 23, .001);
  filter.init_dyn_share(get_f, df_dx, df_dw, no_lidar, 3, limits);
  auto state = filter.get_x();
  state.pos = Eigen::Vector3d(.08, -.03, .05);
  state.vel = Eigen::Vector3d(.2, .1, -.1);
  filter.change_x(state);
  auto covariance = filter.get_P();
  fast_lio_guard::constrain_stationary(filter, Eigen::Vector3d::Zero());
  EXPECT_LT(filter.get_x().pos.norm(), .001);
  EXPECT_LT(filter.get_x().vel.norm(), .001);
  EXPECT_LT(filter.get_P()(12, 12), covariance(12, 12));
  EXPECT_TRUE(filter.get_P().allFinite());
  EXPECT_LT((filter.get_P() - filter.get_P().transpose()).norm(), 1.e-10);
  Eigen::SelfAdjointEigenSolver<decltype(covariance)> eigen(filter.get_P());
  EXPECT_GE(eigen.eigenvalues().minCoeff(), -1.e-10);
}

TEST(StationaryUpdate, BoundsThirtySecondsOfStaticImuBias)
{
  esekfom::esekf<state_ikfom, 12, input_ikfom> filter;
  double limits[23];
  std::fill_n(limits, 23, .001);
  filter.init_dyn_share(get_f, df_dx, df_dw, no_lidar, 3, limits);
  auto state = filter.get_x();
  state.grav = S2(Eigen::Vector3d(0., 0., -9.81));
  filter.change_x(state);
  input_ikfom input;
  input.acc = Eigen::Vector3d(.15, -.1, 9.85);
  input.gyro = Eigen::Vector3d::Zero();
  auto noise = process_noise_cov();
  for (int i = 0; i < 450; ++i) {
    double dt = 1. / 15.;
    filter.predict(dt, noise, input);
    fast_lio_guard::constrain_stationary(filter, Eigen::Vector3d::Zero());
    ASSERT_TRUE(filter.get_x().pos.allFinite());
    EXPECT_LT(filter.get_x().pos.norm(), .02);
    EXPECT_LT(filter.get_x().vel.norm(), .05);
  }
}
