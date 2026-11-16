#ifndef FAST_LIO_STATIONARY_UPDATE_HPP_
#define FAST_LIO_STATIONARY_UPDATE_HPP_

namespace fast_lio_guard
{
// A position/zero-velocity measurement, using IKFoM's manifold update and
// covariance transport rather than overwriting the output odometry.
template<typename Filter>
void constrain_stationary(Filter &filter, const Eigen::Vector3d &anchor)
{
  MTK::vect<6, double> observation;
  for (int i = 0; i < 3; ++i) {
    observation[i] = anchor[i];
    observation[i + 3] = 0.;
  }
  filter.update_iterated_dyn_runtime_share(observation,
    [](state_ikfom &state, esekfom::dyn_runtime_share_datastruct<double> &data) {
      data.h_x = Eigen::MatrixXd::Zero(6, state_ikfom::DOF);
      data.h_x.template block<3, 3>(0, 0).setIdentity();
      data.h_x.template block<3, 3>(3, 12).setIdentity();
      data.h_v = Eigen::MatrixXd::Identity(6, 6);
      data.R = Eigen::MatrixXd::Identity(6, 6) * .0004;
      MTK::vect<6, double> predicted;
      for (int i = 0; i < 3; ++i) {
        predicted[i] = state.pos[i];
        predicted[i + 3] = state.vel[i];
      }
      return predicted;
    });
}
}  // namespace fast_lio_guard
#endif
