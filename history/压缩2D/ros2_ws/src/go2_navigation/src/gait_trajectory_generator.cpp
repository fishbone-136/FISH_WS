// Copyright 2026 Go2 Navigation Contributors
// SPDX-License-Identifier: Apache-2.0

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "dwb_plugins/standard_traj_generator.hpp"
#include "dwb_plugins/xy_theta_iterator.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace go2_navigation
{

class GaitVelocityIterator : public dwb_plugins::XYThetaIterator
{
public:
  explicit GaitVelocityIterator(double minimum) : minimum_(minimum) {}

protected:
  bool isValidVelocity() override
  {
    if (!dwb_plugins::XYThetaIterator::isValidVelocity()) {return false;}
    const double translation = std::hypot(x_it_->getVelocity(), y_it_->getVelocity());
    // Humble's XYThetaIterator accepts min_speed_xy OR min_speed_theta.
    // Consequently a large yaw used to admit unusable 0.055 m/s walking
    // samples. Keep true in-place turns and ordinary walking arcs only.
    return translation < 1.e-5 || translation + 1.e-5 >= minimum_;
  }

private:
  double minimum_;
};

class GaitTrajectoryGenerator : public dwb_plugins::StandardTrajectoryGenerator
{
public:
  void initialize(
    const nav2_util::LifecycleNode::SharedPtr & node, const std::string & name) override
  {
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".gait_min_translation", rclcpp::ParameterValue(0.22));
    minimum_ = node->get_parameter(name + ".gait_min_translation").as_double();
    if (!std::isfinite(minimum_) || minimum_ <= 0.0) {
      throw std::runtime_error("Invalid gait minimum translation");
    }
    StandardTrajectoryGenerator::initialize(node, name);
  }

protected:
  void initializeIterator(const nav2_util::LifecycleNode::SharedPtr & node) override
  {
    velocity_iterator_ = std::make_shared<GaitVelocityIterator>(minimum_);
    velocity_iterator_->initialize(node, kinematics_handler_, plugin_name_);
  }

private:
  double minimum_{0.22};
};
}  // namespace go2_navigation

PLUGINLIB_EXPORT_CLASS(go2_navigation::GaitTrajectoryGenerator, dwb_core::TrajectoryGenerator)
