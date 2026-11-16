// Copyright 2026 Go2 Navigation Contributors
// SPDX-License-Identifier: Apache-2.0

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

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
  void startNewIteration(const nav_2d_msgs::msg::Twist2D & current) override
  {
    StandardTrajectoryGenerator::startNewIteration(current);
    samples_.clear(); next_ = 0;
    while (StandardTrajectoryGenerator::hasMoreTwists()) {
      samples_.push_back(StandardTrajectoryGenerator::nextTwist());
    }
    // A uniform 0..0.65 grid first reaches the walking range at 0.26.
    // Near obstacles the critic caps translation at exactly 0.26, leaving
    // only one walking radius. Include the gait boundary and intermediate
    // radii while retaining the original samples and all collision checks.
    auto k = kinematics_handler_->getKinematics();
    if (std::abs(k.getMinY()) > 1.e-5 || std::abs(k.getMaxY()) > 1.e-5) {return;}
    const double low = dwb_plugins::projectVelocity(current.x, k.getAccX(),
      k.getDecelX(), sim_time_, k.getMinX());
    const double high = dwb_plugins::projectVelocity(current.x, k.getAccX(),
      k.getDecelX(), sim_time_, k.getMaxX());
    const auto original = samples_;
    for (double x : {minimum_, minimum_ + 0.02, minimum_ + 0.04}) {
      if (x < low - 1.e-5 || x > high + 1.e-5 ||
        (k.getMaxSpeedXY() >= 0.0 && x > k.getMaxSpeedXY() + 1.e-5)) {continue;}
      for (const auto & base : original) {
        if (std::abs(base.y) > 1.e-5) {continue;}
        bool duplicate = false;
        for (const auto & v : samples_) {
          if (std::abs(v.x-x)<1.e-5 && std::abs(v.y)<1.e-5 &&
            std::abs(v.theta-base.theta)<1.e-5) {duplicate = true; break;}
        }
        if (!duplicate) {auto v = base; v.x = x; samples_.push_back(v);}
      }
    }
  }

  bool hasMoreTwists() override {return next_ < samples_.size();}
  nav_2d_msgs::msg::Twist2D nextTwist() override {return samples_.at(next_++);}

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
  std::vector<nav_2d_msgs::msg::Twist2D> samples_;
  std::size_t next_{0};
};
}  // namespace go2_navigation

PLUGINLIB_EXPORT_CLASS(go2_navigation::GaitTrajectoryGenerator, dwb_core::TrajectoryGenerator)
