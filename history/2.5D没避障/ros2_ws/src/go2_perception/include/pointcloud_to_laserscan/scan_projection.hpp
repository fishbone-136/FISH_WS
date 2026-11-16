#ifndef POINTCLOUD_TO_LASERSCAN__SCAN_PROJECTION_HPP_
#define POINTCLOUD_TO_LASERSCAN__SCAN_PROJECTION_HPP_
#include <cmath>
#include <Eigen/Geometry>
namespace pointcloud_to_laserscan
{
// Positive XYZ remains in the physical body frame. Only the 2-D scan uses
// Nav2's yaw-only frame, so scan cells and full-pose terrain/memory coincide.
inline Eigen::Matrix3d planar_from_body(const Eigen::Matrix3d & world_from_body)
{
  const double yaw=std::atan2(world_from_body(1,0),world_from_body(0,0));
  return Eigen::AngleAxisd(-yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix()*world_from_body;
}
}
#endif
