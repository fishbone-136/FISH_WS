#ifndef POINTCLOUD_TO_LASERSCAN__GROUND_PLANE_HPP_
#define POINTCLOUD_TO_LASERSCAN__GROUND_PLANE_HPP_

#include <algorithm>
#include <cmath>
#include <random>
#include <map>
#include <deque>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace pointcloud_to_laserscan
{
struct GroundPlane
{
  Eigen::Vector3d normal{0.0, 0.0, 1.0};
  double offset{0.0};
  bool valid{false};
  double height(const Eigen::Vector3d & point) const {return normal.dot(point) + offset;}
};

inline bool is_body_ground(const GroundPlane & plane)
{
  if (!plane.valid || !plane.normal.allFinite() || plane.normal.z() < .9063) {return false;}
  const double height = -plane.offset / plane.normal.z();
  return height >= -1.4 && height <= -.12;
}

// Bounded near-field RANSAC: only downward, approximately horizontal planes
// with 2-D spatial support can be ground. Never classify a vertical wall as floor.
inline GroundPlane estimate_ground(const std::vector<Eigen::Vector3d> & input)
{
  std::vector<Eigen::Vector3d> points;
  // Score independent low surface patches, not the density of wall/box returns.
  std::map<std::pair<int,int>,Eigen::Vector3d> lower;
  const std::size_t stride = std::max<std::size_t>(1, input.size() / 1600);
  for (std::size_t i = 0; i < input.size(); i += stride) {
    const auto & p = input[i];
    const double range = p.head<2>().norm();
    if (p.allFinite() && range > 0.45 && range < 4.5 && p.z() < 0.50 && p.z() > -1.5) {
      const auto key=std::make_pair(static_cast<int>(std::floor(p.x()/.12)),static_cast<int>(std::floor(p.y()/.12)));
      auto old=lower.find(key);
      if(old==lower.end() || p.z()<old->second.z())lower[key]=p;
    }
  }
  for (const auto & item:lower) {points.push_back(item.second);}
  GroundPlane best;
  if (points.size() < 40) {return best;}
  std::mt19937 random(17);
  std::uniform_int_distribution<std::size_t> choose(0, points.size() - 1);
  std::size_t best_count = 0;
  for (int iteration = 0; iteration < 180; ++iteration) {
    const auto & a = points[choose(random)];
    Eigen::Vector3d normal =
      (points[choose(random)] - a).cross(points[choose(random)] - a);
    if (normal.norm() < 1.e-5) {continue;}
    normal.normalize();
    if (normal.z() < 0) {normal = -normal;}
    const double offset = -normal.dot(a);
    if (!is_body_ground({normal, offset, true})) {continue;}
    std::size_t count = 0;
    std::size_t below=0;
    for (const auto & p : points) {const double h=normal.dot(p)+offset;count+=std::abs(h)<.025;below+=h<-.06;}
    // Ground cannot pass substantially above the measured lower surface.
    if(below>std::max<std::size_t>(3,points.size()/20))continue;
    if (count > best_count) {
      best_count = count;
      best = {normal, offset, true};
    }
  }
  if (best_count < 40) {return GroundPlane{};}
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto & p : points) {
    if (std::abs(best.height(p)) < 0.025) {mean += p;}
  }
  mean /= static_cast<double>(best_count);
  for (const auto & p : points) {
    if (std::abs(best.height(p)) < 0.025) {
      covariance += (p - mean) * (p - mean).transpose();
    }
  }
  covariance /= static_cast<double>(best_count);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success || solver.eigenvalues()[1] < 0.01) {
    return GroundPlane{};
  }
  Eigen::Vector3d normal = solver.eigenvectors().col(0);
  if (normal.z() < 0) {normal = -normal;}
  best.normal = normal;
  best.offset = -normal.dot(mean);
  return is_body_ground(best) ? best : GroundPlane{};
}
// A single wrong floor plane must never become persistent obstacle evidence.
// Bootstrap from repeated compatible fits; retain a recent good fit through a
// sparse HAP frame, and fail closed when no stable model remains fresh.
class GroundTracker
{
public:
  explicit GroundTracker(double smoothing=1.) : smoothing_(smoothing) {}
  GroundPlane observe(const GroundPlane & candidate, double stamp)
  {
    if (!std::isfinite(stamp)) {return GroundPlane{};}
    if (stamp < seen_) {return GroundPlane{};}
    if (stamp == seen_) {return fresh(stamp) ? current_ : GroundPlane{};}
    seen_=stamp;
    while (!history_.empty() && stamp-history_.front().first>.6) {history_.pop_front();}
    if (candidate.valid) {
      history_.emplace_back(stamp,candidate);
      while (history_.size()>8) {history_.pop_front();}
      if (fresh(stamp) && compatible(current_,candidate,.06,8.)) {
        const auto normal=(1.-smoothing_)*current_.normal+smoothing_*candidate.normal;
        const double length=normal.norm();
        current_={normal/length,((1.-smoothing_)*current_.offset+smoothing_*candidate.offset)/length,true};
        accepted_=stamp;
      } else if (!fresh(stamp)) {
        int votes=0;
        for (const auto & entry:history_) {
          votes+=compatible(candidate,entry.second,.04,5.);
        }
        if (votes>=3) {current_=candidate;accepted_=stamp;}
      }
    }
    return fresh(stamp) ? current_ : GroundPlane{};
  }
  bool observation_supported(double stamp) const {return fresh(stamp) && stamp==accepted_;}
  GroundPlane predict(double stamp) const {return fresh(stamp) ? current_ : GroundPlane{};}
  void confirm(double stamp) {if(fresh(stamp) && stamp==seen_)accepted_=stamp;}
private:
  static bool compatible(const GroundPlane & a,const GroundPlane & b,double dz,double degrees)
  {
    return std::abs(a.offset/a.normal.z()-b.offset/b.normal.z())<=dz &&
      a.normal.dot(b.normal)>=std::cos(degrees*3.14159265358979323846/180.);
  }
  bool fresh(double stamp) const {return current_.valid && stamp>=accepted_ && stamp-accepted_<=1.;}
  GroundPlane current_;
  double smoothing_;
  double seen_{-1.e30},accepted_{-1.e30};
  std::deque<std::pair<double,GroundPlane>> history_;
};

inline GroundPlane transform_ground(const GroundPlane & plane,
  const Eigen::Matrix3d & rotation,const Eigen::Vector3d & translation)
{
  if(!plane.valid)return GroundPlane{};
  const Eigen::Vector3d normal=rotation*plane.normal;
  return {normal,plane.offset-normal.dot(translation),true};
}

inline GroundPlane refine_supported_ground(const std::vector<Eigen::Vector3d> & points,
  const GroundPlane & plane)
{
  if(!plane.valid)return GroundPlane{};
  std::size_t count=0;Eigen::Vector3d sum=Eigen::Vector3d::Zero();
  Eigen::Matrix3d square=Eigen::Matrix3d::Zero();
  const std::size_t stride=std::max<std::size_t>(1,points.size()/1600);
  for(std::size_t i=0;i<points.size();i+=stride) {
    const auto & p=points[i];const double range=p.head<2>().norm();
    if(!p.allFinite() || range<.45 || range>4.5 || std::abs(plane.height(p))>.025)continue;
    ++count;sum+=p;square+=p*p.transpose();
  }
  if(count<40)return GroundPlane{};
  const Eigen::Vector3d mean=sum/static_cast<double>(count);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solve(square/count-mean*mean.transpose());
  if(solve.info()!=Eigen::Success || solve.eigenvalues()[1]<=.01)return GroundPlane{};
  Eigen::Vector3d normal=solve.eigenvectors().col(0);
  if(normal.dot(plane.normal)<0.)normal=-normal;
  if(normal.dot(plane.normal)<std::cos(8.*3.14159265358979323846/180.))return GroundPlane{};
  const GroundPlane measured{normal,-normal.dot(mean),true};
  return is_body_ground(measured) ? measured : GroundPlane{};
}

// Track floor geometry in continuous odometry, so a body pitch/height change
// during turning is not confused with the ground itself suddenly moving.
class ReferencedGroundTracker
{
public:
  GroundPlane observe(const GroundPlane & body_candidate,double stamp,
    const std::vector<Eigen::Vector3d> & points,const Eigen::Matrix3d & rotation,
    const Eigen::Vector3d & translation)
  {
    const auto previous=tracker_.predict(stamp);
    auto world=tracker_.observe(transform_ground(body_candidate,rotation,translation),stamp);
    auto body=transform_ground(world,rotation.transpose(),-rotation.transpose()*translation);
    auto measured=refine_supported_ground(points,body);
    if(!measured.valid && previous.valid && body_candidate.valid && tracker_.observation_supported(stamp)) {
      const auto prior=transform_ground(previous,rotation.transpose(),-rotation.transpose()*translation);
      // A slightly lagged attitude may tilt the predicted plane away from
      // the whole current FOV. Admit the directly measured plane only when
      // its base-height still agrees; never use a raised tabletop this way.
      if(std::abs(prior.offset/prior.normal.z()-
        body_candidate.offset/body_candidate.normal.z())<.025)
        measured=refine_supported_ground(points,body_candidate);
    }
    supported_=measured.valid;
    // Use current measured geometry for filtering. Smoothing the body plane
    // itself would lag a fast pitch change and could turn flat floor into
    // apparent obstacles even though the world-floor reference was stable.
    if (supported_) {
      tracker_.confirm(stamp);
      return measured;
    }
    return is_body_ground(body) ? body : GroundPlane{};
  }
  bool supported() const {return supported_;}
private:
  GroundTracker tracker_{.15};
  bool supported_{false};
};
}  // namespace pointcloud_to_laserscan
#endif
