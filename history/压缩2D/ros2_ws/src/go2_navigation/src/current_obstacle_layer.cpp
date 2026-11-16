#include <algorithm>
#include <limits>
#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace go2_navigation
{
// Independent obstacle_memory owns persistent occupied geometry in LIO mode.
// This layer contributes only the latest scan. Otherwise ObstacleLayer keeps
// its own second history, even after the upstream memory proved a volume free.
class CurrentObstacleLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  void updateBounds(double x,double y,double yaw,double * min_x,double * min_y,
    double * max_x,double * max_y) override
  {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
    resetMaps();
    double lo_x=std::numeric_limits<double>::max(),lo_y=lo_x;
    double hi_x=std::numeric_limits<double>::lowest(),hi_y=hi_x;
    nav2_costmap_2d::ObstacleLayer::updateBounds(x,y,yaw,&lo_x,&lo_y,&hi_x,&hi_y);
    // Updating the previous bounds makes the master rebuild vacated cells
    // from current terrain + persistent memory, including inflation halos.
    *min_x=std::min({*min_x,lo_x,previous_min_x_});
    *min_y=std::min({*min_y,lo_y,previous_min_y_});
    *max_x=std::max({*max_x,hi_x,previous_max_x_});
    *max_y=std::max({*max_y,hi_y,previous_max_y_});
    previous_min_x_=lo_x;previous_min_y_=lo_y;
    previous_max_x_=hi_x;previous_max_y_=hi_y;
  }
private:
  double previous_min_x_{std::numeric_limits<double>::max()};
  double previous_min_y_{std::numeric_limits<double>::max()};
  double previous_max_x_{std::numeric_limits<double>::lowest()};
  double previous_max_y_{std::numeric_limits<double>::lowest()};
};
}
PLUGINLIB_EXPORT_CLASS(go2_navigation::CurrentObstacleLayer,nav2_costmap_2d::Layer)
