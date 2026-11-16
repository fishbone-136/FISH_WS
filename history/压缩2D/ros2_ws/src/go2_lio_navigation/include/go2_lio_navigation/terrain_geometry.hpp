#pragma once

#include <cmath>
#include <algorithm>
#include <vector>

namespace go2_lio_navigation
{
// A rolling window may move by whole cells, but world cell boundaries must
// not slide with gait/odometry jitter. Local 8 cm cells then also align with
// the 4 cm and 8 cm navigation overlays.
inline double aligned_grid_origin(double center, int cells, double resolution)
{
  return std::floor((center - 0.5 * cells * resolution) / resolution + 1.e-9) * resolution;
}

// Estimate a surface over a physical neighbourhood, rather than turning one
// noisy 8 cm height difference into a 30 degree slope. Steps and vertical
// extents are checked separately and must never be smoothed away here.
template<class Cell>
double surface_slope(const std::vector<Cell> & cells, int width, int height,
  int x, int y, double resolution, double baseline, int minimum_points)
{
  const int radius = std::max(1, static_cast<int>(std::ceil(baseline / resolution)));
  double n=0., sx=0., sy=0., sz=0., sxx=0., syy=0., sxy=0., sxz=0., syz=0.;
  for (int ny=std::max(0,y-radius); ny<=std::min(height-1,y+radius); ++ny) {
    for (int nx=std::max(0,x-radius); nx<=std::min(width-1,x+radius); ++nx) {
      const auto & cell=cells[ny*width+nx];
      if (cell.count<static_cast<std::size_t>(minimum_points)) {continue;}
      const double dx=(nx-x)*resolution, dy=(ny-y)*resolution, z=cell.minimum;
      n+=1.;sx+=dx;sy+=dy;sz+=z;sxx+=dx*dx;syy+=dy*dy;sxy+=dx*dy;
      sxz+=dx*z;syz+=dy*z;
    }
  }
  if (n<6.) {return 0.;}
  const double xx=sxx-sx*sx/n, yy=syy-sy*sy/n, xy=sxy-sx*sy/n;
  const double xz=sxz-sx*sz/n, yz=syz-sy*sz/n, determinant=xx*yy-xy*xy;
  if (std::min(xx,yy)/n<baseline*baseline/16. || determinant<=1.e-12) {return 0.;}
  return std::atan(std::hypot((xz*yy-yz*xy)/determinant,
    (yz*xx-xz*xy)/determinant))*180./3.14159265358979323846;
}
}  // namespace go2_lio_navigation
