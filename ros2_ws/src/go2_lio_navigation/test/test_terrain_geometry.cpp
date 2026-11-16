#include <cmath>
#include <vector>
#include "gtest/gtest.h"
#include "go2_lio_navigation/terrain_geometry.hpp"

TEST(TerrainGrid, StaticWorldPointKeepsSameCellAcrossRobotJitter)
{
  for (double resolution : {0.04, 0.08, 0.10}) {
    for (double point : {-3.193, -0.031, 0.019, 2.413}) {
      const double expected = (std::floor(point / resolution) + 0.5) * resolution;
      for (int i = -25; i <= 25; ++i) {
        double origin = go2_lio_navigation::aligned_grid_origin(i * .001, 250, resolution);
        double index = std::floor((point - origin) / resolution);
        EXPECT_NEAR(origin + (index + .5) * resolution, expected, 1.e-10);
      }
    }
  }
}

TEST(TerrainGrid, NavigationRefinementSharesWorldBoundaries)
{
  for (double center : {-2.831, -.001, 0., .037, 5.279}) {
    double origin = go2_lio_navigation::aligned_grid_origin(center, 125, .08);
    EXPECT_NEAR(origin / .04, std::round(origin / .04), 1.e-10);
    EXPECT_NEAR(origin / .08, std::round(origin / .08), 1.e-10);
    EXPECT_LE(origin, center - 5. + 1.e-9);
    EXPECT_GT(origin + 10., center + 4.92 - 1.e-9);
  }
}

namespace {
struct SurfaceCell {std::size_t count{2};double minimum{0.};};
}

TEST(TerrainSlope, IsolatedFiveCentimetreNoiseDoesNotMakeFlatFloorLethal)
{
  std::vector<SurfaceCell> cells(81);
  cells[4*9+4].minimum=.05;
  for(int y=1;y<8;++y)for(int x=1;x<8;++x) {
    EXPECT_LT(go2_lio_navigation::surface_slope(cells,9,9,x,y,.08,.24,2),10.);
  }
}

TEST(TerrainSlope, MeasuresTrueInclinesAndRejectsCollinearSupport)
{
  for(double degrees:{15.,35.,45.}) {
    std::vector<SurfaceCell> cells(81);
    for(int y=0;y<9;++y)for(int x=0;x<9;++x) {
      cells[y*9+x].minimum=.08*x*std::tan(degrees*3.14159265358979323846/180.);
    }
    EXPECT_NEAR(go2_lio_navigation::surface_slope(cells,9,9,4,4,.08,.24,2),degrees,1.e-8);
    for(int y=0;y<9;++y)for(int x=0;x<9;++x)if(y!=4)cells[y*9+x].count=0;
    EXPECT_EQ(go2_lio_navigation::surface_slope(cells,9,9,4,4,.08,.24,2),0.);
  }
}
