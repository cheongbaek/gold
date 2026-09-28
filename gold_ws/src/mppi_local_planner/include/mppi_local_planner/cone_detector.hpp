#pragma once
#include <cmath>
#include <vector>

#include "mppi_local_planner/ego_costmap.hpp"
#include "mppi_local_planner/frenet_types.hpp"

namespace mppi_local_planner
{

// Cone centroid in the ego frame (x forward, y left).
struct ConeObs
{
  double x = 0.0;
  double y = 0.0;
  int cells = 0;
};

struct ConeStats
{
  int lethal_cells = 0;
  int clusters_all = 0;
  int clusters_rejected = 0;
};

struct ConeDetectParams
{
  int min_cells = 20;
  // Ground sheets are wide; a cone's lethal blob is ~0.9 m across.
  double max_span_m = 2.00;
};

// Cluster lethal costmap cells. Centroid ≈ cone centre (inflation is not lethal).
std::vector<ConeObs> detectCones(
  const CostmapSnapshot & snap,
  const ConeDetectParams & params,
  ConeStats * stats = nullptr);

// Ahead of pass_x and inside the mapped lane in reference-d.
// Do not require ego |y| ≤ y_max: after a heading swing the next cone's
// ego-y exceeds 2 m while its d is still in-lane.
inline bool coneInAvoidLane(
  const OdomPose & odom,
  const ConeObs & c,
  double pass_x,
  double range,
  double y_max,
  double ego_y_loose = 5.0)
{
  if (c.x < pass_x || c.x > range) {
    return false;
  }
  if (std::abs(c.y) > ego_y_loose) {
    return false;
  }
  double s = 0.0;
  double d = 0.0;
  egoToOdom(odom, c.x, c.y, s, d);
  (void)s;
  return std::abs(d) <= y_max;
}

}  // namespace mppi_local_planner
