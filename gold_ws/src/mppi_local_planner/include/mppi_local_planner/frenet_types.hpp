#pragma once
#include <cmath>
#include <vector>

#include "mppi_local_planner/vehicle_model.hpp"

namespace mppi_local_planner
{

// Persistent frame after L-zone rearm: x along the reference heading (s),
// y left of that line (d, GPS CTE), yaw relative to the line.
struct OdomPose
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

inline void egoToOdom(
  const OdomPose & o, double ex, double ey, double & s, double & d)
{
  const double c = std::cos(o.yaw);
  const double sn = std::sin(o.yaw);
  s = o.x + ex * c - ey * sn;
  d = o.y + ex * sn + ey * c;
}

inline void odomToEgo(
  const OdomPose & o, double s, double d, double & ex, double & ey)
{
  const double c = std::cos(o.yaw);
  const double sn = std::sin(o.yaw);
  const double ds = s - o.x;
  const double dd = d - o.y;
  ex = ds * c + dd * sn;
  ey = -ds * sn + dd * c;
}

struct FrenetPoint
{
  double s = 0.0;      // [m] along reference
  double d = 0.0;      // [m] left of reference
  double d_s = 0.0;    // dd/ds
  double yaw = 0.0;    // [rad] atan(d_s), relative to reference
  double kappa = 0.0;  // [1/m] path curvature
};

// Sampled d(s) through slalom gates. s is strictly increasing.
struct FrenetPath
{
  std::vector<FrenetPoint> pts;
  bool valid = false;
  int n_gates = 0;
  double target_d = 0.0;  // d at current s — diag field [9]

  FrenetPoint sample(double s) const
  {
    if (pts.empty()) {
      return {};
    }
    if (s <= pts.front().s) {
      return pts.front();
    }
    if (s >= pts.back().s) {
      return pts.back();
    }
    // N is ~60; linear scan is cheaper than a bisect setup.
    for (size_t i = 1; i < pts.size(); ++i) {
      if (s <= pts[i].s) {
        const FrenetPoint & a = pts[i - 1];
        const FrenetPoint & b = pts[i];
        const double span = b.s - a.s;
        const double t = (span > 1e-9) ? (s - a.s) / span : 0.0;
        FrenetPoint p;
        p.s = s;
        p.d = a.d + t * (b.d - a.d);
        p.d_s = a.d_s + t * (b.d_s - a.d_s);
        p.yaw = wrapAngle(a.yaw + t * wrapAngle(b.yaw - a.yaw));
        p.kappa = a.kappa + t * (b.kappa - a.kappa);
        return p;
      }
    }
    return pts.back();
  }
};

}  // namespace mppi_local_planner
