#include "mppi_local_planner/frenet_planner.hpp"

#include <algorithm>
#include <cmath>

namespace mppi_local_planner
{

FrenetPlanner::FrenetPlanner(
  const FrenetPlannerParams & params, const VehicleParams & vehicle)
: params_(params), vehicle_(vehicle)
{
}

void FrenetPlanner::reset()
{
  last_stats_ = {};
  last_cones_ego_.clear();
  last_path_ = {};
  mem_.clear();
  lost_n_ = 0;
  held_gate_s_ = 0.0;
  held_last_s_ = 0.0;
}

double FrenetPlanner::laneMaxOffset() const
{
  double max_off = std::max(0.2, params_.max_offset_m);
  if (params_.lane_half_m > 0.5) {
    const double body = 0.5 * vehicle_.track_width + params_.lane_margin_m;
    max_off = std::min(max_off, std::max(0.35, params_.lane_half_m - body));
  }
  return max_off;
}

std::vector<FrenetPlanner::Gate> FrenetPlanner::makeGates(
  const OdomPose & odom, const std::vector<ConeObs> & cones)
{
  const double half_w = 0.5 * vehicle_.track_width;
  const double clear =
    half_w + std::max(params_.cone_half_m, params_.inflation_m) + params_.margin_m;
  const double max_off = laneMaxOffset();

  std::vector<Gate> gates;
  gates.reserve(cones.size());

  struct Hit
  {
    double s = 0.0;
    double d = 0.0;
    double ego_y = 0.0;
  };
  std::vector<Hit> hits;
  hits.reserve(cones.size());
  for (const auto & c : cones) {
    if (!coneInAvoidLane(
          odom, c, -params_.keep_behind_m, params_.range_m, params_.cone_y_max_m))
    {
      continue;
    }
    Hit h;
    h.ego_y = c.y;
    egoToOdom(odom, c.x, c.y, h.s, h.d);
    hits.push_back(h);
  }
  std::sort(hits.begin(), hits.end(),
            [](const Hit & a, const Hit & b) { return a.s < b.s; });

  auto side_from_ego = [](double ego_y) -> int {
    return (ego_y > 0.0) ? -1 : +1;
  };
  auto pick_side = [&](const Hit & h, int locked, int prev) -> int {
    const int ego_side = side_from_ego(h.ego_y);
    // GPS CTE can flip Frenet d (RTK off). Pass opposite the cone in lidar
    // ego-y so reversing the layout still steers away from the cone.
    if (std::abs(h.ego_y) >= 0.12) {
      if (locked != 0 && locked != ego_side && std::abs(h.ego_y) >= 0.20) {
        return ego_side;
      }
      if (locked != 0) {
        return locked;
      }
      return ego_side;
    }
    if (locked != 0) {
      return locked;
    }
    if (std::abs(h.d) >= 0.12) {
      return (h.d > 0.0) ? -1 : +1;
    }
    if (prev != 0) {
      return -prev;
    }
    return (odom.y >= 0.0) ? +1 : -1;
  };

  int prev_side = 0;
  for (const auto & h : hits) {
    int locked = 0;
    for (const auto & m : mem_) {
      if (std::abs(m.s - h.s) <= params_.new_cone_ds_m) {
        locked = m.side;
        break;
      }
    }
    const int side = pick_side(h, locked, prev_side);
    double pass_d = h.d + static_cast<double>(side) * clear;
    pass_d = std::clamp(pass_d, -max_off, max_off);
    Gate g;
    g.s = h.s;
    g.cone_d = h.d;
    g.pass_d = pass_d;
    g.side = side;
    gates.push_back(g);
    prev_side = side;
  }
  if (!gates.empty()) {
    mem_ = gates;
  }
  return gates;
}

void FrenetPlanner::augmentGatesFromCostmap(
  const OdomPose & odom, const CostmapSnapshot & snap, std::vector<Gate> & gates) const
{
  if (!snap.valid) {
    return;
  }
  const double half_w = 0.5 * vehicle_.track_width;
  const double clear =
    half_w + std::max(params_.cone_half_m, params_.inflation_m) + params_.margin_m;
  const double max_off = laneMaxOffset();
  const double thr = EgoCostmap::kLethalCost * 0.50;
  const double s0 = odom.x + 1.0;
  const double s1 = odom.x + params_.range_m;
  for (double s = s0; s <= s1; s += 0.45) {
    bool near = false;
    for (const auto & g : gates) {
      if (std::abs(g.s - s) < 1.5) {
        near = true;
        break;
      }
    }
    if (near) {
      continue;
    }
    double best_c = 0.0;
    double best_d = 0.0;
    for (double d = -2.0; d <= 2.0; d += 0.12) {
      double ex = 0.0;
      double ey = 0.0;
      odomToEgo(odom, s, d, ex, ey);
      const double c = snap.getCost(ex, ey);
      if (c > best_c) {
        best_c = c;
        best_d = d;
      }
    }
    if (best_c < thr || std::abs(best_d) > params_.cone_y_max_m) {
      continue;
    }
    Gate g;
    g.s = s;
    g.cone_d = best_d;
    g.side = (best_d >= 0.0) ? -1 : +1;
    g.pass_d = std::clamp(best_d + static_cast<double>(g.side) * clear, -max_off, max_off);
    gates.push_back(g);
  }
  std::sort(gates.begin(), gates.end(),
            [](const Gate & a, const Gate & b) { return a.s < b.s; });
}

void FrenetPlanner::applyKeepOut(
  FrenetPath & path, const std::vector<Gate> & gates) const
{
  const double body_clear =
    0.5 * vehicle_.track_width +
    std::max(params_.cone_half_m, params_.inflation_m) + 0.20;
  const double max_off = laneMaxOffset();
  // Only the hold window. Clamping the weave (the old 1.8 m) snapped d and
  // put a kink back in front of every cone.
  const double window = std::max(0.8, params_.hold_near_m) + 0.15;
  for (auto & p : path.pts) {
    for (const auto & g : gates) {
      if (std::abs(p.s - g.s) > window) {
        continue;
      }
      if (g.side < 0) {
        p.d = std::min(p.d, g.cone_d - body_clear);
      } else {
        p.d = std::max(p.d, g.cone_d + body_clear);
      }
      p.d = std::clamp(p.d, -max_off, max_off);
    }
  }
}

FrenetPath FrenetPlanner::buildThroughGates(
  const std::vector<Gate> & ahead, const Knot & start) const
{
  std::vector<Knot> knots;
  knots.push_back(start);
  // Hold only near the cone. The cubic between holds is the weave, and it
  // needs the whole inter-cone gap. arrive_before is the first cone only:
  // using it on every gate (with wrap) collapsed a 5–6 m gap into a kink
  // the 23° steer limit cannot track.
  const double hold = std::max(0.6, params_.hold_near_m);
  const double arrive = std::max(hold + 0.5, params_.arrive_before_m);
  // Later cones open 2.4 m early. Using arrive_before (4.5 m) here eats a
  // 5–6 m gap; using only `hold` (the old code) reaches the offset too late
  // and the body is still crossing when it passes the cone.
  const double later = std::min(arrive, std::max(2.4, hold + 0.6));
  for (size_t i = 0; i < ahead.size(); ++i) {
    const Gate & g = ahead[i];
    double s_hold_end = g.s + hold;
    if (i + 1 < ahead.size()) {
      const double next_in = ahead[i + 1].s - later;
      s_hold_end = std::min(s_hold_end, next_in - 1.2);
      s_hold_end = std::max(s_hold_end, g.s + 0.35);
    }

    double s_reach = (i == 0) ? (g.s - arrive) : (g.s - later);
    s_reach = std::max(s_reach, knots.back().s + 0.8);
    s_reach = std::min(s_reach, std::max(knots.back().s + 0.3, g.s - 0.4));
    // Do not move the start knot onto pass_d. That teleports the axle.
    if (s_reach > knots.back().s + 0.25) {
      knots.push_back({s_reach, g.pass_d, 0.0});
    }
    if (g.s > knots.back().s + 0.25) {
      knots.push_back({g.s, g.pass_d, 0.0});
    }
    if (s_hold_end > knots.back().s + 0.25) {
      knots.push_back({s_hold_end, g.pass_d, 0.0});
    }
  }
  const double Lret = std::max(3.0, params_.return_after_last_m);
  knots.push_back({knots.back().s + Lret, 0.0, 0.0});
  knots.push_back({knots.back().s + 3.0, 0.0, 0.0});
  FrenetPath path = samplePath(knots);
  applyKeepOut(path, ahead);
  finishDerivatives(path);
  path.n_gates = static_cast<int>(ahead.size());
  path.target_d = ahead.front().pass_d;
  path.valid = path.pts.size() >= 2;
  return path;
}

void FrenetPlanner::nudgeOffObstacles(
  FrenetPath & path, const OdomPose & odom, const CostmapSnapshot & snap) const
{
  if (!path.valid || path.pts.empty() || !snap.valid) {
    return;
  }
  const double thr = EgoCostmap::kLethalCost * 0.45;
  const double max_off = laneMaxOffset();
  const double step = 0.06;
  for (auto & p : path.pts) {
    int prefer = 0;
    for (const auto & g : mem_) {
      if (std::abs(p.s - g.s) < 2.5) {
        prefer = g.side;
        break;
      }
    }
    for (int k = 0; k < 28; ++k) {
      double ex = 0.0;
      double ey = 0.0;
      odomToEgo(odom, p.s, p.d, ex, ey);
      const double c0 = snap.getCost(ex, ey);
      if (c0 < thr) {
        break;
      }
      double ex_l = 0.0;
      double ey_l = 0.0;
      double ex_r = 0.0;
      double ey_r = 0.0;
      const double d_l = std::min(max_off, p.d + step);
      const double d_r = std::max(-max_off, p.d - step);
      odomToEgo(odom, p.s, d_l, ex_l, ey_l);
      odomToEgo(odom, p.s, d_r, ex_r, ey_r);
      const double cl = snap.getCost(ex_l, ey_l);
      const double cr = snap.getCost(ex_r, ey_r);
      if (prefer > 0 && d_l != p.d) {
        p.d = d_l;
      } else if (prefer < 0 && d_r != p.d) {
        p.d = d_r;
      } else if (cl <= cr && d_l != p.d) {
        p.d = d_l;
      } else if (d_r != p.d) {
        p.d = d_r;
      } else {
        break;
      }
    }
  }
  finishDerivatives(path);
}

FrenetPath FrenetPlanner::samplePath(const std::vector<Knot> & knots) const
{
  FrenetPath path;
  if (knots.size() < 2) {
    return path;
  }
  const double ds = std::max(0.10, params_.ds_m);
  const double max_off = laneMaxOffset();

  auto push = [&](double s, double d, double d_s) {
    if (!path.pts.empty() && s <= path.pts.back().s + 1e-6) {
      return;
    }
    FrenetPoint p;
    p.s = s;
    p.d = std::clamp(d, -max_off, max_off);
    p.d_s = d_s;
    p.yaw = std::atan(d_s);
    path.pts.push_back(p);
  };

  for (size_t k = 0; k + 1 < knots.size(); ++k) {
    const Knot & a = knots[k];
    const Knot & b = knots[k + 1];
    const double L = b.s - a.s;
    if (L < 1e-4) {
      continue;
    }
    for (double s = a.s; s < b.s - 0.5 * ds; s += ds) {
      const double u = (s - a.s) / L;
      const double u2 = u * u;
      const double u3 = u2 * u;
      const double h00 = 2.0 * u3 - 3.0 * u2 + 1.0;
      const double h10 = u3 - 2.0 * u2 + u;
      const double h01 = -2.0 * u3 + 3.0 * u2;
      const double h11 = u3 - u2;
      const double d = h00 * a.d + h10 * L * a.d_s + h01 * b.d + h11 * L * b.d_s;
      const double dh00 = 6.0 * u2 - 6.0 * u;
      const double dh10 = 3.0 * u2 - 4.0 * u + 1.0;
      const double dh01 = -6.0 * u2 + 6.0 * u;
      const double dh11 = 3.0 * u2 - 2.0 * u;
      const double d_s =
        (dh00 * a.d + dh10 * L * a.d_s + dh01 * b.d + dh11 * L * b.d_s) / L;
      push(s, d, d_s);
    }
    push(b.s, b.d, b.d_s);
  }
  finishDerivatives(path);
  path.valid = path.pts.size() >= 2;
  return path;
}

void FrenetPlanner::finishDerivatives(FrenetPath & path) const
{
  auto & pts = path.pts;
  if (pts.size() < 2) {
    return;
  }
  for (size_t i = 0; i < pts.size(); ++i) {
    const size_t i0 = (i == 0) ? 0 : i - 1;
    const size_t i1 = (i + 1 == pts.size()) ? i : i + 1;
    const double span = pts[i1].s - pts[i0].s;
    if (span > 1e-6) {
      pts[i].d_s = (pts[i1].d - pts[i0].d) / span;
    }
    pts[i].yaw = std::atan(pts[i].d_s);
  }
  for (size_t i = 0; i < pts.size(); ++i) {
    const size_t i0 = (i == 0) ? 0 : i - 1;
    const size_t i1 = (i + 1 == pts.size()) ? i : i + 1;
    const double span = pts[i1].s - pts[i0].s;
    double d_ss = 0.0;
    if (span > 1e-6) {
      d_ss = (pts[i1].d_s - pts[i0].d_s) / span;
    }
    const double g = 1.0 + pts[i].d_s * pts[i].d_s;
    pts[i].kappa = d_ss / std::pow(g, 1.5);
  }
}

FrenetPath FrenetPlanner::plan(const OdomPose & odom, const CostmapSnapshot & snap)
{
  ConeDetectParams dp;
  dp.min_cells = std::max(1, params_.cone_min_cells);
  last_cones_ego_ = detectCones(snap, dp, &last_stats_);

  FrenetPath centre;
  {
    Knot a{odom.x, odom.y, std::tan(std::clamp(odom.yaw, -0.7, 0.7))};
    Knot b{odom.x + params_.plan_distance_m, 0.0, 0.0};
    centre = samplePath({a, b});
    centre.n_gates = 0;
    centre.target_d = odom.y;
    centre.valid = true;
  }

  if (!params_.enable) {
    last_path_ = centre;
    return last_path_;
  }

  std::vector<Gate> gates = makeGates(odom, last_cones_ego_);
  // A cost blob beside a real cone becomes a second gate and kinks the weave.
  // Use the costmap only when clustering found nothing.
  if (gates.empty()) {
    augmentGatesFromCostmap(odom, snap, gates);
  }
  if (!gates.empty()) {
    mem_ = gates;
  }
  if (gates.empty()) {
    if (!mem_.empty() && ++lost_n_ < std::max(1, params_.cone_lost_ticks)) {
      gates = mem_;
    } else if (last_path_.valid && last_path_.n_gates > 0 &&
               odom.x < held_gate_s_ + std::max(1.2, params_.wrap_after_m))
    {
      // Detection dropped while still wrapping — do not return into the cone.
      last_path_.target_d = last_path_.sample(odom.x).d;
      return last_path_;
    } else {
      lost_n_ = 0;
      mem_.clear();
      held_gate_s_ = 0.0;
      held_last_s_ = 0.0;
      last_path_ = centre;
      return last_path_;
    }
  } else {
    lost_n_ = 0;
  }

  std::vector<Gate> ahead;
  ahead.reserve(gates.size());
  const double keep_behind = std::max(0.3, params_.keep_behind_m);
  for (const auto & g : gates) {
    if (g.s >= odom.x - keep_behind) {
      ahead.push_back(g);
    }
  }
  if (ahead.empty()) {
    held_gate_s_ = 0.0;
    held_last_s_ = 0.0;
    const double Lret = std::max(3.0, params_.return_after_last_m);
    if (std::abs(odom.y) < 0.12 && std::abs(odom.yaw) < 0.08) {
      last_path_ = centre;
      last_path_.n_gates = 0;
      return last_path_;
    }
    const double d_s_ret = std::clamp((0.0 - odom.y) / Lret, -0.45, 0.45);
    FrenetPath back = samplePath({
      {odom.x, odom.y, d_s_ret},
      {odom.x + Lret, 0.0, 0.0},
      {odom.x + Lret + 3.0, 0.0, 0.0},
    });
    back.n_gates = 0;
    back.target_d = 0.0;
    back.valid = back.pts.size() >= 2;
    last_path_ = back;
    return last_path_;
  }

  const bool same_gate = last_path_.valid && last_path_.n_gates > 0 &&
    std::abs(ahead.front().s - held_gate_s_) <= params_.new_cone_ds_m;
  const bool new_tail = ahead.size() != static_cast<size_t>(last_path_.n_gates) ||
    (ahead.size() >= 2 &&
     std::abs(ahead.back().s - held_last_s_) > params_.new_cone_ds_m);

  Knot start;
  start.s = odom.x;
  start.d = odom.y;
  // Keep the early S while the car is still on it. Replanning from the axle
  // every tick shrinks the ramp and postpones the turn. If the car has left
  // the curve, keeping that frozen path is what winds the heading up.
  if (same_gate && last_path_.valid && !new_tail) {
    const FrenetPoint p0 = last_path_.sample(odom.x);
    const double e = std::abs(odom.y - p0.d);
    const double ye = std::abs(wrapAngle(odom.yaw - p0.yaw));
    if (e < 0.50 && ye < 0.40) {
      nudgeOffObstacles(last_path_, odom, snap);
      last_path_.target_d = last_path_.sample(odom.x).d;
      return last_path_;
    }
    start.d_s = std::tan(std::clamp(odom.yaw, -0.45, 0.45));
  } else if (same_gate && last_path_.valid && new_tail) {
    const FrenetPoint p0 = last_path_.sample(odom.x);
    if (std::abs(odom.y - p0.d) < 0.50) {
      start.d = p0.d;
      start.d_s = p0.d_s;
    } else {
      start.d_s = std::tan(std::clamp(odom.yaw, -0.45, 0.45));
    }
  } else {
    const double arrive = std::max(1.5, params_.arrive_before_m);
    const double s_arr = ahead.front().s - arrive;
    const double L = std::max(2.5, s_arr - odom.x);
    const double toward =
      std::clamp((ahead.front().pass_d - odom.y) / L, -0.42, 0.42);
    const double yaw_s = std::tan(std::clamp(odom.yaw, -0.55, 0.55));
    start.d_s = (toward * yaw_s < 0.0) ? toward : (0.75 * toward + 0.25 * yaw_s);
    start.d_s = std::clamp(start.d_s, -0.42, 0.42);
  }

  const double max_off = laneMaxOffset();
  const double thr = EgoCostmap::kLethalCost * 0.45;
  for (auto & g : ahead) {
    for (int k = 0; k < 14; ++k) {
      double ex = 0.0;
      double ey = 0.0;
      odomToEgo(odom, g.s, g.pass_d, ex, ey);
      if (snap.getCost(ex, ey) < thr) {
        break;
      }
      const double dir = (g.side != 0) ? static_cast<double>(g.side)
                                       : ((g.pass_d >= g.cone_d) ? 1.0 : -1.0);
      g.pass_d = std::clamp(g.pass_d + dir * 0.10, -max_off, max_off);
    }
  }

  FrenetPath path = buildThroughGates(ahead, start);
  nudgeOffObstacles(path, odom, snap);
  held_gate_s_ = ahead.front().s;
  held_last_s_ = ahead.back().s;
  last_path_ = path;
  return last_path_;
}

}  // namespace mppi_local_planner
