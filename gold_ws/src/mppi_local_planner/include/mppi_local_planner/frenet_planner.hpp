#pragma once
#include "mppi_local_planner/cone_detector.hpp"
#include "mppi_local_planner/frenet_types.hpp"
#include "mppi_local_planner/vehicle_model.hpp"

namespace mppi_local_planner
{

struct FrenetPlannerParams
{
  bool enable = true;
  double plan_distance_m = 16.0;
  double ds_m = 0.25;
  double return_after_last_m = 3.5;
  double weave_m = 16.0;
  double lane_half_m = 1.75;
  double lane_margin_m = 0.10;
  double hold_s_m = 6.5;
  // First cone only: be on the pass offset this far ahead, so a ~3.6 m
  // pursuit point is already on the offset. Later cones do NOT use this —
  // adding it to the hold ate a 5 m gap and left a kink.
  double arrive_before_m = 4.5;
  // Stay on the pass offset this close to each cone (both sides). The weave
  // between cones uses whatever gap is left.
  double hold_near_m = 0.8;
  // Keep pass_d until this far BEHIND the cone, then return.
  // Returning while the cone is still ahead is a head-on hit.
  double wrap_after_m = 2.5;
  // Drop a gate only after it is this far behind the rear axle [m].
  double keep_behind_m = 0.60;

  double range_m = 16.0;
  double cone_half_m = 0.20;
  double margin_m = 0.20;
  double inflation_m = 0.40;
  double max_offset_m = 1.05;
  double pass_x_m = 0.90;
  double freeze_x_m = 4.0;
  double cone_y_max_m = 2.0;
  int cone_min_cells = 20;
  int cone_lost_ticks = 6;
  double new_cone_ds_m = 1.5;
};

// Builds a d(s) path through slalom cones. Does not emit steering.
//
// Reference is the L-zone odom line (s = x, d = y). Each cone becomes a gate
// at the side that stays closer to the centreline; Hermite segments connect
// current pose → gates → return to d=0. MPPI tracks the sampled curve.
class FrenetPlanner
{
public:
  FrenetPlanner(const FrenetPlannerParams & params, const VehicleParams & vehicle);

  void setParams(const FrenetPlannerParams & params) { params_ = params; }

  void reset();

  FrenetPath plan(const OdomPose & odom, const CostmapSnapshot & snap);

  void nudgeOffObstacles(
    FrenetPath & path, const OdomPose & odom, const CostmapSnapshot & snap) const;

  const ConeStats & lastStats() const { return last_stats_; }
  const std::vector<ConeObs> & lastConesEgo() const { return last_cones_ego_; }

private:
  struct Gate
  {
    double s = 0.0;
    double cone_d = 0.0;
    double pass_d = 0.0;
    int side = 0;  // -1 right / +1 left of the cone
  };

  struct Knot
  {
    double s = 0.0;
    double d = 0.0;
    double d_s = 0.0;
  };

  std::vector<Gate> makeGates(const OdomPose & odom, const std::vector<ConeObs> & cones);
  void augmentGatesFromCostmap(
    const OdomPose & odom, const CostmapSnapshot & snap, std::vector<Gate> & gates) const;
  FrenetPath buildThroughGates(
    const std::vector<Gate> & ahead, const Knot & start) const;
  FrenetPath samplePath(const std::vector<Knot> & knots) const;
  void finishDerivatives(FrenetPath & path) const;
  void applyKeepOut(FrenetPath & path, const std::vector<Gate> & gates) const;
  double laneMaxOffset() const;

  FrenetPlannerParams params_;
  VehicleParams vehicle_;
  ConeStats last_stats_{};
  std::vector<ConeObs> last_cones_ego_;
  FrenetPath last_path_;
  std::vector<Gate> mem_;
  int lost_n_ = 0;
  double held_gate_s_ = 0.0;
  double held_last_s_ = 0.0;
};

}  // namespace mppi_local_planner
