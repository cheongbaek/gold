#pragma once
#include <random>
#include <vector>

#include "mppi_local_planner/ego_costmap.hpp"
#include "mppi_local_planner/frenet_types.hpp"
#include "mppi_local_planner/vehicle_model.hpp"

namespace mppi_local_planner
{

struct MPPIParams
{
  int horizon_steps = 60;     // dt=0.05 → 3.0 s
  double dt = 0.05;
  int num_samples = 1200;
  double lambda = 2.0;

  double noise_std_v = 0.08;      // [m/s]
  double noise_std_delta = 0.12;  // [rad]
  double noise_correlation = 0.70;

  double desired_speed = 0.32;    // [m/s]

  double weight_obstacle = 1.2;
  // When |d - d_ref| is small, inflation must not beat the plan (extra dodge).
  double obstacle_on_path_scale = 0.18;
  double on_path_cte_m = 0.35;
  double weight_path = 40.0;
  double weight_heading = 10.0;
  double weight_speed = 2.0;
  double weight_smooth_v = 0.6;
  double weight_smooth_delta = 4.0;

  double stanley_lookahead = 4.0;  // [m] geometric-steer fallback only

  double weight_path_terminal = 40.0;
  double weight_heading_terminal = 20.0;

  double max_lateral_offset = 1.90;   // [m] soft wall on |d|
  double lateral_hard = 2.5;
  double weight_lateral_hard = 4000.0;
  double max_heading_dev = 0.52;      // [rad] vs Frenet heading, not the centreline
  double weight_heading_wall = 800.0;
  double weight_lateral_wall = 2000.0;

  // Obstacle probes along the Frenet path beyond the control horizon.
  double lookahead_distance = 5.0;
  double lookahead_step = 0.40;
  double weight_lookahead = 0.55;

  double stop_cost_threshold = 750.0;
};

struct MPPIResult
{
  Control control;
  bool stopped_for_collision = false;
  double min_cost = 0.0;
};

class MPPIController
{
public:
  MPPIController(const MPPIParams & mppi_params, const VehicleParams & vehicle_params);

  // Track `ref` (Frenet d(s)). Empty/invalid ref = centreline d=0.
  MPPIResult computeControl(
    const OdomPose & current_odom_pose,
    const CostmapSnapshot & costmap,
    const FrenetPath & ref,
    int shift_steps = 1);

  std::vector<State> getLastRolloutTrajectory() const;

  // Drop warm-start after a planning gap (L-zone rearm).
  void reset();

  double dt() const { return params_.dt; }

  void setParams(const MPPIParams & p) { params_ = p; }

private:
  double rolloutCost(
    const std::vector<Control> & controls,
    const OdomPose & current_odom_pose,
    const CostmapSnapshot & costmap) const;

  void ensureBuffers();
  void shiftNominal(int steps);

  MPPIParams params_;
  VehicleParams vehicle_params_;
  FrenetPath ref_path_;
  std::vector<std::pair<double, double>> footprint_;
  std::vector<Control> nominal_;
  Control last_executed_;
  std::mt19937 rng_;
  std::vector<State> last_trajectory_;

  std::vector<std::vector<Control>> samples_;
  std::vector<double> costs_;
  std::vector<double> weights_;
};

}  // namespace mppi_local_planner
