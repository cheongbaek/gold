#pragma once
#include <random>
#include <vector>

#include "mppi_local_planner/ego_costmap.hpp"
#include "mppi_local_planner/vehicle_model.hpp"

namespace mppi_local_planner
{

struct MPPIParams
{
  int horizon_steps = 60;     // with dt=0.05 -> 3.0 s horizon
  double dt = 0.05;           // [s] MUST match control period (1/control_frequency)
  int num_samples = 1200;
  double lambda = 2.0;        // temperature: higher keeps S-curve / exploratory samples

  double noise_std_v = 0.08;      // [m/s]
  double noise_std_delta = 0.12;  // [rad]
  // Temporal correlation of the noise process in [0, 1). Higher = smoother
  // exploration sequences (OU-like). 0 = independent per-step noise.
  double noise_correlation = 0.70;

  double desired_speed = 0.32;    // [m/s] cruise speed the planner tries to hold

  double weight_obstacle = 1.2;   // costmap already returns large numbers; keep ~1
  double weight_path = 0.45;      // gold: weak return so dodge does not snap back
  double weight_heading = 0.30;
  double weight_speed = 2.0;
  double weight_smooth_v = 0.6;
  double weight_smooth_delta = 4.0;

  // Larger L = gentler post-dodge heading. 1/5 zigzag used ~1.0 (snappy S).
  double stanley_lookahead = 5.0;  // [m]

  double s_curve_dodge_frac = 0.40;   // free-dodge fraction of horizon
  double s_curve_return_power = 1.30; // >1 => slower return ramp
  double path_progress_floor = 0.08;

  double avoid_path_scale = 0.20;
  double avoid_obs_gain = 80.0;
  double offset_return_y = 0.55;      // [m] do not slam-return at 12 cm offset
  double offset_return_scale = 0.35;

  double weight_return_clear = 0.50;
  double return_clear_cost = 50.0;

  double weight_path_terminal = 6.0;
  double weight_heading_terminal = 2.5;

  double max_lateral_offset = 2.20;   // [m]
  double weight_lateral_wall = 5.0;

  // Extra obstacle probes beyond the control horizon (along final heading).
  double lookahead_distance = 5.0;   // [m] beyond last rollout pose
  double lookahead_step = 0.40;      // [m] sampling pitch
  double weight_lookahead = 0.55;    // scales weight_obstacle on those probes

  // Stop if (min_cost / horizon_steps) exceeds this average-per-step value.
  double stop_cost_threshold = 750.0;
};

// Pose of the robot's CURRENT planning-cycle origin, expressed in a persistent
// "local odom" frame that the node maintains by dead-reckoning (gyro yaw + last
// commanded speed -- see the node for caveats). The reference line the planner
// tries to return to is simply y_odom == 0.
struct OdomPose
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
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

  // Runs one MPPI iteration. shift_steps advances the warm-start by that many
  // dt intervals (normally 1 when control_frequency == 1/dt; can be 0 if the
  // control loop ran faster than the model step).
  MPPIResult computeControl(
    const OdomPose & current_odom_pose,
    const CostmapSnapshot & costmap,
    int shift_steps = 1);

  // Ego-frame trajectory from the last chosen sequence (for RViz).
  std::vector<State> getLastRolloutTrajectory() const;

  double dt() const { return params_.dt; }

private:
  double rolloutCost(
    const std::vector<Control> & controls,
    const OdomPose & current_odom_pose,
    const CostmapSnapshot & costmap) const;

  void ensureBuffers();
  void shiftNominal(int steps);

  MPPIParams params_;
  VehicleParams vehicle_params_;
  std::vector<std::pair<double, double>> footprint_;
  std::vector<Control> nominal_;  // warm-started control sequence
  Control last_executed_;
  std::mt19937 rng_;
  std::vector<State> last_trajectory_;

  // Reused every cycle to avoid allocation churn.
  std::vector<std::vector<Control>> samples_;
  std::vector<double> costs_;
  std::vector<double> weights_;
};

}  // namespace mppi_local_planner
