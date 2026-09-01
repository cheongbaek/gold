#include "mppi_local_planner/mppi_controller.hpp"

#include <algorithm>
#include <cmath>

namespace mppi_local_planner
{

MPPIController::MPPIController(const MPPIParams & mppi_params, const VehicleParams & vehicle_params)
: params_(mppi_params),
  vehicle_params_(vehicle_params),
  footprint_(footprintOffsets(vehicle_params)),
  rng_(std::random_device{}())
{
  nominal_.assign(static_cast<size_t>(params_.horizon_steps), Control{0.0, 0.0});
  last_executed_ = Control{0.0, 0.0};
  ensureBuffers();
}

void MPPIController::ensureBuffers()
{
  const int K = params_.num_samples;
  const int T = params_.horizon_steps;
  if (static_cast<int>(samples_.size()) != K) {
    samples_.assign(static_cast<size_t>(K), std::vector<Control>(static_cast<size_t>(T)));
    costs_.assign(static_cast<size_t>(K), 0.0);
    weights_.assign(static_cast<size_t>(K), 0.0);
  } else if (!samples_.empty() && static_cast<int>(samples_[0].size()) != T) {
    for (auto & seq : samples_) {
      seq.assign(static_cast<size_t>(T), Control{});
    }
  }
}

void MPPIController::reset()
{
  // ensureBuffers() sizes nominal_ on the next computeControl(); zeroing what
  // is there now is enough — Control{} is v=0, delta=0.
  for (auto & u : nominal_) {
    u = Control{};
  }
  last_trajectory_.clear();
}

void MPPIController::shiftNominal(int steps)
{
  const int T = params_.horizon_steps;
  if (steps <= 0 || T <= 0) {
    return;
  }
  steps = std::min(steps, T);
  for (int t = 0; t + steps < T; ++t) {
    nominal_[static_cast<size_t>(t)] = nominal_[static_cast<size_t>(t + steps)];
  }
  const Control tail = nominal_[static_cast<size_t>(std::max(0, T - steps - 1))];
  for (int t = T - steps; t < T; ++t) {
    if (t >= 0) {
      nominal_[static_cast<size_t>(t)] = tail;
    }
  }
}

double MPPIController::rolloutCost(
  const std::vector<Control> & controls,
  const OdomPose & current_odom_pose,
  const CostmapSnapshot & costmap) const
{
  State s{};  // ego frame: rollout starts at the origin (rear axle)
  double cost = 0.0;
  Control prev = last_executed_;

  const double cos_o = std::cos(current_odom_pose.yaw);
  const double sin_o = std::sin(current_odom_pose.yaw);
  const int T = params_.horizon_steps;
  const double L_stan = std::max(0.25, params_.stanley_lookahead);
  const double dodge_frac = std::clamp(params_.s_curve_dodge_frac, 0.05, 0.6);
  const double return_pow = std::max(0.3, params_.s_curve_return_power);
  const double progress_floor = std::clamp(params_.path_progress_floor, 0.0, 1.0);
  const double y0 = current_odom_pose.y;
  const double abs_y0 = std::abs(y0);

  auto footprintObs = [&](const State & st) {
    double obs = 0.0;
    for (const auto & fp : footprint_) {
      double ex = 0.0;
      double ey = 0.0;
      bodyToEgo(st, fp.first, fp.second, ex, ey);
      obs = std::max(obs, costmap.getCost(ex, ey));
    }
    return obs;
  };

  // Soften path/heading only while still near the reference line. Once already
  // offset after cone #1, force strong return even if cone #2 still blocks the
  // centerline — otherwise the car stays on the wrong side of the zigzag.
  double avoid_scale = 1.0;
  if (params_.avoid_obs_gain > 1e-3 && abs_y0 < params_.offset_return_y) {
    double ahead = 0.0;
    for (double ax = 0.6; ax <= 3.0; ax += 0.6) {
      ahead = std::max(ahead, costmap.getCost(ax, 0.0));
      ahead = std::max(ahead, costmap.getCost(ax, 0.35));
      ahead = std::max(ahead, costmap.getCost(ax, -0.35));
    }
    const double floor_s = std::clamp(params_.avoid_path_scale, 0.0, 1.0);
    const double t = std::clamp(ahead / params_.avoid_obs_gain, 0.0, 1.0);
    avoid_scale = 1.0 - t * (1.0 - floor_s);
  } else if (abs_y0 >= params_.offset_return_y) {
    avoid_scale = std::max(avoid_scale, params_.offset_return_scale);
  }

  // How free is the *reference corridor* ahead (for return-clear bonus)?
  double ref_corridor = 0.0;
  for (double ax = 0.8; ax <= 3.5; ax += 0.5) {
    ref_corridor = std::max(ref_corridor, costmap.getCost(ax, 0.0));
    ref_corridor = std::max(ref_corridor, costmap.getCost(ax, 0.25));
    ref_corridor = std::max(ref_corridor, costmap.getCost(ax, -0.25));
  }
  const double clear_frac = std::clamp(
    1.0 - ref_corridor / std::max(1.0, params_.return_clear_cost), 0.0, 1.0);

  for (int t = 0; t < T; ++t) {
    const Control & u = controls[static_cast<size_t>(t)];
    s = step(s, u, params_.dt, vehicle_params_);

    cost += params_.weight_obstacle * footprintObs(s);

    // Transform rear-axle point into the persistent odom frame for path cost.
    const double y_odom = current_odom_pose.y + s.x * sin_o + s.y * cos_o;
    const double yaw_odom = wrapAngle(current_odom_pose.yaw + s.yaw);

    const double cross_track = y_odom;
    // Snappy Stanley return: when offset left (y>0), point right to close the S.
    const double target_yaw = -std::atan2(cross_track, L_stan);
    const double heading_err = wrapAngle(yaw_odom - target_yaw);

    // Parabolic S path scale: free early wrap, then aggressive return.
    const double frac = static_cast<double>(t + 1) / static_cast<double>(std::max(1, T));
    double path_scale = progress_floor;
    if (frac > dodge_frac) {
      const double u = (frac - dodge_frac) / std::max(1e-6, 1.0 - dodge_frac);
      path_scale = progress_floor + (1.0 - progress_floor) * std::pow(u, return_pow);
    }
    // Already offset this cycle: do not wait for dodge_frac — return now.
    if (abs_y0 >= params_.offset_return_y) {
      path_scale = std::max(path_scale, params_.offset_return_scale * (0.4 + 0.6 * frac));
    }
    path_scale *= avoid_scale;

    // Extra return pressure once past first-obstacle envelope (ref looks free).
    if (params_.weight_return_clear > 1e-6 && clear_frac > 0.2) {
      path_scale += params_.weight_return_clear * clear_frac *
        (std::abs(cross_track) / std::max(0.2, params_.max_lateral_offset));
    }

    cost += params_.weight_path * path_scale * cross_track * cross_track;
    cost += params_.weight_heading * path_scale * heading_err * heading_err;

    // Soft lateral wall: discourage permanent side-lane after first dodge.
    const double y_abs = std::abs(cross_track);
    if (y_abs > params_.max_lateral_offset) {
      const double over = y_abs - params_.max_lateral_offset;
      cost += params_.weight_lateral_wall * over * over;
    }

    const double dv = params_.desired_speed - u.v;
    cost += params_.weight_speed * dv * dv;

    cost += params_.weight_smooth_v * (u.v - prev.v) * (u.v - prev.v);
    cost += params_.weight_smooth_delta * (u.delta - prev.delta) * (u.delta - prev.delta);

    prev = u;
  }

  // Terminal: end of horizon must sit back on the IMU line (complete the S).
  {
    const double y_f = current_odom_pose.y + s.x * sin_o + s.y * cos_o;
    const double yaw_f = wrapAngle(current_odom_pose.yaw + s.yaw);
    const double target_yaw_f = -std::atan2(y_f, L_stan);
    const double h_f = wrapAngle(yaw_f - target_yaw_f);
    cost += params_.weight_path_terminal * y_f * y_f;
    cost += params_.weight_heading_terminal * h_f * h_f;
  }

  // ---- Extended obstacle lookahead (beyond control horizon) ----
  if (params_.lookahead_distance > 1e-3 && params_.lookahead_step > 1e-3) {
    const double w_look = params_.weight_obstacle * params_.weight_lookahead;
    const double c = std::cos(s.yaw);
    const double sn = std::sin(s.yaw);
    for (double d = params_.lookahead_step; d <= params_.lookahead_distance + 1e-9;
         d += params_.lookahead_step)
    {
      State probe = s;
      probe.x = s.x + d * c;
      probe.y = s.y + d * sn;
      cost += w_look * footprintObs(probe);
    }
  }

  return cost;
}

// Equal-length segment primitive (legacy helper).
static void fillPrimitive(
  std::vector<Control> & seq,
  double v_des,
  const std::vector<double> & deltas,
  const Control & last_executed,
  double dt,
  const VehicleParams & vp)
{
  Control prev = last_executed;
  const int T = static_cast<int>(seq.size());
  const int nseg = static_cast<int>(deltas.size());
  for (int t = 0; t < T; ++t) {
    const int seg = std::min(nseg - 1, (t * nseg) / std::max(1, T));
    Control u;
    u.v = v_des;
    u.delta = deltas[static_cast<size_t>(seg)];
    u = clampControl(u, prev, dt, vp);
    seq[static_cast<size_t>(t)] = u;
    prev = u;
  }
}

// Uneven segment lengths (fractions sum ~1) for short-dodge / long-return S.
static void fillPrimitiveFrac(
  std::vector<Control> & seq,
  double v_des,
  const std::vector<double> & deltas,
  const std::vector<double> & fracs,
  const Control & last_executed,
  double dt,
  const VehicleParams & vp)
{
  Control prev = last_executed;
  const int T = static_cast<int>(seq.size());
  const int nseg = static_cast<int>(deltas.size());
  if (nseg <= 0 || T <= 0) {
    return;
  }
  // Cumulative fraction boundaries.
  std::vector<double> edges(static_cast<size_t>(nseg) + 1, 0.0);
  double sum = 0.0;
  for (int i = 0; i < nseg; ++i) {
    const double f = (i < static_cast<int>(fracs.size())) ? std::max(0.0, fracs[static_cast<size_t>(i)]) : 1.0;
    sum += f;
    edges[static_cast<size_t>(i + 1)] = sum;
  }
  if (sum < 1e-9) {
    fillPrimitive(seq, v_des, deltas, last_executed, dt, vp);
    return;
  }
  for (int i = 0; i <= nseg; ++i) {
    edges[static_cast<size_t>(i)] /= sum;
  }

  for (int t = 0; t < T; ++t) {
    const double u = (static_cast<double>(t) + 0.5) / static_cast<double>(T);
    int seg = nseg - 1;
    for (int i = 0; i < nseg; ++i) {
      if (u <= edges[static_cast<size_t>(i + 1)]) {
        seg = i;
        break;
      }
    }
    Control c;
    c.v = v_des;
    c.delta = deltas[static_cast<size_t>(seg)];
    c = clampControl(c, prev, dt, vp);
    seq[static_cast<size_t>(t)] = c;
    prev = c;
  }
}

MPPIResult MPPIController::computeControl(
  const OdomPose & current_odom_pose,
  const CostmapSnapshot & costmap,
  int shift_steps)
{
  ensureBuffers();

  const int K = params_.num_samples;
  const int T = params_.horizon_steps;
  const double corr = std::clamp(params_.noise_correlation, 0.0, 0.99);
  const double innov = std::sqrt(std::max(1e-9, 1.0 - corr * corr));
  const double v_des = params_.desired_speed;
  const double dmax = vehicle_params_.max_steering_angle;
  const double dhalf = 0.5 * dmax;
  const double d75 = 0.75 * dmax;
  const double y0 = current_odom_pose.y;

  std::normal_distribution<double> noise_v(0.0, params_.noise_std_v);
  std::normal_distribution<double> noise_delta(0.0, params_.noise_std_delta);

  // Side of nearest forward obstacle (for first-cone wrap direction).
  double left_obs = 0.0;
  double right_obs = 0.0;
  for (double ax = 0.6; ax <= 3.5; ax += 0.5) {
    left_obs = std::max(left_obs, costmap.getCost(ax, 0.55));
    left_obs = std::max(left_obs, costmap.getCost(ax, 0.90));
    right_obs = std::max(right_obs, costmap.getCost(ax, -0.55));
    right_obs = std::max(right_obs, costmap.getCost(ax, -0.90));
  }
  // Prefer dodge toward free side: +1 = obstacle more on left (dodge right first),
  // -1 = obstacle more on right (dodge left first).
  const int prefer_sign = (left_obs > right_obs + 15.0) ? -1 :
                          (right_obs > left_obs + 15.0) ? +1 : 0;

  // catkin_ws 원본과 같은 프리미티브 세트 (짧은 회피 + IMU S 복귀).
  // 금색차는 dmax·rate 를 params 에서 낮춰 같은 모양이 부드럽게 나가게 한다.
  constexpr int kNumPrimitives = 20;
  if (K >= kNumPrimitives) {
    int k = 0;
    fillPrimitive(samples_[static_cast<size_t>(k++)], v_des, {0.0},
      last_executed_, params_.dt, vehicle_params_);

    fillPrimitiveFrac(
      samples_[static_cast<size_t>(k++)], v_des,
      {dmax, dmax, -dmax, -d75, 0.0},
      {0.12, 0.12, 0.38, 0.22, 0.16},
      last_executed_, params_.dt, vehicle_params_);
    fillPrimitiveFrac(
      samples_[static_cast<size_t>(k++)], v_des,
      {-dmax, -dmax, dmax, d75, 0.0},
      {0.12, 0.12, 0.38, 0.22, 0.16},
      last_executed_, params_.dt, vehicle_params_);
    fillPrimitiveFrac(
      samples_[static_cast<size_t>(k++)], v_des,
      {dmax, -dmax, -dmax, -dhalf, 0.0},
      {0.18, 0.28, 0.28, 0.16, 0.10},
      last_executed_, params_.dt, vehicle_params_);
    fillPrimitiveFrac(
      samples_[static_cast<size_t>(k++)], v_des,
      {-dmax, dmax, dmax, dhalf, 0.0},
      {0.18, 0.28, 0.28, 0.16, 0.10},
      last_executed_, params_.dt, vehicle_params_);

    fillPrimitiveFrac(
      samples_[static_cast<size_t>(k++)], v_des,
      {d75, dhalf, -d75, -dhalf, 0.0},
      {0.15, 0.15, 0.35, 0.20, 0.15},
      last_executed_, params_.dt, vehicle_params_);
    fillPrimitiveFrac(
      samples_[static_cast<size_t>(k++)], v_des,
      {-d75, -dhalf, d75, dhalf, 0.0},
      {0.15, 0.15, 0.35, 0.20, 0.15},
      last_executed_, params_.dt, vehicle_params_);

    fillPrimitiveFrac(
      samples_[static_cast<size_t>(k++)], v_des,
      {dmax, -dmax, -dmax, dmax, dhalf, 0.0},
      {0.14, 0.20, 0.18, 0.20, 0.16, 0.12},
      last_executed_, params_.dt, vehicle_params_);
    fillPrimitiveFrac(
      samples_[static_cast<size_t>(k++)], v_des,
      {-dmax, dmax, dmax, -dmax, -dhalf, 0.0},
      {0.14, 0.20, 0.18, 0.20, 0.16, 0.12},
      last_executed_, params_.dt, vehicle_params_);

    fillPrimitive(samples_[static_cast<size_t>(k++)], v_des, {dhalf},
      last_executed_, params_.dt, vehicle_params_);
    fillPrimitive(samples_[static_cast<size_t>(k++)], v_des, {-dhalf},
      last_executed_, params_.dt, vehicle_params_);

    if (prefer_sign >= 0) {
      fillPrimitiveFrac(
        samples_[static_cast<size_t>(k++)], v_des,
        {dmax, dmax, -dmax, -dmax, 0.0},
        {0.14, 0.12, 0.36, 0.22, 0.16},
        last_executed_, params_.dt, vehicle_params_);
      fillPrimitiveFrac(
        samples_[static_cast<size_t>(k++)], v_des,
        {d75, -dmax, -d75, 0.0},
        {0.20, 0.40, 0.25, 0.15},
        last_executed_, params_.dt, vehicle_params_);
    } else {
      fillPrimitiveFrac(
        samples_[static_cast<size_t>(k++)], v_des,
        {-dmax, -dmax, dmax, dmax, 0.0},
        {0.14, 0.12, 0.36, 0.22, 0.16},
        last_executed_, params_.dt, vehicle_params_);
      fillPrimitiveFrac(
        samples_[static_cast<size_t>(k++)], v_des,
        {-d75, dmax, d75, 0.0},
        {0.20, 0.40, 0.25, 0.15},
        last_executed_, params_.dt, vehicle_params_);
    }

    const double ret = (y0 >= 0.0) ? -dmax : dmax;
    const double ret2 = (y0 >= 0.0) ? -d75 : d75;
    fillPrimitiveFrac(
      samples_[static_cast<size_t>(k++)], v_des,
      {ret, ret, ret2, 0.0},
      {0.30, 0.30, 0.25, 0.15},
      last_executed_, params_.dt, vehicle_params_);
    fillPrimitiveFrac(
      samples_[static_cast<size_t>(k++)], v_des,
      {ret, ret2, 0.0, 0.0},
      {0.35, 0.30, 0.20, 0.15},
      last_executed_, params_.dt, vehicle_params_);
    fillPrimitive(
      samples_[static_cast<size_t>(k++)], v_des, {ret},
      last_executed_, params_.dt, vehicle_params_);
    fillPrimitive(
      samples_[static_cast<size_t>(k++)], v_des, {ret2},
      last_executed_, params_.dt, vehicle_params_);

    fillPrimitive(
      samples_[static_cast<size_t>(k++)], v_des, {dmax, dmax, -dmax, -dhalf, 0.0},
      last_executed_, params_.dt, vehicle_params_);
    fillPrimitive(
      samples_[static_cast<size_t>(k++)], v_des, {-dmax, -dmax, dmax, dhalf, 0.0},
      last_executed_, params_.dt, vehicle_params_);

    fillPrimitive(samples_[static_cast<size_t>(k++)], v_des, {0.0, 0.0},
      last_executed_, params_.dt, vehicle_params_);

    // Fill any leftover slots if k < kNumPrimitives (shouldn't happen).
    while (k < kNumPrimitives) {
      fillPrimitive(samples_[static_cast<size_t>(k++)], v_des, {0.0},
        last_executed_, params_.dt, vehicle_params_);
    }
  }

  const int k_start = (K >= kNumPrimitives) ? kNumPrimitives : 0;
  for (int k = k_start; k < K; ++k) {
    double nv = 0.0;
    double nd = 0.0;
    Control prev = last_executed_;
    // When already offset, bias noise exploration toward counter-steer.
    const double return_bias =
      (std::abs(y0) > params_.offset_return_y) ?
      ((y0 > 0.0) ? -0.35 * dmax : 0.35 * dmax) : 0.0;
    for (int t = 0; t < T; ++t) {
      nv = corr * nv + innov * noise_v(rng_);
      nd = corr * nd + innov * noise_delta(rng_);

      Control u;
      u.v = nominal_[static_cast<size_t>(t)].v + nv;
      u.delta = nominal_[static_cast<size_t>(t)].delta + nd + return_bias;
      if (std::abs(nominal_[static_cast<size_t>(t)].v) < 0.05) {
        u.v = v_des + nv;
      }
      u = clampControl(u, prev, params_.dt, vehicle_params_);
      samples_[static_cast<size_t>(k)][static_cast<size_t>(t)] = u;
      prev = u;
    }
  }

  for (int k = 0; k < K; ++k) {
    costs_[static_cast<size_t>(k)] =
      rolloutCost(samples_[static_cast<size_t>(k)], current_odom_pose, costmap);
  }

  const double min_cost = *std::min_element(costs_.begin(), costs_.end());

  MPPIResult result;
  result.min_cost = min_cost;

  const double avg_cost = min_cost / static_cast<double>(std::max(1, T));
  if (avg_cost >= params_.stop_cost_threshold) {
    result.control = Control{0.0, 0.0};
    result.stopped_for_collision = true;
    last_executed_ = result.control;
    for (auto & u : nominal_) {
      u = Control{0.0, 0.0};
    }
    last_trajectory_.clear();
    last_trajectory_.push_back(State{});
    return result;
  }

  double weight_sum = 0.0;
  for (int k = 0; k < K; ++k) {
    const double w = std::exp(-(costs_[static_cast<size_t>(k)] - min_cost) / params_.lambda);
    weights_[static_cast<size_t>(k)] = w;
    weight_sum += w;
  }

  std::vector<Control> updated(static_cast<size_t>(T), Control{0.0, 0.0});
  for (int k = 0; k < K; ++k) {
    const double w = weights_[static_cast<size_t>(k)] / weight_sum;
    for (int t = 0; t < T; ++t) {
      updated[static_cast<size_t>(t)].v +=
        w * samples_[static_cast<size_t>(k)][static_cast<size_t>(t)].v;
      updated[static_cast<size_t>(t)].delta +=
        w * samples_[static_cast<size_t>(k)][static_cast<size_t>(t)].delta;
    }
  }

  {
    Control prev = last_executed_;
    for (int t = 0; t < T; ++t) {
      updated[static_cast<size_t>(t)] =
        clampControl(updated[static_cast<size_t>(t)], prev, params_.dt, vehicle_params_);
      prev = updated[static_cast<size_t>(t)];
    }
  }

  nominal_ = std::move(updated);

  {
    last_trajectory_.clear();
    last_trajectory_.reserve(static_cast<size_t>(T) + 1);
    State s{};
    last_trajectory_.push_back(s);
    for (int t = 0; t < T; ++t) {
      s = step(s, nominal_[static_cast<size_t>(t)], params_.dt, vehicle_params_);
      last_trajectory_.push_back(s);
    }
  }

  result.control = nominal_.front();
  result.stopped_for_collision = false;

  shiftNominal(std::max(1, shift_steps));

  last_executed_ = result.control;
  return result;
}

std::vector<State> MPPIController::getLastRolloutTrajectory() const
{
  return last_trajectory_;
}

}  // namespace mppi_local_planner
