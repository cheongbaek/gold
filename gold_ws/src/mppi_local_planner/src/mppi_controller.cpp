#include "mppi_local_planner/mppi_controller.hpp"

#include <algorithm>
#include <cmath>

namespace mppi_local_planner
{

MPPIController::MPPIController(
  const MPPIParams & mppi_params, const VehicleParams & vehicle_params)
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
  for (auto & u : nominal_) {
    u = Control{};
  }
  last_executed_ = Control{};
  last_trajectory_.clear();
  ref_path_ = {};
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
  State s{};
  double cost = 0.0;
  Control prev = last_executed_;

  const double cos_o = std::cos(current_odom_pose.yaw);
  const double sin_o = std::sin(current_odom_pose.yaw);
  const int T = params_.horizon_steps;
  const bool have_ref = ref_path_.valid && ref_path_.pts.size() >= 2;

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

  auto refAt = [&](double s_odom) {
    if (have_ref) {
      return ref_path_.sample(s_odom);
    }
    FrenetPoint p;
    p.s = s_odom;
    return p;
  };

  for (int t = 0; t < T; ++t) {
    const Control & u = controls[static_cast<size_t>(t)];
    s = step(s, u, params_.dt, vehicle_params_);

    const double s_odom =
      current_odom_pose.x + s.x * cos_o - s.y * sin_o;
    const double d_odom =
      current_odom_pose.y + s.x * sin_o + s.y * cos_o;
    const double yaw_odom = wrapAngle(current_odom_pose.yaw + s.yaw);

    const FrenetPoint rp = refAt(s_odom);
    const double cte = d_odom - rp.d;
    const double heading_err = wrapAngle(yaw_odom - rp.yaw);

    const double obs = footprintObs(s);
    const bool lethal = obs >= EgoCostmap::kLethalCost * 0.99;
    const double w_obs =
      (lethal || std::abs(cte) > params_.on_path_cte_m)
        ? params_.weight_obstacle
        : params_.weight_obstacle * params_.obstacle_on_path_scale;
    cost += w_obs * obs;

    cost += params_.weight_path * cte * cte;
    cost += params_.weight_heading * heading_err * heading_err;

    const double d_abs = std::abs(d_odom);
    if (d_abs > params_.max_lateral_offset) {
      const double over = d_abs - params_.max_lateral_offset;
      cost += params_.weight_lateral_wall * over * over;
    }
    if (d_abs > params_.lateral_hard) {
      const double over = d_abs - params_.lateral_hard;
      cost += params_.weight_lateral_hard * over * over;
    }
    {
      const double h_abs = std::abs(heading_err);
      if (h_abs > params_.max_heading_dev) {
        const double over = h_abs - params_.max_heading_dev;
        cost += params_.weight_heading_wall * over * over;
      }
    }

    const double dv = params_.desired_speed - u.v;
    cost += params_.weight_speed * dv * dv;
    cost += params_.weight_smooth_v * (u.v - prev.v) * (u.v - prev.v);
    cost += params_.weight_smooth_delta * (u.delta - prev.delta) * (u.delta - prev.delta);
    prev = u;
  }

  {
    const double s_f = current_odom_pose.x + s.x * cos_o - s.y * sin_o;
    const double d_f = current_odom_pose.y + s.x * sin_o + s.y * cos_o;
    const double yaw_f = wrapAngle(current_odom_pose.yaw + s.yaw);
    const FrenetPoint rp = refAt(s_f);
    const double cte_f = d_f - rp.d;
    const double h_f = wrapAngle(yaw_f - rp.yaw);
    cost += params_.weight_path_terminal * cte_f * cte_f;
    cost += params_.weight_heading_terminal * h_f * h_f;
  }

  // Probe the planned path ahead of the horizon (not the current heading).
  // A slalom return heading used to sweep the body through a just-passed cone.
  if (params_.lookahead_distance > 1e-3 && params_.lookahead_step > 1e-3) {
    const double w_look = params_.weight_obstacle * params_.weight_lookahead;
    const double s_end = current_odom_pose.x + s.x * cos_o - s.y * sin_o;
    for (double ds = params_.lookahead_step;
         ds <= params_.lookahead_distance + 1e-9;
         ds += params_.lookahead_step)
    {
      const FrenetPoint rp = refAt(s_end + ds);
      double ex = 0.0;
      double ey = 0.0;
      odomToEgo(current_odom_pose, rp.s, rp.d, ex, ey);
      State probe;
      probe.x = ex;
      probe.y = ey;
      probe.yaw = wrapAngle(rp.yaw - current_odom_pose.yaw);
      cost += w_look * footprintObs(probe);
    }
  }

  return cost;
}

static void fillPathTracking(
  std::vector<Control> & seq,
  double v_des,
  double steer_scale,
  double delta_bias,
  const FrenetPath & path,
  const OdomPose & odom,
  double L_stan,
  const Control & last_executed,
  double dt,
  const VehicleParams & vp)
{
  Control prev = last_executed;
  double s = odom.x;
  const int T = static_cast<int>(seq.size());
  const bool have = path.valid && path.pts.size() >= 2;
  const double d_ref0 = have ? path.sample(odom.x).d : 0.0;
  const double cte0 = odom.y - d_ref0;
  const double d_cte = -std::atan(cte0 / std::max(4.0, L_stan));
  for (int t = 0; t < T; ++t) {
    double kappa = 0.0;
    if (have) {
      kappa = path.sample(s).kappa;
    }
    Control u;
    u.v = v_des;
    u.delta = steer_scale * std::atan(vp.wheelbase * kappa) + d_cte + delta_bias;
    u = clampControl(u, prev, dt, vp);
    seq[static_cast<size_t>(t)] = u;
    prev = u;
    s += std::max(0.05, u.v) * dt;
  }
}

static void fillConstant(
  std::vector<Control> & seq,
  double v_des,
  double delta,
  const Control & last_executed,
  double dt,
  const VehicleParams & vp)
{
  Control prev = last_executed;
  for (size_t t = 0; t < seq.size(); ++t) {
    Control u;
    u.v = v_des;
    u.delta = delta;
    u = clampControl(u, prev, dt, vp);
    seq[t] = u;
    prev = u;
  }
}

MPPIResult MPPIController::computeControl(
  const OdomPose & current_odom_pose,
  const CostmapSnapshot & costmap,
  const FrenetPath & ref,
  int shift_steps)
{
  ensureBuffers();
  ref_path_ = ref;

  const int K = params_.num_samples;
  const int T = params_.horizon_steps;
  const double corr = std::clamp(params_.noise_correlation, 0.0, 0.99);
  const double innov = std::sqrt(std::max(1e-9, 1.0 - corr * corr));
  const double v_des = params_.desired_speed;
  const double dmax = vehicle_params_.max_steering_angle;
  const double s0 = current_odom_pose.x;
  const FrenetPoint r0 =
    (ref.valid && !ref.pts.empty()) ? ref.sample(s0) : FrenetPoint{};
  const double kappa0 = r0.kappa;
  const double ff0 = std::atan(vehicle_params_.wheelbase * kappa0);
  const double need_d = r0.d - current_odom_pose.y;
  if (need_d * last_executed_.delta < 0.0 && std::abs(need_d) > 0.12) {
    last_executed_.delta = 0.0;
    for (auto & u : nominal_) {
      if (need_d * u.delta < 0.0) {
        u.delta = 0.0;
      }
    }
  }
  const double toward = (need_d >= 0.0) ? 1.0 : -1.0;

  std::normal_distribution<double> noise_v(0.0, params_.noise_std_v);
  std::normal_distribution<double> noise_delta(0.0, params_.noise_std_delta);

  // Seeds follow the Frenet curve. Extra lock/1.15× curvature was steering
  // more than the drawn path.
  const double L_stan = std::max(3.2, params_.stanley_lookahead);
  constexpr int kNumPrimitives = 10;
  if (K >= kNumPrimitives) {
    int k = 0;
    fillPathTracking(samples_[static_cast<size_t>(k++)], v_des, 1.00, 0.0,
      ref, current_odom_pose, L_stan, last_executed_, params_.dt, vehicle_params_);
    fillPathTracking(samples_[static_cast<size_t>(k++)], v_des, 0.92, 0.0,
      ref, current_odom_pose, L_stan, last_executed_, params_.dt, vehicle_params_);
    fillPathTracking(samples_[static_cast<size_t>(k++)], v_des, 1.05, 0.0,
      ref, current_odom_pose, L_stan, last_executed_, params_.dt, vehicle_params_);
    fillPathTracking(samples_[static_cast<size_t>(k++)], v_des, 1.00, 0.14 * dmax * toward,
      ref, current_odom_pose, L_stan, last_executed_, params_.dt, vehicle_params_);
    fillPathTracking(samples_[static_cast<size_t>(k++)], v_des, 1.08, 0.08 * dmax * toward,
      ref, current_odom_pose, L_stan, last_executed_, params_.dt, vehicle_params_);
    fillPathTracking(samples_[static_cast<size_t>(k++)], 0.85 * v_des, 1.00, 0.0,
      ref, current_odom_pose, L_stan, last_executed_, params_.dt, vehicle_params_);
    fillConstant(samples_[static_cast<size_t>(k++)], v_des, ff0,
      last_executed_, params_.dt, vehicle_params_);
    fillConstant(samples_[static_cast<size_t>(k++)], v_des, last_executed_.delta,
      last_executed_, params_.dt, vehicle_params_);
    fillConstant(samples_[static_cast<size_t>(k++)], v_des, 0.0,
      last_executed_, params_.dt, vehicle_params_);
    while (k < kNumPrimitives) {
      fillConstant(samples_[static_cast<size_t>(k++)], v_des, 0.0,
        last_executed_, params_.dt, vehicle_params_);
    }
  }

  const int k_start = (K >= kNumPrimitives) ? kNumPrimitives : 0;
  for (int k = k_start; k < K; ++k) {
    double nv = 0.0;
    double nd = 0.0;
    Control prev = last_executed_;
    double s = s0;
    for (int t = 0; t < T; ++t) {
      nv = corr * nv + innov * noise_v(rng_);
      nd = corr * nd + innov * noise_delta(rng_);
      Control u;
      u.v = nominal_[static_cast<size_t>(t)].v + nv;
      u.delta = nominal_[static_cast<size_t>(t)].delta + nd;
      if (std::abs(nominal_[static_cast<size_t>(t)].v) < 0.05) {
        u.v = v_des + nv;
        const double kap =
          (ref.valid && !ref.pts.empty()) ? ref.sample(s).kappa : 0.0;
        u.delta = std::atan(vehicle_params_.wheelbase * kap) + nd;
      }
      u = clampControl(u, prev, params_.dt, vehicle_params_);
      samples_[static_cast<size_t>(k)][static_cast<size_t>(t)] = u;
      prev = u;
      s += std::max(0.05, u.v) * params_.dt;
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
    const double w =
      std::exp(-(costs_[static_cast<size_t>(k)] - min_cost) / params_.lambda);
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
    State st{};
    last_trajectory_.push_back(st);
    for (int t = 0; t < T; ++t) {
      st = step(st, nominal_[static_cast<size_t>(t)], params_.dt, vehicle_params_);
      last_trajectory_.push_back(st);
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
