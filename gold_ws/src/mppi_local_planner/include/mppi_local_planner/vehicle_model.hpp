#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

namespace mppi_local_planner
{

// Kinematic bicycle-model state, expressed in the EGO frame: the robot's own
// pose at the start of the current planning cycle is always (0, 0, 0).
// x: forward [m], y: left [m], yaw: heading [rad]
// Reference point is the rear axle (standard bicycle model).
struct State
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

// Control applied for one timestep.
struct Control
{
  double v = 0.0;      // forward speed [m/s]
  double delta = 0.0;  // front-wheel steering angle [rad], + = steer left
};

struct VehicleParams
{
  // Defaults = 금색차 kasa (lidar/kasa_units.hpp · white/kasa_units.py 실측).
  // 1/5카 원본은 wheelbase 0.75 / track 0.65 / max_steer 0.40 였다.
  double wheelbase = 1.25;           // [m] 축거 1250mm 실측
  double track_width = 1.10;         // [m] 윤거 1100mm 실측 (lidar yaml 의 0.65 는 1/5 잔재)
  double max_speed = 2.05;           // [m/s] 2펄스(1.768) 위 여유. 액추에이터는 정수 펄스로 자른다
  double min_speed = 0.80;           // [m/s] 회피 중 1펄스 허용
  double max_steering_angle = 0.40;  // [rad] ≈23°. 원본 3params 와 동일

  // Body extents relative to the rear-axle reference point (ego origin).
  // Used for multi-point collision checks during rollouts.
  // 라이다 원점→앞범퍼 1.2 m (cone_lidar.yaml vehicle_front_m), 축거 1.25 m.
  double rear_overhang = 0.30;   // [m] how far the body extends behind the rear axle
  double front_overhang = 0.0;   // [m] how far the body extends ahead of the front axle
  // total length ≈ rear_overhang + wheelbase + front_overhang

  // Hard rate limits applied when sampling / executing controls.
  double max_accel = 2.0;              // [m/s^2] |dv/dt|
  double max_steering_rate = 1.20;     // [rad/s] 첫 틱이 조향 불감대를 넘기게
};

inline double wrapAngle(double a)
{
  // Stable wrap to (-pi, pi]
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += 2.0 * M_PI;
  }
  return a - M_PI;
}

// Clamp absolute limits and rate limits relative to a previous control over dt.
inline Control clampControl(const Control & u_in, const Control & prev, double dt, const VehicleParams & p)
{
  Control u = u_in;
  u.v = std::clamp(u.v, p.min_speed, p.max_speed);
  u.delta = std::clamp(u.delta, -p.max_steering_angle, p.max_steering_angle);

  if (dt > 1e-6) {
    const double max_dv = p.max_accel * dt;
    const double max_dd = p.max_steering_rate * dt;
    u.v = std::clamp(u.v, prev.v - max_dv, prev.v + max_dv);
    u.delta = std::clamp(u.delta, prev.delta - max_dd, prev.delta + max_dd);
    // Re-clamp absolute bounds after rate limiting.
    u.v = std::clamp(u.v, p.min_speed, p.max_speed);
    u.delta = std::clamp(u.delta, -p.max_steering_angle, p.max_steering_angle);
  }
  return u;
}

// Propagate one step of the kinematic bicycle model (rear-axle reference point).
inline State step(const State & s, const Control & u_in, double dt, const VehicleParams & p)
{
  Control u = u_in;
  u.v = std::clamp(u.v, p.min_speed, p.max_speed);
  u.delta = std::clamp(u.delta, -p.max_steering_angle, p.max_steering_angle);

  State next;
  next.x = s.x + u.v * std::cos(s.yaw) * dt;
  next.y = s.y + u.v * std::sin(s.yaw) * dt;
  next.yaw = wrapAngle(s.yaw + (u.v / p.wheelbase) * std::tan(u.delta) * dt);
  return next;
}

// Body footprint sample points in the vehicle body frame (rear axle at origin,
// x forward). The costmap already inflates obstacles by ~half-width, so we
// sample the longitudinal centerline heavily and only add front corners for
// swing-out (not full side corners — that would double-count lateral margin).
inline std::vector<std::pair<double, double>> footprintOffsets(const VehicleParams & p)
{
  const double half_w = p.track_width * 0.5;
  const double x_rear = -p.rear_overhang;
  const double x_front = p.wheelbase + p.front_overhang;
  const double x_mid = 0.5 * (x_rear + x_front);
  return {
    {x_rear, 0.0},
    {0.0, 0.0},           // rear axle
    {x_mid, 0.0},
    {p.wheelbase, 0.0},   // front axle
    {x_front, 0.0},
    {x_front, -half_w},   // front corners (turn swing-out)
    {x_front, half_w},
  };
}

// Transform a body-frame offset into the ego/world frame given vehicle state.
inline void bodyToEgo(
  const State & s, double bx, double by, double & ex, double & ey)
{
  const double c = std::cos(s.yaw);
  const double sn = std::sin(s.yaw);
  ex = s.x + bx * c - by * sn;
  ey = s.y + bx * sn + by * c;
}

}  // namespace mppi_local_planner
