#pragma once
#include <algorithm>
#include <cmath>

#include "mppi_local_planner/ego_costmap.hpp"
#include "mppi_local_planner/frenet_types.hpp"
#include "mppi_local_planner/vehicle_model.hpp"

namespace mppi_local_planner
{

// ══════════════════════════════════════════════════════════════════════════
//  ★[2026-09-28] Frenet 경로 추종 + 정지 판정 — 노드와 모의실험이 같은 코드를 쓴다★
// ══════════════════════════════════════════════════════════════════════════
//  ① 조향 = 경로 위 Ld 앞 점의 순수추종 + 뒤차축 횡오차 약보정 (종전 식 그대로).
//     ★다른 점은 하나 — 조향 지연을 앞당겨 본다★ 이 차의 조향은 불감시간 0.25 s ·
//     63% 0.55 s 다(CLAUDE.md 1.3). 지금 자세로 겨누면 그 명령이 먹힐 때의 차는
//     이미 0.4~0.7 m 앞에 있어 매번 늦는다. 지금 조향으로 predict_s 만큼 굴린
//     자세에서 겨눈다.
//  ② 정지 = 그 경로를 ★실제로 따라갔을 때★ 차 중심선이 치사 원반에 드는가.
//     종전 commandArcHits 는 ★지금 조향을 2.2 m 고정★ 했다. 콘 옆에서 되돌아오는
//     조향은 그 순간 콘 쪽을 향하므로 '고정 원호' 는 부딪히는데 경로는 비어 있다 —
//     오늘 네 주행의 정지가 전부 그 모양이었고, 서 있는 동안에도 같은 원호라
//     영영 풀리지 않았다.
struct TrackerParams
{
  double wheelbase = 1.25;
  double max_steer = 0.40;     // [rad] 도로휠
  double lookahead = 3.2;      // [m] 순수추종 앞 점
  double lookahead_min = 3.2;
  double k_cte = 0.5;          // 뒤차축 횡오차 약보정 (atan(k·e/Ld))
  double predict_s = 0.15;     // [s] 조향 지연 보상
  double lag_tau_s = 0.30;     // [s] 예측용 조향 1차 지연
  double slew_rad_s = 0.63;    // [rad/s] 예측용 조향 속도 상한 (≈36°/s, cmd.steer_slew)
};

// ★조향 편향 적분 [2026-09-28]★ 이 차는 pot −3° 를 물어야 직진한다(CLAUDE.md 4.1d).
// L 구간은 driving 의 트림 추정(15 s 뒤 적용)보다 먼저 오는 일이 많아, 그 편향이
// 그대로 경로 오른쪽 치우침으로 남는다(모의: 콘 여유 0.47 → 0.27 m). 경로 추종
// 오차가 작을 때만(gate) 천천히 쌓는다 — 회피 중의 큰 오차는 적분하지 않는다.
struct TrackIntegrator
{
  double value_deg = 0.0;
  void reset() { value_deg = 0.0; }
  double update(double e_m, double dt, double ki_deg, double max_deg, double gate_m, bool moving)
  {
    if (moving && ki_deg > 0.0 && std::abs(e_m) < gate_m) {
      value_deg = std::clamp(value_deg - ki_deg * e_m * dt, -max_deg, max_deg);
    }
    return value_deg;
  }
};

// ★[2026-09-29] 적분은 '곧은 경로를 곧게 따라갈 때' 만 쌓는다★
// 9/28 23:05 실주행에서 적분이 회피 기동 중에 +2.1° 까지 쌓였다 — 첫 줄을 비키느라
// 뒤처진 오차를 편향으로 읽은 것이고, 그 값이 둘째 줄로 넘어가는 반대 조향을 늦췄다.
// 편향(직진 트림)은 곧은 구간에서만 드러나므로, 지금 자리와 조금 앞이 모두 곧고
// 기준선과 나란할 때만 쌓는다(통과 오프셋을 지키는 평탄 구간은 여기에 든다).
inline bool steadyTracking(const FrenetPath & path, double s, double ahead_m = 2.0)
{
  if (!path.valid || path.pts.size() < 2) {
    return false;
  }
  constexpr double kKappaMax = 0.04;   // [1/m] R 25 m 보다 곧다
  constexpr double kSlopeMax = 0.10;   // 기준선과 5.7° 안
  for (const double ds : {0.0, ahead_m}) {
    const FrenetPoint p = path.sample(s + ds);
    if (std::abs(p.kappa) > kKappaMax || std::abs(p.d_s) > kSlopeMax) {
      return false;
    }
  }
  return true;
}

// 기준선 (s,d) 를 평면으로 보고 자세를 v·dt 만큼 굴린다.
inline OdomPose advancePose(const OdomPose & p, double v, double steer, double dt, double L)
{
  OdomPose q = p;
  q.x += v * std::cos(p.yaw) * dt;
  q.y += v * std::sin(p.yaw) * dt;
  q.yaw = wrapAngle(p.yaw + v * std::tan(steer) / L * dt);
  return q;
}

/// 도로휠각 [rad], + = 좌. `steer_now` 는 지금 내고 있는 도로휠각(지연 보상용).
inline double purePursuitSteer(
  const OdomPose & pose, const FrenetPath & path, const TrackerParams & p,
  double v, double steer_now, bool predict = true)
{
  if (!path.valid || path.pts.size() < 2) {
    return 0.0;
  }
  OdomPose q = pose;
  if (predict && p.predict_s > 1e-3 && v > 0.05) {
    constexpr int kN = 6;
    const double dt = p.predict_s / kN;
    for (int i = 0; i < kN; ++i) {
      q = advancePose(q, v, steer_now, dt, p.wheelbase);
    }
  }
  const double Ld = std::max(p.lookahead_min, p.lookahead);
  const FrenetPoint tgt = path.sample(q.x + Ld);
  double ex = 0.0;
  double ey = 0.0;
  odomToEgo(q, tgt.s, tgt.d, ex, ey);
  double delta = 0.0;
  if (ex > 0.4) {
    const double dist = std::max(2.4, std::hypot(ex, ey));
    const double alpha = std::atan2(ey, ex);
    delta = std::atan2(2.0 * p.wheelbase * std::sin(alpha), dist);
  }
  // 뒤차축이 경로 옆에 남아 있는 만큼만 약하게. 주 조향은 앞 점이다.
  const FrenetPoint rp = path.sample(q.x);
  const double e = q.y - rp.d;  // + = 차가 경로의 왼쪽 → 우조향
  delta -= std::atan2(std::max(0.0, p.k_cte) * e, Ld);
  return std::clamp(delta, -p.max_steer, p.max_steer);
}

/// 지금 자세·조향에서 경로를 따라 horizon_m 을 가 보며 차 중심선이 치사 원반에
/// 들어가는가. 원점 근처(ex < 0.5)는 자기 차체 잔상이라 보지 않는다(종전과 같다).
inline bool predictTrackHits(
  const OdomPose & pose, const FrenetPath & path, const CostmapSnapshot & snap,
  const TrackerParams & p, double v, double steer_now, double horizon_m,
  double * hit_at_m = nullptr)
{
  if (!snap.valid) {
    return false;
  }
  v = std::max(0.5, v);
  constexpr double dt = 0.05;
  const int n = std::max(4, static_cast<int>(std::ceil(horizon_m / (v * dt))));
  State st{};
  double steer = std::clamp(steer_now, -p.max_steer, p.max_steer);
  const double cy = std::cos(pose.yaw);
  const double sy = std::sin(pose.yaw);
  const double xs[5] = {-0.15, 0.35, 0.75, 1.05, 1.25};
  for (int i = 0; i < n; ++i) {
    OdomPose q;
    q.x = pose.x + st.x * cy - st.y * sy;
    q.y = pose.y + st.x * sy + st.y * cy;
    q.yaw = wrapAngle(pose.yaw + st.yaw);
    const double cmd = purePursuitSteer(q, path, p, v, steer, false);
    double want = steer + (cmd - steer) * std::min(1.0, dt / std::max(dt, p.lag_tau_s));
    const double dmax = p.slew_rad_s * dt;
    want = std::clamp(want, steer - dmax, steer + dmax);
    steer = std::clamp(want, -p.max_steer, p.max_steer);
    st.x += v * std::cos(st.yaw) * dt;
    st.y += v * std::sin(st.yaw) * dt;
    st.yaw = wrapAngle(st.yaw + v * std::tan(steer) / p.wheelbase * dt);
    for (double bx : xs) {
      double ex = 0.0;
      double ey = 0.0;
      bodyToEgo(st, bx, 0.0, ex, ey);
      if (ex < 0.5) {
        continue;
      }
      if (snap.getCost(ex, ey) >= EgoCostmap::kLethalCost * 0.99) {
        if (hit_at_m) {
          *hit_at_m = (i + 1) * v * dt;
        }
        return true;
      }
    }
  }
  return false;
}

}  // namespace mppi_local_planner
