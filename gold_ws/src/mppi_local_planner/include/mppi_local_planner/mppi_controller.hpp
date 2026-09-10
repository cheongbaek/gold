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
  // ══════════════════════════════════════════════════════════════════════
  //  ★[2026-09-11] 횡목표 — '어느 쪽으로 비킬지' 를 노드가 미리 정한다★
  // ══════════════════════════════════════════════════════════════════════
  //  ★왜 필요한가 (실측)★ route_20260910_102050-20260910_231626 :
  //      인계 시 차 y −0.36(중심선 오른쪽), 장애물 4.1m 정면(obs_y 0.00)
  //      → 플래너가 ★왼쪽★ 으로 +1.4 → +18.2° 를 냈다
  //  이유는 비용함수에 '어느 쪽으로 비킬까' 라는 개념이 아예 없기 때문이다.
  //  경로항 weight_path·y² 가 y 를 0 으로 당기는데, 차가 오른쪽에 있으니
  //  그 힘의 방향이 ★왼쪽★ 이다. 장애물이 정면이라 좌우 대칭이면 그 경로항이
  //  타이브레이커가 되어 ★장애물 쪽으로 먼저 꺾는다.★ 그 뒤 장애물이 왼쪽으로
  //  드러나면(obs_y +0.40) 이미 늦어서 크게 되꺾는다 — 관측된 그대로다.
  //
  //  ★고치는 방법★ 비킬 쪽을 노드가 기하로 먼저 정하고(선택 근거는 노드의
  //  updateLateralTarget), 그 목표 횡위치를 여기로 넘긴다. 경로항은 y 가 아니라
  //  ★(y − lateral_target)★ 을 0 으로 당긴다 — 즉 '중심선' 이 아니라
  //  '비켜 갈 자리' 를 겨눈다. 장애물이 없으면 lateral_target = 0 이라
  //  종전과 완전히 같다(중심선 복귀).
  //  ★횡벽(max_lateral_offset)은 그대로 절대값 y 에 건다★ 목표가 어디든
  //  중심선에서 그 이상 벗어나면 안 된다는 뜻은 변하지 않는다.
  double lateral_target = 0.0;        // [m] + 왼쪽. 노드가 매 틱 갱신한다
  // ══════════════════════════════════════════════════════════════════════
  //  ★[2026-09-11] 완만한 S — 두 개의 벽 (사용자 지시)★
  // ══════════════════════════════════════════════════════════════════════
  //  ★실측 route_20260910_102050-20260910_232807★ 주행은 마쳤지만
  //      ld_y  −2.96 ~ +3.59 m  (진폭 ★6.55 m★)
  //      yaw   −48° ~ +73°
  //      road  ±22.9° = max_steering_angle 에 ★플래너가 스스로 포화★
  //  종전 max_lateral_offset 1.00 · weight_lateral_wall 12 는 전혀 안 들었다 —
  //  장애물 비용이 수백까지 가는데 벽은 |y|=2 에서 12·1²=12 뿐이라 묻힌다.
  //
  //  ★그런데 진짜 원인은 벽이 아니라 stanley_lookahead 였다★
  //      target_yaw = atan(횡오차 / L_stan),  L_stan 이 2.0 m 였다
  //      횡오차 1.0m → 목표 헤딩 ★26.6°★ / 3.5m → ★60.3°★
  //  즉 조금만 벗어나도 '아주 비스듬히 되돌아가라' 를 목표로 삼고, 그 헤딩이
  //  실제로 만들어지면 횡위치가 그 각도로 계속 달아난다. 헤딩 ±73° 가 그것이다.
  //  L_stan 을 4.0 m 로 늘리면 같은 1.0m 에 목표가 ★14°★ — 그것이 완만한 S 다.
  //  (구 값 2.0 은 1/5카 1.2 를 축거비로 스케일한 것이고, 그 차는 코스가 훨씬
  //   촘촘했다. 이 차는 최소회전반경 2.02 m 라 급한 S 자체가 불가능하다.)
  //
  //  ★그래도 벽 둘을 세운다 — 되돌아오는 중에도 한계는 있어야 한다★
  //   ① 소프트 벽 (max_lateral_offset) : 여기서부터 되돌리는 힘이 커진다
  //   ② ★하드 벽 (lateral_hard)★ : 사용자가 정한 '넘을 필요 없는' 선.
  //      넘으면 길을 벗어날 위험이 있으므로 비용을 압도적으로 준다.
  //   ③ 헤딩 벽 : S 를 완만하게 만드는 직접적인 수단. 횡위치는 헤딩의 적분이라,
  //      헤딩을 묶으면 횡위치가 달아나는 속도 자체가 묶인다.
  double lateral_hard = 2.5;          // [m] ★이 이상은 벗어날 필요가 없다★
  double weight_lateral_hard = 4000.0;
  double max_heading_dev = 0.52;      // [rad] ≈30°. 기준선 대비 헤딩 허용치
  double weight_heading_wall = 4000.0;
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

  // Drops the warm-started control sequence. [2026-09-01]
  //  ★Call this whenever planning restarts after a gap★ — nominal_ is warm
  //  started from the previous cycle, so after minutes of silence (GPS 추종이
  //  차를 몰던 구간) it holds a steering/speed plan for a pose that no longer
  //  exists. Without this the first cycle after a handover steers by that
  //  stale plan; the cost function then only nudges it, so the error decays
  //  over several cycles instead of being absent.
  void reset();

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
