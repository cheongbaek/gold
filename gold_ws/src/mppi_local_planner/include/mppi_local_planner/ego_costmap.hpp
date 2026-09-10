#pragma once
#include <cmath>
#include <mutex>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace mppi_local_planner
{

struct CostmapParams
{
  double size_x = 12.0;         // [m] total grid extent along x (forward/back), robot-centered
  double size_y = 12.0;         // [m] total grid extent along y (left/right)
  double resolution = 0.1;      // [m/cell]

  // Ground / overhang height filter, in the SENSOR's own frame (before offset correction
  // below is applied to x/y only -- z filtering happens pre-offset, tune to mount height).
  double ground_z_min = -0.25;  // points below this z are treated as ground and dropped
  double ground_z_max = 0.05;   // points above this z are treated as overhead and dropped

  double robot_half_width = 0.45;  // lethal radius (~track_width/2 + margin)
  double inflation_radius = 0.8;   // extra radius over which cost decays smoothly to 0

  // Points closer than this to the ego origin are dropped entirely, in the
  // corrected base frame (post sensor_offset/yaw_offset).
  double min_range = 0.3;

  // Static sensor extrinsics relative to the vehicle's base/ego frame (x fwd, y left).
  double sensor_offset_x = 0.0;
  double sensor_offset_y = 0.0;
  // [rad] Ouster os_lidar -> os_sensor yaw correction (typically M_PI).
  double sensor_yaw_offset = M_PI;

  // Temporal fusion: blend previous occupancy with the latest scan so sparse
  // returns / single-frame dropouts do not flicker the map or false-clear
  // the reference-reset corridor. 0 = no memory (replace every scan),
  // 0.7 keeps ~70% of previous occupancy when a cell is free this frame.
  double occupancy_decay = 0.6;

  // Occupancy self-clear (circle, mount / very near body). Keep SMALL so nearby
  // 라바콘 are not erased. Oversized clear was masking close cones → stop-gate.
  double ego_clear_radius = 0.50;

  // Rectangular occupancy clear (rear-axle origin, x forward / y left).
  // MUST stay tight: large x_max hid near cones inside a white free square on RViz.
  // Disabled when x_max <= x_min.
  double ego_clear_x_min = -0.25;  // behind rear axle [m]
  double ego_clear_x_max = 0.45;   // ahead of rear axle [m] — hard-capped by front_max
  double ego_clear_y_half = 0.40;  // half-width of clear box [m]
  // ══════════════════════════════════════════════════════════════════════
  //  ★[2026-09-11] 지나친 라바콘이 복귀를 막는다 — 그것을 지운다★
  // ══════════════════════════════════════════════════════════════════════
  //  ★실측 route_20260910_102050-20260911_001017★
  //    t=16.9~17.6 : 콘이 차 ★왼쪽 옆★ (x 0.3~0.4, y +1.1) 에 붙어 있는 동안
  //      road   −7.5 → −1.9 → +0.9 → +0.3 → +0.1   (조향이 0 으로 죽는다)
  //      ld_y   −1.56 → −2.31,  yaw −34° → −43°
  //      cost      83 →  3169   → 정지 래치까지 걸렸다
  //  차가 −2.3 m 에 헤딩 −43° 인데 ★왼쪽으로 되돌리지 않는다.★ 이유는 명확하다 —
  //  왼쪽으로 도는 롤아웃은 차체를 그 콘 쪽으로 쓸고 지나가므로 충돌비용이
  //  폭증한다. ★이미 옆에 있는 콘은 조향으로 피할 수 없는데도 계속 피하려 든다.★
  //
  //  ★사용자 지시★ "라바콘을 지나치는 '즉시' 그 반대쪽으로 꺾는다. 그대로
  //  벗어나지 말고." → 지나친 콘은 판단에서 빠져야 한다.
  //
  //  ★그런데 앞쪽까지 넓게 지우면 안 된다★ 아직 피해야 할 콘까지 지운다.
  //  그래서 ★앞차축 뒤에서만★ 넓게 지운다 — 그 뒤에 있는 것은 조향으로
  //  피할 수 있는 대상이 아니다(앞바퀴가 이미 지나갔다). 앞쪽 상자는 종전 그대로.
  //     x ∈ [x_min, pass_x] 에서 반폭 = ego_clear_y_half_passed
  //     x ∈ (pass_x, x_max]  에서 반폭 = ego_clear_y_half        (종전)
  //  0 이면 이 기능은 꺼진다(종전 거동).
  double ego_clear_pass_x = 0.0;        // [m] 이 앞쪽은 종전대로 (0 = 끔)
  double ego_clear_y_half_passed = 0.0; // [m] 그 뒤쪽의 넓은 반폭

  // After inflation, only wipe a SMALL circle on the cost layer (inflation bleed
  // into the origin). Do NOT re-apply the full occupancy rect here — that was
  // deleting cost of 라바콘 sitting just outside the body.
  // 0 => reuse ego_clear_radius. Typical 0.35–0.50.
  double ego_cost_clear_radius = 0.42;
};

// Lock-free read-only view of the cost grid for one planning cycle.
// Built once under the costmap mutex; rollouts query this without locking.
struct CostmapSnapshot
{
  std::vector<float> cost;
  int cells_x = 0;
  int cells_y = 0;
  double size_x = 0.0;
  double size_y = 0.0;
  double resolution = 0.1;
  bool valid = false;

  double getCost(double x, double y) const
  {
    if (!valid) {
      return 0.0;
    }
    const int ix = static_cast<int>(std::floor((x + size_x / 2.0) / resolution));
    const int iy = static_cast<int>(std::floor((y + size_y / 2.0) / resolution));
    if (ix < 0 || ix >= cells_x || iy < 0 || iy >= cells_y) {
      // Outside the local map -> discourage rollouts from leaving the sensed area.
      return 200.0;
    }
    return cost[static_cast<size_t>(iy) * static_cast<size_t>(cells_x) + static_cast<size_t>(ix)];
  }
};

// Ego-centric occupancy/cost grid, rebuilt from the latest LiDAR scan every time a
// new point cloud arrives. Always centered on the robot's current position, x
// forward / y left, matching the ego frame used by the MPPI rollouts.
class EgoCostmap
{
public:
  explicit EgoCostmap(const CostmapParams & params);

  // Rebuild the grid from a raw PointCloud2 (fields x, y, z, float32).
  void updateFromPointCloud(const sensor_msgs::msg::PointCloud2 & cloud);

  // Obstacle cost at an ego-frame (x, y). Prefer snapshot() inside tight loops.
  double getCost(double x, double y) const;

  // Copy cost grid under lock once per control cycle for lock-free lookups.
  CostmapSnapshot snapshot() const;

  bool hasData() const;

  // Export the current cost grid for RViz. header.frame_id/stamp left for caller.
  nav_msgs::msg::OccupancyGrid toOccupancyGrid() const;

  // Lethal cost value stamped inside the inflated robot footprint.
  static constexpr float kLethalCost = 1000.0f;

private:
  int worldToIndex(double val, double size, double resolution) const;
  void inflate();
  // Occupancy: tight circle + short body rect (self-hits only).
  void clearEgoOccupancy(std::vector<float> & grid) const;
  // Cost layer: small circle only (do not wipe near-cone costs).
  void clearCircle(std::vector<float> & grid, double radius) const;

  CostmapParams params_;
  int cells_x_;
  int cells_y_;
  std::vector<float> occupancy_;  // soft occupancy [0,1] with temporal decay
  std::vector<float> cost_;       // inflated cost derived from occupancy_
  mutable std::mutex mutex_;
  bool has_data_ = false;
};

}  // namespace mppi_local_planner
