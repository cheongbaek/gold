#pragma once
#include <cmath>
#include <vector>

#include "mppi_local_planner/ego_costmap.hpp"
#include "mppi_local_planner/frenet_types.hpp"

namespace mppi_local_planner
{

// ══════════════════════════════════════════════════════════════════════════
//  ★[2026-09-28] 장애물 군집은 ★점유 셀★ 로 만든다 — 치사층이 아니다★
// ══════════════════════════════════════════════════════════════════════════
//  종전 detectCones 는 치사층(점마다 반경 0.62 m 원반)을 8-이웃으로 묶었다.
//  그래서 ① 1.2 m 안의 콘이 한 덩어리가 되고 ② 덩어리 가장자리가 콘보다
//  0.62 m 바깥에 섰다. 가로 3개 블록이 폭 2.3 m 덩어리가 되자 모서리 네 점을
//  '콘' 으로 내보냈고(9/28 작업트리), 계획기가 그 모서리에 여유 1.26 m 를
//  ★또★ 더해 목표가 −2.87 m 까지 나갔다(실측 route_…-20260928_213917).
//  → 라이다가 실제로 본 점유 셀을 묶는다. 경계 = 장애물 표면이다.

// 차체 기준 (x 앞 / y 왼쪽, 원점 = 뒤차축). 경계는 점유 셀의 바깥 모서리다.
struct ObstacleCluster
{
  double x0 = 0.0;
  double x1 = 0.0;
  double y0 = 0.0;
  double y1 = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  int cells = 0;
};

// /lidar_cones 한 항목 (x, y, cells) — white1/record 의 .cones.csv 규약.
struct ConeObs
{
  double x = 0.0;
  double y = 0.0;
  int cells = 0;
};

// /lidar_diag [11..13]
struct ConeStats
{
  int lethal_cells = 0;       // 치사 셀 총수 — 0 이면 점군 자체가 없다
  int clusters_all = 0;       // 찾은 점유 군집 수 (거르기 전)
  int clusters_rejected = 0;  // 너무 작거나 너무 넓어서 버린 군집 수
};

struct ClusterParams
{
  // 이 칸 수(체비셰프 거리) 안의 점유 셀은 같은 군집. 0.1 m 격자에서 3 = 0.3 m.
  // 콘 하나의 점은 서로 0.1~0.2 m 라 붙고, 0.5 m 간격 콘은 붙거나 떨어져도
  // 블록 묶음(frenet.block_link_*)이 어차피 하나로 본다.
  int link_cells = 3;
  int min_cells = 1;
  double x_min = -0.6;
  double x_max = 17.5;
  double y_abs_max = 7.5;
  // 이보다 가로로 넓으면 콘이 아니라 지면·벽이다 — 회피 계획에서 뺀다.
  // ★정지는 따로 본다★ 노드의 정지 판정은 이 군집이 아니라 치사층 전체를 본다.
  double max_width_m = 5.0;
};

std::vector<ObstacleCluster> clusterOccupancy(
  const CostmapSnapshot & snap,
  const ClusterParams & params,
  ConeStats * stats = nullptr);

// Ahead of pass_x and inside the mapped lane in reference-d.
// Do not require ego |y| ≤ y_max: after a heading swing the next cone's
// ego-y exceeds 2 m while its d is still in-lane.
inline bool coneInAvoidLane(
  const OdomPose & odom,
  const ConeObs & c,
  double pass_x,
  double range,
  double y_max,
  double ego_y_loose = 5.0)
{
  if (c.x < pass_x || c.x > range) {
    return false;
  }
  if (std::abs(c.y) > ego_y_loose) {
    return false;
  }
  double s = 0.0;
  double d = 0.0;
  egoToOdom(odom, c.x, c.y, s, d);
  (void)s;
  return std::abs(d) <= y_max;
}

}  // namespace mppi_local_planner
