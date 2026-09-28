#pragma once
#include <string>
#include <vector>

#include "mppi_local_planner/cone_detector.hpp"
#include "mppi_local_planner/frenet_types.hpp"
#include "mppi_local_planner/vehicle_model.hpp"

namespace mppi_local_planner
{

// ══════════════════════════════════════════════════════════════════════════
//  ★[2026-09-28] 블록 계획기 — 설계 근거는 frenet_planner.cpp 머리 상자★
// ══════════════════════════════════════════════════════════════════════════
struct FrenetPlannerParams
{
  bool enable = true;
  double plan_distance_m = 16.0;  // 경로는 뒤차축에서 최소 이만큼 앞까지
  double ds_m = 0.25;

  // ── 검출 ──
  double range_m = 17.5;          // 차체 x 이 안의 점유 군집만 본다
  int cluster_link_cells = 3;     // 0.1 m 격자 3칸 = 0.3 m 안의 점은 한 군집
  int cluster_min_cells = 1;
  double cluster_max_width_m = 5.0;

  // ── 블록 (한 덩어리로 돌아갈 장애물) ──
  double block_link_s_m = 2.5;    // 진행방향 틈이 이보다 작으면 사이로 못 누빈다
  double block_link_d_m = 1.9;    // 가로 틈이 이보다 작으면 사이로 못 지나간다
  int confirm_hits = 2;           // 이만큼 연속으로 보여야 계획에 넣는다 (잡음 한 점 거름)
  double lost_s = 2.5;            // 보여야 할 자리인데 이만큼 안 보이면 버린다
  double blind_x_m = 2.2;         // 차체 x 이보다 가까우면 안 보이는 게 정상 (자기 차체 지움)
  double assoc_gap_m = 1.0;       // 새 관측을 기억 블록에 붙이는 허용 틈

  // ── 통과 ──
  double max_offset_m = 3.0;      // |d| 상한 = GPS 궤적 ± 3 m (사용자 지시)
  double pass_gap_m = 0.45;       // 차체 옆면 ↔ 장애물 표면
  double hold_pre_m = 0.30;       // 장애물 앞면 이만큼 앞에서 통과 d 에 도착 (뒤차축 기준)
  double hold_post_m = 0.30;      // 장애물 뒷면을 이만큼 지날 때까지 유지
  double bias_deadband_m = 0.15;  // 블록 중심 |d| 가 이 안이면 '가운데'
  // 멀수록 넓힌다 — 헤딩 1.7° 오차면 거리 1 m 당 횡위치 3 cm 가 틀린다.
  double bias_deadband_per_m = 0.03;
  // '가운데' 블록은 이웃 블록과의 상대 위치로 정한다 (블록은 좌↔우로 번갈아 놓인다).
  double rel_margin_m = 0.40;
  double commit_dist_m = 5.0;     // 앞범퍼 ~ 블록 앞면이 이 안이면 쪽을 확정한다
  int flip_ticks = 5;             // 확정 전, 규칙이 이만큼 연속 반대를 말해야 바꾼다

  // ── 경로 모양 (5차 다항식 구간) ──
  double kappa_comfort = 0.125;   // [1/m] 교대·복귀 길이를 정하는 곡률 (R 8 m)
  double kappa_max = 0.34;        // [1/m] 이보다 굽으면 '빡빡' (확정 전이면 반대쪽 검토)
  double shift_len_min_m = 4.0;
  double shift_len_max_m = 12.0;
  double return_len_min_m = 6.0;
  double return_len_max_m = 12.0;
  double flat_min_m = 2.0;        // 두 블록 사이에서 궤적(0)으로 돌아왔다 가려면 남아야 할 평지
  double entry_len_min_m = 1.5;   // 이미 창 안에 있으면 이 거리 안에 통과 d 로

  // ── 시작 상태 ──
  double on_path_d_m = 0.30;      // 차가 직전 경로에서 이 안이면 그 경로에서 이어 그린다
  double on_path_yaw_rad = 0.15;
};

// 진단·그림·로그용으로 내보내는 블록 (기준선 s, d)
struct PlanBlock
{
  int id = 0;
  double s0 = 0.0;
  double s1 = 0.0;
  double d0 = 0.0;
  double d1 = 0.0;
  int side = 0;          // +1 왼쪽으로 통과 / -1 오른쪽으로 통과
  bool confirmed = false;
  bool committed = false;
  double target = 0.0;   // 이 블록 옆에서 뒤차축이 있을 d
};

struct PlanDiag
{
  int n_blocks = 0;         // 앞에 남은 확정 블록 수
  double target_d = 0.0;    // 다음 블록의 통과 d (없으면 0)
  double max_kappa = 0.0;   // 경로 최대 |κ| [1/m]
  double min_clear_m = 99.0;  // 경로를 따라간 차체 ↔ 블록 최소 여유
  bool tight = false;       // max_kappa > kappa_max
};

// 콘·블록을 기준선 (s,d) 에 올리고 5차 다항식 d(s) 를 만든다. 조향은 내지 않는다.
class FrenetPlanner
{
public:
  FrenetPlanner(const FrenetPlannerParams & params, const VehicleParams & vehicle);

  void setParams(const FrenetPlannerParams & params) { params_ = params; }

  void reset();

  // kappa0 = 지금 조향이 만드는 곡률 [1/m] (+ 왼쪽). 모르면 0.
  FrenetPath plan(
    const OdomPose & odom, const CostmapSnapshot & snap, double now_s, double kappa0);

  const ConeStats & lastStats() const { return last_stats_; }
  const std::vector<ObstacleCluster> & lastClustersEgo() const { return last_clusters_; }
  const std::vector<PlanBlock> & blocks() const { return out_blocks_; }
  const PlanDiag & lastDiag() const { return diag_; }
  // 블록 발견·쪽 결정·통과 같은 드문 사건. 노드가 한 번씩 로그로 낸다.
  std::vector<std::string> takeEvents();

private:
  struct Box
  {
    double s0 = 0.0;
    double s1 = 0.0;
    double d0 = 0.0;
    double d1 = 0.0;
  };

  struct Block
  {
    int id = 0;
    Box box;                    // 전체 외곽 (쪽 결정·로그)
    //  ★구성 군집 상자★ 통과 창은 이것으로 만든다. 외곽 하나로 보면 'ㄱ' 자 배치
    //  (가로 줄 + 길게 이어진 가장자리 콘)가 줄 옆 32 m 전체를 막는 상자로 부푼다.
    std::vector<Box> parts;
    int side = 0;
    int hits = 0;
    int disagree = 0;
    bool confirmed = false;
    bool committed = false;
    bool feas_locked = false;   // 곡률 때문에 바꾼 쪽 — 규칙으로 되돌리지 않는다
    bool announced = false;     // 궤적을 침범한다고 알렸다 (로그 한 번)
    double first_t = 0.0;
    double last_t = 0.0;
  };

  struct Group
  {
    double a = 0.0;   // 창 시작 (뒤차축 s)
    double b = 0.0;   // 창 끝
    double lo = -1e9;
    double hi = 1e9;
    double target = 0.0;
    bool active = true;         // 통과 d 가 0 이 아니다 (궤적에서 비켜야 한다)
    std::vector<int> members;   // mem_ 인덱스
  };

  // 궤적을 침범하지 않는 군집의 제약 — 창을 만들지 않고, 경로가 넘어갈 때만 묶는다.
  struct Bound
  {
    double a = 0.0;
    double b = 0.0;
    double lo = -1e9;
    double hi = 1e9;
  };

  struct Knot
  {
    double s = 0.0;
    double d = 0.0;
    double d1 = 0.0;   // dd/ds
    double d2 = 0.0;   // d²d/ds²
  };

  double clearance() const;
  // 궤적(0)에서 clearance 안으로 들어와 있다 = 실제로 비켜야 하는 블록.
  // 차로 가장자리 콘·벽처럼 궤적에서 먼 것은 쪽 결정·교대·통과 창에 끼지 않는다.
  bool intrudes(const Box & b) const;
  double bodyFront() const;
  double bodyRear() const;
  // 규칙이 원하는 쪽 (+1 왼쪽 / −1 오른쪽). why 에 근거를 적는다.
  // strong = false 면 약한 보조 규칙(차가 있는 쪽)이라 이미 정한 쪽을 뒤집지 않는다.
  int desiredSide(const std::vector<int> & order, size_t pos, int prev,
                  const OdomPose & odom, const char ** why, bool * strong) const;
  void observe(const OdomPose & odom, const CostmapSnapshot & snap, double now_s);
  void prune(const OdomPose & odom, double now_s);
  void decideSides(const OdomPose & odom);
  std::vector<Group> makeGroups(std::vector<Bound> * bounds) const;
  void mergeAndTarget(std::vector<Group> & groups, const std::vector<Bound> & bounds) const;
  Knot startState(const OdomPose & odom, double kappa0) const;
  FrenetPath buildPath(const Knot & start, const std::vector<Group> & groups) const;
  void appendQuintic(FrenetPath & path, const Knot & a, const Knot & b) const;
  void appendFlat(FrenetPath & path, double s_from, double s_to, double d) const;
  double entryKappa(const Knot & start, const Group & g) const;
  void fillDiag(const OdomPose & odom, const FrenetPath & path, const std::vector<Group> & groups);
  void exportBlocks(const std::vector<Group> & groups);
  void event(const std::string & msg) { events_.push_back(msg); }

  FrenetPlannerParams params_;
  VehicleParams vehicle_;
  ConeStats last_stats_{};
  std::vector<ObstacleCluster> last_clusters_;
  std::vector<Block> mem_;
  std::vector<PlanBlock> out_blocks_;
  std::vector<std::string> events_;
  FrenetPath last_path_;
  PlanDiag diag_{};
  int next_id_ = 1;
  int last_side_ = 0;          // 마지막으로 지나친 블록의 쪽 (가운데 블록의 교대 규칙)
  double last_plan_t_ = -1.0;
};

}  // namespace mppi_local_planner
