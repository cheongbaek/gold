#include "mppi_local_planner/frenet_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

// ══════════════════════════════════════════════════════════════════════════
//  ★★ [2026-09-28] 블록 계획기로 다시 짰다 — 오늘 네 주행이 전부 섰다 ★★
// ══════════════════════════════════════════════════════════════════════════
//  실측 route_20260916_211406-20260928_{211444,212000,212902,213917} :
//  L 구간 진입 4~5초 만에 네 번 모두 stop latch → 리니어 2단으로 서서 사람이
//  끌 때까지 그대로였다. 배치는 사용자 그림과 같다 — 가로 3개 블록 두 줄,
//  첫 줄은 궤적 가운데~오른쪽, 8 m 뒤 둘째 줄은 가운데~왼쪽.
//
//    ① ★계획을 인계 순간에야 시작했다★ 라이다는 인계 순간 이미 16 m 앞의 둘째
//       줄까지 보고 있었는데(15.7~15.8 m), 노드가 허락 전에는 아무것도 안 해서
//       첫 줄을 7.5 m 앞에서 처음 '봤다'. → 인계 전 미리보기(노드 preview.*).
//    ② ★통과 오프셋에 콘 4.5 m 앞에서 도착하려 했다★ (arrive_before 4.5)
//       7.5 m 앞에서 1.6 m 를 비키려면 3 m 안에 해야 하니 곡선이 R 1.2 m —
//       이 차는 R 2.96 m(조향 상한 0.40 rad) 밑으로 못 돈다. 못 따라가서 늦게
//       크게 돌았고, 그것이 '바로 앞에서 꺾고 결국 크게 돈다' 의 정체다.
//    ③ ★옆으로 1.11 m 밖에 못 나갔다★ 차선 반폭 1.75 모델(laneMaxOffset).
//       첫 줄을 오른쪽으로 돌려면 −1.95 m 가 필요했는데 −1.11 에 잘려 경로가
//       콘 가장자리를 관통했고, 정지 판정이 차를 세웠다.
//    ④ ★쪽이 틱마다 뒤집혔다★ 치사층 덩어리의 모서리 네 점을 '콘' 으로 내보내고
//       (가장자리가 0.62 m 부풀어 있다) 여유 1.26 m 를 또 더해 목표가 −2.87 까지
//       갔다. 21:14 주행은 목표가 −1.11 ↔ +1.11 로 왕복했다.
//
//  ★새 구조★
//    검출  점유 셀(라이다가 실제로 본 점)을 0.3 m 로 묶는다 — cone_detector.hpp
//    블록  진행 2.5 m · 가로 1.9 m 보다 가까운 군집은 한 덩어리 (사이로 못 지나간다)
//          기억은 기준선 (s,d) 에 둔다 — 코앞에서 자기 차체 지움에 가려져도 남는다
//    쪽    ★궤적 기준 치우친 쪽의 반대★ (사용자 규칙). 블록 중심 d < 0 이면 왼쪽.
//          이것은 '|통과 d| 가 작은 쪽' 과 수학적으로 같다(d_max+c < −d_min+c ⇔ 중심<0).
//          가운데면 앞 블록의 반대(교대 배치) → 없으면 차가 있는 쪽.
//          앞범퍼에서 commit_dist 안이면 확정. 그 전에는 규칙이 flip_ticks 연속으로
//          반대를 말해야 바꾼다(먼 거리 부분 관측으로 정한 것을 바로잡는다).
//    창    뒤차축이 [앞면 − 0.4, 뒷면 + 0.4] 에서 통과 d 를 지킨다.
//          ★앞면 − 차 길이가 아니다★ 차체는 경로 접선을 따라 서 있고, 통과 d 로
//          올라가며 평평해지는 곡선(오목)의 접선은 곡선보다 바깥이라, 뒤차축이
//          앞면에서 통과 d 에 닿으면 앞범퍼는 이미 그 바깥이다. 빠져나올 때도
//          같은 이유로 뒷면까지만 지키면 된다(뒤 오버행은 반대쪽으로 돈다).
//          이것이 둘째 줄까지의 교대 거리를 1.25 m 늘려 준다.
//    경로  5차 다항식(최소 저크)을 이어 붙인다 — d, 기울기, 곡률이 모두 연속이라
//          조향이 0 에서 시작해 0 으로 끝난다. 구간 길이는 쓸 수 있는 거리를
//          ★전부★ 쓴다(= 가장 완만하다). 오늘 배치 검산 :
//            인계 (d −0.31, ψ −8°) → 첫 줄 왼쪽 +1.20 : 7.7 m, R 4.65 m, 조향 15°
//            첫 줄 +1.20 → 둘째 줄 오른쪽 −0.93      : 7.1 m, R 4.42 m, 조향 16°
//          복귀는 R 8 m 로 6~12 m. 5차 다항식은 단조라 궤적을 넘어가지 않는다.
//    폭    |d| ≤ 3.0 m (GPS 궤적 ± 3 m, 사용자 지시). 차선 모델은 쓰지 않는다.
// ══════════════════════════════════════════════════════════════════════════

namespace mppi_local_planner
{

namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();

// 5차 다항식 한 구간. u = (s − s0)/L 에서 d(u) = Σ c_i u^i.
struct Quintic
{
  double s0 = 0.0;
  double L = 1.0;
  double c[6] = {0, 0, 0, 0, 0, 0};

  void eval(double s, double & d, double & d1, double & d2) const
  {
    const double u = std::clamp((s - s0) / L, 0.0, 1.0);
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double u4 = u3 * u;
    const double u5 = u4 * u;
    d = c[0] + c[1] * u + c[2] * u2 + c[3] * u3 + c[4] * u4 + c[5] * u5;
    d1 = (c[1] + 2 * c[2] * u + 3 * c[3] * u2 + 4 * c[4] * u3 + 5 * c[5] * u4) / L;
    d2 = (2 * c[2] + 6 * c[3] * u + 12 * c[4] * u2 + 20 * c[5] * u3) / (L * L);
  }
};

Quintic makeQuintic(
  double sa, double da, double va, double aa,
  double sb, double db, double vb, double ab)
{
  Quintic q;
  q.s0 = sa;
  q.L = std::max(1e-3, sb - sa);
  const double L = q.L;
  q.c[0] = da;
  q.c[1] = va * L;
  q.c[2] = 0.5 * aa * L * L;
  const double D = db - (q.c[0] + q.c[1] + q.c[2]);
  const double V = vb * L - (q.c[1] + 2 * q.c[2]);
  const double A = ab * L * L - 2 * q.c[2];
  q.c[3] = 10 * D - 4 * V + 0.5 * A;
  q.c[4] = -15 * D + 7 * V - A;
  q.c[5] = 6 * D - 3 * V + 0.5 * A;
  return q;
}

// 기울기 0 → 0 인 5차 다항식의 최대 곡률은 ≈ 5.77·Δ/L². 원하는 곡률에서 길이를 푼다.
double lenForShift(double delta, double kappa, double lo, double hi)
{
  const double L = std::sqrt(5.77 * std::abs(delta) / std::max(kappa, 1e-3));
  return std::clamp(L, lo, hi);
}

double kappaOf(double d1, double d2)
{
  return d2 / std::pow(1.0 + d1 * d1, 1.5);
}

// 점 → 축정렬 상자 거리. 안쪽이면 음수(가장 가까운 변까지의 거리).
double pointBoxDist(double s, double d, double s0, double s1, double d0, double d1)
{
  const double ds = std::max({s0 - s, 0.0, s - s1});
  const double dd = std::max({d0 - d, 0.0, d - d1});
  if (ds > 0.0 || dd > 0.0) {
    return std::hypot(ds, dd);
  }
  return -std::min({s - s0, s1 - s, d - d0, d1 - d});
}

const char * sideName(int side)
{
  return side > 0 ? "왼쪽" : (side < 0 ? "오른쪽" : "미정");
}

}  // namespace

FrenetPlanner::FrenetPlanner(
  const FrenetPlannerParams & params, const VehicleParams & vehicle)
: params_(params), vehicle_(vehicle)
{
}

void FrenetPlanner::reset()
{
  last_stats_ = {};
  last_clusters_.clear();
  mem_.clear();
  out_blocks_.clear();
  last_path_ = {};
  diag_ = {};
  last_side_ = 0;
  last_plan_t_ = -1.0;
}

std::vector<std::string> FrenetPlanner::takeEvents()
{
  std::vector<std::string> out;
  out.swap(events_);
  return out;
}

double FrenetPlanner::clearance() const
{
  return 0.5 * vehicle_.track_width + params_.pass_gap_m;
}

bool FrenetPlanner::intrudes(const Box & b) const
{
  const double c = clearance();
  return b.d0 < c && b.d1 > -c;
}

double FrenetPlanner::bodyFront() const
{
  return vehicle_.wheelbase + vehicle_.front_overhang;
}

double FrenetPlanner::bodyRear() const
{
  return vehicle_.rear_overhang;
}

int FrenetPlanner::desiredSide(
  const std::vector<int> & order, size_t pos, int prev,
  const OdomPose & odom, const char ** why, bool * strong) const
{
  *strong = true;
  const Block & b = mem_[static_cast<size_t>(order[pos])];
  const double c = 0.5 * (b.box.d0 + b.box.d1);
  const double x = b.box.s0 - odom.x;
  //  ① ★궤적 기준 치우친 쪽의 반대★ (사용자 규칙). 멀수록 '가운데' 폭을 넓힌다.
  const double db = params_.bias_deadband_m +
    params_.bias_deadband_per_m * std::max(0.0, x - 6.0);
  if (c < -db) {
    *why = "궤적 오른쪽으로 치우침";
    return +1;
  }
  if (c > db) {
    *why = "궤적 왼쪽으로 치우침";
    return -1;
  }
  //  ② 가운데면 ★이웃 블록과의 상대 위치★ — 블록은 좌↔우로 번갈아 놓인다(사용자).
  //     헤딩·GPS 오차는 두 블록을 같은 쪽으로 옮기므로 차이는 남는다.
  const Block * nb = nullptr;
  double best = 1e9;
  for (size_t k = 0; k < order.size(); ++k) {
    if (k == pos) {
      continue;
    }
    const Block & o = mem_[static_cast<size_t>(order[k])];
    if (!intrudes(o.box)) {
      continue;                        // 궤적에서 먼 것(가장자리 콘·벽)은 비교하지 않는다
    }
    const double ds = std::abs(o.box.s0 - b.box.s0);
    if (ds < best && ds < 16.0) {
      best = ds;
      nb = &o;
    }
  }
  if (nb) {
    const double cn = 0.5 * (nb->box.d0 + nb->box.d1);
    if (cn - c >= params_.rel_margin_m) {
      *why = "가운데 — 이웃 블록보다 오른쪽";
      return +1;
    }
    if (c - cn >= params_.rel_margin_m) {
      *why = "가운데 — 이웃 블록보다 왼쪽";
      return -1;
    }
  }
  //  ③ 앞 블록의 반대 → ④ 차가 있는 쪽.
  if (prev != 0) {
    *why = "가운데 — 앞 블록의 반대";
    return -prev;
  }
  //  ★약한 규칙★ — 차가 궤적을 가로지를 때마다 뒤집히면 안 되므로 처음 정할 때만 쓴다.
  *strong = false;
  *why = "가운데 — 차가 있는 쪽";
  return (odom.y >= 0.0) ? +1 : -1;
}

void FrenetPlanner::observe(const OdomPose & odom, const CostmapSnapshot & snap, double now_s)
{
  ClusterParams cp;
  cp.link_cells = params_.cluster_link_cells;
  cp.min_cells = params_.cluster_min_cells;
  cp.x_min = 0.0;   // 뒤차축 뒤의 새 관측은 계획에 쓸 데가 없다 (지난 것은 기억에 있다)
  cp.x_max = params_.range_m;
  cp.y_abs_max = std::min(0.5 * snap.size_y - 0.1,
                          params_.max_offset_m + clearance() + 2.5);
  cp.max_width_m = params_.cluster_max_width_m;
  last_clusters_ = clusterOccupancy(snap, cp, &last_stats_);

  //  기준선 (s,d) 상자로 옮긴다. 회피 폭(±3 m) 바깥은 계획에 영향이 없다.
  const double band = params_.max_offset_m + clearance() + 0.5;
  std::vector<Box> boxes;
  boxes.reserve(last_clusters_.size());
  for (const auto & c : last_clusters_) {
    Box b{kInf, -kInf, kInf, -kInf};
    const double xs[2] = {c.x0, c.x1};
    const double ys[2] = {c.y0, c.y1};
    for (double x : xs) {
      for (double y : ys) {
        double s = 0.0;
        double d = 0.0;
        egoToOdom(odom, x, y, s, d);
        b.s0 = std::min(b.s0, s);
        b.s1 = std::max(b.s1, s);
        b.d0 = std::min(b.d0, d);
        b.d1 = std::max(b.d1, d);
      }
    }
    if (b.d1 < -band || b.d0 > band) {
      continue;
    }
    boxes.push_back(b);
  }

  //  ★블록으로 묶는다★ 사이로 못 지나갈 만큼 가까우면 한 덩어리다.
  const int n = static_cast<int>(boxes.size());
  std::vector<int> parent(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    parent[static_cast<size_t>(i)] = i;
  }
  auto root = [&](int i) {
    while (parent[static_cast<size_t>(i)] != i) {
      parent[static_cast<size_t>(i)] = parent[static_cast<size_t>(parent[static_cast<size_t>(i)])];
      i = parent[static_cast<size_t>(i)];
    }
    return i;
  };
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      const Box & a = boxes[static_cast<size_t>(i)];
      const Box & b = boxes[static_cast<size_t>(j)];
      const double gap_s = std::max(0.0, std::max(a.s0, b.s0) - std::min(a.s1, b.s1));
      const double gap_d = std::max(0.0, std::max(a.d0, b.d0) - std::min(a.d1, b.d1));
      if (gap_s <= params_.block_link_s_m && gap_d <= params_.block_link_d_m) {
        parent[static_cast<size_t>(root(j))] = root(i);
      }
    }
  }
  struct Meas
  {
    Box box;
    std::vector<Box> parts;
  };
  std::vector<Meas> meas;
  std::vector<int> slot(static_cast<size_t>(n), -1);
  for (int i = 0; i < n; ++i) {
    const int r = root(i);
    const Box & b = boxes[static_cast<size_t>(i)];
    if (slot[static_cast<size_t>(r)] < 0) {
      slot[static_cast<size_t>(r)] = static_cast<int>(meas.size());
      meas.push_back(Meas{b, {b}});
    } else {
      Meas & m = meas[static_cast<size_t>(slot[static_cast<size_t>(r)])];
      m.box.s0 = std::min(m.box.s0, b.s0);
      m.box.s1 = std::max(m.box.s1, b.s1);
      m.box.d0 = std::min(m.box.d0, b.d0);
      m.box.d1 = std::max(m.box.d1, b.d1);
      m.parts.push_back(b);
    }
  }

  //  ★기억과 맞춘다★ 겹치면 같은 블록. 둘 이상과 겹치면 기억끼리 합친다
  //  (멀리서 따로 보이던 콘이 가까이서 한 줄로 이어지는 경우).
  const double gap = params_.assoc_gap_m;
  auto overlaps = [&](const Box & a, const Box & b) {
    return a.s0 - gap <= b.s1 && b.s0 - gap <= a.s1 &&
           a.d0 - gap <= b.d1 && b.d0 - gap <= a.d1;
  };
  //  ★구성 군집끼리 본다★ 외곽 상자로 보면 길게 이어진 좌·우 가장자리 블록이
  //  서로의 외곽과 겹쳐 하나로 합쳐진다(모의 edges_tight 에서 실제로 그랬다).
  auto matches = [&](const Meas & m, const Block & b) {
    if (!overlaps(m.box, b.box)) {
      return false;
    }
    if (b.parts.empty()) {
      return true;
    }
    for (const Box & x : m.parts) {
      for (const Box & y : b.parts) {
        if (overlaps(x, y)) {
          return true;
        }
      }
    }
    return false;
  };
  std::vector<char> dead(mem_.size(), 0);
  for (const Meas & mm : meas) {
    const Box & m = mm.box;
    int best = -1;
    for (size_t k = 0; k < mem_.size(); ++k) {
      if (dead[k] || !matches(mm, mem_[k])) {
        continue;
      }
      if (best < 0) {
        best = static_cast<int>(k);
        continue;
      }
      Block & keep = mem_[static_cast<size_t>(best)];
      Block & gone = mem_[k];
      keep.box.s0 = std::min(keep.box.s0, gone.box.s0);
      keep.box.s1 = std::max(keep.box.s1, gone.box.s1);
      keep.box.d0 = std::min(keep.box.d0, gone.box.d0);
      keep.box.d1 = std::max(keep.box.d1, gone.box.d1);
      if (gone.hits > keep.hits) {
        keep.side = gone.side;
        keep.committed = gone.committed;
        keep.feas_locked = gone.feas_locked;
      }
      keep.announced = keep.announced || gone.announced;
      keep.hits = std::max(keep.hits, gone.hits);
      keep.confirmed = keep.confirmed || gone.confirmed;
      keep.parts.insert(keep.parts.end(), gone.parts.begin(), gone.parts.end());
      dead[k] = 1;
    }
    if (best < 0) {
      Block nb;
      nb.id = next_id_++;
      nb.box = m;
      nb.parts = mm.parts;
      nb.hits = 1;
      nb.first_t = now_s;
      nb.last_t = now_s;
      nb.confirmed = params_.confirm_hits <= 1;
      mem_.push_back(nb);
      dead.push_back(0);
      continue;
    }
    Block & b = mem_[static_cast<size_t>(best)];
    //  ★넓어지는 것은 빨리, 좁아지는 것은 천천히★ 멀리서는 줄의 일부만 보이다가
    //  가까워지며 드러난다(넓어짐). 좁아 보이는 것은 대개 가림·자기 차체 지움이라
    //  ★코앞에서는 아예 줄이지 않는다★.
    double ex = 0.0;
    double ey = 0.0;
    odomToEgo(odom, m.s0, 0.5 * (m.d0 + m.d1), ex, ey);
    const bool partial = ex < params_.blind_x_m;
    //  ★한 번에 넓히지 않는다★ 잘못 옮겨진 관측 한 장(재무장 첫 틱 자세 0 이 그랬다)이
    //  블록을 영구히 부풀리면 안 된다. 반씩 넓히면 0.2 s 안에 따라가고, 한 장짜리
    //  오류는 절반만 들어왔다가 줄어든다.
    constexpr double kGrowA = 0.5;
    constexpr double kShrinkA = 0.15;
    auto blend = [&](double & e, double v, bool v_is_wider) {
      if (v_is_wider) {
        e += kGrowA * (v - e);
      } else if (!partial) {
        e += kShrinkA * (v - e);
      }
    };
    blend(b.box.s0, m.s0, m.s0 < b.box.s0);
    blend(b.box.s1, m.s1, m.s1 > b.box.s1);
    blend(b.box.d0, m.d0, m.d0 < b.box.d0);
    blend(b.box.d1, m.d1, m.d1 > b.box.d1);
    //  구성 군집은 온전히 보일 때만 새로 받는다 — 코앞에서 잘린 관측으로 바꾸면
    //  옆을 지나는 동안 창이 사라진다.
    if (!partial || b.parts.empty()) {
      b.parts = mm.parts;
    }
    ++b.hits;
    b.last_t = now_s;
    if (!b.confirmed && b.hits >= params_.confirm_hits) {
      b.confirmed = true;
    }
  }
  std::vector<Block> kept;
  kept.reserve(mem_.size());
  for (size_t k = 0; k < mem_.size(); ++k) {
    if (!dead[k]) {
      kept.push_back(mem_[k]);
    }
  }
  mem_.swap(kept);
}

void FrenetPlanner::prune(const OdomPose & odom, double now_s)
{
  const double rear_s = odom.x - bodyRear();
  std::vector<Block> kept;
  kept.reserve(mem_.size());
  for (const Block & b : mem_) {
    if (!b.confirmed && (now_s - b.last_t) > 0.3) {
      continue;                       // 한두 번 보인 잡음
    }
    if (b.box.s1 + params_.hold_post_m < rear_s) {
      if (b.confirmed && b.side != 0 && intrudes(b.box)) {
        last_side_ = b.side;
        char buf[160];
        std::snprintf(buf, sizeof(buf),
          "✅ 블록 #%d 통과 (%s) — 다음 가운데 블록은 반대쪽", b.id, sideName(b.side));
        event(buf);
      }
      continue;
    }
    if (b.confirmed && (now_s - b.last_t) > params_.lost_s) {
      double ex = 0.0;
      double ey = 0.0;
      odomToEgo(odom, b.box.s0, 0.5 * (b.box.d0 + b.box.d1), ex, ey);
      if (ex > params_.blind_x_m) {
        char buf[160];
        std::snprintf(buf, sizeof(buf),
          "❔ 블록 #%d 이 %.1fs 안 보인다 (앞 %.1f m) — 버린다", b.id,
          now_s - b.last_t, ex);
        event(buf);
        continue;
      }
    }
    kept.push_back(b);
  }
  mem_.swap(kept);
}

void FrenetPlanner::decideSides(const OdomPose & odom)
{
  std::vector<int> order;
  for (size_t k = 0; k < mem_.size(); ++k) {
    if (mem_[k].confirmed) {
      order.push_back(static_cast<int>(k));
    }
  }
  std::sort(order.begin(), order.end(), [&](int a, int b) {
    return mem_[static_cast<size_t>(a)].box.s0 < mem_[static_cast<size_t>(b)].box.s0;
  });
  const double clear = clearance();
  const double max_off = params_.max_offset_m + 1e-6;
  const double front_s = odom.x + bodyFront();
  int prev = last_side_;
  for (size_t pos = 0; pos < order.size(); ++pos) {
    Block & b = mem_[static_cast<size_t>(order[pos])];
    const double lp = b.box.d1 + clear;
    const double rp = b.box.d0 - clear;
    const bool lp_ok = lp <= max_off;
    const bool rp_ok = rp >= -max_off;
    const double centre = 0.5 * (b.box.d0 + b.box.d1);
    const double ahead = b.box.s0 - front_s;
    const char * why = "";
    bool strong = true;
    int want = desiredSide(order, pos, prev, odom, &why, &strong);
    if (want > 0 && !lp_ok && rp_ok) {
      want = -1;
      why = "왼쪽은 ±3 m 밖";
    } else if (want < 0 && !rp_ok && lp_ok) {
      want = +1;
      why = "오른쪽은 ±3 m 밖";
    }
    char buf[240];
    const bool matters = intrudes(b.box);
    const char * why_now = why;
    if (b.side == 0) {
      b.side = want;
      b.disagree = 0;
    } else if (!b.committed) {
      //  ★확정 전에만 바꾼다★ ±3 m 밖이면 곧바로, 규칙이 달라졌으면 flip_ticks 연속일 때.
      const bool infeasible = (b.side > 0 && !lp_ok && rp_ok) || (b.side < 0 && !rp_ok && lp_ok);
      bool flip = false;
      if (infeasible) {
        flip = true;
      } else if (!b.feas_locked && strong && want != b.side) {
        flip = ++b.disagree >= std::max(1, params_.flip_ticks);
      } else {
        b.disagree = 0;
      }
      if (flip) {
        b.side = want;
        b.disagree = 0;
        if (matters) {
          std::snprintf(buf, sizeof(buf),
            "↔️ 블록 #%d 쪽 변경 → %s (%s) — 앞 %.1f m, d [%+.2f, %+.2f]",
            b.id, sideName(want), why, ahead, b.box.d0, b.box.d1);
          event(buf);
        }
      }
    }
    //  ★궤적을 처음 막는 순간 한 번 알린다★ 멀리서 가장자리 콘만 보이다가 줄이
    //  합류하는 경우처럼, 쪽이 먼저 조용히 정해져 있었어도 여기서 근거와 함께 남긴다.
    if (matters && !b.announced) {
      b.announced = true;
      std::snprintf(buf, sizeof(buf),
        "🚧 블록 #%d 앞 %.1f m, d [%+.2f, %+.2f] (중심 %+.2f) → %s으로 통과 %+.2f m (%s)",
        b.id, ahead, b.box.d0, b.box.d1, centre, sideName(b.side),
        std::clamp(b.side > 0 ? lp : rp, -params_.max_offset_m, params_.max_offset_m),
        b.side == want ? why_now : "먼저 정한 쪽 유지");
      event(buf);
    }
    if (!b.committed && ahead <= params_.commit_dist_m) {
      b.committed = true;
    }
    if (matters) {
      prev = b.side;                  // 교대 규칙은 실제로 비킨 블록끼리만
    }
  }
}

std::vector<FrenetPlanner::Group> FrenetPlanner::makeGroups(std::vector<Bound> * bounds) const
{
  const double clear = clearance();
  std::vector<Group> wins;
  std::vector<Bound> bnds;
  for (size_t k = 0; k < mem_.size(); ++k) {
    const Block & b = mem_[k];
    if (!b.confirmed || b.side == 0) {
      continue;
    }
    const std::vector<Box> parts = b.parts.empty() ? std::vector<Box>{b.box} : b.parts;
    for (const Box & p : parts) {
      const double a = p.s0 - params_.hold_pre_m;
      const double e = p.s1 + params_.hold_post_m;
      const double lo = (b.side > 0) ? p.d1 + clear : -1e9;
      const double hi = (b.side < 0) ? p.d0 - clear : 1e9;
      if (intrudes(p)) {
        Group g;
        g.a = a;
        g.b = e;
        g.lo = lo;
        g.hi = hi;
        g.members.push_back(static_cast<int>(k));
        wins.push_back(g);
      } else {
        bnds.push_back(Bound{a, e, lo, hi});
      }
    }
  }
  mergeAndTarget(wins, bnds);
  if (bounds) {
    bounds->swap(bnds);
  }
  return wins;
}

void FrenetPlanner::mergeAndTarget(
  std::vector<Group> & groups, const std::vector<Bound> & bounds) const
{
  std::sort(groups.begin(), groups.end(), [](const Group & x, const Group & y) { return x.a < y.a; });
  std::vector<Group> out;
  for (const Group & g : groups) {
    if (!out.empty() && g.a <= out.back().b) {
      Group & m = out.back();
      m.b = std::max(m.b, g.b);
      m.lo = std::max(m.lo, g.lo);
      m.hi = std::min(m.hi, g.hi);
      m.members.insert(m.members.end(), g.members.begin(), g.members.end());
      if (m.members.empty()) {
        m.target = g.target;
      }
    } else {
      out.push_back(g);
    }
  }
  for (Group & g : out) {
    //  궤적을 침범하지 않는 군집은 겹치는 창의 경계로만 들어온다.
    for (const Bound & bd : bounds) {
      if (bd.a <= g.b && g.a <= bd.b) {
        g.lo = std::max(g.lo, bd.lo);
        g.hi = std::min(g.hi, bd.hi);
      }
    }
    //  ★궤적에 제일 가까운 합법 d★ (블록이 있는 창). 경계만으로 생긴 창은 지금 경로에
    //  제일 가까운 d(= 만들 때 넣은 경계값). 좌우 제약이 엇갈리면(문 모양) 가운데로.
    if (g.lo <= g.hi) {
      g.target = g.members.empty() ? std::clamp(g.target, g.lo, g.hi)
                                   : std::clamp(0.0, g.lo, g.hi);
    } else {
      g.target = 0.5 * (g.lo + g.hi);
    }
    g.target = std::clamp(g.target, -params_.max_offset_m, params_.max_offset_m);
    g.active = true;
  }
  groups.swap(out);
}

FrenetPlanner::Knot FrenetPlanner::startState(const OdomPose & odom, double kappa0) const
{
  Knot k;
  k.s = odom.x;
  //  ★직전 경로 위에 있으면 그 경로에서 이어 그린다★ 5차 다항식은 끝 조건이
  //  같으면 중간에서 다시 그려도 같은 곡선이라, 장애물이 그대로면 경로도 그대로다.
  //  차의 실제 자세에서 매번 새로 그리면 추종 지연만큼 곡선이 가팔라지며 밀린다.
  if (last_path_.valid && last_path_.pts.size() >= 2 &&
      odom.x >= last_path_.pts.front().s && odom.x <= last_path_.pts.back().s)
  {
    const FrenetPoint p = last_path_.sample(odom.x);
    if (std::abs(odom.y - p.d) <= params_.on_path_d_m &&
        std::abs(wrapAngle(odom.yaw - p.yaw)) <= params_.on_path_yaw_rad)
    {
      k.d = p.d;
      k.d1 = p.d_s;
      k.d2 = p.kappa * std::pow(1.0 + p.d_s * p.d_s, 1.5);
      return k;
    }
  }
  k.d = odom.y;
  k.d1 = std::tan(std::clamp(odom.yaw, -0.55, 0.55));
  k.d2 = std::clamp(kappa0, -0.35, 0.35) * std::pow(1.0 + k.d1 * k.d1, 1.5);
  return k;
}

void FrenetPlanner::appendQuintic(FrenetPath & path, const Knot & a, const Knot & b) const
{
  if (b.s <= a.s + 1e-3) {
    return;
  }
  const Quintic q = makeQuintic(a.s, a.d, a.d1, a.d2, b.s, b.d, b.d1, b.d2);
  const double ds = std::max(0.05, params_.ds_m);
  auto push = [&](double s) {
    if (!path.pts.empty() && s <= path.pts.back().s + 1e-6) {
      return;
    }
    FrenetPoint p;
    double d = 0.0;
    double d1 = 0.0;
    double d2 = 0.0;
    q.eval(s, d, d1, d2);
    p.s = s;
    p.d = d;
    p.d_s = d1;
    p.yaw = std::atan(d1);
    p.kappa = kappaOf(d1, d2);
    path.pts.push_back(p);
  };
  for (double s = a.s; s < b.s - 0.5 * ds; s += ds) {
    push(s);
  }
  push(b.s);
}

void FrenetPlanner::appendFlat(FrenetPath & path, double s_from, double s_to, double d) const
{
  if (s_to <= s_from + 1e-3) {
    return;
  }
  const double ds = std::max(0.05, params_.ds_m);
  auto push = [&](double s) {
    if (!path.pts.empty() && s <= path.pts.back().s + 1e-6) {
      return;
    }
    FrenetPoint p;
    p.s = s;
    p.d = d;
    path.pts.push_back(p);
  };
  for (double s = s_from; s < s_to - 0.5 * ds; s += ds) {
    push(s);
  }
  push(s_to);
}

double FrenetPlanner::entryKappa(const Knot & start, const Group & g) const
{
  const double a = std::max(g.a, start.s + params_.entry_len_min_m);
  const Quintic q = makeQuintic(start.s, start.d, start.d1, start.d2, a, g.target, 0.0, 0.0);
  double kmax = 0.0;
  for (int i = 0; i <= 40; ++i) {
    double d = 0.0;
    double d1 = 0.0;
    double d2 = 0.0;
    q.eval(start.s + q.L * i / 40.0, d, d1, d2);
    kmax = std::max(kmax, std::abs(kappaOf(d1, d2)));
  }
  return kmax;
}

FrenetPath FrenetPlanner::buildPath(const Knot & start, const std::vector<Group> & groups) const
{
  FrenetPath path;
  const double kc = params_.kappa_comfort;
  Knot cur = start;
  cur.d1 = std::clamp(cur.d1, -0.6, 0.6);
  cur.d2 = std::clamp(cur.d2, -0.4, 0.4);
  bool first = true;
  for (const Group & g : groups) {
    if (!g.active || g.b <= cur.s + 0.05) {
      continue;                      // 비킬 필요 없는 창 · 이미 지나간 창
    }
    //  ★진입은 남은 거리를 전부 쓴다★ 가장 완만한 곡선이 된다.
    //  이미 창 안이면(추종이 늦었다) entry_len_min 안에 붙는다 — 경로가 끊기지 않게
    //  둘째 창부터도 같은 하한을 둔다.
    const double a = std::max(g.a, cur.s + params_.entry_len_min_m);
    if (first) {
      appendQuintic(path, cur, Knot{a, g.target, 0.0, 0.0});
    } else {
      const double gap = a - cur.s;
      const double l_ret = lenForShift(cur.d, kc, params_.return_len_min_m, params_.return_len_max_m);
      const double l_sh = lenForShift(g.target - 0.0, kc, params_.shift_len_min_m, params_.shift_len_max_m);
      const bool from_zero = std::abs(cur.d) <= 0.05;
      const bool to_zero = std::abs(g.target) <= 0.05;
      if (!from_zero && !to_zero && gap >= l_ret + l_sh + params_.flat_min_m) {
        //  블록 사이가 넉넉하면 궤적(0)으로 돌아왔다가 다시 비킨다.
        appendQuintic(path, cur, Knot{cur.s + l_ret, 0.0, 0.0, 0.0});
        appendFlat(path, cur.s + l_ret, a - l_sh, 0.0);
        appendQuintic(path, Knot{a - l_sh, 0.0, 0.0, 0.0}, Knot{a, g.target, 0.0, 0.0});
      } else if (from_zero && gap >= l_sh + params_.flat_min_m) {
        appendFlat(path, cur.s, a - l_sh, cur.d);
        appendQuintic(path, Knot{a - l_sh, cur.d, 0.0, 0.0}, Knot{a, g.target, 0.0, 0.0});
      } else if (to_zero && gap >= l_ret + params_.flat_min_m) {
        appendQuintic(path, cur, Knot{cur.s + l_ret, g.target, 0.0, 0.0});
        appendFlat(path, cur.s + l_ret, a, g.target);
      } else {
        //  ★교대 — 두 창 사이 거리를 전부 쓴다★
        appendQuintic(path, cur, Knot{a, g.target, 0.0, 0.0});
      }
    }
    if (g.b > a) {
      appendFlat(path, a, g.b, g.target);
    }
    cur = Knot{std::max(a, g.b), g.target, 0.0, 0.0};
    first = false;
  }
  //  ★마지막 블록 뒤 — R 8 m 로 궤적에 돌아온다★ 단조 곡선이라 넘어가지 않는다.
  if (std::abs(cur.d) > 0.01 || std::abs(cur.d1) > 0.005 || std::abs(cur.d2) > 0.005) {
    const double mag = std::max(std::abs(cur.d), 2.0 * std::abs(cur.d1));
    const double l_ret = lenForShift(mag, kc, params_.return_len_min_m, params_.return_len_max_m);
    appendQuintic(path, cur, Knot{cur.s + l_ret, 0.0, 0.0, 0.0});
    cur = Knot{cur.s + l_ret, 0.0, 0.0, 0.0};
  }
  appendFlat(path, cur.s, std::max(cur.s + 3.0, start.s + params_.plan_distance_m), 0.0);
  path.valid = path.pts.size() >= 2;
  return path;
}

void FrenetPlanner::fillDiag(
  const OdomPose & odom, const FrenetPath & path, const std::vector<Group> & groups)
{
  diag_ = PlanDiag{};
  const double rear_s = odom.x - bodyRear();
  for (const Block & b : mem_) {
    if (b.confirmed && intrudes(b.box) && b.box.s1 + params_.hold_post_m >= rear_s) {
      ++diag_.n_blocks;
    }
  }
  for (const Group & g : groups) {
    if (g.active && g.b > odom.x) {
      diag_.target_d = g.target;
      break;
    }
  }
  const double hw = 0.5 * vehicle_.track_width;
  const double front = bodyFront();
  const double rear = bodyRear();
  double min_clear = 99.0;
  int k = 0;
  for (const FrenetPoint & p : path.pts) {
    if (p.s < odom.x - 0.01) {
      continue;
    }
    diag_.max_kappa = std::max(diag_.max_kappa, std::abs(p.kappa));
    if (p.s > odom.x + 14.0 || (k++ % 2) != 0) {
      continue;
    }
    const double c = std::cos(p.yaw);
    const double sn = std::sin(p.yaw);
    auto check = [&](double bx, double by) {
      const double s = p.s + bx * c - by * sn;
      const double d = p.d + bx * sn + by * c;
      for (const Block & b : mem_) {
        if (!b.confirmed) {
          continue;
        }
        min_clear = std::min(min_clear,
          pointBoxDist(s, d, b.box.s0, b.box.s1, b.box.d0, b.box.d1));
      }
    };
    for (double bx = -rear; bx <= front + 1e-6; bx += 0.1) {
      check(bx, hw);
      check(bx, -hw);
    }
    for (double by = -hw; by <= hw + 1e-6; by += 0.1) {
      check(front, by);
      check(-rear, by);
    }
  }
  diag_.min_clear_m = min_clear;
  diag_.tight = diag_.max_kappa > params_.kappa_max;
}

void FrenetPlanner::exportBlocks(const std::vector<Group> & groups)
{
  out_blocks_.clear();
  for (size_t k = 0; k < mem_.size(); ++k) {
    const Block & b = mem_[k];
    PlanBlock o;
    o.id = b.id;
    o.s0 = b.box.s0;
    o.s1 = b.box.s1;
    o.d0 = b.box.d0;
    o.d1 = b.box.d1;
    o.side = b.side;
    o.confirmed = b.confirmed;
    o.committed = b.committed;
    for (const Group & g : groups) {
      if (std::find(g.members.begin(), g.members.end(), static_cast<int>(k)) != g.members.end()) {
        o.target = g.target;
      }
    }
    out_blocks_.push_back(o);
  }
}

FrenetPath FrenetPlanner::plan(
  const OdomPose & odom, const CostmapSnapshot & snap, double now_s, double kappa0)
{
  //  ★계획이 1초 넘게 끊겼으면 기억을 버린다★ 기준선 s 가 그 사이 이어졌다는
  //  보장이 없다(인계 재무장·추측항법 등).
  if (last_plan_t_ >= 0.0 && (now_s - last_plan_t_) > 1.0) {
    const double gap = now_s - last_plan_t_;
    reset();
    char buf[96];
    std::snprintf(buf, sizeof(buf), "🧹 블록 기억 초기화 — 계획이 %.1fs 끊겼다", gap);
    event(buf);
  }
  last_plan_t_ = now_s;

  observe(odom, snap, now_s);
  if (!params_.enable) {
    mem_.clear();
  }
  prune(odom, now_s);
  decideSides(odom);
  std::vector<Bound> bounds;
  std::vector<Group> groups = makeGroups(&bounds);
  const Knot start = startState(odom, kappa0);

  //  ★확정 전인 첫 블록이 너무 빡빡하면 반대쪽을 본다★ 규칙대로 가면 조향 상한을
  //  넘는 곡선이 되는 경우(인계가 늦었거나 헤딩이 크게 틀어져 있을 때)뿐이다.
  //  반대쪽이 확실히 완만할 때만 바꾸고, 바꾼 쪽은 규칙으로 되돌리지 않는다.
  auto firstActive = [&](const std::vector<Group> & gs) -> const Group * {
    for (const Group & g : gs) {
      if (g.active && g.b > start.s + 0.05) {
        return &g;
      }
    }
    return nullptr;
  };
  const Group * g0 = firstActive(groups);
  if (g0 && g0->members.size() == 1) {
    Block & b = mem_[static_cast<size_t>(g0->members.front())];
    const double k0 = entryKappa(start, *g0);
    //  ★반대쪽이 ±3 m 안일 때만 본다★ 9/28 23:03 : 폭 밖의 쪽으로 바꿨다가 decideSides
    //  가 같은 틱에 되돌렸다(⚠️ → ↔️). 갈 수 없는 쪽은 후보가 아니다.
    const double clear = clearance();
    const bool other_ok = (b.side > 0) ? (b.box.d0 - clear >= -params_.max_offset_m)
                                       : (b.box.d1 + clear <= params_.max_offset_m);
    if (k0 > params_.kappa_max && !b.committed && !b.feas_locked && other_ok) {
      b.side = -b.side;
      std::vector<Bound> alt_bounds;
      std::vector<Group> alt = makeGroups(&alt_bounds);
      const Group * g1 = firstActive(alt);
      const double k1 = g1 ? entryKappa(start, *g1) : 0.0;
      if (k1 < 0.85 * k0) {
        b.feas_locked = true;
        char buf[200];
        std::snprintf(buf, sizeof(buf),
          "⚠️ 블록 #%d 쪽 변경 → %s — 반대쪽은 R %.1f m 로 못 돈다 (이쪽 R %.1f m)",
          b.id, sideName(b.side), 1.0 / std::max(k0, 1e-3), 1.0 / std::max(k1, 1e-3));
        event(buf);
        groups.swap(alt);
        bounds.swap(alt_bounds);
      } else {
        b.side = -b.side;
      }
    }
  }

  FrenetPath path = buildPath(start, groups);
  //  ★궤적을 침범하지 않던 군집도 경로가 넘어가면 제약이 된다★ 가장자리 콘·벽은 창을
  //  만들지 않지만, 다른 블록을 비키려다 경로가 그쪽으로 넘어가면 그 군집 자리에서
  //  넘지 않는 가장 가까운 d 로 묶는다. 군집마다 창이 작아 두 번이면 충분하다.
  for (int it = 0; it < 2; ++it) {
    std::vector<Group> extra;
    for (const Bound & bd : bounds) {
      if (bd.b <= start.s + 0.05) {
        continue;
      }
      double worst = 0.0;
      double want = 0.0;
      for (double s = std::max(bd.a, start.s); s <= bd.b + 1e-6; s += 0.25) {
        const double d = path.sample(s).d;
        if (d - bd.hi > worst) {
          worst = d - bd.hi;
          want = bd.hi;
        }
        if (bd.lo - d > worst) {
          worst = bd.lo - d;
          want = bd.lo;
        }
      }
      if (worst > 0.02) {
        Group h;
        h.a = bd.a;
        h.b = bd.b;
        h.lo = bd.lo;
        h.hi = bd.hi;
        h.target = want;
        extra.push_back(h);
      }
    }
    if (extra.empty()) {
      break;
    }
    groups.insert(groups.end(), extra.begin(), extra.end());
    mergeAndTarget(groups, bounds);
    path = buildPath(start, groups);
  }
  fillDiag(odom, path, groups);
  exportBlocks(groups);
  path.n_gates = diag_.n_blocks;
  path.target_d = path.valid ? path.sample(odom.x).d : 0.0;
  last_path_ = path;
  return path;
}

}  // namespace mppi_local_planner
