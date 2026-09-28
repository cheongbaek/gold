// ══════════════════════════════════════════════════════════════════════════
//  라바콘 회피 폐루프 모의실험 [2026-09-28] — 실제 EgoCostmap · FrenetPlanner ·
//  path_tracker.hpp 를 그대로 쓰고, 조향 액추에이터(불감시간 0.25 s · 슬루 55°/s ·
//  불감대 1.41° · 상한 31.7°)와 노드 filterSteer(LPF 0.7 · 36°/s)를 흉내 낸다.
//  ★colcon 빌드에 넣지 않는다★ — 계획기·추종기를 고친 뒤 손으로 돌리는 도구다.
//
//    cd gold_ws/src/mppi_local_planner
//    P="sensor_msgs nav_msgs std_msgs geometry_msgs builtin_interfaces rosidl_runtime_cpp"
//    P="$P rosidl_runtime_c rosidl_typesupport_interface rcutils"
//    INC=""; for p in $P; do INC="$INC -I/opt/ros/humble/include/$p"; done
//    SRC="src/frenet_planner.cpp src/cone_detector.cpp src/ego_costmap.cpp"
//    g++ -std=c++17 -O2 -Iinclude $INC test/avoid_sim.cpp $SRC -o /tmp/avoid_sim
//    QUIET=1 /tmp/avoid_sim /tmp/trace.csv /tmp/paths.csv      # 요약만
//    SCEN=diagram /tmp/avoid_sim /tmp/t.csv /tmp/p.csv          # 장면 이름 일부로 거르고 사건 로그까지
//  환경변수로 값을 덮는다: PASS_GAP HOLD_PRE HOLD_POST KAPPA_MAX COMMIT LD PRED KCTE KI IMAX
//  한 줄 요약 : min_clear(차체↔콘 최소 여유) · max|d| · max|psi| · stops · rev(복귀 뒤
//  조향 반전) · tail(마지막 콘 +11 m 뒤 최대 |d| = 궤적 복귀 잔차).
// ══════════════════════════════════════════════════════════════════════════
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <deque>
#include <random>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "mppi_local_planner/ego_costmap.hpp"
#include "mppi_local_planner/frenet_planner.hpp"
#include "mppi_local_planner/path_tracker.hpp"

using namespace mppi_local_planner;

struct Cone { double s, d; };
struct Scenario
{
  std::string name;
  std::vector<Cone> cones;
  double s_start = -8.0;
  double s_handover = 0.0;
  double d_h = 0.0, psi_h_deg = 0.0;
  double v = 1.3;
  double s_end = 36.0;
  bool preview = true;
  double steer_bias_deg = 0.0;
  double yaw_err_deg = 0.0;
  unsigned seed = 1;
};

struct Result
{
  double min_clear = 99, max_abs_d = 0, max_abs_psi = 0, max_steer = 0, d_end = 0;
  int reversals = 0;          // steering sign flips (|cmd|>2 deg) after the last cone
  double overshoot = 0.0;     // max |d| after the last cone + 3 m (return overshoot)
  int stops = 0;
  double stop_time = 0;
  bool collided = false;
  std::vector<double> per_cone;
};

static const double kR = 0.12;          // cone radius at the scan band
static const double kHalfW = 0.54, kFront = 1.25, kRear = 0.30;

static double rectPointDist(double bx, double by)
{
  const double dx = std::max({-kRear - bx, 0.0, bx - kFront});
  const double dy = std::max({-kHalfW - by, 0.0, by - kHalfW});
  if (dx > 0 || dy > 0) return std::hypot(dx, dy);
  return -std::min({bx + kRear, kFront - bx, by + kHalfW, kHalfW - by});
}

static sensor_msgs::msg::PointCloud2 makeCloud(
  const std::vector<Cone> & cones, double X, double Y, double psi, std::mt19937 & rng)
{
  std::vector<float> pts;
  std::uniform_real_distribution<double> U(0, 1);
  const double c = std::cos(psi), s = std::sin(psi);
  for (const auto & k : cones) {
    for (int a = 0; a < 16; ++a) {
      const double th = a * 2 * M_PI / 16;
      const double ws = k.s + kR * std::cos(th), wd = k.d + kR * std::sin(th);
      const double dx = ws - X, dy = wd - Y;
      const double ex = dx * c + dy * s, ey = -dx * s + dy * c;   // ego
      const double r = std::hypot(ex, ey);
      if (r > 18.0) continue;
      if (r > 13.0 && U(rng) > 0.55) continue;                   // sparse far returns
      // os_lidar is flipped (yaw pi): sensor = -ego
      pts.push_back(static_cast<float>(-ex));
      pts.push_back(static_cast<float>(-ey));
      pts.push_back(-0.5f);
    }
  }
  sensor_msgs::msg::PointCloud2 m;
  m.height = 1;
  m.width = static_cast<uint32_t>(pts.size() / 3);
  sensor_msgs::PointCloud2Modifier mod(m);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(m.width);
  sensor_msgs::PointCloud2Iterator<float> ix(m, "x"), iy(m, "y"), iz(m, "z");
  for (size_t i = 0; i < m.width; ++i, ++ix, ++iy, ++iz) {
    *ix = pts[3 * i]; *iy = pts[3 * i + 1]; *iz = pts[3 * i + 2];
  }
  return m;
}


static Result run(const Scenario & sc, FILE * trace, FILE * paths)
{
  std::mt19937 rng(sc.seed);
  std::normal_distribution<double> nd(0.0, 1.0);
  CostmapParams cp;
  cp.size_x = 36; cp.size_y = 16; cp.resolution = 0.1;
  cp.ground_z_min = 0.30 - 1.17; cp.ground_z_max = 1.50 - 1.17;
  cp.robot_half_width = 1.08 / 2 + 0.08; cp.inflation_radius = 0.40; cp.min_range = 0.40;
  cp.sensor_yaw_offset = M_PI; cp.occupancy_decay = 0.25;
  cp.ego_clear_radius = 0.90; cp.ego_clear_x_min = -0.45; cp.ego_clear_x_max = 1.95;
  cp.ego_clear_y_half = 0.66; cp.ego_clear_pass_x = 0.90; cp.ego_clear_y_half_passed = 1.60;
  cp.ego_cost_clear_radius = 1.00;
  EgoCostmap cm(cp);
  VehicleParams vp; vp.wheelbase = 1.25; vp.track_width = 1.08; vp.max_steering_angle = 0.40;
  vp.rear_overhang = 0.30; vp.front_overhang = 0.0;

  FrenetPlannerParams fp;
  auto env = [](const char * k, double def) { const char * v = std::getenv(k); return v ? std::atof(v) : def; };
  fp.pass_gap_m = env("PASS_GAP", fp.pass_gap_m);
  fp.hold_pre_m = env("HOLD_PRE", fp.hold_pre_m);
  fp.hold_post_m = env("HOLD_POST", fp.hold_post_m);
  fp.kappa_max = env("KAPPA_MAX", fp.kappa_max);
  fp.commit_dist_m = env("COMMIT", fp.commit_dist_m);
  FrenetPlanner planner(fp, vp);
  TrackerParams tp;
  tp.lookahead = env("LD", tp.lookahead);
  tp.lookahead_min = std::min(tp.lookahead_min, tp.lookahead);
  tp.predict_s = env("PRED", tp.predict_s);
  tp.k_cte = env("KCTE", tp.k_cte);
  const double ki = env("KI", 2.0), i_max = env("IMAX", 3.0);
  TrackIntegrator integ;
  const double L = 1.25, dt = 0.05;
  double X = sc.s_start, Y = sc.d_h + std::tan(sc.psi_h_deg * M_PI / 180) * (sc.s_start - sc.s_handover);
  double psi = sc.psi_h_deg * M_PI / 180;
  double v = sc.v;
  double steer_act = 0.0;                      // road wheel actual [rad]
  std::deque<double> dead(5, 0.0);             // 0.25 s dead time
  double steer_filt = 0.0, last_pub = 0.0;     // node filterSteer state [deg]
  int stop_enter = 0, stop_exit = 0; bool latched = false; double latch_t = 0; bool braking = false;
  bool handed = false;
  double prev_sign = 0.0;
  Result R; R.per_cone.assign(sc.cones.size(), 99.0);
  double t = 0.0;
  int tick = 0;
  while (X < sc.s_end && t < 60.0) {
    const bool permitted = X >= sc.s_handover;
    // lidar
    auto cloud = makeCloud(sc.cones, X, Y, psi, rng);
    cm.updateFromPointCloud(cloud);
    const CostmapSnapshot snap = cm.snapshot();
    OdomPose pose;
    pose.x = X; pose.y = Y + 0.02 * nd(rng);
    pose.yaw = psi + (sc.yaw_err_deg + 0.3 * nd(rng)) * M_PI / 180;
    double cmd_deg = 0.0;
    if (!permitted) {
      // driving holds the approach line; mppi previews (new) or is silent (old)
      if (sc.preview) planner.plan(pose, snap, t, 0.0);
      X += v * std::cos(psi) * dt;
      Y = sc.d_h + std::tan(sc.psi_h_deg * M_PI / 180) * (X - sc.s_handover);
      t += dt; ++tick;
      continue;
    }
    if (!handed) {
      handed = true;
      if (!sc.preview) planner.reset();
      steer_filt = last_pub = 0.0;
      steer_act = psi * 0; // actual steering at handover ~ straight
    }
    const double kappa0 = std::tan(last_pub * M_PI / 180) / L;
    FrenetPath path = planner.plan(pose, snap, t, kappa0);
    for (const auto & e : planner.takeEvents()) if (!std::getenv("QUIET")) std::fprintf(stdout, "    [%5.2f s=%5.1f] %s\n", t, X, e.c_str());
    double raw = purePursuitSteer(pose, path, tp, v, last_pub * M_PI / 180) * 180 / M_PI;
    {
      const double e = pose.y - path.sample(pose.x).d;
      raw += integ.update(e, dt, ki, i_max, 0.5, v > 0.3 && !latched);
      raw = std::clamp(raw, -22.9, 22.9);
    }
    // filterSteer (node): LPF 0.7 + slew 36 deg/s
    steer_filt = 0.70 * raw + 0.30 * steer_filt;
    const double md = 36.0 * dt;
    steer_filt = std::clamp(steer_filt, last_pub - md, last_pub + md);
    last_pub = steer_filt;
    cmd_deg = steer_filt;
    const bool raw_stop = predictTrackHits(pose, path, snap, tp, 1.768, cmd_deg * M_PI / 180, 3.0);
    if (raw_stop) { ++stop_enter; stop_exit = 0; } else { ++stop_exit; stop_enter = 0; }
    if (!latched && stop_enter >= 8) { latched = true; latch_t = t; ++R.stops; }
    else if (latched && stop_exit >= 12) { latched = false; braking = false; }
    if (latched && t - latch_t >= 0.35) braking = true;
    // vehicle speed
    if (braking) v = std::max(0.0, v - 2.2 * dt);
    else if (latched) v = std::max(0.0, v - 0.41 * dt);
    else v = std::min(sc.v, v + 1.0 * dt);
    if (latched) R.stop_time += dt;
    // steering actuator: dead time, deadband 1.41 deg, slew 55 deg/s
    dead.push_back(cmd_deg * M_PI / 180);
    const double target = dead.front(); dead.pop_front();
    const double err = target - steer_act;
    if (std::abs(err) > 1.41 * M_PI / 180) {
      const double rate = std::min(55.0 * M_PI / 180, std::abs(err) / 0.15);
      steer_act += std::copysign(std::min(std::abs(err), rate * dt), err);
    }
    steer_act = std::clamp(steer_act, -31.7 * M_PI / 180, 31.7 * M_PI / 180);
    const double d_eff = steer_act + sc.steer_bias_deg * M_PI / 180;
    // kinematics
    X += v * std::cos(psi) * dt;
    Y += v * std::sin(psi) * dt;
    psi += v * std::tan(d_eff) / L * dt;
    t += dt; ++tick;
    // metrics
    for (size_t k = 0; k < sc.cones.size(); ++k) {
      const double dx = sc.cones[k].s - X, dy = sc.cones[k].d - Y;
      const double bx = dx * std::cos(psi) + dy * std::sin(psi), by = -dx * std::sin(psi) + dy * std::cos(psi);
      const double c = rectPointDist(bx, by) - kR;
      R.per_cone[k] = std::min(R.per_cone[k], c);
      R.min_clear = std::min(R.min_clear, c);
    }
    {
      double last_s = -1e9; for (const auto & k : sc.cones) last_s = std::max(last_s, k.s);
      if (X > last_s + 1.0) {
        const double sg = (cmd_deg > 2.0) ? 1.0 : (cmd_deg < -2.0 ? -1.0 : 0.0);
        if (sg != 0.0 && prev_sign != 0.0 && sg != prev_sign) ++R.reversals;
        if (sg != 0.0) prev_sign = sg;
      } else { prev_sign = 0.0; }
      if (X > last_s + 3.0 + 8.0) R.overshoot = std::max(R.overshoot, std::abs(Y));
    }
    R.max_abs_d = std::max(R.max_abs_d, std::abs(Y));
    R.max_abs_psi = std::max(R.max_abs_psi, std::abs(psi) * 180 / M_PI);
    R.max_steer = std::max(R.max_steer, std::abs(cmd_deg));
    if (trace) {
      std::fprintf(trace, "%s,%.2f,%.3f,%.3f,%.2f,%.2f,%.2f,%d,%.3f,%.2f\n", sc.name.c_str(), t, X, Y,
        psi * 180 / M_PI, cmd_deg, steer_act * 180 / M_PI, latched ? 1 : 0, path.valid ? path.sample(X).d : 0.0, v);
    }
    if (paths && tick % 10 == 0 && path.valid) {
      std::fprintf(paths, "%s,%.2f", sc.name.c_str(), X);
      for (size_t i = 0; i < path.pts.size(); i += 2) std::fprintf(paths, ",%.2f:%.3f", path.pts[i].s, path.pts[i].d);
      std::fprintf(paths, "\n");
    }
    if (latched && v <= 0.0 && R.stop_time > 8.0) break;   // stuck
  }
  R.d_end = Y;
  R.collided = R.min_clear < 0.0;
  return R;
}

int main(int argc, char ** argv)
{
  std::vector<Scenario> S;
  auto row = [](double s, std::vector<double> ds) { std::vector<Cone> v; for (double d : ds) v.push_back({s, d}); return v; };
  auto cat = [](std::vector<Cone> a, const std::vector<Cone> & b) { a.insert(a.end(), b.begin(), b.end()); return a; };
  {  // today's layout (reconstructed from 213917 / 212902)
    Scenario s; s.name = "today"; s.d_h = -0.31; s.psi_h_deg = -8.0; s.v = 1.3;
    s.cones = cat(row(8.5, {-0.71, -0.35, 0.01}), row(16.7, {0.45, 1.01, 1.57}));
    S.push_back(s);
    s.name = "today_bias"; s.steer_bias_deg = -2.4; S.push_back(s);
    s.name = "today_nopreview"; s.preview = false; s.steer_bias_deg = 0; S.push_back(s);
    s.name = "today_fast"; s.preview = true; s.v = 1.77; S.push_back(s);
    s.name = "today_yawerr"; s.v = 1.3; s.yaw_err_deg = 2.0; S.push_back(s);
  }
  {  // user's diagram: right-biased first (--OOO), then left-biased (OOO--)
    Scenario s; s.name = "diagram_RL"; s.d_h = 0.0; s.psi_h_deg = 0.0; s.v = 1.5;
    s.cones = cat(row(10.0, {0.0, -0.5, -1.0}), row(18.0, {0.0, 0.5, 1.0}));
    S.push_back(s);
    s.name = "diagram_LR"; s.cones = cat(row(10.0, {0.0, 0.5, 1.0}), row(18.0, {0.0, -0.5, -1.0}));
    S.push_back(s);
    s.name = "diagram_RL_6m"; s.cones = cat(row(9.0, {0.0, -0.5, -1.0}), row(15.0, {0.0, 0.5, 1.0}));
    S.push_back(s);
    s.name = "wide_RL"; s.cones = cat(row(10.0, {0.0, -1.0, -2.0}), row(18.0, {0.0, 1.0, 2.0}));
    S.push_back(s);
    s.name = "late_RL"; s.s_start = 0.0; s.preview = false;   // no preview, blocks 7 m after handover
    s.cones = cat(row(7.0, {0.0, -0.5, -1.0}), row(15.0, {0.0, 0.5, 1.0}));
    S.push_back(s);
  }
  {  // edge cases
    Scenario s; s.v = 1.5;
    s.name = "centre_block"; s.cones = row(12.0, {-0.3, 0.0, 0.3}); S.push_back(s);
    s.name = "gate"; s.cones = cat(row(12.0, {-2.2, -1.9}), row(12.0, {1.9, 2.2})); S.push_back(s);
    s.name = "full_block"; s.cones = row(12.0, {-3.6, -3.0, -2.4, -1.8, -1.2, -0.6, 0.0, 0.6, 1.2, 1.8, 2.4, 3.0, 3.6}); S.push_back(s);
    s.name = "wide_right"; s.cones = row(12.0, {-2.5, -2.0, -1.5, -1.0, -0.5, 0.0, 0.5}); S.push_back(s);
    s.name = "late5m"; s.s_start = 0.0; s.preview = false; s.cones = cat(row(5.0, {0.0, -0.5, -1.0}), row(13.0, {0.0, 0.5, 1.0})); S.push_back(s);
    s.name = "hdg_m12"; s.s_start = -8.0; s.preview = true; s.d_h = 0.2; s.psi_h_deg = -12.3;
    s.cones = cat(row(8.5, {-0.71, -0.35, 0.01}), row(16.7, {0.45, 1.01, 1.57})); S.push_back(s);
    s.name = "hdg_p12"; s.psi_h_deg = +12.0; s.d_h = -0.2; S.push_back(s);
    //  차로 가장자리 콘 줄(±3.4 m, 2 m 간격) + 사용자 그림 — 가장자리가 S 를 방해하면 안 된다
    s.name = "edges_RL"; s.psi_h_deg = 0.0; s.d_h = 0.0;
    s.cones = cat(row(10.0, {0.0, -0.5, -1.0}), row(18.0, {0.0, 0.5, 1.0}));
    for (double e = 0.0; e <= 32.0; e += 2.0) { s.cones.push_back({e, 3.4}); s.cones.push_back({e, -3.4}); }
    S.push_back(s);
    //  가장자리가 ±2.6 m 로 좁으면 둘째 줄 오른쪽 통과가 그 콘에 걸린다 — 경로가 넘지 않아야 한다
    s.name = "edges_tight"; s.cones = cat(row(10.0, {0.0, -0.5, -1.0}), row(18.0, {0.0, 0.5, 1.0}));
    for (double e = 0.0; e <= 32.0; e += 2.0) { s.cones.push_back({e, 2.6}); s.cones.push_back({e, -2.6}); }
    S.push_back(s);
  }
  {  // slalom of single cones (older course)
    Scenario s; s.name = "slalom"; s.v = 1.5;
    s.cones = {{8.0, 0.5}, {14.5, -0.5}, {21.0, 0.5}};
    S.push_back(s);
  }
  FILE * trace = std::fopen(argc > 1 ? argv[1] : "trace.csv", "w");
  FILE * paths = std::fopen(argc > 2 ? argv[2] : "paths.csv", "w");
  std::fprintf(trace, "name,t,x,y,psi,cmd,act,stop,path_d,v\n");
  const char * only = std::getenv("SCEN");
  const bool quiet = std::getenv("QUIET") != nullptr;
  for (const auto & sc : S) {
    if (only && sc.name.find(only) == std::string::npos) continue;
    if (!quiet) std::printf("── %s\n", sc.name.c_str());
    const Result r = run(sc, trace, paths);
    std::printf("%-16s min_clear %+5.2f m  per-cone [", sc.name.c_str(), r.min_clear);
    for (double c : r.per_cone) std::printf("%+.2f ", c);
    std::printf("]  max|d| %.2f  max|psi| %4.1f  max|steer| %4.1f  d_end %+5.2f  stops %d (%.1fs) rev %d tail %.2f%s\n",
      r.max_abs_d, r.max_abs_psi, r.max_steer, r.d_end, r.stops, r.stop_time, r.reversals, r.overshoot, r.collided ? "  ★COLLISION★" : "");
  }
  std::fclose(trace);
  std::fclose(paths);
  return 0;
}
