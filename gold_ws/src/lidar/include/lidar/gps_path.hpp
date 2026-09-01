// ============================================================================
// gps_path.hpp — ENU / straight-line fit / Stanley tracking (no ROS)
//
// Convention matches white/gps_imu + driving:
//   x = East, y = North
//   heading 0° = East, 90° = North (ENU atan2(dy, dx))
//   vehicle +Y is left. Stanley e_y > 0 → left steer (path is left of vehicle).
// ============================================================================
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace lidar {

inline constexpr double kEarthRadiusM = 6378137.0;

struct EnuPoint {
  double x = 0.0;
  double y = 0.0;
};

struct LatLon {
  double lat = 0.0;
  double lon = 0.0;
};

struct GpsSample {
  double lat = 0.0;
  double lon = 0.0;
  double heading_deg = 0.0;
  double speed = 0.0;
  double steer = 0.0;
  int direction = 1;
  double pitch = 0.0;
  double terrain = 0.0;
};

struct StraightFit {
  bool valid = false;
  double x0 = 0.0;  // ENU start on the fitted line
  double y0 = 0.0;
  double ux = 1.0;  // unit direction (start → end)
  double uy = 0.0;
  double length = 0.0;
  double rms = 0.0;  // residual [m]
  double heading_deg = 0.0;
  double lat0 = 0.0;
  double lon0 = 0.0;
};

struct TrackState {
  double s = 0.0;            // progress along line [m]
  double remaining = 0.0;    // length - s
  double e_y = 0.0;          // Stanley lateral (path left of vehicle → +)
  double heading_err = 0.0;  // path - vehicle [rad]
  EnuPoint closest;
  EnuPoint lookahead;
};

inline double normalizeRad(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

inline double normalizeDeg(double a) {
  while (a > 180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}

inline EnuPoint latlonToEnu(double lat, double lon, double lat0, double lon0) {
  EnuPoint p;
  p.x = kEarthRadiusM * std::cos(lat0 * M_PI / 180.0) *
        (lon - lon0) * M_PI / 180.0;
  p.y = kEarthRadiusM * (lat - lat0) * M_PI / 180.0;
  return p;
}

inline LatLon enuToLatLon(double x, double y, double lat0, double lon0) {
  LatLon ll;
  ll.lat = lat0 + (y / kEarthRadiusM) * 180.0 / M_PI;
  const double c = std::cos(lat0 * M_PI / 180.0);
  ll.lon = lon0 + (x / (kEarthRadiusM * std::max(c, 1e-9))) * 180.0 / M_PI;
  return ll;
}

// First principal axis through the cloud. Origin is the projection of the
// first sample; direction is flipped so it points toward the last sample.
inline StraightFit fitStraightLine(const std::vector<EnuPoint>& pts,
                                   double lat0, double lon0,
                                   double min_length = 2.0) {
  StraightFit fit;
  fit.lat0 = lat0;
  fit.lon0 = lon0;
  if (pts.size() < 2) return fit;

  double mx = 0.0, my = 0.0;
  for (const auto& p : pts) {
    mx += p.x;
    my += p.y;
  }
  mx /= static_cast<double>(pts.size());
  my /= static_cast<double>(pts.size());

  double sxx = 0.0, sxy = 0.0, syy = 0.0;
  for (const auto& p : pts) {
    const double dx = p.x - mx;
    const double dy = p.y - my;
    sxx += dx * dx;
    sxy += dx * dy;
    syy += dy * dy;
  }

  // Dominant eigenvector of [[sxx,sxy],[sxy,syy]]
  const double trace = sxx + syy;
  const double det = sxx * syy - sxy * sxy;
  const double disc = std::max(0.0, trace * trace * 0.25 - det);
  const double l1 = trace * 0.5 + std::sqrt(disc);
  double ux = sxy;
  double uy = l1 - sxx;
  if (std::hypot(ux, uy) < 1e-9) {
    ux = l1 - syy;
    uy = sxy;
  }
  const double n = std::hypot(ux, uy);
  if (n < 1e-9) {
    // Degenerate: fall back to first→last chord
    ux = pts.back().x - pts.front().x;
    uy = pts.back().y - pts.front().y;
  } else {
    ux /= n;
    uy /= n;
  }
  double chord_x = pts.back().x - pts.front().x;
  double chord_y = pts.back().y - pts.front().y;
  if (ux * chord_x + uy * chord_y < 0.0) {
    ux = -ux;
    uy = -uy;
  }
  const double un = std::hypot(ux, uy);
  if (un < 1e-9) return fit;
  ux /= un;
  uy /= un;

  double s_min = 1e300, s_max = -1e300;
  double sse = 0.0;
  for (const auto& p : pts) {
    const double s = ux * (p.x - mx) + uy * (p.y - my);
    s_min = std::min(s_min, s);
    s_max = std::max(s_max, s);
    const double cx = mx + s * ux;
    const double cy = my + s * uy;
    const double dx = p.x - cx;
    const double dy = p.y - cy;
    sse += dx * dx + dy * dy;
  }
  const double length = s_max - s_min;
  if (length < min_length) return fit;

  fit.valid = true;
  fit.ux = ux;
  fit.uy = uy;
  fit.x0 = mx + s_min * ux;
  fit.y0 = my + s_min * uy;
  fit.length = length;
  fit.rms = std::sqrt(sse / static_cast<double>(pts.size()));
  fit.heading_deg = normalizeDeg(std::atan2(uy, ux) * 180.0 / M_PI);
  return fit;
}

inline std::vector<EnuPoint> resampleLine(const StraightFit& fit,
                                          double spacing) {
  std::vector<EnuPoint> out;
  if (!fit.valid || fit.length <= 0.0) return out;
  const double ds = std::max(spacing, 0.05);
  const int n = std::max(2, static_cast<int>(std::lround(fit.length / ds)) + 1);
  out.reserve(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    const double s =
        fit.length * static_cast<double>(i) / static_cast<double>(n - 1);
    out.push_back({fit.x0 + s * fit.ux, fit.y0 + s * fit.uy});
  }
  return out;
}

inline std::vector<GpsSample> samplesFromFit(const StraightFit& fit,
                                             double spacing) {
  std::vector<GpsSample> out;
  const auto pts = resampleLine(fit, spacing);
  out.reserve(pts.size());
  for (const auto& p : pts) {
    const LatLon ll = enuToLatLon(p.x, p.y, fit.lat0, fit.lon0);
    GpsSample s;
    s.lat = ll.lat;
    s.lon = ll.lon;
    s.heading_deg = fit.heading_deg;
    s.direction = 1;
    out.push_back(s);
  }
  return out;
}

inline TrackState trackStraight(const StraightFit& fit, EnuPoint pos,
                                double heading_rad, double lookahead_m) {
  TrackState st;
  if (!fit.valid) return st;
  const double s_raw = fit.ux * (pos.x - fit.x0) + fit.uy * (pos.y - fit.y0);
  st.s = std::clamp(s_raw, 0.0, fit.length);
  st.remaining = std::max(0.0, fit.length - st.s);
  st.closest.x = fit.x0 + st.s * fit.ux;
  st.closest.y = fit.y0 + st.s * fit.uy;

  // Signed distance left of directed line. Vehicle south of an east-going
  // line is to the RIGHT of the path → s_left < 0. Stanley e_y wants the
  // opposite sign (path left of vehicle → +).
  const double s_left =
      fit.ux * (pos.y - fit.y0) - fit.uy * (pos.x - fit.x0);
  st.e_y = -s_left;

  const double path_psi = std::atan2(fit.uy, fit.ux);
  st.heading_err = normalizeRad(path_psi - heading_rad);

  const double s_ld = std::clamp(st.s + std::max(lookahead_m, 0.3), 0.0,
                                 fit.length);
  st.lookahead.x = fit.x0 + s_ld * fit.ux;
  st.lookahead.y = fit.y0 + s_ld * fit.uy;
  return st;
}

// Stanley steer [rad]. e_y > 0 and heading_err > 0 both command left steer.
inline double stanleyGps(double e_y, double heading_err, double v, double k,
                         double max_steer_rad, double psi_weight = 1.0) {
  const double v_eff = std::max(std::fabs(v), 0.5);
  double delta = psi_weight * heading_err + std::atan2(k * e_y, v_eff);
  return std::clamp(delta, -max_steer_rad, max_steer_rad);
}

inline double distanceSpeedScale(double d, double start_d, double end_d) {
  if (!std::isfinite(d)) return 1.0;
  if (d <= end_d) return 0.0;
  if (d >= start_d) return 1.0;
  return (d - end_d) / std::max(start_d - end_d, 1e-3);
}

/// 경로 CSV 를 어디에 둘 것인가. ★white1 의 paths.py 와 같은 폴더를 봐야 한다★
/// (prompt·mapping·driving 이 각자 상수를 들면 반드시 어긋난다 — paths.py 헤더가
///  구 white 에서 실제로 그랬다고 적어 두었다.)
/// 우선순위는 paths.py 와 같게 맞춘다:
///   1) 노드 파라미터 data_dir (부르는 쪽)
///   2) 환경변수 WHITE1_DATA_DIR
///   3) 호출자가 넘겨준 힌트(설치본에서 되찾은 white1 소스 트리)
///   4) ~/white1/gps_data
inline std::string defaultGpsDataDir(const std::string& hint = "") {
  const char* env = std::getenv("WHITE1_DATA_DIR");
  if (env && env[0] != '\0') return std::string(env);
  if (!hint.empty()) return hint;
  const char* home = std::getenv("HOME");
  if (home && home[0] != '\0') {
    return std::string(home) + "/white1/gps_data";
  }
  return "/tmp/gps_data";
}

}  // namespace lidar
