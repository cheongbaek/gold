// ============================================================================
// line_ekf.hpp - vehicle-frame corridor centerline EKF
//
// State x = [c0, c1, half_width]^T for centerline y = c0 + c1 * x (os_sensor /
// vehicle frame, x forward, y left).
//
// Predict (no wheel odom; use commanded speed + IMU yaw rate):
//   s  = v * dt
//   dψ = ω * dt
//   c0' = c0 + c1 * s
//   c1' = c1 - dψ
//   w'  = w
//
// Update: direct measurement of (c0, c1) and optionally half_width from
// near-field left/right cone line fits, with innovation gating.
// ============================================================================
#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace lidar {

struct LineEkfConfig {
  double initial_c0 = 0.0;
  double initial_c1 = 0.0;
  double initial_half_width = 2.2;
  double half_width_min = 1.2;
  double half_width_max = 3.5;

  // Process noise densities (scaled by dt in predict)
  double q_c0 = 0.04;
  double q_c1 = 0.015;
  double q_half_width = 0.005;

  // Base measurement noise
  double r_c0 = 0.12;
  double r_c1 = 0.04;
  double r_half_width = 0.20;

  // Innovation gates
  double innov_gate_c0 = 1.0;   // m
  double innov_gate_c1 = 0.30;  // slope

  // Soft state clamps (straight corridor prior)
  double max_abs_c0 = 1.0;   // m
  double max_abs_c1 = 0.22;  // ~12.5 deg — straight mission

  double p0_c0 = 0.5;
  double p0_c1 = 0.15;
  double p0_half_width = 0.4;
};

class LineEkf {
public:
  static constexpr int N = 3;

  explicit LineEkf(const LineEkfConfig& cfg = {}) : cfg_(cfg) { reset(); }

  void reset() {
    x_ = {cfg_.initial_c0, cfg_.initial_c1, cfg_.initial_half_width};
    P_ = {};
    P_[0][0] = cfg_.p0_c0;
    P_[1][1] = cfg_.p0_c1;
    P_[2][2] = cfg_.p0_half_width;
    initialized_ = false;
  }

  void setState(double c0, double c1, double half_width) {
    x_[0] = c0;
    x_[1] = c1;
    x_[2] = clampHalfWidth(half_width);
    initialized_ = true;
  }

  bool initialized() const { return initialized_; }
  double c0() const { return x_[0]; }
  double c1() const { return x_[1]; }
  double halfWidth() const { return x_[2]; }
  const std::array<double, N>& state() const { return x_; }
  const std::array<std::array<double, N>, N>& covariance() const { return P_; }

  void predict(double v, double omega, double dt) {
    if (dt <= 0.0 || dt > 1.0) return;
    const double s = v * dt;
    const double dpsi = omega * dt;

    const double c0 = x_[0];
    const double c1 = x_[1];
    x_[0] = c0 + c1 * s;
    x_[1] = c1 - dpsi;
    clampState();

    // F: c0' = c0 + c1*s, c1' = c1 - dpsi, w' = w
    const double F[N][N] = {
        {1.0, s, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0},
    };

    double FP[N][N] = {};
    for (int i = 0; i < N; ++i)
      for (int j = 0; j < N; ++j)
        for (int k = 0; k < N; ++k) FP[i][j] += F[i][k] * P_[k][j];

    double Pn[N][N] = {};
    for (int i = 0; i < N; ++i)
      for (int j = 0; j < N; ++j)
        for (int k = 0; k < N; ++k) Pn[i][j] += FP[i][k] * F[j][k];

    Pn[0][0] += cfg_.q_c0 * dt;
    Pn[1][1] += cfg_.q_c1 * dt;
    Pn[2][2] += cfg_.q_half_width * dt;

    for (int i = 0; i < N; ++i)
      for (int j = 0; j < N; ++j) P_[i][j] = Pn[i][j];
    symmetrize();
  }

  // r_scale > 1 inflates R (weak / single-side measurements).
  // Returns false if rejected by innovation gate.
  bool update(double z_c0, double z_c1, bool has_half_width, double z_half_width,
              double r_scale = 1.0) {
    if (!initialized_) {
      x_[0] = std::clamp(z_c0, -cfg_.max_abs_c0, cfg_.max_abs_c0);
      x_[1] = std::clamp(z_c1, -cfg_.max_abs_c1, cfg_.max_abs_c1);
      if (has_half_width) x_[2] = clampHalfWidth(z_half_width);
      initialized_ = true;
      return true;
    }

    if (std::fabs(z_c0 - x_[0]) > cfg_.innov_gate_c0 ||
        std::fabs(z_c1 - x_[1]) > cfg_.innov_gate_c1) {
      return false;
    }

    const double rs = std::max(r_scale, 1.0);
    const double r0 = cfg_.r_c0 * rs;
    const double r1 = cfg_.r_c1 * rs;
    const double r2 = cfg_.r_half_width * rs;

    if (has_half_width) {
      const double y0 = z_c0 - x_[0];
      const double y1 = z_c1 - x_[1];
      const double y2 = clampHalfWidth(z_half_width) - x_[2];

      double S[N][N];
      for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) S[i][j] = P_[i][j];
      S[0][0] += r0;
      S[1][1] += r1;
      S[2][2] += r2;

      double Sinv[N][N];
      if (!invert3(S, Sinv)) return false;

      double K[N][N] = {};
      for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j)
          for (int k = 0; k < N; ++k) K[i][j] += P_[i][k] * Sinv[k][j];

      x_[0] += K[0][0] * y0 + K[0][1] * y1 + K[0][2] * y2;
      x_[1] += K[1][0] * y0 + K[1][1] * y1 + K[1][2] * y2;
      x_[2] += K[2][0] * y0 + K[2][1] * y1 + K[2][2] * y2;
      clampState();

      // P = (I - K) P
      double IKH[N][N] = {
          {1.0 - K[0][0], -K[0][1], -K[0][2]},
          {-K[1][0], 1.0 - K[1][1], -K[1][2]},
          {-K[2][0], -K[2][1], 1.0 - K[2][2]},
      };
      double Pn[N][N] = {};
      for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j)
          for (int k = 0; k < N; ++k) Pn[i][j] += IKH[i][k] * P_[k][j];
      for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) P_[i][j] = Pn[i][j];
      symmetrize();
      return true;
    }

    // 2D measurement (c0, c1)
    const double y0 = z_c0 - x_[0];
    const double y1 = z_c1 - x_[1];
    const double s00 = P_[0][0] + r0;
    const double s01 = P_[0][1];
    const double s10 = P_[1][0];
    const double s11 = P_[1][1] + r1;
    const double det = s00 * s11 - s01 * s10;
    if (std::fabs(det) < 1e-12) return false;
    const double inv00 = s11 / det;
    const double inv01 = -s01 / det;
    const double inv10 = -s10 / det;
    const double inv11 = s00 / det;

    double K[N][2] = {};
    for (int i = 0; i < N; ++i) {
      K[i][0] = P_[i][0] * inv00 + P_[i][1] * inv10;
      K[i][1] = P_[i][0] * inv01 + P_[i][1] * inv11;
    }

    for (int i = 0; i < N; ++i) x_[i] += K[i][0] * y0 + K[i][1] * y1;
    clampState();

    double KH[N][N] = {};
    for (int i = 0; i < N; ++i) {
      KH[i][0] = K[i][0];
      KH[i][1] = K[i][1];
      KH[i][2] = 0.0;
    }
    double Pn[N][N] = {};
    for (int i = 0; i < N; ++i)
      for (int j = 0; j < N; ++j) {
        double sum = 0.0;
        for (int k = 0; k < N; ++k) sum += KH[i][k] * P_[k][j];
        Pn[i][j] = P_[i][j] - sum;
      }
    for (int i = 0; i < N; ++i)
      for (int j = 0; j < N; ++j) P_[i][j] = Pn[i][j];
    symmetrize();
    return true;
  }

  // c0-only update (when slope observation is unreliable). H = [1, 0, 0]
  bool updateC0(double z_c0, double r_scale = 1.0) {
    if (!initialized_) {
      x_[0] = std::clamp(z_c0, -cfg_.max_abs_c0, cfg_.max_abs_c0);
      initialized_ = true;
      return true;
    }
    if (std::fabs(z_c0 - x_[0]) > cfg_.innov_gate_c0) return false;

    const double r0 = cfg_.r_c0 * std::max(r_scale, 1.0);
    const double innov = z_c0 - x_[0];
    const double S = P_[0][0] + r0;
    if (S < 1e-12) return false;

    double K[N];
    for (int i = 0; i < N; ++i) K[i] = P_[i][0] / S;

    for (int i = 0; i < N; ++i) x_[i] += K[i] * innov;
    clampState();

    // P = (I - K H) P, H picks column 0
    double Pcol0[N];
    for (int i = 0; i < N; ++i) Pcol0[i] = P_[i][0];
    for (int i = 0; i < N; ++i)
      for (int j = 0; j < N; ++j) P_[i][j] -= K[i] * Pcol0[j];
    symmetrize();
    if (P_[0][0] < 1e-6) P_[0][0] = 1e-6;
    return true;
  }

  // Stanley straight-hold steer [rad].
  // e_y = c0 (+ path left of vehicle → left steer), e_ψ = atan(c1)
  // psi_weight scales heading term ( <1 damps noisy slope on straight courses ).
  static double stanleySteer(double c0, double c1, double v, double k_stanley,
                             double max_steer_rad, double psi_weight = 0.5) {
    const double e_y = c0;
    const double e_psi = psi_weight * std::atan(c1);
    const double v_eff = std::max(std::fabs(v), 0.5);
    double delta = e_psi + std::atan2(k_stanley * e_y, v_eff);
    return std::clamp(delta, -max_steer_rad, max_steer_rad);
  }

private:
  double clampHalfWidth(double w) const {
    return std::clamp(w, cfg_.half_width_min, cfg_.half_width_max);
  }

  void clampState() {
    x_[0] = std::clamp(x_[0], -cfg_.max_abs_c0, cfg_.max_abs_c0);
    x_[1] = std::clamp(x_[1], -cfg_.max_abs_c1, cfg_.max_abs_c1);
    x_[2] = clampHalfWidth(x_[2]);
  }

  void symmetrize() {
    for (int i = 0; i < N; ++i)
      for (int j = i + 1; j < N; ++j) {
        const double m = 0.5 * (P_[i][j] + P_[j][i]);
        P_[i][j] = P_[j][i] = m;
      }
  }

  static bool invert3(const double A[N][N], double out[N][N]) {
    const double a = A[0][0], b = A[0][1], c = A[0][2];
    const double d = A[1][0], e = A[1][1], f = A[1][2];
    const double g = A[2][0], h = A[2][1], i = A[2][2];
    const double A_ = e * i - f * h;
    const double B_ = f * g - d * i;
    const double C_ = d * h - e * g;
    const double D_ = c * h - b * i;
    const double E_ = a * i - c * g;
    const double F_ = b * g - a * h;
    const double G_ = b * f - c * e;
    const double H_ = c * d - a * f;
    const double I_ = a * e - b * d;
    const double det = a * A_ + b * B_ + c * C_;
    if (std::fabs(det) < 1e-12) return false;
    const double inv = 1.0 / det;
    out[0][0] = A_ * inv;
    out[0][1] = D_ * inv;
    out[0][2] = G_ * inv;
    out[1][0] = B_ * inv;
    out[1][1] = E_ * inv;
    out[1][2] = H_ * inv;
    out[2][0] = C_ * inv;
    out[2][1] = F_ * inv;
    out[2][2] = I_ * inv;
    return true;
  }

  LineEkfConfig cfg_;
  std::array<double, N> x_{};
  std::array<std::array<double, N>, N> P_{};
  bool initialized_ = false;
};

}  // namespace lidar
