// ============================================================================
// test_kasa_units.cpp — ★환산 계층 단위시험★ (ROS spin 없음, 순수 산수)
//
//  이 이식에서 제일 위험한 것은 "빌드도 되고 토픽도 붙는데 차만 다르게 움직이는"
//  조용한 실패다. 그 실패는 전부 세 가지 환산에서 나온다:
//      ① m/s → 정수 펄스   ② 도로휠각 → pot 지령   ③ + 좌 → − 좌
//  숫자의 근거는 white1/CHANGELOG.md·BRAKING.md 에 있고, 여기서는 그 값이
//  코드에 실제로 실렸는지만 확인한다.
// ============================================================================
#include <cmath>
#include <cstdio>
#include <string>

#include "lidar/kasa_units.hpp"

namespace ku = lidar::kasa;

static int g_fail = 0;

static void check(bool ok, const std::string& what) {
  std::printf("%s  %s\n", ok ? "  ok " : "★FAIL", what.c_str());
  if (!ok) ++g_fail;
}

static void near(double got, double want, double tol, const std::string& what) {
  const bool ok = std::fabs(got - want) <= tol;
  std::printf("%s  %-52s got=%.4f want=%.4f\n", ok ? "  ok " : "★FAIL",
              what.c_str(), got, want);
  if (!ok) ++g_fail;
}

int main() {
  std::printf("\n=== 1. 제원 (white1/driving.py 상수절) ===\n");
  near(ku::WHEELBASE_M, 1.25, 1e-9, "휠베이스 1.25 m (1/5카 0.75 아님)");
  near(ku::STEER_ROAD_MAX_DEG, 31.746, 0.01, "도로휠 상한 = 40/1.26");
  near(ku::MIN_TURN_RADIUS_M, 2.02, 0.01, "최소회전반경");
  near(ku::LFD_NO_SATURATE_M, 4.04, 0.01, "포화 없는 LFD = 2R");
  // 검산 : L / tan(31.7°) 가 정말 2.02 인가
  near(ku::WHEELBASE_M / std::tan(ku::STEER_ROAD_MAX_RAD), 2.02, 0.02,
       "L/tan(δmax) 검산");

  std::printf("\n=== 2. m/s → 펄스 (1펄스 = 0.884 m/s) ===\n");
  check(ku::msToPulse(0.0) == 0, "0 m/s → 0펄스");
  check(ku::msToPulse(-3.0) == 0, "음수 → 0펄스 (후진 없음)");
  check(ku::msToPulse(0.884) == 1, "0.884 → 1펄스");
  check(ku::msToPulse(1.768) == 2, "1.768 → 2펄스 ★기본 순항★");
  check(ku::msToPulse(3.536) == 4, "3.536 → 4펄스");
  check(ku::msToPulse(8.333) == 4, "8.333(30km/h) → 4펄스 (운용 상한에서 잘림)");
  check(ku::msToPulse(8.333, ku::PULSE_PROTOCOL_MAX) == 9,
        "8.333 → 9펄스 (상한을 15로 열었을 때)");
  // ★원본 기본값이 이 차에서 무엇이 되는가★ — 이식 사고의 진원지
  check(ku::msToPulse(2.0) == 2, "e_stop 원본 linear_speed 2.0 → 2펄스");
  check(ku::msToPulse(0.32) == 0, "mppi 원본 desired_speed 0.32 → ★0펄스★");
  near(ku::pulseToKmh(2), 6.364, 0.01, "2펄스 = 6.36 km/h");
  near(ku::pulseToKmh(4), 12.728, 0.01, "4펄스 = 12.73 km/h");

  std::printf("\n=== 3. 도로휠각 → pot 지령 (pot = 1.26δ + 5.17·v²tanδ/L) ===\n");
  // 정지 상태(v=0)면 언더스티어 항이 사라져 링키지비만 남는다
  near(ku::roadWheelToPotDeg(10.0, 0.0), 12.6, 1e-6, "δ=10°, v=0 → 12.6°");
  // CHANGELOG 2026-08-12(3) 의 검산표 : 도로휠 10° / v=3.54 → pot 21.74°
  near(ku::roadWheelToPotDeg(10.0, 3.54), 21.74, 0.05,
       "δ=10°, v=3.54 → 21.74° (CHANGELOG 검산표)");
  // 같은 표 : 도로휠 5° / v=3.54 → 10.83°
  near(ku::roadWheelToPotDeg(5.0, 3.54), 10.83, 0.05,
       "δ=5°, v=3.54 → 10.83° (CHANGELOG 검산표)");
  // 같은 표 : 도로휠 20° / v=3.54 → 40° 포화
  near(ku::roadWheelToPotDeg(20.0, 3.54), 40.0, 1e-6, "δ=20° → ±40 포화");
  // ★부호 보존★ (반전은 KasaActuator::drive() 가 한다 — 여기서는 하지 않는다)
  near(ku::roadWheelToPotDeg(-10.0, 3.54), -21.74, 0.05, "부호 보존 (좌)");
  near(ku::roadWheelToPotDeg(0.0, 3.54), 0.0, 1e-9, "0 → 0");
  // ★보정을 빼먹으면 얼마나 덜 꺾이나★ — 이식에서 놓치기 제일 쉬운 곳
  check(ku::roadWheelToPotDeg(10.0, 0.0) > 10.0 * 1.2,
        "pot 지령은 도로휠각보다 크다 (1.26배 이상)");

  std::printf("\n=== 4. 정지거리 (BRAKING.md 4절 표) ===\n");
  const double v4 = ku::pulseToMs(4);      // 3.536 m/s
  const double v2 = ku::pulseToMs(2);      // 1.768 m/s
  near(ku::stopDist(v4, ku::A_BRAKE2_MS2, ku::BRAKE2_LAG_S), 3.90, 0.10,
       "4펄스 2단 (지연 0.30 포함)");
  near(v4 * v4 / (2.0 * ku::A_BRAKE1_MS2), 4.81, 0.05,
       "4펄스 1단 4.8 m (1.30 뭉갠값, 지연 이중계상 없이)");
  near(v4 * v4 / (2.0 * ku::A_COAST_MS2), 15.25, 0.20,
       "4펄스 코스트 15.2 m ★AEB 8.5m 안에 못 선다★");
  near(v2 * v2 / (2.0 * ku::A_COAST_MS2), 3.81, 0.10,
       "2펄스 코스트 3.8 m");
  near(v2 * v2 / (2.0 * ku::A_BRAKE1_MS2), 1.20, 0.05,
       "2펄스 1단 1.2 m");
  check(ku::stopDist(0.0, ku::A_BRAKE1_MS2, ku::BRAKE1_LAG_S) == 0.0,
        "정지 중 정지거리 0");

  std::printf("\n=== 5. 반올림 규칙 (arduino.py _round_half_away 와 동일) ===\n");
  check(ku::roundHalfAway(0.5) == 1, "0.5 → 1");
  check(ku::roundHalfAway(1.5) == 2, "1.5 → 2 (은행가 반올림 아님)");
  check(ku::roundHalfAway(2.5) == 3, "2.5 → 3");
  check(ku::roundHalfAway(-0.5) == -1, "-0.5 → -1");

  std::printf("\n%s  (%d fail)\n\n", g_fail ? "★★ 실패 ★★" : "전부 통과", g_fail);
  return g_fail == 0 ? 0 : 1;
}
