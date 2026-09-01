// ============================================================================
// kasa_units.hpp — ★금색차(kasa) 액추에이터 계약 스냅샷★
//
//  정본은 `lidar/include/lidar/kasa_units.hpp` 다. 이 패키지가 lidar 에
//  컴파일 의존하지 않도록 복사해 둔다. 펄스·pot·부호·리니어 규칙을 바꿀 때는
//  ★정본을 먼저 고치고 여기도 맞춰라★. namespace 는 정본과 같이 lidar::kasa.
//
//  이 패키지의 주행 노드들은 원래 1/5카(헤네스 브룬 T870)용으로 쓰인 것이고,
//  거기서는 `/cmd_vel_raw` 가 **m/s + 도로휠각(+ = 좌)** 이었다. 금색차의
//  `nxde/arduino.py` 는 구 `white/motor.py` 의 **토픽 이름과 타입을 일부러
//  그대로 물려받았지만**, 세 필드의 뜻이 전부 다르다.
//
//    ┌──────────────┬─────────────────────┬──────────────────────────────────┐
//    │ 필드         │ 1/5카 (motor.py)    │ 금색차 (arduino.py)              │
//    ├──────────────┼─────────────────────┼──────────────────────────────────┤
//    │ linear.x     │ m/s (실수)          │ ★주행 목표펄스 정수 0~15★        │
//    │ angular.z 부호│ + = 좌              │ ★− 좌 / + 우★                    │
//    │ angular.z 물리│ 도로휠각 δ          │ ★pot 지령 (링키지비·언더스티어)★ │
//    │ 제동         │ 없음(속도지령이 곧) │ ★/brake_level 0/1/2 별 토픽★     │
//    └──────────────┴─────────────────────┴──────────────────────────────────┘
//
//  ★그래서 그대로 꽂으면 에러 없이 조용히 틀린다★ — 빌드도 되고 토픽도 붙고
//  echo 도 정상인데 차가 전혀 다르게 움직인다. 그 변환을 노드마다 흩뿌리면
//  한 곳만 고쳐지는 사고가 반드시 나므로, **이 파일 하나가 소유한다.**
//
//  ※ 값의 정본은 `white1/white1/driving.py` 상단 상수절과 `white1/BRAKING.md`,
//    `white1/BOARD_B.md`, `white1/CHANGELOG.md` 다. 실측 근거·로그 파일명까지
//    거기에 있으므로 여기서는 값과 한 줄 근거만 옮긴다. **한쪽만 고치지 말 것.**
// ============================================================================
#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace lidar {
namespace kasa {

// ══════════════════════════════════════════════════════════════════════════
//  1. 차량 제원 — 전부 실측값이다 (driving.py 'AA 차량 제원' 절)
// ══════════════════════════════════════════════════════════════════════════
//  1/5카 → 금색차로 바뀐 것 :
//     휠베이스 0.73 → ★1.25 m★ / 조향 최대 ±21° → ★도로휠 ±31.7°★
//     최소회전반경 1.90 → ★2.02 m★
//  ★휠베이스는 순수추종 조향각에 정비례한다★ 0.75 를 그대로 두면 모든 조향이
//  60% 로 축소된다 — 차를 바꿀 때 반드시 갈아야 하는 유일한 기하값이다.
constexpr double WHEELBASE_M = 1.25;      // [m] 축거 1250mm 실측

// ── 조향 전달계 (CHANGELOG 2026-08-12(3), 126표본 최소자승 / 잔차 RMS 4.2°) ──
//    pot_지령[deg] = 1.26 · δ_도로휠[deg] + 5.17 · v²/R
//  ★±40° 는 '도로휠각'이 아니다★ kasa_0804_B.ino 의 STEER_ANGLE_MAX 는 조향
//  가변저항 하드리밋 사이 ★전체 행정에 붙인 이름★ 일 뿐이다. 이 역모델이 없던
//  2026-08-11 주행에서 코너에 34° 가 필요한데 21° 만 나가 두 경로 모두 이탈했다.
constexpr double STEER_POT_MAX_DEG = 40.0;   // B보드 수용 상한 (= STEER_ANGLE_MAX)
constexpr double STEER_PLANT_GAIN  = 1.26;   // pot 지령 / 도로휠각 (링키지비)
constexpr double STEER_UNDERSTEER  = 5.17;   // [deg/(m/s²)] 타이어 슬립 보정
constexpr double STEER_ROAD_MAX_DEG = STEER_POT_MAX_DEG / STEER_PLANT_GAIN;  // 31.7°
constexpr double STEER_ROAD_MAX_RAD = STEER_ROAD_MAX_DEG * M_PI / 180.0;     // 0.553

// 최소회전반경 = L / tan(31.7°) = 2.02 m
//  ★포화 없는 선행거리(LFD) 문턱 = 2·R = 4.04 m★ 이보다 짧은 lookahead 를 쓰면
//  조향이 포화한다. 원본의 lookahead_min(1.5) 은 이 문턱의 37% 였다.
constexpr double MIN_TURN_RADIUS_M   = 2.02;
constexpr double LFD_NO_SATURATE_M   = 2.0 * MIN_TURN_RADIUS_M;

// ── 조향 불감대 (kasa_0804_B.ino STEER_TOLERANCE_EXIT = 6 raw 카운트) ──
//  신 매핑 3.375 카운트/도 → pot 1.78° → ★도로휠 1.41°★
//  LFD 5.16 m 기준 이 각도가 못 지우는 측방오차가 ★0.26 m★ 다. 그보다 작은 CTE
//  문턱을 두어도 조향이 물리적으로 반응하지 못한다(원본의 0.10~0.12 m 문턱들).
constexpr double STEER_DEADBAND_ROAD_DEG = 1.41;
constexpr double CTE_BLIND_M             = 0.26;

// ══════════════════════════════════════════════════════════════════════════
//  2. 속도 — ★m/s 가 아니라 정수 펄스다★
// ══════════════════════════════════════════════════════════════════════════
//  1펄스 = 0.884 m/s = 3.182 km/h. 바퀴 1회전 96펄스(3상 홀 6엣지 × 16극쌍),
//  175/60R13 둘레 1.6971 m, 20ms 창 기준. (mad-code/CLAUDE.md 1절)
//  ★쓸 수 있는 속도 지령이 사실상 {0,1,2,3,4} 다섯 개뿐이다★ — 연속량을 전제한
//  감속 슬루(max_drop 등)는 대부분 같은 펄스에 머물러 아무 일도 하지 않는다.
constexpr double MS_PER_PULSE  = 0.884;
constexpr double KMH_PER_PULSE = 3.182;
constexpr int    PULSE_PROTOCOL_MAX  = 15;   // A보드 수용 상한 (47.7 km/h)
constexpr int    PULSE_OPERATING_MAX = 4;    // white1 MAX_PULSE_LIMIT (12.7 km/h)

// ★A보드 재가속 함정 (kasa_0804_A.ino / CHANGELOG 2026-08-18(2))★
//    if (pwm > 0 && pwm < PWM_MAX && abs(err) < I_ACCUM_ERR_MAX /* =4 */) pidI += err;
//  정지한 차에 목표 4펄스를 주면 err 가 정확히 4 → 적분이 자라지 않아 PWM 92 에
//  영원히 고정된다. ★목표 4펄스가 목표 2펄스(PWM 111)보다 약하다.★
//  BRAKING.md 부록 A 의 "4펄스 12.4초 / 엔코더 0 / GPS 이동 0.03m" 가 이것이다.
//  → **정지 상태에서 재출발할 때는 4펄스를 피한다.**
constexpr int PULSE_RESTART_TRAP = 4;
constexpr int PULSE_RESTART_SAFE = 3;

// ══════════════════════════════════════════════════════════════════════════
//  3. 제동 — ★펄스 0 은 '정지'가 아니라 '코스트'다★ (BRAKING.md 전체)
// ══════════════════════════════════════════════════════════════════════════
//  리니어는 시킨 대로만 움직인다. 스스로 체결하지도 복귀하지도 않는다.
//    코스트(펄스 0)  0.29 ~ 0.54  대표 0.41   ← 인휠 자연감속뿐
//    1단(구동 살아있음) 0.62~1.05  평균 0.88   ← 코너 선행제동이 쓰는 값
//    1단(구동 차단)   ★1.30★ (전 구간 에너지식) / 램프 이후 1.95
//                     ※ 물린 뒤 ★첫 0.55초는 감속도 0★ (행정 램프 290카운트)
//    2단              2.2 ~ 3.8   하한 2.2    ← '감속'이 아니라 '정지'
//
//  4펄스(3.54 m/s) 정지거리 :  코스트 15.2m / 1단 4.8~5.1m / 2단 2.8m
//  ★원본의 aeb_max_decel = 5.0 은 2단 최대(3.8)보다도 크다 — 이 차에 없는 값이다★
constexpr double A_COAST_MS2  = 0.41;
constexpr double A_BRAKE1_MS2 = 1.30;
constexpr double A_BRAKE2_MS2 = 2.20;
constexpr double BRAKE1_LAG_S = 0.55;   // 1단 행정 램프 (제동력이 0인 구간)
constexpr double BRAKE2_LAG_S = 0.30;   // 2단은 행정이 길어도 제동력이 빨리 선다

constexpr int BRAKE_OFF  = 0;   // 놓음
constexpr int BRAKE_SOFT = 1;   // 행정 1/3 (pot raw 600)
constexpr int BRAKE_FULL = 2;   // 풀브레이킹 (pot raw 850)

// ★재확인 주기★ /brake_level 은 발행자가 여럿인 '마지막 발행자가 이기는' 토픽이다.
//   master 레버가 0.5초마다 자기 값(0단)을 재발행하므로, 물고 있는 동안 주기적으로
//   다시 내지 않으면 ★2단을 건 0.5초 뒤 리니어가 도로 풀린다★(실측 로그).
constexpr double BRAKE_KEEPALIVE_S = 0.25;
// ★최소 물림★ 1단 행정은 290카운트 ≈ 0.54초다. 0.2초만 물면 행정의 40% 도 못 가서
//   제동력은 거의 안 나오는데 기구만 왕복한다 — 리니어에 제일 나쁜 사용법이다.
constexpr double BRAKE_MIN_HOLD_S = 0.50;
// ★해제 유예★ arduino.py BRAKE_RELEASE_HOLD_S. 0 이 아닌 요청 뒤 이 시간 동안은
//   더 낮은 값으로 내려가지 않는다(그동안 제동 100% 유지). 해제 선행량에 포함된다.
constexpr double BRAKE_RELEASE_HOLD_S = 0.50;

// ══════════════════════════════════════════════════════════════════════════
//  3-1. ★/aeb_stop — 수동조종 중에도 통하는 유일한 제동 경로★ [2026-08-25]
// ══════════════════════════════════════════════════════════════════════════
//  위 /brake_level 은 ★자율주행에서만★ 통한다. arduino.py compose() 의 수동조종
//  분기(D5 개방)는 브레이크를 ★항상 0★ 으로 보낸다 — "제동은 사람 발이 한다,
//  모드 전환은 절대로 리니어를 체결하지 않는다" 는 2026-08-04/05 의 불변식이다.
//  그래서 사람이 페달로 몰고 있는 동안 라이다가 장애물을 봐도, /brake_level 을
//  아무리 발행해도 ★리니어는 움직이지 않는다★ (조용히 아무 일도 안 일어난다).
//
//  → 그 하나의 경우를 위해 arduino.py 에 ★별 토픽★ 을 열었다:
//        /aeb_stop (std_msgs/Bool)   true = 전방 장애물 확정
//    받으면 arduino 는 모드와 무관하게 A보드 구동을 끊고(단일값 "0" → 직접 PWM
//    해제 → 코스트) 리니어를 aeb_brake_level 단으로 물린다. 조향은 수동조종이면
//    'x'(힘빼기) 그대로다 — 사람이 핸들을 쥐고 있다.
//
//  ★왜 /brake_level 을 재사용하지 않았나★ 그 토픽은 발행자가 여럿인 '마지막
//  발행자가 이기는' 요청 토픽이고, 수동조종에서 그것을 통하게 열면 신호등 인지
//  같은 무관한 발행자가 사람이 운전하는 중에 리니어를 물릴 수 있다(2026-08-05 에
//  실제로 문제가 됐던 경로다). 이름을 갈라 두면 "리니어가 왜 나왔나" 의 답이
//  ★AEB 하나로 좁혀진다★.
//
//  ★계약★ 판단자는 이 토픽을 ★상태로, 끊기지 않게★ 낸다(20Hz 권장, true/false
//  둘 다). arduino 는 신선도(aeb_stale_s)를 보고 끊기면 해제한다 — 그래야
//  '판단자가 죽었다' 와 '장애물이 없다' 가 구별되고, 죽은 노드가 리니어를 영구히
//  물고 있는 일이 없다. 제동 ★단계★ 는 여기서 정하지 않는다(액추에이터 정책 =
//  arduino 의 aeb_brake_level). 판단자는 '섰어야 하는가' 만 말한다.
constexpr const char* AEB_STOP_TOPIC = "/aeb_stop";

// ══════════════════════════════════════════════════════════════════════════
//  4. 환산 함수
// ══════════════════════════════════════════════════════════════════════════

/// 반올림 — 0 에서 먼 쪽으로 (arduino.py `_round_half_away` 와 같은 규칙).
inline int roundHalfAway(double x) {
  return static_cast<int>(x < 0.0 ? std::ceil(x - 0.5) : std::floor(x + 0.5));
}

/// m/s → 주행 목표펄스. ★후진은 없다★ (A보드가 음수를 안 받는다).
inline int msToPulse(double v_ms, int max_pulse = PULSE_OPERATING_MAX) {
  if (!std::isfinite(v_ms) || v_ms <= 0.0) return 0;
  const int p = roundHalfAway(v_ms / MS_PER_PULSE);
  return std::clamp(p, 0, std::clamp(max_pulse, 0, PULSE_PROTOCOL_MAX));
}

inline double pulseToMs(int pulse) { return pulse * MS_PER_PULSE; }
inline double pulseToKmh(int pulse) { return pulse * KMH_PER_PULSE; }

/// 도로휠각[deg] → B보드 pot 지령[deg]. ★부호는 입력 그대로 보존한다★
///     pot = 1.26·|δ| + 5.17·v²·tan|δ|/L,   그다음 ±40 클램프
/// (white1 `driving.steer_command()` 와 같은 식 — 한쪽만 고치지 말 것)
inline double roadWheelToPotDeg(double road_deg, double v_ms) {
  const double d = std::fabs(road_deg);
  if (!std::isfinite(d) || d < 1e-6) return 0.0;
  const double v = std::isfinite(v_ms) ? v_ms : 0.0;
  const double pot = STEER_PLANT_GAIN * d
                   + STEER_UNDERSTEER * v * v
                       * std::tan(d * M_PI / 180.0) / WHEELBASE_M;
  return std::copysign(std::min(STEER_POT_MAX_DEG, pot), road_deg);
}

/// 정지거리 [m] — 반응지연 동안 그대로 간 거리 + 감속도 a 로 멈추는 거리.
/// 여유(margin)는 넣지 않는다. 쓰는 쪽이 단계마다 다르게 더한다.
inline double stopDist(double v_ms, double a_ms2, double lag_s) {
  if (!std::isfinite(v_ms) || v_ms <= 0.0) return 0.0;
  return v_ms * lag_s + (v_ms * v_ms) / (2.0 * std::max(a_ms2, 0.05));
}

// ══════════════════════════════════════════════════════════════════════════
//  5. KasaActuator — 세 노드가 공유하는 출력·게이트 계층
// ══════════════════════════════════════════════════════════════════════════
//  발행 : /cmd_vel_raw (Twist)   /control_state (Bool)   /brake_level (Int32)
//  구독 : /vehicle_mode (Bool)   /estop (Bool)
//
//  ★왜 게이트까지 여기 있나★ 원본 노드들은 D5 스위치도 E-STOP 도 보지 않는다.
//  1/5카에는 그런 것이 없었기 때문이다. 금색차에서는 그 둘이 하드웨어 사실이라,
//  모르는 채로 명령을 계속 내면 "노드는 도는데 차가 안 움직인다"가 되고 원인이
//  로그 어디에도 안 남는다. 세 노드에 같은 코드를 세 번 쓰지 않으려고 여기 둔다.
class KasaActuator {
public:
  explicit KasaActuator(rclcpp::Node* node) : node_(node) {
    max_pulse_ = declare<int>("kasa.max_pulse", PULSE_OPERATING_MAX);
    max_pulse_ = std::clamp(max_pulse_, 0, PULSE_PROTOCOL_MAX);
    require_auto_mode_ = declare<bool>("kasa.require_auto_mode", true);
    require_estop_clear_ = declare<bool>("kasa.require_estop_clear", true);
    steer_road_max_deg_ = declare<double>("kasa.steer_road_max_deg",
                                          STEER_ROAD_MAX_DEG);

    const auto qos = rclcpp::QoS(10);
    cmd_pub_   = node_->create_publisher<geometry_msgs::msg::Twist>(
        declare<std::string>("kasa.cmd_vel_topic", "/cmd_vel_raw"), qos);
    state_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/control_state", qos);
    brake_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/brake_level", qos);

    mode_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/vehicle_mode", qos,
        [this](const std_msgs::msg::Bool::ConstSharedPtr& m) {
          if (!has_mode_ || auto_mode_ != m->data) {
            RCLCPP_INFO(node_->get_logger(), "/vehicle_mode → %s",
                        m->data ? "자율주행" : "★수동조종★ (ROS 명령 무시됨)");
          }
          auto_mode_ = m->data;
          has_mode_ = true;
        });
    estop_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/estop", qos,
        [this](const std_msgs::msg::Bool::ConstSharedPtr& m) {
          if (m->data != estop_) {
            RCLCPP_WARN(node_->get_logger(), "/estop → %s",
                        m->data ? "★체결★" : "해제");
          }
          estop_ = m->data;
        });
  }

  // ── 게이트 ────────────────────────────────────────────────────────────
  /// 지금 ROS 명령이 실제로 차를 움직일 수 있는 상태인가.
  /// ★`/vehicle_mode` 를 한 번도 못 받았으면 false 다★ — arduino 노드가 안 떠
  /// 있다는 뜻이고, 그때 '가능하다'고 보면 안 움직이는 이유가 안 드러난다.
  bool ready() const {
    if (require_estop_clear_ && estop_) return false;
    if (require_auto_mode_ && !(has_mode_ && auto_mode_)) return false;
    return true;
  }
  const char* blockReason() const {
    if (require_estop_clear_ && estop_) return "E-STOP 체결 중";
    if (require_auto_mode_ && !has_mode_)
      return "/vehicle_mode 미수신 — nxde arduino 가 떠 있는지 확인";
    if (require_auto_mode_ && !auto_mode_) return "D5 스위치가 수동조종";
    return "";
  }
  bool estop() const { return estop_; }
  bool autoMode() const { return has_mode_ && auto_mode_; }

  // ── 출력 ──────────────────────────────────────────────────────────────
  /// 주행 지령. `v_ms` = m/s, `road_deg` = ★도로휠각, + = 좌 (ROS 규약)★.
  /// 여기서 ① 펄스 환산 ② pot 지령 환산 ③ 부호 반전이 한 번에 일어난다.
  /// ★부호가 뒤집히는 지점은 이 함수 하나뿐이다 — 두 번 뒤집으면 조용히 좌우가 바뀐다★
  void drive(double v_ms, double road_deg, bool control_enable) {
    const double clamped_road =
        std::clamp(road_deg, -steer_road_max_deg_, steer_road_max_deg_);
    const double pot_left_positive = roadWheelToPotDeg(clamped_road, v_ms);

    geometry_msgs::msg::Twist msg;
    msg.linear.x  = static_cast<double>(msToPulse(v_ms, max_pulse_));
    msg.angular.z = -pot_left_positive;      // ★+좌 → 보드 규약(− 좌 / + 우)★
    cmd_pub_->publish(msg);

    last_pulse_ = static_cast<int>(msg.linear.x);
    last_pot_deg_ = msg.angular.z;

    std_msgs::msg::Bool st;
    st.data = control_enable;                // ★매 틱 낸다★ (엣지 발행은 늦게 뜬
    state_pub_->publish(st);                 //   구독자를 놓친다 — QoS volatile)
  }

  /// 정지 지령 — 펄스 0, 조향은 마지막 값 유지(정면 급조향 방지).
  /// ★리니어를 여기서 만지지 않는다★ 제동은 brake() 로 명시적으로만 건다.
  void hold(bool control_enable = false) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x  = 0.0;
    msg.angular.z = last_pot_deg_;
    cmd_pub_->publish(msg);
    last_pulse_ = 0;

    std_msgs::msg::Bool st;
    st.data = control_enable;
    state_pub_->publish(st);
  }

  // ── 제동 ──────────────────────────────────────────────────────────────
  /// 브레이크 단계 요청. ★한 제동 구간 안에서 단계는 올라가기만 한다★
  /// 낮추려면 releaseBrake() 를 쓴다 — 리니어는 물리적으로 왕복하는 장치이고
  /// 그 왕복이 제일 나쁘다(2026-08-14 의 2↔0 flip-flop 과 같은 문제의식).
  ///
  /// ★1단 → 2단 승격은 최소 물림(0.5s)이 지난 뒤에만★ B보드는 진행 중인 행정이
  /// 끝나야 다음 이동을 시작하므로(`lin_state != LIN_IDLE`), 같은 틱에 1→2 를
  /// 물면 리니어를 한 번 왕복시키면서 2단이 그만큼 늦는다. 처음부터 늦었다면
  /// 1단을 건너뛰고 곧장 brake(BRAKE_FULL) 을 부를 것.
  void brake(int level) {
    level = std::clamp(level, BRAKE_OFF, BRAKE_FULL);
    if (level == BRAKE_OFF) return;                    // 해제는 releaseBrake()
    const double now = nowSec();
    if (level < stage_) return;                        // 내려가지 않는다
    if (level > stage_ && stage_ > BRAKE_OFF
        && (now - stage_t_) < BRAKE_MIN_HOLD_S) {
      return;                                          // 최소 물림 중 — 다음 틱에
    }
    if (level != stage_) {
      RCLCPP_WARN(node_->get_logger(), "🔻 브레이크 %d단 → %d단", stage_, level);
      stage_ = level;
      stage_t_ = now;
    }
    publishBrake(now, /*force=*/true);
  }

  /// 제동 해제. ★0 은 재확인하지 않는다★ '놓음'을 계속 주장하면 반대로 남의
  /// 정지를 푼다(신호등·GUI 도 같은 토픽의 발행자다).
  void releaseBrake() {
    if (stage_ == BRAKE_OFF) return;
    const double now = nowSec();
    if ((now - stage_t_) < BRAKE_MIN_HOLD_S) return;   // 행정을 끝까지 내고 푼다
    RCLCPP_INFO(node_->get_logger(), "🟢 브레이크 해제 (%d단 → 0단)", stage_);
    stage_ = BRAKE_OFF;
    stage_t_ = now;
    std_msgs::msg::Int32 m;
    m.data = 0;
    brake_pub_->publish(m);
    brake_out_ = 0;
  }

  /// 물고 있는 동안 주기적으로 다시 낸다. ★제어 루프에서 매 틱 부를 것★
  void keepBrake() {
    if (stage_ <= BRAKE_OFF) return;
    publishBrake(nowSec(), /*force=*/false);
  }

  int brakeStage() const { return stage_; }
  /// 제동을 물기 시작한 뒤 흐른 시간 [s] (안 물고 있으면 0).
  double brakeHeldSec() const {
    return stage_ > BRAKE_OFF ? (nowSec() - stage_t_) : 0.0;
  }

  /// 종료·비상 경로에서 부른다 — 구동 0, 제동 0, 조향 유지.
  void shutdown() {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 0.0;
    msg.angular.z = last_pot_deg_;
    cmd_pub_->publish(msg);
    std_msgs::msg::Bool st;
    st.data = false;
    state_pub_->publish(st);
    if (stage_ != BRAKE_OFF) {
      std_msgs::msg::Int32 m;
      m.data = 0;
      brake_pub_->publish(m);
      stage_ = BRAKE_OFF;
    }
  }

  int lastPulse() const { return last_pulse_; }
  double lastPotDeg() const { return last_pot_deg_; }
  int maxPulse() const { return max_pulse_; }
  double steerRoadMaxDeg() const { return steer_road_max_deg_; }
  /// 이 노드가 낼 수 있는 최고 속도 [m/s] — 게이트·정지거리 계산의 상한.
  double maxSpeedMs() const { return pulseToMs(max_pulse_); }

private:
  double nowSec() const { return node_->now().seconds(); }

  void publishBrake(double now, bool force) {
    if (!force && brake_out_ == stage_
        && (now - brake_t_) < BRAKE_KEEPALIVE_S) {
      return;
    }
    std_msgs::msg::Int32 m;
    m.data = stage_;
    brake_pub_->publish(m);
    brake_out_ = stage_;
    brake_t_ = now;
  }

  template <typename T>
  T declare(const std::string& name, const T& def) {
    if (!node_->has_parameter(name)) node_->declare_parameter<T>(name, def);
    return node_->get_parameter(name).get_value<T>();
  }

  rclcpp::Node* node_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr brake_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;

  int    max_pulse_ = PULSE_OPERATING_MAX;
  bool   require_auto_mode_ = true;
  bool   require_estop_clear_ = true;
  double steer_road_max_deg_ = STEER_ROAD_MAX_DEG;

  bool auto_mode_ = false;
  bool has_mode_  = false;
  bool estop_     = false;

  int    last_pulse_   = 0;
  double last_pot_deg_ = 0.0;

  int    stage_     = BRAKE_OFF;   // 지금 걸고 있는 단계
  double stage_t_   = 0.0;         // 그 단계로 바뀐 시각
  int    brake_out_ = 0;           // 마지막으로 발행한 값
  double brake_t_   = 0.0;         // 마지막 발행 시각
};

}  // namespace kasa
}  // namespace lidar
