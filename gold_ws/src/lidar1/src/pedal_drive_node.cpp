// ============================================================================
// pedal_drive_node.cpp — ★페달 주행 담당 + AEB★ (수동조종 시험용 구동 노드)
//
//   ros2 launch lidar one_launch.py        ← 이 노드를 함께 띄운다
//   ros2 run lidar pedal_drive_node        ← 단독 (파라미터 기본값으로)
//
// ══════════════════════════════════════════════════════════════════════════
//  ★★ 한 줄 요약 : '수동조종 모드에 AEB 하나만 얹은 것' ★★
// ══════════════════════════════════════════════════════════════════════════
//  주행은 ★사람이 페달로★ 한다 (B보드 D5 = 수동조종). 페달은 arduino.py 가
//  ★직접 PWM 16~255★ 로 A보드에 내린다 — white1 과 같다(끝까지 밟으면 PWM 255).
//  이 노드는 그 위에 '앞이 막히면 세운다' 하나만 더한다.
//
//    cone_lidar_node ──/cone_lidar_node/stop_signal──▶ 이 노드 ──/aeb_stop──▶ arduino
//        (인지·판정)                                   (확정·래치)   (구동차단 + 리니어)
//                                                          └───────────▶ nxde sound
//                                                                        (경고음 반복)
//
//  ★drive_lidar_node 를 쓰지 않는 이유★ 그쪽은 라바콘 복도를 ★스스로 출발해서★
//  고정 2펄스로 달리는 자율주행 노드다(런치 = 출발). 비상정지 사슬만 시험하려면
//  차를 스스로 움직이는 부분이 하나도 없어야 한다.
//
// ══════════════════════════════════════════════════════════════════════════
//  ★★ 발행하지 않는 것 — 이 노드의 안전 성질 ★★
// ══════════════════════════════════════════════════════════════════════════
//  /cmd_vel_raw · /control_state · /brake_level 을 ★하나도 발행하지 않는다★.
//    · 수동조종에서 차를 굴리는 것은 사람 발이다. 소프트웨어가 구동을 낼 길이
//      있으면 그만큼 사람 조작과 다툰다(arduino.py (2) 분기의 결론과 같다).
//    · ★수동조종에서는 /brake_level 이 아예 통하지 않는다★ arduino 의 수동 분기가
//      브레이크를 항상 0 으로 보내기 때문이다(제동은 사람 발이 한다는 불변식).
//      그래서 '수동조종 중에 소프트웨어가 세울' 유일한 경로가 /aeb_stop 이다 —
//      그것만 arduino 의 우선순위 (1-1) 로 수동 분기보다 먼저 걸린다.
//    · 제동 ★단계★(0/1/2)는 액추에이터 정책이라 arduino 가 소유한다
//      (aeb_brake_level). 이 노드는 '섰어야 하는가' 만 말한다.
//  → 발행은 ★/aeb_stop (Bool) 하나뿐★. 이 노드를 잘못 띄워도 차가 앞으로
//    나가는 일은 원리적으로 없다.
//
// 발행:
//   /aeb_stop (std_msgs/Bool)  20Hz 로 ★항상★ 낸다 (true / false 둘 다)
//        ★엣지가 아니라 상태를 계속 낸다★ 구독자(arduino·sound)가 이 토픽의
//        ★신선도★ 로 '판단자가 살아 있는가' 를 보기 때문이다. 엣지만 내면
//        이 노드가 죽은 것과 '장애물 없음' 이 구별되지 않는다.
//
// 구독:
//   ~stop_signal_topic       (Bool)    cone_lidar_node 의 확정 정지신호  ← ★판단 입력★
//   ~obstacle_distance_topic (Float32) 최근접 전방 거리 [m]
//   /encoder                 (Int32)   실측 주행펄스 ★좌+우 합★ → 안전속도 감시
//   /vehicle_mode            (Bool)    D5 (로그 전용 — 판단에 쓰지 않는다)
//   /estop                   (Bool)    하드웨어 E-STOP (로그 전용)
//
//  ★모드·E-STOP 을 판단에 쓰지 않는 이유★ 둘 중 무엇이든 걸려 있으면 arduino 가
//  ★자기 분기 우선순위로★ 이미 차를 세우고 있다. 여기서 /aeb_stop 을 같이
//  내려버리면 "라이다는 장애물을 보는데 토픽은 false" 인 구간이 생겨 로스백에서
//  인지 성능을 못 읽는다. ★이 토픽은 인지의 사실을 말한다★.
//
// ══════════════════════════════════════════════════════════════════════════
//  ★★ 래치 — 리니어를 왕복시키지 않는 것이 요점이다 ★★
// ══════════════════════════════════════════════════════════════════════════
//  리니어는 물리적으로 왕복하는 장치이고 1단 행정만 해도 ≈0.54초다(BRAKING.md).
//  깜빡이는 신호를 그대로 흘리면 제동력은 안 나오는데 기구만 왕복한다 — 제일
//  나쁜 사용법이다. 그래서 세 가지로 붙잡는다:
//     engage_frames    확정에 필요한 연속 프레임 (cone_lidar 의 confirm_frames 뒤 한 번 더)
//     min_engage_s     한 번 물면 최소 이만큼 유지 (행정을 끝까지 낸다)
//     release_clear_s  '비었다' 가 이만큼 이어져야 해제 (사각지대 통과 방어)
//
//  ★신선도 — 판정이 끊기면 fail-open★ stop_signal 이 signal_stale_s 넘게 안 오면
//  (cone_lidar 가 죽었거나 라이다가 끊겼다) 장애물 없음으로 보고 해제한다.
//    · 그 상태는 ★AEB 가 아예 없는 수동조종★ = 원래 상태다. 사람이 페달과 핸들을
//      쥐고 있으므로, 여기서 제동을 물면 사람이 예상하지 못한 정지가 된다.
//    · 대신 ★조용히 넘기지 않는다★ 1초마다 경고한다. arduino 도 /aeb_stop 신선도를
//      따로 보므로(aeb_stale_s) 이 노드가 통째로 죽어도 결론은 같다.
//
// ══════════════════════════════════════════════════════════════════════════
//  ★★ 속도 제한이 없다 — 페달이 곧 속도다 ★★
// ══════════════════════════════════════════════════════════════════════════
//  [2026-08-25] 이 노드는 ★속도에 관해 아무것도 하지 않는다★.
//    · 제한하지 않는다 — 애초에 구동 지령을 만들지 않으니 제한할 수단도 없다.
//    · ★판정하지도, 경고하지도 않는다★ 한때 '리니어 2단으로 설 수 있는 속도
//      상한' 을 계산해 로그에 찍었는데, 그 숫자가 로그에 남아 있는 것 자체가
//      '무언가가 걸려 있다' 로 읽혔다. 실제로 제한한 적은 없지만, ★있지도 않은
//      상한을 읽게 만드는 로그는 없는 편이 낫다★ — 그래서 계산까지 없앴다.
//  속도를 실제로 묶어야 하면 곳은 하나다: 런치의 ★manual_pwm_max★.
//  여기서는 실측 속도를 ★그냥 보여줄 뿐★ 이다 — 판정에 쓰지 않는다.
//
//  ⚠️ 그러므로 '앞이 막히면 선다' 는 ★감지 거리 안에서 설 수 있는 속도일 때만★
//     성립한다. 그 판단은 사람이 한다. 처음 시험은 라바콘·박스로, 살짝만 밟아서.
// ============================================================================

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "lidar/kasa_units.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"

namespace {

/// /encoder 1카운트 = 0.442 m/s. ★좌+우 펄스의 합★ 이므로 바퀴 하나 기준으로
/// 보려면 반으로 접는다 (white1 mapping.ENC_SUM_TO_PULSE = 0.5 와 같은 값).
/// ★한쪽만 고치지 말 것★ — 같은 토픽을 두 노드가 다르게 읽으면 조용히 틀린다.
constexpr double ENC_SUM_TO_MS = 0.5 * lidar::kasa::MS_PER_PULSE;   // 0.442

}  // namespace

class PedalDriveNode : public rclcpp::Node {
public:
  PedalDriveNode() : Node("pedal_drive_node") {
    // ── 토픽 이름 ── cone_lidar_node 와 arduino 양쪽과 맞아야 한다.
    stop_signal_topic_ = declare_parameter<std::string>(
        "stop_signal_topic", "/cone_lidar_node/stop_signal");
    obstacle_distance_topic_ = declare_parameter<std::string>(
        "obstacle_distance_topic", "/cone_lidar_node/obstacle_distance");
    // ★arduino 의 aeb_topic 과 같아야 한다★ 한쪽만 바꾸면 이름이 갈라져
    //   "노드는 도는데 차가 안 선다" 가 된다 — 런치가 두 값을 함께 넘긴다.
    aeb_stop_topic_ = declare_parameter<std::string>(
        "aeb_stop_topic", lidar::kasa::AEB_STOP_TOPIC);

    // ── 래치 ── 근거는 파일 헤더 '래치' 절.
    publish_period_s_ = declare_parameter<double>("publish_period_s", 0.05);
    engage_frames_    = declare_parameter<int>("engage_frames", 1);
    min_engage_s_     = declare_parameter<double>("min_engage_s", 1.0);
    release_clear_s_  = declare_parameter<double>("release_clear_s", 1.5);
    signal_stale_s_   = declare_parameter<double>("signal_stale_s", 1.0);
    log_period_s_     = declare_parameter<double>("log_period_s", 1.0);

    // ── 실측 속도 표시 ── ★로그 전용이다★ 판정·제한에 쓰지 않는다(헤더 참고).
    use_encoder_       = declare_parameter<bool>("use_encoder", true);
    //   ★/encoder 신선도★ 이 시간 넘게 안 오면 속도를 '모른다(0)' 로 본다.
    //   arduino 가 20Hz 로 계속 내므로 1.0 은 20틱이 빠진 것이다. ★얼어붙은
    //   마지막 값을 계속 쓰지 않는다★ — 이 파일에서 제일 조용히 틀릴 수 있는
    //   곳이 그것이다(로그에는 그럴듯한 숫자가 계속 찍힌다). 대신 끊긴 사실을
    //   경고로 남긴다: 그동안 안전속도 감시는 눈을 감은 상태다.
    enc_stale_s_       = declare_parameter<double>("enc_stale_s", 1.0);

    engage_frames_ = std::max(1, engage_frames_);

    const auto qos = rclcpp::QoS(10);
    aeb_pub_ = create_publisher<std_msgs::msg::Bool>(aeb_stop_topic_, qos);

    // ── ★판단 입력★ 이 하나뿐이다 ──
    stop_sub_ = create_subscription<std_msgs::msg::Bool>(
        stop_signal_topic_, rclcpp::QoS(5),
        [this](const std_msgs::msg::Bool::ConstSharedPtr& m) {
          signal_ = m->data;
          last_signal_t_ = nowSec();
          if (!signal_seen_) {
            signal_seen_ = true;
            RCLCPP_INFO(get_logger(), "✅ 정지신호 수신 시작 — %s",
                        stop_signal_topic_.c_str());
          }
        });
    dist_sub_ = create_subscription<std_msgs::msg::Float32>(
        obstacle_distance_topic_, rclcpp::QoS(5),
        [this](const std_msgs::msg::Float32::ConstSharedPtr& m) {
          dist_m_ = m->data;
        });

    // ── 실측 속도 ── 안전속도 감시·로그 전용. ★판단에 쓰지 않는다★
    //   A보드 기동 블랭킹 구간에 허수 홀 카운트가 섞여 나오므로(white1/speed.py)
    //   중앙값 3 으로 스파이크만 걷어낸다. 그 이상 평활하면 경고가 늦는다.
    if (use_encoder_) {
      encoder_sub_ = create_subscription<std_msgs::msg::Int32>(
          "/encoder", qos,
          [this](const std_msgs::msg::Int32::ConstSharedPtr& m) {
            enc_buf_[enc_idx_ % enc_buf_.size()] = m->data;
            enc_idx_++;
            has_encoder_ = enc_idx_ >= enc_buf_.size();
            last_enc_t_ = nowSec();
          });
    }

    // ── 아래 둘은 ★로그 전용★ 이다 (판단에 쓰지 않는다 — 헤더 참고) ──
    mode_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/vehicle_mode", qos,
        [this](const std_msgs::msg::Bool::ConstSharedPtr& m) {
          if (!has_mode_ || auto_mode_ != m->data) {
            RCLCPP_INFO(get_logger(), "/vehicle_mode → %s",
                        m->data ? "자율주행 (★이 시험은 수동조종이 기본이다★)"
                                : "수동조종 (이 시험의 기본)");
          }
          auto_mode_ = m->data;
          has_mode_ = true;
        });
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/estop", qos,
        [this](const std_msgs::msg::Bool::ConstSharedPtr& m) {
          if (m->data != estop_) {
            RCLCPP_WARN(get_logger(), "/estop → %s",
                        m->data ? "★체결★" : "해제");
          }
          estop_ = m->data;
        });

    timer_ = create_wall_timer(
        std::chrono::duration<double>(std::max(0.01, publish_period_s_)),
        [this]() { tick(); });

    RCLCPP_INFO(
        get_logger(),
        "🛑 페달 주행 + AEB 감시 시작\n"
        "     구독 : %s\n"
        "     발행 : %s (Bool, %.0f Hz)\n"
        "     래치 : 확정 %d프레임 / 최소물림 %.1fs / 해제 %.1fs 연속\n"
        "     페달 구동은 nxde arduino 가 맡는다 (이 런치는 직접 PWM 16~255). "
        "이 노드는 속도를 판정하지 않는다\n"
        "     ★이 노드는 /cmd_vel_raw·/control_state·/brake_level 을 발행하지 "
        "않는다★ — 구동차단과 리니어는 nxde arduino 가 한다",
        stop_signal_topic_.c_str(), aeb_stop_topic_.c_str(),
        1.0 / std::max(0.01, publish_period_s_),
        engage_frames_, min_engage_s_, release_clear_s_);
  }

private:
  double nowSec() const { return this->now().seconds(); }

  /// /encoder 중앙값 3 → m/s. 표본이 안 찼거나 ★끊긴 지 오래면 0(모른다)★.
  /// 0 을 돌려주는 쪽이 안전한 방향이다 — 경고를 못 내는 것이지 제동을 거는
  /// 것이 아니다. 끊긴 사실 자체는 encoderFresh() 를 보는 쪽이 경고한다.
  double speedMs() const {
    if (!encoderFresh()) return 0.0;
    const int a = enc_buf_[0], b = enc_buf_[1], c = enc_buf_[2];
    const int med = std::max(std::min(a, b), std::min(std::max(a, b), c));
    return std::max(0, med) * ENC_SUM_TO_MS;
  }

  bool encoderFresh() const {
    if (!has_encoder_) return false;
    if (enc_stale_s_ <= 0.0) return true;          // 감시 끔
    return (nowSec() - last_enc_t_) <= enc_stale_s_;
  }

  void tick() {
    const double now = nowSec();

    // ── 판정의 신선도 ── 끊기면 '장애물 없음' 으로 본다 (fail-open, 헤더 참고)
    const bool fresh =
        signal_seen_ && (now - last_signal_t_) <= signal_stale_s_;
    if (!fresh) {
      if (signal_seen_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "⚠️ 정지신호가 %.1f초 끊겼다 (%s) — ★지금은 AEB 없는 수동조종★ 이다. "
            "cone_lidar_node 와 /ouster/points 를 확인할 것",
            now - last_signal_t_, stop_signal_topic_.c_str());
      } else {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "⏳ 정지신호를 아직 한 번도 못 받았다 (%s) — 라이다·cone_lidar_node 가 "
            "뜨기를 기다린다. ★그동안 AEB 는 없다★",
            stop_signal_topic_.c_str());
      }
    }

    const bool hit = fresh && signal_;
    if (hit) {
      if (hit_streak_ < engage_frames_) hit_streak_++;
      last_hit_t_ = now;
    } else {
      hit_streak_ = 0;
    }

    if (!engaged_) {
      if (hit_streak_ >= engage_frames_) {
        engaged_ = true;
        engaged_t_ = now;
        RCLCPP_WARN(get_logger(),
                    "🛑 ★AEB 정지★ 전방 %.2f m / 속도 %.2f m/s — 구동을 끊고 "
                    "리니어를 물린다 (해제는 전방이 %.1f초 이상 비워진 뒤)",
                    dist_m_, speedMs(), release_clear_s_);
      }
    } else {
      const bool held_long_enough = (now - engaged_t_) >= min_engage_s_;
      const bool clear_long_enough =
          !hit && (now - last_hit_t_) >= release_clear_s_;
      if (held_long_enough && clear_long_enough) {
        engaged_ = false;
        RCLCPP_INFO(get_logger(),
                    "🟢 AEB 해제 — 전방이 %.1f초간 비었다 (%.1f초 물고 있었다). "
                    "페달로 다시 가면 된다",
                    now - last_hit_t_, now - engaged_t_);
      }
    }

    std_msgs::msg::Bool msg;
    msg.data = engaged_;
    aeb_pub_->publish(msg);

    logEncoderStale();
    logHeartbeat(now);
  }

  /// /encoder 가 끊긴 사실만 알린다. ★속도를 판정하지 않는다★ — 그래도 이것은
  /// 남겨야 한다: 로그의 속도 숫자가 '0.00' 으로 보이는 이유가 '안 움직인다' 인지
  /// '못 읽는다' 인지 구별되어야 하기 때문이다.
  void logEncoderStale() {
    if (!use_encoder_ || !has_encoder_ || encoderFresh()) return;
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "⚠️ /encoder 가 %.1f초 넘게 끊겼다 — 로그의 속도 표시가 멈춘다 "
        "(AEB 는 그대로 돈다 / 속도는 애초에 판정에 쓰지 않는다). "
        "arduino 와 A보드 연결을 확인할 것",
        enc_stale_s_);
  }

  /// ★상태를 주기적으로 남긴다★ 시험 중에 "지금 무엇을 보고 있나" 를 로그만
  /// 보고 알 수 있어야 한다 (echo 를 세 개 띄우지 않도록).
  void logHeartbeat(double now) {
    if ((now - last_log_t_) < log_period_s_) return;
    last_log_t_ = now;
    const char* mode = !has_mode_ ? "모드미수신(arduino?)"
                                  : (auto_mode_ ? "자율주행" : "수동조종");
    const double v = speedMs();
    // ★속도를 '모르는' 상태를 0 으로 찍지 않는다★ 얼어붙은 값도, 그럴듯한 0 도
    //   둘 다 로그를 읽는 사람을 속인다.
    const char* enc = (!use_encoder_)      ? " / 엔코더 미사용"
                      : (!has_encoder_)    ? " / 엔코더 대기"
                      : (!encoderFresh())  ? " / ★엔코더 끊김★"
                                           : "";
    if (engaged_) {
      RCLCPP_WARN(get_logger(),
                  "🛑 AEB 유지 %.1fs — 전방 %.2f m / %.2f m/s / 비운 시간 "
                  "%.1f/%.1fs / %s%s%s",
                  now - engaged_t_, dist_m_, v, now - last_hit_t_,
                  release_clear_s_, mode, enc, estop_ ? " / ★E-STOP★" : "");
    } else {
      RCLCPP_INFO(get_logger(),
                  "감시 중 — 전방 %.2f m / %.2f m/s (%.1f km/h) / %s%s%s",
                  dist_m_, v, v * 3.6, mode, enc,
                  estop_ ? " / ★E-STOP★" : "");
    }
  }

  // ── 파라미터 ──
  std::string stop_signal_topic_;
  std::string obstacle_distance_topic_;
  std::string aeb_stop_topic_;
  double publish_period_s_ = 0.05;
  int    engage_frames_ = 1;
  double min_engage_s_ = 1.0;
  double release_clear_s_ = 1.5;
  double signal_stale_s_ = 1.0;
  double log_period_s_ = 1.0;
  bool   use_encoder_ = true;
  double enc_stale_s_ = 1.0;

  // ── 상태 ──
  bool   signal_ = false;
  bool   signal_seen_ = false;
  double last_signal_t_ = 0.0;
  float  dist_m_ = std::numeric_limits<float>::infinity();
  bool   engaged_ = false;
  double engaged_t_ = 0.0;
  double last_hit_t_ = 0.0;
  int    hit_streak_ = 0;
  double last_log_t_ = 0.0;
  bool   auto_mode_ = false;
  bool   has_mode_ = false;
  bool   estop_ = false;
  std::array<int, 3> enc_buf_{{0, 0, 0}};
  size_t enc_idx_ = 0;
  bool   has_encoder_ = false;
  double last_enc_t_ = 0.0;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr aeb_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr dist_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr encoder_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PedalDriveNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
