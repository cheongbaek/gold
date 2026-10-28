// ============================================================================
// cone_lidar_node.cpp — 가상 범퍼 AEB (전방 최근접 거리 + 정지신호)
//
//   ★출처★ catkin_ws/src/e_stop/src/cone_lidar_node.cpp (1/5카용) 를 금색차
//   kasa 로 이식한 것이다. ★이 노드는 차를 움직이지 않는다★ — 판단만 내고
//   실제 제동은 이것을 구독하는 주행 노드가 한다. 이식에서 바뀐 것은
//   ★기하★ 이다(장착 높이 0.80 → 1.17 m, 차량 앞끝 1.2 m, OS1-32 하단각).
//   단위·부호 계약(펄스/pot/제동)은 액추에이터를 만지는 노드들의 몫이고
//   lidar/kasa_units.hpp 가 소유한다.
//
//   ★[2026-10-28 방안 1 개편] 고속(30 km/h) 종단 라바콘 줄 정밀 검출★
//     1) 전방 노면 링 하위 15% 백분위 직선 피팅(Dynamic Ground Leveling):
//        가속 시 후륜 스쿼트(Nose-Up) 및 노면 경사 변화를 매 프레임 추적하여
//        수평 복원 → roi_agl_min 을 0.14 m 까지 낮춰 콘 하단 두꺼운 몸통 포착.
//     2) 종방향 연속 벽면 사전 제거(max_wall_x_extent):
//        차량 요동 시 시야에 들어오는 콘크리트 방호벽·펜스 등 긴 벽면 배제.
//     3) 횡방향 종단 라바콘 줄 기하 판별(Transverse Cone-Row Matching) +
//        초근거리 중앙 비상/점검 폴백(Close-Range Fallback):
//        양옆 차선 라바콘 오인식(0%)을 유지하면서 15~20 m 종단 콘 줄 조기 감지.
//
// 발행:
//   ~/stop_signal          (std_msgs/Bool)    — ROI 내 유효 장애물
//   ~/obstacle_distance    (std_msgs/Float32) — 최근접 전방 거리 [m]
//                                               (미검출 시 +inf)
//   ~/roi_cloud            (PointCloud2)
//   ~/debug_markers        (MarkerArray)
//
// ⚠️ ★frame_id 를 "os_sensor" 로 덮어쓴다★ (cloudCallback 첫 줄) — 드라이버
//   기본값은 point_cloud_frame=os_lidar 이고, 두 프레임은 Ouster 규약상 z축
//   180° 차이다. 이 노드는 TF 를 쓰지 않고 flip_lidar_xy 로 직접 뒤집으므로
//   덮어쓴 이름은 ★RViz 표시용★ 일 뿐이다. 장착 방향이 확정되면 flip_lidar_xy
//   와 드라이버의 point_cloud_frame 중 ★한쪽만★ 쓰도록 정리할 것.
//   (두 번 뒤집으면 조용히 앞뒤가 바뀐다)
// ============================================================================

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class ConeLidarNode : public rclcpp::Node {
public:
  ConeLidarNode() : Node("cone_lidar_node") {
    this->declare_parameter<std::string>("lidar_topic", "/ouster/points");
    this->declare_parameter<bool>("flip_lidar_xy", true);

    // ══════════════════════════════════════════════════════════════════
    //  ★★ 감지 박스 — 자작차 실장 기하로 다시 잡았다 (2026-08-25) ★★
    // ══════════════════════════════════════════════════════════════════
    //  실측 : 라이다 빔 원점 지상 ★1.17 m★ / 차량 앞끝이 라이다 ★앞 1.2 m★
    //  목표 : 운전석·전방 탑승자를 장애물로 오탐하지 않으면서,
    //         라이다로부터 ★2 m 이상★ 떨어진 70 cm 라바콘을 검출
    //
    //  ┌ OS1-32 실측 빔각(메타데이터 992210000037, 하단 −21.38°) ────────┐
    //  │  32빔, 평균 1.355° 간격, FOV +20.63° … −21.38°                  │
    //  │                                                                   │
    //  │  ① 지면 링이 처음 보이는 거리 = H/tan21.38° = ★2.99 m★           │
    //  │     → 2 m 근처에는 지면 점이 ★아예 없다★. 근거리 헛 AEB 는        │
    //  │       지면 때문이 아니라 차체·탑승자 때문이다.                    │
    //  │                                                                   │
    //  │  ② 라바콘이 FOV 밖으로 사라지는 거리                              │
    //  │     = (H−0.70)/tan21.38° = ★1.20 m★                              │
    //  │     → 그 안쪽은 70cm 콘이 ★원리적으로 안 보인다★(사각지대).       │
    //  │       roi_x_min 을 아무리 낮춰도 소용없다 — 물리다.               │
    //  │                                                                   │
    //  │  ③ 차체/탑승자가 슬랩에 들어오는가                                │
    //  │     운전석 x≲1.0 m : 최하단 빔 AGL ≥ 0.78 m → 슬랩(0.75) 밖.     │
    //  │     차량 앞끝 x=1.2 m : 최하단 빔 AGL = 0.70 m, 슬랩에 2빔.       │
    //  │     x=1.5 m : AGL 0.58 m, 슬랩 5빔 — 보닛·앞사람이 잡힌다.        │
    //  │     → ★roi_x_min = 2.0 m★ 가 탑승자/차체 오탐을 끊는 하한이다.   │
    //  │                                                                   │
    //  │  ④ 2.0 m 에서 70cm 콘 (슬랩 AGL 0.25–0.75)                       │
    //  │     최하단 빔 AGL 0.387 m. 콘을 찌르는 빔 7개                     │
    //  │     (−13.68° … −21.38°). 테이퍼 콘 ≈ 77점.                       │
    //  └───────────────────────────────────────────────────────────────────┘
    this->declare_parameter<double>("roi_x_min", 2.0);
    // 감지 박스 전방 끝. AGL[0.25,0.75] 슬랩에서 거리별 라바콘 점수
    // (OS1-32 실측 빔각, 700 mm 테이퍼 콘 base r=0.19 / top r=0.03):
    //     2m 77점 / 3m 53 / 4m 31 / 5m 19 / 6m 14 / 8m 6
    //   min_point_count=5 기준 유효 검출한계가 ≈6.5 m 라 6.0 으로 잡았다.
    //   금색차 정지거리(구동차단): 2펄스 1단 1.2m·2단 0.7m / 4펄스 1단 4.8m·2단 2.8m
    //   → 콘까지 6.0 m(범퍼 기준 4.8 m)면 4펄스 1단으로도 겨우 서고 2단은 여유롭다.
    // 전방 감지 상한 [m]. ★런치가 cone_lidar.yaml 을 넘기면 여기 기본값은 무시된다★
    //   실내에서 줄이려면 YAML 의 stop_distance_threshold 를 고쳐라 (재빌드 불필요).
    this->declare_parameter<double>("stop_distance_threshold", 6.0);

    // ★차량 앞끝까지의 거리 [m]★ 라이다 원점에서 앞범퍼까지. 이 앞은 전부 차체다.
    //   roi_x_min 이 이 값보다 작으면 ★자기 차를 장애물로 본다★ — 아래에서 강제로
    //   끌어올리고 경고한다(설정 실수를 조용히 넘기지 않는다).
    this->declare_parameter<double>("vehicle_front_m", 1.2);
    // 센서 FOV 하단 [deg]. 기동 로그의 사각지대·지면 링 계산에만 쓴다(검출엔 무관).
    //   OS1-32 SN 992210000037 beam_altitude_angles 마지막 채널.
    this->declare_parameter<double>("fov_bottom_deg", -21.38);
    // 검출 대상 라바콘 높이 [m]. 위와 같이 진단용.
    this->declare_parameter<double>("cone_height_m", 0.70);

    // ── ★z 슬랩을 '지상 높이(AGL)'로 선언한다 [금색차 이식]★ ────────────────
    //  종전에는 센서 기준 z 를 직접 상수로 박아 두고 주석에 "Sensor 0.80 m AGL"
    //  이라고만 적어 두었다. 차가 바뀌어 장착 높이가 달라지면 ★주석은 남고 값만
    //  틀리는★ 형태로 조용히 깨진다(실제로 이 이식에서 그럴 뻔했다).
    //  → 사람이 재는 값(sensor_height_m)과 물리적으로 뜻이 있는 값(AGL 범위)을
    //    따로 두고, 센서 기준 z 는 여기서 유도한다.
    //
    //  자작차 : 라이다 빔 원점 ★지상 1.17 m★ (2026-08-25 실측)
    //           → 지면은 센서 기준 z = −1.17
    //  ⚠️ 재는 기준에 주의 : 점군은 os_lidar 원점 기준이고, 그 원점은 os_sensor
    //     (마운트 바닥면)보다 ★36.18 mm 위★ 다(메타데이터 lidar_to_sensor_transform).
    //     바닥면까지 쟀다면 여기에 +0.036 을 더할 것.
    //
    //  ★슬랩 하한 0.25 를 고른 이유★ 지면(AGL 0)까지 0.25 m 여유이고, 이것이
    //    곧 ★피치 허용각★ 이다 — 거리 r 에서 atan(0.25/r):
    //        3 m 4.8°  /  4 m 3.6°  /  5 m 2.9°  /  6 m 2.4°
    //    0.15 로 낮추면 8 m 에서 1.1° 밖에 안 남아 제동 피치에 지면이 들어온다.
    //    ★2 m 에서는 어느 값이든 결과가 같다★ — 거기서 콘에 닿는 최하단 빔이
    //    이미 AGL 0.387 이라 0.25 든 0.35 든 걸리는 점이 없다.
    //  ★상한 0.75★ 2 m 에서 콘 꼭대기에 닿는 빔이 AGL 0.683 이다. 0.70 로 자르면
    //    측정오차·노면 요철에 그 빔이 잘려 나간다 — 콘 높이 + 5 cm 여유.
    //    운전석 머리/헬멧(≳0.9 m AGL)은 이 상한 위에 있다.
    this->declare_parameter<double>("sensor_height_m", 1.17);
    this->declare_parameter<double>("roi_agl_min", 0.14);
    this->declare_parameter<double>("roi_agl_max", 0.78);
    // ★센서 상하 각도 보정 [deg]★ 전방 숙임 = 양수(+), 전방 들림 = 음수(-)
    //   센서가 바닥을 향해 숙여져 먼 거리 바닥(6~10m)을 장애물로 오탐할 때
    //   이 값을 1.5 ~ 2.5 등으로 주면 점군이 수평 지면으로 회전 복원된다.
    this->declare_parameter<double>("pitch_angle_deg", 0.0);

    // ⚠️ ★차폭 미실측★ — 이 값은 ★차 반폭 + 여유★ 여야 한다.
    //   좁으면 차가 칠 콘을 못 보고, 넓으면 갓길 풀에 헛 AEB 가 뜬다.
    //   1톤급이면 전폭 1.5~1.6 m → 0.80 안팎이 맞다. 재서 갱신할 것.
    this->declare_parameter<double>("path_corridor_half_width", 0.8);
    this->declare_parameter<double>("corridor_center_y", 0.0);
    // 실측 빔각 계산 기준 거리별 콘 점수 : 2m 77 / 3m 53 / 5m 19 / 6m 14 / 8m 6
    //   7 로 두면 유효 검출한계가 ≈5.5 m 로 짧아진다. 5 면 ≈6.5 m.
    this->declare_parameter<int>("min_point_count", 5);
    // 연속 프레임 확인 (지면 깜빡임 오탐 완화)
    this->declare_parameter<int>("confirm_frames", 1);

    // ── ★[방안 1] 동적 지면 수평 보정 & 횡방향 종단 라바콘 줄 판별 파라미터★ ──
    this->declare_parameter<bool>("enable_dynamic_ground", true);
    this->declare_parameter<double>("ground_fit_x_min", 2.5);
    this->declare_parameter<double>("ground_fit_x_max", 16.0);
    this->declare_parameter<double>("ground_fit_half_width", 1.5);
    this->declare_parameter<double>("ground_ema_alpha", 0.35);
    this->declare_parameter<double>("max_dynamic_pitch_deg", 4.0);

    this->declare_parameter<bool>("enable_transverse_row", true);
    this->declare_parameter<double>("observe_half_width", 2.6);
    this->declare_parameter<double>("max_wall_x_extent", 0.95);
    this->declare_parameter<int>("min_row_cones", 3);
    this->declare_parameter<double>("min_cone_y_sep", 0.50);
    this->declare_parameter<double>("min_row_y_span", 1.15);
    this->declare_parameter<double>("max_row_y_gap", 1.65);
    this->declare_parameter<double>("max_row_slope", 0.22);
    this->declare_parameter<double>("row_line_tol", 0.26);
    this->declare_parameter<double>("straddle_margin", 0.45);

    // 초근거리 중앙 비상/점검 폴백 (정지 상태 3 m 앞 사람 점검 + 2 m 사각지대 진입 추적)
    this->declare_parameter<bool>("close_fallback_enable", true);
    this->declare_parameter<double>("close_fallback_max_x", 5.0);
    this->declare_parameter<double>("close_fallback_half_width", 0.50);
    this->declare_parameter<int>("close_fallback_min_pts", 4);
    this->declare_parameter<double>("close_fallback_agl_min", 0.18);

    // 기존 YAML 호환용 선언 (cone_lidar_txa1.yaml 등 레거시 파일 로드 시 경고 방지)
    this->declare_parameter<double>("cluster_cell_size", 0.30);
    this->declare_parameter<int>("min_cluster_points", 1);
    this->declare_parameter<double>("max_cluster_extent", 0.8);
    this->declare_parameter<double>("max_row_x_span", 1.0);
    this->declare_parameter<double>("row_x_gate", 0.75);
    this->declare_parameter<double>("transverse_aspect", 1.2);

    // ══════════════════════════════════════════════════════════════════
    //  ★★ 사각지대 래치 — roi_x_min 을 올린 대가를 갚는 장치 ★★
    // ══════════════════════════════════════════════════════════════════
    //  위 ② 에서 계산했듯 70cm 라바콘은 ★1.20 m 안쪽에서 원리적으로 안 보인다★.
    //  거기에 roi_x_min=2.0 이 더해지면, 다가오던 콘이 2 m 를 지나는 순간
    //  ROI 에서 사라진다 → 종전 로직은 hit_streak_ 를 0 으로 떨어뜨리고
    //  stop_signal 을 ★false 로 바꾼다★ → 주행 노드가 브레이크를 풀고 그대로
    //  콘을 친다. ★"안 보인다"를 "없다"로 읽는 것이 이 구조의 유일한 구멍이다.★
    //
    //  → 사각지대 문턱 안에서 마지막으로 본 장애물이 있으면, 사라져도 정지신호를
    //    유지한다. 무한 래치는 차를 영영 못 움직이게 하므로 시간 상한을 둔다.
    //  ※ 근본 해법은 주행 노드가 '비었다'를 ★적극적으로 확인★ 한 뒤에만 재출발
    //    하는 것이다(속도를 아는 쪽이 그쪽이다). 여기 것은 최소 방어다.
    this->declare_parameter<bool>("blind_zone_latch", true);
    // 래치 유지 시간 [s]. 2펄스(1.77 m/s)면 3초가 5.3 m — 1단 정지거리 1.2 m 의
    //   네 배다. 이 시간이 지나도 안 보이면 지나갔거나 치웠다고 본다.
    this->declare_parameter<double>("blind_latch_hold_s", 3.0);
    // 마지막 확정 거리가 roi_x_min + 이 값 이내일 때만 래치한다.
    //   멀리서 한 프레임 놓친 것을 "사각지대에 들어갔다"로 읽지 않기 위함.
    this->declare_parameter<double>("blind_latch_near_m", 0.5);

    this->declare_parameter<bool>("publish_debug", true);

    std::string lidar_topic = this->get_parameter("lidar_topic").as_string();
    this->get_parameter("flip_lidar_xy", flip_lidar_xy_);
    this->get_parameter("stop_distance_threshold", stop_distance_threshold_);
    this->get_parameter("roi_x_min", roi_x_min_);
    this->get_parameter("sensor_height_m", sensor_height_m_);
    this->get_parameter("roi_agl_min", roi_agl_min_);
    this->get_parameter("roi_agl_max", roi_agl_max_);
    this->get_parameter("pitch_angle_deg", pitch_angle_deg_);
    updatePitchTrig();
    recomputeZSlab();
    resetDynamicGroundState();

    this->get_parameter("path_corridor_half_width", corridor_half_width_);
    this->get_parameter("corridor_center_y", corridor_center_y_);
    this->get_parameter("min_point_count", min_point_count_);
    this->get_parameter("confirm_frames", confirm_frames_);
    this->get_parameter("publish_debug", publish_debug_);
    this->get_parameter("vehicle_front_m", vehicle_front_m_);
    this->get_parameter("fov_bottom_deg", fov_bottom_deg_);
    this->get_parameter("cone_height_m", cone_height_m_);
    this->get_parameter("blind_zone_latch", blind_zone_latch_);
    this->get_parameter("blind_latch_hold_s", blind_latch_hold_s_);
    this->get_parameter("blind_latch_near_m", blind_latch_near_m_);

    this->get_parameter("enable_dynamic_ground", enable_dynamic_ground_);
    this->get_parameter("ground_fit_x_min", ground_fit_x_min_);
    this->get_parameter("ground_fit_x_max", ground_fit_x_max_);
    this->get_parameter("ground_fit_half_width", ground_fit_half_width_);
    this->get_parameter("ground_ema_alpha", ground_ema_alpha_);
    this->get_parameter("max_dynamic_pitch_deg", max_dynamic_pitch_deg_);

    this->get_parameter("enable_transverse_row", enable_transverse_row_);
    this->get_parameter("observe_half_width", observe_half_width_);
    this->get_parameter("max_wall_x_extent", max_wall_x_extent_);
    this->get_parameter("min_row_cones", min_row_cones_);
    this->get_parameter("min_cone_y_sep", min_cone_y_sep_);
    this->get_parameter("min_row_y_span", min_row_y_span_);
    this->get_parameter("max_row_y_gap", max_row_y_gap_);
    this->get_parameter("max_row_slope", max_row_slope_);
    this->get_parameter("row_line_tol", row_line_tol_);
    this->get_parameter("straddle_margin", straddle_margin_);
    this->get_parameter("row_x_gate", row_x_gate_);
    this->get_parameter("cluster_cell_size", cluster_cell_size_);

    this->get_parameter("close_fallback_enable", close_fallback_enable_);
    this->get_parameter("close_fallback_max_x", close_fallback_max_x_);
    this->get_parameter("close_fallback_half_width", close_fallback_half_width_);
    this->get_parameter("close_fallback_min_pts", close_fallback_min_pts_);
    this->get_parameter("close_fallback_agl_min", close_fallback_agl_min_);

    // ★설정 실수를 조용히 넘기지 않는다★ roi_x_min 이 차량 앞끝보다 앞이면
    //   자기 차체가 ROI 에 들어온다. 경고만 하고 넘기면 "왜 계속 AEB 가 뜨지"
    //   로 며칠을 쓰게 된다 — 여기서 강제로 끌어올린다.
    clampRoiToVehicleFront(/*warn=*/true);
    logGeometry();

    stop_signal_pub_ = this->create_publisher<std_msgs::msg::Bool>("~/stop_signal", 5);
    obstacle_distance_pub_ =
        this->create_publisher<std_msgs::msg::Float32>("~/obstacle_distance", 5);

    if (publish_debug_) {
      roi_cloud_pub_ =
          this->create_publisher<sensor_msgs::msg::PointCloud2>("~/roi_cloud", 5);
      marker_pub_ =
          this->create_publisher<visualization_msgs::msg::MarkerArray>(
              "~/debug_markers", 5);
    }

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic, rclcpp::SensorDataQoS(),
        std::bind(&ConeLidarNode::cloudCallback, this, std::placeholders::_1));

    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&ConeLidarNode::onParamChange, this, std::placeholders::_1));

    RCLCPP_INFO(
        this->get_logger(),
        "cone_lidar AEB: topic=%s range=%.1f..%.1fm AGL=[%.2f,%.2f]m (센서 %.2fm) "
        "dyn_gnd=%d trans_row=%d(obs_w=%.2fm, min_cones=%d, span>=%.2fm) "
        "close_fb=%d(<=%.1fm) confirm=%d flip_xy=%d",
        lidar_topic.c_str(), roi_x_min_, stop_distance_threshold_,
        roi_agl_min_, roi_agl_max_, sensor_height_m_,
        enable_dynamic_ground_ ? 1 : 0, enable_transverse_row_ ? 1 : 0,
        observe_half_width_, min_row_cones_, min_row_y_span_,
        close_fallback_enable_ ? 1 : 0, close_fallback_max_x_,
        confirm_frames_, flip_lidar_xy_ ? 1 : 0);
  }

private:
  struct Pt2D {
    float x;
    float y;
    float z;
  };

  struct VoxelNode {
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float min_x = std::numeric_limits<float>::max();
    int count = 0;
  };

  struct RowCentroid {
    float x = 0.0f;
    float y = 0.0f;
    float min_x = 0.0f;
  };

  static int64_t packCell(int gx, int gy) {
    return (static_cast<int64_t>(gx) << 32) |
           static_cast<uint32_t>(gy);
  }

  /// 기동 시 ★이 장착에서 물리적으로 무엇이 가능한가★ 를 찍는다.
  /// 파라미터만 보고는 "왜 2 m 앞 콘이 안 잡히지"의 답이 안 나온다 — 답은
  /// 전부 FOV 하단각과 장착 높이의 기하이고, 그것을 여기서 계산해 보여 준다.
  void logGeometry() {
    const double t = std::tan(std::fabs(fov_bottom_deg_) * M_PI / 180.0);
    const double ground_r = (t > 1e-6) ? sensor_height_m_ / t : INFINITY;
    const double cone_gone_r =
        (t > 1e-6) ? std::max(0.0, sensor_height_m_ - cone_height_m_) / t : INFINITY;
    auto agl_at = [this, t](double x) {
      return sensor_height_m_ - x * t;
    };

    RCLCPP_INFO(this->get_logger(),
                "── 장착 기하 (H=%.2fm, FOV하단 %.2f°, 콘 %.2fm) ──",
                sensor_height_m_, fov_bottom_deg_, cone_height_m_);
    RCLCPP_INFO(this->get_logger(),
                "   지면 링 최근접 %.2f m  → 그 안쪽엔 지면 점이 없다",
                ground_r);
    RCLCPP_INFO(this->get_logger(),
                "   ★사각지대 %.2f m★ → %.2fm 콘이 FOV 밖으로 사라진다 "
                "(roi_x_min 을 낮춰도 안 보인다)", cone_gone_r, cone_height_m_);
    RCLCPP_INFO(this->get_logger(),
                "   차량 앞끝 %.2f m / 감지 %.2f~%.2f m "
                "(범퍼 기준 %.2f~%.2f m)",
                vehicle_front_m_, roi_x_min_, stop_distance_threshold_,
                roi_x_min_ - vehicle_front_m_,
                stop_distance_threshold_ - vehicle_front_m_);
    if (has_pitch_) {
      RCLCPP_INFO(this->get_logger(),
                  "   ★기본 피치 보정 : %.2f° (전방 %s)★",
                  std::fabs(pitch_angle_deg_),
                  (pitch_angle_deg_ > 0 ? "숙임/Down" : "들림/Up"));
    }
    if (enable_dynamic_ground_) {
      RCLCPP_INFO(this->get_logger(),
                  "   ★동적 노면 피치·높이 추적 ON★ (x=%.1f~%.1fm, 허용 ±%.1f°)",
                  ground_fit_x_min_, ground_fit_x_max_, max_dynamic_pitch_deg_);
    }
    if (enable_transverse_row_) {
      RCLCPP_INFO(this->get_logger(),
                  "   ★횡방향 종단 콘 줄 판별 ON★ (관측반폭 ±%.2fm, 최소 %d콘, 폭≥%.2fm, 벽제거 x≤%.2fm)",
                  observe_half_width_, min_row_cones_, min_row_y_span_, max_wall_x_extent_);
    }
    RCLCPP_INFO(this->get_logger(),
                "   최하단 빔 AGL : 앞끝 %.2fm→%.2fm / 감지시작 %.2fm→%.2fm",
                vehicle_front_m_, agl_at(vehicle_front_m_),
                roi_x_min_, agl_at(roi_x_min_));
    if (!enable_dynamic_ground_ && stop_distance_threshold_ > ground_r) {
      const double pitch_deg =
          std::atan(roi_agl_min_ / stop_distance_threshold_) * 180.0 / M_PI;
      RCLCPP_INFO(this->get_logger(),
                  "   지면 여유 %.2f m → 최원거리 %.1f m 에서 피치 %.1f° 까지 견딘다",
                  roi_agl_min_, stop_distance_threshold_, pitch_deg);
      if (pitch_deg < 1.5) {
        RCLCPP_WARN(this->get_logger(),
                    "   ⚠️ 피치 여유 %.1f° 는 제동 시 노즈다이브보다 작습니다 — "
                    "enable_dynamic_ground 를 켜거나 roi_agl_min 을 올리세요",
                    pitch_deg);
      }
    }
    if (blind_zone_latch_) {
      RCLCPP_INFO(this->get_logger(),
                  "   사각지대 래치 ON (문턱+%.2fm, 최대 %.1fs 유지)",
                  blind_latch_near_m_, blind_latch_hold_s_);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "   ⚠️ 사각지대 래치 OFF — 콘이 %.2f m 안으로 들어가면 "
                  "정지신호가 풀립니다", roi_x_min_);
    }
  }

  /// 지상높이(AGL) → 센서 기준 z. ★두 값의 소유자는 AGL 쪽이다★
  ///   z = AGL - 센서높이   (z-up, 센서 원점이 z=0)
  void recomputeZSlab() {
    roi_z_min_ = roi_agl_min_ - sensor_height_m_;
    roi_z_max_ = roi_agl_max_ - sensor_height_m_;
  }

  void updatePitchTrig() {
    const double rad = pitch_angle_deg_ * M_PI / 180.0;
    cos_pitch_ = static_cast<float>(std::cos(rad));
    sin_pitch_ = static_cast<float>(std::sin(rad));
    has_pitch_ = std::fabs(pitch_angle_deg_) > 1e-4;
  }

  void resetDynamicGroundState() {
    dyn_slope_ = 0.0f;
    dyn_z0_ = static_cast<float>(-sensor_height_m_);
    ground_initialized_ = false;
  }

  void clampRoiToVehicleFront(bool warn) {
    if (roi_x_min_ >= vehicle_front_m_) return;
    if (warn) {
      RCLCPP_ERROR(this->get_logger(),
                   "roi_x_min(%.2f m) 이 차량 앞끝(vehicle_front_m=%.2f m)보다 "
                   "가깝습니다 — ★자기 차체를 장애물로 봅니다★. %.2f m 로 "
                   "끌어올립니다.", roi_x_min_, vehicle_front_m_, vehicle_front_m_);
    }
    roi_x_min_ = vehicle_front_m_;
  }

  static bool fitLineOLS(const std::vector<float>& xs,
                         const std::vector<float>& zs,
                         float& out_slope,
                         float& out_z0) {
    const size_t n = xs.size();
    if (n < 3) return false;
    double sx = 0.0, sz = 0.0, sxx = 0.0, sxz = 0.0;
    for (size_t i = 0; i < n; ++i) {
      sx += xs[i];
      sz += zs[i];
      sxx += static_cast<double>(xs[i]) * xs[i];
      sxz += static_cast<double>(xs[i]) * zs[i];
    }
    const double denom = static_cast<double>(n) * sxx - sx * sx;
    if (std::fabs(denom) < 1e-6) return false;
    out_slope = static_cast<float>((static_cast<double>(n) * sxz - sx * sz) / denom);
    out_z0 = static_cast<float>((sz - out_slope * sx) / static_cast<double>(n));
    return true;
  }

  void updateDynamicGround(const std::vector<Pt2D>& base_pts) {
    constexpr int kNumBins = 12;
    const float x_min = static_cast<float>(ground_fit_x_min_);
    const float x_max = static_cast<float>(ground_fit_x_max_);
    if (x_max <= x_min + 1.0f) return;
    const float bin_w = (x_max - x_min) / static_cast<float>(kNumBins);
    const float half_w = static_cast<float>(ground_fit_half_width_);
    const float cy = static_cast<float>(corridor_center_y_);
    const float nom_z = static_cast<float>(-sensor_height_m_);

    std::array<std::vector<float>, kNumBins> bin_x;
    std::array<std::vector<float>, kNumBins> bin_z;
    for (int b = 0; b < kNumBins; ++b) {
      bin_x[b].reserve(64);
      bin_z[b].reserve(64);
    }

    for (const auto& p : base_pts) {
      if (p.x < x_min || p.x >= x_max) continue;
      if (std::fabs(p.y - cy) > half_w) continue;
      if (p.z < nom_z - 0.55f || p.z > nom_z + 0.55f) continue;
      int b = static_cast<int>((p.x - x_min) / bin_w);
      if (b < 0) b = 0;
      if (b >= kNumBins) b = kNumBins - 1;
      bin_x[b].push_back(p.x);
      bin_z[b].push_back(p.z);
    }

    std::vector<float> bx;
    std::vector<float> bz;
    bx.reserve(kNumBins);
    bz.reserve(kNumBins);

    for (int b = 0; b < kNumBins; ++b) {
      auto& xs = bin_x[b];
      auto& zs = bin_z[b];
      if (zs.size() < 12) continue;
      const size_t mid_idx = xs.size() / 2;
      std::nth_element(xs.begin(), xs.begin() + static_cast<std::ptrdiff_t>(mid_idx), xs.end());
      const size_t p15_idx = (zs.size() * 15) / 100;
      std::nth_element(zs.begin(), zs.begin() + static_cast<std::ptrdiff_t>(p15_idx), zs.end());
      bx.push_back(xs[mid_idx]);
      bz.push_back(zs[p15_idx]);
    }

    if (bx.size() < 4) return;

    const double max_slope = std::tan(max_dynamic_pitch_deg_ * M_PI / 180.0);
    int best_inliers = 0;
    float best_l1 = std::numeric_limits<float>::max();
    float cand_slope = 0.0f, cand_z0 = nom_z;

    for (size_t i = 0; i < bx.size(); ++i) {
      for (size_t j = i + 1; j < bx.size(); ++j) {
        const float dx = bx[j] - bx[i];
        if (dx < 2.5f) continue;
        const float s = (bz[j] - bz[i]) / dx;
        if (std::fabs(s) > max_slope) continue;
        const float z0_ij = bz[i] - s * bx[i];
        if (std::fabs(z0_ij - nom_z) > 0.25f) continue;

        int cnt = 0;
        float l1 = 0.0f;
        for (size_t k = 0; k < bx.size(); ++k) {
          const float err = std::fabs(bz[k] - (s * bx[k] + z0_ij));
          if (err <= 0.035f) {
            ++cnt;
            l1 += err;
          }
        }
        if (cnt > best_inliers || (cnt == best_inliers && l1 < best_l1)) {
          best_inliers = cnt;
          best_l1 = l1;
          cand_slope = s;
          cand_z0 = z0_ij;
        }
      }
    }

    if (best_inliers < 4) return;

    std::vector<float> bx_in, bz_in;
    bx_in.reserve(bx.size());
    bz_in.reserve(bz.size());
    for (size_t i = 0; i < bx.size(); ++i) {
      if (std::fabs(bz[i] - (cand_slope * bx[i] + cand_z0)) <= 0.035f) {
        bx_in.push_back(bx[i]);
        bz_in.push_back(bz[i]);
      }
    }

    float slope = cand_slope, z0 = cand_z0;
    if (!fitLineOLS(bx_in, bz_in, slope, z0)) return;
    if (std::fabs(slope) > max_slope) return;
    if (std::fabs(z0 - nom_z) > 0.25f) return;

    const float alpha = static_cast<float>(std::clamp(ground_ema_alpha_, 0.05, 1.0));
    if (!ground_initialized_) {
      dyn_slope_ = slope;
      dyn_z0_ = z0;
      ground_initialized_ = true;
    } else {
      dyn_slope_ += alpha * (slope - dyn_slope_);
      dyn_z0_ += alpha * (z0 - dyn_z0_);
    }
  }

  rcl_interfaces::msg::SetParametersResult onParamChange(
      const std::vector<rclcpp::Parameter>& params) {
    for (const auto& p : params) {
      const auto& name = p.get_name();
      if (name == "stop_distance_threshold")
        stop_distance_threshold_ = p.as_double();
      else if (name == "roi_x_min")
        roi_x_min_ = p.as_double();
      else if (name == "vehicle_front_m")
        vehicle_front_m_ = p.as_double();
      else if (name == "fov_bottom_deg")
        fov_bottom_deg_ = p.as_double();
      else if (name == "cone_height_m")
        cone_height_m_ = p.as_double();
      else if (name == "sensor_height_m") {
        sensor_height_m_ = p.as_double();
        recomputeZSlab();
        resetDynamicGroundState();
      } else if (name == "roi_agl_min") {
        roi_agl_min_ = p.as_double();
        recomputeZSlab();
      } else if (name == "roi_agl_max") {
        roi_agl_max_ = p.as_double();
        recomputeZSlab();
      } else if (name == "pitch_angle_deg") {
        pitch_angle_deg_ = p.as_double();
        updatePitchTrig();
        resetDynamicGroundState();
        RCLCPP_INFO(this->get_logger(),
                    "🔄 [AEB] pitch_angle_deg 갱신: %.2f deg (has_pitch=%d)",
                    pitch_angle_deg_, has_pitch_ ? 1 : 0);
      } else if (name == "path_corridor_half_width")
        corridor_half_width_ = p.as_double();
      else if (name == "corridor_center_y")
        corridor_center_y_ = p.as_double();
      else if (name == "min_point_count")
        min_point_count_ = static_cast<int>(p.as_int());
      else if (name == "confirm_frames")
        confirm_frames_ = static_cast<int>(p.as_int());
      else if (name == "flip_lidar_xy")
        flip_lidar_xy_ = p.as_bool();
      else if (name == "blind_zone_latch")
        blind_zone_latch_ = p.as_bool();
      else if (name == "blind_latch_hold_s")
        blind_latch_hold_s_ = p.as_double();
      else if (name == "blind_latch_near_m")
        blind_latch_near_m_ = p.as_double();
      else if (name == "enable_dynamic_ground") {
        enable_dynamic_ground_ = p.as_bool();
        resetDynamicGroundState();
      } else if (name == "ground_fit_x_min")
        ground_fit_x_min_ = p.as_double();
      else if (name == "ground_fit_x_max")
        ground_fit_x_max_ = p.as_double();
      else if (name == "ground_fit_half_width")
        ground_fit_half_width_ = p.as_double();
      else if (name == "ground_ema_alpha")
        ground_ema_alpha_ = p.as_double();
      else if (name == "max_dynamic_pitch_deg")
        max_dynamic_pitch_deg_ = p.as_double();
      else if (name == "enable_transverse_row")
        enable_transverse_row_ = p.as_bool();
      else if (name == "observe_half_width")
        observe_half_width_ = p.as_double();
      else if (name == "max_wall_x_extent")
        max_wall_x_extent_ = p.as_double();
      else if (name == "min_row_cones")
        min_row_cones_ = static_cast<int>(p.as_int());
      else if (name == "min_cone_y_sep")
        min_cone_y_sep_ = p.as_double();
      else if (name == "min_row_y_span")
        min_row_y_span_ = p.as_double();
      else if (name == "max_row_y_gap")
        max_row_y_gap_ = p.as_double();
      else if (name == "max_row_slope")
        max_row_slope_ = p.as_double();
      else if (name == "row_line_tol")
        row_line_tol_ = p.as_double();
      else if (name == "straddle_margin")
        straddle_margin_ = p.as_double();
      else if (name == "row_x_gate")
        row_x_gate_ = p.as_double();
      else if (name == "cluster_cell_size")
        cluster_cell_size_ = p.as_double();
      else if (name == "close_fallback_enable")
        close_fallback_enable_ = p.as_bool();
      else if (name == "close_fallback_max_x")
        close_fallback_max_x_ = p.as_double();
      else if (name == "close_fallback_half_width")
        close_fallback_half_width_ = p.as_double();
      else if (name == "close_fallback_min_pts")
        close_fallback_min_pts_ = static_cast<int>(p.as_int());
      else if (name == "close_fallback_agl_min")
        close_fallback_agl_min_ = p.as_double();
    }
    clampRoiToVehicleFront(/*warn=*/true);
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg) {
    std_msgs::msg::Header corrected_header = cloud_msg->header;
    corrected_header.frame_id = "os_sensor";

    auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

    // 1. 전후 반전 및 기본 정적 피치 회전 적용 (전방 섹터만 추출해 연산량 최소화)
    const float max_scan_x =
        static_cast<float>(std::max(stop_distance_threshold_ + 2.0, ground_fit_x_max_ + 1.0));
    const float max_scan_y = static_cast<float>(
        std::max({observe_half_width_, corridor_half_width_, ground_fit_half_width_}) +
        std::fabs(corridor_center_y_) + 0.5);

    std::vector<Pt2D> base_pts;
    base_pts.reserve(pcl_cloud->size() / 6);

    for (const auto& p : pcl_cloud->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        continue;

      float x = p.x, y = p.y, z = p.z;
      if (flip_lidar_xy_) {
        x = -p.x;
        y = -p.y;
      }
      if (has_pitch_) {
        const float x_corr =  x * cos_pitch_ + z * sin_pitch_;
        const float z_corr = -x * sin_pitch_ + z * cos_pitch_;
        x = x_corr;
        z = z_corr;
      }
      if (x < 1.0f || x > max_scan_x || std::fabs(y) > max_scan_y) continue;
      base_pts.push_back(Pt2D{x, y, z});
    }

    // 2. [방안 1-1] 매 프레임 전방 노면 링 하위 15% 백분위 직선 피팅 (스쿼트·경사 보정)
    float cos_dp = 1.0f, sin_dp = 0.0f;
    float gnd_z0 = static_cast<float>(-sensor_height_m_);
    if (enable_dynamic_ground_) {
      updateDynamicGround(base_pts);
      const float dp_rad = std::atan(dyn_slope_);
      cos_dp = std::cos(dp_rad);
      sin_dp = std::sin(dp_rad);
      gnd_z0 = dyn_z0_ * cos_dp;
    }

    auto bumper_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (publish_debug_) {
      bumper_cloud->reserve(256);
    }

    bool raw_hit = false;
    int points_in_bumper = 0;
    float closest = std::numeric_limits<float>::max();
    has_detected_row_line_ = false;

    const float x_min_f = static_cast<float>(roi_x_min_);
    const float x_max_f = static_cast<float>(stop_distance_threshold_);
    const float agl_min_f = static_cast<float>(roi_agl_min_);
    const float agl_max_f = static_cast<float>(roi_agl_max_);
    const float cy_f = static_cast<float>(corridor_center_y_);

    if (!enable_transverse_row_) {
      // ── 레거시 단일 고정 코리도 박스 모드 ──
      const float hw_f = static_cast<float>(corridor_half_width_);
      for (const auto& p : base_pts) {
        const float xl =  p.x * cos_dp + p.z * sin_dp;
        const float zl = -p.x * sin_dp + p.z * cos_dp;
        const float agl = zl - gnd_z0;
        if (agl < agl_min_f || agl > agl_max_f) continue;
        if (std::fabs(p.y - cy_f) > hw_f) continue;
        if (xl < x_min_f || xl > x_max_f) continue;
        ++points_in_bumper;
        if (xl < closest) closest = xl;
        if (publish_debug_) bumper_cloud->points.emplace_back(xl, p.y, zl);
      }
      raw_hit = (points_in_bumper >= min_point_count_);
    } else {
      // ── [방안 1-2] 종방향 연속 벽면 제거 + 횡방향 종단 라바콘 줄 기하 판별 ──
      const float obs_hw_f = static_cast<float>(observe_half_width_);
      const float wall_margin_x = x_max_f + 1.5f;  // 경계면 걸침 벽 길이 측정 여유

      struct CandPt {
        float x;
        float y;
        float z;
        float agl;
      };
      std::vector<CandPt> cand_pts;
      cand_pts.reserve(256);

      for (const auto& p : base_pts) {
        const float xl =  p.x * cos_dp + p.z * sin_dp;
        const float zl = -p.x * sin_dp + p.z * cos_dp;
        const float agl = zl - gnd_z0;
        if (agl < agl_min_f || agl > agl_max_f) continue;
        if (xl < x_min_f || xl > wall_margin_x) continue;
        if (std::fabs(p.y - cy_f) > obs_hw_f) continue;
        cand_pts.push_back(CandPt{xl, p.y, zl, agl});
      }

      // 0.30m 격자 8방향 연결성분으로 콘크리트 방호벽·가드레일(x_extent > max_wall_x_extent_) 제거
      const float cell_sz = static_cast<float>(std::max(0.15, cluster_cell_size_));
      std::unordered_map<int64_t, std::vector<size_t>> grid;
      std::unordered_map<int64_t, std::pair<int, int>> grid_coords;
      grid.reserve(cand_pts.size());
      grid_coords.reserve(cand_pts.size());

      for (size_t idx = 0; idx < cand_pts.size(); ++idx) {
        const int gx = static_cast<int>(std::floor(cand_pts[idx].x / cell_sz));
        const int gy = static_cast<int>(std::floor(cand_pts[idx].y / cell_sz));
        const int64_t key = packCell(gx, gy);
        grid[key].push_back(idx);
        grid_coords.emplace(key, std::make_pair(gx, gy));
      }

      std::unordered_set<int64_t> visited;
      visited.reserve(grid.size());
      std::vector<CandPt> cone_pts;
      cone_pts.reserve(cand_pts.size());

      std::vector<std::pair<int, int>> stack;
      std::vector<size_t> comp_indices;
      for (const auto& kv : grid) {
        if (visited.count(kv.first)) continue;
        visited.insert(kv.first);
        stack.clear();
        comp_indices.clear();
        stack.push_back(grid_coords[kv.first]);

        float min_cx = std::numeric_limits<float>::max();
        float max_cx = -std::numeric_limits<float>::max();

        while (!stack.empty()) {
          const auto [kx, ky] = stack.back();
          stack.pop_back();
          const int64_t cur_key = packCell(kx, ky);
          const auto it = grid.find(cur_key);
          if (it != grid.end()) {
            for (const size_t pt_i : it->second) {
              comp_indices.push_back(pt_i);
              min_cx = std::min(min_cx, cand_pts[pt_i].x);
              max_cx = std::max(max_cx, cand_pts[pt_i].x);
            }
          }
          for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
              if (dx == 0 && dy == 0) continue;
              const int nx = kx + dx;
              const int ny = ky + dy;
              const int64_t nb_key = packCell(nx, ny);
              if (grid.count(nb_key) && !visited.count(nb_key)) {
                visited.insert(nb_key);
                stack.emplace_back(nx, ny);
              }
            }
          }
        }

        if (static_cast<double>(max_cx - min_cx) <= max_wall_x_extent_) {
          for (const size_t pt_i : comp_indices) {
            if (cand_pts[pt_i].x <= x_max_f) {
              cone_pts.push_back(cand_pts[pt_i]);
            }
          }
        }
      }

      if (publish_debug_) {
        for (const auto& cp : cone_pts) {
          bumper_cloud->points.emplace_back(cp.x, cp.y, cp.z);
        }
      }

      // 0.20m 복셀 압축 후 횡방향 직선 슬라이스 탐색
      constexpr float kVoxSize = 0.20f;
      std::unordered_map<int64_t, VoxelNode> vox_map;
      vox_map.reserve(cone_pts.size());
      for (const auto& cp : cone_pts) {
        const int vx = static_cast<int>(std::floor(cp.x / kVoxSize));
        const int vy = static_cast<int>(std::floor(cp.y / kVoxSize));
        auto& v = vox_map[packCell(vx, vy)];
        v.sum_x += cp.x;
        v.sum_y += cp.y;
        v.min_x = std::min(v.min_x, cp.x);
        ++v.count;
      }

      std::vector<RowCentroid> nodes;
      nodes.reserve(vox_map.size());
      for (const auto& kv : vox_map) {
        const auto& v = kv.second;
        const float inv = 1.0f / static_cast<float>(v.count);
        nodes.push_back(RowCentroid{v.sum_x * inv, v.sum_y * inv, v.min_x});
      }
      std::sort(nodes.begin(), nodes.end(),
                [](const RowCentroid& a, const RowCentroid& b) { return a.x < b.x; });

      bool row_found = false;
      float best_row_x = std::numeric_limits<float>::max();
      int best_row_support = 0;
      const size_t n_nodes = nodes.size();
      const float x_gate_f = static_cast<float>(row_x_gate_);
      const float min_pair_dy = static_cast<float>(std::max(0.6, min_row_y_span_ - 0.05));
      const float max_slope_f = static_cast<float>(max_row_slope_);
      const float line_tol_f = static_cast<float>(row_line_tol_);
      const float min_span_f = static_cast<float>(min_row_y_span_);
      const float max_gap_f = static_cast<float>(max_row_y_gap_);
      const float straddle_f = static_cast<float>(straddle_margin_);
      const float min_sep_f = static_cast<float>(min_cone_y_sep_);

      std::vector<RowCentroid> inliers;
      inliers.reserve(n_nodes);

      for (size_t i = 0; i < n_nodes; ++i) {
        for (size_t j = i + 1; j < n_nodes; ++j) {
          if (nodes[j].x - nodes[i].x > x_gate_f) break;
          const float dy = nodes[j].y - nodes[i].y;
          if (std::fabs(dy) < min_pair_dy) continue;
          const float slope = (nodes[j].x - nodes[i].x) / dy;
          if (std::fabs(slope) > max_slope_f) continue;
          const float x0 = nodes[i].x - slope * nodes[i].y;

          inliers.clear();
          for (const auto& nd : nodes) {
            if (std::fabs(nd.x - (slope * nd.y + x0)) <= line_tol_f) {
              inliers.push_back(nd);
            }
          }
          if (static_cast<int>(inliers.size()) < min_row_cones_) continue;

          std::sort(inliers.begin(), inliers.end(),
                    [](const RowCentroid& a, const RowCentroid& b) { return a.y < b.y; });

          const float y_lo = inliers.front().y;
          const float y_hi = inliers.back().y;
          if ((y_hi - y_lo) < min_span_f) continue;
          if (y_lo > cy_f + straddle_f || y_hi < cy_f - straddle_f) continue;

          float max_gap = 0.0f;
          for (size_t k = 0; k + 1 < inliers.size(); ++k) {
            max_gap = std::max(max_gap, inliers[k + 1].y - inliers[k].y);
          }
          if (max_gap > max_gap_f) continue;

          int distinct_cnt = 1;
          float last_y = inliers.front().y;
          float row_min_x = inliers.front().min_x;
          for (size_t k = 1; k < inliers.size(); ++k) {
            row_min_x = std::min(row_min_x, inliers[k].min_x);
            if (inliers[k].y - last_y >= min_sep_f) {
              ++distinct_cnt;
              last_y = inliers[k].y;
            }
          }

          if (distinct_cnt >= min_row_cones_ && row_min_x < best_row_x) {
            row_found = true;
            best_row_x = row_min_x;
            best_row_support = distinct_cnt;
            has_detected_row_line_ = true;
            det_row_x1_ = slope * y_lo + x0;
            det_row_y1_ = y_lo;
            det_row_x2_ = slope * y_hi + x0;
            det_row_y2_ = y_hi;
          }
        }
      }

      if (row_found) {
        raw_hit = true;
        closest = best_row_x;
        points_in_bumper = best_row_support;
      }

      // 초근거리 중앙 비상/점검 폴백 (정지 중 전방 3m 점검 및 2~5m 사각지대 진입 연속 추적)
      if (close_fallback_enable_) {
        const float cf_max_x = static_cast<float>(close_fallback_max_x_);
        const float cf_hw = static_cast<float>(close_fallback_half_width_);
        const float cf_agl = static_cast<float>(close_fallback_agl_min_);
        int close_cnt = 0;
        float close_min_x = std::numeric_limits<float>::max();
        for (const auto& cp : cone_pts) {
          if (cp.x <= cf_max_x && std::fabs(cp.y - cy_f) <= cf_hw && cp.agl >= cf_agl) {
            ++close_cnt;
            close_min_x = std::min(close_min_x, cp.x);
          }
        }
        if (close_cnt >= close_fallback_min_pts_) {
          raw_hit = true;
          if (close_min_x < closest) closest = close_min_x;
          if (!row_found) points_in_bumper = close_cnt;
        }
      }
    }

    if (raw_hit)
      ++hit_streak_;
    else
      hit_streak_ = 0;

    const bool confirmed =
        hit_streak_ >= std::max(confirm_frames_, 1);
    const rclcpp::Time now = this->now();

    if (confirmed && std::isfinite(closest)) {
      last_confirmed_distance_ = closest;
      last_confirm_time_ = now;
    }

    bool latched = false;
    if (!confirmed && blind_zone_latch_ &&
        std::isfinite(last_confirmed_distance_) &&
        last_confirmed_distance_ <= roi_x_min_ + blind_latch_near_m_) {
      const double held = (now - last_confirm_time_).seconds();
      if (held >= 0.0 && held < blind_latch_hold_s_)
        latched = true;
    }

    if (!confirmed && !latched)
      last_confirmed_distance_ = std::numeric_limits<float>::infinity();

    std_msgs::msg::Float32 dist_msg;
    if (raw_hit && std::isfinite(closest)) {
      dist_msg.data = closest;
    } else if (latched && std::isfinite(last_confirmed_distance_)) {
      dist_msg.data = last_confirmed_distance_;
    } else {
      dist_msg.data = std::numeric_limits<float>::infinity();
    }
    last_obstacle_distance_ = dist_msg.data;
    obstacle_distance_pub_->publish(dist_msg);

    const bool stop = confirmed || latched;
    std_msgs::msg::Bool stop_msg;
    stop_msg.data = stop;
    stop_signal_pub_->publish(stop_msg);

    if (confirmed) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 500,
          "🚨 [AEB] obstacle d=%.2fm support/pts=%d (streak=%d, dyn_pitch=%+.2f°)",
          closest, points_in_bumper, hit_streak_,
          std::atan(dyn_slope_) * 180.0 / M_PI);
    } else if (latched) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 500,
          "🚨 [AEB] blind-zone latch d=%.2fm (hold %.1fs)",
          last_confirmed_distance_, blind_latch_hold_s_);
    }

    if (publish_debug_) {
      bumper_cloud->width = static_cast<uint32_t>(bumper_cloud->points.size());
      bumper_cloud->height = 1;
      bumper_cloud->is_dense = true;
      sensor_msgs::msg::PointCloud2 roi_msg;
      pcl::toROSMsg(*bumper_cloud, roi_msg);
      roi_msg.header = corrected_header;
      roi_cloud_pub_->publish(roi_msg);
      publishDebugMarkers(corrected_header, stop);
    }
  }

  void publishDebugMarkers(const std_msgs::msg::Header& header, bool is_stopped) {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header = header;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    const double bound_hw =
        enable_transverse_row_ ? observe_half_width_ : corridor_half_width_;

    for (int side = 0; side < 2; ++side) {
      const float y = static_cast<float>(
          corridor_center_y_ + (side == 0 ? bound_hw : -bound_hw));
      visualization_msgs::msg::Marker line;
      line.header = header;
      line.ns = "bumper_boundary";
      line.id = side;
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.scale.x = 0.05;
      if (is_stopped) {
        line.color.r = 1.0f;
        line.color.g = 0.0f;
        line.color.b = 0.0f;
        line.color.a = 1.0f;
      } else {
        line.color.r = 1.0f;
        line.color.g = 1.0f;
        line.color.b = 0.0f;
        line.color.a = 0.8f;
      }
      line.lifetime = rclcpp::Duration::from_seconds(0.2);
      line.pose.orientation.w = 1.0;
      geometry_msgs::msg::Point p1, p2;
      p1.x = roi_x_min_;
      p1.y = y;
      p1.z = 0.0;
      p2.x = stop_distance_threshold_;
      p2.y = y;
      p2.z = 0.0;
      line.points.push_back(p1);
      line.points.push_back(p2);
      marker_array.markers.push_back(line);
    }

    if (has_detected_row_line_) {
      visualization_msgs::msg::Marker row_mk;
      row_mk.header = header;
      row_mk.ns = "transverse_stopline";
      row_mk.id = 10;
      row_mk.type = visualization_msgs::msg::Marker::LINE_STRIP;
      row_mk.action = visualization_msgs::msg::Marker::ADD;
      row_mk.scale.x = 0.12;
      row_mk.color.r = 1.0f;
      row_mk.color.g = 0.1f;
      row_mk.color.b = 0.1f;
      row_mk.color.a = 1.0f;
      row_mk.lifetime = rclcpp::Duration::from_seconds(0.2);
      row_mk.pose.orientation.w = 1.0;
      geometry_msgs::msg::Point q1, q2;
      q1.x = det_row_x1_;
      q1.y = det_row_y1_;
      q1.z = 0.0;
      q2.x = det_row_x2_;
      q2.y = det_row_y2_;
      q2.z = 0.0;
      row_mk.points.push_back(q1);
      row_mk.points.push_back(q2);
      marker_array.markers.push_back(row_mk);
    }

    marker_pub_->publish(marker_array);
  }

  bool flip_lidar_xy_ = true;
  double stop_distance_threshold_ = 6.0;
  double roi_x_min_ = 2.0;
  double vehicle_front_m_ = 1.2;
  double fov_bottom_deg_ = -21.38;
  double cone_height_m_ = 0.70;
  // ★roi_z_* 는 유도값이다★ 소유자는 아래 AGL 세 값이고, recomputeZSlab() 만
  //   이 둘을 쓴다. 직접 파라미터로 열어 두지 않았다 — 두 경로가 생기면 어느
  //   쪽이 이겼는지 로그로 알 수 없게 된다.
  double roi_z_min_ = -1.03, roi_z_max_ = -0.39;
  double sensor_height_m_ = 1.17;  // 지면 → 센서 원점 [m]
  double pitch_angle_deg_ = 0.0;   // 센서 전방 숙임 각도 [deg] (+: 앞 숙임, -: 뒤 젖힘)
  float cos_pitch_ = 1.0f;
  float sin_pitch_ = 0.0f;
  bool has_pitch_ = false;
  double roi_agl_min_ = 0.14;      // 슬랩 하단 [m AGL]
  double roi_agl_max_ = 0.78;      // 슬랩 상단 [m AGL]
  double corridor_half_width_ = 0.8;
  double corridor_center_y_ = 0.0;
  int min_point_count_ = 5;
  int confirm_frames_ = 1;
  bool publish_debug_ = true;
  bool blind_zone_latch_ = true;
  double blind_latch_hold_s_ = 3.0;
  double blind_latch_near_m_ = 0.5;

  // [방안 1] 동적 노면 추적 & 횡방향 종단 콘 줄 판별 상태
  bool enable_dynamic_ground_ = true;
  double ground_fit_x_min_ = 2.5;
  double ground_fit_x_max_ = 16.0;
  double ground_fit_half_width_ = 1.5;
  double ground_ema_alpha_ = 0.35;
  double max_dynamic_pitch_deg_ = 4.0;
  float dyn_slope_ = 0.0f;
  float dyn_z0_ = -1.17f;
  bool ground_initialized_ = false;

  bool enable_transverse_row_ = true;
  double observe_half_width_ = 2.6;
  double max_wall_x_extent_ = 0.95;
  int min_row_cones_ = 3;
  double min_cone_y_sep_ = 0.50;
  double min_row_y_span_ = 1.15;
  double max_row_y_gap_ = 1.65;
  double max_row_slope_ = 0.22;
  double row_line_tol_ = 0.26;
  double straddle_margin_ = 0.45;
  double row_x_gate_ = 0.75;
  double cluster_cell_size_ = 0.30;

  bool close_fallback_enable_ = true;
  double close_fallback_max_x_ = 5.0;
  double close_fallback_half_width_ = 0.50;
  int close_fallback_min_pts_ = 4;
  double close_fallback_agl_min_ = 0.18;

  bool has_detected_row_line_ = false;
  float det_row_x1_ = 0.0f, det_row_y1_ = 0.0f;
  float det_row_x2_ = 0.0f, det_row_y2_ = 0.0f;

  int hit_streak_ = 0;
  float last_obstacle_distance_ = std::numeric_limits<float>::infinity();
  float last_confirmed_distance_ = std::numeric_limits<float>::infinity();
  rclcpp::Time last_confirm_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_signal_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr obstacle_distance_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr roi_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<ConeLidarNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("cone_lidar_node"), "fatal: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
