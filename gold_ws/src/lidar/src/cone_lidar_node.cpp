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
#include <cmath>
#include <limits>
#include <memory>
#include <string>
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
    this->declare_parameter<double>("roi_agl_min", 0.25);
    this->declare_parameter<double>("roi_agl_max", 0.75);

    // ⚠️ ★차폭 미실측★ — 이 값은 ★차 반폭 + 여유★ 여야 한다.
    //   좁으면 차가 칠 콘을 못 보고, 넓으면 갓길 풀에 헛 AEB 가 뜬다.
    //   1톤급이면 전폭 1.5~1.6 m → 0.80 안팎이 맞다. 재서 갱신할 것.
    this->declare_parameter<double>("path_corridor_half_width", 0.8);
    this->declare_parameter<double>("corridor_center_y", 0.0);
    // 실측 빔각 계산 기준 거리별 콘 점수 : 2m 77 / 3m 53 / 5m 19 / 6m 14 / 8m 6
    //   7 로 두면 유효 검출한계가 ≈5.5 m 로 짧아진다. 5 면 ≈6.5 m.
    this->declare_parameter<int>("min_point_count", 5);
    // 연속 프레임 확인 (지면 깜빡임 오탐 완화)
    this->declare_parameter<int>("confirm_frames", 2);
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
    recomputeZSlab();
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

    RCLCPP_INFO(this->get_logger(),
                "cone_lidar AEB: topic=%s range=%.1f..%.1fm "
                "AGL=[%.2f,%.2f]m (센서 %.2fm → z=[%.2f,%.2f]) "
                "half_w=%.2f min_pts=%d confirm=%d flip_xy=%d",
                lidar_topic.c_str(), roi_x_min_, stop_distance_threshold_,
                roi_agl_min_, roi_agl_max_, sensor_height_m_,
                roi_z_min_, roi_z_max_, corridor_half_width_, min_point_count_,
                confirm_frames_, flip_lidar_xy_ ? 1 : 0);
  }

private:
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
    RCLCPP_INFO(this->get_logger(),
                "   최하단 빔 AGL : 앞끝 %.2fm→%.2fm / 감지시작 %.2fm→%.2fm",
                vehicle_front_m_, agl_at(vehicle_front_m_),
                roi_x_min_, agl_at(roi_x_min_));
    // 슬랩 하한이 각 거리에서 견디는 피치각 — 지면이 슬랩에 들어오는 문턱이다
    if (stop_distance_threshold_ > ground_r) {
      const double pitch_deg =
          std::atan(roi_agl_min_ / stop_distance_threshold_) * 180.0 / M_PI;
      RCLCPP_INFO(this->get_logger(),
                  "   지면 여유 %.2f m → 최원거리 %.1f m 에서 피치 %.1f° 까지 견딘다",
                  roi_agl_min_, stop_distance_threshold_, pitch_deg);
      if (pitch_deg < 1.5) {
        RCLCPP_WARN(this->get_logger(),
                    "   ⚠️ 피치 여유 %.1f° 는 제동 시 노즈다이브보다 작습니다 — "
                    "roi_agl_min 을 올리거나 stop_distance_threshold 를 줄이세요",
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
      } else if (name == "roi_agl_min") {
        roi_agl_min_ = p.as_double();
        recomputeZSlab();
      } else if (name == "roi_agl_max") {
        roi_agl_max_ = p.as_double();
        recomputeZSlab();
      }
      else if (name == "path_corridor_half_width")
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

    auto bumper_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    bumper_cloud->reserve(pcl_cloud->size() / 8);

    int points_in_bumper = 0;
    float closest = std::numeric_limits<float>::max();

    for (const auto& p : pcl_cloud->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        continue;

      float x = p.x, y = p.y, z = p.z;
      if (flip_lidar_xy_) {
        x = -p.x;
        y = -p.y;
      }

      if (z < roi_z_min_ || z > roi_z_max_) continue;
      if (std::fabs(y - corridor_center_y_) > corridor_half_width_) continue;
      if (x < roi_x_min_ || x > stop_distance_threshold_) continue;

      ++points_in_bumper;
      if (x < closest) closest = x;
      if (publish_debug_) bumper_cloud->points.emplace_back(x, y, z);
    }

    const bool raw_hit = points_in_bumper >= min_point_count_;
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
          "🚨 [AEB] obstacle d=%.2fm pts=%d (streak=%d)", closest,
          points_in_bumper, hit_streak_);
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

    for (int side = 0; side < 2; ++side) {
      float y =
          corridor_center_y_ + (side == 0 ? corridor_half_width_ : -corridor_half_width_);
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
  double roi_z_min_ = -0.92, roi_z_max_ = -0.42;
  double sensor_height_m_ = 1.17;  // 지면 → 센서 원점 [m]
  double roi_agl_min_ = 0.25;      // 슬랩 하단 [m AGL]
  double roi_agl_max_ = 0.75;      // 슬랩 상단 [m AGL]
  double corridor_half_width_ = 0.8;
  double corridor_center_y_ = 0.0;
  int min_point_count_ = 5;
  int confirm_frames_ = 2;
  bool publish_debug_ = true;
  bool blind_zone_latch_ = true;
  double blind_latch_hold_s_ = 3.0;
  double blind_latch_near_m_ = 0.5;
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
