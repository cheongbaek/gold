// ============================================================================
// drive_lidar_node.cpp — 측량 잠금 헤딩홀드 (v6) · ★금색차 kasa 이식판★
//
// 정지 관측 → 좌/우 라바콘을 여러 스캔 누적 → 복도 직선/헤딩을 잠근 뒤
// 그 방향으로만 IMU 헤딩홀드 직진. 주행 중 콘을 다시 붙잡지 않는다
// (스캔마다 몇 개 놓치면 옆길로 새던 v5 EKF 추종의 원인).
//
// 전방 AEB 는 cone_lidar_node. 이 노드는 stop_signal / obstacle_distance 만 본다.
//
// ══════════════════════════════════════════════════════════════════════════
//  ★★ 1/5카 → 금색차에서 바뀐 것 (전부 lidar/kasa_units.hpp 가 소유) ★★
// ══════════════════════════════════════════════════════════════════════════
//  ① 출력이 `cmd_vel_pub_` 직접 발행 → `KasaActuator` 경유로 바뀌었다.
//     원본은 `linear.x = m/s`, `angular.z = 도로휠각 deg (+ = 좌)` 를 그대로
//     냈다. 금색차는 ★펄스 정수 / pot 지령 / − 좌 + 우★ 다. 그 환산·부호 반전이
//     일어나는 지점은 KasaActuator::drive() ★한 곳뿐★ 이다.
//  ② `/control_state` 를 낸다. 원본에는 이 발행자가 아예 없었고, 그래서 이
//     노드만 띄우면 ★1/5카에서도 차가 움직이지 않았다★(motor.py 의 초기값도
//     state_enable=false 다). 금색차 arduino.py 도 같다 — 없으면 A보드에 0.
//  ③ ★제동 경로를 새로 넣었다★ 원본에서 '정지'는 속도지령 0 이 전부였는데,
//     금색차에서 펄스 0 은 코스트(자연감속 0.41 m/s²)일 뿐이다. 4펄스에서
//     코스트 정지거리는 15.2 m 라 AEB 문턱 8.5 m 안에 ★설 수 없다★.
//     → 감속 구간은 `펄스 0 + 리니어 1단`, 정지 확정은 `2단`. 근거·규칙은
//       white1/BRAKING.md 와 kasa_units.hpp 3절.
//  ④ 제원 : 휠베이스 0.75 → 1.25 m / 조향 상한 0.366 → 0.553 rad(도로휠 31.7°)
//  ⑤ 게이트 : `/vehicle_mode`(D5) · `/estop` 을 본다. 원본은 둘 다 몰랐다.
//  ⑥ z 슬랩을 지상높이(AGL)로 선언한다 — 장착 1.17 m 기준.
// ============================================================================

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "lidar/kasa_units.hpp"
#include "lidar/line_ekf.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class DriveLidarNode : public rclcpp::Node {
public:
  DriveLidarNode() : Node("drive_lidar_node") {
    this->declare_parameter<std::string>("lidar_topic", "/ouster/points");
    this->declare_parameter<bool>("flip_lidar_xy", true);
    this->declare_parameter<double>("roi_x_min", 0.5);
    this->declare_parameter<double>("roi_x_max", 8.0);
    this->declare_parameter<double>("roi_y_max", 6.0);
    // ★z 슬랩은 지상높이(AGL)로 선언한다 [금색차 이식]★ 소유자는 AGL 쪽이고
    //   센서 기준 z 는 recomputeZSlab() 이 유도한다. 근거는 cone_lidar_node 주석.
    //   금색차 라이다 : ★1.17 m AGL / 차량 좌우 가운데 / 후륜차축 바로 위★
    //   0.20~1.20 m AGL 은 1/5카(센서 0.80m, z −0.6~0.4)와 같은 물리 구간이다.
    this->declare_parameter<double>("sensor_height_m", 1.17);
    this->declare_parameter<double>("roi_agl_min", 0.20);
    this->declare_parameter<double>("roi_agl_max", 1.20);

    this->declare_parameter<double>("cluster_cell_size", 0.35);
    this->declare_parameter<int>("min_cluster_points", 2);
    this->declare_parameter<double>("max_cluster_extent", 0.8);
    this->declare_parameter<double>("lane_band_min", 1.0);
    this->declare_parameter<double>("lane_band_max", 4.0);
    this->declare_parameter<double>("line_fit_residual_gate", 0.7);

    this->declare_parameter<double>("initial_half_width", 2.2);
    this->declare_parameter<double>("corridor_center_y", 0.0);

    // ★금색차 실측 1.25 m★ 0.75(1/5카)를 그대로 두면 순수추종·기하가 전부
    //   60% 로 축소된다. kasa_units.hpp 가 값의 정본이다.
    this->declare_parameter<double>("wheelbase", lidar::kasa::WHEELBASE_M);
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_raw");
    // linear_speed [m/s]. 현재 기본 2.0 m/s ≈ 7.2 km/h.
    // 런치: ros2 launch lidar drive_lidar.launch.py linear_speed:=<m/s>
    // 노드: ros2 run lidar drive_lidar_node --ros-args -p linear_speed:=<m/s>
    //   5 km/h → 1.389   10 km/h → 2.778
    //  20 km/h → 5.556   30 km/h → 8.333
    // ★기본 2펄스 = 1.768 m/s = 6.4 km/h (지시, 2026-08-25)★
    //   금색차의 속도 지령은 ★정수 펄스★ 다 — 실효 선택지가 {0,1,2,3,4} 뿐이고
    //   1펄스 = 0.884 m/s = 3.18 km/h 계단이다. 여기 m/s 로 무엇을 넣든
    //   KasaActuator 가 반올림하므로, 계단 중간값을 넣는 것은 뜻이 없다.
    //   ★4펄스는 피한다★ 정지 상태 재출발에서 A보드 적분이 동결되는 값이다
    //   (err 가 정확히 4 → PWM 92 고정, 2펄스보다 약하다. kasa_units.hpp 2절)
    this->declare_parameter<double>("linear_speed",
                                    lidar::kasa::pulseToMs(2));
    this->declare_parameter<double>("k_heading", 1.8);
    this->declare_parameter<double>("k_stanley", 0.0);
    // ★도로휠각 상한 [rad]★ 0.366(=21°, 1/5카)이 아니라 금색차의 31.7° 다.
    //   pot ±40° 는 도로휠각이 아니라 가변저항 행정 이름이며, 그 환산은
    //   KasaActuator 가 한다 — 여기서 다루는 값은 언제나 ★도로휠각★ 이다.
    this->declare_parameter<double>("max_angular_speed",
                                    lidar::kasa::STEER_ROAD_MAX_RAD);
    this->declare_parameter<double>("max_angular_step", 0.12);

    this->declare_parameter<std::string>("aeb_stop_signal_topic",
                                         "/cone_lidar_node/stop_signal");
    this->declare_parameter<std::string>("aeb_obstacle_distance_topic",
                                         "/cone_lidar_node/obstacle_distance");
    this->declare_parameter<bool>("listen_to_aeb_stop_signal", true);
    this->declare_parameter<int>("aeb_confirm_frames", 2);
    this->declare_parameter<bool>("aeb_require_own_gate", false);
    this->declare_parameter<double>("aeb_own_gate_max_range", 12.0);
    this->declare_parameter<double>("gate_stop_start_distance", 6.0);
    this->declare_parameter<double>("gate_stop_end_distance", 1.5);
    this->declare_parameter<double>("aeb_brake_start_distance", 8.5);
    this->declare_parameter<double>("aeb_brake_end_distance", 1.5);
    this->declare_parameter<double>("aeb_max_decel", 5.0);
    this->declare_parameter<bool>("aeb_use_distance_brake", true);

    this->declare_parameter<std::string>("imu_topic", "/ouster/imu");
    this->declare_parameter<bool>("use_imu_yaw_rate", true);
    this->declare_parameter<double>("imu_yaw_rate_lpf", 0.3);

    // Survey → lock
    this->declare_parameter<double>("survey_duration", 2.5);
    this->declare_parameter<double>("survey_timeout", 6.0);
    this->declare_parameter<double>("survey_roi_x_max", 16.0);
    this->declare_parameter<double>("survey_merge_dist", 0.45);
    this->declare_parameter<int>("survey_min_side_cones", 2);
    this->declare_parameter<double>("survey_min_x_span", 3.0);
    this->declare_parameter<double>("survey_max_heading_deg", 25.0);
    this->declare_parameter<bool>("lock_use_cte", false);

    // Kept so older yaml (gazebo / live-EKF era) still loads cleanly.
    this->declare_parameter<double>("side_inner_margin", 1.0);
    this->declare_parameter<int>("side_hold_frames", 5);
    this->declare_parameter<int>("no_detection_stop_frames", 15);
    this->declare_parameter<double>("half_width_min", 1.2);
    this->declare_parameter<double>("half_width_max", 3.5);
    this->declare_parameter<bool>("fixed_half_width", true);
    this->declare_parameter<bool>("require_dual_side", false);
    this->declare_parameter<double>("ekf_q_c0", 0.05);
    this->declare_parameter<double>("ekf_q_c1", 0.02);
    this->declare_parameter<double>("ekf_q_half_width", 0.01);
    this->declare_parameter<double>("ekf_r_c0", 0.15);
    this->declare_parameter<double>("ekf_r_c1", 0.05);
    this->declare_parameter<double>("ekf_r_half_width", 0.20);
    // WARNING: 금색차 윤거 미실측 — 1/5카 값(0.65)이 그대로다. 줄자로 재서 갱신할 것.
    this->declare_parameter<double>("track_width", 0.65);
    this->declare_parameter<double>("speed_reduction_on_turn", 0.0);
    this->declare_parameter<bool>("stop_on_no_detection", false);

    // ══ ★리니어 제동 [금색차 이식 신설]★ ══════════════════════════════
    //  원본에서 '정지'는 속도지령 0 이 전부였다. 금색차에서 펄스 0 은
    //  ★코스트(0.41 m/s²)★ 일 뿐이라 4펄스 정지거리가 15.2 m 다 — AEB 문턱
    //  8.5 m 안에 설 수 없다. 리니어를 물어야 실제로 선다.
    //     1단 1.30 m/s² (구동차단 실측) → 2펄스 정지거리 1.2 m / 4펄스 4.8 m
    //     2단 2.20 m/s² (실측 하한)     → 2펄스 0.7 m / 4펄스 2.8 m
    //  근거는 white1/BRAKING.md, 규칙은 kasa_units.hpp 3절.
    this->declare_parameter<bool>("brake_enable", true);
    // 체결 여유 [m] — ★모자란 쪽으로 틀리게 하는 값★ 크면 일찍 문다
    this->declare_parameter<double>("brake_margin_m", 1.0);
    // 해제 이력(hysteresis). 필요거리의 이 배를 넘게 회복해야 푼다
    this->declare_parameter<double>("brake_release_k", 1.5);
    // 엔코더 실측속도 사용. ★체결은 기하로 / 해제는 실측으로★ 의 실측 쪽이다
    this->declare_parameter<bool>("use_encoder_speed", true);

    this->declare_parameter<bool>("publish_debug", true);

    loadParamsToMembers();

    const std::string lidar_topic = this->get_parameter("lidar_topic").as_string();
    const std::string cmd_vel_topic =
        this->get_parameter("cmd_vel_topic").as_string();
    const std::string aeb_topic =
        this->get_parameter("aeb_stop_signal_topic").as_string();
    const std::string aeb_dist_topic =
        this->get_parameter("aeb_obstacle_distance_topic").as_string();
    const std::string imu_topic = this->get_parameter("imu_topic").as_string();

    // ★출력은 전부 KasaActuator 를 거친다★ 펄스 환산·pot 환산·부호 반전·
    //   /control_state·/brake_level·D5·E-STOP 게이트가 모두 그 안에 있다.
    //   (cmd_vel_topic 파라미터는 액추에이터의 kasa.cmd_vel_topic 으로 옮겼다)
    if (cmd_vel_topic != "/cmd_vel_raw" && !this->has_parameter("kasa.cmd_vel_topic")) {
      this->declare_parameter<std::string>("kasa.cmd_vel_topic", cmd_vel_topic);
    }
    actuator_ = std::make_unique<lidar::kasa::KasaActuator>(this);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("~/corridor_path", 5);
    steering_target_pub_ =
        this->create_publisher<geometry_msgs::msg::PointStamped>(
            "~/steering_target", 5);
    if (publish_debug_) {
      left_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "~/left_cone_cloud", 5);
      right_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "~/right_cone_cloud", 5);
      marker_pub_ =
          this->create_publisher<visualization_msgs::msg::MarkerArray>(
              "~/debug_markers", 5);
    }

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic, rclcpp::SensorDataQoS(),
        std::bind(&DriveLidarNode::cloudCallback, this, std::placeholders::_1));

    if (listen_to_aeb_stop_signal_) {
      aeb_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
          aeb_topic, 10,
          std::bind(&DriveLidarNode::aebStopCallback, this,
                    std::placeholders::_1));
      aeb_dist_sub_ = this->create_subscription<std_msgs::msg::Float32>(
          aeb_dist_topic, 10,
          std::bind(&DriveLidarNode::aebDistCallback, this,
                    std::placeholders::_1));
    }
    // ★실측 속도 [금색차 이식 신설]★ /encoder 는 A보드 좌+우 펄스의 ★합★ 이고
    //   1카운트 = 0.442 m/s 다(nxde/README 6절). 제동 ★해제★ 판정에만 쓴다 —
    //   체결은 기하가 정하고 실측은 '푸는 쪽'으로만 작용시켜야 되먹임이 안 생긴다
    //   (white1 corner_brake / goal_approach 의 비대칭과 같은 규칙).
    //   ⚠️ A보드 기동 블랭킹이 정지 중에도 허수 카운트를 쏟으므로 3점 중앙값을 건다.
    if (use_encoder_speed_) {
      encoder_sub_ = this->create_subscription<std_msgs::msg::Int32>(
          "/encoder", 10,
          [this](const std_msgs::msg::Int32::ConstSharedPtr& m) {
            enc_hist_[enc_idx_ % 3] = std::max(0, static_cast<int>(m->data));
            ++enc_idx_;
            has_encoder_ = enc_idx_ >= 3;
          });
    }

    if (use_imu_yaw_rate_) {
      imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
          imu_topic, rclcpp::SensorDataQoS(),
          std::bind(&DriveLidarNode::imuCallback, this, std::placeholders::_1));
    }

    param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(
        &DriveLidarNode::onParamChange, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "drive_lidar v6: survey %.1fs then lock heading + go straight. "
                "lidar=%s cruise=%.2f m/s (%.1f km/h) L=%.2fm survey_x<=%.1fm",
                survey_duration_, lidar_topic.c_str(), linear_speed_,
                linear_speed_ * 3.6, wheelbase_, survey_roi_x_max_);
  }

private:
  /// 지금 실제로 몇 m/s 로 구르고 있나. 엔코더가 없으면 지령속도로 대체한다.
  /// ★대체할 때는 지령이 실측보다 크게 나오는 쪽이라 '덜 푼다' = 안전한 방향★
  double measuredMs() const {
    if (!has_encoder_) return prev_cmd_speed_;
    int a = enc_hist_[0], b = enc_hist_[1], c = enc_hist_[2];
    const int med = std::max(std::min(a, b), std::min(std::max(a, b), c));
    return med * 0.442;   // /encoder 1카운트 = 0.442 m/s (좌+우 합)
  }

  /// 지상높이(AGL) → 센서 기준 z. z-up, 센서 원점이 z=0.
  void recomputeZSlab() {
    roi_z_min_ = roi_agl_min_ - sensor_height_m_;
    roi_z_max_ = roi_agl_max_ - sensor_height_m_;
  }

  enum class Phase { Survey, Drive, Failed };

  struct Cluster {
    double x = 0.0, y = 0.0;
    int n = 0;
    double extent = 0.0;
    std::vector<int> pt_idx;
  };
  struct LineFit {
    bool valid = false;
    double b0 = 0.0, b1 = 0.0;
    int n_cones = 0;
    double x_span = 0.0;
  };

  static double normalizeAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  void loadParamsToMembers() {
    this->get_parameter("flip_lidar_xy", flip_lidar_xy_);
    this->get_parameter("roi_x_min", roi_x_min_);
    this->get_parameter("roi_x_max", roi_x_max_);
    this->get_parameter("roi_y_max", roi_y_max_);
    this->get_parameter("sensor_height_m", sensor_height_m_);
    this->get_parameter("roi_agl_min", roi_agl_min_);
    this->get_parameter("roi_agl_max", roi_agl_max_);
    recomputeZSlab();
    this->get_parameter("cluster_cell_size", cluster_cell_size_);
    this->get_parameter("min_cluster_points", min_cluster_points_);
    this->get_parameter("max_cluster_extent", max_cluster_extent_);
    this->get_parameter("lane_band_min", lane_band_min_);
    this->get_parameter("lane_band_max", lane_band_max_);
    this->get_parameter("line_fit_residual_gate", line_fit_residual_gate_);
    this->get_parameter("initial_half_width", initial_half_width_);
    this->get_parameter("corridor_center_y", corridor_center_y_);
    this->get_parameter("wheelbase", wheelbase_);
    this->get_parameter("linear_speed", linear_speed_);
    this->get_parameter("k_heading", k_heading_);
    this->get_parameter("k_stanley", k_stanley_);
    this->get_parameter("max_angular_speed", max_angular_speed_);
    this->get_parameter("max_angular_step", max_angular_step_);
    this->get_parameter("listen_to_aeb_stop_signal", listen_to_aeb_stop_signal_);
    this->get_parameter("aeb_confirm_frames", aeb_confirm_frames_);
    this->get_parameter("aeb_require_own_gate", aeb_require_own_gate_);
    this->get_parameter("aeb_own_gate_max_range", aeb_own_gate_max_range_);
    this->get_parameter("gate_stop_start_distance", gate_stop_start_distance_);
    this->get_parameter("gate_stop_end_distance", gate_stop_end_distance_);
    this->get_parameter("aeb_brake_start_distance", aeb_brake_start_distance_);
    this->get_parameter("aeb_brake_end_distance", aeb_brake_end_distance_);
    this->get_parameter("aeb_max_decel", aeb_max_decel_);
    this->get_parameter("aeb_use_distance_brake", aeb_use_distance_brake_);
    this->get_parameter("use_imu_yaw_rate", use_imu_yaw_rate_);
    this->get_parameter("imu_yaw_rate_lpf", imu_yaw_rate_lpf_);
    this->get_parameter("survey_duration", survey_duration_);
    this->get_parameter("survey_timeout", survey_timeout_);
    this->get_parameter("survey_roi_x_max", survey_roi_x_max_);
    this->get_parameter("survey_merge_dist", survey_merge_dist_);
    this->get_parameter("survey_min_side_cones", survey_min_side_cones_);
    this->get_parameter("survey_min_x_span", survey_min_x_span_);
    this->get_parameter("survey_max_heading_deg", survey_max_heading_deg_);
    this->get_parameter("lock_use_cte", lock_use_cte_);
    this->get_parameter("publish_debug", publish_debug_);
    this->get_parameter("brake_enable", brake_enable_);
    this->get_parameter("brake_margin_m", brake_margin_m_);
    this->get_parameter("brake_release_k", brake_release_k_);
    this->get_parameter("use_encoder_speed", use_encoder_speed_);
  }

  rcl_interfaces::msg::SetParametersResult onParamChange(
      const std::vector<rclcpp::Parameter>& params) {
    for (const auto& p : params) {
      const auto& name = p.get_name();
      if (name == "roi_x_min")
        roi_x_min_ = p.as_double();
      else if (name == "roi_x_max")
        roi_x_max_ = p.as_double();
      else if (name == "roi_y_max")
        roi_y_max_ = p.as_double();
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
      else if (name == "linear_speed")
        linear_speed_ = p.as_double();
      else if (name == "k_heading")
        k_heading_ = p.as_double();
      else if (name == "k_stanley")
        k_stanley_ = p.as_double();
      else if (name == "max_angular_speed")
        max_angular_speed_ = p.as_double();
      else if (name == "max_angular_step")
        max_angular_step_ = p.as_double();
      else if (name == "listen_to_aeb_stop_signal")
        listen_to_aeb_stop_signal_ = p.as_bool();
      else if (name == "aeb_confirm_frames")
        aeb_confirm_frames_ = static_cast<int>(p.as_int());
      else if (name == "survey_duration")
        survey_duration_ = p.as_double();
      else if (name == "lock_use_cte")
        lock_use_cte_ = p.as_bool();
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void aebStopCallback(const std_msgs::msg::Bool::ConstSharedPtr& msg) {
    aeb_raw_signal_ = msg->data;
    aeb_raw_signal_received_ = true;
  }

  void aebDistCallback(const std_msgs::msg::Float32::ConstSharedPtr& msg) {
    aeb_obstacle_distance_ = msg->data;
    aeb_distance_received_ = true;
  }

  static double distanceSpeedScale(double d, double start_d, double end_d) {
    if (!std::isfinite(d)) return 1.0;
    if (d <= end_d) return 0.0;
    if (d >= start_d) return 1.0;
    return (d - end_d) / std::max(start_d - end_d, 1e-3);
  }

  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
    const double raw = msg->angular_velocity.z;
    const double now = rclcpp::Time(msg->header.stamp).seconds();
    if (now > 0.0 && last_imu_time_ > 0.0 && phase_ == Phase::Drive) {
      double dt = now - last_imu_time_;
      if (dt > 0.0 && dt < 0.2) {
        psi_ += (raw - gyro_bias_) * dt;
        psi_ = normalizeAngle(psi_);
      }
    }
    last_imu_time_ = now;

    if (!imu_yaw_rate_valid_) {
      imu_yaw_rate_ = raw;
      imu_yaw_rate_valid_ = true;
    } else {
      const double a = std::clamp(imu_yaw_rate_lpf_, 0.0, 0.99);
      imu_yaw_rate_ = a * imu_yaw_rate_ + (1.0 - a) * raw;
    }
    imu_received_ = true;

    if (phase_ == Phase::Survey) {
      gyro_bias_sum_ += raw;
      ++gyro_bias_count_;
    }
  }

  std::vector<Cluster> clusterPoints(const std::vector<float>& xs,
                                     const std::vector<float>& ys) const {
    std::vector<Cluster> clusters;
    const double cell = std::max(cluster_cell_size_, 0.05);
    std::unordered_map<int64_t, std::vector<int>> grid;
    grid.reserve(xs.size() * 2);
    auto key_of = [cell](float x, float y) {
      int64_t gx = static_cast<int64_t>(std::floor(x / cell)) + (1 << 20);
      int64_t gy = static_cast<int64_t>(std::floor(y / cell)) + (1 << 20);
      return (gx << 21) | gy;
    };
    for (size_t i = 0; i < xs.size(); ++i)
      grid[key_of(xs[i], ys[i])].push_back(static_cast<int>(i));

    std::unordered_map<int64_t, bool> visited;
    visited.reserve(grid.size() * 2);
    std::vector<int64_t> stack;
    for (const auto& kv : grid) {
      if (visited[kv.first]) continue;
      stack.clear();
      stack.push_back(kv.first);
      visited[kv.first] = true;
      Cluster c;
      float min_x = std::numeric_limits<float>::max(), max_x = -min_x;
      float min_y = min_x, max_y = -min_x;
      double sum_x = 0.0, sum_y = 0.0;
      while (!stack.empty()) {
        int64_t k = stack.back();
        stack.pop_back();
        auto it = grid.find(k);
        if (it == grid.end()) continue;
        for (int idx : it->second) {
          c.pt_idx.push_back(idx);
          sum_x += xs[idx];
          sum_y += ys[idx];
          min_x = std::min(min_x, xs[idx]);
          max_x = std::max(max_x, xs[idx]);
          min_y = std::min(min_y, ys[idx]);
          max_y = std::max(max_y, ys[idx]);
        }
        int64_t gx = k >> 21, gy = k & ((1LL << 21) - 1);
        for (int dx = -1; dx <= 1; ++dx) {
          for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue;
            int64_t nb = ((gx + dx) << 21) | (gy + dy);
            if (grid.count(nb) && !visited[nb]) {
              visited[nb] = true;
              stack.push_back(nb);
            }
          }
        }
      }
      c.n = static_cast<int>(c.pt_idx.size());
      if (c.n == 0) continue;
      c.x = sum_x / c.n;
      c.y = sum_y / c.n;
      c.extent = std::max(max_x - min_x, max_y - min_y);
      clusters.push_back(std::move(c));
    }
    return clusters;
  }

  // Unweighted-in-range LS. Far cones keep their say so heading is not
  // dominated by the 2 m pair next to the bumper.
  LineFit fitLine(const std::vector<const Cluster*>& cones) const {
    LineFit f;
    if (cones.empty()) return f;

    std::vector<double> x, y, w;
    for (const auto* c : cones) {
      x.push_back(c->x);
      y.push_back(c->y);
      w.push_back(std::sqrt(std::min(static_cast<double>(c->n), 80.0)));
    }
    auto span_of = [](const std::vector<double>& v) {
      auto [mn, mx] = std::minmax_element(v.begin(), v.end());
      return *mx - *mn;
    };

    if (x.size() >= 2 && span_of(x) > 0.5) {
      for (int pass = 0; pass < 2; ++pass) {
        double sw = 0, swx = 0, swy = 0, swxx = 0, swxy = 0;
        for (size_t i = 0; i < x.size(); ++i) {
          sw += w[i];
          swx += w[i] * x[i];
          swy += w[i] * y[i];
          swxx += w[i] * x[i] * x[i];
          swxy += w[i] * x[i] * y[i];
        }
        const double det = sw * swxx - swx * swx;
        if (std::fabs(det) < 1e-9) break;
        f.b1 = (sw * swxy - swx * swy) / det;
        f.b0 = (swy - f.b1 * swx) / sw;
        f.valid = true;
        if (pass == 1) break;
        std::vector<double> x2, y2, w2;
        for (size_t i = 0; i < x.size(); ++i) {
          if (std::fabs(y[i] - (f.b0 + f.b1 * x[i])) < line_fit_residual_gate_) {
            x2.push_back(x[i]);
            y2.push_back(y[i]);
            w2.push_back(w[i]);
          }
        }
        if (x2.size() < 2 || x2.size() == x.size()) break;
        x = std::move(x2);
        y = std::move(y2);
        w = std::move(w2);
      }
    }
    if (f.valid) {
      f.n_cones = static_cast<int>(x.size());
      f.x_span = span_of(x);
    }
    return f;
  }

  void mergeSurveyCone(double x, double y, int n) {
    for (auto& c : survey_cones_) {
      if (std::hypot(x - c.x, y - c.y) < survey_merge_dist_) {
        const double w0 = static_cast<double>(c.n);
        const double w1 = static_cast<double>(std::max(n, 1));
        const double wt = w0 + w1;
        c.x = (c.x * w0 + x * w1) / wt;
        c.y = (c.y * w0 + y * w1) / wt;
        c.n += n;
        return;
      }
    }
    Cluster c;
    c.x = x;
    c.y = y;
    c.n = std::max(n, 1);
    survey_cones_.push_back(c);
  }

  bool tryLockCorridor() {
    std::vector<const Cluster*> left, right;
    for (const auto& c : survey_cones_) {
      const double a = std::fabs(c.y - corridor_center_y_);
      if (a < lane_band_min_ || a > lane_band_max_) continue;
      if (c.y >= corridor_center_y_)
        left.push_back(&c);
      else
        right.push_back(&c);
    }

    const LineFit lf = fitLine(left);
    const LineFit rf = fitLine(right);
    const bool left_ok =
        lf.valid && lf.n_cones >= survey_min_side_cones_ &&
        lf.x_span >= survey_min_x_span_;
    const bool right_ok =
        rf.valid && rf.n_cones >= survey_min_side_cones_ &&
        rf.x_span >= survey_min_x_span_;

    const double hw = initial_half_width_;
    const double x_ref = 5.0;
    double c0 = 0.0, c1 = 0.0;
    if (left_ok && right_ok) {
      c1 = 0.5 * (lf.b1 + rf.b1);
      const double yl = lf.b0 + lf.b1 * x_ref;
      const double yr = rf.b0 + rf.b1 * x_ref;
      c0 = 0.5 * (yl + yr) - c1 * x_ref;
    } else if (left_ok && !requireBothSides()) {
      c1 = lf.b1;
      c0 = (lf.b0 + lf.b1 * x_ref) - hw - c1 * x_ref;
    } else if (right_ok && !requireBothSides()) {
      c1 = rf.b1;
      c0 = (rf.b0 + rf.b1 * x_ref) + hw - c1 * x_ref;
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "survey not ready L=%d(span=%.1f n=%d) R=%d(span=%.1f n=%d) "
                  "merged=%zu",
                  left_ok ? 1 : 0, lf.x_span, lf.n_cones, right_ok ? 1 : 0,
                  rf.x_span, rf.n_cones, survey_cones_.size());
      return false;
    }

    const double heading = std::atan(c1);
    if (std::fabs(heading) * 180.0 / M_PI > survey_max_heading_deg_) {
      RCLCPP_ERROR(this->get_logger(),
                   "복도 헤딩 %.1f° 가 너무 큼 — 잠그지 않음",
                   heading * 180.0 / M_PI);
      return false;
    }

    lock_c0_ = c0;
    lock_c1_ = c1;
    lock_psi_ = heading;
    lock_left_ = lf;
    lock_right_ = rf;
    has_lock_left_ = left_ok;
    has_lock_right_ = right_ok;
    if (gyro_bias_count_ > 5) gyro_bias_ = gyro_bias_sum_ / gyro_bias_count_;
    psi_ = 0.0;
    x_w_ = 0.0;
    y_w_ = 0.0;
    has_prev_angular_ = false;
    has_prev_speed_ = false;
    prev_cmd_speed_ = 0.0;
    locked_ = true;
    last_imu_time_ = 0.0;
    phase_ = Phase::Drive;

    RCLCPP_INFO(this->get_logger(),
                "🔒 복도 잠금: heading=%.2f° c0=%.2fm L=%d/%d R=%d/%d "
                "merged=%zu gyro_bias=%.4f rad/s cte_hold=%s",
                lock_psi_ * 180.0 / M_PI, lock_c0_, lf.n_cones,
                static_cast<int>(left.size()), rf.n_cones,
                static_cast<int>(right.size()), survey_cones_.size(),
                gyro_bias_, lock_use_cte_ ? "on" : "off");
    return true;
  }

  bool requireBothSides() const {
    // Dual-side preferred; single-side allowed unless explicitly many cones
    // exist on neither side after timeout we still try single.
    return false;
  }

  /// 구동 정지 — 펄스 0, 구동허용 off. ★리니어는 만지지 않는다★
  /// (측량 단계는 차가 이미 서 있고, 모드 전환·대기가 제동 지시는 아니다)
  void publishStopCmd() {
    actuator_->hold(/*control_enable=*/false);
    actuator_->keepBrake();
    prev_cmd_speed_ = 0.0;
  }

  void cloudCallback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg) {
    std_msgs::msg::Header header = cloud_msg->header;
    header.frame_id = "os_sensor";

    const double t = rclcpp::Time(cloud_msg->header.stamp).seconds();
    double dt = 0.1;
    if (has_last_cloud_time_) {
      dt = t - last_cloud_time_;
      if (dt <= 0.0 || dt > 0.5) dt = 0.1;
    }
    last_cloud_time_ = t;
    has_last_cloud_time_ = true;

    if (phase_ == Phase::Survey && !survey_started_) {
      survey_started_ = true;
      survey_t0_ = t;
      RCLCPP_INFO(this->get_logger(),
                  "🛑 정지 관측 %.1fs — 양쪽 라바콘 누적 중", survey_duration_);
    }

    const double x_max =
        (phase_ == Phase::Survey) ? survey_roi_x_max_ : std::max(roi_x_max_, 12.0);

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*cloud_msg, pcl_cloud);
    std::vector<float> xs, ys, zs;
    xs.reserve(pcl_cloud.size() / 8);
    for (const auto& p : pcl_cloud.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        continue;
      float x = p.x, y = p.y, z = p.z;
      if (flip_lidar_xy_) {
        x = -p.x;
        y = -p.y;
      }
      if (z < roi_z_min_ || z > roi_z_max_) continue;
      if (x < roi_x_min_ || x > x_max) continue;
      if (std::fabs(y) > roi_y_max_) continue;
      xs.push_back(x);
      ys.push_back(y);
      zs.push_back(z);
    }

    std::vector<Cluster> clusters = clusterPoints(xs, ys);
    std::vector<const Cluster*> cones;
    for (const auto& c : clusters) {
      if (c.n >= min_cluster_points_ && c.extent <= max_cluster_extent_)
        cones.push_back(&c);
    }

    // Live gate row (AEB assist only — never used for steering after lock)
    double nearest_gate_x = std::numeric_limits<double>::max();
    int gate_count = 0;
    for (const auto* c : cones) {
      if (std::fabs(c->y) < 0.75 && c->x > 1.0) {
        nearest_gate_x = std::min(nearest_gate_x, c->x);
        ++gate_count;
      }
    }
    const bool has_own_gate = gate_count >= 2 && std::isfinite(nearest_gate_x) &&
                              nearest_gate_x <= aeb_own_gate_max_range_;

    if (phase_ == Phase::Survey) {
      for (const auto* c : cones) mergeSurveyCone(c->x, c->y, c->n);
      publishStopCmd();

      const double elapsed = t - survey_t0_;
      const bool time_ok = elapsed >= survey_duration_;
      if (time_ok && tryLockCorridor()) {
        publishDebugSurvey(header, xs, ys, zs);
        return;
      }
      if (elapsed >= survey_timeout_) {
        if (!tryLockCorridor()) {
          phase_ = Phase::Failed;
          RCLCPP_ERROR(this->get_logger(),
                       "❌ 관측 실패 (%.1fs, cones=%zu) — 정지 유지", elapsed,
                       survey_cones_.size());
        }
      }
      if (publish_debug_) publishDebugSurvey(header, xs, ys, zs);
      if (static_cast<int>(elapsed * 10) % 10 == 0) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                             "survey %.1f/%.1fs merged=%zu", elapsed,
                             survey_duration_, survey_cones_.size());
      }
      return;
    }

    if (phase_ == Phase::Failed) {
      publishStopCmd();
      return;
    }

    // ---- Drive: heading hold on locked corridor, no live cone steering ----
    if (use_imu_yaw_rate_ && imu_yaw_rate_valid_ && last_imu_time_ <= 0.0) {
      // IMU stamps missing: integrate LPF rate on the lidar clock
      psi_ += (imu_yaw_rate_ - gyro_bias_) * dt;
      psi_ = normalizeAngle(psi_);
    }

    const double v_prev = has_prev_speed_ ? prev_cmd_speed_ : 0.0;
    x_w_ += v_prev * std::cos(psi_) * dt;
    y_w_ += v_prev * std::sin(psi_) * dt;

    const double e_psi = normalizeAngle(lock_psi_ - psi_);
    // Vehicle-frame lateral of the locked line at the origin:
    // signed dist to y = c0 + c1 x, left-of-path negative in (c1 x - y + c0).
    const double nrm = std::hypot(1.0, lock_c1_);
    const double e_y =
        lock_use_cte_ ? (lock_c1_ * x_w_ - y_w_ + lock_c0_) / nrm : 0.0;

    double angular = std::clamp(k_heading_ * e_psi, -max_angular_speed_,
                                max_angular_speed_);
    if (lock_use_cte_) {
      angular += lidar::LineEkf::stanleySteer(e_y, 0.0, std::max(linear_speed_, 0.5),
                                               k_stanley_, max_angular_speed_, 0.0);
      angular = std::clamp(angular, -max_angular_speed_, max_angular_speed_);
    }
    if (has_prev_angular_) {
      const double step = std::clamp(angular - prev_angular_z_, -max_angular_step_,
                                     max_angular_step_);
      angular = prev_angular_z_ + step;
    }
    prev_angular_z_ = angular;
    has_prev_angular_ = true;

    if (listen_to_aeb_stop_signal_ && aeb_raw_signal_)
      ++aeb_true_streak_;
    else
      aeb_true_streak_ = 0;
    const bool aeb_confirmed =
        listen_to_aeb_stop_signal_ && aeb_raw_signal_received_ &&
        aeb_true_streak_ >= std::max(aeb_confirm_frames_, 1);
    const bool aeb_range_ok =
        aeb_distance_received_ && std::isfinite(aeb_obstacle_distance_) &&
        aeb_obstacle_distance_ < aeb_brake_start_distance_;
    const bool aeb_trusted =
        aeb_confirmed &&
        (!aeb_require_own_gate_ || has_own_gate ||
         (aeb_use_distance_brake_ && aeb_range_ok));
    if (aeb_trusted && !aeb_stop_active_) {
      RCLCPP_WARN(this->get_logger(),
                  "🚨 [AEB] brake (own_gate=%d d=%.2f)", has_own_gate ? 1 : 0,
                  aeb_distance_received_ ? aeb_obstacle_distance_ : -1.0);
    }
    aeb_stop_active_ = aeb_trusted;

    double gate_scale = 1.0;
    if (has_own_gate) {
      gate_scale = distanceSpeedScale(
          nearest_gate_x, gate_stop_start_distance_, gate_stop_end_distance_);
    }
    double aeb_scale = 1.0;
    if (aeb_use_distance_brake_ && aeb_distance_received_ &&
        std::isfinite(aeb_obstacle_distance_)) {
      aeb_scale = distanceSpeedScale(aeb_obstacle_distance_,
                                     aeb_brake_start_distance_,
                                     aeb_brake_end_distance_);
      if (!aeb_stop_active_ && aeb_scale > 0.4) aeb_scale = 1.0;
      if (!aeb_stop_active_ && aeb_scale <= 0.4)
        aeb_scale = std::max(aeb_scale, 0.35);
    } else if (aeb_stop_active_) {
      aeb_scale = 0.0;
    }

    double desired_v = linear_speed_ * std::min(gate_scale, aeb_scale);
    {
      const double t_now = this->now().seconds();
      double cdt = 0.1;
      if (has_last_cmd_time_)
        cdt = std::clamp(t_now - last_cmd_time_, 0.02, 0.25);
      last_cmd_time_ = t_now;
      has_last_cmd_time_ = true;
      if (!has_prev_speed_) {
        prev_cmd_speed_ = desired_v;
        has_prev_speed_ = true;
      }
      const double max_drop = std::max(aeb_max_decel_, 0.5) * cdt;
      const double max_rise = 2.0 * cdt;
      double v = desired_v;
      if (v < prev_cmd_speed_ - max_drop) v = prev_cmd_speed_ - max_drop;
      if (v > prev_cmd_speed_ + max_rise) v = prev_cmd_speed_ + max_rise;
      if (v < 0.0) v = 0.0;
      prev_cmd_speed_ = v;

      // ══ ★리니어 제동 판정 [금색차 이식 신설]★ ══════════════════════════
      //  원본은 여기서 v(=지령 m/s)와 조향각을 그대로 발행하고 끝이었다.
      //  금색차에서는 그것만으로 서지 않는다 — 아래 두 줄이 그 차이다.
      //
      //   ★체결은 기하로 / 해제는 실측으로★ (white1 과 같은 비대칭)
      //     · 물 것인가 : '설 지점까지 거리' vs '그 속도의 정지거리' — 실측 안 봄
      //     · 풀 것인가 : 엔코더 실측이 회복을 확인해야 — 푸는 쪽으로만 작용
      //   ★단계는 올라가기만 한다★ 리니어 왕복이 제일 나쁘다(KasaActuator 가 강제)
      //   ★이미 늦었으면 1단을 건너뛰고 곧장 2단★ B보드는 진행 중인 행정이
      //     끝나야 다음 이동을 시작해서, 같은 틱에 1→2 를 물면 2단이 그만큼 늦는다.
      const double v_meas = measuredMs();

      // '어디서 서야 하는가' — AEB 장애물과 자기 게이트 중 가까운 쪽
      double d_stop = std::numeric_limits<double>::infinity();
      if (aeb_distance_received_ && std::isfinite(aeb_obstacle_distance_)) {
        d_stop = aeb_obstacle_distance_ - aeb_brake_end_distance_;
      }
      if (has_own_gate) {
        d_stop = std::min(d_stop, nearest_gate_x - gate_stop_end_distance_);
      }

      const double need1 = lidar::kasa::stopDist(
          v_meas, lidar::kasa::A_BRAKE1_MS2, lidar::kasa::BRAKE1_LAG_S)
          + brake_margin_m_;
      const double need2 = 1.2 * lidar::kasa::stopDist(
          v_meas, lidar::kasa::A_BRAKE2_MS2, lidar::kasa::BRAKE2_LAG_S)
          + brake_margin_m_;

      if (brake_enable_) {
        if (d_stop <= need2) {
          actuator_->brake(lidar::kasa::BRAKE_FULL);
        } else if (d_stop <= need1 || aeb_stop_active_) {
          actuator_->brake(lidar::kasa::BRAKE_SOFT);
        } else if (actuator_->brakeStage() > lidar::kasa::BRAKE_OFF
                   && !aeb_stop_active_
                   && d_stop > brake_release_k_ * need1) {
          actuator_->releaseBrake();   // 최소 물림은 액추에이터가 지킨다
        }
        actuator_->keepBrake();
      }

      // ★제동 중에는 구동을 내지 않는다★ arduino 도 brake>0 이면 A보드 REF 를
      //   0 으로 덮지만, 여기서도 명시적으로 0 을 낸다 — 둘이 갈라지면 "리니어는
      //   물렸는데 펄스는 2" 조합이 한 틱이라도 생기고, 그것이 정확히 구동과
      //   제동이 서로 미는 상태다(white1 goal_approach 주석과 같은 이유).
      const bool braking = actuator_->brakeStage() > lidar::kasa::BRAKE_OFF;
      const double v_out = braking ? 0.0 : v;
      const double steer_road_deg =
          ((v_out < 0.15 && aeb_stop_active_) || braking)
              ? 0.0 : angular * 180.0 / M_PI;

      // ★여기가 이 노드의 유일한 출력이다★ m/s → 펄스, 도로휠각 → pot 지령,
      //   + 좌 → − 좌 반전이 전부 drive() 안에서 한 번에 일어난다.
      if (!actuator_->ready()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "⛔ 구동 불가 — %s", actuator_->blockReason());
        actuator_->hold(/*control_enable=*/false);
      } else {
        actuator_->drive(v_out, steer_road_deg, /*control_enable=*/true);
      }
    }

    publishLockedPath(header);
    if (publish_debug_) {
      publishDebugDrive(header, xs, ys, zs, cones, e_psi, e_y);
    }

    if (log_counter_++ % 20 == 0) {
      RCLCPP_INFO(this->get_logger(),
                  "[LOCK] psi=%.2f/%.2f deg ey=%.2f v=%.2f aeb=%d d=%.1f",
                  psi_ * 180.0 / M_PI, lock_psi_ * 180.0 / M_PI, e_y,
                  prev_cmd_speed_, aeb_stop_active_ ? 1 : 0,
                  aeb_distance_received_ ? aeb_obstacle_distance_ : -1.0);
    }
  }

  // Locked line from survey frame → current vehicle frame for RViz (os_sensor)
  void worldToVeh(double x, double y, double& xo, double& yo) const {
    const double dx = x - x_w_;
    const double dy = y - y_w_;
    const double c = std::cos(psi_);
    const double s = std::sin(psi_);
    xo = c * dx + s * dy;
    yo = -s * dx + c * dy;
  }

  void publishLockedPath(const std_msgs::msg::Header& header) {
    nav_msgs::msg::Path path;
    path.header = header;
    const double x_max = std::max(survey_roi_x_max_, 12.0);
    for (double s = 0.0; s <= x_max; s += 0.5) {
      const double xw = s;
      const double yw = lock_c0_ + lock_c1_ * s;
      double xv, yv;
      worldToVeh(xw, yw, xv, yv);
      geometry_msgs::msg::PoseStamped ps;
      ps.header = header;
      ps.pose.position.x = xv;
      ps.pose.position.y = yv;
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    path_pub_->publish(path);

    geometry_msgs::msg::PointStamped tgt;
    tgt.header = header;
    const double ld = std::clamp(0.6 * linear_speed_, 2.0, 6.0);
    double xv, yv;
    worldToVeh(x_w_ + ld, lock_c0_ + lock_c1_ * (x_w_ + ld), xv, yv);
    tgt.point.x = xv;
    tgt.point.y = yv;
    steering_target_pub_->publish(tgt);
  }

  void publishDebugSurvey(const std_msgs::msg::Header& header,
                          const std::vector<float>& xs,
                          const std::vector<float>& ys,
                          const std::vector<float>& zs) {
    std::vector<const Cluster*> left, right;
    for (const auto& c : survey_cones_) {
      if (c.y >= corridor_center_y_)
        left.push_back(&c);
      else
        right.push_back(&c);
    }
    publishDebugClouds(header, xs, ys, zs, left, right);

    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    visualization_msgs::msg::Marker dots;
    dots.header = header;
    dots.ns = "survey_cones";
    dots.id = 0;
    dots.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    dots.action = visualization_msgs::msg::Marker::ADD;
    dots.scale.x = dots.scale.y = dots.scale.z = 0.18;
    dots.color.b = 1.0f;
    dots.color.a = 0.9f;
    dots.pose.orientation.w = 1.0;
    for (const auto& c : survey_cones_) {
      geometry_msgs::msg::Point p;
      p.x = c.x;
      p.y = c.y;
      dots.points.push_back(p);
    }
    arr.markers.push_back(dots);
    if (marker_pub_) marker_pub_->publish(arr);
  }

  void publishDebugDrive(const std_msgs::msg::Header& header,
                         const std::vector<float>& xs,
                         const std::vector<float>& ys,
                         const std::vector<float>& zs,
                         const std::vector<const Cluster*>& live,
                         double e_psi, double e_y) {
    std::vector<const Cluster*> left, right;
    for (const auto* c : live) {
      if (c->y >= 0.0)
        left.push_back(c);
      else
        right.push_back(c);
    }
    publishDebugClouds(header, xs, ys, zs, left, right);

    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    auto add_line = [&](const std::string& ns, int id, double b0, double b1,
                        float r, float g, float b) {
      visualization_msgs::msg::Marker line;
      line.header = header;
      line.ns = ns;
      line.id = id;
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.scale.x = 0.07;
      line.color.r = r;
      line.color.g = g;
      line.color.b = b;
      line.color.a = 0.95f;
      line.pose.orientation.w = 1.0;
      line.lifetime = rclcpp::Duration::from_seconds(0.3);
      const double x_max = std::max(survey_roi_x_max_, 12.0);
      for (double s = 0.0; s <= x_max; s += x_max) {
        double xv, yv;
        worldToVeh(s, b0 + b1 * s, xv, yv);
        geometry_msgs::msg::Point p;
        p.x = xv;
        p.y = yv;
        line.points.push_back(p);
      }
      arr.markers.push_back(line);
    };
    if (has_lock_left_)
      add_line("left_cone_boundary", 0, lock_left_.b0, lock_left_.b1, 0.1f, 0.3f,
               1.0f);
    if (has_lock_right_)
      add_line("right_cone_boundary", 1, lock_right_.b0, lock_right_.b1, 1.0f,
               0.6f, 0.0f);
    add_line("corridor_center_path", 2, lock_c0_, lock_c1_, 0.0f, 1.0f, 0.2f);
    (void)e_psi;
    (void)e_y;
    if (marker_pub_) marker_pub_->publish(arr);
  }

  void publishDebugClouds(const std_msgs::msg::Header& header,
                          const std::vector<float>& xs,
                          const std::vector<float>& ys,
                          const std::vector<float>& zs,
                          const std::vector<const Cluster*>& left_cand,
                          const std::vector<const Cluster*>& right_cand) {
    if (!left_cloud_pub_ || !right_cloud_pub_) return;
    auto make_cloud = [&](const std::vector<const Cluster*>& side) {
      pcl::PointCloud<pcl::PointXYZ> cloud;
      for (const auto* c : side) {
        if (c->pt_idx.empty()) {
          cloud.points.emplace_back(static_cast<float>(c->x),
                                    static_cast<float>(c->y), 0.0f);
          continue;
        }
        for (int idx : c->pt_idx)
          cloud.points.emplace_back(xs[idx], ys[idx], zs[idx]);
      }
      cloud.width = static_cast<uint32_t>(cloud.points.size());
      cloud.height = 1;
      cloud.is_dense = true;
      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(cloud, msg);
      msg.header = header;
      return msg;
    };
    left_cloud_pub_->publish(make_cloud(left_cand));
    right_cloud_pub_->publish(make_cloud(right_cand));
  }

  bool flip_lidar_xy_ = true;
  double roi_x_min_ = 0.5, roi_x_max_ = 8.0, roi_y_max_ = 6.0;
  // ★유도값★ 소유자는 아래 AGL 세 값이다 (recomputeZSlab)
  double roi_z_min_ = -0.80, roi_z_max_ = 0.20;
  double sensor_height_m_ = 1.17;
  double roi_agl_min_ = 0.20;
  double roi_agl_max_ = 1.20;
  double cluster_cell_size_ = 0.35;
  int min_cluster_points_ = 2;
  double max_cluster_extent_ = 0.8;
  double lane_band_min_ = 1.0, lane_band_max_ = 4.0;
  double line_fit_residual_gate_ = 0.7;
  double initial_half_width_ = 2.2;
  double corridor_center_y_ = 0.0;
  double wheelbase_ = lidar::kasa::WHEELBASE_M;
  double linear_speed_ = 2.0;
  double k_heading_ = 1.8;
  double k_stanley_ = 0.0;
  double max_angular_speed_ = 0.366;
  double max_angular_step_ = 0.12;
  bool listen_to_aeb_stop_signal_ = true;
  int aeb_confirm_frames_ = 2;
  bool aeb_require_own_gate_ = false;
  double aeb_own_gate_max_range_ = 12.0;
  double gate_stop_start_distance_ = 6.0;
  double gate_stop_end_distance_ = 1.5;
  double aeb_brake_start_distance_ = 8.5;
  double aeb_brake_end_distance_ = 1.5;
  double aeb_max_decel_ = 5.0;
  bool aeb_use_distance_brake_ = true;
  bool use_imu_yaw_rate_ = true;
  double imu_yaw_rate_lpf_ = 0.3;
  double survey_duration_ = 2.5;
  double survey_timeout_ = 6.0;
  double survey_roi_x_max_ = 16.0;
  double survey_merge_dist_ = 0.45;
  int survey_min_side_cones_ = 2;
  double survey_min_x_span_ = 3.0;
  double survey_max_heading_deg_ = 25.0;
  bool lock_use_cte_ = false;
  bool publish_debug_ = true;

  Phase phase_ = Phase::Survey;
  bool survey_started_ = false;
  double survey_t0_ = 0.0;
  std::vector<Cluster> survey_cones_;

  bool locked_ = false;
  double lock_c0_ = 0.0, lock_c1_ = 0.0, lock_psi_ = 0.0;
  LineFit lock_left_, lock_right_;
  bool has_lock_left_ = false, has_lock_right_ = false;
  double psi_ = 0.0, x_w_ = 0.0, y_w_ = 0.0;
  double gyro_bias_ = 0.0, gyro_bias_sum_ = 0.0;
  int gyro_bias_count_ = 0;
  double last_imu_time_ = 0.0;

  bool aeb_stop_active_ = false;
  bool aeb_raw_signal_ = false;
  bool aeb_raw_signal_received_ = false;
  int aeb_true_streak_ = 0;
  bool aeb_distance_received_ = false;
  float aeb_obstacle_distance_ = std::numeric_limits<float>::infinity();

  bool has_prev_angular_ = false;
  double prev_angular_z_ = 0.0;
  bool has_prev_speed_ = false;
  double prev_cmd_speed_ = 0.0;
  bool has_last_cmd_time_ = false;
  double last_cmd_time_ = 0.0;
  bool imu_received_ = false;
  bool imu_yaw_rate_valid_ = false;
  double imu_yaw_rate_ = 0.0;
  bool has_last_cloud_time_ = false;
  double last_cloud_time_ = 0.0;
  int log_counter_ = 0;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr aeb_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr aeb_dist_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr encoder_sub_;
  int enc_hist_[3] = {0, 0, 0};
  unsigned enc_idx_ = 0;
  bool has_encoder_ = false;
  bool use_encoder_speed_ = true;
  bool brake_enable_ = true;
  double brake_margin_m_ = 1.0;
  double brake_release_k_ = 1.5;
  std::unique_ptr<lidar::kasa::KasaActuator> actuator_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
      steering_target_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr left_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr right_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<DriveLidarNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("drive_lidar_node"), "fatal: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
