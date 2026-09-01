// ============================================================================
// drive_gps_node.cpp — GPS 일자 매핑 + 웨이포인트 추종 + AEB
//                      ★금색차 kasa 이식판★
//
// ⚠️ ★이 노드는 white1 의 driving.py 와 기능이 정면으로 겹친다★
//    driving.py 는 실차 로그 열댓 번으로 튜닝된 것이고(코너 선행제동·CTE 적분·
//    크립 재출발·GPS 품질 판정·헤딩 초기화), 이 노드는 스탠리 한 겹이다.
//    ★둘을 동시에 띄우지 말 것★ — /cmd_vel_raw 발행자가 겹친다.
//    실주행은 white1 을 쓰고, 이 노드는 ‘라이다 AEB 를 GPS 주행에 물렸을 때
//    어떻게 되는가’를 시험하는 용도로 둔다.
//
// 입력 :
//   /gps_fused   (Float64MultiArray)  white1 gps 노드. ★배열 규약이 1/5카의
//                /ego_state 와 완전히 다르다★ — cbEgo() 주석 참고
//   /mapping_cmd (Bool)               구 white prompt 와 같은 계약
//   /drive_cmd   (String)             파일명 | LAST | STOP
//
// 상시 AEB :
//   /cone_lidar_node/stop_signal
//   /cone_lidar_node/obstacle_distance
//
// 출력 : ★전부 KasaActuator 경유★ (lidar/kasa_units.hpp)
//   /cmd_vel_raw   linear.x = ★펄스 정수★, angular.z = ★pot 지령 − 좌 / + 우★
//   /control_state 구동 허용
//   /brake_level   ★리니어 0/1/2 — 이식에서 신설★ 펄스 0 은 코스트일 뿐이다
//
// 매핑은 최소자승 ENU 직선을 피팅해 원본 CSV 와 *_straight.csv 를 함께 쓴다.
// 저장 위치 기본값은 white1 패키지의 gps_data 다(paths.py 와 같은 폴더).
// ============================================================================

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "lidar/gps_path.hpp"
#include "lidar/kasa_units.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

/// white1 패키지의 gps_data 폴더를 되찾는다. ★paths.py 와 같은 방법이다★ —
/// colcon 을 --symlink-install 로 빌드하면 share/white1/package.xml 이 소스
/// 트리를 가리키는 심볼릭 링크라, realpath 로 소스 위치를 되찾을 수 있다.
/// 못 찾으면 빈 문자열(→ defaultGpsDataDir 의 다음 순위로 내려간다).
std::string white1DataDir() {
  try {
    const auto share = ament_index_cpp::get_package_share_directory("white1");
    const std::filesystem::path pkg_xml =
        std::filesystem::path(share) / "package.xml";
    std::error_code ec;
    const auto real = std::filesystem::canonical(pkg_xml, ec);
    const std::filesystem::path root =
        ec ? std::filesystem::path(share) : real.parent_path();
    const auto dir = root / "gps_data";
    if (std::filesystem::exists(dir)) return dir.string();
  } catch (const std::exception&) {
    // white1 이 설치돼 있지 않은 환경(단위시험 등) — 조용히 다음 순위로
  }
  return {};
}

}  // namespace

class DriveGpsNode : public rclcpp::Node {
public:
  DriveGpsNode() : Node("drive_gps_node") {
    this->declare_parameter<std::string>("data_dir",
                                        lidar::defaultGpsDataDir(white1DataDir()));
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_raw");
    // ★/ego_state 가 아니라 /gps_fused 다★ white1 에는 /ego_state 도 있지만
    //   배열이 [x, y, heading, enc_pulse, wp_idx, n_wp, fix_ok] 라 위경도가 없다.
    //   게다가 그 토픽의 발행자는 driving.py 이고, 이 노드를 쓰는 상황은
    //   driving.py 를 안 띄우는 상황이라 아예 오지도 않는다.
    this->declare_parameter<std::string>("ego_state_topic", "/gps_fused");
    this->declare_parameter<std::string>("mapping_cmd_topic", "/mapping_cmd");
    this->declare_parameter<std::string>("drive_cmd_topic", "/drive_cmd");
    this->declare_parameter<std::string>(
        "aeb_stop_signal_topic", "/cone_lidar_node/stop_signal");
    this->declare_parameter<std::string>(
        "aeb_obstacle_distance_topic", "/cone_lidar_node/obstacle_distance");

    this->declare_parameter<double>("map_record_hz", 5.0);
    this->declare_parameter<double>("control_hz", 20.0);
    this->declare_parameter<double>("waypoint_spacing", 0.25);
    this->declare_parameter<double>("min_map_length", 3.0);
    this->declare_parameter<double>("max_map_rms", 0.80);

    // ★기본 2펄스 = 1.768 m/s = 6.4 km/h (지시, 2026-08-25)★
    //   지령은 정수 펄스라 실효 선택지가 {0,1,2,3,4} 뿐이다. 4펄스는 정지 상태
    //   재출발에서 A보드 적분이 동결되므로 피한다(kasa_units.hpp 2절).
    this->declare_parameter<double>("linear_speed", lidar::kasa::pulseToMs(2));
    this->declare_parameter<double>("min_speed", 0.6);
    // ★금색차 실측 1.25 m★ (kasa_units.hpp 가 정본)
    this->declare_parameter<double>("wheelbase", lidar::kasa::WHEELBASE_M);
    this->declare_parameter<double>("k_stanley", 0.7);
    // ★도로휠각 상한 [rad]★ 0.366(=21°, 1/5카) → 금색차 0.553(=31.7°).
    //   pot ±40° 는 도로휠각이 아니다 — 환산은 KasaActuator 가 한다.
    this->declare_parameter<double>("max_angular_speed",
                                    lidar::kasa::STEER_ROAD_MAX_RAD);
    this->declare_parameter<double>("max_angular_step", 0.12);
    this->declare_parameter<double>("stanley_psi_weight", 0.85);
    this->declare_parameter<double>("lookahead_gain", 0.6);
    // ★포화 없는 선행거리 문턱 = 2·최소회전반경 = 4.04 m★ 1/5카의 1.5 는 그
    //   문턱의 37% 라, 금색차에서 그대로 쓰면 조향이 포화한다(kasa_units.hpp 1절).
    this->declare_parameter<double>("lookahead_min",
                                    lidar::kasa::LFD_NO_SATURATE_M);
    this->declare_parameter<double>("lookahead_max", 6.0);

    this->declare_parameter<double>("approach_start_dist", 8.0);
    this->declare_parameter<double>("final_stop_dist", 0.60);
    this->declare_parameter<double>("cte_abort_m", 3.0);
    this->declare_parameter<double>("ego_timeout", 0.60);
    this->declare_parameter<bool>("stop_at_end", true);
    this->declare_parameter<bool>("fit_straight_on_load", true);

    this->declare_parameter<bool>("listen_to_aeb_stop_signal", true);
    this->declare_parameter<int>("aeb_confirm_frames", 2);
    this->declare_parameter<bool>("aeb_use_distance_brake", true);
    this->declare_parameter<double>("aeb_brake_start_distance", 8.5);
    this->declare_parameter<double>("aeb_brake_end_distance", 1.5);
    this->declare_parameter<double>("aeb_max_decel", 5.0);

    // ══ ★리니어 제동 [금색차 이식 신설]★ ══════════════════════════════
    //  펄스 0 은 코스트(0.41 m/s²)일 뿐이다. 종점·AEB 에서 실제로 서려면
    //  리니어를 물어야 한다. 규칙과 실측값은 kasa_units.hpp 3절 / BRAKING.md.
    this->declare_parameter<bool>("brake_enable", true);
    this->declare_parameter<double>("brake_margin_m", 1.0);
    this->declare_parameter<double>("brake_release_k", 1.5);

    this->declare_parameter<bool>("publish_debug", true);

    loadParams();

    std::filesystem::create_directories(data_dir_);

    const auto qos = rclcpp::QoS(10);
    // ★출력은 전부 KasaActuator 를 거친다★ /cmd_vel_raw · /control_state ·
    //   /brake_level 발행과 D5·E-STOP 게이트가 전부 그 안에 있다.
    {
      const auto topic = this->get_parameter("cmd_vel_topic").as_string();
      if (topic != "/cmd_vel_raw" && !this->has_parameter("kasa.cmd_vel_topic")) {
        this->declare_parameter<std::string>("kasa.cmd_vel_topic", topic);
      }
    }
    actuator_ = std::make_unique<lidar::kasa::KasaActuator>(this);
    status_pub_ =
        this->create_publisher<std_msgs::msg::String>("/drive_status", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("~/gps_path", 5);
    remaining_pub_ =
        this->create_publisher<std_msgs::msg::Float32>("~/remaining", 5);
    if (publish_debug_) {
      target_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
          "~/steering_target", 5);
      marker_pub_ =
          this->create_publisher<visualization_msgs::msg::MarkerArray>(
              "~/debug_markers", 5);
    }

    ego_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        this->get_parameter("ego_state_topic").as_string(), qos,
        std::bind(&DriveGpsNode::cbEgo, this, std::placeholders::_1));
    map_cmd_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        this->get_parameter("mapping_cmd_topic").as_string(), qos,
        std::bind(&DriveGpsNode::cbMappingCmd, this, std::placeholders::_1));
    drive_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
        this->get_parameter("drive_cmd_topic").as_string(), qos,
        std::bind(&DriveGpsNode::cbDriveCmd, this, std::placeholders::_1));

    if (listen_to_aeb_) {
      aeb_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
          this->get_parameter("aeb_stop_signal_topic").as_string(), qos,
          std::bind(&DriveGpsNode::cbAebStop, this, std::placeholders::_1));
      aeb_dist_sub_ = this->create_subscription<std_msgs::msg::Float32>(
          this->get_parameter("aeb_obstacle_distance_topic").as_string(), qos,
          std::bind(&DriveGpsNode::cbAebDist, this, std::placeholders::_1));
    }

    const double map_hz = std::max(1.0, map_record_hz_);
    const double ctrl_hz = std::max(5.0, control_hz_);
    map_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / map_hz),
        std::bind(&DriveGpsNode::recordLoop, this));
    ctrl_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / ctrl_hz),
        std::bind(&DriveGpsNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(),
                "drive_gps: map/drive on white /ego_state + cone_lidar AEB. "
                "data_dir=%s speed=%.2f m/s (%.1f km/h) L=%.2fm spacing=%.2fm",
                data_dir_.c_str(), linear_speed_, linear_speed_ * 3.6,
                wheelbase_, waypoint_spacing_);
    RCLCPP_INFO(this->get_logger(),
                "cmds: /mapping_cmd Bool, /drive_cmd String "
                "(filename | LAST | STOP). Do NOT run white driving alongside.");
  }

private:
  enum class Mode { Idle, Mapping, Driving };

  void loadParams() {
    data_dir_ = this->get_parameter("data_dir").as_string();
    this->get_parameter("map_record_hz", map_record_hz_);
    this->get_parameter("control_hz", control_hz_);
    this->get_parameter("waypoint_spacing", waypoint_spacing_);
    this->get_parameter("min_map_length", min_map_length_);
    this->get_parameter("max_map_rms", max_map_rms_);
    this->get_parameter("linear_speed", linear_speed_);
    this->get_parameter("min_speed", min_speed_);
    this->get_parameter("wheelbase", wheelbase_);
    this->get_parameter("k_stanley", k_stanley_);
    this->get_parameter("max_angular_speed", max_angular_speed_);
    this->get_parameter("max_angular_step", max_angular_step_);
    this->get_parameter("stanley_psi_weight", stanley_psi_weight_);
    this->get_parameter("lookahead_gain", lookahead_gain_);
    this->get_parameter("lookahead_min", lookahead_min_);
    this->get_parameter("lookahead_max", lookahead_max_);
    this->get_parameter("approach_start_dist", approach_start_dist_);
    this->get_parameter("final_stop_dist", final_stop_dist_);
    this->get_parameter("cte_abort_m", cte_abort_m_);
    this->get_parameter("ego_timeout", ego_timeout_);
    this->get_parameter("stop_at_end", stop_at_end_);
    this->get_parameter("fit_straight_on_load", fit_straight_on_load_);
    this->get_parameter("listen_to_aeb_stop_signal", listen_to_aeb_);
    this->get_parameter("aeb_confirm_frames", aeb_confirm_frames_);
    this->get_parameter("aeb_use_distance_brake", aeb_use_distance_brake_);
    this->get_parameter("aeb_brake_start_distance", aeb_brake_start_distance_);
    this->get_parameter("aeb_brake_end_distance", aeb_brake_end_distance_);
    this->get_parameter("aeb_max_decel", aeb_max_decel_);
    this->get_parameter("publish_debug", publish_debug_);
    this->get_parameter("brake_enable", brake_enable_);
    this->get_parameter("brake_margin_m", brake_margin_m_);
    this->get_parameter("brake_release_k", brake_release_k_);
    ego_topic_ = this->get_parameter("ego_state_topic").as_string();
  }

  /// ★/gps_fused 배열 규약 (white1/white1/gps.py:998 이 정본)★
  ///   [0] lat      [1] lon       [2] quality(0~4) [3] sigma[m]
  ///   [4] pos_ok   [5] is_raw    [6] raw_age[s]   [7] dr_dist[m]
  ///   [8] kmh      [9] course[deg, ENU 0°=동] [10] mode
  ///   [11] reject_n [12] resid_m
  ///
  /// 1/5카의 /ego_state 는 [lat, lon, x, y, heading, speed, steer, pitch,
  /// terrain] 이었다 — ★자리가 하나도 안 겹친다★. 헤딩만은 정의가 같아서
  /// (x=동/y=북, atan2(dy,dx)) 값을 그대로 쓸 수 있다(gps_path.hpp 규약과 동일).
  ///
  /// ★pos_ok(=[4]) 가 0 이면 받지 않는다★ gps.py 가 품질 미달로 판정한
  /// 좌표다. 원본에는 이 개념이 없어 무조건 믿었다.
  void cbEgo(const std_msgs::msg::Float64MultiArray::ConstSharedPtr& msg) {
    if (msg->data.size() < 10) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "%s 배열이 %zu개다 — /gps_fused 는 13개여야 한다. 발행자를 확인할 것",
          ego_topic_.c_str(), msg->data.size());
      return;
    }
    ego_quality_  = msg->data[2];
    ego_sigma_m_  = msg->data[3];
    const bool pos_ok = msg->data[4] > 0.5;
    if (!pos_ok) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "GPS 품질 미달 (q=%.0f σ=%.2fm) — 좌표를 받지 않는다",
                           ego_quality_, ego_sigma_m_);
      return;
    }
    ego_lat_         = msg->data[0];
    ego_lon_         = msg->data[1];
    ego_speed_       = msg->data[8] / 3.6;    // km/h → m/s
    ego_heading_deg_ = msg->data[9];
    has_ego_ = std::isfinite(ego_lat_) && std::isfinite(ego_lon_);
    last_ego_time_ = this->now();
  }

  void cbMappingCmd(const std_msgs::msg::Bool::ConstSharedPtr& msg) {
    if (msg->data && mode_ != Mode::Mapping) {
      startMapping();
    } else if (!msg->data && mode_ == Mode::Mapping) {
      stopMapping();
    }
  }

  void cbDriveCmd(const std_msgs::msg::String::ConstSharedPtr& msg) {
    const std::string cmd = trim(msg->data);
    RCLCPP_INFO(this->get_logger(), "/drive_cmd '%s'", cmd.c_str());
    if (cmd == "STOP" || cmd == "stop") {
      requestStop("drive_cmd STOP");
      return;
    }
    if (mode_ == Mode::Mapping) {
      RCLCPP_WARN(this->get_logger(), "매핑 중 — 주행 명령을 무시합니다");
      return;
    }
    if (cmd == "LAST" || cmd == "LATEST") {
      if (last_straight_path_.empty() ||
          !std::filesystem::exists(last_straight_path_)) {
        last_straight_path_ = findLatestCsv(/*prefer_straight=*/true);
      }
      if (last_straight_path_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "저장된 경로가 없습니다");
        return;
      }
      loadAndDrive(last_straight_path_);
      return;
    }
    std::filesystem::path p(cmd);
    if (p.is_relative()) p = std::filesystem::path(data_dir_) / p;
    loadAndDrive(p.string());
  }

  void cbAebStop(const std_msgs::msg::Bool::ConstSharedPtr& msg) {
    aeb_raw_ = msg->data;
    aeb_raw_received_ = true;
  }

  void cbAebDist(const std_msgs::msg::Float32::ConstSharedPtr& msg) {
    aeb_distance_ = msg->data;
    aeb_distance_received_ = true;
  }

  void startMapping() {
    if (mode_ == Mode::Driving) requestStop("mapping starts");
    if (!has_ego_) {
      RCLCPP_ERROR(this->get_logger(),
                   "매핑 시작 실패: /ego_state 없음 (white gps_imu 헤딩 고정 확인)");
      return;
    }
    samples_.clear();
    const auto stamp = timestampNow();
    raw_csv_path_ = (std::filesystem::path(data_dir_) /
                     ("route_" + stamp + ".csv"))
                        .string();
    raw_csv_.open(raw_csv_path_, std::ios::out | std::ios::trunc);
    if (!raw_csv_) {
      RCLCPP_ERROR(this->get_logger(), "CSV 열기 실패: %s", raw_csv_path_.c_str());
      return;
    }
    raw_csv_ << "latitude,longitude,heading,speed,steer,direction,pitch,terrain\n";
    flush_counter_ = 0;
    mode_ = Mode::Mapping;
    setControlState(false);
    publishStatus("MAPPING " + raw_csv_path_);
    RCLCPP_INFO(this->get_logger(), "🗺️ 매핑 시작: %s", raw_csv_path_.c_str());
  }

  void stopMapping() {
    if (mode_ != Mode::Mapping) return;
    mode_ = Mode::Idle;
    if (raw_csv_.is_open()) {
      raw_csv_.flush();
      raw_csv_.close();
    }
    RCLCPP_INFO(this->get_logger(), "✅ 원본 저장: %s (%zu pts)",
                raw_csv_path_.c_str(), samples_.size());

    if (samples_.size() < 2) {
      RCLCPP_ERROR(this->get_logger(), "포인트가 부족해 일자 피팅을 건너뜁니다");
      publishStatus("MAP_FAIL too_few_points");
      return;
    }

    const double lat0 = samples_.front().lat;
    const double lon0 = samples_.front().lon;
    std::vector<lidar::EnuPoint> enu;
    enu.reserve(samples_.size());
    for (const auto& s : samples_)
      enu.push_back(lidar::latlonToEnu(s.lat, s.lon, lat0, lon0));

    auto fit = lidar::fitStraightLine(enu, lat0, lon0, min_map_length_);
    if (!fit.valid) {
      RCLCPP_ERROR(this->get_logger(),
                   "일자 피팅 실패 (길이<%.1fm). 더 길게 매핑하세요.",
                   min_map_length_);
      publishStatus("MAP_FAIL short");
      return;
    }
    if (fit.rms > max_map_rms_) {
      RCLCPP_WARN(this->get_logger(),
                  "일자 RMS=%.2fm > %.2fm — 직진 매핑이 흔들렸습니다. "
                  "경로는 저장하지만 재매핑을 권장합니다.",
                  fit.rms, max_map_rms_);
    }

    const auto wps = lidar::samplesFromFit(fit, waypoint_spacing_);
    last_straight_path_ = raw_csv_path_;
    const auto pos = last_straight_path_.rfind(".csv");
    if (pos != std::string::npos)
      last_straight_path_.insert(pos, "_straight");
    if (!writeCsv(last_straight_path_, wps)) {
      RCLCPP_ERROR(this->get_logger(), "일자 CSV 저장 실패");
      publishStatus("MAP_FAIL write");
      return;
    }

    fit_ = fit;
    has_fit_ = true;
    publishPath(fit_);
    publishStatus("MAPPED " + last_straight_path_);
    RCLCPP_INFO(this->get_logger(),
                "📏 일자 경로: L=%.1fm heading=%.1f° rms=%.3fm n=%zu → %s",
                fit.length, fit.heading_deg, fit.rms, wps.size(),
                last_straight_path_.c_str());
  }

  void recordLoop() {
    if (mode_ != Mode::Mapping || !has_ego_ || !raw_csv_.is_open()) return;
    lidar::GpsSample s;
    s.lat = ego_lat_;
    s.lon = ego_lon_;
    s.heading_deg = ego_heading_deg_;
    s.speed = ego_speed_;
    s.steer = ego_steer_;
    s.direction = (ego_speed_ < -0.05) ? -1 : 1;
    s.pitch = ego_pitch_;
    s.terrain = ego_terrain_;
    samples_.push_back(s);
    raw_csv_ << std::fixed << std::setprecision(8) << s.lat << "," << s.lon
             << "," << std::setprecision(2) << s.heading_deg << ","
             << std::setprecision(4) << s.speed << "," << s.steer << ","
             << s.direction << "," << s.pitch << "," << s.terrain << "\n";
    if (++flush_counter_ >= 10) {
      raw_csv_.flush();
      flush_counter_ = 0;
    }
  }

  void loadAndDrive(const std::string& path) {
    std::vector<lidar::GpsSample> wps;
    if (!readCsv(path, wps)) {
      RCLCPP_ERROR(this->get_logger(), "경로 로드 실패: %s", path.c_str());
      return;
    }
    const double lat0 = wps.front().lat;
    const double lon0 = wps.front().lon;
    std::vector<lidar::EnuPoint> enu;
    enu.reserve(wps.size());
    for (const auto& s : wps)
      enu.push_back(lidar::latlonToEnu(s.lat, s.lon, lat0, lon0));

    lidar::StraightFit fit;
    if (fit_straight_on_load_) {
      fit = lidar::fitStraightLine(enu, lat0, lon0, min_map_length_);
    } else if (has_fit_ && last_straight_path_ == path) {
      fit = fit_;
    }
    if (!fit.valid && enu.size() >= 2) {
      // Chord fallback: first → last (still a 일자)
      fit.ux = enu.back().x - enu.front().x;
      fit.uy = enu.back().y - enu.front().y;
      const double n = std::hypot(fit.ux, fit.uy);
      if (n >= min_map_length_) {
        fit.ux /= n;
        fit.uy /= n;
        fit.x0 = enu.front().x;
        fit.y0 = enu.front().y;
        fit.length = n;
        fit.lat0 = lat0;
        fit.lon0 = lon0;
        fit.heading_deg =
            lidar::normalizeDeg(std::atan2(fit.uy, fit.ux) * 180.0 / M_PI);
        fit.valid = true;
      }
    }
    if (!fit.valid) {
      RCLCPP_ERROR(this->get_logger(), "주행 경로를 일자로 만들지 못했습니다");
      return;
    }

    fit_ = fit;
    has_fit_ = true;
    last_straight_path_ = path;
    mode_ = Mode::Driving;
    arrived_ = false;
    has_prev_angular_ = false;
    has_prev_speed_ = false;
    prev_cmd_speed_ = 0.0;
    aeb_true_streak_ = 0;
    aeb_stop_active_ = false;
    setControlState(true);
    publishPath(fit_);
    publishStatus("DRIVING " + path);
    RCLCPP_INFO(this->get_logger(),
                "🚗 GPS 일자 주행 시작 L=%.1fm heading=%.1f° file=%s",
                fit_.length, fit_.heading_deg, path.c_str());
  }

  void requestStop(const std::string& why) {
    if (mode_ == Mode::Mapping) {
      stopMapping();
    }
    mode_ = Mode::Idle;
    arrived_ = true;
    // ★정지 지시는 리니어 2단이다 [금색차 이식]★ 원본은 속도지령 0 이 전부였고
    //   그것으로 1/5카는 섰다. 금색차에서 펄스 0 은 코스트라 4펄스에서 15.2 m,
    //   2펄스에서도 3.8 m 를 더 굴러간다. 'STOP' 이 그런 뜻일 수는 없다.
    //   (white1 driving.py 도 DRIVE_DONE 에서 곧바로 BRAKE_FULL 을 문다)
    if (brake_enable_) actuator_->brake(lidar::kasa::BRAKE_FULL);
    setControlState(false);
    publishCmd(0.0, 0.0);
    publishStatus("STOP " + why);
    RCLCPP_WARN(this->get_logger(), "🛑 정지: %s", why.c_str());
  }

  void controlLoop() {
    if (mode_ != Mode::Driving || !has_fit_) return;

    const auto now = this->now();
    if (!has_ego_ || (now - last_ego_time_).seconds() > ego_timeout_) {
      requestStop("ego_state timeout");
      return;
    }

    const lidar::EnuPoint pos =
        lidar::latlonToEnu(ego_lat_, ego_lon_, fit_.lat0, fit_.lon0);
    const double heading_rad = ego_heading_deg_ * M_PI / 180.0;
    const double ld = std::clamp(lookahead_gain_ * std::max(linear_speed_, 0.5),
                                 lookahead_min_, lookahead_max_);
    const auto st = lidar::trackStraight(fit_, pos, heading_rad, ld);

    if (std::fabs(st.e_y) > cte_abort_m_) {
      requestStop("CTE abort " + std::to_string(st.e_y) + "m");
      return;
    }

    // AEB confirm
    if (listen_to_aeb_ && aeb_raw_)
      ++aeb_true_streak_;
    else
      aeb_true_streak_ = 0;
    const bool aeb_confirmed =
        listen_to_aeb_ && aeb_raw_received_ &&
        aeb_true_streak_ >= std::max(aeb_confirm_frames_, 1);

    double aeb_scale = 1.0;
    if (aeb_use_distance_brake_ && aeb_distance_received_ &&
        std::isfinite(aeb_distance_)) {
      aeb_scale = lidar::distanceSpeedScale(
          aeb_distance_, aeb_brake_start_distance_, aeb_brake_end_distance_);
      if (!aeb_confirmed && aeb_scale > 0.4) aeb_scale = 1.0;
      if (!aeb_confirmed && aeb_scale <= 0.4)
        aeb_scale = std::max(aeb_scale, 0.35);
    } else if (aeb_confirmed) {
      aeb_scale = 0.0;
    }
    if (aeb_confirmed && !aeb_stop_active_) {
      RCLCPP_WARN(this->get_logger(),
                  "🚨 [AEB] GPS 주행 중 제동 d=%.2f",
                  aeb_distance_received_ ? aeb_distance_ : -1.0);
    }
    aeb_stop_active_ = aeb_confirmed;

    double end_scale = 1.0;
    if (stop_at_end_) {
      end_scale = lidar::distanceSpeedScale(st.remaining, approach_start_dist_,
                                             final_stop_dist_);
    }

    double angular = lidar::stanleyGps(st.e_y, st.heading_err, linear_speed_,
                                        k_stanley_, max_angular_speed_,
                                        stanley_psi_weight_);
    if (has_prev_angular_) {
      const double step =
          std::clamp(angular - prev_angular_z_, -max_angular_step_,
                     max_angular_step_);
      angular = prev_angular_z_ + step;
    }
    prev_angular_z_ = angular;
    has_prev_angular_ = true;

    double desired_v = linear_speed_ * std::min(aeb_scale, end_scale);
    if (desired_v > 0.0 && desired_v < min_speed_ &&
        end_scale > 0.05 && aeb_scale > 0.05) {
      desired_v = min_speed_;
    }
    if (stop_at_end_ && st.remaining <= final_stop_dist_) desired_v = 0.0;
    if (aeb_scale <= 0.0) desired_v = 0.0;

    const double dt = 1.0 / std::max(control_hz_, 5.0);
    if (!has_prev_speed_) {
      prev_cmd_speed_ = desired_v;
      has_prev_speed_ = true;
    }
    const double max_drop = std::max(aeb_max_decel_, 0.5) * dt;
    const double max_rise = 2.0 * dt;
    double v = desired_v;
    if (v < prev_cmd_speed_ - max_drop) v = prev_cmd_speed_ - max_drop;
    if (v > prev_cmd_speed_ + max_rise) v = prev_cmd_speed_ + max_rise;
    if (v < 0.0) v = 0.0;
    prev_cmd_speed_ = v;

    double steer_deg = angular * 180.0 / M_PI;
    if (v < 0.15 && (aeb_stop_active_ || st.remaining <= final_stop_dist_))
      steer_deg = 0.0;
    // ══ ★리니어 제동 판정 [금색차 이식 신설]★ ═══════════════════════════
    //  원본은 aeb_scale·end_scale 로 ★지령 속도만★ 깎았다. 금색차에서 그것은
    //  '엑셀을 뗀다'까지이고, 실제로 세우는 것은 리니어다.
    //   · 감속 구간(scale < 1)            → ★펄스 0 + 1단★
    //   · 정지 확정(AEB 확정 / 종점 도달) → ★2단★
    //  ★체결은 기하로 / 해제는 실측으로★ 여기서 '실측'은 /gps_fused 의 속도다.
    double v_out = v;
    if (brake_enable_) {
      const double v_meas =
          (ego_speed_ > 0.0 && std::isfinite(ego_speed_)) ? ego_speed_ : v;
      // '설 지점까지 거리' — AEB 장애물과 종점 중 가까운 쪽
      double d_stop = st.remaining - final_stop_dist_;
      if (aeb_distance_received_ && std::isfinite(aeb_distance_)) {
        d_stop = std::min(d_stop, aeb_distance_ - aeb_brake_end_distance_);
      }
      const double need1 = lidar::kasa::stopDist(
          v_meas, lidar::kasa::A_BRAKE1_MS2, lidar::kasa::BRAKE1_LAG_S)
          + brake_margin_m_;
      const double need2 = 1.2 * lidar::kasa::stopDist(
          v_meas, lidar::kasa::A_BRAKE2_MS2, lidar::kasa::BRAKE2_LAG_S)
          + brake_margin_m_;

      if (aeb_stop_active_ || d_stop <= need2) {
        // ★이미 늦었으면 1단을 건너뛰고 곧장 2단★ B보드는 진행 중인 행정이
        //   끝나야 다음 이동을 시작하므로, 같은 틱에 1→2 를 물면 리니어를 한 번
        //   왕복시키면서 2단이 그만큼 늦는다(white1 goal_approach 와 같은 결함).
        actuator_->brake(lidar::kasa::BRAKE_FULL);
      } else if (d_stop <= need1) {
        actuator_->brake(lidar::kasa::BRAKE_SOFT);
      } else if (actuator_->brakeStage() > lidar::kasa::BRAKE_OFF
                 && d_stop > brake_release_k_ * need1) {
        actuator_->releaseBrake();   // 최소 물림(0.5s)은 액추에이터가 지킨다
      }
      actuator_->keepBrake();
      // ★제동 중에는 구동을 내지 않는다★ arduino 도 brake>0 이면 REF 를 0 으로
      //   덮지만, 여기서도 명시적으로 0 을 낸다(둘이 갈라지면 안 된다).
      if (actuator_->brakeStage() > lidar::kasa::BRAKE_OFF) {
        v_out = 0.0;
        steer_deg = 0.0;   // 정지 중 바퀴를 일직선으로
      }
    }
    publishCmd(v_out, steer_deg);

    std_msgs::msg::Float32 rem;
    rem.data = static_cast<float>(st.remaining);
    remaining_pub_->publish(rem);

    if (stop_at_end_ && st.remaining <= final_stop_dist_ && v < 0.12) {
      requestStop("arrived");
      return;
    }

    if (publish_debug_) publishDebug(st);

    if (log_counter_++ % 20 == 0) {
      RCLCPP_INFO(this->get_logger(),
                  "[GPS] s=%.1f/%.1f rem=%.1f cte=%.2f herr=%.1fdeg "
                  "v=%.2f aeb=%.2f stop=%d",
                  st.s, fit_.length, st.remaining, st.e_y,
                  st.heading_err * 180.0 / M_PI, v, aeb_scale,
                  aeb_stop_active_ ? 1 : 0);
    }
  }

  /// ★이 노드의 유일한 출력 지점★ `steer_deg` 는 ★도로휠각, + = 좌★ 다.
  /// m/s → 펄스, 도로휠각 → pot 지령, + 좌 → − 좌 반전이 전부 drive() 안에서
  /// 한 번에 일어난다 — ★부호가 뒤집히는 곳은 코드 전체에서 거기 한 줄뿐이다★
  void publishCmd(double v_ms, double steer_deg) {
    if (!actuator_->ready()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "⛔ 구동 불가 — %s", actuator_->blockReason());
      actuator_->hold(/*control_enable=*/false);
      return;
    }
    actuator_->drive(v_ms, steer_deg, /*control_enable=*/control_on_);
  }

  /// 구동 허용 플래그. ★실제 발행은 매 틱 drive()/hold() 가 한다★
  /// 원본은 여기서 엣지에 한 번만 발행했는데, QoS 가 volatile 이라
  /// arduino 노드가 나중에 뜨면 그 True 를 영영 못 받는다(white1 은 매 틱 낸다).
  void setControlState(bool on) {
    control_on_ = on;
    if (!on) actuator_->hold(false);
  }

  void publishStatus(const std::string& s) {
    std_msgs::msg::String msg;
    msg.data = s;
    status_pub_->publish(msg);
  }

  void publishPath(const lidar::StraightFit& fit) {
    nav_msgs::msg::Path path;
    path.header.stamp = this->now();
    path.header.frame_id = "gps_enu";
    const auto pts = lidar::resampleLine(fit, std::max(waypoint_spacing_, 0.5));
    path.poses.reserve(pts.size());
    for (const auto& p : pts) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    path_pub_->publish(path);
  }

  void publishDebug(const lidar::TrackState& st) {
    geometry_msgs::msg::PointStamped tgt;
    tgt.header.stamp = this->now();
    tgt.header.frame_id = "gps_enu";
    tgt.point.x = st.lookahead.x;
    tgt.point.y = st.lookahead.y;
    target_pub_->publish(tgt);

    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.header = tgt.header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    visualization_msgs::msg::Marker line;
    line.header = tgt.header;
    line.ns = "gps_line";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.08;
    line.color.g = 1.0f;
    line.color.a = 0.9f;
    line.pose.orientation.w = 1.0;
    geometry_msgs::msg::Point a, b;
    a.x = fit_.x0;
    a.y = fit_.y0;
    b.x = fit_.x0 + fit_.length * fit_.ux;
    b.y = fit_.y0 + fit_.length * fit_.uy;
    line.points.push_back(a);
    line.points.push_back(b);
    arr.markers.push_back(line);

    visualization_msgs::msg::Marker ego;
    ego.header = tgt.header;
    ego.ns = "ego";
    ego.id = 1;
    ego.type = visualization_msgs::msg::Marker::SPHERE;
    ego.action = visualization_msgs::msg::Marker::ADD;
    ego.scale.x = ego.scale.y = ego.scale.z = 0.35;
    ego.color.b = 1.0f;
    ego.color.a = 1.0f;
    const auto pos =
        lidar::latlonToEnu(ego_lat_, ego_lon_, fit_.lat0, fit_.lon0);
    ego.pose.position.x = pos.x;
    ego.pose.position.y = pos.y;
    ego.pose.orientation.w = 1.0;
    arr.markers.push_back(ego);
    marker_pub_->publish(arr);
  }

  static std::string trim(std::string s) {
    const auto a = s.find_first_not_of(" \t\r\n");
    const auto b = s.find_last_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    return s.substr(a, b - a + 1);
  }

  static std::string timestampNow() {
    const auto t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
  }

  static std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> out;
    std::string cur;
    std::istringstream ss(line);
    while (std::getline(ss, cur, ',')) out.push_back(cur);
    return out;
  }

  bool writeCsv(const std::string& path,
                const std::vector<lidar::GpsSample>& wps) {
    std::ofstream f(path);
    if (!f) return false;
    f << "latitude,longitude,heading,speed,steer,direction,pitch,terrain\n";
    f << std::fixed;
    for (const auto& s : wps) {
      f << std::setprecision(8) << s.lat << "," << s.lon << ","
        << std::setprecision(2) << s.heading_deg << "," << s.speed << ","
        << s.steer << "," << s.direction << "," << s.pitch << "," << s.terrain
        << "\n";
    }
    return true;
  }

  bool readCsv(const std::string& path, std::vector<lidar::GpsSample>& out) {
    std::ifstream f(path);
    if (!f) return false;
    std::string header;
    if (!std::getline(f, header)) return false;
    auto cols = splitCsv(header);
    int i_lat = -1, i_lon = -1, i_hdg = -1, i_dir = -1;
    for (int i = 0; i < static_cast<int>(cols.size()); ++i) {
      if (cols[i] == "latitude") i_lat = i;
      else if (cols[i] == "longitude") i_lon = i;
      else if (cols[i] == "heading") i_hdg = i;
      else if (cols[i] == "direction") i_dir = i;
    }
    if (i_lat < 0 || i_lon < 0) return false;
    std::string line;
    while (std::getline(f, line)) {
      if (line.empty()) continue;
      auto v = splitCsv(line);
      if (static_cast<int>(v.size()) <= std::max(i_lat, i_lon)) continue;
      lidar::GpsSample s;
      try {
        s.lat = std::stod(v[i_lat]);
        s.lon = std::stod(v[i_lon]);
        if (i_hdg >= 0 && i_hdg < static_cast<int>(v.size()))
          s.heading_deg = std::stod(v[i_hdg]);
        if (i_dir >= 0 && i_dir < static_cast<int>(v.size()))
          s.direction = static_cast<int>(std::stod(v[i_dir]));
      } catch (...) {
        continue;
      }
      out.push_back(s);
    }
    return !out.empty();
  }

  std::string findLatestCsv(bool prefer_straight) {
    std::string best;
    std::filesystem::file_time_type best_t;
    bool have = false;
    std::error_code ec;
    for (const auto& e :
         std::filesystem::directory_iterator(data_dir_, ec)) {
      if (!e.is_regular_file()) continue;
      const auto name = e.path().filename().string();
      if (name.size() < 4 || name.substr(name.size() - 4) != ".csv") continue;
      if (prefer_straight && name.find("_straight") == std::string::npos)
        continue;
      const auto t = e.last_write_time();
      if (!have || t > best_t) {
        best = e.path().string();
        best_t = t;
        have = true;
      }
    }
    if (have) return best;
    if (prefer_straight) return findLatestCsv(false);
    return {};
  }

  // params
  std::string data_dir_;
  double map_record_hz_ = 5.0;
  double control_hz_ = 20.0;
  double waypoint_spacing_ = 0.25;
  double min_map_length_ = 3.0;
  double max_map_rms_ = 0.80;
  double linear_speed_ = 2.0;
  double min_speed_ = 0.6;
  double wheelbase_ = 0.75;
  double k_stanley_ = 0.7;
  double max_angular_speed_ = 0.366;
  double max_angular_step_ = 0.12;
  double stanley_psi_weight_ = 0.85;
  double lookahead_gain_ = 0.6;
  double lookahead_min_ = 1.5;
  double lookahead_max_ = 6.0;
  double approach_start_dist_ = 8.0;
  double final_stop_dist_ = 0.60;
  double cte_abort_m_ = 3.0;
  double ego_timeout_ = 0.60;
  bool stop_at_end_ = true;
  bool fit_straight_on_load_ = true;
  bool listen_to_aeb_ = true;
  int aeb_confirm_frames_ = 2;
  bool aeb_use_distance_brake_ = true;
  double aeb_brake_start_distance_ = 8.5;
  double aeb_brake_end_distance_ = 1.5;
  double aeb_max_decel_ = 5.0;
  bool publish_debug_ = true;

  Mode mode_ = Mode::Idle;
  bool has_ego_ = false;
  double ego_lat_ = 0, ego_lon_ = 0, ego_heading_deg_ = 0;
  double ego_speed_ = 0, ego_steer_ = 0, ego_pitch_ = 0, ego_terrain_ = 0;
  rclcpp::Time last_ego_time_{0, 0, RCL_ROS_TIME};

  std::vector<lidar::GpsSample> samples_;
  std::ofstream raw_csv_;
  std::string raw_csv_path_;
  std::string last_straight_path_;
  int flush_counter_ = 0;

  lidar::StraightFit fit_;
  bool has_fit_ = false;
  bool arrived_ = false;

  bool aeb_raw_ = false;
  bool aeb_raw_received_ = false;
  int aeb_true_streak_ = 0;
  bool aeb_distance_received_ = false;
  float aeb_distance_ = std::numeric_limits<float>::infinity();
  bool aeb_stop_active_ = false;

  bool has_prev_angular_ = false;
  double prev_angular_z_ = 0.0;
  bool has_prev_speed_ = false;
  double prev_cmd_speed_ = 0.0;
  int log_counter_ = 0;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ego_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr map_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr drive_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr aeb_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr aeb_dist_sub_;
  std::unique_ptr<lidar::kasa::KasaActuator> actuator_;
  std::string ego_topic_;
  double ego_quality_ = 0.0;
  double ego_sigma_m_ = 0.0;
  bool   brake_enable_ = true;
  double brake_margin_m_ = 1.0;
  double brake_release_k_ = 1.5;
  bool   control_on_ = false;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr remaining_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr map_timer_;
  rclcpp::TimerBase::SharedPtr ctrl_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<DriveGpsNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("drive_gps_node"), "fatal: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
