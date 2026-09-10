#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "mppi_local_planner/ego_costmap.hpp"
#include "mppi_local_planner/kasa_units.hpp"
#include "mppi_local_planner/mppi_controller.hpp"
#include "mppi_local_planner/vehicle_model.hpp"

using namespace std::chrono_literals;

namespace mppi_local_planner
{

namespace
{
geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny, cosy);
}

bool orientationValid(const sensor_msgs::msg::Imu & msg)
{
  const auto & q = msg.orientation;
  const double n2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  if (!(n2 > 0.25 && n2 < 4.0)) {
    return false;
  }
  // ROS: covariance[0] < 0 이면 orientation 미제공
  if (msg.orientation_covariance[0] < 0.0) {
    return false;
  }
  return true;
}
}  // namespace

class MPPILocalPlannerNode : public rclcpp::Node
{
public:
  MPPILocalPlannerNode()
  : Node("mppi_local_planner_node")
  {
    declareParameters();
    readParameters();

    // Separate callback groups so LiDAR processing and the control timer can
    // run concurrently under a MultiThreadedExecutor.
    cloud_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    imu_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    costmap_ = std::make_unique<EgoCostmap>(costmap_params_);
    controller_ = std::make_unique<MPPIController>(mppi_params_, vehicle_params_);
    if (cmd_vel_topic_ != "/cmd_vel_raw" && !has_parameter("kasa.cmd_vel_topic")) {
      declare_parameter<std::string>("kasa.cmd_vel_topic", cmd_vel_topic_);
    }
    //  ★kasa.max_pulse 도 cruise_pulse 에서 유도한다 [2026-09-07]★
    //  이 값이 ★실제로 차를 묶는 상한★ 이다(msToPulse 의 clamp). KasaActuator 가
    //  스스로 declare 하므로 ★그보다 먼저★ 여기서 박아 두어야 이긴다.
    //  yaml/런치가 명시했으면 그쪽을 존중한다(has_parameter 검사).
    if (cruise_pulse_ > 0 && !has_parameter("kasa.max_pulse")) {
      declare_parameter<int>("kasa.max_pulse", cruise_pulse_);
    }
    actuator_ = std::make_unique<lidar::kasa::KasaActuator>(this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    rclcpp::SubscriptionOptions cloud_opts;
    cloud_opts.callback_group = cloud_cb_group_;
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MPPILocalPlannerNode::cloudCallback, this, std::placeholders::_1),
      cloud_opts);

    rclcpp::SubscriptionOptions imu_opts;
    imu_opts.callback_group = imu_cb_group_;
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MPPILocalPlannerNode::imuCallback, this, std::placeholders::_1),
      imu_opts);

    // ════════════════════════════════════════════════════════════════════
    //  ★조종권 계약 (white1 driving.py 와 양방향) [2026-09-01]★
    // ════════════════════════════════════════════════════════════════════
    //  /cmd_vel_raw 는 마지막 발행자가 이기는 토픽이고, nxde/arduino.py 의
    //  cb_cmd_vel 은 ★신선도를 보지 않고 마지막 펄스를 래치한다★. 그래서 이 노드와
    //  driving.py 가 동시에 내면 20Hz 로 서로를 덮는다. 겹치지 않게 하는 것이
    //  이 계약의 전부다 — 자세한 설계는 driving.py 헤더 '라이다 구간 이양' 절.
    //
    //    구독 /lstatus      : driving → 나. 구간 문자 '0' | 'L' | 'S'
    //    발행 /lidar_active : 나 → driving. "나 살아 있다" (매 틱. ★값보다 신선도★)
    //
    //  ★허락이 없으면 이 노드는 아무것도 발행하지 않는다★ hold() 조차 부르지
    //  않는다 — 그 함수도 /cmd_vel_raw 에 0 을 실제로 발행하기 때문이다.
    //  ★[2026-09-07] /lidar_permit(Bool) → /lstatus(String) 로 대체★
    //  driving 이 경로의 terrain 을 정규화해 매 틱 낸다. 'L' 이 곧 허락이고,
    //  driving 이 조종권을 회수하면(E-STOP·GPS 두절) 경로가 그대로여도 '0' 이 온다.
    //  ★/lstatus 가 끊기면 침묵한다★ (lstatus_stale_s). driving 이 죽었을 때
    //  마지막 True 를 붙들고 계속 몰지 않기 위해서다 — 신선도가 곧 허락이다.
    //  ★실측 펄스 [2026-09-11]★ 저속 기동 보정이 볼 유일한 값이다.
    //  ★/encoder 는 좌+우 ★합★ 이다★ — 소비측이 ×0.5 로 바퀴 하나 기준으로
    //  되돌린다(white1 ENC_SUM_TO_PULSE 와 같은 규약. 한쪽만 고치지 말 것).
    //  ★중앙값 3점★ A보드 기동 블랭킹 구간에 허수 카운트가 쏟아진다(실측 중앙 16,
    //  최대 34 — 정상 4~5). 그걸 그대로 믿으면 '이미 구르고 있다' 로 읽어 킥이
    //  걸리지 않는다. white1 cb_encoder 와 같은 필터를 쓴다.
    encoder_sub_ = create_subscription<std_msgs::msg::Int32>(
      get_parameter("cmd.encoder_topic").as_string(), rclcpp::QoS(10),
      [this](const std_msgs::msg::Int32::ConstSharedPtr & m) {
        std::lock_guard<std::mutex> lk(enc_mutex_);
        enc_buf_[enc_i_ % 3] = static_cast<double>(m->data);
        ++enc_i_;
        double a = enc_buf_[0], b = enc_buf_[1], c = enc_buf_[2];
        const double med = std::max(std::min(a, b), std::min(std::max(a, b), c));
        enc_pulse_.store(med * 0.5, std::memory_order_relaxed);   // 합 → 바퀴 하나
        enc_t_.store(nowSeconds(), std::memory_order_relaxed);
      });

    // ══════════════════════════════════════════════════════════════════════
    //  ★[2026-09-11] GPS 기준선 — 추측항법을 대체한다 (사용자 지시)★
    // ══════════════════════════════════════════════════════════════════════
    //  ★왜 필요한가★ 이 노드의 odom.y 는 ★지령 속도로 적분한 추측항법★ 이고
    //  (updateOdom 참고) 그것을 바로잡을 절대 관측이 하나도 없었다. 실측에서
    //  지령 1.26 m/s / 실제 0.76 m/s — 1.65배라 9.4m 를 가며 15.6m 를 갔다고
    //  믿었다. 비용함수가 y² 와 |y|>1.2m 벽으로 중심선을 지키게 되어 있는데
    //  ★그 y 가 틀리면 벽이 서 있지 않은 것과 같다★ — 실제로 CTE 가 −0.23 →
    //  −4.54m 로 벌어졌다(ros2bag route_20260910_212627-20260910_220129).
    //
    //  white1/driving 은 매핑 경로를 들고 있으므로 그 두 값을 정확히 안다.
    //  받아서 odom.y / odom.yaw 를 ★덮어쓴다★ — 그러면 이 노드의 기준선이
    //  '인계 시점 헤딩으로 그은 가상의 직선' 에서 ★실제 매핑 중심선★ 이 된다.
    //  라바콘이 그 중심선 좌우로 번갈아 놓이므로 중심선이 정확해야 S자가 된다.
    //
    //  ★부호가 그대로 맞는다★ driving 의 CTE 는 '+ = 차가 경로 왼쪽', 이 노드의
    //  odom.y 도 '+ = 기준선 왼쪽'(y += v·sin(yaw)) 이다. yaw 도 '기준방위 대비
    //  + = 왼쪽' 이라 heading_err 과 같다. 뒤집지 않는다.
    ref_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      ref_topic_, rclcpp::QoS(10),
      [this](const std_msgs::msg::Float64MultiArray::ConstSharedPtr & m) {
        if (m->data.size() < 3) return;
        const double cte = m->data[0], herr = m->data[1];
        const bool ok = (m->data[2] > 0.5) && std::isfinite(cte) && std::isfinite(herr);
        std::lock_guard<std::mutex> lock(odom_mutex_);
        ref_valid_ = ok;
        ref_t_ = nowSeconds();
        if (!ok) return;
        ref_cte_ = cte;
        ref_herr_ = herr * M_PI / 180.0;
        ref_zone_left_ = (m->data.size() > 3) ? m->data[3]
                                              : std::numeric_limits<double>::quiet_NaN();
      });

    lstatus_sub_ = create_subscription<std_msgs::msg::String>(
      lstatus_topic_, rclcpp::QoS(10),
      [this](const std_msgs::msg::String::ConstSharedPtr & m) {
        lstatus_.store(m->data.empty() ? '0' : m->data[0], std::memory_order_relaxed);
        lstatus_stamp_.store(nowSeconds(), std::memory_order_relaxed);
      });
    active_pub_ = create_publisher<std_msgs::msg::Bool>(active_topic_, rclcpp::QoS(10));

    costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(costmap_topic_, 1);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 1);
    reference_path_pub_ = create_publisher<nav_msgs::msg::Path>(reference_path_topic_, 1);
    // ══════════════════════════════════════════════════════════════════════
    //  ★[2026-09-11] /lidar_diag — 회피를 사후에 볼 수 있게 (사용자 지시)★
    // ══════════════════════════════════════════════════════════════════════
    //  ★L 구간에서만 나간다★ 이 노드가 실제로 몰고 있을 때만 발행하므로,
    //  record.py 의 열은 자연히 L 구간에서만 채워지고 그 밖에서는 빈칸이다 —
    //  '어디서부터 라이다가 몰았나' 가 열 모양으로 바로 드러난다.
    //  ★진단 목적은 하나다 — 라바콘 사이를 잘 지났는가.★ 그래서 '기준선 대비
    //  어디에 있었나(y·yaw)' 와 '무엇을 보고 얼마나 꺾었나(장애물·조향)' 를
    //  한 배열에 담는다. 코스트맵·롤아웃은 CSV 에 담기엔 너무 크다.
    diag_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(diag_topic_, 10);

    // Use nanosecond period to avoid millisecond truncation.
    const auto period = std::chrono::duration<double>(1.0 / control_frequency_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MPPILocalPlannerNode::controlLoop, this),
      control_cb_group_);

    last_control_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "mppi_local_planner_node started (kasa 금색차) "
      "wheelbase=%.2f m, track=%.2f m, road_steer_max=%.1f deg, "
      "cruise=%.2f m/s (%.1f km/h ≈ %d pulse), ctrl=%.1f Hz, mppi.dt=%.3f s",
      vehicle_params_.wheelbase, vehicle_params_.track_width,
      vehicle_params_.max_steering_angle * 180.0 / M_PI,
      mppi_params_.desired_speed, mppi_params_.desired_speed * 3.6,
      lidar::kasa::msToPulse(mppi_params_.desired_speed, actuator_->maxPulse()),
      control_frequency_, mppi_params_.dt);
    RCLCPP_INFO(
      get_logger(),
      "LiDAR mount: height=%.2f m AGL, z_slab=[%.2f, %.2f] (sensor frame), "
      "yaw_offset=%.2f rad (flip_lidar_xy=%s)",
      sensor_height_m_, costmap_params_.ground_z_min, costmap_params_.ground_z_max,
      costmap_params_.sensor_yaw_offset,
      flip_lidar_xy_ ? "true" : "false");
    RCLCPP_INFO(
      get_logger(),
      "헤딩 IMU = '%s'  use_orientation=%s  "
      "(기본 /imu = 외장 iAHRS AHRS 쿼터니언. /ouster/imu 는 자이로만이라 드리프트가 크다)",
      imu_topic_.c_str(), imu_use_orientation_ ? "true" : "false");
    RCLCPP_INFO(
      get_logger(),
      "Ego clear (tight): occ circle r=%.2f m, rect x=[%.2f, %.2f] y_half=%.2f | "
      "cost bleed circle r=%.2f (near-cone safe)",
      costmap_params_.ego_clear_radius,
      costmap_params_.ego_clear_x_min, costmap_params_.ego_clear_x_max,
      costmap_params_.ego_clear_y_half,
      costmap_params_.ego_cost_clear_radius > 0.0 ?
        costmap_params_.ego_cost_clear_radius : costmap_params_.ego_clear_radius);

    if (std::abs(control_frequency_ * mppi_params_.dt - 1.0) > 0.15) {
      RCLCPP_WARN(
        get_logger(),
        "control_frequency (%.1f Hz) and mppi.dt (%.3f s) are not matched "
        "(product=%.3f, ideal=1.0). Warm-start shift uses elapsed time, but "
        "setting control_frequency ≈ 1/mppi.dt is recommended.",
        control_frequency_, mppi_params_.dt, control_frequency_ * mppi_params_.dt);
    }

    RCLCPP_INFO(
      get_logger(),
      "Subscribing lidar_topic='%s', imu_topic='%s' (SensorDataQoS)",
      cloud_sub_->get_topic_name(), imu_sub_->get_topic_name());
  }

  ~MPPILocalPlannerNode() override
  {
    if (actuator_) {
      actuator_->shutdown();
    }
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("lidar_topic", "/ouster/points");
    declare_parameter<std::string>("imu_topic", "/imu");
    declare_parameter<bool>("imu_use_orientation", true);
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_raw");
    declare_parameter<double>("control_frequency", 20.0);

    declare_parameter<std::string>("costmap_topic", "/mppi_local_planner/costmap");
    declare_parameter<std::string>("path_topic", "/mppi_local_planner/local_path");
    declare_parameter<std::string>("reference_path_topic", "/mppi_local_planner/reference_path");
    // ── 조종권 계약 [2026-09-01] (위 생성자의 상자 참고) ──
    declare_parameter<std::string>("handover.lstatus_topic", "/lstatus");
    declare_parameter<std::string>("handover.active_topic", "/lidar_active");
    //  ★GPS 기준선 [2026-09-11]★ 배열 규약의 소유자는 white1/driving.py 다.
    declare_parameter<std::string>("handover.ref_topic", "/lidar_ref");
    declare_parameter<std::string>("diag_topic", "/lidar_diag");
    //  ★회피 방향 결정 [2026-09-11]★ (updateLateralTarget)
    declare_parameter<double>("avoid.range_m", 8.0);      // 이 안의 장애물만 본다
    //  ★8 m 인 이유★ 콘 간격이 5~8 m 라(사용자), 5 m 면 #1 을 지난 뒤 #2 가
    //  사거리 밖이라 목표가 잠깐 0 으로 돌아갔다 다시 튄다.
    declare_parameter<double>("avoid.cone_half_m", 0.20); // 라바콘 반폭
    declare_parameter<double>("avoid.margin_m", 0.25);    // 그 위 안전여유
    declare_parameter<double>("avoid.max_offset_m", 1.25); // ★회피 목표 상한★
    //  ★이보다 가까운 콘은 "지나쳤다" 로 본다 [2026-09-11]★ 앞차축 1.25 m.
    declare_parameter<double>("avoid.pass_x_m", 0.90);
    //  ★기하 조향 [2026-09-11]★ false 면 종전 MPPI 조향으로 되돌아간다.
    declare_parameter<bool>("avoid.geometric_steer", true);
    declare_parameter<double>("avoid.k_psi", 1.0);    // 방향항 (감쇠)
    declare_parameter<double>("avoid.k_cte", 0.6);    // 위치항 (수렴)
    declare_parameter<double>("avoid.v_min", 1.0);    // atan 분모 하한 [m/s]
    //  ★붙들고 있던 콘보다 이만큼 멀어지면 "다음 콘" 으로 본다★
    declare_parameter<double>("avoid.new_cone_dx_m", 1.5);
    //  ★콘 군집 판정★ 치사 셀이 이만큼 안 모이면 잡음으로 본다.
    //  콘 하나의 치사 원반(반경 0.63 m)은 0.1 m 격자에서 ≈124 셀이다.
    declare_parameter<int>("avoid.cone_min_cells", 20);
    declare_parameter<double>("avoid.cone_y_max_m", 3.0);  // 이보다 옆은 무시
    //  ★코스트맵 팽창반경을 여기서도 안다★ 회피 목표가 팽창 경계 안에 앉으면
    //  플래너가 목표에서도 비용을 보고 밖으로 달아난다(updateLateralTarget).
    declare_parameter<double>("avoid.inflation_m", 0.40);
    declare_parameter<double>("handover.ref_stale_s", 0.5);
    declare_parameter<bool>("handover.use_gps_ref", true);
    //  ★false 로 두면 종전처럼 '런치 = 출발' 이다★ 라이다 단독 시험용. 실차에서
    //  white1 one_launch 로 띄울 때는 반드시 true 여야 한다 — 아니면 이 노드가
    //  GPS 추종 구간에서도 /cmd_vel_raw 를 내며 driving.py 와 다툰다.
    declare_parameter<bool>("handover.require_lstatus", true);
    //  허락이 이보다 낡으면 '허락 없음'. driving 은 20Hz 로 내므로 1.0s 는 20틱 여유.
    declare_parameter<double>("handover.lstatus_stale_s", 1.0);
    declare_parameter<std::string>("base_frame_id", "os_sensor");

    // ★금색차 실측 (lidar/kasa_units.hpp · drive_lidar.yaml)★
    declare_parameter<double>("wheelbase", lidar::kasa::WHEELBASE_M);
    declare_parameter<double>("track_width", 1.10);  // white/kasa_units.py 윤거 실측
    // ══════════════════════════════════════════════════════════════════════
    //  ★[2026-09-07] 순항속도를 ★한 값★ 으로 묶었다 (사용자 지시)★
    // ══════════════════════════════════════════════════════════════════════
    //  종전에는 네 곳이 짝이었다 — mppi.desired_speed(순항) · max_speed(플래너
    //  하드캡) · min_speed(하한) · kasa.max_pulse(액추에이터 상한). 그래서
    //  ★한쪽만 올리면 조용히 안 듣는다★:
    //    · desired_speed 만 올리면 max_speed 가 먼저 자른다
    //    · 둘을 올리고 kasa.max_pulse 를 안 올리면 msToPulse 가 자른다
    //  런치 주석도 "짝을 맞춰 둘 것" 이라고만 적혀 있었다 — 그 주석이 필요하다는
    //  것 자체가 설계 결함이다.
    //
    //  ★지금은 cruise_pulse 하나만 고치면 된다★ 나머지 셋은 여기서 유도한다.
    //  0 보다 크면 유도가 이기고, 0 이면 종전처럼 개별 파라미터를 그대로 쓴다
    //  (오래된 params.yaml·런치 호환).
    declare_parameter<int>("cruise_pulse", 2);
    declare_parameter<double>("max_speed", lidar::kasa::pulseToMs(2) * 1.15);
    declare_parameter<double>("min_speed", lidar::kasa::pulseToMs(1) * 0.90);
    declare_parameter<double>("max_steering_angle", 0.40);
    declare_parameter<double>("rear_overhang", 0.30);
    declare_parameter<double>("front_overhang", 0.0);
    declare_parameter<double>("max_accel", 2.0);
    declare_parameter<double>("max_steering_rate", 1.20);

    // 금색차 지령 층. 1/5카는 m/s 연속이라 MPPI 출력을 그대로 탔지만,
    // 여기선 펄스 계단 + 리니어 때문에 그대로 내면 1펄스↔정지가 반복되고
    // 조향이 좌우로 탄다.
    declare_parameter<int>("cmd.stop_enter_frames", 8);
    declare_parameter<int>("cmd.stop_exit_frames", 12);
    declare_parameter<double>("cmd.brake_after_s", 1.2);
    declare_parameter<double>("cmd.steer_lpf_alpha", 0.35);
    declare_parameter<double>("cmd.steer_slew_deg_s", 28.0);
    declare_parameter<double>("cmd.steer_deadband_deg", 0.0);
    declare_parameter<double>("cmd.dodge_steer_deg", 6.0);
    // ══════════════════════════════════════════════════════════════════════
    //  ★[2026-09-11] 회피 중 펄스를 1 → 2 로 (사용자 지시)★
    // ══════════════════════════════════════════════════════════════════════
    //  종전에는 |조향| > dodge_steer_deg 이면 ★1펄스★ 로 떨어뜨렸다(원본 1/5카의
    //  '회피 중 서행 3km/h' 를 그대로 옮긴 값). 그런데 금색차는 ★1펄스에서 거의
    //  안 움직인다★ — 실차 확인. 인휠 FF 테이블이 1펄스에 PWM 60 이고, 그 아래로는
    //  정지마찰을 못 이긴다. 게다가 A보드 PID 의 적분 누적 조건(|err|<4)과 겹쳐
    //  1펄스 지령은 제어 자체가 성립하지 않는다.
    //  → 회피 중에도 2펄스를 낸다. 라바콘 사이는 원래 서행 구간이라 2펄스(6.4km/h)
    //    면 충분하고, '움직이지 않는 것' 보다 훨씬 안전하다.
    declare_parameter<int>("cmd.dodge_pulse", 2);

    // ══════════════════════════════════════════════════════════════════════
    //  ★[2026-09-11] 저속 기동 보정(킥) — white1 low_speed_trim 이식★
    // ══════════════════════════════════════════════════════════════════════
    //  ★같은 문제를 white1 은 이미 풀어 놓았다★ 저속 지령은 이 차에서 '지령대로
    //  구르지 않는' 구간이라, driving.py 가 실측을 보고 REF 를 밀어 준다:
    //      out = REF + clamp(REF − 실측펄스, −2, +2),  0 ≤ out ≤ 15
    //  예) REF 2 인데 실측 0 → out 4 로 밀어 굴리기 시작하고, 실측이 2 가 되는
    //      순간 보정이 0 이 되어 out 2 로 돌아간다(속도 유지).
    //  mppi 는 순항이 2펄스라 ★항상 이 구간에서 논다★ — 그래서 더 필요하다.
    //
    //  ★20Hz 로 매 틱 다시 계산하면 채터링이 난다★ 보정은 속도가 따라올 시간을
    //  줘야 하므로 hold 주기로만 다시 판단한다(white1 과 같은 0.3s).
    //  ★출력 상한은 max_pulse 를 넘는다★ 킥은 '순항 천장' 이 아니라 '기동 가산'
    //  이라 성격이 다르다(driveRaw 가 그래서 있다). white1 의 REF_TRIM_OUT_MAX 와
    //  같은 이유이고, 여기서는 순항 2 + 보정 2 = 4펄스가 실효 상한이다.
    declare_parameter<bool>("cmd.trim_enable", true);
    declare_parameter<int>("cmd.trim_max_pulse", 2);     // 보정량 상한 ±2펄스
    declare_parameter<int>("cmd.trim_ref_max", 3);       // REF 가 이 이하일 때만
    declare_parameter<int>("cmd.trim_out_max", 6);       // 보정 출력 절대 상한
    declare_parameter<double>("cmd.trim_hold_s", 0.3);
    declare_parameter<std::string>("cmd.encoder_topic", "/encoder");

    // 장착 (cone_lidar.yaml / drive_lidar.yaml 2026-08-25 실측)
    declare_parameter<double>("sensor_height_m", 1.17);
    declare_parameter<double>("roi_agl_min", 0.20);
    declare_parameter<double>("roi_agl_max", 1.50);
    // lidar flip_lidar_xy:true = os_lidar xy 180° = sensor_yaw_offset π.
    declare_parameter<bool>("flip_lidar_xy", true);
    declare_parameter<bool>("brake_enable", true);

    declare_parameter<double>("costmap.size_x", 18.0);
    declare_parameter<double>("costmap.size_y", 14.0);
    declare_parameter<double>("costmap.resolution", 0.1);
    declare_parameter<double>("costmap.ground_z_min", 0.0);  // 0 => AGL 슬랩에서 유도
    declare_parameter<double>("costmap.ground_z_max", 0.0);
    declare_parameter<double>("costmap.inflation_radius", 0.40);
    declare_parameter<double>("costmap.min_range", 0.40);
    declare_parameter<double>("costmap.sensor_offset_x", 0.0);
    declare_parameter<double>("costmap.sensor_offset_y", 0.0);
    declare_parameter<double>("costmap.sensor_yaw_offset", 0.0);  // 0 => flip_lidar_xy 로 결정
    declare_parameter<double>("costmap.occupancy_decay", 0.50);
    // cone_lidar roi_x_min=2.0 : 1.2~1.5 m 는 차체·탑승자. 여기도 2.0 m 까지 지운다.
    // 1.4 m 만 지우면 보닛 반사가 전방 벽이 되어 stop-gate → 리니어 2단이 뜬다.
    declare_parameter<double>("costmap.ego_clear_radius", 0.90);
    declare_parameter<double>("costmap.ego_clear_margin", 0.12);
    declare_parameter<double>("costmap.ego_clear_front_max", 2.00);
    declare_parameter<double>("costmap.ego_clear_x_min", -0.45);
    declare_parameter<double>("costmap.ego_clear_x_max", 1.95);
    declare_parameter<double>("costmap.ego_clear_y_half", 0.85);
    //  ★지나친 콘이 복귀를 막지 않게 [2026-09-11]★ (ego_costmap.hpp 주석의 실측)
    //  앞차축(1.25) 뒤에서는 더 넓게 지운다 — 그 뒤는 조향으로 피할 대상이 아니다.
    declare_parameter<double>("costmap.ego_clear_pass_x", 1.30);
    declare_parameter<double>("costmap.ego_clear_y_half_passed", 1.60);
    declare_parameter<double>("costmap.ego_cost_clear_radius", 1.00);

    declare_parameter<int>("mppi.horizon_steps", 60);
    declare_parameter<double>("mppi.dt", 0.05);
    declare_parameter<int>("mppi.num_samples", 1200);
    declare_parameter<double>("mppi.lambda", 3.2);
    declare_parameter<double>("mppi.noise_std_v", 0.12);
    declare_parameter<double>("mppi.noise_std_delta", 0.28);
    declare_parameter<double>("mppi.noise_correlation", 0.65);
    declare_parameter<double>("mppi.desired_speed", lidar::kasa::pulseToMs(2));  // 2펄스 ≈ 6.4 km/h
    declare_parameter<double>("mppi.weight_obstacle", 1.4);
    declare_parameter<double>("mppi.weight_path", 1.0);
    declare_parameter<double>("mppi.weight_heading", 0.9);
    declare_parameter<double>("mppi.weight_speed", 2.0);
    declare_parameter<double>("mppi.weight_smooth_v", 0.6);
    declare_parameter<double>("mppi.weight_smooth_delta", 0.85);
    declare_parameter<double>("mppi.stanley_lookahead", 2.0);
    declare_parameter<double>("mppi.s_curve_dodge_frac", 0.22);
    declare_parameter<double>("mppi.s_curve_return_power", 0.70);
    declare_parameter<double>("mppi.path_progress_floor", 0.12);
    declare_parameter<double>("mppi.avoid_path_scale", 0.30);
    declare_parameter<double>("mppi.avoid_obs_gain", 80.0);
    declare_parameter<double>("mppi.offset_return_y", 0.18);
    declare_parameter<double>("mppi.offset_return_scale", 0.90);
    declare_parameter<double>("mppi.weight_return_clear", 3.5);
    declare_parameter<double>("mppi.return_clear_cost", 40.0);
    declare_parameter<double>("mppi.weight_path_terminal", 22.0);
    declare_parameter<double>("mppi.weight_heading_terminal", 12.0);
    declare_parameter<double>("mppi.max_lateral_offset", 1.20);
    declare_parameter<double>("mppi.weight_lateral_wall", 12.0);
    //  ★완만한 S — 하드 벽 + 헤딩 벽 [2026-09-11]★ (근거는 MPPIParams 주석)
    declare_parameter<double>("mppi.lateral_hard", 2.5);
    declare_parameter<double>("mppi.weight_lateral_hard", 4000.0);
    declare_parameter<double>("mppi.max_heading_dev", 0.52);
    declare_parameter<double>("mppi.weight_heading_wall", 4000.0);
    declare_parameter<double>("mppi.lookahead_distance", 7.0);
    declare_parameter<double>("mppi.lookahead_step", 0.40);
    declare_parameter<double>("mppi.weight_lookahead", 0.55);
    declare_parameter<double>("mppi.stop_cost_threshold", 750.0);

    declare_parameter<int>("imu_bias_calibration_samples", 30);

    declare_parameter<bool>("reference_reset.enable", true);
    declare_parameter<double>("reference_reset.clear_seconds", 1.5);
    // Must stay "blocked" this long (cost above blocked_threshold) before leaving
    // the clear latch -- prevents single noisy scans from unlatching.
    declare_parameter<double>("reference_reset.blocked_seconds", 0.5);
    declare_parameter<double>("reference_reset.check_distance", 4.0);
    declare_parameter<double>("reference_reset.check_half_width", 0.7);
    // Hysteresis band on corridor cost (EMA of max cell cost ahead):
    //   enter/keep-clear when ema < clear_threshold
    //   leave clear when ema > blocked_threshold
    // Legacy alias: cost_threshold maps to clear_threshold if the new keys are absent.
    declare_parameter<double>("reference_reset.cost_threshold", 25.0);
    declare_parameter<double>("reference_reset.clear_threshold", 20.0);
    declare_parameter<double>("reference_reset.blocked_threshold", 50.0);
    // EMA alpha for corridor max-cost (1 = no filter, ~0.2 = strong smoothing).
    declare_parameter<double>("reference_reset.cost_ema_alpha", 0.30);
    // After a re-anchor, ignore further resets for this long (even if clear).
    declare_parameter<double>("reference_reset.min_reset_interval", 5.0);
    // Re-anchor only after the vehicle has returned to the original reference
    // path (cross-track + heading). Prevents wiping the IMU heading mid-return
    // once the last obstacle clears and the corridor looks open.
    declare_parameter<double>("reference_reset.return_y_threshold", 0.10);
    declare_parameter<double>("reference_reset.return_yaw_threshold", 0.08);
    // Must stay inside return thresholds continuously this long before any reset.
    // Stops one-frame "looks returned" after the first cone from unlocking the path.
    declare_parameter<double>("reference_reset.return_hold_seconds", 1.5);
    // Soft re-anchor while latched-clear is the main path-walk after each gap in
    // a zigzag course. Default OFF: only rising-edge reset (and only x if preserve*).
    declare_parameter<bool>("reference_reset.soft_reset_enable", false);
    // CRITICAL for zigzag: never bake residual post-dodge yaw/y into a new
    // reference. Only zero along-track x (numerical hygiene). Full pose zero
    // made the yellow IMU line "unlock" after the first obstacle.
    declare_parameter<bool>("reference_reset.preserve_heading", true);
    declare_parameter<bool>("reference_reset.preserve_lateral", true);
  }

  void readParameters()
  {
    lidar_topic_ = get_parameter("lidar_topic").as_string();
    imu_topic_ = get_parameter("imu_topic").as_string();
    imu_use_orientation_ = get_parameter("imu_use_orientation").as_bool();
    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    control_frequency_ = get_parameter("control_frequency").as_double();

    costmap_topic_ = get_parameter("costmap_topic").as_string();
    path_topic_ = get_parameter("path_topic").as_string();
    reference_path_topic_ = get_parameter("reference_path_topic").as_string();
    ref_topic_    = get_parameter("handover.ref_topic").as_string();
    diag_topic_   = get_parameter("diag_topic").as_string();
    lat_target_range_ = get_parameter("avoid.range_m").as_double();
    lat_cone_half_    = get_parameter("avoid.cone_half_m").as_double();
    lat_margin_       = get_parameter("avoid.margin_m").as_double();
    lat_max_offset_   = get_parameter("avoid.max_offset_m").as_double();
    lat_inflation_    = get_parameter("avoid.inflation_m").as_double();
    lat_pass_x_       = get_parameter("avoid.pass_x_m").as_double();
    geometric_steer_  = get_parameter("avoid.geometric_steer").as_bool();
    geo_k_psi_        = get_parameter("avoid.k_psi").as_double();
    geo_k_cte_        = get_parameter("avoid.k_cte").as_double();
    geo_v_min_        = get_parameter("avoid.v_min").as_double();
    lat_new_cone_dx_  = get_parameter("avoid.new_cone_dx_m").as_double();
    cone_min_cells_   = static_cast<int>(get_parameter("avoid.cone_min_cells").as_int());
    cone_y_max_       = get_parameter("avoid.cone_y_max_m").as_double();
    ref_stale_s_  = get_parameter("handover.ref_stale_s").as_double();
    use_gps_ref_  = get_parameter("handover.use_gps_ref").as_bool();
    lstatus_topic_ = get_parameter("handover.lstatus_topic").as_string();
    active_topic_ = get_parameter("handover.active_topic").as_string();
    require_lstatus_ = get_parameter("handover.require_lstatus").as_bool();
    lstatus_stale_s_ =
      std::max(0.0, get_parameter("handover.lstatus_stale_s").as_double());
    base_frame_id_ = get_parameter("base_frame_id").as_string();

    vehicle_params_.wheelbase = get_parameter("wheelbase").as_double();
    vehicle_params_.track_width = get_parameter("track_width").as_double();
    vehicle_params_.max_speed = get_parameter("max_speed").as_double();
    vehicle_params_.min_speed = get_parameter("min_speed").as_double();
    //  ★cruise_pulse 가 이긴다★ (위 선언부의 상자 참고)
    cruise_pulse_ = get_parameter("cruise_pulse").as_int();
    if (cruise_pulse_ > 0) {
      cruise_pulse_ = std::min(cruise_pulse_, lidar::kasa::PULSE_PROTOCOL_MAX);
      //  하드캡은 순항의 15% 위 — 플래너가 순항을 낼 수 있게 여유만 준다.
      vehicle_params_.max_speed = lidar::kasa::pulseToMs(cruise_pulse_) * 1.15;
      //  하한은 회피 중 1펄스를 허용한다(원본 3params 와 같은 뜻).
      vehicle_params_.min_speed = lidar::kasa::pulseToMs(1) * 0.90;
    }
    vehicle_params_.max_steering_angle = get_parameter("max_steering_angle").as_double();
    vehicle_params_.rear_overhang = get_parameter("rear_overhang").as_double();
    vehicle_params_.front_overhang = get_parameter("front_overhang").as_double();
    vehicle_params_.max_accel = get_parameter("max_accel").as_double();
    vehicle_params_.max_steering_rate = get_parameter("max_steering_rate").as_double();

    stop_enter_frames_ = std::max(1, static_cast<int>(get_parameter("cmd.stop_enter_frames").as_int()));
    stop_exit_frames_ = std::max(1, static_cast<int>(get_parameter("cmd.stop_exit_frames").as_int()));
    brake_after_s_ = std::max(0.0, get_parameter("cmd.brake_after_s").as_double());
    steer_lpf_alpha_ = std::clamp(get_parameter("cmd.steer_lpf_alpha").as_double(), 0.05, 1.0);
    steer_slew_deg_s_ = std::max(5.0, get_parameter("cmd.steer_slew_deg_s").as_double());
    steer_deadband_deg_ = std::max(0.0, get_parameter("cmd.steer_deadband_deg").as_double());
    dodge_steer_deg_ = std::max(0.0, get_parameter("cmd.dodge_steer_deg").as_double());
    //  ★as_int() 는 int64_t 다★ int 멤버에 먼저 담고 나서 클램프한다
    //  (std::clamp 는 인자 타입이 같아야 추론된다 — cruise_pulse_ 와 같은 방식).
    dodge_pulse_     = static_cast<int>(get_parameter("cmd.dodge_pulse").as_int());
    dodge_pulse_     = std::max(1, dodge_pulse_);
    trim_enable_     = get_parameter("cmd.trim_enable").as_bool();
    trim_max_pulse_  = static_cast<int>(get_parameter("cmd.trim_max_pulse").as_int());
    trim_max_pulse_  = std::max(0, trim_max_pulse_);
    trim_ref_max_    = static_cast<int>(get_parameter("cmd.trim_ref_max").as_int());
    trim_ref_max_    = std::max(0, trim_ref_max_);
    trim_out_max_    = static_cast<int>(get_parameter("cmd.trim_out_max").as_int());
    trim_out_max_    = std::clamp(trim_out_max_, 0, lidar::kasa::PULSE_PROTOCOL_MAX);
    trim_hold_s_     = std::max(0.0, get_parameter("cmd.trim_hold_s").as_double());

    sensor_height_m_ = get_parameter("sensor_height_m").as_double();
    roi_agl_min_ = get_parameter("roi_agl_min").as_double();
    roi_agl_max_ = get_parameter("roi_agl_max").as_double();
    flip_lidar_xy_ = get_parameter("flip_lidar_xy").as_bool();
    brake_enable_ = get_parameter("brake_enable").as_bool();

    costmap_params_.size_x = get_parameter("costmap.size_x").as_double();
    costmap_params_.size_y = get_parameter("costmap.size_y").as_double();
    costmap_params_.resolution = get_parameter("costmap.resolution").as_double();
    costmap_params_.ground_z_min = get_parameter("costmap.ground_z_min").as_double();
    costmap_params_.ground_z_max = get_parameter("costmap.ground_z_max").as_double();
    // YAML 이 0 을 주면 lidar 와 같이 AGL 슬랩에서 센서 z 를 유도한다.
    if (costmap_params_.ground_z_max <= costmap_params_.ground_z_min) {
      costmap_params_.ground_z_min = roi_agl_min_ - sensor_height_m_;
      costmap_params_.ground_z_max = roi_agl_max_ - sensor_height_m_;
    }
    costmap_params_.inflation_radius = get_parameter("costmap.inflation_radius").as_double();
    costmap_params_.min_range = get_parameter("costmap.min_range").as_double();
    costmap_params_.sensor_offset_x = get_parameter("costmap.sensor_offset_x").as_double();
    costmap_params_.sensor_offset_y = get_parameter("costmap.sensor_offset_y").as_double();
    costmap_params_.sensor_yaw_offset = get_parameter("costmap.sensor_yaw_offset").as_double();
    // lidar flip_lidar_xy:true = xy 동시 부호반전 = yaw π. 두 번 뒤집지 말 것.
    if (std::abs(costmap_params_.sensor_yaw_offset) < 1e-9 && flip_lidar_xy_) {
      costmap_params_.sensor_yaw_offset = M_PI;
    }
    costmap_params_.occupancy_decay = get_parameter("costmap.occupancy_decay").as_double();
    costmap_params_.ego_clear_radius = get_parameter("costmap.ego_clear_radius").as_double();
    costmap_params_.robot_half_width = vehicle_params_.track_width / 2.0 + 0.08;
    // Do NOT force ego_clear_radius >= robot_half_width — that expanded the
    // white free zone over nearby 라바콘. Inflation bleed is handled by the
    // separate small ego_cost_clear_radius on the cost layer only.

    // Tight rectangular occupancy clear. Auto from body dims but HARD-CAPPED
    // forward by ego_clear_front_max so cones at ~0.5–1.0 m stay visible.
    const double clear_margin = std::max(0.0, get_parameter("costmap.ego_clear_margin").as_double());
    const double front_max = std::max(0.15, get_parameter("costmap.ego_clear_front_max").as_double());
    double cx_min = get_parameter("costmap.ego_clear_x_min").as_double();
    double cx_max = get_parameter("costmap.ego_clear_x_max").as_double();
    double cy_half = get_parameter("costmap.ego_clear_y_half").as_double();
    if (cx_max <= cx_min || cy_half <= 0.0) {
      cx_min = -(vehicle_params_.rear_overhang + clear_margin);
      // Only pad a short way past the rear axle / mount — not full body length.
      cx_max = std::min(
        front_max,
        std::max(0.25, vehicle_params_.front_overhang + clear_margin + 0.15));
      cy_half = vehicle_params_.track_width * 0.5 + clear_margin;
    } else {
      // Even manual x_max is capped so a bad yaml cannot re-mask near cones.
      cx_max = std::min(cx_max, front_max);
    }
    costmap_params_.ego_clear_x_min = cx_min;
    costmap_params_.ego_clear_x_max = cx_max;
    costmap_params_.ego_clear_y_half = cy_half;
    costmap_params_.ego_clear_pass_x =
      get_parameter("costmap.ego_clear_pass_x").as_double();
    costmap_params_.ego_clear_y_half_passed =
      get_parameter("costmap.ego_clear_y_half_passed").as_double();
    costmap_params_.ego_cost_clear_radius =
      get_parameter("costmap.ego_cost_clear_radius").as_double();

    mppi_params_.horizon_steps = get_parameter("mppi.horizon_steps").as_int();
    mppi_params_.dt = get_parameter("mppi.dt").as_double();
    mppi_params_.num_samples = get_parameter("mppi.num_samples").as_int();
    mppi_params_.lambda = get_parameter("mppi.lambda").as_double();
    mppi_params_.noise_std_v = get_parameter("mppi.noise_std_v").as_double();
    mppi_params_.noise_std_delta = get_parameter("mppi.noise_std_delta").as_double();
    mppi_params_.noise_correlation = get_parameter("mppi.noise_correlation").as_double();
    mppi_params_.desired_speed = get_parameter("mppi.desired_speed").as_double();
    if (cruise_pulse_ > 0) {
      mppi_params_.desired_speed = lidar::kasa::pulseToMs(cruise_pulse_);
    }
    mppi_params_.weight_obstacle = get_parameter("mppi.weight_obstacle").as_double();
    mppi_params_.weight_path = get_parameter("mppi.weight_path").as_double();
    mppi_params_.weight_heading = get_parameter("mppi.weight_heading").as_double();
    mppi_params_.weight_speed = get_parameter("mppi.weight_speed").as_double();
    mppi_params_.weight_smooth_v = get_parameter("mppi.weight_smooth_v").as_double();
    mppi_params_.weight_smooth_delta = get_parameter("mppi.weight_smooth_delta").as_double();
    mppi_params_.stanley_lookahead = get_parameter("mppi.stanley_lookahead").as_double();
    mppi_params_.s_curve_dodge_frac = get_parameter("mppi.s_curve_dodge_frac").as_double();
    mppi_params_.s_curve_return_power = get_parameter("mppi.s_curve_return_power").as_double();
    mppi_params_.path_progress_floor = get_parameter("mppi.path_progress_floor").as_double();
    mppi_params_.avoid_path_scale = get_parameter("mppi.avoid_path_scale").as_double();
    mppi_params_.avoid_obs_gain = get_parameter("mppi.avoid_obs_gain").as_double();
    mppi_params_.offset_return_y = get_parameter("mppi.offset_return_y").as_double();
    mppi_params_.offset_return_scale = get_parameter("mppi.offset_return_scale").as_double();
    mppi_params_.weight_return_clear = get_parameter("mppi.weight_return_clear").as_double();
    mppi_params_.return_clear_cost = get_parameter("mppi.return_clear_cost").as_double();
    mppi_params_.weight_path_terminal = get_parameter("mppi.weight_path_terminal").as_double();
    mppi_params_.weight_heading_terminal = get_parameter("mppi.weight_heading_terminal").as_double();
    mppi_params_.max_lateral_offset = get_parameter("mppi.max_lateral_offset").as_double();
    mppi_params_.weight_lateral_wall = get_parameter("mppi.weight_lateral_wall").as_double();
    mppi_params_.lateral_hard = get_parameter("mppi.lateral_hard").as_double();
    mppi_params_.weight_lateral_hard =
      get_parameter("mppi.weight_lateral_hard").as_double();
    mppi_params_.max_heading_dev = get_parameter("mppi.max_heading_dev").as_double();
    mppi_params_.weight_heading_wall =
      get_parameter("mppi.weight_heading_wall").as_double();
    mppi_params_.lookahead_distance = get_parameter("mppi.lookahead_distance").as_double();
    mppi_params_.lookahead_step = get_parameter("mppi.lookahead_step").as_double();
    mppi_params_.weight_lookahead = get_parameter("mppi.weight_lookahead").as_double();
    mppi_params_.stop_cost_threshold = get_parameter("mppi.stop_cost_threshold").as_double();

    imu_bias_calibration_samples_ = get_parameter("imu_bias_calibration_samples").as_int();

    reference_reset_enable_ = get_parameter("reference_reset.enable").as_bool();
    reference_reset_clear_seconds_ = get_parameter("reference_reset.clear_seconds").as_double();
    reference_reset_blocked_seconds_ = get_parameter("reference_reset.blocked_seconds").as_double();
    reference_reset_check_distance_ = get_parameter("reference_reset.check_distance").as_double();
    reference_reset_check_half_width_ = get_parameter("reference_reset.check_half_width").as_double();

    // Prefer explicit clear/blocked thresholds; fall back to legacy cost_threshold.
    const double legacy_th = get_parameter("reference_reset.cost_threshold").as_double();
    reference_reset_clear_threshold_ = get_parameter("reference_reset.clear_threshold").as_double();
    reference_reset_blocked_threshold_ = get_parameter("reference_reset.blocked_threshold").as_double();
    // If user only set the old key (or left new keys at defaults equal to each other
    // while legacy differs), keep a sensible band above the clear threshold.
    if (reference_reset_clear_threshold_ <= 0.0) {
      reference_reset_clear_threshold_ = legacy_th;
    }
    if (reference_reset_blocked_threshold_ <= reference_reset_clear_threshold_) {
      reference_reset_blocked_threshold_ =
        std::max(legacy_th, reference_reset_clear_threshold_) + 30.0;
    }
    reference_reset_cost_ema_alpha_ =
      std::clamp(get_parameter("reference_reset.cost_ema_alpha").as_double(), 0.01, 1.0);
    reference_reset_min_interval_ =
      std::max(0.0, get_parameter("reference_reset.min_reset_interval").as_double());
    reference_reset_return_y_ =
      std::max(0.0, get_parameter("reference_reset.return_y_threshold").as_double());
    reference_reset_return_yaw_ =
      std::max(0.0, get_parameter("reference_reset.return_yaw_threshold").as_double());
    reference_reset_return_hold_ =
      std::max(0.0, get_parameter("reference_reset.return_hold_seconds").as_double());
    reference_reset_soft_enable_ =
      get_parameter("reference_reset.soft_reset_enable").as_bool();
    reference_reset_preserve_heading_ =
      get_parameter("reference_reset.preserve_heading").as_bool();
    reference_reset_preserve_lateral_ =
      get_parameter("reference_reset.preserve_lateral").as_bool();
  }

  // Steady clock seconds — used only for permit freshness, so it must not
  // depend on /clock or ROS time jumps.
  static double nowSeconds()
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  /// 지금 이 노드가 /cmd_vel_raw 를 내도 되는가.
  /// ★신선도가 곧 허락이다★ driving 이 죽어 토픽이 끊기면 마지막 'L' 을 붙들지
  /// 않고 손을 뗀다 — 붙들면 아무도 감시하지 않는 채로 차를 계속 몬다.
  /// ★'L' 하나만 본다★ '0'(GPS 추종)·'S'(일시정지)에서는 자동으로 침묵이므로
  /// 일시정지 로직을 위해 이 노드에 더 넣을 코드가 없다.
  bool permitted() const
  {
    if (!require_lstatus_) {
      return true;                       // 라이다 단독 시험 — 종전의 '런치 = 출발'
    }
    const double stamp = lstatus_stamp_.load(std::memory_order_relaxed);
    if (stamp <= 0.0) {
      return false;                      // 한 번도 못 받았다
    }
    if (lstatus_stale_s_ > 0.0 && (nowSeconds() - stamp) > lstatus_stale_s_) {
      return false;                      // driving 이 죽었다
    }
    return lstatus_.load(std::memory_order_relaxed) == 'L';
  }

  /// 살아 있다는 신고. ★매 틱 낸다 — driving 은 값이 아니라 신선도를 본다★
  void publishActive(bool active)
  {
    std_msgs::msg::Bool m;
    m.data = active;
    active_pub_->publish(m);
  }

  /// ★L 구간에 들어올 때마다 기준선을 다시 잡는다 [2026-09-01]★
  ///
  /// heading_ref_yaw_ 는 원래 ★런치 직후 자이로 보정 단계에서 딱 한 번★ 잡히고
  /// 그 뒤로는 다시 잡히지 않는다(applyReferenceReset 은 x 만 0 으로 만들고
  /// yaw·y 는 기본 보존이다). one_launch 로 white1 과 함께 띄우면 그 값은
  /// ★출발 주차 위치의 방위★ 이고, 수백 미터 뒤 L 구간에서 이 노드는 그 선으로
  /// 복귀하려 한다 — 전혀 다른 곳이다.
  ///
  /// odom_pose_.x/y 도 함께 0 으로 돌린다. 그 둘은 ★last_commanded_v_ 로 적분★
  /// 되므로 침묵 구간(GPS 추종이 몰던 시간)에는 v=0 이라 제자리에 멈춰 있는데
  /// 차는 그동안 수백 미터를 갔다. 즉 지금 값은 현실과 아무 관계가 없다.
  ///
  /// 래치·EMA·warm start 도 모두 지운다. 지난 L 구간의 '복도가 비었다' 판정과
  /// 직전 제어열을 들고 새 구간을 시작하면 첫 몇 틱이 낡은 계획으로 조향한다.
  void rearmReference()
  {
    //  ★혹시 남아 있는 제동을 먼저 놓는다★ 하강엣지의 releaseBrake() 는 최소
    //  물림(BRAKE_MIN_HOLD_S) 안이면 아무 일도 하지 않고 돌아간다 — 그때 stage_ 가
    //  0 이 아닌 채로 남으면 이 구간이 '이미 제동 중' 으로 시작해 구동을 내지 않는다.
    //  두 L 구간 사이에는 최소 물림보다 훨씬 긴 시간이 지나므로 여기서는 반드시 풀린다.
    actuator_->releaseBrake();
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (has_abs_yaw_) {
        heading_ref_yaw_ = last_abs_yaw_;   // 지금 방위가 새 기준선이다
      }
      odom_pose_ = OdomPose{};              // x = y = yaw = 0
    }
    corridor_clear_latched_ = false;
    lat_latched_ = false;              // 회피 방향 래치도 푼다 [2026-09-11]
    lat_last_side_ = 0;                // 교대 기억도 지운다
    lat_locked_ox_ = 0.0;
    mppi_params_.lateral_target = 0.0;
    corridor_cost_ema_ = 0.0;
    corridor_cost_ema_init_ = false;
    clear_ahead_seconds_ = 0.0;
    blocked_ahead_seconds_ = 0.0;
    path_return_hold_seconds_ = 0.0;
    last_reference_reset_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    stop_latched_ = false;
    stop_enter_count_ = 0;
    stop_exit_count_ = 0;
    stop_latch_t_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    steer_filt_deg_ = 0.0;
    last_pub_steer_deg_ = 0.0;
    last_commanded_v_.store(0.0, std::memory_order_relaxed);
    controller_->reset();
    RCLCPP_INFO(
      get_logger(),
      "🛞 조종권 인수 — 기준선 재설정 (yaw0=%.1f deg%s). "
      "이 방위의 직선으로 복귀하며 라바콘을 피한다",
      heading_ref_yaw_ * 180.0 / M_PI,
      has_abs_yaw_ ? "" : " ★절대방위 미수신 — 현재 자세를 0 으로 둔다★");
  }

  void publishStop(bool apply_brake)
  {
    last_commanded_v_.store(0.0, std::memory_order_relaxed);
    // D5 수동·E-STOP 에서는 리니어를 물지 않는다. 물면 HUD 만 2단으로 보이고
    // arduino 수동 분기는 /brake_level 을 무시한다.
    const bool can_act = actuator_->ready();
    if (apply_brake && brake_enable_ && can_act) {
      actuator_->brake(lidar::kasa::BRAKE_FULL);
    }
    actuator_->keepBrake();
    if (apply_brake && can_act) {
      actuator_->drive(0.0, 0.0, /*control_enable=*/true);
    } else {
      actuator_->hold(/*control_enable=*/false);
    }
  }

  /// ★[2026-09-11] 인자가 m/s 에서 ★펄스★ 로 바뀌었다★ 저속 기동 보정(킥)이
  /// max_pulse 를 넘겨야 하는데, m/s 로 넘기면 msToPulse 가 거기서 잘라 버린다.
  void publishDrive(int pulse, double road_deg)
  {
    if (brake_enable_ && actuator_->brakeStage() > lidar::kasa::BRAKE_OFF) {
      actuator_->releaseBrake();
    }
    actuator_->keepBrake();

    if (!actuator_->ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "⛔ 구동 불가 — %s", actuator_->blockReason());
      last_commanded_v_.store(0.0, std::memory_order_relaxed);
      actuator_->hold(/*control_enable=*/false);
      return;
    }

    // 리니어가 아직 최소 물림 중이면 구동을 내지 않는다 — 구동과 제동이
    // 서로 미는 상태가 된다(drive_lidar_node 와 같은 이유).
    const bool braking = actuator_->brakeStage() > lidar::kasa::BRAKE_OFF;
    const int p_out = braking ? 0 : pulse;
    //  ★pot 환산은 실측 속도로★ 지령(특히 킥으로 올린 값)을 넣으면 언더스티어
    //  항이 v² 로 부풀어 조향이 ±40 에 포화한다(driveRaw 주석의 실측).
    const double enc_age2 = nowSeconds() - enc_t_.load(std::memory_order_relaxed);
    const double v_meas = (enc_t_.load(std::memory_order_relaxed) > 0.0 && enc_age2 <= 1.0)
        ? enc_pulse_.load(std::memory_order_relaxed) * lidar::kasa::MS_PER_PULSE
        : -1.0;
    actuator_->driveRaw(p_out, braking ? 0.0 : road_deg, /*control_enable=*/true,
                        v_meas);
    last_commanded_v_.store(
      lidar::kasa::pulseToMs(actuator_->lastPulse()), std::memory_order_relaxed);
  }

  /// 저속에서 실측을 보고 REF 를 밀어 준다 → 이번 틱에 실제로 낼 펄스.
  /// [2026-09-11 white1 low_speed_trim 이식 — 근거는 cmd.trim_* 선언부]
  ///
  ///     out = REF + clamp(REF − 실측펄스, −trim_max, +trim_max)
  ///
  /// ★동작 조건이 좁다★ 1 ≤ REF ≤ trim_ref_max 일 때만이다.
  ///   · REF 0 에서는 절대 걸지 않는다 — 세우려는 지시를 보정이 뒤집으면 안 된다.
  ///   · 엔코더가 낡았으면(노드 사망) 보정하지 않는다 — 모르면 밀지 않는다.
  /// ★hold 주기로만 다시 판단한다★ 20Hz 로 매 틱 계산하면 속도가 따라오기 전에
  /// 보정이 널뛴다. REF 가 바뀌면 그 자리에서 즉시 다시 본다.
  int lowSpeedTrim(int ref)
  {
    if (!trim_enable_ || ref <= 0 || ref > trim_ref_max_) {
      trim_ = 0;
      trim_ref_ = ref;
      return ref;
    }
    const double now = nowSeconds();
    const double enc_age = now - enc_t_.load(std::memory_order_relaxed);
    if (enc_t_.load(std::memory_order_relaxed) <= 0.0 || enc_age > 1.0) {
      trim_ = 0;                      // 실측을 모르면 밀지 않는다
      trim_ref_ = ref;
      return ref;
    }
    if (ref != trim_ref_ || (now - trim_t_) >= trim_hold_s_) {
      const double meas = enc_pulse_.load(std::memory_order_relaxed);
      const int err = static_cast<int>(std::lround(ref - meas));
      trim_ = std::clamp(err, -trim_max_pulse_, trim_max_pulse_);
      trim_t_ = now;
      trim_ref_ = ref;
    }
    return std::clamp(ref + trim_, 0, trim_out_max_);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "cloudCallback: %u points, frame_id='%s'",
      msg->width * msg->height, msg->header.frame_id.c_str());

    last_cloud_frame_id_ = msg->header.frame_id;

    if (!static_tf_published_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = rclcpp::Time(0);
      tf_msg.header.frame_id = base_frame_id_;
      tf_msg.child_frame_id = msg->header.frame_id;
      tf_msg.transform.translation.x = costmap_params_.sensor_offset_x;
      tf_msg.transform.translation.y = costmap_params_.sensor_offset_y;
      tf_msg.transform.translation.z = 0.0;
      tf_msg.transform.rotation = yawToQuaternion(costmap_params_.sensor_yaw_offset);
      static_tf_broadcaster_->sendTransform(tf_msg);
      static_tf_published_ = true;
      RCLCPP_INFO(
        get_logger(), "Broadcast static TF: %s -> %s (offset x=%.2f y=%.2f yaw=%.2f rad)",
        base_frame_id_.c_str(), msg->header.frame_id.c_str(),
        costmap_params_.sensor_offset_x, costmap_params_.sensor_offset_y,
        costmap_params_.sensor_yaw_offset);
    }

    costmap_->updateFromPointCloud(*msg);

    nav_msgs::msg::OccupancyGrid grid = costmap_->toOccupancyGrid();
    grid.header.stamp = msg->header.stamp;
    grid.header.frame_id = base_frame_id_;
    costmap_pub_->publish(grid);
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "imuCallback: wz=%.3f, frame_id='%s'",
      msg->angular_velocity.z, msg->header.frame_id.c_str());
    const rclcpp::Time stamp(msg->header.stamp);
    std::lock_guard<std::mutex> lock(odom_mutex_);
    const bool use_ori = imu_use_orientation_ && orientationValid(*msg);

    if (!gyro_bias_calibrated_.load(std::memory_order_relaxed)) {
      last_imu_time_ = stamp;
      const int n = ++gyro_bias_sample_count_;
      if (use_ori) {
        heading_ref_yaw_ = yawFromQuaternion(msg->orientation);
        last_abs_yaw_ = heading_ref_yaw_;   // 재무장이 꺼내 쓴다 (rearmReference)
        has_abs_yaw_ = true;
      } else {
        gyro_bias_sum_ += msg->angular_velocity.z;
      }
      if (n >= imu_bias_calibration_samples_) {
        if (use_ori) {
          imu_heading_from_quat_ = true;
        } else {
          gyro_bias_wz_ = gyro_bias_sum_ / static_cast<double>(n);
          imu_heading_from_quat_ = false;
        }
        imu_initialized_ = true;
        gyro_bias_calibrated_.store(true, std::memory_order_release);
        if (imu_heading_from_quat_) {
          RCLCPP_INFO(
            get_logger(),
            "AHRS 헤딩 잠금 (%d 표본, yaw0=%.1f deg) — 외장 iAHRS 쿼터니언. "
            "자이로 적분을 쓰지 않아 드리프트가 작다. 잠글 때까지 차를 세워 둘 것.",
            n, heading_ref_yaw_ * 180.0 / M_PI);
        } else {
          RCLCPP_INFO(
            get_logger(),
            "Gyro bias calibrated over %d samples: wz_bias=%.5f rad/s "
            "(orientation 없음 → 자이로 적분. 드리프트가 쌓인다)",
            n, gyro_bias_wz_);
        }
      }
      return;
    }

    if (!imu_initialized_) {
      last_imu_time_ = stamp;
      imu_initialized_ = true;
      return;
    }
    const double dt = (stamp - last_imu_time_).seconds();
    last_imu_time_ = stamp;
    if (dt <= 0.0 || dt > 0.5) {
      return;
    }

    if (use_ori) {
      const double yaw_abs = yawFromQuaternion(msg->orientation);
      last_abs_yaw_ = yaw_abs;              // 재무장이 꺼내 쓴다 (rearmReference)
      has_abs_yaw_ = true;
      odom_pose_.yaw = wrapAngle(yaw_abs - heading_ref_yaw_);
      imu_heading_from_quat_ = true;
    } else {
      double wz = msg->angular_velocity.z - gyro_bias_wz_;
      const double v_now = last_commanded_v_.load(std::memory_order_relaxed);
      if (v_now < 0.25 && std::abs(wz) < 0.08) {
        wz = 0.0;
      }
      odom_pose_.yaw = wrapAngle(odom_pose_.yaw + wz * dt);
    }
    //  ★x 는 여전히 추측항법이다★ 전방 진행거리는 코스트맵 조회에만 쓰이고
    //  절대 기준이 필요 없다. ★속도는 실측(엔코더)을 쓴다★ — 지령으로 적분하면
    //  1.65배 부풀려진다(실측). 엔코더가 없으면 종전대로 지령으로 떨어진다.
    const double enc_age = nowSeconds() - enc_t_.load(std::memory_order_relaxed);
    const double v_meas = enc_pulse_.load(std::memory_order_relaxed)
                          * lidar::kasa::MS_PER_PULSE;
    const double v = (enc_t_.load(std::memory_order_relaxed) > 0.0 && enc_age <= 1.0)
                       ? v_meas
                       : last_commanded_v_.load(std::memory_order_relaxed);
    odom_pose_.x += v * std::cos(odom_pose_.yaw) * dt;

    // ══════════════════════════════════════════════════════════════════════
    //  ★y·yaw 는 GPS 기준선이 있으면 그것으로 덮는다 [2026-09-11]★
    // ══════════════════════════════════════════════════════════════════════
    //  추측항법의 y 는 누적오차를 바로잡을 방법이 없다(위 ref_sub_ 주석의 실측).
    //  driving 이 매핑 중심선 기준의 CTE·방위오차를 20Hz 로 주므로 그대로 쓴다.
    //  ★신선하지 않으면 종전 거동으로 떨어진다★ — driving 이 죽었거나 GPS 품질이
    //  나쁘면 valid=0 이 오고, 그때는 추측항법이 그래도 없는 것보다 낫다.
    if (use_gps_ref_ && ref_valid_ &&
        (nowSeconds() - ref_t_) <= ref_stale_s_)
    {
      odom_pose_.y   = ref_cte_;    // + = 중심선 왼쪽 (driving 과 같은 부호)
      odom_pose_.yaw = ref_herr_;   // + = 중심선보다 왼쪽을 향함
      gps_ref_live_ = true;
    } else {
      gps_ref_live_ = false;
      odom_pose_.y += v * std::sin(odom_pose_.yaw) * dt;
    }
  }

  // Max cost in the forward corridor used for reference-reset decisions.
  double corridorMaxCost(const CostmapSnapshot & snap) const
  {
    double max_c = 0.0;
    for (double x = 0.5; x <= reference_reset_check_distance_; x += 0.5) {
      for (double y = -reference_reset_check_half_width_;
           y <= reference_reset_check_half_width_; y += 0.4)
      {
        max_c = std::max(max_c, snap.getCost(x, y));
      }
    }
    return max_c;
  }

  // Re-anchor the reference line with hysteresis so noisy costmaps cannot
  // flip clear/blocked every scan (which made the orange reference path look
  // like two vibrating lines and jerked path/heading costs in MPPI).
  //
  // State machine:
  //   !latched:  ema < clear_th for clear_seconds  -> latch (+ optional reset)
  //   latched:   ema > blocked_th for blocked_seconds -> unlatch
  //   soft reset (optional, default OFF): while latched + returned + hold
  //
  // Zigzag fix: full odom zero after the first cone gap baked residual yaw into
  // a new "straight ahead", so the yellow IMU reference unlocked and walked.
  // Defaults preserve heading + lateral (only zero x) and disable soft reset.
  void maybeResetReference(const CostmapSnapshot & snap)
  {
    if (!reference_reset_enable_ || !snap.valid) {
      return;
    }

    const double period = 1.0 / std::max(1.0, control_frequency_);
    const double raw_max = corridorMaxCost(snap);

    // Low-pass the corridor cost so single-frame lidar flicker is ignored.
    if (!corridor_cost_ema_init_) {
      corridor_cost_ema_ = raw_max;
      corridor_cost_ema_init_ = true;
    } else {
      const double a = reference_reset_cost_ema_alpha_;
      corridor_cost_ema_ = a * raw_max + (1.0 - a) * corridor_cost_ema_;
    }

    std::lock_guard<std::mutex> lock(odom_mutex_);
    const rclcpp::Time t_now = now();
    const bool path_returned = isReturnedToReference(odom_pose_);

    // Continuous hold on the original path before any re-anchor is allowed.
    if (path_returned) {
      path_return_hold_seconds_ += period;
    } else {
      path_return_hold_seconds_ = 0.0;
    }
    const bool hold_ok =
      path_return_hold_seconds_ >= reference_reset_return_hold_;

    if (!corridor_clear_latched_) {
      // Trying to enter "open corridor" mode.
      if (corridor_cost_ema_ < reference_reset_clear_threshold_) {
        clear_ahead_seconds_ += period;
        blocked_ahead_seconds_ = 0.0;
      } else {
        clear_ahead_seconds_ = 0.0;
      }

      if (clear_ahead_seconds_ >= reference_reset_clear_seconds_) {
        corridor_clear_latched_ = true;
        clear_ahead_seconds_ = 0.0;
        blocked_ahead_seconds_ = 0.0;
        // Rising-edge re-anchor only if held on path (not a one-frame blip
        // between zigzag cones) and interval allows.
        //  ★GPS 기준선이 살아 있으면 재설정하지 않는다 [2026-09-11]★
        //  이 재설정은 ★추측항법 드리프트를 털어내려고★ 있는 장치다. 기준선이
        //  실제 매핑 중심선일 때 같은 일을 하면 ★진짜 횡오차를 0 으로 지워★
        //  차가 벗어난 자리를 새 중심선으로 삼는다 — 정확히 반대 효과다.
        if (!gps_ref_live_ && path_returned && hold_ok && canResetNow(t_now)) {
          applyReferenceReset(odom_pose_);
          last_reference_reset_time_ = t_now;
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "Reference re-anchored (clear latch + path hold %.1fs): "
            "corridor_ema=%.1f preserve_yaw=%d preserve_y=%d | "
            "y=%.2f yaw=%.1f deg",
            path_return_hold_seconds_, corridor_cost_ema_,
            static_cast<int>(reference_reset_preserve_heading_),
            static_cast<int>(reference_reset_preserve_lateral_),
            odom_pose_.y, odom_pose_.yaw * 180.0 / M_PI);
        } else if (!path_returned || !hold_ok) {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Clear corridor but holding IMU reference: "
            "y=%.2f m (th=%.2f), yaw=%.1f deg (th=%.1f), hold=%.1f/%.1f s",
            odom_pose_.y, reference_reset_return_y_,
            odom_pose_.yaw * 180.0 / M_PI,
            reference_reset_return_yaw_ * 180.0 / M_PI,
            path_return_hold_seconds_, reference_reset_return_hold_);
        }
      }
    } else {
      // Latched clear: only leave after sustained high cost (hysteresis).
      if (corridor_cost_ema_ > reference_reset_blocked_threshold_) {
        blocked_ahead_seconds_ += period;
        clear_ahead_seconds_ = 0.0;
        // Do NOT re-anchor while approaching the next cone group.
      } else {
        blocked_ahead_seconds_ = 0.0;
        // Soft re-anchor is OFF by default. When enabled, still only applies
        // applyReferenceReset (preserve yaw/y unless explicitly disabled).
        if (!gps_ref_live_ && reference_reset_soft_enable_ &&
            path_returned && hold_ok && canResetNow(t_now))
        {
          applyReferenceReset(odom_pose_);
          last_reference_reset_time_ = t_now;
          RCLCPP_DEBUG_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "Reference soft re-anchored (open + path hold, interval ok)");
        } else if (!path_returned || !hold_ok) {
          RCLCPP_DEBUG_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Open corridor, holding IMU reference: y=%.2f yaw=%.1f deg hold=%.1f s",
            odom_pose_.y, odom_pose_.yaw * 180.0 / M_PI,
            path_return_hold_seconds_);
        }
      }

      if (blocked_ahead_seconds_ >= reference_reset_blocked_seconds_) {
        corridor_clear_latched_ = false;
        blocked_ahead_seconds_ = 0.0;
        clear_ahead_seconds_ = 0.0;
        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Reference clear-latch released: corridor_ema=%.1f > blocked_th=%.1f",
          corridor_cost_ema_, reference_reset_blocked_threshold_);
      }
    }
  }

  // Apply a reference reset without unlocking the original IMU heading.
  // - always zero along-track x (does not change direction of the yellow line)
  // - zero y only if preserve_lateral == false
  // - zero yaw only if preserve_heading == false  (this is what used to walk the path)
  void applyReferenceReset(OdomPose & pose) const
  {
    pose.x = 0.0;
    if (!reference_reset_preserve_lateral_) {
      pose.y = 0.0;
    }
    if (!reference_reset_preserve_heading_) {
      pose.yaw = 0.0;
    }
  }

  // True when lateral offset and heading error to the straight reference
  // (y_odom==0, yaw_odom==0) are both small enough to allow re-anchor.
  bool isReturnedToReference(const OdomPose & pose) const
  {
    return std::abs(pose.y) <= reference_reset_return_y_ &&
           std::abs(wrapAngle(pose.yaw)) <= reference_reset_return_yaw_;
  }

  bool canResetNow(const rclcpp::Time & t_now) const
  {
    if (reference_reset_min_interval_ <= 0.0) {
      return true;
    }
    if (last_reference_reset_time_.nanoseconds() == 0) {
      return true;
    }
    return (t_now - last_reference_reset_time_).seconds() >= reference_reset_min_interval_;
  }

  void controlLoop()
  {
    const rclcpp::Time t_now = now();
    const double elapsed = (t_now - last_control_time_).seconds();
    last_control_time_ = t_now;

    // ════════════════════════════════════════════════════════════════════
    //  ★조종권 게이트 — 발행 이전에 있어야 한다★ (생성자의 상자 참고)
    // ════════════════════════════════════════════════════════════════════
    //  ★publishStop() 으로 빠지지 않는다★ 그 함수는 actuator_->hold() 를 부르고,
    //  hold() 는 /cmd_vel_raw 에 linear.x=0 을 ★실제로 발행한다★. 허락이 없는
    //  동안 그것을 내면 GPS 추종이 낸 펄스를 20Hz 로 0 으로 덮어 버린다 —
    //  KasaActuator::ready() 를 false 로 만드는 것만으로는 침묵이 되지 않는다.
    //  침묵은 ★아무 발행 경로도 타지 않고 그냥 빠지는 것★ 이다.
    if (!permitted()) {
      if (was_permitted_) {
        //  ★내가 물고 있던 리니어를 내가 놓는다★ 놓지 않으면 /brake_level 의
        //  마지막 값이 2단인 채로 남는다 — 그 토픽은 마지막 발행자가 이기고,
        //  driving 쪽은 '0 은 재확인하지 않는다'는 규칙 때문에 스스로 풀어 주지
        //  않는다. 그러면 GPS 추종이 ★브레이크를 물고 달린다★.
        //  (driving 의 end_lidar_zone 도 0 을 한 번 강제로 내 이중으로 막는다)
        actuator_->releaseBrake();
        actuator_->keepBrake();
        last_commanded_v_.store(0.0, std::memory_order_relaxed);
        RCLCPP_INFO(
          get_logger(),
          "🛰️ 조종권 반환 — GPS 추종이 /cmd_vel_raw 를 다시 낸다. 이 노드는 침묵한다");
        was_permitted_ = false;
      }
      publishActive(false);      // ★값은 false 지만 신선도로 생존을 알린다★
      return;
    }
    if (!was_permitted_) {
      rearmReference();          // ★L 구간마다 기준선을 다시 잡는다★ (그쪽 주석)
      was_permitted_ = true;
    }
    publishActive(true);

    if (!costmap_->hasData()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for LiDAR data...");
      publishStop(/*apply_brake=*/false);
      return;
    }

    if (!gyro_bias_calibrated_.load(std::memory_order_acquire)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for IMU heading lock (%d / %d samples)... hold still. topic='%s'",
        gyro_bias_sample_count_.load(std::memory_order_relaxed),
        imu_bias_calibration_samples_, imu_topic_.c_str());
      publishStop(/*apply_brake=*/false);
      return;
    }

    // One lock-free snapshot for the whole planning cycle.
    const CostmapSnapshot snap = costmap_->snapshot();
    //  ★비켜 갈 쪽을 먼저 정한다★ (updateLateralTarget 주석의 실측 근거)
    updateLateralTarget(snap);
    if (!snap.valid) {
      publishStop(/*apply_brake=*/false);
      return;
    }

    maybeResetReference(snap);

    OdomPose current_pose;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      current_pose = odom_pose_;
    }

    // How many model steps elapsed since the last control cycle.
    // When frequency == 1/dt this is normally 1; if the loop lagged it can be >1.
    int shift_steps = 1;
    if (mppi_params_.dt > 1e-6 && elapsed > 0.0 && elapsed < 1.0) {
      shift_steps = std::max(1, static_cast<int>(std::lround(elapsed / mppi_params_.dt)));
      shift_steps = std::min(shift_steps, mppi_params_.horizon_steps);
    }

    const MPPIResult result = controller_->computeControl(current_pose, snap, shift_steps);
    const double avg_cost = result.min_cost /
      static_cast<double>(std::max(1, mppi_params_.horizon_steps));
    const bool latched_stop = updateStopLatch(result.stopped_for_collision);

    if (latched_stop) {
      const double held = stop_latch_t_.nanoseconds() == 0 ? 0.0 :
        (t_now - stop_latch_t_).seconds();
      const bool use_brake = brake_enable_ && held >= brake_after_s_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "MPPI stop latch: avg_cost=%.1f th=%.1f  held=%.1fs  brake=%d",
        avg_cost, mppi_params_.stop_cost_threshold, held, use_brake ? 1 : 0);
      publishStop(use_brake);
      publishRolloutPath();
      publishReferencePath(current_pose);
      return;
    }

    // 직진은 순항 펄스(2). 회피 중(|조향| 큼)에도 ★2펄스★ — 1펄스는 이 차가
    // 거의 움직이지 않는다(cmd.dodge_pulse 선언부의 근거).
    //  ══════════════════════════════════════════════════════════════════
    //  ★[2026-09-11] 조향을 ★기하로 직접★ 만든다 (사용자 지시)★
    //  ══════════════════════════════════════════════════════════════════
    //  ★왜 MPPI 를 안 쓰는가 — 세 번 고쳤는데 세 번 다 같은 실패였다★
    //  20260910_234828 / 20260911_001017 / 20260911_001853 이 전부 같다:
    //    콘을 지나 차 옆에 붙는 순간 ld_y −1.5 → −2.5, yaw −34° → −43°,
    //    ★road 가 0 으로 죽고★ cost 가 3000 을 넘어 정지 래치까지 걸린다.
    //  원인은 매번 같다 — ★되돌아가는 롤아웃만 충돌비용을 먹는다.★ 콘이 차
    //  왼쪽에 있으면 왼쪽으로 도는 궤적이 차체를 그 콘 쪽으로 쓸고 지나가기
    //  때문이다. costmap 클리어를 넓혀도 ★팽창(inflation)은 남아서★ 같은 일이
    //  반복된다. 앞을 더 넓게 지우면 이번엔 피해야 할 콘까지 지운다.
    //
    //  ★그런데 이 문제는 애초에 최적화기를 쓸 문제가 아니다★
    //   · 기준선을 정확히 안다 (/lidar_ref — GPS 매핑 중심선)
    //   · 콘의 위치를 안다 (nearestObstacle)
    //   · 해야 할 동작이 정해져 있다 — "콘의 반대쪽 1 m 로 지나고, 지나치는
    //     즉시 기준선으로 돌아온다"(사용자). 고를 것이 없다.
    //  → ★목표 횡위치 y_t 를 기하로 정하고(updateLateralTarget), 거기로
    //    스탠리로 붙인다.★ braketest.py 가 같은 식으로 직선을 따라가고 있고
    //    실차에서 검증됐다(±0.4 m). MPPI 는 ★비상정지 판정★ 으로만 남는다
    //    (stop latch — 정말 막혔으면 그쪽이 세운다).
    //
    //  ★avoid.geometric_steer:=false 로 종전(MPPI 조향)으로 되돌릴 수 있다.★
    double raw_steer_deg = result.control.delta * 180.0 / M_PI;
    if (geometric_steer_) {
      raw_steer_deg = geometricSteerDeg(current_pose);
    }
    const double steer_deg = filterSteer(raw_steer_deg);
    const int cruise = std::max(1, actuator_->maxPulse());
    const int ref = (std::abs(steer_deg) > dodge_steer_deg_)
                      ? std::min(dodge_pulse_, cruise) : cruise;
    const int out = lowSpeedTrim(ref);
    publishDrive(out, steer_deg);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "drive ref=%d → out=%d pulse (%.2f m/s, enc %.1f)  "
      "mppi_steer=%.1f → out=%.1f deg  avg_cost=%.0f  "
      "| 기준선 %s y=%.2f m yaw=%.1f deg",
      ref, actuator_->lastPulse(), lidar::kasa::pulseToMs(actuator_->lastPulse()),
      enc_pulse_.load(std::memory_order_relaxed),
      result.control.delta * 180.0 / M_PI, steer_deg, avg_cost,
      gps_ref_live_ ? "★GPS★" : "추측항법",
      current_pose.y, current_pose.yaw * 180.0 / M_PI);

    publishDiag(current_pose, steer_deg, avg_cost, snap);
    publishRolloutPath();
    publishReferencePath(current_pose);
  }

  bool updateStopLatch(bool raw_stop)
  {
    if (raw_stop) {
      ++stop_enter_count_;
      stop_exit_count_ = 0;
    } else {
      ++stop_exit_count_;
      stop_enter_count_ = 0;
    }
    if (!stop_latched_ && stop_enter_count_ >= stop_enter_frames_) {
      stop_latched_ = true;
      stop_latch_t_ = now();
    } else if (stop_latched_ && stop_exit_count_ >= stop_exit_frames_) {
      stop_latched_ = false;
      stop_latch_t_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
    return stop_latched_;
  }

  double filterSteer(double road_deg)
  {
    const double dt = 1.0 / std::max(1.0, control_frequency_);
    steer_filt_deg_ = steer_lpf_alpha_ * road_deg
                    + (1.0 - steer_lpf_alpha_) * steer_filt_deg_;
    const double max_d = steer_slew_deg_s_ * dt;
    // 슬루는 필터 상태 기준. 불감대로 last_pub 을 0 에 묶으면
    // 다음 틱도 1.2° 아래에서 영원히 조향 0 이 된다.
    steer_filt_deg_ = std::clamp(
      steer_filt_deg_, last_pub_steer_deg_ - max_d, last_pub_steer_deg_ + max_d);
    last_pub_steer_deg_ = steer_filt_deg_;
    if (std::abs(steer_filt_deg_) < steer_deadband_deg_) {
      return 0.0;
    }
    return steer_filt_deg_;
  }

  void publishRolloutPath()
  {
    const auto trajectory = controller_->getLastRolloutTrajectory();
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = base_frame_id_;
    path.poses.reserve(trajectory.size());
    for (const auto & s : trajectory) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = s.x;
      ps.pose.position.y = s.y;
      ps.pose.orientation = yawToQuaternion(s.yaw);
      path.poses.push_back(ps);
    }
    path_pub_->publish(path);
  }

  /// ★비켜 갈 자리를 정한다★ → mppi_params_.lateral_target [m, + 왼쪽]
  /// [2026-09-11 신설 — 사용자 지시 '궤적 기준 살짝만 틀어진다']
  ///
  /// ★왜 필요한가 (실측)★ route_20260910_102050-20260910_231626 :
  ///   인계 시 차 y −0.36(중심선 오른쪽), 장애물 4.1m ★정면★
  ///   → 플래너가 ★왼쪽★ 으로 +1.4 → +18.2° (장애물 쪽으로!)
  /// 비용함수에 '어느 쪽으로 비킬까' 라는 개념이 없어서, 좌우가 대칭일 때
  /// ★경로항(y 를 0 으로 당기는 힘)이 타이브레이커★ 가 된다. 차가 오른쪽에
  /// 있으면 그 힘이 왼쪽이라 장애물 쪽으로 먼저 꺾는다.
  ///
  /// ★규칙 — 중심선에서 제일 덜 벗어나는 쪽으로 비킨다★
  ///   여유 clear = 차 반폭 + 콘 반폭 + 여유
  ///   오른쪽 통과 : y = obs_y − clear     왼쪽 통과 : y = obs_y + clear
  ///   → |y| 가 작은 쪽을 고른다. 라바콘이 왼쪽(+)이면 오른쪽 통과가 자동으로
  ///     선택된다. ★사용자가 말한 'S자' 가 이 규칙 하나에서 나온다★ —
  ///     콘이 좌우로 번갈아 놓이면 목표도 좌우로 번갈아 잡힌다.
  ///
  /// ★정면일 때(좌우 대칭)는 차가 이미 있는 쪽으로 간다★ 그래야 장애물 앞을
  /// 가로지르지 않는다. 위 실측이 정확히 이 경우였고, 이 한 줄이 그것을 고친다.
  ///
  /// ★한 번 정하면 그 장애물을 지날 때까지 유지한다★ 매 틱 다시 고르면 콘이
  /// 좌우 경계에 있을 때 목표가 왕복해 조향이 떨린다(래치).
  void updateLateralTarget(const CostmapSnapshot & snap)
  {
    //  ★콘을 하나씩 분리해 ★거리 순★ 으로 본다★ (detectCones 주석의 근거)
    //  앞쪽(pass_x 이상)에 있는 것 중 ★가장 가까운 콘★ 이 지금 상대다.
    //  #1 을 지나면 목록에서 빠지고 #2 가 자동으로 첫 번째가 된다.
    const std::vector<ConeObs> cones = detectCones(snap);
    double ox = std::numeric_limits<double>::quiet_NaN();
    double oy = std::numeric_limits<double>::quiet_NaN();
    for (const auto & c : cones) {
      if (c.x < lat_pass_x_ || std::abs(c.y) > cone_y_max_) continue;
      ox = c.x; oy = c.y; break;                 // 이미 x 오름차순이다
    }
    n_cones_ahead_ = 0;
    for (const auto & c : cones) {
      if (c.x >= lat_pass_x_ && c.x <= lat_target_range_ &&
          std::abs(c.y) <= cone_y_max_) ++n_cones_ahead_;
    }
    //  ══════════════════════════════════════════════════════════════════
    //  ★지나친 콘은 판단에서 뺀다 [2026-09-11 — 사용자 지시]★
    //  ══════════════════════════════════════════════════════════════════
    //  "라바콘을 지나치는 ★즉시★ 그 반대쪽으로 꺾는다. 그대로 벗어나지 말고."
    //  ★앞차축(1.25 m)을 지난 콘은 조향으로 피할 대상이 아니다★ — 앞바퀴가
    //  이미 지나갔다. 그런데 종전에는 그 콘이 사거리(8 m) 안이라는 이유로
    //  ★계속 래치를 붙들고 있어서★, 차가 목표를 한참 넘어 나가도 목표가
    //  갱신되지 않았다. 실측(20260911_001017)에서 목표 −0.39 인데 −2.31 까지
    //  갔고, 그 콘이 costmap 에 남아 복귀 방향 롤아웃만 비용을 먹었다.
    //  → 앞쪽에 있는 콘만 본다. 지나친 순간 래치가 풀리고, 다음 콘(반대쪽)이나
    //    중심선으로 목표가 ★그 틱에 바로★ 바뀐다.
    const bool seen = std::isfinite(ox) && ox <= lat_target_range_;

    if (!seen) {
      //  ★지나갔다 — 즉시 반대쪽(복귀)으로★ 다음 콘이 아직 안 보이면 중심선이다.
      //  다음 콘이 보이면 아래에서 그 콘 기준으로 새 목표가 잡힌다.
      if (lat_latched_) {
        RCLCPP_INFO(
          get_logger(),
          "🛞 콘 통과 — 래치 해제, 목표를 %+.2f → 0.00 m 로 (즉시 복귀)",
          mppi_params_.lateral_target);
      }
      lat_latched_ = false;
      mppi_params_.lateral_target = 0.0;
      return;
    }
    //  ══════════════════════════════════════════════════════════════════
    //  ★래치는 '이 콘' 에 대한 것이다 — 콘이 바뀌면 다시 정한다 [2026-09-11]★
    //  ══════════════════════════════════════════════════════════════════
    //  ★종전 버그★ 해제 조건이 '앞에 콘이 하나도 없을 때' 였다. 그런데 콘 간격이
    //  5~8 m 이고 사거리가 8 m 라 ★항상 다음 콘이 보인다★ — 래치가 영영 안 풀린다.
    //  시뮬레이션(콘 3개, 6.5 m 간격 좌우 교대)에서 첫 콘 목표 −0.59 를 슬라럼
    //  내내 붙들고, 두 번째 콘(y −0.50)을 ★같은 쪽으로 지나갔다.★
    //  실차에서 "지나쳐도 안 돌아온다" 로 보이던 것의 정체가 이것이다.
    //
    //  ★콘이 바뀐 것을 어떻게 아는가★ 다가가는 동안 ox 는 계속 ★줄어든다.★
    //  그것이 ★늘어나면★ 보고 있던 콘을 지나쳐 다음 콘을 새로 잡은 것이다.
    //  (GPS·라이다 잡음으로 조금 늘 수 있으므로 문턱을 둔다.)
    if (lat_latched_) {
      if (ox <= lat_locked_ox_ + lat_new_cone_dx_) {
        lat_locked_ox_ = std::min(lat_locked_ox_, ox);
        //  ══════════════════════════════════════════════════════════════
        //  ★쪽만 고정한다 — 목표 크기는 매 틱 다시 잰다 [2026-09-11]★
        //  ══════════════════════════════════════════════════════════════
        //  ★왜★ cone_y = y + ox·sin(ψ) + oy·cos(ψ) 에서 ★ox 가 지렛대★ 다.
        //  8 m 앞 콘을 헤딩오차 5° 로 보면 위치가 0.70 m 틀린다. 그런데 래치를
        //  값까지 걸면 ★제일 멀 때(= 제일 부정확할 때) 정한 값을 끝까지 쓴다.★
        //  시뮬에서 콘 #2 목표가 +0.59 여야 하는데 +0.46 으로 나와 여유가
        //  1.09 → 0.96 m 로 깎였다.
        //  → 좌/우 ★결정★ 만 유지하고(그것이 왕복을 막는 목적이다), 목표
        //    위치는 콘이 가까워질수록 ★계속 정확해지게★ 다시 계산한다.
        const double clear2 = 0.5 * vehicle_params_.track_width
                              + std::max(lat_cone_half_, lat_inflation_) + lat_margin_;
        const double so2 = std::sin(odom_pose_.yaw), co2 = std::cos(odom_pose_.yaw);
        const double cone_y2 = odom_pose_.y + ox * so2 + oy * co2;
        mppi_params_.lateral_target = std::clamp(
          cone_y2 + lat_last_side_ * clear2, -lat_max_offset_, lat_max_offset_);
        return;                       // 같은 콘 — 정한 쪽을 지킨다
      }
      RCLCPP_INFO(
        get_logger(),
        "🛞 다음 콘 — %.2f m 에서 %.2f m 로 멀어졌다(이전 콘 통과). 방향을 다시 정한다",
        lat_locked_ox_, ox);
      lat_latched_ = false;           // 새 콘이다 — 아래에서 다시 정한다
    }
    //  ══════════════════════════════════════════════════════════════════
    //  ★여유는 ★비용함수가 요구하는 값★ 에서 유도한다 [2026-09-11 수정]★
    //  ══════════════════════════════════════════════════════════════════
    //  종전에는 반폭 + 콘반폭 + 여유 = 0.99 m 였는데, ★그것이 폭주의 원인이었다.★
    //  비용함수가 실제로 요구하는 것은 다르다:
    //    · 풋프린트 앞 모서리가 ±half_w(0.54) 에 있고 (vehicle_model.hpp)
    //    · 코스트맵이 장애물을 inflation_radius(0.40) 만큼 부풀린다
    //    → 차 중심이 콘에서 ★0.94 m★ 안이면 비용이 붙는다
    //  0.99 는 그 경계에서 ★5 cm★ 떨어져 있을 뿐이라, 플래너가 목표 지점에서도
    //  잔여 비용을 보고 계속 밖으로 나간다. 실측(20260910_234828): 목표 −0.49
    //  인데 ★−2.04★ 까지 갔고 그 지점 avg_cost 가 3084 였다.
    //  ★그래서 팽창반경을 그대로 넣는다★ — 콘 반폭은 이미 팽창에 포함돼 있으므로
    //  둘 중 큰 쪽만 센다(이중계산 방지).
    const double clear = 0.5 * vehicle_params_.track_width
                         + std::max(lat_cone_half_, lat_inflation_) + lat_margin_;
    //  ══════════════════════════════════════════════════════════════════
    //  ★콘을 ★기준선 좌표★ 로 옮긴 뒤에 판정한다 [2026-09-11 — 프레임 버그]★
    //  ══════════════════════════════════════════════════════════════════
    //  detectCones 의 (ox, oy) 는 ★차체 기준★ 이다(코스트맵이 ego 중심).
    //  그런데 lateral_target 은 ★기준선 기준★ 으로 쓰인다
    //  (컨트롤러: cross_track = y_odom − lateral_target). 종전에는 차체 기준
    //  oy 로 좌우를 골라 놓고 그 값을 기준선 목표로 썼다 — ★차가 중심선에서
    //  벗어나 있을수록 판정이 틀어진다.★
    //  실측 상황 그대로의 예: 콘 #2 가 기준선 −0.50, 차가 −0.57 일 때
    //     차체 기준 : r −1.02 / l +1.16 → |r| 작다 → ★오른쪽(틀림)★
    //     기준선 기준: r −1.59 / l +0.59 → |l| 작다 → ★왼쪽(맞음)★
    //  왼쪽으로 가야 S 가 되는데 오른쪽을 골라 계속 같은 쪽으로 밀려 나갔다.
    //
    //  ★변환★ 차체 (ox, oy) → 기준선 횡좌표. 컨트롤러가 롤아웃을 옮길 때
    //  쓰는 식과 ★같은 식★ 이다 (y_odom = odom.y + s.x·sin + s.y·cos).
    const double so = std::sin(odom_pose_.yaw), co = std::cos(odom_pose_.yaw);
    const double cone_y = odom_pose_.y + ox * so + oy * co;   // ★기준선 기준★
    const double cand_r = cone_y - clear;      // 콘의 오른쪽으로 비킨다
    const double cand_l = cone_y + clear;      // 콘의 왼쪽으로 비킨다
    double target;
    //  ★|목표| 가 작은 쪽 = 중심선에서 제일 덜 벗어나는 쪽★ (둘 다 기준선 기준)
    if (std::abs(std::abs(cand_r) - std::abs(cand_l)) < 1e-3) {
      //  ★좌우가 같다(콘이 정면) — 이때만 다른 근거가 필요하다★
      //  ① 직전 콘을 지난 반대쪽 : 사용자가 말한 배치 그대로다 —
      //     "첫 콘 왼쪽이면 그 오른쪽 통과, 다음 콘은 왼쪽 통과" = ★교대★.
      //     콘이 좌우로 번갈아 서 있으면 |target| 비교만으로도 교대가 나오지만,
      //     정면이라 좌우가 같을 때는 그 비교가 답을 못 준다. 그때 이것이 정한다.
      //  ② 직전 정보가 없으면 차가 이미 있는 쪽 — 앞을 가로지르지 않는다.
      if (lat_last_side_ != 0) {
        target = (lat_last_side_ > 0) ? cand_r : cand_l;   // 지난번 반대쪽
      } else {
        target = (odom_pose_.y >= 0.0) ? cand_l : cand_r;
      }
    } else {
      target = (std::abs(cand_r) <= std::abs(cand_l)) ? cand_r : cand_l;
    }
    //  ★횡벽(max_lateral_offset)이 아니라 전용 상한으로 자른다 [2026-09-11]★
    //  같은 값을 쓰면 회피 목표가 정확히 벽 위에 앉아 ★서로 밀어낸다★ —
    //  목표로 가려는 힘과 벽이 되미는 힘이 같은 자리에서 싸운다.
    //  벽은 목표보다 바깥에 있어야 한다(params.yaml 의 두 값을 함께 볼 것).
    target = std::clamp(target, -lat_max_offset_, lat_max_offset_);
    mppi_params_.lateral_target = target;
    lat_latched_ = true;
    lat_locked_ox_ = ox;                        // 이 콘을 붙들었다 (멀어지면 새 콘)
    lat_last_side_ = (target < cone_y) ? -1 : +1;  // 콘 대비 어느 쪽으로 지나는가
    RCLCPP_INFO(
      get_logger(),
      "🛞 회피 방향 결정 — 콘 %.2f m 앞, 기준선 y=%+.2f m (중심선 %s) → "
      "★콘의 %s 으로 통과★ 목표 y=%+.2f m (콘과 %.2f m, 필요 %.2f m)",
      ox, cone_y, cone_y >= 0.0 ? "왼쪽" : "오른쪽",
      //  ★'어느 쪽 통과' 는 목표의 부호가 아니라 ★콘 대비★ 로 판정한다★
      //  콘이 +1.0 이고 목표가 +0.01 이면 목표는 양수지만 콘의 '오른쪽' 이다.
      target < cone_y ? "오른쪽" : "왼쪽", target, std::abs(target - cone_y), clear);
  }

  /// ★목표 횡위치로 붙이는 기하 조향★ → 도로휠각 [deg, + = 좌]
  /// [2026-09-11 신설 — 사용자 지시. 근거는 controlLoop 의 호출부 주석]
  ///
  ///     e       = y − y_target        (+ = 목표보다 왼쪽)
  ///     ψ_err   = yaw                 (+ = 기준선보다 왼쪽을 향함)
  ///     δ(+우)  = K_psi·ψ_err + atan(K_cte·e / max(v, v_min))
  ///     δ(+좌)  = −δ(+우)
  ///
  /// ★braketest.py 와 같은 식이다★ 그쪽은 실차에서 검증됐다(직선 ±0.4 m).
  /// 방향항(ψ)이 감쇠를, 위치항(e)이 수렴을 맡는다 — 위치항만 두면 조향→헤딩→
  /// 횡위치가 2중적분이라 감쇠가 없어 발산한다(braketest 에서 실측으로 확인).
  ///
  /// ★저속이라 스탠리 항이 세게 먹는다 — 그것이 여기서는 맞다★
  /// v ≈ 1.77 m/s 에서 e = 1.0 m 면 atan(0.6·1.0/1.77) = 18.7°.
  /// "지나치는 즉시 반대쪽으로 획 꺾는다"(사용자)가 이 항 하나에서 나온다.
  double geometricSteerDeg(const OdomPose & pose) const
  {
    const double v = std::max(geo_v_min_,
                              lidar::kasa::pulseToMs(std::max(1, actuator_->maxPulse())));
    const double e = pose.y - mppi_params_.lateral_target;   // + = 목표보다 왼쪽
    const double psi_deg = pose.yaw * 180.0 / M_PI;          // + = 기준선보다 왼쪽
    const double right_deg = geo_k_psi_ * psi_deg
                             + std::atan(geo_k_cte_ * e / v) * 180.0 / M_PI;
    const double max_deg = vehicle_params_.max_steering_angle * 180.0 / M_PI;
    return std::clamp(-right_deg, -max_deg, max_deg);        // + = 좌
  }

  struct ConeObs { double x, y; int cells; };

  /// ★라바콘을 하나씩 분리해서 각자의 거리를 낸다★ (x 오름차순)
  /// [2026-09-11 신설 — 사용자 지적: "거리는 안 보고 있다/없다만 보면 근본적인
  ///  부정확함을 해결하지 못한다"]
  ///
  /// ★종전 nearestObstacle 의 결함★ 그것은 '비용이 붙은 가장 가까운 ★셀★' 을
  /// 돌려줬다. 그런데 inflate() 가 콘 하나를 반경 0.63 m 치사 원반 + 0.40 m
  /// 감쇠링으로 부풀리므로, 그 셀은 ★콘의 위치가 아니라 팽창 가장자리★ 다.
  /// 실측(20260911_001853)에서 obs 가 (0.3, +1.0) 으로 잡혔는데, 그것은 훨씬
  /// 바깥에 있던 콘의 팽창이었다 — 그 값으로 목표를 정하니 맞을 수가 없다.
  ///
  /// ★치사 셀만 묶으면 콘 위치가 정확히 나온다★ 치사 원반은 콘을 중심으로
  /// 대칭이므로 ★군집의 무게중심 = 콘의 중심★ 이다. 인접(8-이웃) 치사 셀을
  /// 묶어 군집마다 무게중심을 낸다.
  ///
  /// ★이것이 있어야 '콘마다 따로' 가 성립한다★ 콘 #1 과 #2 가 각자의 x 를
  /// 가지므로, #1 을 지나면 목록에서 빠지고 #2 가 자동으로 첫 번째가 된다 —
  /// '있다/없다' 가 아니라 ★거리 순서★ 로 다음 콘이 정해진다.
  std::vector<ConeObs> detectCones(const CostmapSnapshot & snap) const
  {
    std::vector<ConeObs> out;
    if (!snap.valid || snap.cells_x <= 0 || snap.cells_y <= 0) return out;
    const double res = snap.resolution;
    const double thr = EgoCostmap::kLethalCost * 0.99;   // ★치사 셀만★
    const size_t n = static_cast<size_t>(snap.cells_x) * static_cast<size_t>(snap.cells_y);
    std::vector<uint8_t> seen(n, 0);
    std::vector<int> stack;
    auto idx = [&](int ix, int iy) {
      return static_cast<size_t>(iy) * static_cast<size_t>(snap.cells_x) +
             static_cast<size_t>(ix);
    };
    for (int iy = 0; iy < snap.cells_y; ++iy) {
      for (int ix = 0; ix < snap.cells_x; ++ix) {
        const size_t i0 = idx(ix, iy);
        if (seen[i0] || snap.cost[i0] < thr) continue;
        //  8-이웃 flood fill — 한 콘의 치사 원반이 한 군집이 된다
        double sx = 0.0, sy = 0.0; int cnt = 0;
        stack.clear(); stack.push_back(static_cast<int>(i0)); seen[i0] = 1;
        while (!stack.empty()) {
          const int cur = stack.back(); stack.pop_back();
          const int cx = cur % snap.cells_x, cy = cur / snap.cells_x;
          sx += -snap.size_x / 2.0 + (cx + 0.5) * res;
          sy += -snap.size_y / 2.0 + (cy + 0.5) * res;
          ++cnt;
          for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
              const int nx = cx + dx, ny = cy + dy;
              if (nx < 0 || nx >= snap.cells_x || ny < 0 || ny >= snap.cells_y) continue;
              const size_t ni = idx(nx, ny);
              if (seen[ni] || snap.cost[ni] < thr) continue;
              seen[ni] = 1; stack.push_back(static_cast<int>(ni));
            }
          }
        }
        if (cnt >= cone_min_cells_) out.push_back({sx / cnt, sy / cnt, cnt});
      }
    }
    std::sort(out.begin(), out.end(),
              [](const ConeObs & a, const ConeObs & b) { return a.x < b.x; });
    return out;
  }

  /// 전방 코리도 안에서 ★가장 가까운 장애물★ 을 찾는다 → (x, y). 없으면 NaN.
  /// 회피가 잘 됐는지는 결국 '무엇을 얼마나 비켜 갔나' 이므로 이 둘이 핵심이다.
  void nearestObstacle(const CostmapSnapshot & snap, double & ox, double & oy) const
  {
    ox = oy = std::numeric_limits<double>::quiet_NaN();
    if (!snap.valid) return;
    double best = std::numeric_limits<double>::infinity();
    for (double x = 0.3; x <= 8.0; x += snap.resolution) {
      for (double y = -3.0; y <= 3.0; y += snap.resolution) {
        if (snap.getCost(x, y) < 50.0) continue;      // 빈 칸은 건너뛴다
        const double d = std::hypot(x, y);
        if (d < best) { best = d; ox = x; oy = y; }
      }
    }
  }

  /// ★L 구간 전용 진단★ — 이 함수는 실제로 몰고 있을 때만 불린다.
  /// 배열 규약(record.py 가 같은 순서로 읽는다):
  ///   [0] y_m      기준선 대비 횡오차 [m]  + 왼쪽
  ///   [1] yaw_deg  기준선 대비 방위오차 [deg] + 왼쪽
  ///   [2] road_deg 플래너가 낸 도로휠각 [deg] + 왼쪽
  ///   [3] pot_deg  실제 발행 pot [deg] ★− 좌 / + 우 (보드 규약)★
  ///   [4] pulse    실제 발행 펄스
  ///   [5] avg_cost MPPI 평균 비용 (정지 게이트 임계와 비교해서 읽는다)
  ///   [6] gps_ref  1 = 기준선이 GPS, 0 = 추측항법
  ///   [7] obs_x    최근접 장애물 전방거리 [m] (라이다 원점 기준). 없으면 NaN
  ///   [8] obs_y    그 콘의 횡위치 [m] + 왼쪽. ★부호가 곧 '어느 쪽 라바콘인가'★
  ///   [9] target_y ★지금 겨누는 횡목표★ [m] + 왼쪽 (updateLateralTarget)
  ///  [10] n_cones  앞에 보이는 콘 개수 — 0 이면 복귀 구간이다
  void publishDiag(const OdomPose & pose, double road_deg, double avg_cost,
                   const CostmapSnapshot & snap)
  {
    //  ★진단도 군집 중심을 쓴다 [2026-09-11]★ 종전 nearestObstacle 은 팽창
    //  가장자리를 돌려줘서, 기록된 obs_x/obs_y 가 콘의 실제 위치가 아니었다
    //  (실측 20260911_001853 의 (0.3,+1.0) 이 그것이다 — 훨씬 바깥 콘의 팽창).
    double ox = std::numeric_limits<double>::quiet_NaN();
    double oy = std::numeric_limits<double>::quiet_NaN();
    {
      const std::vector<ConeObs> cs = detectCones(snap);
      for (const auto & c : cs) {
        if (std::abs(c.y) > cone_y_max_) continue;
        ox = c.x; oy = c.y; break;              // x 오름차순 — 가장 가까운 콘
      }
    }
    std_msgs::msg::Float64MultiArray m;
    m.data = {
      pose.y, pose.yaw * 180.0 / M_PI, road_deg,
      actuator_->lastPotDeg(), static_cast<double>(actuator_->lastPulse()),
      avg_cost, gps_ref_live_ ? 1.0 : 0.0, ox, oy,
      mppi_params_.lateral_target, static_cast<double>(n_cones_ahead_)};
    diag_pub_->publish(m);
  }

  void publishReferencePath(const OdomPose & odom_pose)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = base_frame_id_;

    const double cos_o = std::cos(odom_pose.yaw);
    const double sin_o = std::sin(odom_pose.yaw);
    for (double dx = -3.0; dx <= 8.0; dx += 0.5) {
      const double dy = 0.0 - odom_pose.y;
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = dx * cos_o + dy * sin_o;
      ps.pose.position.y = -dx * sin_o + dy * cos_o;
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    reference_path_pub_->publish(path);
  }

  // Parameters
  std::string lidar_topic_, imu_topic_, cmd_vel_topic_;
  std::string costmap_topic_, path_topic_, reference_path_topic_, base_frame_id_;
  double control_frequency_ = 20.0;
  VehicleParams vehicle_params_;
  CostmapParams costmap_params_;
  MPPIParams mppi_params_;

  // State
  std::unique_ptr<EgoCostmap> costmap_;
  std::unique_ptr<MPPIController> controller_;
  std::unique_ptr<lidar::kasa::KasaActuator> actuator_;
  bool flip_lidar_xy_ = true;
  bool brake_enable_ = true;
  double sensor_height_m_ = 1.17;
  double roi_agl_min_ = 0.20;
  double roi_agl_max_ = 1.50;

  int stop_enter_frames_ = 8;
  int stop_exit_frames_ = 12;
  int stop_enter_count_ = 0;
  int stop_exit_count_ = 0;
  bool stop_latched_ = false;
  rclcpp::Time stop_latch_t_{0, 0, RCL_ROS_TIME};
  double brake_after_s_ = 1.2;
  double steer_lpf_alpha_ = 0.35;
  double steer_slew_deg_s_ = 28.0;
  double steer_deadband_deg_ = 0.0;
  double dodge_steer_deg_ = 6.0;
  int    dodge_pulse_     = 2;      // ★회피 중 펄스 (1 은 이 차가 안 움직인다)★
  bool   trim_enable_     = true;   // 저속 기동 보정 [2026-09-11]
  int    trim_max_pulse_  = 2;
  int    trim_ref_max_    = 3;
  int    trim_out_max_    = 6;
  double trim_hold_s_     = 0.3;
  int    trim_            = 0;      // 지금 얹고 있는 보정량
  int    trim_ref_        = -1;     // 그 보정을 정할 때의 REF
  double trim_t_          = 0.0;
  std::atomic<double> enc_pulse_{0.0};   // 실측 [펄스, 바퀴 하나 기준]
  std::atomic<double> enc_t_{0.0};
  std::mutex enc_mutex_;
  double enc_buf_[3] = {0.0, 0.0, 0.0};
  unsigned enc_i_ = 0;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr encoder_sub_;
  //  ★GPS 기준선 [2026-09-11]★ (odom_mutex_ 가 지킨다)
  std::string ref_topic_ = "/lidar_ref";
  double ref_stale_s_ = 0.5;
  bool   use_gps_ref_ = true;
  bool   ref_valid_ = false;
  bool   gps_ref_live_ = false;
  double ref_t_ = 0.0, ref_cte_ = 0.0, ref_herr_ = 0.0;
  double ref_zone_left_ = std::numeric_limits<double>::quiet_NaN();
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ref_sub_;
  std::string diag_topic_ = "/lidar_diag";
  //  ★회피 방향 결정 [2026-09-11]★
  double lat_target_range_ = 8.0, lat_cone_half_ = 0.20, lat_margin_ = 0.25;
  double lat_max_offset_ = 1.25;  // 회피 목표 상한 (횡벽과 ★다른 값★)
  double lat_inflation_ = 0.40;   // 코스트맵 팽창반경 (같은 값을 두 곳에 둔다)
  double lat_pass_x_ = 0.90;      // 이보다 가까우면 "지나쳤다"
  bool   geometric_steer_ = true; // ★조향을 기하로 만든다 [2026-09-11]★
  double geo_k_psi_ = 1.0, geo_k_cte_ = 0.6, geo_v_min_ = 1.0;
  int    lat_last_side_ = 0;      // 직전 콘을 어느 쪽으로 지났나 (−1 우 / +1 좌)
  bool   lat_latched_ = false;   // 이 장애물에 대해 쪽을 정했나
  double lat_locked_ox_ = 0.0;   // 붙들고 있는 콘까지의 거리 (늘면 새 콘)
  double lat_new_cone_dx_ = 1.5; // 이만큼 멀어지면 다른 콘으로 본다
  int    cone_min_cells_ = 20;   // 군집 최소 셀 수 (잡음 제거)
  double cone_y_max_ = 3.0;      // 이보다 옆의 콘은 무시
  int    n_cones_ahead_ = 0;     // 진단용 — 앞에 콘이 몇 개 보이나
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr diag_pub_;
  double steer_filt_deg_ = 0.0;
  double last_pub_steer_deg_ = 0.0;
  std::mutex odom_mutex_;
  OdomPose odom_pose_;
  bool imu_use_orientation_ = true;
  bool imu_heading_from_quat_ = false;
  double heading_ref_yaw_ = 0.0;
  bool imu_initialized_ = false;
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  std::atomic<double> last_commanded_v_{0.0};
  std::string last_cloud_frame_id_;
  bool static_tf_published_ = false;
  int imu_bias_calibration_samples_ = 100;
  double gyro_bias_sum_ = 0.0;
  std::atomic<int> gyro_bias_sample_count_{0};
  double gyro_bias_wz_ = 0.0;
  std::atomic<bool> gyro_bias_calibrated_{false};
  bool reference_reset_enable_ = true;
  double reference_reset_clear_seconds_ = 1.5;
  double reference_reset_blocked_seconds_ = 0.5;
  double reference_reset_check_distance_ = 3.0;
  double reference_reset_check_half_width_ = 0.8;
  double reference_reset_clear_threshold_ = 25.0;
  double reference_reset_blocked_threshold_ = 55.0;
  double reference_reset_cost_ema_alpha_ = 0.25;
  double reference_reset_min_interval_ = 5.0;
  double reference_reset_return_y_ = 0.10;
  double reference_reset_return_yaw_ = 0.08;
  double reference_reset_return_hold_ = 1.5;
  bool reference_reset_soft_enable_ = false;
  bool reference_reset_preserve_heading_ = true;
  bool reference_reset_preserve_lateral_ = true;
  double clear_ahead_seconds_ = 0.0;
  double blocked_ahead_seconds_ = 0.0;
  double path_return_hold_seconds_ = 0.0;
  double corridor_cost_ema_ = 0.0;
  bool corridor_cost_ema_init_ = false;
  bool corridor_clear_latched_ = false;
  rclcpp::Time last_reference_reset_time_{0, 0, RCL_ROS_TIME};

  // ── 조종권 계약 [2026-09-01] (생성자의 상자 참고) ──
  int cruise_pulse_ = 2;                    // ★순항 단일 소유자★ (0 = 개별 파라미터)
  std::string lstatus_topic_ = "/lstatus";
  std::string active_topic_ = "/lidar_active";
  bool require_lstatus_ = true;
  double lstatus_stale_s_ = 1.0;
  std::atomic<char> lstatus_{'0'};          // 마지막으로 받은 구간 문자
  std::atomic<double> lstatus_stamp_{0.0};  // 마지막 수신 시각 [s] — 신선도 판정
  bool was_permitted_ = false;              // 상승엣지(재무장) 검출용
  //  ★절대 yaw 를 기억해 둔다★ 재무장 때 heading_ref_yaw_ 를 지금 방위로 갈아야
  //  하는데, imuCallback 안의 지역변수로만 두면 그 값을 꺼낼 수 없다.
  double last_abs_yaw_ = 0.0;
  bool has_abs_yaw_ = false;

  // ROS interfaces
  rclcpp::CallbackGroup::SharedPtr cloud_cb_group_;
  rclcpp::CallbackGroup::SharedPtr imu_cb_group_;
  rclcpp::CallbackGroup::SharedPtr control_cb_group_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lstatus_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

}  // namespace mppi_local_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mppi_local_planner::MPPILocalPlannerNode>();
  // Multi-threaded so cloud inflation and the control timer do not block each other.
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), /*number_of_threads=*/4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
