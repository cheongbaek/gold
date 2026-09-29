#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

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

#include "mppi_local_planner/cone_detector.hpp"
#include "mppi_local_planner/ego_costmap.hpp"
#include "mppi_local_planner/frenet_planner.hpp"
#include "mppi_local_planner/frenet_types.hpp"
#include "mppi_local_planner/kasa_units.hpp"
#include "mppi_local_planner/mppi_controller.hpp"
#include "mppi_local_planner/path_tracker.hpp"
#include "mppi_local_planner/vehicle_model.hpp"

#include <deque>
#include <filesystem>
#include <fstream>

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
    frenet_planner_ = std::make_unique<FrenetPlanner>(frenet_params_, vehicle_params_);
    param_cb_ = add_on_set_parameters_callback(
      std::bind(&MPPILocalPlannerNode::onSetParams, this, std::placeholders::_1));
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
    //  white1/driving 은 매핑 경로를 들고 있으므로 CTE 와 경로 방위를 안다.
    //  odom.y 는 CTE, odom.yaw 는 (차 헤딩 − 경로 방위) 다. 둘 다 매핑 중심선
    //  기준이다. ref 가 낡으면 yaw 는 OS1 적분으로 이어 간다.
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
    //  ★[2026-09-12] 콘 ★목록★ — diag 는 고정 길이라 가변 개수를 못 담는다★
    //  [x1,y1,cells1, x2,y2,cells2, ...] 3개씩 끊어 읽는다. 앞쪽(pass_x 이상) ·
    //  |y| ≤ cone_y_max 로 거른 ★회피 대상★ 만 담는다 — 진단 ox/oy 와 같은 기준.
    //  white1/record 가 이것을 받아 <주행CSV이름>.cones.csv 로 적는다.
    cones_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(cones_topic_, 10);

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
      "헤딩 IMU = '%s'  use_orientation=%s  yaw_sign=%.0f  "
      "(OS1 내장 자이로 적분. 쿼터니언 없음 → 바이어스 보정 후 wz 적분)",
      imu_topic_.c_str(), imu_use_orientation_ ? "true" : "false", imu_yaw_sign_);
    RCLCPP_INFO(
      get_logger(),
      "plan=Frenet d(s)  track=%s  plan_dist=%.1f m  geometric_steer=%s",
      geometric_steer_ ? "pure-pursuit" : "MPPI",
      frenet_params_.plan_distance_m,
      geometric_steer_ ? "true" : "false");
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
    declare_parameter<std::string>("imu_topic", "/ouster/imu");
    declare_parameter<bool>("imu_use_orientation", false);
    //  ★true 면 white1 런치가 /imu 를 넣어도 OS1 내장 자이로를 쓴다★
    declare_parameter<bool>("use_os1_imu", true);
    declare_parameter<double>("imu_yaw_sign", 1.0);
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
    declare_parameter<std::string>("cones_topic", "/lidar_cones");
    // ══════════════════════════════════════════════════════════════════
    //  ★[2026-09-28] 블록 계획기 — 근거는 frenet_planner.cpp 머리 상자★
    // ══════════════════════════════════════════════════════════════════
    declare_parameter<double>("avoid.range_m", 17.5);      // 차체 x 이 안의 점유 군집만
    declare_parameter<double>("avoid.max_offset_m", 3.0);  // ★GPS 궤적 ± 3 m★ (사용자)
    declare_parameter<double>("avoid.pass_gap_m", 0.75);   // 차체 옆면 ↔ 장애물 표면
    //  아래 둘은 /lidar_cones·진단 최근접 콘을 고르는 필터다(계획과 무관).
    declare_parameter<double>("avoid.pass_x_m", 0.90);
    declare_parameter<double>("avoid.cone_y_max_m", 4.0);
    //  ★기하 조향★ true 면 Frenet d(s) 를 순수추종으로 쫓고 MPPI delta 는 버린다.
    declare_parameter<bool>("avoid.geometric_steer", true);
    declare_parameter<double>("avoid.lookahead_m", 3.2);   // 순수추종 앞 점 [m]
    declare_parameter<double>("avoid.k_cte", 0.5);         // 뒤차축 횡오차 약보정
    declare_parameter<double>("avoid.steer_predict_s", 0.15);  // 조향 지연 보상 [s]
    declare_parameter<double>("avoid.k_i_deg", 2.0);       // 편향 적분 [deg/(m·s)]
    declare_parameter<double>("avoid.i_max_deg", 3.0);     // 적분 상한 [deg 도로휠]
    declare_parameter<double>("avoid.i_gate_m", 0.5);      // |오차| 가 이 안일 때만 쌓는다
    declare_parameter<double>("stop.predict_m", 3.0);      // 정지 판정 = 경로 추종 예측 거리
    //  ★인계 전 미리보기★ L 구간이 이 거리 안이면 조종권 없이 계획만 돌린다.
    declare_parameter<bool>("preview.enable", true);
    declare_parameter<double>("preview.zone_dist_m", 25.0);
    // ── Frenet 경로 계획 ──
    declare_parameter<bool>("frenet.enable", true);
    declare_parameter<double>("frenet.plan_distance_m", 16.0);
    declare_parameter<double>("frenet.ds_m", 0.25);
    declare_parameter<double>("frenet.cluster_link_m", 0.30);
    declare_parameter<int>("frenet.cluster_min_cells", 1);
    declare_parameter<double>("frenet.cluster_max_width_m", 5.0);
    declare_parameter<double>("frenet.block_link_s_m", 2.5);
    declare_parameter<double>("frenet.block_link_d_m", 1.9);
    declare_parameter<int>("frenet.confirm_hits", 2);
    declare_parameter<double>("frenet.lost_s", 2.5);
    declare_parameter<double>("frenet.blind_x_m", 2.2);
    declare_parameter<double>("frenet.assoc_gap_m", 1.0);
    declare_parameter<double>("frenet.hold_pre_m", 0.30);
    declare_parameter<double>("frenet.hold_post_m", 0.30);
    declare_parameter<double>("frenet.bias_deadband_m", 0.15);
    declare_parameter<double>("frenet.bias_deadband_per_m", 0.03);
    declare_parameter<double>("frenet.rel_margin_m", 0.40);
    declare_parameter<double>("frenet.commit_dist_m", 5.0);
    declare_parameter<int>("frenet.flip_ticks", 5);
    declare_parameter<double>("frenet.kappa_comfort", 0.125);
    declare_parameter<double>("frenet.kappa_max", 0.34);
    declare_parameter<double>("frenet.shift_len_min_m", 4.0);
    declare_parameter<double>("frenet.shift_len_max_m", 12.0);
    declare_parameter<double>("frenet.return_len_min_m", 6.0);
    declare_parameter<double>("frenet.return_len_max_m", 12.0);
    declare_parameter<double>("frenet.flat_min_m", 2.0);
    declare_parameter<double>("frenet.entry_len_min_m", 1.5);
    declare_parameter<double>("frenet.on_path_d_m", 0.30);
    declare_parameter<double>("frenet.on_path_yaw_rad", 0.15);
    declare_parameter<double>("handover.ref_stale_s", 0.5);
    //  ★GPS 안테나가 라이다 원점보다 이만큼 앞이다 [2026-09-29]★ driving 의 CTE 는 안테나
    //  자리라, 헤딩 ψ 에서 라이다 원점의 횡위치는 CTE − 이 값·sin ψ 다(근거는 imuCallback).
    declare_parameter<double>("handover.gps_antenna_x_m", 0.6);
    declare_parameter<bool>("handover.use_gps_ref", true);
    //  y(CTE) 만 GPS 로 덮는다. yaw 를 덮으면 외장 iAHRS 헤딩이 다시 들어온다.
    declare_parameter<bool>("handover.use_gps_yaw", true);
    //  ★false 로 두면 종전처럼 '런치 = 출발' 이다★ 라이다 단독 시험용. 실차에서
    //  white1 one_launch 로 띄울 때는 반드시 true 여야 한다 — 아니면 이 노드가
    //  GPS 추종 구간에서도 /cmd_vel_raw 를 내며 driving.py 와 다툰다.
    declare_parameter<bool>("handover.require_lstatus", true);
    //  허락이 이보다 낡으면 '허락 없음'. driving 은 20Hz 로 내므로 1.0s 는 20틱 여유.
    declare_parameter<double>("handover.lstatus_stale_s", 1.0);
    declare_parameter<std::string>("base_frame_id", "os_sensor");
    declare_parameter<bool>("debug.dump_enable", true);
    declare_parameter<std::string>("debug.dump_dir", "/tmp/mppi_debug");
    declare_parameter<int>("debug.dump_every_n", 10);

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
    declare_parameter<double>("cmd.steer_lpf_alpha", 0.70);
    declare_parameter<double>("cmd.steer_slew_deg_s", 36.0);
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
    declare_parameter<double>("roi_agl_min", 0.30);
    declare_parameter<double>("roi_agl_max", 1.50);
    // lidar flip_lidar_xy:true = os_lidar xy 180° = sensor_yaw_offset π.
    declare_parameter<bool>("flip_lidar_xy", true);
    declare_parameter<bool>("brake_enable", true);

    declare_parameter<double>("costmap.size_x", 36.0);
    declare_parameter<double>("costmap.size_y", 16.0);
    declare_parameter<double>("costmap.resolution", 0.1);
    declare_parameter<double>("costmap.ground_z_min", 0.0);  // 0 => AGL 슬랩에서 유도
    declare_parameter<double>("costmap.ground_z_max", 0.0);
    declare_parameter<double>("costmap.inflation_radius", 0.40);
    declare_parameter<double>("costmap.min_range", 0.40);
    declare_parameter<double>("costmap.sensor_offset_x", 0.0);
    declare_parameter<double>("costmap.sensor_offset_y", 0.0);
    declare_parameter<double>("costmap.sensor_yaw_offset", 0.0);  // 0 => flip_lidar_xy 로 결정
    declare_parameter<double>("costmap.occupancy_decay", 0.25);
    // cone_lidar roi_x_min=2.0 : 1.2~1.5 m 는 차체·탑승자. 여기도 2.0 m 까지 지운다.
    // 1.4 m 만 지우면 보닛 반사가 전방 벽이 되어 stop-gate → 리니어 2단이 뜬다.
    declare_parameter<double>("costmap.ego_clear_radius", 0.90);
    declare_parameter<double>("costmap.ego_clear_margin", 0.12);
    declare_parameter<double>("costmap.ego_clear_front_max", 2.00);
    declare_parameter<double>("costmap.ego_clear_x_min", -0.45);
    declare_parameter<double>("costmap.ego_clear_x_max", 1.95);
    declare_parameter<double>("costmap.ego_clear_y_half", 0.85);
    //  ★지나친 콘이 복귀를 막지 않게 [2026-09-11]★ (ego_costmap.hpp 주석의 실측)
    //  ★[2026-09-13] 1.30 → 0.90★ 1.30 은 앞차축(1.25) 앞이라, 아직 피해야 할
    //  둘째 콘을 맵에서 지웠다. avoid.pass_x 와 맞춰 앞바퀴가 지난 뒤에서만 넓힌다.
    declare_parameter<double>("costmap.ego_clear_pass_x", 0.90);
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
    declare_parameter<double>("mppi.obstacle_on_path_scale", 0.18);
    declare_parameter<double>("mppi.on_path_cte_m", 0.35);
    declare_parameter<double>("mppi.weight_path", 40.0);
    declare_parameter<double>("mppi.weight_heading", 8.0);
    declare_parameter<double>("mppi.weight_speed", 2.0);
    declare_parameter<double>("mppi.weight_smooth_v", 0.6);
    declare_parameter<double>("mppi.weight_smooth_delta", 0.85);
    declare_parameter<double>("mppi.stanley_lookahead", 4.0);
    // 구 S복귀 키는 yaml 호환으로만 선언한다. 컨트롤러는 쓰지 않는다.
    declare_parameter<double>("mppi.s_curve_dodge_frac", 0.0);
    declare_parameter<double>("mppi.s_curve_return_power", 1.0);
    declare_parameter<double>("mppi.path_progress_floor", 1.0);
    declare_parameter<double>("mppi.avoid_path_scale", 1.0);
    declare_parameter<double>("mppi.avoid_obs_gain", 80.0);
    declare_parameter<double>("mppi.offset_return_y", 99.0);
    declare_parameter<double>("mppi.offset_return_scale", 0.0);
    declare_parameter<double>("mppi.weight_return_clear", 0.0);
    declare_parameter<double>("mppi.return_clear_cost", 40.0);
    declare_parameter<double>("mppi.weight_path_terminal", 40.0);
    declare_parameter<double>("mppi.weight_heading_terminal", 20.0);
    declare_parameter<double>("mppi.max_lateral_offset", 1.90);
    declare_parameter<double>("mppi.weight_lateral_wall", 2000.0);
    //  ★완만한 S — 하드 벽 + 헤딩 벽 [2026-09-11]★ (근거는 MPPIParams 주석)
    declare_parameter<double>("mppi.lateral_hard", 2.5);
    declare_parameter<double>("mppi.weight_lateral_hard", 4000.0);
    declare_parameter<double>("mppi.max_heading_dev", 0.52);
    declare_parameter<double>("mppi.weight_heading_wall", 800.0);
    declare_parameter<double>("mppi.lookahead_distance", 7.0);
    declare_parameter<double>("mppi.lookahead_step", 0.40);
    declare_parameter<double>("mppi.weight_lookahead", 0.55);
    declare_parameter<double>("mppi.stop_cost_threshold", 750.0);

    declare_parameter<int>("imu_bias_calibration_samples", 100);  // OS1 ~100Hz → 1s
    // ══════════════════════════════════════════════════════════════════════
    //  ★[2026-09-12] 자이로 바이어스 자체 복구 (사용자 지시)★
    // ══════════════════════════════════════════════════════════════════════
    //  ★왜 필요한가★ 2026-09-12 두 주행(145723 · 150357)에서 이 노드가 믿은
    //  요레이트가 실제보다 ★-30°/s★ 어긋나 있었다. driving 의 실측 요레이트와
    //  회귀하면 기울기는 +0.98(축·부호 정상)인데 상수항만 -30.07 / -30.24 °/s 다.
    //  9/10 로그 11건은 -2.6 ~ +2.7 °/s 라 ★구조 결함이 아니라 그날의 캘리브
    //  실패★ 였다 — 두 주행의 값이 거의 같은 것은 같은 노드 세션의 바이어스를
    //  계속 쓴 탓이다(gyro_bias_calibrated_ 는 한 번 true 면 다시 재지 않았다).
    //  그 결과 인계 4.15초 만에 헤딩이 125° 어긋나 조향이 좌로 포화(-37°)했고
    //  CTE 가 +0.02 → +6.23 m 로 벌어졌다.
    //
    //  고치는 방법은 둘이다 —
    //    ① 잘못 잡은 것을 ★받아들이지 않는다★ (아래 max_abs · max_std)
    //    ② 잘못 잡혔어도 ★서 있을 때마다 다시 잡는다★ (recal_still_s)
    //  둘 다 OS1 자이로만으로 끝나므로 외장 iAHRS 와 섞이지 않는다.
    //  ★신호등·S 지점에서 자주 서므로 ②의 기회는 충분하다★
    declare_parameter<double>("imu_bias_max_abs", 0.10);     // rad/s = 5.7°/s
    declare_parameter<double>("imu_bias_max_std", 0.05);     // rad/s = 2.9°/s
    declare_parameter<double>("imu_bias_recal_still_s", 1.0);

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

  //  ★계획기 파라미터 표 [2026-09-28]★ 읽기(readParameters)와 실행 중 변경
  //  (onSetParams)이 같은 표를 쓴다 — 한쪽에만 있는 키가 생기지 않게.
  struct FrenetDoubleParam
  {
    const char * name;
    double FrenetPlannerParams::* field;
    double lo;
  };
  struct FrenetIntParam
  {
    const char * name;
    int FrenetPlannerParams::* field;
    int lo;
  };
  static const std::vector<FrenetDoubleParam> & frenetDoubleParams()
  {
    static const std::vector<FrenetDoubleParam> t = {
      {"avoid.range_m", &FrenetPlannerParams::range_m, 4.0},
      {"avoid.max_offset_m", &FrenetPlannerParams::max_offset_m, 0.5},
      {"avoid.pass_gap_m", &FrenetPlannerParams::pass_gap_m, 0.1},
      {"frenet.plan_distance_m", &FrenetPlannerParams::plan_distance_m, 6.0},
      {"frenet.ds_m", &FrenetPlannerParams::ds_m, 0.10},
      {"frenet.cluster_max_width_m", &FrenetPlannerParams::cluster_max_width_m, 1.0},
      {"frenet.block_link_s_m", &FrenetPlannerParams::block_link_s_m, 0.0},
      {"frenet.block_link_d_m", &FrenetPlannerParams::block_link_d_m, 0.0},
      {"frenet.lost_s", &FrenetPlannerParams::lost_s, 0.2},
      {"frenet.blind_x_m", &FrenetPlannerParams::blind_x_m, 0.5},
      {"frenet.assoc_gap_m", &FrenetPlannerParams::assoc_gap_m, 0.2},
      {"frenet.hold_pre_m", &FrenetPlannerParams::hold_pre_m, 0.0},
      {"frenet.hold_post_m", &FrenetPlannerParams::hold_post_m, 0.0},
      {"frenet.bias_deadband_m", &FrenetPlannerParams::bias_deadband_m, 0.0},
      {"frenet.bias_deadband_per_m", &FrenetPlannerParams::bias_deadband_per_m, 0.0},
      {"frenet.rel_margin_m", &FrenetPlannerParams::rel_margin_m, 0.0},
      {"frenet.commit_dist_m", &FrenetPlannerParams::commit_dist_m, 0.0},
      {"frenet.kappa_comfort", &FrenetPlannerParams::kappa_comfort, 0.02},
      {"frenet.kappa_max", &FrenetPlannerParams::kappa_max, 0.05},
      {"frenet.shift_len_min_m", &FrenetPlannerParams::shift_len_min_m, 1.0},
      {"frenet.shift_len_max_m", &FrenetPlannerParams::shift_len_max_m, 1.0},
      {"frenet.return_len_min_m", &FrenetPlannerParams::return_len_min_m, 1.0},
      {"frenet.return_len_max_m", &FrenetPlannerParams::return_len_max_m, 1.0},
      {"frenet.flat_min_m", &FrenetPlannerParams::flat_min_m, 0.0},
      {"frenet.entry_len_min_m", &FrenetPlannerParams::entry_len_min_m, 0.5},
      {"frenet.on_path_d_m", &FrenetPlannerParams::on_path_d_m, 0.0},
      {"frenet.on_path_yaw_rad", &FrenetPlannerParams::on_path_yaw_rad, 0.0},
    };
    return t;
  }
  static const std::vector<FrenetIntParam> & frenetIntParams()
  {
    static const std::vector<FrenetIntParam> t = {
      {"frenet.cluster_min_cells", &FrenetPlannerParams::cluster_min_cells, 1},
      {"frenet.confirm_hits", &FrenetPlannerParams::confirm_hits, 1},
      {"frenet.flip_ticks", &FrenetPlannerParams::flip_ticks, 1},
    };
    return t;
  }
  //  군집 연결 거리 [m] → 격자 칸 수 (0.1 m 격자에서 0.30 m = 3칸)
  int linkCells(double m) const
  {
    const double res = std::max(0.02, get_parameter("costmap.resolution").as_double());
    return std::max(1, static_cast<int>(std::lround(m / res)));
  }

  /// 표에 있는 계획기 파라미터면 적용하고 true.
  bool applyFrenetParam(const rclcpp::Parameter & p)
  {
    for (const auto & f : frenetDoubleParams()) {
      if (p.get_name() == f.name) {
        frenet_params_.*(f.field) = std::max(f.lo, p.as_double());
        return true;
      }
    }
    for (const auto & f : frenetIntParams()) {
      if (p.get_name() == f.name) {
        frenet_params_.*(f.field) = std::max(f.lo, static_cast<int>(p.as_int()));
        return true;
      }
    }
    return false;
  }

  void readParameters()
  {
    lidar_topic_ = get_parameter("lidar_topic").as_string();
    imu_topic_ = get_parameter("imu_topic").as_string();
    imu_use_orientation_ = get_parameter("imu_use_orientation").as_bool();
    use_os1_imu_ = get_parameter("use_os1_imu").as_bool();
    imu_yaw_sign_ = get_parameter("imu_yaw_sign").as_double();
    if (imu_yaw_sign_ >= 0.0) {
      imu_yaw_sign_ = 1.0;
    } else {
      imu_yaw_sign_ = -1.0;
    }
    if (use_os1_imu_) {
      // white1 one_launch 가 imu_topic:=/imu · imu_use_orientation:=true 를
      // 넣어도, 구독은 이 멤버로 만든다 → OS1 내장 자이로가 이긴다.
      if (imu_topic_ != "/ouster/imu" || imu_use_orientation_) {
        RCLCPP_WARN(
          get_logger(),
          "use_os1_imu=true — 런치 IMU '%s' orientation=%s 를 무시하고 "
          "/ouster/imu 자이로 적분을 쓴다",
          imu_topic_.c_str(), imu_use_orientation_ ? "true" : "false");
      }
      imu_topic_ = "/ouster/imu";
      imu_use_orientation_ = false;
    }
    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    control_frequency_ = get_parameter("control_frequency").as_double();

    costmap_topic_ = get_parameter("costmap_topic").as_string();
    path_topic_ = get_parameter("path_topic").as_string();
    reference_path_topic_ = get_parameter("reference_path_topic").as_string();
    ref_topic_    = get_parameter("handover.ref_topic").as_string();
    diag_topic_   = get_parameter("diag_topic").as_string();
    cones_topic_  = get_parameter("cones_topic").as_string();
    lat_pass_x_       = get_parameter("avoid.pass_x_m").as_double();
    cone_y_max_       = get_parameter("avoid.cone_y_max_m").as_double();
    geometric_steer_  = get_parameter("avoid.geometric_steer").as_bool();
    frenet_params_.enable = get_parameter("frenet.enable").as_bool();
    for (const auto & f : frenetDoubleParams()) {
      frenet_params_.*(f.field) = std::max(f.lo, get_parameter(f.name).as_double());
    }
    for (const auto & f : frenetIntParams()) {
      frenet_params_.*(f.field) =
        std::max(f.lo, static_cast<int>(get_parameter(f.name).as_int()));
    }
    frenet_params_.cluster_link_cells = linkCells(get_parameter("frenet.cluster_link_m").as_double());
    tracker_params_.lookahead = std::max(1.5, get_parameter("avoid.lookahead_m").as_double());
    tracker_params_.lookahead_min = std::min(tracker_params_.lookahead, 3.2);
    tracker_params_.k_cte = std::max(0.0, get_parameter("avoid.k_cte").as_double());
    tracker_params_.predict_s =
      std::clamp(get_parameter("avoid.steer_predict_s").as_double(), 0.0, 1.0);
    track_ki_deg_  = std::max(0.0, get_parameter("avoid.k_i_deg").as_double());
    track_i_max_deg_ = std::max(0.0, get_parameter("avoid.i_max_deg").as_double());
    track_i_gate_m_  = std::max(0.0, get_parameter("avoid.i_gate_m").as_double());
    stop_predict_m_  = std::max(1.0, get_parameter("stop.predict_m").as_double());
    preview_enable_  = get_parameter("preview.enable").as_bool();
    preview_zone_dist_m_ = std::max(0.0, get_parameter("preview.zone_dist_m").as_double());
    ref_stale_s_  = get_parameter("handover.ref_stale_s").as_double();
    gps_antenna_x_m_ = get_parameter("handover.gps_antenna_x_m").as_double();
    use_gps_ref_  = get_parameter("handover.use_gps_ref").as_bool();
    use_gps_yaw_  = get_parameter("handover.use_gps_yaw").as_bool();
    lstatus_topic_ = get_parameter("handover.lstatus_topic").as_string();
    active_topic_ = get_parameter("handover.active_topic").as_string();
    require_lstatus_ = get_parameter("handover.require_lstatus").as_bool();
    lstatus_stale_s_ =
      std::max(0.0, get_parameter("handover.lstatus_stale_s").as_double());
    base_frame_id_ = get_parameter("base_frame_id").as_string();
    dump_enable_ = get_parameter("debug.dump_enable").as_bool();
    dump_dir_ = get_parameter("debug.dump_dir").as_string();
    if (dump_dir_.empty() || dump_dir_ == "/tmp/mppi_debug") {
#ifdef MPPI_DEBUG_DIR
      dump_dir_ = MPPI_DEBUG_DIR;
#else
      dump_dir_ = "/tmp/mppi_debug";
#endif
    }
    dump_every_n_ = std::max(1, static_cast<int>(get_parameter("debug.dump_every_n").as_int()));
    RCLCPP_INFO(get_logger(), "debug dump dir = %s", dump_dir_.c_str());

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
    mppi_params_.obstacle_on_path_scale =
      std::clamp(get_parameter("mppi.obstacle_on_path_scale").as_double(), 0.0, 1.0);
    mppi_params_.on_path_cte_m =
      std::max(0.05, get_parameter("mppi.on_path_cte_m").as_double());
    mppi_params_.weight_path = get_parameter("mppi.weight_path").as_double();
    mppi_params_.weight_heading = get_parameter("mppi.weight_heading").as_double();
    mppi_params_.weight_speed = get_parameter("mppi.weight_speed").as_double();
    mppi_params_.weight_smooth_v = get_parameter("mppi.weight_smooth_v").as_double();
    mppi_params_.weight_smooth_delta = get_parameter("mppi.weight_smooth_delta").as_double();
    mppi_params_.stanley_lookahead = get_parameter("mppi.stanley_lookahead").as_double();
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
    imu_bias_max_abs_ = get_parameter("imu_bias_max_abs").as_double();
    imu_bias_max_std_ = get_parameter("imu_bias_max_std").as_double();
    imu_bias_recal_still_s_ = get_parameter("imu_bias_recal_still_s").as_double();

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
    //  추종기·정지 예측이 쓰는 차량값 — 제원과 지령층에서 가져온다(따로 적지 않는다).
    tracker_params_.wheelbase = vehicle_params_.wheelbase;
    tracker_params_.max_steer = vehicle_params_.max_steering_angle;
    tracker_params_.slew_rad_s = steer_slew_deg_s_ * M_PI / 180.0;
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
    //  ★[2026-09-28] 미리보기를 이어받는다★ 인계 직전까지 조종권 없이 계획을 돌렸고
    //  기준선이 GPS(y = CTE, yaw = 경로 헤딩오차)면 좌표가 끊기지 않는다 — 블록 기억과
    //  이미 정한 쪽을 그대로 들고 들어간다. 그래야 인계 첫 틱부터 경로가 콘을 비킨다.
    //  기준선이 추측항법이면 종전처럼 지금 자세를 0 으로 두고 새로 시작한다.
    //  ★마지막 '성공한' 미리보기 시각으로 본다★ 인계 틱에는 driving 의 zone_left 가
    //  다음 구간(없으면 NaN)으로 넘어가 그 틱의 미리보기가 빠진다 — 플래그로 보면
    //  바로 그 틱 때문에 이어받기가 무산된다(ROS 폐루프 시험에서 실제로 그랬다).
    bool keep = false;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      const bool ref_fresh = ref_valid_ && (nowSeconds() - ref_t_) <= ref_stale_s_;
      keep = preview_t_ > 0.0 && (nowSeconds() - preview_t_) < 0.5 &&
             gps_ref_live_ && gps_yaw_live_ && ref_fresh;
      if (has_abs_yaw_) {
        heading_ref_yaw_ = last_abs_yaw_;   // 지금 방위가 새 기준선이다
      }
      if (!keep) {
        odom_pose_ = OdomPose{};            // x = y = yaw = 0
        //  ★GPS 기준선이 살아 있으면 y·yaw 는 곧바로 그 값으로 둔다★ 0 으로 두면
        //  IMU 콜백이 덮기 전 첫 계획 틱이 차를 기준선 위·정면으로 보고 콘을
        //  CTE·헤딩오차만큼 어긋난 자리에 찍는다 — 그것이 블록을 부풀렸다.
        if (ref_fresh && use_gps_yaw_) {
          odom_pose_.yaw = ref_herr_;
        }
        if (ref_fresh && use_gps_ref_) {
          odom_pose_.y = ref_cte_ - gps_antenna_x_m_ * std::sin(odom_pose_.yaw);   // imuCallback 과 같은 식
        }
      }
    }
    corridor_clear_latched_ = false;
    if (!keep) {
      last_frenet_path_ = {};
      trail_odom_.clear();
      if (frenet_planner_) {
        frenet_planner_->reset();
      }
    }
    preview_t_ = 0.0;
    track_integ_.reset();
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
    if (keep) {
      RCLCPP_INFO(
        get_logger(),
        "🛞 조종권 인수 — 미리보기 이어받음 (GPS 기준선, 앞 블록 %d개). "
        "이미 정한 쪽으로 곧바로 비킨다", last_frenet_path_.n_gates);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "🛞 조종권 인수 — 기준선 재설정 (yaw0=%.1f deg%s). "
        "이 방위의 직선으로 복귀하며 라바콘을 피한다",
        heading_ref_yaw_ * 180.0 / M_PI,
        has_abs_yaw_ ? "" : " ★절대방위 미수신 — 현재 자세를 0 으로 둔다★");
    }
  }

  void publishStop(bool apply_brake, double road_deg = 0.0)
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
      // 조향을 0 으로 두면, 콘을 피하려 꺾던 핸들이 제동이 서는 동안
      // 다시 콘 쪽으로 돌아간다. 펄스만 끊고 각은 유지한다.
      const double enc_age = nowSeconds() - enc_t_.load(std::memory_order_relaxed);
      const double v_meas = (enc_t_.load(std::memory_order_relaxed) > 0.0 && enc_age <= 1.0)
          ? enc_pulse_.load(std::memory_order_relaxed) * lidar::kasa::MS_PER_PULSE
          : -1.0;
      actuator_->driveRaw(0, road_deg, /*control_enable=*/true, v_meas);
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
        gyro_bias_sumsq_ += msg->angular_velocity.z * msg->angular_velocity.z;
      }
      if (n >= imu_bias_calibration_samples_) {
        //  ★[2026-09-12] 받아들이기 전에 검사한다★ 캘리브 표본에 실제 회전이
        //  섞이면(런치 순간 차가 움직이고 있었다면) 그것이 그대로 바이어스가 되어
        //  주행 내내 헤딩을 갉아먹는다 — 2026-09-12 의 -30°/s 가 그것이다.
        //  정지 중이라면 |평균| 도 표준편차도 작아야 한다. 둘 중 하나라도 크면
        //  ★버리고 처음부터 다시 모은다★ (잠글 때까지 차를 세워 두면 곧 통과한다).
        if (!use_ori) {
          const double mean = gyro_bias_sum_ / static_cast<double>(n);
          const double var = std::max(0.0, gyro_bias_sumsq_ / static_cast<double>(n) - mean * mean);
          const double sd = std::sqrt(var);
          if (std::abs(mean) > imu_bias_max_abs_ || sd > imu_bias_max_std_) {
            ++gyro_bias_reject_n_;
            RCLCPP_WARN(
              get_logger(),
              "자이로 바이어스 캘리브 거부 #%d — 평균 %.4f rad/s (%.1f°/s, 상한 %.1f) "
              "표준편차 %.4f (상한 %.4f). ★차가 움직이는 중이다★ — 세우면 다시 잡는다.",
              gyro_bias_reject_n_, mean, mean * 180.0 / M_PI,
              imu_bias_max_abs_ * 180.0 / M_PI, sd, imu_bias_max_std_);
            gyro_bias_sum_ = 0.0;
            gyro_bias_sumsq_ = 0.0;
            gyro_bias_sample_count_.store(0, std::memory_order_relaxed);
            return;
          }
        }
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

    //  ══════════════════════════════════════════════════════════════════
    //  ★[2026-09-12] 서 있는 동안 바이어스를 다시 잡는다★
    //  ══════════════════════════════════════════════════════════════════
    //  초기 캘리브가 통과했어도 온도·시간에 따라 흐르고, 무엇보다 ★한 번
    //  잘못 잡히면 노드를 내릴 때까지 그대로였다★. 차가 확실히 서 있는 동안
    //  (엔코더 실측 0 + 지령 0 이 imu_bias_recal_still_s 이상 이어질 때)
    //  표본을 모아 같은 품질검사를 통과하면 갱신한다.
    //  ★정지 중에는 자이로가 곧 바이어스다★ — 별도 기준(GPS·iAHRS)이 필요 없어
    //  '라이다는 자체 IMU' 원칙을 깨지 않는다.
    if (!use_ori) {
      const double enc_age_b = nowSeconds() - enc_t_.load(std::memory_order_relaxed);
      const double v_cmd_b = last_commanded_v_.load(std::memory_order_relaxed);
      const bool still = (enc_age_b < 1.0) &&
                         (enc_pulse_.load(std::memory_order_relaxed) < 0.25) &&
                         (std::abs(v_cmd_b) < 0.05);
      const double tnow = nowSeconds();
      if (!still) {
        recal_still_since_ = -1.0;
        recal_sum_ = recal_sumsq_ = 0.0;
        recal_n_ = 0;
      } else {
        if (recal_still_since_ < 0.0) {
          recal_still_since_ = tnow;
          recal_sum_ = recal_sumsq_ = 0.0;
          recal_n_ = 0;
        }
        //  정지가 충분히 이어진 뒤부터 모은다(멈추는 순간의 잔여 요레이트 배제)
        if (tnow - recal_still_since_ >= imu_bias_recal_still_s_) {
          const double wz_raw = msg->angular_velocity.z;
          recal_sum_ += wz_raw;
          recal_sumsq_ += wz_raw * wz_raw;
          ++recal_n_;
          if (recal_n_ >= imu_bias_calibration_samples_) {
            const double mean = recal_sum_ / static_cast<double>(recal_n_);
            const double var = std::max(
              0.0, recal_sumsq_ / static_cast<double>(recal_n_) - mean * mean);
            const double sd = std::sqrt(var);
            if (std::abs(mean) <= imu_bias_max_abs_ && sd <= imu_bias_max_std_) {
              const double old_bias = gyro_bias_wz_;
              gyro_bias_wz_ = mean;
              if (std::abs(mean - old_bias) > 0.01) {   // 0.57°/s 넘게 바뀌면 알린다
                RCLCPP_INFO(
                  get_logger(),
                  "자이로 바이어스 재캘리브 — %.4f → %.4f rad/s "
                  "(%.2f → %.2f °/s, %d 표본, 정지 %.1fs)",
                  old_bias, mean, old_bias * 180.0 / M_PI, mean * 180.0 / M_PI,
                  recal_n_, tnow - recal_still_since_);
              }
            }
            recal_sum_ = recal_sumsq_ = 0.0;
            recal_n_ = 0;
            recal_still_since_ = tnow;   // 다음 창을 새로 연다
          }
        }
      }
    }

    if (use_ori) {
      const double yaw_abs = yawFromQuaternion(msg->orientation);
      last_abs_yaw_ = yaw_abs;              // 재무장이 꺼내 쓴다 (rearmReference)
      has_abs_yaw_ = true;
      odom_pose_.yaw = wrapAngle(yaw_abs - heading_ref_yaw_);
      imu_heading_from_quat_ = true;
    } else {
      double wz = imu_yaw_sign_ * (msg->angular_velocity.z - gyro_bias_wz_);
      const double v_now = last_commanded_v_.load(std::memory_order_relaxed);
      if (v_now < 0.25 && std::abs(wz) < 0.08) {
        wz = 0.0;
      }
      odom_pose_.yaw = wrapAngle(odom_pose_.yaw + wz * dt);
    }
    //  매핑 경로가 휘면 인계 헤딩(OS1 0 점)과 경로 접선이 갈라진다.
    //  heading_err = 차 헤딩 − 경로 방위 이므로, 신선한 동안 yaw 를 그것으로
    //  바꾼 뒤에 s 를 적분한다. x 를 먼저 적분하면 진행이 옛 직선을 따른다.
    const bool ref_fresh = ref_valid_ &&
      (nowSeconds() - ref_t_) <= ref_stale_s_;
    if (ref_fresh && use_gps_yaw_) {
      odom_pose_.yaw = ref_herr_;
    }
    gps_yaw_live_ = ref_fresh && use_gps_yaw_;
    //  ★x 는 경로를 따라 나간 거리★ 속도는 실측(엔코더). 지령으로 적분하면
    //  1.65배 부풀려진다. 엔코더가 없으면 지령으로 떨어진다.
    const double enc_age = nowSeconds() - enc_t_.load(std::memory_order_relaxed);
    const double v_meas = enc_pulse_.load(std::memory_order_relaxed)
                          * lidar::kasa::MS_PER_PULSE;
    const double v = (enc_t_.load(std::memory_order_relaxed) > 0.0 && enc_age <= 1.0)
                       ? v_meas
                       : last_commanded_v_.load(std::memory_order_relaxed);
    odom_pose_.x += v * std::cos(odom_pose_.yaw) * dt;

    if (use_gps_ref_ && ref_fresh) {
      //  ══════════════════════════════════════════════════════════════════
      //  ★[2026-09-29] CTE 는 ★안테나★ 자리다 — 라이다 원점으로 되돌린다★
      //  ══════════════════════════════════════════════════════════════════
      //  driving 은 GPS 를 뒤차축으로 투영하지 않는다(그 파일 '이 차는 GPS 가 앞차축
      //  위에 있다' 주석). 그 값을 라이다 원점의 y 로 쓰면 헤딩 ψ 만큼 틀어질 때마다
      //  콘이 L·sin ψ 옆으로 옮겨 보인다. 9/28 23:03·23:05 실주행 .cones.csv 회귀:
      //  콘 13개 1080관측 d_obs = d + L·sin ψ → ★L = 0.54~0.72 m★ (잔차 0.172 → 0.138)
      //  첫 줄 안쪽 콘이 헤딩 −8° → +19° 사이에 +0.08 → +0.35 m 로 '움직였고',
      //  그것이 인계 뒤 목표를 +1.20 → +1.54 로 늦게 키워 차가 0.4 m 뒤처진 원인이다.
      //  0.6 m 는 틀려도 손해가 없는 값이다(모의: 실제 0 이면 같고, 0.6·1.25 면 낫다).
      odom_pose_.y = ref_cte_ - gps_antenna_x_m_ * std::sin(odom_pose_.yaw);
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
      runPreview();              // ★[2026-09-28] 조종권 없이 계획만 — 아무것도 발행하지 않는다★
      publishActive(false);      // ★값은 false 지만 신선도로 생존을 알린다★
      return;
    }
    if (!was_permitted_) {
      rearmReference();          // ★L 구간마다 기준선을 다시 잡는다★ (그쪽 주석)
      was_permitted_ = true;
    }
    last_permit_t_ = nowSeconds();
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
    //  ★지금 조향이 만드는 곡률을 넘긴다★ 경로가 차의 실제 곡률에서 이어지면
    //  첫 틱에 조향이 튀지 않는다(5차 다항식의 시작 곡률 조건).
    const double kappa0 = std::tan(last_pub_steer_deg_ * M_PI / 180.0) /
      std::max(0.5, vehicle_params_.wheelbase);
    last_frenet_path_ = frenet_planner_->plan(current_pose, snap, nowSeconds(), kappa0);
    logPlannerEvents(false);
    n_cones_ahead_ = last_frenet_path_.n_gates;
    trail_odom_.push_back({current_pose.x, current_pose.y});
    if (trail_odom_.size() > 600) {
      trail_odom_.pop_front();
    }
    if (dump_enable_ && ++dump_tick_ >= dump_every_n_) {
      dump_tick_ = 0;
      dumpDebugImage(current_pose, snap);
    }

    // How many model steps elapsed since the last control cycle.
    // When frequency == 1/dt this is normally 1; if the loop lagged it can be >1.
    int shift_steps = 1;
    if (mppi_params_.dt > 1e-6 && elapsed > 0.0 && elapsed < 1.0) {
      shift_steps = std::max(1, static_cast<int>(std::lround(elapsed / mppi_params_.dt)));
      shift_steps = std::min(shift_steps, mppi_params_.horizon_steps);
    }

    const MPPIResult result =
      controller_->computeControl(current_pose, snap, last_frenet_path_, shift_steps);
    const double avg_cost = result.min_cost /
      static_cast<double>(std::max(1, mppi_params_.horizon_steps));

    // 직진은 순항 펄스(2). 회피 중(|조향| 큼)에도 ★2펄스★ — 1펄스는 이 차가
    // 거의 움직이지 않는다(cmd.dodge_pulse 선언부의 근거).
    //
    //  계획 = Frenet d(s). geometric_steer 이면 그 곡선을 퓨어 퍼슛으로 쫓고
    //  MPPI 조향은 버린다. MPPI 평균비용으로 세우면, 차가 안 가는 샘플이
    //  비싸다는 이유만으로 회피 중에 선다.
    double raw_steer_deg = result.control.delta * 180.0 / M_PI;
    if (geometric_steer_) {
      //  ★[2026-09-28] 추종·정지는 path_tracker.hpp — 모의실험과 같은 코드다★
      const double v_trk = trackSpeed();
      raw_steer_deg = purePursuitSteer(
        current_pose, last_frenet_path_, tracker_params_, v_trk,
        last_pub_steer_deg_ * M_PI / 180.0) * 180.0 / M_PI;
      const double e_path = current_pose.y - last_frenet_path_.sample(current_pose.x).d;
      //  ★곧은 경로를 곧게 따라갈 때만 쌓는다★ (steadyTracking 주석 — 회피 중 누적 +2.1°)
      raw_steer_deg += track_integ_.update(
        e_path, 1.0 / std::max(1.0, control_frequency_), track_ki_deg_,
        track_i_max_deg_, track_i_gate_m_,
        v_trk > 0.3 && !stop_latched_ && steadyTracking(last_frenet_path_, current_pose.x));
      const double max_deg = vehicle_params_.max_steering_angle * 180.0 / M_PI;
      raw_steer_deg = std::clamp(raw_steer_deg, -max_deg, max_deg);
    }
    const double steer_deg = filterSteer(raw_steer_deg);
    double hit_at_m = 0.0;
    //  ★정지 = 경로를 실제로 따라갔을 때 부딪히는가★ (종전: 지금 조향 2.2 m 고정 원호)
    const bool imminent = geometric_steer_ &&
      predictTrackHits(current_pose, last_frenet_path_, snap, tracker_params_,
                       std::max(0.8, mppi_params_.desired_speed),
                       steer_deg * M_PI / 180.0, stop_predict_m_, &hit_at_m);
    const bool raw_stop = geometric_steer_ ? imminent : result.stopped_for_collision;
    const bool latched_stop = updateStopLatch(raw_stop);

    if (latched_stop) {
      const double held = stop_latch_t_.nanoseconds() == 0 ? 0.0 :
        (t_now - stop_latch_t_).seconds();
      const double brake_wait = geometric_steer_ ? 0.35 : brake_after_s_;
      const bool use_brake = brake_enable_ && held >= brake_wait;
      const PlanDiag & pd = frenet_planner_->lastDiag();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "stop latch: %s  경로 추종 예측 %.1f m 앞 충돌  held=%.1fs  brake=%d  steer=%.1f  "
        "| 경로 여유 %.2f m, 최대곡률 %.2f (블록 %d)",
        geometric_steer_ ? "track" : "mppi",
        raw_stop ? hit_at_m : 0.0, held, use_brake ? 1 : 0, steer_deg,
        pd.min_clear_m, pd.max_kappa, pd.n_blocks);
      publishStop(use_brake, steer_deg);
      publishRolloutPath();
      publishReferencePath(current_pose);
      return;
    }
    const int cruise = std::max(1, actuator_->maxPulse());
    const int ref = (std::abs(steer_deg) > dodge_steer_deg_)
                      ? std::min(dodge_pulse_, cruise) : cruise;
    const int out = lowSpeedTrim(ref);
    publishDrive(out, steer_deg);

    {
      const PlanDiag & pd = frenet_planner_->lastDiag();
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "drive ref=%d → out=%d pulse (%.2f m/s, enc %.1f)  steer=%.1f deg (적분 %+.1f)  "
        "| 기준선 %s y=%.2f m yaw=%.1f deg | 블록 %d 목표 %+.2f 여유 %.2f κ %.2f%s",
        ref, actuator_->lastPulse(), lidar::kasa::pulseToMs(actuator_->lastPulse()),
        enc_pulse_.load(std::memory_order_relaxed), steer_deg, track_integ_.value_deg,
        (gps_ref_live_ && gps_yaw_live_) ? "GPS-y/경로헤딩" :
        gps_ref_live_ ? "GPS-y/OS1-yaw" : "추측항법",
        current_pose.y, current_pose.yaw * 180.0 / M_PI,
        pd.n_blocks, pd.target_d, pd.min_clear_m, pd.max_kappa, pd.tight ? " ★빡빡★" : "");
    }

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

  /// 추종·정지 예측에 쓸 속도 [m/s] — 실측(엔코더)이 신선하면 그것, 아니면 순항.
  double trackSpeed() const
  {
    const double t = enc_t_.load(std::memory_order_relaxed);
    if (t > 0.0 && (nowSeconds() - t) <= 1.0) {
      return std::max(0.3, enc_pulse_.load(std::memory_order_relaxed) * lidar::kasa::MS_PER_PULSE);
    }
    return std::max(0.3, mppi_params_.desired_speed);
  }

  /// 계획기가 남긴 드문 사건(블록 발견·쪽 결정·통과)을 한 번씩 로그로 낸다.
  void logPlannerEvents(bool preview)
  {
    for (const auto & e : frenet_planner_->takeEvents()) {
      RCLCPP_INFO(get_logger(), "%s%s", preview ? "[미리보기] " : "", e.c_str());
    }
  }

  // ══════════════════════════════════════════════════════════════════════
  //  ★[2026-09-28] 인계 전 미리보기 — 조종권 없이 계획만 돌린다★
  // ══════════════════════════════════════════════════════════════════════
  //  실측 : 라이다는 인계 순간 이미 16 m 앞의 둘째 줄까지 보고 있었는데(15.7 m),
  //  이 노드는 허락 전에는 아무것도 하지 않아 첫 줄을 7.5 m 앞에서 처음 '봤다'.
  //  → driving 이 살아 있고(/lstatus 신선), 기준선이 GPS 이고(y·yaw 둘 다),
  //    다음 L 구간이 preview.zone_dist_m 안이면 계획기를 돌려 블록을 기억하고
  //    쪽을 미리 정해 둔다. ★발행은 하나도 하지 않는다★ — 조종권 계약 그대로다.
  //  인계 순간 rearmReference 가 이 기억을 이어받는다.
  void runPreview()
  {
    if (!preview_enable_ || !frenet_planner_ || !costmap_->hasData() ||
        !gyro_bias_calibrated_.load(std::memory_order_acquire))
    {
      return;
    }
    const double stamp = lstatus_stamp_.load(std::memory_order_relaxed);
    if (stamp <= 0.0 || (lstatus_stale_s_ > 0.0 && (nowSeconds() - stamp) > lstatus_stale_s_)) {
      return;                             // driving 이 없다
    }
    OdomPose pose;
    double zone_left = std::numeric_limits<double>::quiet_NaN();
    bool ref_ok = false;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      pose = odom_pose_;
      zone_left = ref_zone_left_;
      ref_ok = gps_ref_live_ && gps_yaw_live_;
    }
    //  ★L 구간 도중에 조종권이 잠깐 회수돼도(E-STOP 일시정지) 계속 본다★ 기억을
    //  버리면 재개 순간 코앞 사각지대(자기 차체 지움)의 콘을 잊는다.
    const bool near_zone = std::isfinite(zone_left) && zone_left <= preview_zone_dist_m_;
    const bool recent = last_permit_t_ > 0.0 && (nowSeconds() - last_permit_t_) < 20.0;
    if (!ref_ok || !(near_zone || recent)) {
      return;
    }
    const CostmapSnapshot snap = costmap_->snapshot();
    if (!snap.valid) {
      return;
    }
    last_frenet_path_ = frenet_planner_->plan(pose, snap, nowSeconds(), 0.0);
    if (near_zone) {
      logPlannerEvents(true);
    } else {
      frenet_planner_->takeEvents();   // 기억만 잇는 중 — 구간 밖 물체의 쪽 결정은 로그로 안 낸다
    }
    preview_t_ = nowSeconds();
    if (near_zone) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "👀 미리보기 — L 구간 %.1f m 앞, 블록 %d, 목표 %+.2f m",
        zone_left, frenet_planner_->lastDiag().n_blocks, frenet_planner_->lastDiag().target_d);
    }
    if (dump_enable_ && ++dump_tick_ >= dump_every_n_) {
      dump_tick_ = 0;
      dumpDebugImage(pose, snap);
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
  ///   [9] target_y ★지금 Frenet d(s_now)★ [m] + 왼쪽
  ///  [10] n_cones  ★앞에 남은 장애물 블록 수★ [2026-09-28] — 0 이면 복귀 구간이다
  ///  [11..13]      치사 셀 수 / 점유 군집 수 / 버린 군집 수 (cone_detector.hpp ConeStats)
  void publishDiag(const OdomPose & pose, double road_deg, double avg_cost,
                   const CostmapSnapshot & snap)
  {
    //  ★[2026-09-28] 계획기가 이번 틱에 본 점유 군집을 그대로 쓴다★ 진단이 따로
    //  검출하면 계획과 다른 것을 기록하게 된다(종전 detectCones 는 치사층 덩어리).
    (void)snap;
    double ox = std::numeric_limits<double>::quiet_NaN();
    double oy = std::numeric_limits<double>::quiet_NaN();
    {
      cone_stats_ = frenet_planner_->lastStats();
      //  ★회피 대상만 골라 목록으로 낸다★ [x, y, cells] 3개씩 — .cones.csv 규약 그대로
      std_msgs::msg::Float64MultiArray cm;
      const auto & cs = frenet_planner_->lastClustersEgo();
      cm.data.reserve(cs.size() * 3);
      for (const auto & c : cs) {
        const ConeObs obs{c.cx, c.cy, c.cells};
        if (!coneInAvoidLane(pose, obs, lat_pass_x_, frenet_params_.range_m, cone_y_max_)) continue;
        if (std::isnan(ox)) { ox = c.cx; oy = c.cy; }   // x0 오름차순 — 가장 가까운 앞쪽
        cm.data.push_back(c.cx);
        cm.data.push_back(c.cy);
        cm.data.push_back(static_cast<double>(c.cells));
      }
      cones_pub_->publish(cm);      // ★콘이 0개여도 빈 배열을 낸다★ — 그래야
                                    //   '안 보였다' 와 '노드가 죽었다' 가 갈린다
    }
    std_msgs::msg::Float64MultiArray m;
    m.data = {
      pose.y, pose.yaw * 180.0 / M_PI, road_deg,
      actuator_->lastPotDeg(), static_cast<double>(actuator_->lastPulse()),
      avg_cost, gps_ref_live_ ? 1.0 : 0.0, ox, oy,
      last_frenet_path_.target_d, static_cast<double>(n_cones_ahead_),
      //  ★[2026-09-12] 검출 진단 3개 추가 (11 → 14)★
      //  ⚠️ white1/record.py 의 _array(14) 와 ★짝★ 이다 — 한쪽만 고치지 말 것.
      static_cast<double>(cone_stats_.lethal_cells),
      static_cast<double>(cone_stats_.clusters_all),
      static_cast<double>(cone_stats_.clusters_rejected)};
    diag_pub_->publish(m);
  }

  void publishReferencePath(const OdomPose & odom_pose)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = base_frame_id_;

    if (last_frenet_path_.valid && last_frenet_path_.pts.size() >= 2) {
      path.poses.reserve(last_frenet_path_.pts.size());
      for (const auto & fp : last_frenet_path_.pts) {
        double ex = 0.0;
        double ey = 0.0;
        odomToEgo(odom_pose, fp.s, fp.d, ex, ey);
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path.header;
        ps.pose.position.x = ex;
        ps.pose.position.y = ey;
        ps.pose.orientation = yawToQuaternion(wrapAngle(fp.yaw - odom_pose.yaw));
        path.poses.push_back(ps);
      }
    } else {
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
    }
    reference_path_pub_->publish(path);
  }

  void plotPx(std::vector<uint8_t> & rgb, int w, int h, int px, int py,
              uint8_t r, uint8_t g, uint8_t b)
  {
    if (px < 0 || py < 0 || px >= w || py >= h) {
      return;
    }
    const size_t o = (static_cast<size_t>(py) * static_cast<size_t>(w) +
                      static_cast<size_t>(px)) * 3;
    rgb[o] = r;
    rgb[o + 1] = g;
    rgb[o + 2] = b;
  }

  void dumpDebugImage(const OdomPose & odom, const CostmapSnapshot & snap)
  {
    if (!snap.valid || snap.cells_x < 4 || snap.cells_y < 4) {
      return;
    }
    namespace fs = std::filesystem;
    std::error_code ec;
    fs::create_directories(dump_dir_, ec);
    if (ec) {
      return;
    }
    // Image: +x forward = up, +y left = left.
    const int w = snap.cells_y;
    const int h = snap.cells_x;
    std::vector<uint8_t> rgb(static_cast<size_t>(w) * static_cast<size_t>(h) * 3, 8);
    const double res = snap.resolution;
    auto toPx = [&](double ex, double ey, int & px, int & py) {
      px = static_cast<int>(std::floor((snap.size_y / 2.0 - ey) / res));
      py = static_cast<int>(std::floor((snap.size_x / 2.0 - ex) / res));
    };
    for (int iy = 0; iy < snap.cells_y; ++iy) {
      for (int ix = 0; ix < snap.cells_x; ++ix) {
        const float v = snap.cost[static_cast<size_t>(iy) * static_cast<size_t>(snap.cells_x) +
                                  static_cast<size_t>(ix)];
        const double ex = -snap.size_x / 2.0 + (ix + 0.5) * res;
        const double ey = -snap.size_y / 2.0 + (iy + 0.5) * res;
        int px = 0;
        int py = 0;
        toPx(ex, ey, px, py);
        uint8_t r = 12, g = 14, b = 18;
        if (v >= EgoCostmap::kLethalCost * 0.99) {
          r = 255; g = 40; b = 50;
        } else if (v > 40.0f) {
          r = 255; g = 140; b = 40;
        }
        plotPx(rgb, w, h, px, py, r, g, b);
      }
    }
    auto stroke = [&](double ex, double ey, uint8_t r, uint8_t g, uint8_t b) {
      int px = 0, py = 0;
      toPx(ex, ey, px, py);
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          plotPx(rgb, w, h, px + dx, py + dy, r, g, b);
        }
      }
    };
    for (const auto & fp : last_frenet_path_.pts) {
      double ex = 0.0, ey = 0.0;
      odomToEgo(odom, fp.s, fp.d, ex, ey);
      stroke(ex, ey, 255, 220, 40);
    }
    //  ★[2026-09-28] 블록(자홍 상자)과 그 옆의 통과 목표(초록 선)★
    auto line = [&](double s0, double d0, double s1, double d1, uint8_t r, uint8_t g, uint8_t b) {
      const int n = std::max(2, static_cast<int>(std::hypot(s1 - s0, d1 - d0) / (0.5 * res)));
      for (int i = 0; i <= n; ++i) {
        double ex = 0.0, ey = 0.0;
        odomToEgo(odom, s0 + (s1 - s0) * i / n, d0 + (d1 - d0) * i / n, ex, ey);
        int px = 0, py = 0;
        toPx(ex, ey, px, py);
        plotPx(rgb, w, h, px, py, r, g, b);
      }
    };
    for (const auto & bk : frenet_planner_->blocks()) {
      const uint8_t c = bk.confirmed ? 235 : 110;
      line(bk.s0, bk.d0, bk.s1, bk.d0, c, 60, c);
      line(bk.s1, bk.d0, bk.s1, bk.d1, c, 60, c);
      line(bk.s1, bk.d1, bk.s0, bk.d1, c, 60, c);
      line(bk.s0, bk.d1, bk.s0, bk.d0, c, 60, c);
      if (bk.confirmed && bk.side != 0) {
        line(bk.s0 - 0.3, bk.target, bk.s1 + 0.3, bk.target, 60, 255, 60);
      }
    }
    for (const auto & tr : trail_odom_) {
      double ex = 0.0, ey = 0.0;
      odomToEgo(odom, tr.first, tr.second, ex, ey);
      stroke(ex, ey, 60, 220, 120);
    }
    const auto roll = controller_->getLastRolloutTrajectory();
    for (const auto & st : roll) {
      stroke(st.x, st.y, 90, 220, 255);
    }
    stroke(0.0, 0.0, 212, 160, 23);

    const auto stamp = now().nanoseconds();
    const std::string path =
      dump_dir_ + "/mppi_" + std::to_string(stamp) + ".ppm";
    std::ofstream out(path, std::ios::binary);
    if (!out) {
      return;
    }
    out << "P6\n" << w << " " << h << "\n255\n";
    out.write(reinterpret_cast<const char *>(rgb.data()),
              static_cast<std::streamsize>(rgb.size()));
  }

  rcl_interfaces::msg::SetParametersResult onSetParams(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & p : params) {
      const std::string & n = p.get_name();
      try {
        if (n == "roi_agl_min") {
          roi_agl_min_ = p.as_double();
        } else if (n == "roi_agl_max") {
          roi_agl_max_ = p.as_double();
        } else if (n == "frenet.enable") {
          frenet_params_.enable = p.as_bool();
        } else if (n == "frenet.cluster_link_m") {
          frenet_params_.cluster_link_cells = linkCells(p.as_double());
        } else if (n == "avoid.lookahead_m") {
          tracker_params_.lookahead = std::max(1.5, p.as_double());
          tracker_params_.lookahead_min = std::min(tracker_params_.lookahead, 3.2);
        } else if (n == "avoid.k_cte") {
          tracker_params_.k_cte = std::max(0.0, p.as_double());
        } else if (n == "avoid.steer_predict_s") {
          tracker_params_.predict_s = std::clamp(p.as_double(), 0.0, 1.0);
        } else if (n == "avoid.k_i_deg") {
          track_ki_deg_ = std::max(0.0, p.as_double());
        } else if (n == "avoid.i_max_deg") {
          track_i_max_deg_ = std::max(0.0, p.as_double());
        } else if (n == "stop.predict_m") {
          stop_predict_m_ = std::max(1.0, p.as_double());
        } else if (n == "preview.enable") {
          preview_enable_ = p.as_bool();
        } else if (n == "preview.zone_dist_m") {
          preview_zone_dist_m_ = std::max(0.0, p.as_double());
        } else if (applyFrenetParam(p)) {
          //  표에 있는 계획기 값 (frenetDoubleParams / frenetIntParams)
        } else if (n == "costmap.inflation_radius") {
          costmap_params_.inflation_radius = p.as_double();
        } else if (n == "mppi.weight_path") {
          mppi_params_.weight_path = p.as_double();
        } else if (n == "mppi.weight_obstacle") {
          mppi_params_.weight_obstacle = p.as_double();
        } else if (n == "debug.dump_enable") {
          dump_enable_ = p.as_bool();
        }
      } catch (const std::exception & e) {
        result.successful = false;
        result.reason = e.what();
        return result;
      }
    }
    costmap_params_.ground_z_min = roi_agl_min_ - sensor_height_m_;
    costmap_params_.ground_z_max = roi_agl_max_ - sensor_height_m_;
    if (costmap_) {
      costmap_->setFilterParams(
        costmap_params_.ground_z_min, costmap_params_.ground_z_max,
        costmap_params_.inflation_radius);
    }
    if (frenet_planner_) {
      frenet_planner_->setParams(frenet_params_);
    }
    if (controller_) {
      controller_->setParams(mppi_params_);
    }
    return result;
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
  std::unique_ptr<FrenetPlanner> frenet_planner_;
  FrenetPlannerParams frenet_params_;
  FrenetPath last_frenet_path_;
  std::deque<std::pair<double, double>> trail_odom_;
  bool dump_enable_ = true;
  std::string dump_dir_ = "/tmp/mppi_debug";
  int dump_every_n_ = 10;
  int dump_tick_ = 0;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  std::unique_ptr<lidar::kasa::KasaActuator> actuator_;
  bool flip_lidar_xy_ = true;
  bool brake_enable_ = true;
  double sensor_height_m_ = 1.17;
  double roi_agl_min_ = 0.30;
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
  double gps_antenna_x_m_ = 0.6;   // 안테나가 라이다 원점보다 앞 [m] (CTE → 라이다 원점 y)
  bool   use_gps_ref_ = true;
  bool   use_gps_yaw_ = true;   // 신선한 heading_err 로 yaw 를 덮음 (경로 방위 기준)
  bool   use_os1_imu_ = true;
  double imu_yaw_sign_ = 1.0;
  bool   ref_valid_ = false;
  bool   gps_ref_live_ = false;
  bool   gps_yaw_live_ = false;
  double ref_t_ = 0.0, ref_cte_ = 0.0, ref_herr_ = 0.0;
  double ref_zone_left_ = std::numeric_limits<double>::quiet_NaN();
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ref_sub_;
  std::string diag_topic_ = "/lidar_diag";
  std::string cones_topic_ = "/lidar_cones";
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cones_pub_;
  ConeStats cone_stats_;         // 마지막 검출 통계 (diag 12~14열)
  //  ★[2026-09-28] 블록 계획기 · 경로 추종★ (값의 뜻은 declareParameters)
  double lat_pass_x_ = 0.90;      // /lidar_cones·진단: 이보다 가까운 콘은 뺀다
  double cone_y_max_ = 4.0;       // /lidar_cones·진단: 기준선 |d| 이보다 옆은 뺀다
  bool   geometric_steer_ = true; // Frenet 경로 순수추종. 정지는 그 경로를 따라간 예측
  TrackerParams tracker_params_;
  TrackIntegrator track_integ_;   // 조향 편향 적분 (재무장마다 0)
  double track_ki_deg_ = 2.0;
  double track_i_max_deg_ = 3.0;
  double track_i_gate_m_ = 0.5;
  double stop_predict_m_ = 3.0;
  bool   preview_enable_ = true;
  double preview_zone_dist_m_ = 25.0;
  double preview_t_ = 0.0;        // 마지막으로 ★성공한★ 미리보기 시각 [steady s] (0 = 없음)
  double last_permit_t_ = 0.0;    // 마지막으로 조종권이 있던 시각 [steady s]
  int    n_cones_ahead_ = 0;     // 진단용 — 앞에 콘이 몇 개 보이나
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr diag_pub_;
  double steer_filt_deg_ = 0.0;
  double last_pub_steer_deg_ = 0.0;
  std::mutex odom_mutex_;
  OdomPose odom_pose_;
  bool imu_use_orientation_ = false;
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
  double gyro_bias_sumsq_ = 0.0;            // ★품질 검증용 (분산)
  std::atomic<int> gyro_bias_sample_count_{0};
  double gyro_bias_wz_ = 0.0;
  std::atomic<bool> gyro_bias_calibrated_{false};
  //  ★[2026-09-12] 바이어스 자체 복구★ — 아래 6개. imuCallback 에서만 만진다.
  double imu_bias_max_abs_ = 0.10;          // [rad/s] 정지 중 |평균| 이 넘으면 캘리브 거부
  double imu_bias_max_std_ = 0.05;          // [rad/s] 표본 표준편차가 넘으면 거부
  double imu_bias_recal_still_s_ = 1.0;     // 이만큼 서 있으면 재캘리브 표본을 쓴다
  int    gyro_bias_reject_n_ = 0;           // 거부 횟수 (로그용)
  double recal_sum_ = 0.0, recal_sumsq_ = 0.0;
  int    recal_n_ = 0;
  double recal_still_since_ = -1.0;         // 정지가 이어지기 시작한 시각 (<0 = 움직이는 중)
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
