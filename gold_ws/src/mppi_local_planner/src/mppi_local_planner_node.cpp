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

    costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(costmap_topic_, 1);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 1);
    reference_path_pub_ = create_publisher<nav_msgs::msg::Path>(reference_path_topic_, 1);

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
    declare_parameter<std::string>("base_frame_id", "os_sensor");

    // ★금색차 실측 (lidar/kasa_units.hpp · drive_lidar.yaml)★
    declare_parameter<double>("wheelbase", lidar::kasa::WHEELBASE_M);
    declare_parameter<double>("track_width", 1.10);  // white/kasa_units.py 윤거 실측
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
    base_frame_id_ = get_parameter("base_frame_id").as_string();

    vehicle_params_.wheelbase = get_parameter("wheelbase").as_double();
    vehicle_params_.track_width = get_parameter("track_width").as_double();
    vehicle_params_.max_speed = get_parameter("max_speed").as_double();
    vehicle_params_.min_speed = get_parameter("min_speed").as_double();
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

  void publishDrive(double v_ms, double road_deg)
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
    const double v_out = braking ? 0.0 : v_ms;
    actuator_->drive(v_out, braking ? 0.0 : road_deg, /*control_enable=*/true);
    last_commanded_v_.store(
      lidar::kasa::pulseToMs(actuator_->lastPulse()), std::memory_order_relaxed);
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
    const double v = last_commanded_v_.load(std::memory_order_relaxed);
    odom_pose_.x += v * std::cos(odom_pose_.yaw) * dt;
    odom_pose_.y += v * std::sin(odom_pose_.yaw) * dt;
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
        if (path_returned && hold_ok && canResetNow(t_now)) {
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
        if (reference_reset_soft_enable_ &&
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

    // 직진은 순항 펄스(2). 회피 중(|조향| 큼)에는 원본 3params 와 같이 ~3 km/h.
    const double steer_deg = filterSteer(result.control.delta * 180.0 / M_PI);
    const int cruise = std::max(1, actuator_->maxPulse());
    const int pulse = (std::abs(steer_deg) > dodge_steer_deg_) ? 1 : cruise;
    const double v_cmd = lidar::kasa::pulseToMs(pulse);
    publishDrive(v_cmd, steer_deg);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "drive v=%.2f m/s (%d pulse)  mppi_steer=%.1f → out=%.1f deg  avg_cost=%.0f",
      v_cmd, actuator_->lastPulse(),
      result.control.delta * 180.0 / M_PI, steer_deg, avg_cost);

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

  // ROS interfaces
  rclcpp::CallbackGroup::SharedPtr cloud_cb_group_;
  rclcpp::CallbackGroup::SharedPtr imu_cb_group_;
  rclcpp::CallbackGroup::SharedPtr control_cb_group_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
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
