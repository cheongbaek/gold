#!/usr/bin/env python3
"""
sensor_monitor.py - 센서 상태 통합 모니터링 노드 [300틱 속도표시 정합]
ROS2 터미널에서 GPS / IMU / 엔코더 / 카메라 / 주행 상태를 실시간으로 확인합니다.
실행: ros2 run white sensor_monitor
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64MultiArray, Int32, String
from sensor_msgs.msg import NavSatFix, Imu
import math
import time

# [kasa 이식] 엔코더 환산 상수의 단일 소유자
from white2 import car_units as ku

# ── [카메라] /lane_metrics 신선도 임계 [s] ──────────────────────────────
# gps_imu.CAM_HEAD_MAX_AGE = 0.40 / driving.CAM_LAT_MAX_AGE = 0.40 과 반드시
# 같은 값이어야 한다. 이보다 오래된 /lane_metrics 는 두 노드 모두 조용히 무시하므로,
# 모니터가 age_str() 의 느슨한 기준(2초)을 쓰면 "모니터는 ✅인데 융합은 이미
# 카메라를 버린" 구간이 보이지 않는다.
CAM_FRESH_MAX_AGE = 0.40

# camera_judgment.py 의 flags 비트 정의와 동일 (F_NO_LANE/F_WIDTH_BAD/F_JUMP/
# F_CONF_LOW/F_SINGLE). 값이 바뀌면 camera_judgment.py 쪽과 같이 고칠 것.
CAM_FLAG_LABELS = (
    (1,  "차선없음"),
    (2,  "폭이상"),
    (4,  "CTE점프"),
    (8,  "conf미달"),
    (16, "단독차선"),
)
# 치명 게이트 — camera_judgment 가 conf_eff=0 으로 죽이는 조합
#   (camera_judgment.py: fatal = flags & (F_WIDTH_BAD | F_JUMP | F_CONF_LOW))
CAM_FATAL_MASK = 2 | 4 | 8


class SensorMonitorNode(Node):
    def __init__(self):
        super().__init__("sensor_monitor_node")

        # ── 상태 변수 ──────────────────────────────────
        self.gps_status_raw   = -1       # NavSatFix status.status
        self.gps_last_time    = None
        self.gps_lat          = 0.0
        self.gps_lon          = 0.0
        self.gps_recv_count   = 0        # 수신 카운터 (Hz 계산용)
        self.gps_hz_buf       = []       # 최근 수신 시각 버퍼

        self.imu_roll         = 0.0
        self.imu_pitch        = 0.0
        self.imu_yaw          = 0.0
        self.imu_last_time    = None
        self.imu_hz_buf       = []

        self.encoder_val      = 0
        self.encoder_speed_ms = 0.0
        self.encoder_last_time= None
        self.encoder_hz_buf   = []

        self.ego_heading      = 0.0
        self.ego_speed        = 0.0
        self.ego_fused_lat    = 0.0
        self.ego_fused_lon    = 0.0
        self.ego_last_time    = None

        self.gps_status_str   = "수신 대기 중..."

        # ── [카메라] /lane_metrics[10] (camera_judgment 발행) ──────────
        # 레이아웃: [0]cte_rear_m [1]cte_near_m [2]theta_lane_deg [3]curvature
        #           [4]conf_eff [5]lane_width_m [6]flags [7]d_near_m [8]conf_raw [9]seq
        self.cam_cte_rear     = 0.0      # [0] 우측+ (GPS CTE 규약과 동일)
        self.cam_theta_deg    = 0.0      # [2] 차선 대비 각도
        self.cam_conf_eff     = 0.0      # [4] 게이트 반영 — 융합이 실제로 쓰는 값
        self.cam_width_m      = 0.0      # [5] 정상 ≈3.2m
        self.cam_flags        = 0        # [6] 비트 (CAM_FLAG_LABELS)
        self.cam_conf_raw     = 0.0      # [8] 게이트 전 원본
        self.cam_last_time    = None
        self.cam_hz_buf       = []

        self.judgment_state   = None     # 게이트 FSM (PASS/RED_STOP/CW_SLOW…)
        self.judgment_time    = None
        self.tl_state         = None     # 신호등 RED/RED_FAR/GREEN/UNKNOWN
        self.tl_time          = None

        # ── 구독 ───────────────────────────────────────
        self.create_subscription(NavSatFix,         "/fix",        self.cb_fix,     10)
        self.create_subscription(Imu,               "/imu/data",   self.cb_imu,     10)
        self.create_subscription(Int32,             "/encoder",    self.cb_encoder, 10)
        self.create_subscription(Float64MultiArray, "/ego_state",  self.cb_ego,     10)
        self.create_subscription(String,            "/gps_status", self.cb_gps_status, 10)
        # [카메라] use_camera:=false 면 이 토픽들은 아무도 발행하지 않는다 →
        #   print_status 가 퍼블리셔 수를 보고 "카메라 미사용"으로 접는다.
        self.create_subscription(Float32MultiArray, "/lane_metrics",
                                 self.cb_lane_metrics, 10)
        self.create_subscription(String, "/judgment_state", self.cb_judgment, 10)
        self.create_subscription(String, "/tl/state",       self.cb_tl,       10)

        # 1초마다 터미널에 상태 출력
        self.create_timer(1.0, self.print_status)

        self.get_logger().info("🖥️  센서 모니터 시작 (1초 주기 상태 출력)")

    # ── 콜백 ───────────────────────────────────────────
    def cb_fix(self, msg):
        now = time.time()
        self.gps_status_raw = msg.status.status
        self.gps_lat        = msg.latitude
        self.gps_lon        = msg.longitude
        self.gps_last_time  = now
        self.gps_hz_buf.append(now)
        self.gps_hz_buf     = [t for t in self.gps_hz_buf if now - t <= 5.0]

    def cb_imu(self, msg):
        now = time.time()
        # 쿼터니언 → 오일러 변환
        x, y, z, w = (msg.orientation.x, msg.orientation.y,
                      msg.orientation.z, msg.orientation.w)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        self.imu_roll  = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

        sinp = 2.0 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        self.imu_pitch = math.degrees(math.asin(sinp))

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        self.imu_yaw   = math.degrees(math.atan2(siny_cosp, cosy_cosp))

        self.imu_last_time = now
        self.imu_hz_buf.append(now)
        self.imu_hz_buf = [t for t in self.imu_hz_buf if now - t <= 5.0]

    def cb_encoder(self, msg):
        """/encoder — ★[kasa] A보드 좌+우 펄스의 합(부호 없음)★

        환산식이 여기 리터럴로 박혀 있었는데(0.8482 / 300 / 0.01), driving.py·gps_imu.py
        와 세 곳이 각자 값을 갖고 있어 하나만 고치면 표시값이 조용히 틀어졌다.
        이제 white/kasa_units.py 하나만 본다."""
        now = time.time()
        self.encoder_val = msg.data
        self.encoder_speed_ms = ku.encoder_count_to_ms(float(msg.data))
        self.encoder_last_time = now
        self.encoder_hz_buf.append(now)
        self.encoder_hz_buf = [t for t in self.encoder_hz_buf if now - t <= 5.0]

    def cb_ego(self, msg):
        if len(msg.data) >= 7:
            self.ego_fused_lat = msg.data[0]
            self.ego_fused_lon = msg.data[1]
            self.ego_heading   = msg.data[4]
            self.ego_speed     = msg.data[5]
            self.ego_last_time = time.time()

    def cb_gps_status(self, msg):
        self.gps_status_str = msg.data

    def cb_lane_metrics(self, msg):
        if len(msg.data) < 9:
            return
        now = time.time()
        self.cam_cte_rear  = msg.data[0]
        self.cam_theta_deg = msg.data[2]
        self.cam_conf_eff  = msg.data[4]
        self.cam_width_m   = msg.data[5]
        self.cam_flags     = int(msg.data[6])
        self.cam_conf_raw  = msg.data[8]
        self.cam_last_time = now
        self.cam_hz_buf.append(now)
        self.cam_hz_buf = [t for t in self.cam_hz_buf if now - t <= 5.0]

    def cb_judgment(self, msg):
        self.judgment_state = msg.data
        self.judgment_time  = time.time()

    def cb_tl(self, msg):
        self.tl_state = msg.data
        self.tl_time  = time.time()

    # ── 유틸 ───────────────────────────────────────────
    def calc_hz(self, buf):
        if len(buf) < 2:
            return 0.0
        return (len(buf) - 1) / (buf[-1] - buf[0]) if (buf[-1] - buf[0]) > 0 else 0.0

    def age_str(self, last_time):
        if last_time is None:
            return "수신 없음"
        age = time.time() - last_time
        if age > 10.0:
            return f"⛔ {age:.0f}초 전 (단절)"
        if age > 2.0:
            return f"⚠️  {age:.1f}초 전"
        return f"✅ {age*1000:.0f}ms 전"

    def cam_age_str(self, last_time):
        """[카메라 전용] 융합이 실제로 쓰는 0.4초 기준으로 신선도를 판정한다.
        age_str() 의 2초/10초 기준을 쓰면 융합이 이미 버린 값을 ✅로 표시하게 된다."""
        if last_time is None:
            return "수신 없음"
        age = time.time() - last_time
        if age > CAM_FRESH_MAX_AGE:
            return f"⛔ {age:.2f}초 전 (융합 미반영)"
        return f"✅ {age*1000:.0f}ms 전"

    def cam_flag_str(self, flags):
        if flags == 0:
            return "없음"
        hits = [label for bit, label in CAM_FLAG_LABELS if flags & bit]
        return f"{'/'.join(hits)} (0b{flags:05b})"

    def gps_status_label(self, status):
        labels = {
            -1: "⛔ NO FIX   (위성 신호 없음)",
             0: "🟡 GPS FIX  (일반, 오차 ~2-5m)",
             1: "🟠 SBAS FIX (보정, 오차 ~1m)",
             2: "🟢 RTK FIX  (고정밀, 오차 ~2cm)",
        }
        return labels.get(status, f"❓ UNKNOWN ({status})")

    # ── 상태 출력 (1초 주기) ────────────────────────────
    def print_status(self):
        now = time.time()
        sep = "═" * 50

        print(f"\n{sep}")
        print(f"  🖥️  센서 상태 모니터  [{time.strftime('%H:%M:%S')}]")
        print(sep)

        # ── GPS ─────────────────────────────────────────
        gps_age = (now - self.gps_last_time) if self.gps_last_time else None
        gps_hz  = self.calc_hz(self.gps_hz_buf)
        print(f"  📡 [GPS / RTK]")
        print(f"     상태    : {self.gps_status_label(self.gps_status_raw)}")
        print(f"     수신    : {self.age_str(self.gps_last_time)}  ({gps_hz:.1f} Hz)")
        if self.gps_last_time:
            print(f"     좌표    : {self.gps_lat:.8f}, {self.gps_lon:.8f}")
        if gps_age and gps_age > 2.0:
            print(f"     ⚠️  GPS 수신 지연 {gps_age:.1f}초 — 위치 정확도 저하!")
        if self.gps_status_raw < 2 and self.gps_last_time:
            print(f"     ⚠️  RTK FIX 아님 — 주행 정확도가 낮을 수 있습니다.")

        print()

        # ── IMU ─────────────────────────────────────────
        imu_hz = self.calc_hz(self.imu_hz_buf)
        print(f"  🧭 [IMU]")
        print(f"     수신    : {self.age_str(self.imu_last_time)}  ({imu_hz:.1f} Hz)")
        if self.imu_last_time:
            print(f"     Roll    : {self.imu_roll:+.2f}°   Pitch: {self.imu_pitch:+.2f}°   Yaw: {self.imu_yaw:+.2f}°")
        if self.imu_last_time is None or (now - self.imu_last_time) > 1.0:
            print(f"     ⛔ IMU 데이터 없음 — /dev/ttyUSB0 연결 확인 필요!")

        print()

        # ── 엔코더 ──────────────────────────────────────
        enc_hz = self.calc_hz(self.encoder_hz_buf)
        print(f"  🔢 [엔코더 / 속도]")
        print(f"     수신    : {self.age_str(self.encoder_last_time)}  ({enc_hz:.1f} Hz)")
        if self.encoder_last_time:
            # [kasa] 값은 좌+우 펄스의 합(부호 없음). 1카운트 = 0.442 m/s = 1.59 km/h.
            #   후진이 없으므로 방향 표기는 하지 않는다(항상 전진 또는 정지).
            kmh = self.encoder_speed_ms * 3.6
            print(f"     카운트  : {self.encoder_val} (좌+우 합)  →  "
                  f"{self.encoder_speed_ms:.3f} m/s ({kmh:.1f} km/h)")
            print(f"     눈금    : 1카운트 = {ku.MS_PER_ENCODER_COUNT:.3f} m/s "
                  f"(양자화 하한 — 이보다 정밀한 속도는 알 수 없다)")
        if self.encoder_last_time is None or (now - self.encoder_last_time) > 1.0:
            print(f"     ⛔ 엔코더 없음 — nxde arduino 노드 / A보드 연결 확인 필요!")

        print()

        # ── 융합 상태 (ego_state) ────────────────────────
        print(f"  🚗 [융합 상태 (ego_state)]")
        print(f"     수신    : {self.age_str(self.ego_last_time)}")
        if self.ego_last_time:
            print(f"     헤딩    : {self.ego_heading:.1f}°")
            print(f"     속도    : {self.ego_speed:.3f} m/s")
            print(f"     융합위치: {self.ego_fused_lat:.8f}, {self.ego_fused_lon:.8f}")
        else:
            print(f"     ⚠️  ego_state 없음 — gps_imu 노드 헤딩 미확정 상태일 수 있음")

        print()

        # ── 카메라 / 차선 ────────────────────────────────
        print(f"  🎥 [카메라 / 차선]")
        if self.cam_last_time is None and self.count_publishers("/lane_metrics") == 0:
            # use_camera:=false 로 띄운 GPS 단독 주행. 고장이 아니므로 ⛔ 를 쓰지 않는다.
            print(f"     카메라 미사용 (GPS 단독 — use_camera:=false)")
        else:
            cam_hz  = self.calc_hz(self.cam_hz_buf)
            cam_age = (now - self.cam_last_time) if self.cam_last_time else None
            print(f"     수신    : {self.cam_age_str(self.cam_last_time)}  ({cam_hz:.1f} Hz)")
            if self.cam_last_time:
                print(f"     신뢰도  : conf_eff {self.cam_conf_eff:.2f}  /  raw {self.cam_conf_raw:.2f}")
                print(f"     차선    : 폭 {self.cam_width_m:.2f}m   CTE {self.cam_cte_rear:+.3f}m   θ {self.cam_theta_deg:+.2f}°")
                print(f"     flags   : {self.cam_flag_str(self.cam_flags)}")
            print(f"     게이트  : {self.judgment_state or '수신 없음'}"
                  f"     신호등: {self.tl_state or '수신 없음'}")

            if self.cam_last_time is None:
                print(f"     ⏳ /lane_metrics 대기 — perception/camera_judgment 기동 확인")
            elif cam_age is not None and cam_age > CAM_FRESH_MAX_AGE:
                print(f"     ⛔ {cam_age:.2f}초 두절 (>{CAM_FRESH_MAX_AGE}s) — "
                      f"gps_imu/driving 이 카메라를 무시 중!")
            elif self.cam_conf_eff <= 0.0 and self.cam_conf_raw > 0.0:
                # 여기가 이 블록의 핵심: 카메라는 잘 보고 있는데(raw>0) 게이트가
                # 죽여서(eff=0) 융합에 안 먹는 상태. 로그엔 아무것도 안 남는다.
                print(f"     ⚠️  게이트 차단 — 차선은 보이나 융합 미반영 "
                      f"(치명 flags: {self.cam_flag_str(self.cam_flags & CAM_FATAL_MASK)})")
            elif self.cam_flags & 16:
                print(f"     ⚠️  단독차선 추정 중 — 편향상쇄가 깨져 횡보정 신뢰도 낮음")

        print(sep)


def main(args=None):
    rclpy.init(args=args)
    node = SensorMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()