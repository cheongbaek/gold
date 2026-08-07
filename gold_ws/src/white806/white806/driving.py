#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
driving.py ― kasa 자율주행 [white806 / GPS+IMU 최소 추종판]
════════════════════════════════════════════════════════════════════════════════
구 white/driving.py(136KB)를 걷어내고 다시 쓴 것이다. 하드웨어가 kasa A/B 2보드로
바뀌어 기존 제어 파라미터가 맞지 않으므로, ★튜닝의 출발점★ 이 될 만큼 단순하게
되돌렸다. 없앤 것 — CTE 적분, 순수추종 기하, 가변 LFD 테이블, 지연보상 예측,
곡률 선행제동, 속도 PID, 후진, 카메라 융합, 지형/피치 보정.

남은 제어는 이것뿐이다:

    e = wrap180(목표WP 방위 − 현재 헤딩)
    조향 = clamp(−Kp · e, ±40°)        ★부호: − 좌 / + 우 (B보드 규약)★
    속도 = 고정 펄스

════════════════════════════════════════════════════════════════════════════════
 이 노드가 혼자 맡는 것 (구 white 와 다른 점)
════════════════════════════════════════════════════════════════════════════════
  · GPS+IMU 융합 — 구 gps_imu.py 노드는 white806 에 없다. /fix 와 /imu 를 직접 받아
    위치·헤딩을 여기서 만든다.
  · 단위 환산 없음 — /cmd_vel_raw 는 이미 ★펄스(0~15)·도(degree)★ 단위다
    (nxde/arduino.py 헤더 규약). 그래서 kasa_units 를 거치지 않는다.
  · 모드 스위치 상태기계 — prompt 의 메뉴가 아니라 ★B보드 D5 스위치의 엣지★ 가
    매핑·주행을 시작시킨다.

════════════════════════════════════════════════════════════════════════════════
 상태기계 — 트리거는 '스위치 전환' 이지 '스위치 위치' 가 아니다
════════════════════════════════════════════════════════════════════════════════
      IDLE ──(자율→수동, 하강)──▶ MAP_HEADING ──(헤딩확정)──▶ MAP_RUN
        ▲                              │                        │
        │                        (수동→자율)              (수동→자율, 상승)
        └──────────────────────────────┴────────────────────────┘  경로 저장

      IDLE ──(수동→자율, 상승)──▶ DRIVE_HEADING ──(헤딩확정)──▶ DRIVE_RUN
        ▲                                                          │
        │                                                   (마지막 WP 도달)
        │                                                          ▼
        └──────────(자율→수동, 하강)────────────────────────── DRIVE_DONE
                    record 저장 + 리니어 0단                  펄스0 + 리니어 2단

  ★같은 '상승 엣지' 라도 IDLE 에서는 주행 시작, MAP_RUN 에서는 매핑 종료다★
  그래서 레벨이 아니라 (상태 × 엣지) 로 갈라야 한다. 스위치가 이미 원하는 쪽에
  올라가 있으면 반대로 한 번 내렸다 올려야 하는 이유도 이것이다 — 엣지가 없으면
  아무 일도 일어나지 않는다.

  E-stop 은 어느 상태에서든 즉시 IDLE 로 되돌린다. 정지 자체는 아두이노가 이미
  하고 있으므로(A·B 보드가 자체 판정) 여기서 따로 정지 명령을 내지 않는다.

════════════════════════════════════════════════════════════════════════════════
 헤딩 초기화 — 왜 앞으로 굴러야 하는가
════════════════════════════════════════════════════════════════════════════════
  GPS 는 위치만 주고 방향을 주지 않는다. IMU 자이로는 '변화량'만 주므로 시작
  기준각이 없으면 절대 방위를 만들 수 없다. 그래서 출발할 때 ★조향 0 으로 곧게
  굴러가 그 변위 벡터를 정면으로 삼는다★.

  거리를 미리 정해 두지 않는다 — ★추정 오차가 목표치 아래로 떨어지는 순간 멈춘다★.
  오차는 이동거리에 반비례하므로(σ ≈ atan(위치노이즈 / 거리)) RTK 가 좋으면 1m
  남짓에서 끝나고, 흔들리면 더 간다. 확정 즉시 목표펄스를 0 으로 준다.

  ┌ 이동거리별 헤딩 오차 (두 점 기준, σ_pos 는 수평 위치 노이즈) ─────────────┐
  │  거리      RTK Fixed(2cm)     RTK Float(30cm)                            │
  │  0.5 m        3.2°               40°                                     │
  │  1.0 m        1.6°               23°     ← Fixed 면 여기서 이미 충분       │
  │  2.0 m        0.8°               16°                                     │
  └──────────────────────────────────────────────────────────────────────────┘
  Float 에서 초기 헤딩이 20° 틀리면 차가 엉뚱하게 조향하고, 그러면 GPS 코스헤딩도
  그 엉뚱한 방향을 가리켜 필터가 '자기 말이 맞다'고 수렴해 버린다 — 경로를 벗어난
  채로. 그래서 ★RTK Fixed 를 요구★ 하고(require_rtk), 직진성(잔차)까지 본다.

  매핑도 같은 절차를 쓴다. 수동조종 모드에서도 nxde/arduino 가 ★쓰로틀 우선, 발을
  뗐을 때만 /cmd_vel_raw 지정펄스★ 규칙으로 이 노드의 펄스를 그대로 실어 주므로
  (arduino.py compose() (2), 2026-08-07 개정), 모드를 속이는 오버라이드 없이 그냥
  펄스를 내보내면 된다. 사람이 페달을 밟는 순간 이 값은 즉시 밀려난다.

  ★단 수동조종에서 조향은 언제나 힘빼기('x')다★ 그러니 매핑 초기화 구간에서는
  ★사람이 핸들을 잡고 일자를 유지해야 한다★. 굽으면 잔차가 커져 헤딩이 확정되지
  않고 최대거리까지 굴러간다(그 경우 경고와 함께 확정한다).

════════════════════════════════════════════════════════════════════════════════
 리니어 브레이크
════════════════════════════════════════════════════════════════════════════════
  인휠은 코스트/회생뿐이라 스스로 빠르게 못 선다. 급감속이 필요할 때만 리니어를 쓴다.

    현재펄스 > 목표펄스 + 3   →  ★목표를 0 으로 덮고★ 리니어 2단
    현재펄스 − 원래목표 ≤ 1   →  즉시 리니어 0단 + 원래 목표펄스 복원
    그 사이(차이 3 미만)      →  브레이크 없이 목표펄스만 낮춰 관성으로 굴러간다

  ★브레이크를 무는 동안 목표펄스는 반드시 0★ 이어야 한다. 인휠이 밀고 리니어가
  잡으면 서로 싸운다(B보드 주석의 "인휠 PID 와 싸운다"와 같은 이유).

  도착·정지명령·자율주행 종료 시점에는 조건과 무관하게 2단을 물고 유지한다.
  푸는 것은 ①수동조종으로 스위치를 내릴 때 ②이 노드가 내려갈 때 두 경우다.
"""

import csv
import math
import os
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Bool, Float64MultiArray, Int32, String

from white806 import paths


# ══════════════════════════════════════════════════════════════════════════════
#  튜닝 상수 — 실차에서 만지는 곳은 전부 여기다
# ══════════════════════════════════════════════════════════════════════════════
CONTROL_HZ = 20.0            # 제어 주기. A보드 텔레메트리(20Hz)와 맞췄다

# ── 속도 (펄스. 1펄스 ≈ 0.884 m/s ≈ 3.18 km/h) ──
DRIVE_PULSE   = 4            # ★주행 고정 속도★ ≈ 12.7 km/h
HEADING_PULSE = 3            # 헤딩 초기화 중 속도 ≈ 9.5 km/h

# ── 조향 ──
STEER_KP      = 0.5          # 헤딩오차[deg] → 조향[deg]. ★사행이 나면 낮춘다★
STEER_MAX_DEG = 40           # B보드 STEER_ANGLE_MAX (kasa_0804_B.ino)

# ── 웨이포인트 ──
WP_REACH_M      = 0.2        # ★도달 허용반경★ 이 안에 들면 다음 WP 로 넘어간다
WP_MAX_SKIP     = 20         # 한 주기에 건너뛸 수 있는 WP 상한(맵 간격 0.25m 대비)

# ── 헤딩 초기화 ──
HEAD_MIN_DIST_M   = 1.0      # 이보다 짧으면 판정하지 않는다
HEAD_MAX_DIST_M   = 5.0      # 여기까지 가면 최선의 추정으로 확정하고 넘어간다
HEAD_MIN_SAMPLES  = 4        # 직진성을 보려면 최소 4점
HEAD_TARGET_SIGMA = 3.0      # [deg] 추정 오차가 이 밑이면 확정
HEAD_SIGMA_FLOOR  = 0.02     # [m] RTK Fixed 수평 노이즈 하한 — 낙관을 막는 바닥값
HEAD_MAX_RESID_M  = 0.15     # 직선 잔차 RMS 상한(곧게 갔는가)
MODE_SETTLE_S     = 0.7      # 스위치 엣지 직후 이만큼은 굴리지 않는다(빠른 토글 대비)

# ── GPS/IMU 융합 ──
FUSE_GAIN          = 0.05    # GPS 코스헤딩으로 끌어당기는 비율(상보필터)
FUSE_MIN_STEP_M    = 0.30    # 이만큼 움직였을 때만 GPS 코스헤딩을 신뢰한다
GPS_TIMEOUT_S      = 2.0     # 이 시간 넘게 /fix 가 없으면 정지
RTK_FIXED_STATUS   = 2       # NavSatFix.status.status (nmea_navsat_driver: GGA q4 → 2)

# ── 브레이크 ──
BRAKE_TRIGGER_DIFF = 3       # 현재펄스 − 목표펄스 가 이 이상이면 2단
BRAKE_RELEASE_DIFF = 1       # 현재펄스 − 원래목표 가 이 이하면 0단
BRAKE_FULL         = 2
BRAKE_NONE         = 0

# ── 엔코더 ──
#   /encoder 는 A보드 좌+우 펄스의 ★합★ 이므로 바퀴 하나 기준으로 보려면 2로 나눈다
#   (/cmd_vel_raw 의 펄스는 바퀴 하나 기준 0~15).
ENC_SUM_TO_PULSE = 0.5

EARTH_R = 6378137.0


def wrap180(deg):
    """각도를 [-180, 180) 로 접는다. ★이게 없으면 359°와 1°의 차이가 358°로
    계산되어 차가 반대로 꺾는다★"""
    return (deg + 180.0) % 360.0 - 180.0


def latlon_to_xy(lat, lon, lat0, lon0):
    """위경도 → 원점 기준 로컬 평면 [m]. x=동, y=북. 수백 m 범위에서 오차 무시."""
    x = EARTH_R * math.radians(lon - lon0) * math.cos(math.radians(lat0))
    y = EARTH_R * math.radians(lat - lat0)
    return x, y


# ══════════════════════════════════════════════════════════════════════════════
#  헤딩 초기화 추정기
# ══════════════════════════════════════════════════════════════════════════════
class HeadingEstimator:
    """곧게 굴러간 구간의 GPS 점들로 진행 방위를 낸다.

    방향은 ★첫 점 → 마지막 점★ 벡터로 낸다. 최소자승 회귀를 쓰지 않는 이유는
    표본이 적기 때문이다 — 등간격 N점 회귀의 방향 오차는 두 끝점 방식의 √(6/N)
    배라서, N ≤ 6 이면 오히려 회귀가 더 나쁘다. GPS 가 10Hz 이고 3펄스로 1m 를
    지나는 데 1초가 안 걸리니 표본은 대개 그 언저리다.

    대신 중간 점들은 ★직진성 검사★ 에 쓴다(두 끝점을 잇는 직선으로부터의 수직
    거리 RMS). 차가 휘었는데 헤딩을 확정해 버리는 것을 막는 장치다.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.pts = []          # [(x, y)]

    def add(self, x, y):
        # 같은 자리에서 여러 번 받는 것은 의미가 없다(정지 중 GPS 노이즈).
        if self.pts:
            px, py = self.pts[-1]
            if math.hypot(x - px, y - py) < 0.02:
                return
        self.pts.append((x, y))

    def distance(self):
        if len(self.pts) < 2:
            return 0.0
        (x0, y0), (x1, y1) = self.pts[0], self.pts[-1]
        return math.hypot(x1 - x0, y1 - y0)

    def solve(self):
        """→ (heading_deg, sigma_deg, resid_rms, n, dist). 표본 부족이면 None."""
        n = len(self.pts)
        if n < 2:
            return None
        (x0, y0), (x1, y1) = self.pts[0], self.pts[-1]
        dx, dy = x1 - x0, y1 - y0
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return None
        heading = math.degrees(math.atan2(dy, dx))

        # 두 끝점을 잇는 직선에서 각 점까지의 수직거리
        ux, uy = dx / d, dy / d          # 진행방향 단위벡터
        resid_sq = 0.0
        for (px, py) in self.pts:
            ex, ey = px - x0, py - y0
            cross = ex * uy - ey * ux    # 법선 방향 성분 = 수직거리(부호 있음)
            resid_sq += cross * cross
        resid_rms = math.sqrt(resid_sq / n)

        # 방향 오차 추정. 잔차가 0 이어도 GPS 노이즈 바닥값(2cm)은 깔고 본다 —
        # 표본이 두 점뿐이면 잔차가 구조적으로 0 이라 그대로 믿으면 안 된다.
        sigma = math.degrees(math.atan2(max(resid_rms, HEAD_SIGMA_FLOOR), d))
        return heading, sigma, resid_rms, n, d


# ══════════════════════════════════════════════════════════════════════════════
#  노드
# ══════════════════════════════════════════════════════════════════════════════
S_IDLE          = 'IDLE'
S_MAP_HEADING   = 'MAP_HEADING'
S_MAP_RUN       = 'MAP_RUN'
S_DRIVE_HEADING = 'DRIVE_HEADING'
S_DRIVE_RUN     = 'DRIVE_RUN'
S_DRIVE_DONE    = 'DRIVE_DONE'
S_ESTOP         = 'ESTOP'


class DrivingNode(Node):

    def __init__(self):
        super().__init__('driving_node')

        self.declare_parameter('data_dir', '')
        self.declare_parameter('drive_pulse', DRIVE_PULSE)
        self.declare_parameter('heading_pulse', HEADING_PULSE)
        self.declare_parameter('steer_kp', STEER_KP)
        self.declare_parameter('wp_reach_m', WP_REACH_M)
        self.declare_parameter('require_rtk', True)

        self.data_dir = paths.data_dir(self.get_parameter('data_dir').value or '')
        self.drive_pulse = int(self.get_parameter('drive_pulse').value)
        self.heading_pulse = int(self.get_parameter('heading_pulse').value)
        self.steer_kp = float(self.get_parameter('steer_kp').value)
        self.wp_reach = float(self.get_parameter('wp_reach_m').value)
        self.require_rtk = bool(self.get_parameter('require_rtk').value)

        # ── 센서 상태 ──
        self.lat0 = self.lon0 = None      # 로컬 평면 원점(첫 fix)
        self.x = self.y = 0.0
        self.fix_ok = False               # RTK 품질 만족
        self.fix_time = 0.0
        self._last_fuse_pt = None         # 코스헤딩 계산용 직전 점

        self.heading = None               # [deg] 확정 전에는 None
        self.gyro_z = 0.0                 # [rad/s] CCW +
        self.imu_time = 0.0

        self.enc_pulse = 0.0              # 바퀴 하나 기준 현재 펄스
        self.auto_mode = None             # /vehicle_mode. None = 미수신
        self.estop = False

        # ── 상태기계 ──
        self.state = S_IDLE
        self.state_t0 = time.time()
        self.head_est = HeadingEstimator()
        self.waypoints = []
        self.wp_idx = 0
        self.route_name = ''
        self.route_path = ''

        # ── 출력 상태 ──
        self.brake_now = BRAKE_NONE
        self.brake_latched = False
        self.brake_hold_target = 0

        # ── 퍼블리셔 ──
        self.pub_cmd   = self.create_publisher(Twist,  '/cmd_vel_raw',      10)
        self.pub_state = self.create_publisher(Bool,   '/control_state',    10)
        self.pub_brake = self.create_publisher(Int32,  '/brake_level',      10)
        # [2026-08-07] /vehicle_mode_cmd 발행이 사라졌다 — 주행모드는 물리 스위치
        #   전용이고, 수동조종에서의 펄스는 arduino 가 직접 받아 준다.
        self.pub_map   = self.create_publisher(Bool,   '/mapping_cmd',      10)
        self.pub_dstate = self.create_publisher(String, '/drive_state',     10)
        self.pub_event = self.create_publisher(String, '/drive_event',      10)
        self.pub_ego   = self.create_publisher(Float64MultiArray, '/ego_state', 10)

        # ── 구독 ──
        self.create_subscription(NavSatFix, '/fix',          self.cb_fix,     10)
        self.create_subscription(Imu,       '/imu',          self.cb_imu,     10)
        self.create_subscription(Int32,     '/encoder',      self.cb_encoder, 10)
        self.create_subscription(Bool,      '/vehicle_mode', self.cb_mode,    10)
        self.create_subscription(Bool,      '/estop',        self.cb_estop,   10)
        self.create_subscription(String,    '/drive_cmd',    self.cb_drive_cmd, 10)

        self.create_timer(1.0 / CONTROL_HZ, self.loop)

        self.event(f"white806 driving 준비 — 경로 폴더 {self.data_dir}")
        self.event("스위치 ↑(수동→자율) = 주행 시작 / ↓(자율→수동) = 매핑 시작")

    # ══════════════════════════════════════════════════════════════════════════
    #  수신
    # ══════════════════════════════════════════════════════════════════════════
    def cb_fix(self, msg: NavSatFix):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        if self.lat0 is None:
            self.lat0, self.lon0 = msg.latitude, msg.longitude
        self.x, self.y = latlon_to_xy(msg.latitude, msg.longitude,
                                      self.lat0, self.lon0)
        self.fix_ok = (not self.require_rtk) or (msg.status.status >= RTK_FIXED_STATUS)
        self.fix_time = time.time()

        if self.state in (S_MAP_HEADING, S_DRIVE_HEADING) and self.fix_ok:
            self.head_est.add(self.x, self.y)
        self._fuse_gps_course()

    def cb_imu(self, msg: Imu):
        now = time.time()
        if self.imu_time > 0.0 and self.heading is not None:
            dt = now - self.imu_time
            if 0.0 < dt < 0.5:
                # 자이로 z(CCW +)를 그대로 적분한다. 절대 기준은 초기화가 잡아준다.
                self.heading = wrap180(self.heading + math.degrees(self.gyro_z) * dt)
        self.gyro_z = float(msg.angular_velocity.z)
        self.imu_time = now

    def cb_encoder(self, msg: Int32):
        self.enc_pulse = float(msg.data) * ENC_SUM_TO_PULSE

    def cb_mode(self, msg: Bool):
        new = bool(msg.data)
        if self.auto_mode is None:
            self.auto_mode = new            # 첫 수신은 엣지로 치지 않는다
            self.event(f"주행모드 최초 인식: {'자율' if new else '수동조종'}")
            return
        if new == self.auto_mode:
            return
        self.auto_mode = new
        self.on_mode_edge(rising=new)

    def cb_estop(self, msg: Bool):
        new = bool(msg.data)
        if new and not self.estop:
            self.estop = True
            self.enter(S_ESTOP, "🚨 E-STOP — 모든 동작 중지, 메인화면 복귀")
        elif self.estop and not new:
            self.estop = False
            self.enter(S_IDLE, "✅ E-stop 해제 — 처음부터 다시")
        self.estop = new

    def cb_drive_cmd(self, msg: String):
        """prompt 하달. 'STOP' = 즉시 정지+리니어 2단 / 그 외 = 경로 선택."""
        cmd = str(msg.data).strip()
        if not cmd:
            return
        if cmd.upper() == 'STOP':
            if self.state in (S_DRIVE_HEADING, S_DRIVE_RUN):
                self.enter(S_DRIVE_DONE, "🛑 정지 명령 — 리니어 2단 체결")
            else:
                self.enter(S_IDLE, "🛑 정지 명령")
            return
        self.select_route(cmd)

    # ══════════════════════════════════════════════════════════════════════════
    #  경로
    # ══════════════════════════════════════════════════════════════════════════
    def select_route(self, name):
        path = os.path.join(self.data_dir, name)
        if not os.path.isfile(path):
            self.event(f"❌ 경로 파일 없음: {path}")
            return
        wps = []
        try:
            with open(path, 'r', encoding='utf-8') as f:
                for row in csv.DictReader(f):
                    try:
                        wps.append((float(row['latitude']), float(row['longitude'])))
                    except (KeyError, ValueError, TypeError):
                        continue
        except Exception as e:
            self.event(f"❌ 경로 읽기 실패: {e}")
            return
        if len(wps) < 2:
            self.event(f"❌ 웨이포인트가 부족하다({len(wps)}개): {name}")
            return
        self.route_name, self.route_path, self.raw_wps = name, path, wps
        self.event(f"📁 경로 선택: {name} (WP {len(wps)}개) — 스위치를 자율로 올리면 출발")

    def build_waypoints(self):
        """선택된 경로를 현재 원점 기준 로컬 좌표로 변환."""
        if self.lat0 is None or not getattr(self, 'raw_wps', None):
            return False
        self.waypoints = [latlon_to_xy(la, lo, self.lat0, self.lon0)
                          for (la, lo) in self.raw_wps]
        self.wp_idx = 0
        return True

    # ══════════════════════════════════════════════════════════════════════════
    #  상태 전이
    # ══════════════════════════════════════════════════════════════════════════
    def on_mode_edge(self, rising: bool):
        """스위치 엣지 → 상태 전이.

        ┌ 현재상태      │ 하강(자율→수동)      │ 상승(수동→자율)          ┐
        │ IDLE          │ MAP_HEADING          │ DRIVE_HEADING            │
        │ MAP_HEADING   │ —                    │ DRIVE_HEADING (매핑취소) │
        │ MAP_RUN       │ —                    │ IDLE (경로 저장)         │
        │ DRIVE_HEADING │ MAP_HEADING (주행취소)│ —                        │
        │ DRIVE_RUN     │ IDLE (주행 중단)     │ —                        │
        │ DRIVE_DONE    │ IDLE (기록저장·해제) │ —                        │
        └───────────────┴──────────────────────┴──────────────────────────┘

        ★준비단계(*_HEADING)끼리 서로 넘어가는 칸이 요령의 핵심이다★
        "이미 자율인데 주행하고 싶으면 수동으로 한 번 내렸다 올린다"가 성립하려면,
        내렸을 때 들어간 MAP_HEADING 에서 올렸을 때 매핑 종료가 아니라 주행 시작이
        되어야 한다. 반대(수동에서 매핑을 다시 시작)도 같은 이유로 대칭이다.
        실제 수집·기록이 시작된 뒤(MAP_RUN / DRIVE_RUN)라야 '종료'로 취급한다.
        """
        if self.state == S_ESTOP:
            return
        if rising:                                   # 수동 → 자율
            if self.state in (S_IDLE, S_MAP_HEADING):
                if not getattr(self, 'raw_wps', None):
                    self.event("⚠️ 경로가 선택되지 않았다 — prompt 에서 먼저 고를 것")
                    self.enter(S_IDLE)
                    return
                self.enter(S_DRIVE_HEADING, f"▶ 주행 시작 [{self.route_name}]")
            elif self.state == S_MAP_RUN:
                self.enter(S_IDLE, "🗺️ 매핑 종료 — 경로 저장")
        else:                                        # 자율 → 수동
            if self.state in (S_IDLE, S_DRIVE_HEADING):
                self.enter(S_MAP_HEADING, "🗺️ 매핑 시작 — 헤딩 초기화")
            elif self.state == S_DRIVE_RUN:
                self.enter(S_IDLE, "■ 주행 중단")
            elif self.state == S_DRIVE_DONE:
                self.enter(S_IDLE, "■ 주행 종료 — 기록 저장, 리니어 해제")

    def enter(self, new_state, msg=""):
        old = self.state
        self.state = new_state
        self.state_t0 = time.time()
        if msg:
            self.event(msg)

        # 매핑 수집은 ★헤딩이 잡힌 뒤(MAP_RUN)★ 시작한다. 준비단계에서 켜면
        # 스위치를 잠깐 토글할 때마다 빈 경로 파일이 쌓인다.
        if new_state == S_MAP_RUN:
            self.pub_map.publish(Bool(data=True))
        elif old == S_MAP_RUN:
            self.pub_map.publish(Bool(data=False))

        if new_state in (S_MAP_HEADING, S_DRIVE_HEADING):
            self.head_est.reset()
            self.heading = None
            if new_state == S_DRIVE_HEADING and not self.build_waypoints():
                self.enter(S_IDLE, "❌ GPS 원점이 없어 경로를 세울 수 없다")
                return

        # 브레이크
        if new_state == S_DRIVE_DONE:
            self.set_brake(BRAKE_FULL)
        elif new_state in (S_IDLE, S_ESTOP):
            self.set_brake(BRAKE_NONE)
            self.brake_latched = False

    # ══════════════════════════════════════════════════════════════════════════
    #  융합
    # ══════════════════════════════════════════════════════════════════════════
    def _fuse_gps_course(self):
        """GPS 변위 방위로 헤딩을 천천히 끌어당긴다(상보필터).

        ★정지·저속에서는 쓰지 않는다★ — 1cm 노이즈가 방향을 180° 뒤집는다.
        FUSE_MIN_STEP_M 이상 움직였을 때만 한 번 반영한다.
        """
        if self.heading is None or not self.fix_ok:
            self._last_fuse_pt = (self.x, self.y)
            return
        if self._last_fuse_pt is None:
            self._last_fuse_pt = (self.x, self.y)
            return
        px, py = self._last_fuse_pt
        dx, dy = self.x - px, self.y - py
        if math.hypot(dx, dy) < FUSE_MIN_STEP_M:
            return
        course = math.degrees(math.atan2(dy, dx))
        self.heading = wrap180(self.heading + FUSE_GAIN * wrap180(course - self.heading))
        self._last_fuse_pt = (self.x, self.y)

    # ══════════════════════════════════════════════════════════════════════════
    #  제어 루프
    # ══════════════════════════════════════════════════════════════════════════
    def loop(self):
        self.publish_state_topics()

        if self.state in (S_IDLE, S_ESTOP):
            self.send(0, 0.0, control=False)
            return

        # GPS 두절 — 위치를 모르면 어떤 판단도 신뢰할 수 없다
        if time.time() - self.fix_time > GPS_TIMEOUT_S:
            self.send(0, 0.0, control=True)
            self.throttle_event("⚠️ GPS 두절 — 정지 유지")
            return

        if self.state == S_MAP_HEADING:
            self.run_heading_init(next_state=S_MAP_RUN)
        elif self.state == S_DRIVE_HEADING:
            self.run_heading_init(next_state=S_DRIVE_RUN)
        elif self.state == S_MAP_RUN:
            # 사람이 페달로 몬다. 이 노드는 아무 명령도 내지 않는다
            # (nxde 가 수동조종에서 /cmd_vel_raw 를 무시하기도 한다).
            self.send(0, 0.0, control=False)
        elif self.state == S_DRIVE_RUN:
            self.run_follow()
        elif self.state == S_DRIVE_DONE:
            self.send(0, 0.0, control=True)     # 브레이크 2단은 enter() 에서 물렸다

    # ── 헤딩 초기화 ────────────────────────────────────────────────────────────
    def run_heading_init(self, next_state):
        # ★진입 직후 잠깐은 굴리지 않는다★ 요령상 스위치를 빠르게 토글하는 경우가
        #   있는데(자율↔수동 왕복), 그 찰나마다 차가 튀어나가면 위험하다.
        if time.time() - self.state_t0 < MODE_SETTLE_S:
            self.send(0, 0.0, control=True)
            return

        self.send(self.heading_pulse, 0.0, control=True)   # 조향 0 으로 곧게

        sol = self.head_est.solve()
        if sol is None:
            return
        heading, sigma, resid, n, dist = sol

        if dist < HEAD_MIN_DIST_M:
            return

        good = (n >= HEAD_MIN_SAMPLES
                and sigma <= HEAD_TARGET_SIGMA
                and resid <= HEAD_MAX_RESID_M)
        forced = dist >= HEAD_MAX_DIST_M

        if not (good or forced):
            return

        self.heading = heading
        self._last_fuse_pt = (self.x, self.y)
        if next_state == S_DRIVE_RUN:
            self.align_start_wp()
        self.send(0, 0.0, control=True)        # ★확정 즉시 목표펄스 0★
        mark = "" if good else "  ⚠️(최대거리 도달 — 정확도 미달인 채 확정)"
        self.event(f"🧭 헤딩 확정 {heading:+.1f}° "
                   f"(±{sigma:.1f}°, {dist:.2f}m, {n}점, 잔차 {resid*100:.1f}cm){mark}")
        self.enter(next_state)

    def align_start_wp(self):
        """출발 WP 를 현재 위치에 맞춘다.

        ★없으면 차가 유턴한다★ 헤딩 초기화로 1~2m 앞으로 나온 뒤 wp_idx=0 을
        그대로 쓰면, 이미 지나온 첫 WP 가 등 뒤에 있어 목표 방위가 180° 로 나온다.
        조향이 포화된 채 경로를 벗어나 원을 그린다(시뮬레이션에서 그대로 재현됐다).

        최근접 WP 를 찾고, 그것이 등 뒤(오차 90° 초과)면 하나 앞을 잡는다.
        """
        if not self.waypoints:
            return
        best_i, best_d = 0, float('inf')
        for i, (wx, wy) in enumerate(self.waypoints):
            d = math.hypot(wx - self.x, wy - self.y)
            if d < best_d:
                best_i, best_d = i, d
        if self.heading is not None and best_i < len(self.waypoints) - 1:
            wx, wy = self.waypoints[best_i]
            ang = wrap180(math.degrees(math.atan2(wy - self.y, wx - self.x))
                          - self.heading)
            if abs(ang) > 90.0:
                best_i += 1
        self.wp_idx = best_i
        self.event(f"📍 출발 WP {best_i}/{len(self.waypoints)} "
                   f"(최근접 {best_d:.2f}m)")

    # ── 경로 추종 ──────────────────────────────────────────────────────────────
    def run_follow(self):
        if self.heading is None or not self.waypoints:
            self.send(0, 0.0, control=True)
            return

        # 목표 WP 전진. ★while 인 이유★ 맵 간격이 0.25m 라 반경 0.2m 라도 한 주기에
        # 여러 개를 지나칠 수 있다. if 로 두면 인덱스가 뒤처져 이미 지나온 점을
        # 향해 조향한다.
        skipped = 0
        while self.wp_idx < len(self.waypoints) - 1 and skipped < WP_MAX_SKIP:
            tx, ty = self.waypoints[self.wp_idx]
            if math.hypot(tx - self.x, ty - self.y) > self.wp_reach:
                break
            self.wp_idx += 1
            skipped += 1

        # 종점 판정 — 마지막 WP 에 도달하면 즉시 정지 + 리니어 2단
        gx, gy = self.waypoints[-1]
        d2goal = math.hypot(gx - self.x, gy - self.y)
        if self.wp_idx >= len(self.waypoints) - 1 and d2goal <= self.wp_reach:
            self.enter(S_DRIVE_DONE,
                       f"🎯 도착 — 마지막 WP {d2goal:.2f}m, 정지 + 리니어 2단")
            return

        tx, ty = self.waypoints[self.wp_idx]
        bearing = math.degrees(math.atan2(ty - self.y, tx - self.x))
        err = wrap180(bearing - self.heading)
        # ★부호★ err>0 은 목표가 반시계(왼쪽) 방향이라는 뜻이고, B보드 규약은
        #   − 가 좌회전이므로 음수를 실어야 한다.
        steer = max(-STEER_MAX_DEG, min(STEER_MAX_DEG, -self.steer_kp * err))
        self.send(self.drive_pulse, steer, control=True)

    # ══════════════════════════════════════════════════════════════════════════
    #  출력
    # ══════════════════════════════════════════════════════════════════════════
    def send(self, target_pulse, steer_deg, control):
        """목표펄스·조향을 내보낸다. 브레이크 개입 판단도 여기서 한다."""
        target_pulse = int(max(0, min(15, target_pulse)))

        if self.state == S_DRIVE_DONE:
            target_pulse = 0                       # 도착 후에는 무조건 0
        else:
            target_pulse = self.apply_brake_policy(target_pulse)

        msg = Twist()
        msg.linear.x = float(target_pulse)         # ★펄스 그대로 (m/s 아님)★
        msg.angular.z = float(steer_deg)           # ★− 좌 / + 우★
        self.pub_cmd.publish(msg)
        self.pub_state.publish(Bool(data=bool(control)))

    def apply_brake_policy(self, target):
        """급감속이 필요할 때만 리니어를 문다. 규칙은 파일 헤더 참고."""
        cur = self.enc_pulse

        if not self.brake_latched:
            if cur > target + BRAKE_TRIGGER_DIFF:
                self.brake_latched = True
                self.brake_hold_target = target
                self.set_brake(BRAKE_FULL)
                self.event(f"🛑 리니어 2단 — 현재 {cur:.1f} > 목표 {target}+"
                           f"{BRAKE_TRIGGER_DIFF}")
                return 0
            return target

        # 물려 있는 동안 목표는 0. 원래 목표와의 차이가 좁혀지면 즉시 푼다.
        if cur - self.brake_hold_target <= BRAKE_RELEASE_DIFF:
            self.brake_latched = False
            self.set_brake(BRAKE_NONE)
            self.event(f"✅ 리니어 해제 — 현재 {cur:.1f}, 목표 {self.brake_hold_target} 복귀")
            return self.brake_hold_target
        return 0

    def set_brake(self, level):
        if level == self.brake_now:
            return
        self.brake_now = level
        self.pub_brake.publish(Int32(data=int(level)))

    def publish_state_topics(self):
        self.pub_dstate.publish(String(data=self.state))
        ego = Float64MultiArray()
        ego.data = [
            float(self.x), float(self.y),
            float(self.heading if self.heading is not None else 0.0),
            float(self.enc_pulse),
            float(self.wp_idx), float(len(self.waypoints)),
            1.0 if self.fix_ok else 0.0,
        ]
        self.pub_ego.publish(ego)

    def event(self, text):
        self.pub_event.publish(String(data=text))
        self.get_logger().info(text)

    def throttle_event(self, text):
        self.get_logger().warning(text, throttle_duration_sec=3.0)

    # ══════════════════════════════════════════════════════════════════════════
    def destroy_node(self):
        # ★내려갈 때 리니어를 풀어 준다★ 물린 채로 죽으면 사람이 차를 못 민다.
        try:
            self.pub_brake.publish(Int32(data=BRAKE_NONE))
            self.pub_state.publish(Bool(data=False))
            self.pub_cmd.publish(Twist())
            # 시리얼로 실제 나갈 시간을 준다(arduino 노드가 아직 살아 있을 때).
            end = time.time() + 0.4
            while time.time() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DrivingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
