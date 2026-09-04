#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hud.py ― kasa 차량 상태 HUD  [white1]
════════════════════════════════════════════════════════════════════════════════
    ros2 run white1 hud
    ros2 launch white1 one_launch.py          # use_hud:=true (기본)
    ros2 launch lidar one_launch.py           # 동일

★구독만 한다★ /cmd_vel_raw · /control_state · /brake_level · /drive_cmd 를
발행하지 않는다. prompt 와 대기 상태를 나눠 갖지 않고, master/joystick 과도
명령이 겹치지 않는다. 떠 있는 스택(white1 · lidar AEB · joy) 위에 그냥 얹는다.

화면 (F1 계기판을 이 차 토픽에 맞춘 것):
    상단   모드 · E-STOP · AEB · 신호등 · GPS · A/B 보드 · /drive_state
    중앙   상면도 차체 + 앞바퀴 조향 + 모서리 원형 게이지 4개
             차 앞 = 라이다 AEB 범위(cone_lidar ROI). 거리 없으면 안 그림
             FL 스로틀%   FR PWM%   RL 브레이크   RR 펄스 라벨
    우측   속도 km/h (/encoder × 3.18, lidar 와 동일) · PWM · 펄스
           NAV 미니맵
             nav_mode=gps  (기본) 매핑 CSV(북쪽 위) + 실시간 GPS. 경로 없으면 NONE
             nav_mode=mppi 차량 기준 2D 탑뷰 — 코스트맵 장애물 + 롤아웃 + IMU 기준선
               (mppi_local_planner one_launch 가 이 모드로 띄운다)
    하단   조향바 · 헤딩 · CTE · 웨이포인트 · 이벤트 · 토픽 신선도

값이 1.5초 넘게 안 오면 '—' / 회색. 죽은 노드의 마지막 숫자를 현재인 양
띄워 두지 않는다.

키: F11 전체화면, Esc 전체화면 해제(창 닫기는 창 버튼).
카메라 미리보기: show_camera:=false 로 끈다.
"""

from __future__ import annotations

import csv
import math
import os
import re
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import Image, Imu, NavSatFix
from std_msgs.msg import Bool, Float32, Float64MultiArray, Int32, String

from white1.gps import GPS_FUSED_FIELDS, Q_LABEL  # fused 길이·품질 라벨의 단일 소유자
from white1 import paths as wpaths

try:
    from nxde.proc_guard import watch_parent
except Exception:  # noqa: BLE001
    watch_parent = None

try:
    import tkinter as tk
    from tkinter import font as tkfont
except ImportError as exc:
    raise SystemExit(
        "tkinter 가 없다 — `sudo apt install python3-tk` 후 다시 실행할 것") from exc

try:
    import cv2
    from cv_bridge import CvBridge
    _HAVE_CV = True
except Exception:  # noqa: BLE001
    cv2 = None
    CvBridge = None
    _HAVE_CV = False


# ── 환산 (arduino.py / driving.py 와 같은 숫자) ──────────────────────────────
THROTTLE_RAW_MIN = 220
THROTTLE_RAW_MAX = 950
PWM_MAX = 255
PULSE_MAX = 15
STEER_MAX = 40
KMH_PER_PULSE = 3.18
STALE_S = 1.5
UI_MS = 50
EARTH_R = 6378137.0
ROUTE_CMD_WORDS = ('STOP', 'MAP_START', 'DRIVE_START')
ROUTE_CSV_RE = re.compile(r'([^\s\[\]/\\]+\.csv)')
MAP_PT_RE = re.compile(
    r'lat=\s*([+-]?\d+(?:\.\d+)?)\s+lon=\s*([+-]?\d+(?:\.\d+)?)', re.I)
MAP_STATES = ('MAP_HEADING', 'MAP_RUN')
DRIVE_STATES = ('DRIVE_HEADING', 'DRIVE_RUN', 'DRIVE_DONE')
TRAIL_MIN_M = 0.20
TRAIL_MAX_N = 4000
NAV_DRAW_MAX = 280

# ── 색 ──────────────────────────────────────────────────────────────────────
BG = '#07090d'
PANEL = '#10141c'
PANEL2 = '#161b24'
FG = '#e8eaed'
DIM = '#6b7380'
GOLD = '#d4a017'
GREEN = '#3dff8a'
GREEN_DIM = '#1e8a4c'
CYAN = '#5ce1ff'
ORANGE = '#ff9d2e'
RED = '#ff3355'
YELLOW = '#ffe066'
BODY_OK = '#2bdc74'
BODY_MAN = '#c9a227'
BODY_HOT = '#ff3355'


def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def _lerp(a, b, t):
    return a + (b - a) * t


def _hex(rgb):
    r, g, b = (int(_clamp(v, 0, 255)) for v in rgb)
    return f'#{r:02x}{g:02x}{b:02x}'


def _mix(c0, c1, t):
    t = _clamp(t, 0.0, 1.0)
    a = tuple(int(c0[i:i + 2], 16) for i in (1, 3, 5))
    b = tuple(int(c1[i:i + 2], 16) for i in (1, 3, 5))
    return _hex(tuple(_lerp(a[i], b[i], t) for i in range(3)))


def _latlon_to_xy(lat, lon, lat0, lon0):
    """위경도 → 로컬 m. x=동, y=북. driving.latlon_to_xy 와 같다."""
    x = EARTH_R * math.radians(lon - lon0) * math.cos(math.radians(lat0))
    y = EARTH_R * math.radians(lat - lat0)
    return x, y


def _yaw_from_quat(x, y, z, w):
    """IMU 쿼터니언 → yaw [deg]. ROS ENU: 0=동, CCW. driving 헤딩과 같은 규약."""
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny, cosy))


def _gauge_color(frac):
    """낮음=녹, 중간=황, 높음=적 — F1 타이어 마모 링과 같은 방향."""
    f = _clamp(frac, 0.0, 1.0)
    if f < 0.45:
        return _mix(GREEN, YELLOW, f / 0.45)
    if f < 0.8:
        return _mix(YELLOW, ORANGE, (f - 0.45) / 0.35)
    return _mix(ORANGE, RED, (f - 0.8) / 0.2)


def _frac_raw(raw, lo, hi):
    if raw is None or hi <= lo:
        return None
    return _clamp((float(raw) - lo) / (hi - lo), 0.0, 1.0)


class Sample:
    """값 + 수신시각. 신선하지 않으면 화면에 숫자를 남기지 않는다."""

    __slots__ = ('v', 't')

    def __init__(self):
        self.v = None
        self.t = 0.0

    def set(self, v):
        self.v = v
        self.t = time.monotonic()

    def age(self):
        return 1e9 if self.t <= 0.0 else time.monotonic() - self.t

    def fresh(self, s=STALE_S):
        return self.v is not None and self.age() < s

    def get(self, s=STALE_S):
        return self.v if self.fresh(s) else None


class HudNode(Node):
    """콜백은 값만 넣는다. 위젯은 만지지 않는다 (Tk 는 메인 스레드 전용)."""

    def __init__(self):
        super().__init__('hud_node')
        self.declare_parameter('throttle_raw_min', THROTTLE_RAW_MIN)
        self.declare_parameter('throttle_raw_max', THROTTLE_RAW_MAX)
        self.declare_parameter('pwm_max', PWM_MAX)
        self.declare_parameter('pulse_max', PULSE_MAX)
        self.declare_parameter('steer_max', STEER_MAX)
        self.declare_parameter('stale_s', STALE_S)
        self.declare_parameter('show_camera', True)
        self.declare_parameter('data_dir', '')
        self.declare_parameter('aeb_dist_topic',
                               '/cone_lidar_node/obstacle_distance')
        self.declare_parameter('aeb_signal_topic',
                               '/cone_lidar_node/stop_signal')
        # cone_lidar.yaml 과 같은 기본값. 라이다 런치가 덮어쓸 수 있다.
        self.declare_parameter('lidar_range_min', 2.0)
        self.declare_parameter('lidar_range_max', 15.0)
        self.declare_parameter('vehicle_front_m', 1.2)
        self.declare_parameter('corridor_half_m', 0.8)
        # NAV 칸. gps = 매핑 CSV 미니맵(white1). mppi = 코스트맵 2D 탑뷰.
        self.declare_parameter('nav_mode', 'gps')
        self.declare_parameter('mppi_costmap_topic',
                               '/mppi_local_planner/costmap')
        self.declare_parameter('mppi_path_topic',
                               '/mppi_local_planner/local_path')
        self.declare_parameter('mppi_ref_path_topic',
                               '/mppi_local_planner/reference_path')
        self.declare_parameter('wheelbase_m', 1.25)
        self.declare_parameter('track_width_m', 1.10)
        self.declare_parameter('rear_overhang_m', 0.30)

        self.thr_lo = int(self.get_parameter('throttle_raw_min').value)
        self.thr_hi = int(self.get_parameter('throttle_raw_max').value)
        self.pwm_max = int(self.get_parameter('pwm_max').value)
        self.pulse_max = int(self.get_parameter('pulse_max').value)
        self.steer_max = float(self.get_parameter('steer_max').value)
        self.stale_s = float(self.get_parameter('stale_s').value)
        self.show_camera = bool(self.get_parameter('show_camera').value)
        self.data_dir = wpaths.data_dir(
            str(self.get_parameter('data_dir').value or ''))
        aeb_dist = str(self.get_parameter('aeb_dist_topic').value)
        aeb_sig = str(self.get_parameter('aeb_signal_topic').value)
        self.lidar_rmin = float(self.get_parameter('lidar_range_min').value)
        self.lidar_rmax = float(self.get_parameter('lidar_range_max').value)
        self.vehicle_front_m = float(self.get_parameter('vehicle_front_m').value)
        self.corridor_half_m = float(self.get_parameter('corridor_half_m').value)
        self.nav_mode = str(self.get_parameter('nav_mode').value or 'gps').strip().lower()
        if self.nav_mode not in ('gps', 'mppi'):
            self.nav_mode = 'gps'
        self.wheelbase_m = float(self.get_parameter('wheelbase_m').value)
        self.track_width_m = float(self.get_parameter('track_width_m').value)
        self.rear_overhang_m = float(self.get_parameter('rear_overhang_m').value)
        if self.lidar_rmax <= self.lidar_rmin:
            self.lidar_rmax = self.lidar_rmin + 1.0

        self.throttle = Sample()
        self.pwm = Sample()
        self.pulse = Sample()
        self.brake_lv = Sample()
        self.brake_pot = Sample()
        self.encoder = Sample()
        self.steer = Sample()
        self.speed = Sample()
        self.mode = Sample()
        self.estop = Sample()
        self.aeb = Sample()
        self.aeb_dist = Sample()
        self.aeb_sig = Sample()
        self.boards = Sample()
        self.ctrl = Sample()
        self.cmd_pulse = Sample()
        self.cmd_steer = Sample()
        self.drive_state = Sample()
        #  ★조종권 [2026-09-01]★ 지금 /cmd_vel_raw 를 누가 내고 있나. 이 표시가
        #  없으면 주행 중에 GPS 추종과 라이다 회피 중 무엇이 차를 몰고 있는지
        #  화면으로 알 수 없다 — 라바콘 구간에서 조향이 이상할 때 원인을 못 가른다.
        self.lidar_permit = Sample()      # driving → mppi : "네가 몰아라"
        self.lidar_active = Sample()      # mppi → driving : "나 살아 있다"
        self.drive_event = Sample()
        self.ego = Sample()
        self.diag = Sample()
        self.fused = Sample()
        self.gps_q = Sample()
        self.fix = Sample()
        self.tl = Sample()
        self.tl_brake = Sample()
        self.tl_enable = Sample()
        self.tl_permit = Sample()
        self.tl_wait = Sample()
        self.tl_near = Sample()
        self.imu = Sample()
        self.vel = Sample()
        self.prompt_wait = Sample()
        self.map_cmd = Sample()
        self.map_point = Sample()

        self._nav_lock = threading.Lock()
        self.route_name = ''
        self.route_wps = []          # [(lat, lon), ...] 매핑 CSV
        self.map_trail = []          # 매핑 중 /mapping_point·/fix 로 쌓는 점

        self._mppi_lock = threading.Lock()
        self._mppi_grid = None       # {w,h,res,ox,oy,data}
        self._mppi_grid_t = 0.0
        self._mppi_path = []         # [(x, y), ...] ego, x 전방 / y 좌
        self._mppi_path_t = 0.0
        self._mppi_ref = []
        self._mppi_ref_t = 0.0

        self._img_lock = threading.Lock()
        self._img_bgr = None
        self._img_t = 0.0
        self._bridge = CvBridge() if _HAVE_CV else None

        qos = 10
        self.create_subscription(Int32, '/throttle_pedal',
                                 lambda m: self.throttle.set(int(m.data)), qos)
        self.create_subscription(Int32, '/drive_pwm_cmd',
                                 lambda m: self.pwm.set(int(m.data)), qos)
        self.create_subscription(Int32, '/drive_pulse_cmd',
                                 lambda m: self.pulse.set(int(m.data)), qos)
        self.create_subscription(Int32, '/brake_level',
                                 lambda m: self.brake_lv.set(int(m.data)), qos)
        self.create_subscription(Int32, '/brake_pot',
                                 lambda m: self.brake_pot.set(int(m.data)), qos)
        self.create_subscription(Int32, '/encoder',
                                 lambda m: self.encoder.set(int(m.data)), qos)
        self.create_subscription(Int32, '/steer_angle_measured',
                                 lambda m: self.steer.set(int(m.data)), qos)
        self.create_subscription(Float32, '/speed',
                                 lambda m: self.speed.set(float(m.data)), qos)
        self.create_subscription(Bool, '/vehicle_mode',
                                 lambda m: self.mode.set(bool(m.data)), qos)
        self.create_subscription(Bool, '/estop',
                                 lambda m: self.estop.set(bool(m.data)), qos)
        self.create_subscription(Bool, '/aeb_stop',
                                 lambda m: self.aeb.set(bool(m.data)), qos)
        self.create_subscription(String, '/board_status',
                                 lambda m: self.boards.set(str(m.data)), qos)
        self.create_subscription(Bool, '/control_state',
                                 lambda m: self.ctrl.set(bool(m.data)), qos)
        self.create_subscription(Twist, '/cmd_vel_raw', self._cb_cmd, qos)
        self.create_subscription(String, '/drive_state',
                                 lambda m: self.drive_state.set(str(m.data)), qos)
        self.create_subscription(String, '/drive_event', self._cb_event, qos)
        self.create_subscription(String, '/drive_cmd', self._cb_drive_cmd, qos)
        self.create_subscription(String, '/mapping_point',
                                 self._cb_map_point, qos)
        self.create_subscription(Float64MultiArray, '/ego_state',
                                 lambda m: self.ego.set(list(m.data)), qos)
        self.create_subscription(Float64MultiArray, '/drive_diag',
                                 lambda m: self.diag.set(list(m.data)), qos)
        self.create_subscription(Float64MultiArray, '/gps_fused',
                                 lambda m: self.fused.set(list(m.data)), qos)
        self.create_subscription(String, '/gps_quality',
                                 lambda m: self.gps_q.set(str(m.data)), qos)
        self.create_subscription(NavSatFix, '/fix', self._cb_fix, qos)
        self.create_subscription(String, '/tl/state',
                                 lambda m: self.tl.set(str(m.data)), qos)
        self.create_subscription(Int32, '/tl_brake_req',
                                 lambda m: self.tl_brake.set(int(m.data)), qos)
        self.create_subscription(Bool, '/tl_enable',
                                 lambda m: self.tl_enable.set(bool(m.data)), qos)
        self.create_subscription(Bool, '/tl_permit',
                                 lambda m: self.tl_permit.set(bool(m.data)), qos)
        self.create_subscription(Bool, '/lidar_permit',
                                 lambda m: self.lidar_permit.set(bool(m.data)), qos)
        self.create_subscription(Bool, '/lidar_active',
                                 lambda m: self.lidar_active.set(bool(m.data)), qos)
        self.create_subscription(Bool, '/tl/stop_line_wait',
                                 lambda m: self.tl_wait.set(bool(m.data)), qos)
        self.create_subscription(Float32, '/tl/near_metric',
                                 lambda m: self.tl_near.set(float(m.data)), qos)
        self.create_subscription(Imu, '/imu', self._cb_imu, qos)
        self.create_subscription(TwistStamped, '/vel',
                                 lambda m: self.vel.set(float(m.twist.linear.x)),
                                 qos)
        self.create_subscription(String, '/prompt_wait',
                                 lambda m: self.prompt_wait.set(str(m.data)), qos)
        self.create_subscription(Bool, '/mapping_cmd', self._cb_map_cmd, qos)
        self.create_subscription(Float32, aeb_dist,
                                 lambda m: self.aeb_dist.set(float(m.data)), qos)
        self.create_subscription(Bool, aeb_sig,
                                 lambda m: self.aeb_sig.set(bool(m.data)), qos)
        if self.show_camera and self._bridge is not None:
            self.create_subscription(
                Image, '/image_raw', self._cb_image, qos_profile_sensor_data)
        if self.nav_mode == 'mppi':
            cmap = str(self.get_parameter('mppi_costmap_topic').value)
            pth = str(self.get_parameter('mppi_path_topic').value)
            ref = str(self.get_parameter('mppi_ref_path_topic').value)
            self.create_subscription(OccupancyGrid, cmap, self._cb_mppi_grid, 1)
            self.create_subscription(Path, pth, self._cb_mppi_path, 1)
            self.create_subscription(Path, ref, self._cb_mppi_ref, 1)

        self.get_logger().info(
            f'kasa HUD — 구독 전용. NAV={self.nav_mode}  경로 폴더 {self.data_dir}')

    def _cb_drive_cmd(self, m):
        text = str(m.data).strip()
        if not text or text.upper() in ROUTE_CMD_WORDS:
            if text.upper() == 'MAP_START':
                self._clear_trail()
                with self._nav_lock:
                    self.route_name = ''
                    self.route_wps = []
            return
        if text.lower().endswith('.csv'):
            self._load_route(text)

    def _cb_event(self, m):
        text = str(m.data)
        self.drive_event.set(text)
        found = ROUTE_CSV_RE.search(text)
        if found and ('경로 선택' in text or '주행 시작' in text):
            self._load_route(found.group(1))

    def _cb_map_cmd(self, m):
        on = bool(m.data)
        self.map_cmd.set(on)
        if on:
            self._clear_trail()

    def _cb_map_point(self, m):
        text = str(m.data)
        self.map_point.set(text)
        g = MAP_PT_RE.search(text)
        if g:
            self._append_trail(float(g.group(1)), float(g.group(2)))

    def _clear_trail(self):
        with self._nav_lock:
            self.map_trail = []

    def _append_trail(self, lat, lon):
        if not (math.isfinite(lat) and math.isfinite(lon)):
            return
        with self._nav_lock:
            if self.map_trail:
                plat, plon = self.map_trail[-1]
                de, dn = _latlon_to_xy(lat, lon, plat, plon)
                if math.hypot(de, dn) < TRAIL_MIN_M:
                    return
            self.map_trail.append((lat, lon))
            if len(self.map_trail) > TRAIL_MAX_N:
                self.map_trail = self.map_trail[-TRAIL_MAX_N:]

    def _route_dirs(self):
        """driving 과 같은 data_dir + 워크스페이스 src/white1/gps_data 보조."""
        dirs = []
        for d in (self.data_dir, wpaths.data_dir('')):
            if d and d not in dirs:
                dirs.append(d)
        here = os.path.dirname(os.path.realpath(__file__))
        p = here
        for _ in range(8):
            p = os.path.dirname(p)
            cand = os.path.join(p, 'src', 'white1', 'gps_data')
            if os.path.isdir(cand) and cand not in dirs:
                dirs.append(cand)
        return dirs

    def _load_route(self, name):
        raw = str(name).strip()
        base = os.path.basename(raw)
        if not base.lower().endswith('.csv'):
            return
        candidates = []
        if os.path.isabs(raw):
            candidates.append(raw)
        for d in self._route_dirs():
            candidates.append(os.path.join(d, base))
        path = next((p for p in candidates if os.path.isfile(p)), None)
        if path is None:
            self.get_logger().warning(f'HUD 경로 파일 없음: {base}')
            return
        wps = []
        try:
            with open(path, 'r', encoding='utf-8') as f:
                for row in csv.DictReader(f):
                    try:
                        la, lo = float(row['latitude']), float(row['longitude'])
                    except (KeyError, ValueError, TypeError):
                        continue
                    if math.isfinite(la) and math.isfinite(lo):
                        wps.append((la, lo))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'HUD 경로 읽기 실패: {exc}')
            return
        if len(wps) < 2:
            self.get_logger().warning(f'HUD 웨이포인트 부족({len(wps)}): {base}')
            return
        with self._nav_lock:
            self.route_name = base
            self.route_wps = wps
        self.get_logger().info(f'HUD 경로 로드 {base}  WP {len(wps)}')

    def _cb_cmd(self, m):
        now = time.monotonic()
        self.cmd_pulse.v = float(m.linear.x)
        self.cmd_pulse.t = now
        self.cmd_steer.v = float(m.angular.z)
        self.cmd_steer.t = now

    def _cb_fix(self, m):
        lat, lon = float(m.latitude), float(m.longitude)
        self.fix.set((lat, lon, int(m.status.status)))
        st = self.drive_state.get(5.0)
        if st in MAP_STATES:
            self._append_trail(lat, lon)

    def _cb_imu(self, m):
        q = m.orientation
        self.imu.set((float(q.x), float(q.y), float(q.z), float(q.w)))

    def _cb_image(self, m):
        if self._bridge is None:
            return
        try:
            bgr = self._bridge.imgmsg_to_cv2(m, desired_encoding='bgr8')
        except Exception:  # noqa: BLE001
            return
        with self._img_lock:
            self._img_bgr = bgr
            self._img_t = time.monotonic()

    def _cb_mppi_grid(self, msg):
        info = msg.info
        with self._mppi_lock:
            self._mppi_grid = {
                'w': int(info.width),
                'h': int(info.height),
                'res': float(info.resolution),
                'ox': float(info.origin.position.x),
                'oy': float(info.origin.position.y),
                'data': tuple(msg.data),
            }
            self._mppi_grid_t = time.monotonic()

    @staticmethod
    def _path_xy(msg):
        pts = []
        for ps in msg.poses:
            x = float(ps.pose.position.x)
            y = float(ps.pose.position.y)
            if math.isfinite(x) and math.isfinite(y):
                pts.append((x, y))
        return pts

    def _cb_mppi_path(self, msg):
        pts = self._path_xy(msg)
        with self._mppi_lock:
            self._mppi_path = pts
            self._mppi_path_t = time.monotonic()

    def _cb_mppi_ref(self, msg):
        pts = self._path_xy(msg)
        with self._mppi_lock:
            self._mppi_ref = pts
            self._mppi_ref_t = time.monotonic()
            self._img_t = time.monotonic()


class HudApp:
    def __init__(self, node: HudNode):
        self.n = node
        self._shown = {}
        self._photo = None
        self._nav_photo = None
        self._nav_photo_key = None
        self._fscreen = False

        self.root = tk.Tk()
        self.root.title('kasa HUD')
        self.root.configure(bg=BG)
        self.root.minsize(980, 620)
        self.root.geometry('1280x720')
        self.root.protocol('WM_DELETE_WINDOW', self.on_quit)
        self.root.bind('<F11>', self.toggle_full)
        self.root.bind('<Escape>', self.on_esc)

        fam = self._pick_font()
        self.fam = fam
        self.cv = tk.Canvas(self.root, bg=BG, highlightthickness=0)
        self.cv.pack(fill='both', expand=True)
        self.cv.bind('<Configure>', lambda _e: None)
        self._alive = True
        self.tick()

    def _pick_font(self):
        names = {n.lower(): n for n in tkfont.families()}
        for cand in ('Noto Sans CJK KR', 'Noto Sans KR', 'NanumGothic',
                     'NanumBarunGothic', 'UnDotum', 'DejaVu Sans'):
            if cand.lower() in names:
                return names[cand.lower()]
        return 'TkDefaultFont'

    def font(self, size, weight='normal'):
        h = max(self.cv.winfo_height(), 1)
        px = max(8, int(size * h / 720.0))
        return (self.fam, px, weight)

    def on_quit(self):
        self._alive = False
        try:
            self.root.destroy()
        except tk.TclError:
            pass

    def on_esc(self, _e=None):
        if self._fscreen:
            self.toggle_full()

    def toggle_full(self, _e=None):
        self._fscreen = not self._fscreen
        self.root.attributes('-fullscreen', self._fscreen)

    def run(self):
        self.root.mainloop()

    def smooth(self, key, target, a=0.28):
        if target is None:
            return self._shown.get(key)
        cur = self._shown.get(key)
        if cur is None:
            self._shown[key] = float(target)
            return float(target)
        v = cur + (float(target) - cur) * a
        self._shown[key] = v
        return v

    # ── 매 틱 ──────────────────────────────────────────────────────────────
    def tick(self):
        if not self._alive:
            return
        # ★★ [2026-09-04] 여기가 '런치 종료가 15초 걸리던' 자리다 ★★
        #   증상 : one_launch.py 를 Ctrl-C 로 내리면 hud 만 안 나가서
        #     `failed to terminate '5' seconds after receiving 'SIGINT'` →
        #     `failed to terminate '10.0' seconds after 'SIGTERM'` → SIGKILL.
        #     그 15초 동안 launch 는 "ctrl-c again, ignoring..." 만 찍는다.
        #   원인 : ★rclpy.init 이 SIGINT·SIGTERM 을 자기 처리기로 가로챈다★
        #     그 처리기는 컨텍스트를 닫을 뿐 프로세스를 끝내지 않는다. 다른 노드는
        #     rclpy.spin 이 그 순간 예외를 던져 main 이 빠져나가지만, 이 노드의 본
        #     스레드는 tkinter mainloop 에 있어서 ★아무 일도 일어나지 않는다★
        #     (spin 은 데몬 스레드에 있다). SIGTERM 이 기본동작이면 즉사할 텐데,
        #     rclpy 가 그것마저 가로채므로 SIGKILL 밖에 남지 않는다.
        #   고침 : ★틱마다 컨텍스트의 생존을 본다★ 신호가 오면 rclpy 가 컨텍스트를
        #     닫고, 우리는 늦어도 UI_MS(50ms) 안에 그것을 보고 창을 닫는다.
        #     신호 처리기를 우리가 다시 설치하지 않는 것이 요점이다 — rclpy 와
        #     처리기를 다투면 종료 경로가 둘로 갈라져 더 나빠진다.
        if not rclpy.ok():
            self.on_quit()
            return
        try:
            self.draw()
        except tk.TclError:
            return
        self.root.after(UI_MS, self.tick)

    def draw(self):
        c = self.cv
        w = max(c.winfo_width(), 2)
        h = max(c.winfo_height(), 2)
        n = self.n
        stale = n.stale_s
        c.delete('all')

        # 배경 비네트
        c.create_rectangle(0, 0, w, h, fill=BG, outline='')
        c.create_oval(-w * 0.1, h * 0.55, w * 1.1, h * 1.55,
                      fill='#05070a', outline='')

        cam_w = 0
        if n.show_camera:
            cam_w = self._draw_camera(c, 16, 58, int(w * 0.22), int(h * 0.28))

        cluster_l = cam_w + (36 if cam_w else 20)
        cluster_r = w - 20
        cluster_t = 56
        cluster_b = h - 132
        cx = cluster_l + (cluster_r - cluster_l) * 0.40
        cy = cluster_t + (cluster_b - cluster_t) * 0.50
        scale = min(cluster_r - cluster_l, cluster_b - cluster_t)

        self._draw_topbar(c, w, stale)
        self._draw_cluster(c, cx, cy, scale, stale)
        sx = cluster_l + (cluster_r - cluster_l) * 0.82
        self._draw_speed(c, sx, cluster_t + int((cluster_b - cluster_t) * 0.18),
                         stale)
        nav = int(min(240, max(150, (cluster_b - cluster_t) * 0.38)))
        nav_top = int(cy + 20)
        if nav_top + nav > cluster_b - 4:
            nav = max(140, cluster_b - 4 - nav_top)
        self._draw_nav(c, int(sx - nav / 2), nav_top, nav, nav, stale)
        self._draw_bottom(c, w, h, stale)
        self._draw_leds(c, w, h, stale)

    # ── 상단 필 ────────────────────────────────────────────────────────────
    def _draw_topbar(self, c, w, stale):
        n = self.n
        c.create_rectangle(0, 0, w, 48, fill='#0b0e14', outline='')
        c.create_text(18, 24, text='KASA', fill=GOLD, anchor='w',
                      font=self.font(16, 'bold'))
        c.create_text(78, 24, text='HUD', fill=DIM, anchor='w',
                      font=self.font(11))

        x = 140
        x = self._pill(c, x, 10, *self._mode_pill(stale))
        x = self._pill(c, x, 10, *self._estop_pill(stale))
        x = self._pill(c, x, 10, *self._aeb_pill(stale))
        x = self._pill(c, x, 10, *self._tl_pill(stale))
        x = self._pill(c, x, 10, *self._drive_by_pill(stale))
        x = self._pill(c, x, 10, *self._gps_pill(stale))
        x = self._pill(c, x, 10, *self._board_pill(stale))

        st = n.drive_state.get(stale)
        st_s = st if st else '—'
        col = DIM
        if st in ('DRIVE_RUN',):
            col = GREEN
        elif st in ('DRIVE_HEADING', 'MAP_HEADING', 'MAP_RUN'):
            col = CYAN
        elif st in ('DRIVE_DONE',):
            col = ORANGE
        c.create_text(w - 18, 24, text=st_s, fill=col, anchor='e',
                      font=self.font(13, 'bold'))

        wait = n.prompt_wait.get(2.5)
        if wait:
            c.create_text(w - 18, 42, text=str(wait), fill=ORANGE, anchor='e',
                          font=self.font(8))

    def _mode_pill(self, stale):
        m = self.n.mode.get(stale)
        if m is None:
            return 'MODE —', PANEL2, DIM
        if m:
            return 'AUTO', '#12331f', GREEN
        return 'MANUAL', '#33280c', GOLD

    def _estop_pill(self, stale):
        e = self.n.estop.get(stale)
        if e is None:
            return 'E-STOP —', PANEL2, DIM
        if e:
            flash = int(time.monotonic() * 4) % 2
            bg = RED if flash else '#5a1020'
            return 'E-STOP', bg, '#ffffff'
        return 'E-STOP', '#1a2420', DIM

    def _drive_by_pill(self, stale):
        """지금 /cmd_vel_raw 를 누가 내고 있나. ★GPS / 라이다 / 라이다 없음★

        · GPS      허락이 내려가 있다 = driving 이 낸다 (평소)
        · 라이다   허락이 올라가 있고 mppi 도 살아 있다 = mppi 가 낸다 (L 구간)
        · 라이다?  허락은 올라갔는데 mppi 신고가 없다 — ★이 상태로는 아무도 몰지
                   않는다★. driving 이 곧 정지시키므로 오래 보이지는 않지만,
                   보인다면 use_lidar:=false 로 띄웠거나 mppi 가 죽은 것이다.
        · 라이다 대기  mppi 는 살아 있고 회피 구간을 기다린다 (평소 GPS 와 함께 뜸)
        """
        p = self.n.lidar_permit.get(stale)
        a = self.n.lidar_active.get(stale)
        if p is None and a is None:
            return '조종 —', PANEL2, DIM          # 이양 기능이 없는 구성이다
        if p:
            if a is None:
                flash = int(time.monotonic() * 4) % 2
                return '라이다?', RED if flash else '#5a1020', '#ffffff'
            return '라이다', '#12331f', GREEN
        return 'GPS', PANEL2, CYAN

    def _aeb_pill(self, stale):
        a = self.n.aeb.get(stale)
        d = self.n.aeb_dist.get(stale)
        dist_s = ''
        if d is not None and math.isfinite(d):
            dist_s = ' inf' if d > 50 else f' {d:.1f}m'
        if a is None and d is None:
            return 'AEB —', PANEL2, DIM
        if a:
            flash = int(time.monotonic() * 4) % 2
            bg = RED if flash else '#5a1020'
            return f'AEB{dist_s}', bg, '#ffffff'
        return f'AEB{dist_s}', '#1a2420', GREEN_DIM

    def _tl_pill(self, stale):
        s = self.n.tl.get(stale)
        if not s:
            return 'TL —', PANEL2, DIM
        s = s.upper()
        if s == 'RED':
            return 'TL RED', '#4a1018', RED
        if s == 'RED_FAR':
            return 'TL FAR', '#3a2010', ORANGE
        if s == 'GREEN':
            return 'TL GREEN', '#12331f', GREEN
        return f'TL {s}', PANEL2, DIM

    def _gps_pill(self, stale):
        fused = self.n.fused.get(stale)
        qtxt = self.n.gps_q.get(3.0)
        label = None
        sigma = None
        if fused and len(fused) >= 4:  # [2] quality [3] sigma — 전체 GPS_FUSED_FIELDS
            q = int(fused[2]) if math.isfinite(fused[2]) else 0
            label = Q_LABEL.get(q, f'Q{q}')
            if math.isfinite(fused[3]):
                sigma = fused[3]
        elif qtxt:
            label = qtxt.split()[0]
        if not label:
            return 'GPS —', PANEL2, DIM
        extra = f' {sigma:.2f}m' if sigma is not None else ''
        if label in ('RTK_FIXED', 'FIXED'):
            return f'{label}{extra}', '#12331f', GREEN
        if label in ('RTK_FLOAT', 'FLOAT', 'DGPS'):
            return f'{label}{extra}', '#2a2410', ORANGE
        return f'{label}{extra}', PANEL2, DIM

    def _board_pill(self, stale):
        s = self.n.boards.get(stale)
        if not s:
            return 'A/B —', PANEL2, DIM
        # "A:1,B:1,ESTOP:0,MODE:1"
        a = 'A?'
        b = 'B?'
        for part in s.split(','):
            if part.startswith('A:'):
                a = 'A' if part.endswith('1') else 'A×'
            elif part.startswith('B:'):
                b = 'B' if part.endswith('1') else 'B×'
        ok = a == 'A' and b == 'B'
        return f'{a} {b}', '#12331f' if ok else '#3a1018', GREEN if ok else RED

    def _pill(self, c, x, y, text, bg, fg):
        pad_x = 10
        f = self.font(10, 'bold')
        tw = max(48, int(len(text) * 8 * max(c.winfo_height(), 1) / 720.0))
        h = 28
        self._round_rect(c, x, y, x + tw + pad_x * 2, y + h, 8,
                         fill=bg, outline='#242a33')
        c.create_text(x + pad_x + tw / 2, y + h / 2, text=text, fill=fg,
                      font=f)
        return x + tw + pad_x * 2 + 8

    # ── 중앙 클러스터 ──────────────────────────────────────────────────────
    def _draw_cluster(self, c, cx, cy, scale, stale):
        n = self.n
        unit = scale * 0.38
        self._round_rect(c, cx - unit * 1.70, cy - unit * 1.18,
                         cx + unit * 1.70, cy + unit * 1.18,
                         22, fill=PANEL, outline='#1c2430')

        thr = _frac_raw(n.throttle.get(stale), n.thr_lo, n.thr_hi)
        pwm_v = n.pwm.get(stale)
        pwm_f = None if pwm_v is None else _clamp(pwm_v / max(1, n.pwm_max), 0, 1)
        pulse_v = n.pulse.get(stale)
        pulse_f = None if pulse_v is None else _clamp(
            pulse_v / max(1, n.pulse_max), 0, 1)
        brk = n.brake_lv.get(stale)
        brk_f = None if brk is None else _clamp(brk / 2.0, 0, 1)

        gx = unit * 1.05
        gy = unit * 0.86
        self._gauge(c, cx - gx, cy - gy, unit * 0.24,
                    self.smooth('thr', 0.0 if thr is None else thr),
                    None if thr is None else f'{int(round(thr * 100))}%',
                    '스로틀', _gauge_color(thr or 0.0), thr is not None)
        self._gauge(c, cx + gx, cy - gy, unit * 0.24,
                    self.smooth('pwm', 0.0 if pwm_f is None else pwm_f),
                    None if pwm_v is None else f'{int(round((pwm_f or 0) * 100))}%',
                    'PWM', _gauge_color(pwm_f or 0.0), pwm_f is not None)
        self._gauge(c, cx - gx, cy + gy, unit * 0.24,
                    self.smooth('brk', 0.0 if brk_f is None else brk_f),
                    None if brk is None else f'{int(brk)}',
                    '브레이크', RED if (brk or 0) >= 2 else (
                        ORANGE if (brk or 0) == 1 else GREEN_DIM),
                    brk is not None)
        self._gauge(c, cx + gx, cy + gy, unit * 0.24,
                    self.smooth('pls', 0.0 if pulse_f is None else pulse_f),
                    None if pulse_v is None else f'{int(pulse_v)}',
                    '펄스', _gauge_color(pulse_f or 0.0), pulse_f is not None)

        steer = n.steer.get(stale)
        steer_s = self.smooth('steer', 0.0 if steer is None else float(steer))
        body = self._body_color(stale)
        car_s = unit * 0.42
        lidar_on = self._draw_lidar_range(c, cx, cy, car_s, stale)
        self._car(c, cx, cy, car_s, steer_s or 0.0, body, stale)

        # 라이다 범위가 그 자리를 쓴다. 없을 때만 구동 경로를 적는다.
        if not lidar_on:
            if pwm_v is not None and pwm_v > 0:
                path, path_c = '직접 PWM', CYAN
            elif pulse_v is not None and pulse_v > 0:
                path, path_c = '펄스 PID', ORANGE
            else:
                path, path_c = '정지', DIM
            c.create_text(cx, cy - unit * 1.05, text=path, fill=path_c,
                          font=self.font(9, 'bold'))

    def _body_color(self, stale):
        n = self.n
        if n.estop.get(stale) or n.aeb.get(stale):
            return BODY_HOT if int(time.monotonic() * 4) % 2 else '#8a1a2c'
        m = n.mode.get(stale)
        st = n.drive_state.get(stale)
        if st == 'DRIVE_RUN':
            return BODY_OK
        if m is False:
            return BODY_MAN
        return BODY_OK

    def _gauge(self, c, x, y, r, frac, text, label, color, live):
        frac = 0.0 if frac is None else _clamp(frac, 0.0, 1.0)
        track = '#2a3140' if live else '#1a1e26'
        ring = color if live else DIM
        # 바닥이 빈 270° 링
        wline = max(6, int(r * 0.18))
        bbox = (x - r, y - r, x + r, y + r)
        c.create_arc(*bbox, start=230, extent=-280, style='arc',
                     outline=track, width=wline)
        if live and frac > 0.004:
            c.create_arc(*bbox, start=230, extent=-280 * frac, style='arc',
                         outline=ring, width=wline)
        inner = r - wline * 0.9
        c.create_oval(x - inner, y - inner, x + inner, y + inner,
                      fill='#0c1016', outline='#1c2430')
        shown = text if (live and text is not None) else '—'
        c.create_text(x, y - 2, text=shown, fill=FG if live else DIM,
                      font=self.font(13 if r > 40 else 11, 'bold'))
        c.create_text(x, y + r + 14, text=label, fill=DIM,
                      font=self.font(9))

    def _draw_lidar_range(self, c, cx, cy, s, stale):
        """차 앞 AEB ROI. /obstacle_distance 가 한 번도 없으면 그리지 않는다(white1)."""
        n = self.n
        dist = n.aeb_dist.get(stale)
        if not (n.aeb_dist.fresh(stale) or n.aeb_sig.fresh(stale)
                or n.aeb.fresh(stale)):
            return False

        rmin = n.lidar_rmin
        rmax = n.lidar_rmax
        front = n.vehicle_front_m
        half_m = n.corridor_half_m
        # 차체 로컬: 범퍼 y=1.20, 차폭 ≈ 바퀴 간격 1.20. 1 m → 1.20/1.6 로컬
        m_to_loc = 1.20 / 1.60
        nose = 1.06             # 앞날개 위에 붙여 차와 범위가 떨어지지 않게
        fan_len = 1.38          # 패널 안쪽까지. 15 m 를 이 길이에 압축
        half0 = 0.42            # 범퍼에서 폭
        half1 = max(half0, half_m * m_to_loc * 0.85)

        def y_of(d_lidar):
            d_b = max(0.0, float(d_lidar) - front)
            t = _clamp(d_b / max(0.1, rmax - front), 0.0, 1.0)
            return nose + t * fan_len

        def half_at(y):
            t = _clamp((y - nose) / fan_len, 0.0, 1.0)
            return _lerp(half0, half1, t)

        def xy(px, py):
            return cx + px * s, cy - py * s

        def trap(y0, y1):
            h0, h1 = half_at(y0), half_at(y1)
            return [xy(-h0, y0), xy(h0, y0), xy(h1, y1), xy(-h1, y1)]

        def poly(pts, **kw):
            flat = []
            for x, y in pts:
                flat.extend((x, y))
            c.create_polygon(*flat, **kw)

        sig = n.aeb_sig.get(stale)
        aeb = n.aeb.get(stale)
        hit = False
        d_show = None
        if dist is not None and math.isfinite(dist) and dist < rmax:
            hit = True
            d_show = dist
        if sig or aeb:
            hit = True

        if aeb:
            fill, edge, label_c = '#4a1018', RED, RED
        elif hit and d_show is not None and d_show < rmin + 1.0:
            fill, edge, label_c = '#3a1808', RED, RED
        elif hit:
            fill, edge, label_c = '#2a2208', ORANGE, ORANGE
        else:
            fill, edge, label_c = '#0d2818', GREEN_DIM, GREEN

        y_end = y_of(rmax)
        if hit and d_show is not None:
            y_hit = y_of(d_show)
            poly(trap(nose, y_hit), fill=fill, outline='')
            h = half_at(y_hit)
            c.create_line(*xy(-h, y_hit), *xy(h, y_hit), fill=edge, width=3)
        else:
            poly(trap(nose, y_end), fill=fill, outline='')

        # ROI 외곽 + 거리 링
        h0, h1 = half_at(nose), half_at(y_end)
        c.create_line(*xy(-h0, nose), *xy(-h1, y_end), fill=edge, width=1)
        c.create_line(*xy(h0, nose), *xy(h1, y_end), fill=edge, width=1)
        c.create_line(*xy(-h1, y_end), *xy(h1, y_end), fill=edge, width=1)
        for mark in (5.0, 10.0, rmax):
            if mark > rmax + 0.05:
                continue
            ym = y_of(mark)
            hm = half_at(ym)
            c.create_line(*xy(-hm, ym), *xy(hm, ym), fill='#1c2430', width=1)
            c.create_text(*xy(hm + 0.18, ym), text=f'{mark:.0f}',
                          fill=DIM, font=self.font(7), anchor='w')

        if aeb:
            txt = 'AEB'
        elif hit and d_show is not None:
            txt = f'{d_show:.1f} m'
        elif dist is not None and math.isinf(float(dist)):
            txt = 'CLEAR'
        else:
            txt = 'LIDAR'
        c.create_text(*xy(0.0, nose + fan_len * 0.52), text=txt, fill=label_c,
                      font=self.font(9, 'bold'))
        return True

    def _car(self, c, cx, cy, s, steer_deg, body, stale):
        """상면도. s = 차체 반높이(px). 앞이 위. 조향 −좌/+우 → 앞바퀴 회전."""
        # 화면: 왼쪽 = −x. 토픽 − = 좌회전 이므로 부호를 뒤집어 앞바퀴가 왼쪽으로 꺾이게 한다.
        ang = -math.radians(_clamp(steer_deg, -self.n.steer_max, self.n.steer_max))
        outline = '#0a0d10'
        dark = _mix(body, '#05070a', 0.45)

        def xy(px, py):
            return cx + px * s, cy - py * s

        def rot_poly(pts, ox, oy, a):
            ca, sa = math.cos(a), math.sin(a)
            out = []
            for px, py in pts:
                out.extend((ox + (px * ca - py * sa) * s,
                            oy - (px * sa + py * ca) * s))
            return out

        def ellipse(rx, ry, n=14):
            return [(rx * math.cos(2 * math.pi * i / n),
                     ry * math.sin(2 * math.pi * i / n)) for i in range(n)]

        def poly(pts, **kw):
            flat = []
            for x, y in pts:
                flat.extend((x, y))
            c.create_polygon(*flat, **kw)

        c.create_oval(cx - 0.70 * s, cy - 0.10 * s,
                      cx + 0.70 * s, cy + 1.10 * s,
                      fill='#05070a', outline='')

        # 앞날개
        poly([xy(-0.55, 0.92), xy(0.55, 0.92), xy(0.50, 1.02), xy(-0.50, 1.02)],
             fill='#1a1e24', outline=outline, smooth=True)
        # 노즈
        poly([xy(0.00, 1.20), xy(0.11, 0.86), xy(0.08, 0.55),
              xy(-0.08, 0.55), xy(-0.11, 0.86)],
             fill=body, outline=outline, smooth=True, width=2)
        # 본체 + 사이드포드
        poly([xy(-0.16, 0.58), xy(-0.38, 0.22), xy(-0.46, -0.08),
              xy(-0.40, -0.58), xy(-0.22, -0.88),
              xy(0.22, -0.88), xy(0.40, -0.58), xy(0.46, -0.08),
              xy(0.38, 0.22), xy(0.16, 0.58)],
             fill=body, outline=outline, smooth=True, width=2)
        poly([xy(-0.12, 0.20), xy(-0.34, 0.02), xy(-0.36, -0.48),
              xy(-0.14, -0.62), xy(0.14, -0.62), xy(0.36, -0.48),
              xy(0.34, 0.02), xy(0.12, 0.20)],
             fill=dark, outline='', smooth=True)
        # 리어윙
        poly([xy(-0.46, -0.90), xy(0.46, -0.90), xy(0.46, -1.02), xy(-0.46, -1.02)],
             fill='#1a1e24', outline=outline)
        # 코크핏 + 헤일로
        c.create_oval(*xy(-0.17, 0.22), *xy(0.17, -0.18),
                      fill='#0c1014', outline='#2a3140')
        c.create_arc(*xy(-0.22, 0.32), *xy(0.22, -0.10),
                     start=8, extent=164, style='arc',
                     outline='#dfe6ee', width=max(2, int(s * 0.04)))
        poly([xy(-0.07, 0.48), xy(0.07, 0.48), xy(0.0, 0.66)],
             fill='#0c1014', outline='')

        moving = (self.n.speed.get(stale) or 0.0) > 0.4 or (
            self.n.encoder.get(stale) or 0) > 0
        rubber = '#14161c'
        rim = GOLD if moving else '#4a5160'
        wheel = ellipse(0.13, 0.26)
        for px, py, a in ((-0.58, 0.78, ang), (0.58, 0.78, ang),
                          (-0.60, -0.70, 0.0), (0.60, -0.70, 0.0)):
            ox, oy = xy(px, py)
            c.create_polygon(*rot_poly(wheel, ox, oy, a),
                             fill=rubber, outline='#3a3f48', width=2)
            c.create_oval(ox - 0.07 * s, oy - 0.07 * s,
                          ox + 0.07 * s, oy + 0.07 * s,
                          fill=rim, outline='')

    # ── 속도 ──────────────────────────────────────────────────────────────
    def _draw_speed(self, c, x, y, stale):
        n = self.n
        enc = n.encoder.get(stale)
        imu = n.speed.get(stale)
        # ★엔코더 우선★ lidar HUD 와 같다. /speed(IMU 적분)는 절대속도를 못 믿어서
        #   큰 숫자에는 쓰지 않는다. 엔코더가 끊겼을 때만 IMU 로 내려간다.
        if enc is not None:
            sp = enc * KMH_PER_PULSE
            src = 'ENC'
        elif imu is not None:
            sp = imu
            src = 'IMU'
        else:
            sp = None
            src = ''
        shown = self.smooth('spd', 0.0 if sp is None else sp)
        live = sp is not None
        txt = f'{shown:.1f}' if live else '—'
        c.create_text(x, y - 18, text=txt, fill=FG if live else DIM,
                      font=self.font(42, 'bold'))
        c.create_text(x, y + 22, text='km/h', fill=DIM, font=self.font(12))
        if src:
            c.create_text(x, y + 42, text=src, fill=DIM, font=self.font(8))

        pwm = n.pwm.get(stale)
        pulse = n.pulse.get(stale)
        enc_s = '—' if enc is None else str(int(enc))
        pwm_s = '—' if pwm is None else str(int(pwm))
        pls_s = '—' if pulse is None else str(int(pulse))
        c.create_text(x, y + 78,
                      text=f'PWM  {pwm_s}    펄스  {pls_s}    ENC  {enc_s}',
                      fill=FG if live else DIM, font=self.font(11))

        cmd_p = n.cmd_pulse.get(stale)
        cmd_s = n.cmd_steer.get(stale)
        if cmd_p is not None or cmd_s is not None:
            ps = '—' if cmd_p is None else f'{cmd_p:.1f}'
            ss = '—' if cmd_s is None else f'{cmd_s:+.1f}'
            c.create_text(x, y + 102, text=f'cmd  pulse {ps}   steer {ss}',
                          fill=DIM, font=self.font(9))

    def _live_pose(self, stale):
        """실시간 위경도·헤딩. 없으면 (None, None, None)."""
        n = self.n
        lat = lon = None
        fused = n.fused.get(stale)
        if fused and len(fused) >= 2 and math.isfinite(fused[0]) and math.isfinite(fused[1]):
            lat, lon = float(fused[0]), float(fused[1])
        else:
            fx = n.fix.get(3.0)
            if fx and math.isfinite(fx[0]):
                lat, lon = float(fx[0]), float(fx[1])
        heading = None
        ego = n.ego.get(stale)
        if ego and len(ego) >= 3 and math.isfinite(ego[2]):
            heading = float(ego[2])
        if heading is None and fused and len(fused) >= GPS_FUSED_FIELDS and math.isfinite(fused[9]):
            heading = float(fused[9])
        if heading is None:
            imu = n.imu.get(stale)
            if imu:
                heading = _yaw_from_quat(*imu)
        return lat, lon, heading

    def _nav_points(self):
        """표시할 경로 점. 선택된 CSV가 있으면 그걸, 없으면 매핑 트레일."""
        n = self.n
        with n._nav_lock:
            wps = list(n.route_wps)
            trail = list(n.map_trail)
            name = n.route_name
        if len(wps) >= 2:
            return wps, name, 'route'
        if len(trail) >= 2:
            return trail, 'mapping', 'trail'
        return [], '', 'none'

    def _mppi_snapshot(self):
        n = self.n
        with n._mppi_lock:
            return (n._mppi_grid, n._mppi_grid_t,
                    list(n._mppi_path), n._mppi_path_t,
                    list(n._mppi_ref), n._mppi_ref_t)

    def _mppi_cost_photo(self, iw, ih, x0, x1, y0, y1, grid, key):
        """코스트맵 → 작은 PPM. 같은 key 면 캐시. 실패하면 None."""
        if grid is None or iw < 4 or ih < 4:
            self._nav_photo = None
            self._nav_photo_key = None
            return None
        if self._nav_photo is not None and self._nav_photo_key == key:
            return self._nav_photo
        gw = grid['w']
        gh = grid['h']
        res = grid['res']
        ox = grid['ox']
        oy = grid['oy']
        data = grid['data']
        if gw < 1 or gh < 1 or res <= 1e-6 or len(data) < gw * gh:
            return None
        dx = x1 - x0
        dy = y1 - y0
        raw = bytearray([12, 16, 22] * (iw * ih))
        for py in range(ih):
            ex = x1 - (py + 0.5) / ih * dx
            ix = int((ex - ox) / res)
            if ix < 0 or ix >= gw:
                continue
            row = py * iw * 3
            for px in range(iw):
                ey = y1 - (px + 0.5) / iw * dy
                iy = int((ey - oy) / res)
                if iy < 0 or iy >= gh:
                    continue
                v = data[iy * gw + ix]
                o = row + px * 3
                if v < 0:
                    continue
                if v == 0:
                    # 빈 공간 — 어두운 청회색. 장애물(빨강)과 반대로 읽히게.
                    raw[o] = 28
                    raw[o + 1] = 36
                    raw[o + 2] = 48
                    continue
                if v >= 100:
                    # 치사(벽·차체) 
                    raw[o] = 255
                    raw[o + 1] = 60
                    raw[o + 2] = 80
                elif v >= 50:
                    t = (v - 50) / 50.0
                    raw[o] = int(180 + 50 * t)
                    raw[o + 1] = int(110 - 30 * t)
                    raw[o + 2] = int(40)
                else:
                    # 여유(인플레이션) — 희미한 금색. 방을 온통 빨갛게 칠하지 않는다.
                    t = v / 50.0
                    raw[o] = int(70 + 80 * t)
                    raw[o + 1] = int(62 + 40 * t)
                    raw[o + 2] = int(28)
        try:
            header = f'P6 {iw} {ih} 255 '.encode('ascii')
            img = tk.PhotoImage(data=header + bytes(raw))
        except tk.TclError:
            try:
                header = f'P6\n{iw} {ih}\n255\n'.encode('ascii')
                img = tk.PhotoImage(data=header + bytes(raw))
            except tk.TclError:
                self._nav_photo = None
                self._nav_photo_key = None
                return None
        self._nav_photo = img
        self._nav_photo_key = key
        return img

    def _draw_mppi_cells(self, c, grid, x0, x1, y0, y1, scr):
        """PhotoImage 실패 시 점유 셀만 사각형으로. 성기게 샘플한다."""
        gw, gh, res = grid['w'], grid['h'], grid['res']
        ox, oy, data = grid['ox'], grid['oy'], grid['data']
        step = max(1, int(0.35 / max(res, 0.05)))
        r = max(1.5, 0.18 / (x1 - x0) * 80)
        for iy in range(0, gh, step):
            ey = oy + (iy + 0.5) * res
            if ey < y0 or ey > y1:
                continue
            row = iy * gw
            for ix in range(0, gw, step):
                v = data[row + ix]
                if v < 40:
                    continue
                ex = ox + (ix + 0.5) * res
                if ex < x0 or ex > x1:
                    continue
                px, py = scr(ex, ey)
                col = RED if v >= 100 else ORANGE
                c.create_rectangle(px - r, py - r, px + r, py + r,
                                   fill=col, outline='')

    def _draw_nav_mppi(self, c, x, y, w, h, stale):
        """차량 기준 2D 탑뷰. 앞=위, 왼쪽=+y. 장애물=코스트맵, 경로=롤아웃."""
        n = self.n
        self._round_rect(c, x, y, x + w, y + h, 12,
                         fill='#0c1016', outline='#1c2430')
        c.create_text(x + 12, y + 12, text='NAV', fill=GOLD, anchor='nw',
                      font=self.font(9, 'bold'))
        c.create_text(x + w - 10, y + 12, text='MPPI', fill=CYAN, anchor='ne',
                      font=self.font(8))

        grid, gt, path, pt, ref, rt = self._mppi_snapshot()
        now = time.monotonic()
        grid_live = grid is not None and (now - gt) < max(stale, 1.0)

        # 전방 위주 창. 차는 아래쪽, 앞이 화면 위.
        x0, x1 = -2.0, 10.0
        y0, y1 = -6.0, 6.0
        pad = 28
        avail = max(20, min(w, h) - pad - 10)
        if y + 26 + avail > y + h - 8:
            avail = max(20, y + h - 8 - (y + 26))
        # 코스트맵 이미지와 경로·차체가 같은 픽셀 격자를 쓰게 맞춘다.
        pix = max(64, min(int(avail), 140))
        inner = float(pix)
        ix0 = x + (w - pix) / 2
        iy0 = y + 26
        ix1 = ix0 + pix
        iy1 = iy0 + pix

        def scr(ex, ey):
            px = ix0 + (y1 - ey) / (y1 - y0) * inner
            py = iy0 + (x1 - ex) / (x1 - x0) * inner
            return px, py

        if grid_live:
            key = (gt, pix, pix, round(x0, 2), round(x1, 2),
                   round(y0, 2), round(y1, 2))
            photo = self._mppi_cost_photo(pix, pix, x0, x1, y0, y1, grid, key)
            if photo is not None:
                c.create_image(ix0, iy0, image=photo, anchor='nw')
            else:
                self._draw_mppi_cells(c, grid, x0, x1, y0, y1, scr)
        else:
            c.create_text((ix0 + ix1) / 2, (iy0 + iy1) / 2,
                          text='WAITING', fill=DIM,
                          font=self.font(16, 'bold'))
            c.create_text((ix0 + ix1) / 2, (iy0 + iy1) / 2 + 20,
                          text='코스트맵 없음', fill='#3a4250',
                          font=self.font(8))

        # 거리 링 (전방)
        for dist in (4.0, 8.0):
            p0 = scr(dist, y0)
            p1 = scr(dist, y1)
            c.create_line(p0[0], p0[1], p1[0], p1[1], fill='#1c2430', width=1)
            lx, ly = scr(dist, 0.0)
            c.create_text(lx + 8, ly, text=f'{dist:.0f}m', fill='#3a4250',
                          font=self.font(7), anchor='w')
        # 중심선 (IMU 기준 y=0)
        a = scr(x0, 0.0)
        b = scr(x1, 0.0)
        c.create_line(a[0], a[1], b[0], b[1], fill='#2a3140', width=1,
                      dash=(3, 3))

        def polyline(seq, color, width):
            if len(seq) < 2:
                return
            if len(seq) > 80:
                step = max(1, len(seq) // 80)
                seq = seq[::step] + [seq[-1]]
            flat = []
            for ex, ey in seq:
                flat.extend(scr(ex, ey))
            c.create_line(*flat, fill=color, width=width, smooth=True)

        if ref and (now - rt) < max(stale, 1.0):
            polyline(ref, YELLOW, 2)
        if path and (now - pt) < max(stale, 1.0):
            polyline(path, CYAN, 3)

        # 차체 (ego 원점 = 뒷차축, 앞= +x)
        front = max(0.4, n.vehicle_front_m)
        rear = -max(0.1, n.rear_overhang_m)
        hw = max(0.25, n.track_width_m * 0.5)
        body = [
            (rear, hw), (front * 0.72, hw), (front, 0.0),
            (front * 0.72, -hw), (rear, -hw),
        ]
        flat = []
        for ex, ey in body:
            flat.extend(scr(ex, ey))
        c.create_polygon(*flat, fill=BODY_OK, outline='#fff2b0', width=1)
        # 앞바퀴 조향 힌트
        steer = n.cmd_steer.get(stale)
        if steer is None:
            steer = n.steer.get(stale)
        if steer is not None:
            ang = -math.radians(_clamp(float(steer), -n.steer_max, n.steer_max))
            ax = n.wheelbase_m
            for side in (hw * 0.85, -hw * 0.85):
                ca, sa = math.cos(ang), math.sin(ang)
                p1 = (ax + 0.18 * ca, side + 0.18 * sa)
                p2 = (ax - 0.18 * ca, side - 0.18 * sa)
                q1, r1 = scr(*p1)
                q2, r2 = scr(*p2)
                c.create_line(q1, r1, q2, r2, fill='#14161c', width=3)

        c.create_text(x + 12, y + h - 12, text='앞 ↑', fill=DIM,
                      font=self.font(8), anchor='sw')
        # 범례: 스크린샷에서 빨강을 공간으로 읽던 오해를 막는다.
        c.create_rectangle(x + w / 2 - 52, y + h - 16,
                           x + w / 2 - 44, y + h - 8,
                           fill=RED, outline='')
        c.create_text(x + w / 2 - 42, y + h - 12, text='장애물',
                      fill=DIM, font=self.font(7), anchor='w')
        c.create_rectangle(x + w / 2 + 8, y + h - 16,
                           x + w / 2 + 16, y + h - 8,
                           fill='#1c2430', outline='#3a4250')
        c.create_text(x + w / 2 + 18, y + h - 12, text='빈공간',
                      fill=DIM, font=self.font(7), anchor='w')

    # ── NAV 미니맵 (북쪽 위). 경로 없으면 NONE ────────────────────────────
    def _draw_nav(self, c, x, y, w, h, stale):
        n = self.n
        if n.nav_mode == 'mppi':
            self._draw_nav_mppi(c, x, y, w, h, stale)
            return
        self._round_rect(c, x, y, x + w, y + h, 12,
                         fill='#0c1016', outline='#1c2430')
        c.create_text(x + 12, y + 12, text='NAV', fill=GOLD, anchor='nw',
                      font=self.font(9, 'bold'))

        pts, name, kind = self._nav_points()
        live_lat, live_lon, heading = self._live_pose(stale)

        if kind == 'none':
            c.create_text(x + w / 2, y + h / 2, text='NONE',
                          fill=DIM, font=self.font(22, 'bold'))
            c.create_text(x + w / 2, y + h / 2 + 26, text='경로 없음',
                          fill='#3a4250', font=self.font(9))
            return

        cap = name if name and name != 'mapping' else '매핑 중'
        if len(cap) > 22:
            cap = cap[:19] + '…'
        c.create_text(x + w - 10, y + 12, text=cap, fill=DIM, anchor='ne',
                      font=self.font(8))

        # 원점: 경로 첫 점. 실시간 GPS 도 같은 평면에 올린다.
        lat0, lon0 = pts[0]
        xy = [_latlon_to_xy(la, lo, lat0, lon0) for la, lo in pts]
        if len(xy) > NAV_DRAW_MAX:
            step = max(1, len(xy) // NAV_DRAW_MAX)
            xy_draw = xy[::step]
            if xy_draw[-1] != xy[-1]:
                xy_draw.append(xy[-1])
        else:
            xy_draw = xy

        live_xy = None
        if live_lat is not None:
            live_xy = _latlon_to_xy(live_lat, live_lon, lat0, lon0)

        xs = [p[0] for p in xy_draw]
        ys = [p[1] for p in xy_draw]
        if live_xy is not None:
            xs.append(live_xy[0])
            ys.append(live_xy[1])
        minx, maxx = min(xs), max(xs)
        miny, maxy = min(ys), max(ys)
        span = max(maxx - minx, maxy - miny, 8.0)
        pad = span * 0.16
        span += pad * 2
        midx = (minx + maxx) / 2.0
        midy = (miny + maxy) / 2.0
        inner = min(w, h) - 36
        sc = inner / span
        cx = x + w / 2
        cy = y + h / 2 + 8

        def scr(east, north):
            return cx + (east - midx) * sc, cy - (north - midy) * sc

        # 지나온 구간 / 남은 구간
        wp_idx = 0
        ego = n.ego.get(stale)
        if ego and len(ego) >= 6:
            wp_idx = max(0, min(int(ego[4]), len(xy_draw) - 1))
        if kind == 'trail':
            wp_idx = max(0, len(xy_draw) - 1)

        def polyline(seq, color, width):
            if len(seq) < 2:
                return
            flat = []
            for east, north in seq:
                flat.extend(scr(east, north))
            c.create_line(*flat, fill=color, width=width, smooth=True)

        if wp_idx >= 1:
            polyline(xy_draw[:wp_idx + 1], GREEN_DIM, 2)
        polyline(xy_draw[wp_idx:], CYAN, 2)

        sx, sy = scr(*xy_draw[0])
        gx, gy = scr(*xy_draw[-1])
        c.create_oval(sx - 4, sy - 4, sx + 4, sy + 4, fill=GREEN, outline='')
        c.create_oval(gx - 5, gy - 5, gx + 5, gy + 5, fill=ORANGE, outline='')

        # 실시간 위치
        if live_xy is not None:
            px, py = scr(*live_xy)
            ang = 0.0 if heading is None else math.radians(heading)
            tip = 11
            # heading 0=동, 90=북. 화면 x=동, y 위=북.
            tx = px + tip * math.cos(ang)
            ty = py - tip * math.sin(ang)
            bx = px - 0.6 * tip * math.cos(ang)
            by = py + 0.6 * tip * math.sin(ang)
            left = math.radians(heading + 90 if heading is not None else 90)
            lx = bx + 6 * math.cos(left)
            ly = by - 6 * math.sin(left)
            rx = bx - 6 * math.cos(left)
            ry = by + 6 * math.sin(left)
            c.create_polygon(tx, ty, lx, ly, rx, ry,
                             fill=GOLD, outline='#fff2b0')
            # 라이브가 경로에서 얼마나 떨어졌나 (대략)
            cte = None
            diag = n.diag.get(stale)
            if diag and len(diag) >= 1 and math.isfinite(diag[0]):
                cte = float(diag[0])
            if cte is not None:
                c.create_text(x + w / 2, y + h - 14,
                              text=f'CTE {cte:+.2f} m',
                              fill=ORANGE if abs(cte) > 0.4 else DIM,
                              font=self.font(8))
        else:
            c.create_text(x + w / 2, y + h - 14, text='GPS —',
                          fill=DIM, font=self.font(8))

        # 북쪽
        c.create_text(x + w - 12, y + 28, text='N', fill=CYAN, anchor='ne',
                      font=self.font(8, 'bold'))

    # ── 하단 ──────────────────────────────────────────────────────────────
    def _draw_bottom(self, c, w, h, stale):
        n = self.n
        y0 = h - 108
        c.create_rectangle(0, y0, w, h, fill='#0b0e14', outline='')

        steer = n.steer.get(stale)
        self._steer_bar(c, 20, y0 + 22, w * 0.30, 28, steer, stale)

        # 헤딩 / CTE / WP / GPS
        ego = n.ego.get(stale)
        fused = n.fused.get(stale)
        diag = n.diag.get(stale)
        heading = None
        wp_s = '—'
        if ego and len(ego) >= 6:
            heading = float(ego[2])
            wp_s = f'{int(ego[4])}/{int(ego[5])}'
        if (heading is None and fused and len(fused) >= GPS_FUSED_FIELDS
                and math.isfinite(fused[9])):
            heading = float(fused[9])  # [9] course_deg

        cte = None
        herr = None
        goal = None
        if diag and len(diag) >= 5:
            if math.isfinite(diag[0]):
                cte = float(diag[0])
            if math.isfinite(diag[1]):
                herr = float(diag[1])
            if math.isfinite(diag[4]):
                goal = float(diag[4])

        lat = lon = None
        if fused and len(fused) >= 2:
            lat, lon = fused[0], fused[1]
        else:
            fx = n.fix.get(3.0)
            if fx:
                lat, lon = fx[0], fx[1]

        items = [
            (w * 0.42, 'HEAD',
             None if heading is None else f'{heading:.1f}°'),
            (w * 0.54, 'CTE',
             None if cte is None else f'{cte:+.2f}m'),
            (w * 0.66, 'WP', wp_s if ego else None),
            (w * 0.78, 'GOAL',
             None if goal is None else f'{goal:.1f}m'),
        ]
        for x, cap, val in items:
            c.create_text(x, y0 + 16, text=cap, fill=DIM, font=self.font(8),
                          anchor='w')
            c.create_text(x, y0 + 36,
                          text='—' if val is None else val,
                          fill=FG if val and val != '—' else DIM,
                          font=self.font(13, 'bold'), anchor='w')

        if herr is not None:
            c.create_text(w * 0.42, y0 + 56, text=f'herr {herr:+.1f}°',
                          fill=DIM, font=self.font(8), anchor='w')

        gps_line = 'GPS —'
        if lat is not None and lon is not None and math.isfinite(lat):
            gps_line = f'{lat:.7f}  {lon:.7f}'
        c.create_text(w * 0.66, y0 + 56, text=gps_line, fill=DIM,
                      font=self.font(8), anchor='w')

        ev = n.drive_event.get(4.0)
        if ev:
            c.create_text(20, y0 + 62, text=str(ev)[:80], fill=ORANGE,
                          font=self.font(9), anchor='w')

        # 스로틀 raw 숫자 — 페달 매핑 디버그
        raw = n.throttle.get(stale)
        pot = n.brake_pot.get(stale)
        raw_s = '—' if raw is None else str(int(raw))
        pot_s = '—' if pot is None else str(int(pot))
        c.create_text(w - 16, y0 + 16,
                      text=f'throttle raw {raw_s}   brake pot {pot_s}',
                      fill=DIM, font=self.font(8), anchor='e')

        ctrl = n.ctrl.get(stale)
        ctrl_s = '—' if ctrl is None else ('ROS ON' if ctrl else 'ROS OFF')
        c.create_text(w - 16, y0 + 36, text=ctrl_s,
                      fill=GREEN if ctrl else DIM,
                      font=self.font(9, 'bold'), anchor='e')

    def _steer_bar(self, c, x, y, w, h, steer, stale):
        self._round_rect(c, x, y, x + w, y + h, 8,
                         fill=PANEL2, outline='#1c2430')
        mid = x + w / 2
        c.create_line(mid, y + 4, mid, y + h - 4, fill='#2a3140')
        c.create_text(x + 8, y + h / 2, text='L', fill=DIM, font=self.font(8),
                      anchor='w')
        c.create_text(x + w - 8, y + h / 2, text='R', fill=DIM, font=self.font(8),
                      anchor='e')
        val = '—' if steer is None else f'{steer:+d}°'
        if steer is not None:
            frac = _clamp(float(steer) / max(1.0, self.n.steer_max), -1.0, 1.0)
            px = mid + frac * (w * 0.40)
            c.create_oval(px - 7, y + h / 2 - 7, px + 7, y + h / 2 + 7,
                          fill=GOLD, outline='#fff2b0')
        c.create_text(mid, y - 11, text=f'STEER {val}', fill=DIM,
                      font=self.font(8))

    # ── 토픽 신선도 LED ────────────────────────────────────────────────────
    def _draw_leds(self, c, w, h, stale):
        n = self.n
        leds = [
            ('THR', n.throttle),
            ('PWM', n.pwm),
            ('PLS', n.pulse),
            ('BRK', n.brake_lv),
            ('ENC', n.encoder),
            ('STR', n.steer),
            ('SPD', n.speed),
            ('IMU', n.imu),
            ('GPS', n.fused),
            ('FIX', n.fix),
            ('MOD', n.mode),
            ('EST', n.estop),
            ('AEB', n.aeb),
            ('LDR', n.aeb_dist),
            ('TL', n.tl),
            ('CAM', None),
            ('NAV', None),
        ]
        x = 16
        y = h - 18
        cam_live = (n.show_camera and n._img_t > 0 and
                    (time.monotonic() - n._img_t) < stale)
        with n._nav_lock:
            nav_live = len(n.route_wps) >= 2 or len(n.map_trail) >= 2
        mppi_live = False
        mppi_ever = False
        if n.nav_mode == 'mppi':
            mppi_ever = n._mppi_grid_t > 0
            mppi_live = mppi_ever and (time.monotonic() - n._mppi_grid_t) < stale
        for name, samp in leds:
            if name == 'CAM':
                live = cam_live
                ever = n._img_t > 0
            elif name == 'NAV':
                if n.nav_mode == 'mppi':
                    live = mppi_live
                    ever = mppi_ever
                else:
                    live = nav_live
                    ever = nav_live
            else:
                live = samp.fresh(stale)
                ever = samp.t > 0
            col = GREEN if live else (RED if ever else '#333840')
            c.create_oval(x, y - 5, x + 10, y + 5, fill=col, outline='')
            c.create_text(x + 14, y, text=name, fill=DIM, font=self.font(7),
                          anchor='w')
            x += 50

    # ── 카메라 ────────────────────────────────────────────────────────────
    def _draw_camera(self, c, x, y, tw, th):
        n = self.n
        self._round_rect(c, x, y, x + tw, y + th, 10,
                         fill='#05070a', outline='#1c2430')
        if not n.show_camera or not _HAVE_CV:
            c.create_text(x + tw / 2, y + th / 2, text='CAM off',
                          fill=DIM, font=self.font(10))
            return tw + 8
        frame = None
        with n._img_lock:
            if n._img_bgr is not None:
                frame = n._img_bgr
            age = time.monotonic() - n._img_t if n._img_t else 1e9
        if frame is None or age > n.stale_s:
            c.create_text(x + tw / 2, y + th / 2, text='/image_raw —',
                          fill=DIM, font=self.font(10))
            return tw + 8
        ih, iw = frame.shape[:2]
        if iw < 2 or ih < 2:
            return tw + 8
        scale = min((tw - 8) / iw, (th - 8) / ih)
        nw, nh = max(2, int(iw * scale)), max(2, int(ih * scale))
        try:
            small = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode('.png', small)
            if not ok:
                return tw + 8
            import base64
            self._photo = tk.PhotoImage(
                data=base64.b64encode(buf.tobytes()).decode('ascii'))
            c.create_image(x + tw / 2, y + th / 2, image=self._photo)
        except Exception:  # noqa: BLE001
            c.create_text(x + tw / 2, y + th / 2, text='CAM err',
                          fill=RED, font=self.font(10))
        return tw + 8

    @staticmethod
    def _round_rect(c, x1, y1, x2, y2, r, **kw):
        r = min(r, abs(x2 - x1) / 2, abs(y2 - y1) / 2)
        pts = [
            x1 + r, y1,
            x2 - r, y1,
            x2, y1, x2, y1 + r,
            x2, y2 - r, x2, y2,
            x2 - r, y2,
            x1 + r, y2,
            x1, y2, x1, y2 - r,
            x1, y1 + r, x1, y1,
        ]
        return c.create_polygon(pts, smooth=True, **kw)


def main(args=None):
    if not os.environ.get('DISPLAY'):
        print('HUD: DISPLAY 가 없다 — 창을 열 수 없다.', flush=True)
        return

    rclpy.init(args=args)
    node = HudNode()
    th = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    th.start()
    if watch_parent is not None:
        watch_parent()
    try:
        HudApp(node).run()
    except KeyboardInterrupt:
        pass
    except tk.TclError as exc:
        node.get_logger().error(f'HUD 창 실패: {exc}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        th.join(timeout=1.0)
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass


if __name__ == '__main__':
    main()
