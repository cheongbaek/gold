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
    상단   ★AS 상태★(AS-ON·RUN / AS-EMERGENCY·PAUSE / AS-OFF) · AEB · 신호등 · GPS
           · A/B 보드 · /drive_state                       [2026-10-07 — as_state()]
    중앙   상면도 차체 + 앞바퀴 조향 + 모서리 원형 게이지 4개
             차 앞 = 라이다 AEB 범위(cone_lidar ROI). 거리 없으면 안 그림
             FL 스로틀%   FR PWM%   RL 브레이크   RR 펄스 라벨
    우측   속도 km/h ★/gps_fused[8] 원시 fix 변위속도★ (폴백 IMU→ENC)
           · PWM · ★L_Pulse / R_Pulse★ (A보드 좌·우 펄스 원값, 목표펄스와 같은 눈금)
           NAV 미니맵
             nav_mode=gps  (기본) 매핑 CSV(북쪽 위) + 실시간 GPS
                           terrain 열 0/L/S/T 로 구간을 칠하고, 다음 구간까지 거리를 적는다
                           라이다가 살아 있으면 코스트맵·롤아웃을 같은 칸에 겹친다
             nav_mode=mppi 차량 기준 2D 탑뷰만 (mppi one_launch)
    좌측   EVENT 로그 — /drive_event 를 쌓아 둔다 (한 줄이 4초 뒤에 사라지지 않는다)
    하단   조향바 · 헤딩 · CTE · 웨이포인트 · 제동 이유 · 토픽 신선도

값이 1.5초 넘게 안 오면 '—' / 회색. 죽은 노드의 마지막 숫자를 현재인 양
띄워 두지 않는다.

키: F11 전체화면, Esc 전체화면 해제(창 닫기는 창 버튼).
카메라 미리보기: show_camera:=false 로 끈다.
"""

from __future__ import annotations

import csv
import glob
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
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

#  ★라이다 판정 상수의 소유자는 driving.py 다★ [2026-09-16]
#  LDR LED 가 '주행을 시작할 수 있는 라이다 상태' 와 ★같은 판정★ 을 해야
#  "LED 는 녹색인데 왜 출발을 안 하지" 가 생기지 않는다. 리터럴을 여기 베끼면
#  반드시 어긋난다(prompt.py 도 같은 이유로 그쪽 상수를 가져온다).

from white1.gps import GPS_FUSED_FIELDS, Q_LABEL  # fused 길이·품질 라벨의 단일 소유자
from white1 import driving as dv
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
#  ★[2026-09-09] /encoder 는 좌+우 ★합★ 이다 — 바퀴 하나 기준으로 접어야 한다★
#  종전 큰 숫자는 `enc * 3.18` 이라 ★실제의 2배★ 를 찍고 있었다(합에 바퀴 하나
#  기준 환산을 그대로 곱했다). 지금 큰 숫자는 GPS 라 이 상수는 폴백에만 쓰이지만,
#  틀린 채로 두면 GPS 가 끊긴 구간에서 다시 2배가 된다.
#  (nxde/master.py · white1/driving.py 의 ENC_SUM_TO_PULSE 와 같은 값·같은 이유)
ENC_SUM_TO_PULSE = 0.5
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
EVENT_LOG_MAX = 40
# driving.py 의 LIDAR_ZONE_CHARS / STOP_ZONE_CHARS / TL_ZONE_CHARS 와 같은 문자.
# 그 외(빈 칸 · '0' · 모르는 글자)는 GPS 추종이다.
ZONE_GPS, ZONE_LIDAR, ZONE_STOP, ZONE_TL = '0', 'L', 'S', 'T'

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
# 경로 구간. T 는 빨강(AEB·E-STOP)과 겹치지 않게 보라.
ZONE_COLOR = {
    ZONE_GPS: CYAN,
    ZONE_LIDAR: ORANGE,
    ZONE_STOP: YELLOW,
    ZONE_TL: '#d46bff',
}
ZONE_NAME = {ZONE_GPS: 'GPS', ZONE_LIDAR: 'L', ZONE_STOP: 'S', ZONE_TL: 'T'}


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


def _zone_char(raw):
    """CSV terrain 한 칸 → '0'|'L'|'S'|'T'. driving 이 구간으로 인정하는 문자만.
    [2026-09-30] 본선 코스의 'T1'~'T5' 도 'T' 로 접는다(판정은 driving.tl_label)."""
    if dv.tl_label(raw):
        return ZONE_TL
    z = str(raw or '').strip().upper()
    if z in (ZONE_LIDAR, ZONE_STOP):
        return z
    return ZONE_GPS


def _fmt_m(d):
    return f'{d:.1f} m' if d < 10.0 else f'{d:.0f} m'


def _zone_status(xy, zone, wp_idx):
    """지금 구간과 다음 구간까지 호길이.

    반환 (문구, 색). 경로가 없으면 (None, None).
    S 는 한 행짜리 지점이라 '남음' 대신 'S 지점' 으로 적는다.
    """
    n = len(xy)
    if n < 2 or len(zone) != n:
        return None, None
    i = max(0, min(int(wp_idx), n - 1))
    here = zone[i]
    dist = 0.0
    nxt = None
    for k in range(i, n - 1):
        seg = math.hypot(xy[k + 1][0] - xy[k][0], xy[k + 1][1] - xy[k][1])
        if zone[k + 1] != here:
            nxt = zone[k + 1]
            dist += seg
            break
        dist += seg
    if here == ZONE_STOP:
        return 'S 지점', ZONE_COLOR[ZONE_STOP]
    if nxt is None:
        if here == ZONE_GPS:
            return '끝까지 GPS', DIM
        if dist < 0.05:
            return f'{ZONE_NAME[here]} 구간', ZONE_COLOR[here]
        return f'{ZONE_NAME[here]} · 끝까지 {_fmt_m(dist)}', ZONE_COLOR[here]
    if here == ZONE_GPS:
        return f'다음 {ZONE_NAME[nxt]}  {_fmt_m(dist)}', ZONE_COLOR[nxt]
    return f'{ZONE_NAME[here]} · 남음 {_fmt_m(dist)}', ZONE_COLOR[here]


def _kept_indices(n, zone, wp_idx, limit=NAV_DRAW_MAX):
    """그릴 점의 인덱스. 구간 경계와 S 한 행은 간격을 건너뛰어도 남긴다."""
    if n <= 0:
        return []
    if n <= limit:
        return list(range(n))
    step = max(1, n // limit)
    keep = set(range(0, n, step))
    keep.add(n - 1)
    keep.add(max(0, min(int(wp_idx), n - 1)))
    for i in range(n):
        if zone[i] == ZONE_STOP or (i > 0 and zone[i] != zone[i - 1]):
            keep.add(i)
            if i > 0:
                keep.add(i - 1)
    return sorted(keep)


AS_UNKNOWN = 'AS —'
AS_ON = 'AS-ON'
AS_EMERGENCY = 'AS-EMERGENCY'
AS_OFF = 'AS-OFF'


def _b_link(board_status):
    """/board_status 의 B 칸. True 연결 / False 단절 / None 모름(토픽 없음·형식 다름)."""
    if not board_status:
        return None
    for part in str(board_status).split(','):
        if part.startswith('B:'):
            return part[2:] == '1'
    return None


def as_state(mode, estop, b_link):
    """★AS 상태 (대회 규정 2.16.1 <표 4>·<표 5>)★ [2026-10-07]

        AS-ON        · RUN   : D5 자율, E-STOP 해제
        AS-EMERGENCY · PAUSE : D5 자율, E-STOP 체결 — B보드가 리니어 2단
        AS-OFF               : D5 수동 — ★E-STOP 을 보지 않는다★, 리니어도 안 쓴다

    입력은 arduino 가 내는 /vehicle_mode · /estop · /board_status 그대로다(새 토픽 없음).
    ★/estop 은 이미 '효력' 이다★ — kasa_1007_B 는 수동에서 E-STOP 을 보지 않아 "STOP" 을
    안 내고, arduino 는 STOP 을 받으면 모드를 자율로 둔다. 그래서 estop 이면 모드와
    무관하게 AS-EMERGENCY 다(구 펌웨어 0909 를 꽂아 수동에서 STOP 이 와도, 그 펌웨어는
    실제로 리니어 2단을 무니 이 표시가 맞다).

    ★B보드 단절은 '모름' 이다★ arduino 는 보드가 빠져도 /vehicle_mode·/estop 을 ★마지막
    값으로 계속 발행★ 한다 — 토픽 신선도만으로는 못 거른다. 모르면 낮게 본다.
    """
    if b_link is False or mode is None or estop is None:
        return AS_UNKNOWN
    if estop:
        return AS_EMERGENCY
    return AS_ON if mode else AS_OFF


def _brake_reason_text(estop, aeb, tl_brake, lstatus, goal_phase, cb_state,
                       goal_need, drive_state, brake_lv):
    """리니어가 밟힌 이유. None 은 그 토픽이 신선하지 않다는 뜻.

    우선순위는 driving 이 소유권을 넘기는 순서와 같다 — E-STOP, AEB, 신호등,
    일시정지 S, 종점, 코너. 종점 크립은 브레이크를 푼 뒤의 단계라 해제보다 먼저
    적는다. diag[8](brake_latched)는 도착과 종점 2단이 같은 1 이라 쓰지 않는다.
    """
    def need_s():
        if goal_need is None or not math.isfinite(goal_need):
            return ''
        return f' · 필요 {_fmt_m(goal_need)}'

    if estop:
        return '제동  E-STOP', RED
    if aeb:
        return '제동  AEB', RED
    if tl_brake is not None and tl_brake > 0:
        return f'제동  신호등 {int(tl_brake)}단', RED
    if lstatus is not None and str(lstatus).upper() == ZONE_STOP:
        return '제동  일시정지 S', YELLOW
    if goal_phase == 3:
        return f'제동  종점 2단{need_s()}', RED
    if goal_phase == 1:
        return f'제동  종점 1단{need_s()}', ORANGE
    if cb_state == 1:
        return '제동  코너 1단', ORANGE
    if drive_state == 'DRIVE_DONE':
        return '제동  도착 2단', ORANGE
    if goal_phase == 2:
        return '제동  종점 크립', CYAN
    if brake_lv is None and goal_phase is None and cb_state is None:
        return '제동  —', DIM
    if brake_lv is not None and brake_lv > 0:
        return f'제동  리니어 {int(brake_lv)}단', ORANGE
    return '제동  해제', DIM


def _event_color(text):
    if text.startswith(('❌', '🚨', '🛑')):
        return RED
    if text.startswith(('⚠️', '🔻')):
        return ORANGE
    if text.startswith(('✅', '🟢', '▶')):
        return GREEN
    return FG


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
        self.declare_parameter('mppi_debug_dir', '')
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
        dbg = str(self.get_parameter('mppi_debug_dir').value or '').strip()
        if not dbg:
            here = os.path.dirname(os.path.abspath(__file__))
            dbg = os.path.normpath(os.path.join(
                here, '..', '..', 'mppi_local_planner', 'debug'))
        self.mppi_debug_dir = dbg
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
        #  ★[2026-09-09] A보드 좌·우 펄스★ /encoder 는 둘의 합이라 어느 바퀴가
        #  덜 도는지 안 보인다. 인휠 2개가 각자 PID 를 닫으므로 좌우가 갈린다.
        self.enc_l = Sample()
        self.enc_r = Sample()
        self.steer = Sample()
        self.speed = Sample()
        self.mode = Sample()
        self.estop = Sample()
        self.aeb = Sample()
        self.aeb_dist = Sample()
        #  라이다 센서 생존 (LDR LED). 누적 수를 함께 센다 — driving 이
        #  LIDAR_SENSOR_MIN_N 개를 요구하므로 같은 기준으로 판정한다.
        self.ouster = Sample()
        self._ouster_n = 0
        self.aeb_sig = Sample()
        self.boards = Sample()
        self.ctrl = Sample()
        self.cmd_pulse = Sample()
        self.cmd_steer = Sample()
        self.drive_state = Sample()
        #  ★조종권 [2026-09-01]★ 지금 /cmd_vel_raw 를 누가 내고 있나. 이 표시가
        #  없으면 주행 중에 GPS 추종과 라이다 회피 중 무엇이 차를 몰고 있는지
        #  화면으로 알 수 없다 — 라바콘 구간에서 조향이 이상할 때 원인을 못 가른다.
        self.lstatus = Sample()           # driving → mppi : 구간 문자 '0'|'L'|'S'
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
        self.route_zone = []         # route_wps 와 같은 길이. '0'|'L'|'S'|'T'
        self.map_trail = []          # 매핑 중 /mapping_point·/fix 로 쌓는 점
        self._event_lock = threading.Lock()
        self.event_log = []          # [(unix_time, text), ...] 최근 EVENT_LOG_MAX

        self._mppi_lock = threading.Lock()
        self._mppi_grid = None       # {w,h,res,ox,oy,data}
        self._mppi_grid_t = 0.0
        self._mppi_path = []         # [(x, y), ...] ego, x 전방 / y 좌
        self._mppi_path_t = 0.0
        self._mppi_ref = []
        self._mppi_ref_t = 0.0

        self._img_lock = threading.Lock()
        self._img_bgr = None
        self._tl_bgr = None
        self._tl_t = 0.0
        self._img_t = 0.0
        self._tl_t = 0.0
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
        self.create_subscription(Int32, '/encoder_l',
                                 lambda m: self.enc_l.set(int(m.data)), qos)
        self.create_subscription(Int32, '/encoder_r',
                                 lambda m: self.enc_r.set(int(m.data)), qos)
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
        #  ★[2026-09-07] /lidar_permit(Bool) → /lstatus(String)★ 값이 곧 조종권이다.
        self.create_subscription(String, '/lstatus',
                                 lambda m: self.lstatus.set(str(m.data)), qos)
        self.create_subscription(Bool, '/lidar_active',
                                 lambda m: self.lidar_active.set(bool(m.data)), qos)
        self.create_subscription(Bool, '/tl/stop_line_wait',
                                 lambda m: self.tl_wait.set(bool(m.data)), qos)
        self.create_subscription(Float32, '/tl/near_metric',
                                 lambda m: self.tl_near.set(float(m.data)), qos)
        self.create_subscription(Imu, '/imu', self._cb_imu, qos)
        #  ★[2026-09-16] 라이다 센서가 실제로 흐르는가 — LDR LED 의 근거★
        #  ★/ouster/points 가 아니라 /ouster/imu 를 본다★ 포인트클라우드는 한 프레임이
        #  수 MB 라 화면 노드가 20Hz 로 받을 이유가 없다. IMU 는 작고 100Hz 이며,
        #  둘 다 같은 드라이버가 낸다 — driving.py 가 게이트에서 이 토픽을 보는 것과
        #  같은 이유다(그쪽 OUSTER_IMU_TOPIC 주석).
        #  ★QoS 는 sensor_data 여야 한다★ ouster 드라이버가 use_system_default_qos:
        #  false 로 BEST_EFFORT 를 쓰므로, 기본 QoS 로 구독하면 ★조용히 안 붙는다★.
        self.create_subscription(Imu, dv.OUSTER_IMU_TOPIC, self._cb_ouster_imu,
                                 qos_profile_sensor_data)
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
        if self._bridge is not None:
            self.create_subscription(
                Image, '/image_raw', self._cb_image, qos_profile_sensor_data)
            self.create_subscription(
                Image, '/tl/debug_image', self._cb_tl_debug, qos_profile_sensor_data)
        cmap = str(self.get_parameter('mppi_costmap_topic').value)
        pth = str(self.get_parameter('mppi_path_topic').value)
        ref = str(self.get_parameter('mppi_ref_path_topic').value)
        self.create_subscription(OccupancyGrid, cmap, self._cb_mppi_grid, 1)
        self.create_subscription(Path, pth, self._cb_mppi_path, 1)
        self.create_subscription(Path, ref, self._cb_mppi_ref, 1)

        self._mppi_param_cli = self.create_client(
            SetParameters, '/mppi_local_planner_node/set_parameters')
        self.get_logger().info(
            f'kasa HUD — 구독 전용. NAV={self.nav_mode}  경로 폴더 {self.data_dir}')

    def set_mppi_param(self, name, value):
        if not self._mppi_param_cli.service_is_ready():
            return
        p = ParamMsg()
        p.name = str(name)
        p.value = ParameterValue()
        p.value.type = ParameterType.PARAMETER_DOUBLE
        p.value.double_value = float(value)
        req = SetParameters.Request()
        req.parameters = [p]
        self._mppi_param_cli.call_async(req)

    def _cb_drive_cmd(self, m):
        text = str(m.data).strip()
        if not text or text.upper() in ROUTE_CMD_WORDS:
            if text.upper() == 'MAP_START':
                self._clear_trail()
                with self._nav_lock:
                    self.route_name = ''
                    self.route_wps = []
                    self.route_zone = []
            return
        if text.lower().endswith('.csv'):
            self._load_route(text)

    def _cb_event(self, m):
        text = str(m.data).strip()
        self.drive_event.set(text)
        if text:
            with self._event_lock:
                self.event_log.append((time.time(), text))
                if len(self.event_log) > EVENT_LOG_MAX:
                    del self.event_log[:-EVENT_LOG_MAX]
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
        zone = []
        try:
            with open(path, 'r', encoding='utf-8') as f:
                for row in csv.DictReader(f):
                    try:
                        la, lo = float(row['latitude']), float(row['longitude'])
                    except (KeyError, ValueError, TypeError):
                        continue
                    if not (math.isfinite(la) and math.isfinite(lo)):
                        continue
                    # 좌표가 성립한 행에서만 terrain 을 읽는다. 인덱스가 어긋나면
                    # 구간 색이 조용히 밀린다 — driving.select_route 와 같은 순서.
                    wps.append((la, lo))
                    zone.append(_zone_char(row.get('terrain', '')))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'HUD 경로 읽기 실패: {exc}')
            return
        if len(wps) < 2:
            self.get_logger().warning(f'HUD 웨이포인트 부족({len(wps)}): {base}')
            return
        with self._nav_lock:
            self.route_name = base
            self.route_wps = wps
            self.route_zone = zone
        self.get_logger().info(
            f'HUD 경로 로드 {base}  WP {len(wps)}  '
            f"L {zone.count(ZONE_LIDAR)}  S {zone.count(ZONE_STOP)}  "
            f"T {zone.count(ZONE_TL)}")

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

    def _cb_ouster_imu(self, msg: Imu):
        """라이다 드라이버 생존 신호. ★값은 쓰지 않는다 — 온다는 사실만 본다★"""
        self._ouster_n += 1
        self.ouster.set(self._ouster_n)

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

    def _cb_tl_debug(self, m):
        if self._bridge is None:
            return
        try:
            bgr = self._bridge.imgmsg_to_cv2(m, desired_encoding='bgr8')
        except Exception:  # noqa: BLE001
            return
        with self._img_lock:
            self._tl_bgr = bgr
            self._tl_t = time.monotonic()

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
        self.page = 'hud'  # hud | menu | map | params
        self.cv = tk.Canvas(self.root, bg=BG, highlightthickness=0)
        self.cv.pack(fill='both', expand=True)
        self.cv.bind('<Configure>', lambda _e: None)
        self._build_param_frame()
        self._debug_idx = 0
        self._debug_files = []
        self._debug_photo = None
        self._debug_shown = ''
        self._debug_scan_t = 0.0
        self._alive = True
        self.tick()

    def _build_param_frame(self):
        self.param_frame = tk.Frame(self.root, bg=BG)
        bar = tk.Frame(self.param_frame, bg='#0b0e14', height=48)
        bar.pack(fill='x')
        tk.Label(bar, text='KASA  파라미터', bg='#0b0e14', fg=GOLD,
                 font=(self.fam, 14, 'bold')).pack(side='left', padx=16, pady=10)
        tk.Button(bar, text='계기판', command=lambda: self._goto('hud'),
                  bg=PANEL2, fg=GREEN, relief='flat', padx=12).pack(
                      side='right', padx=8, pady=8)
        tk.Button(bar, text='메뉴', command=lambda: self._goto('menu'),
                  bg=PANEL2, fg=GOLD, relief='flat', padx=12).pack(
                      side='right', padx=4, pady=8)
        body = tk.Frame(self.param_frame, bg=BG)
        body.pack(fill='both', expand=True, padx=24, pady=12)
        tk.Label(
            body,
            text='드래그하면 mppi_local_planner 에 바로 적용됩니다. 지면 제거 높이 = roi_agl_min.',
            bg=BG, fg=DIM, font=(self.fam, 10),
        ).pack(anchor='w', pady=(0, 10))
        specs = [
            ('지면 제거 높이 AGL [m]', 'roi_agl_min', 0.10, 0.55, 0.30),
            ('콘 앞 오프셋 시작 [m]', 'frenet.arrive_before_m', 1.0, 8.0, 2.5),
            ('콘 뒤 감싸기 [m]', 'frenet.wrap_after_m', 1.0, 5.0, 2.5),
            ('통과 여유 margin [m]', 'avoid.margin_m', 0.10, 0.50, 0.32),
            ('오프셋 상한 [m]', 'avoid.max_offset_m', 0.60, 1.50, 1.12),
            ('검출 사거리 [m]', 'avoid.range_m', 6.0, 18.0, 16.0),
            ('코스트맵 팽창 [m]', 'costmap.inflation_radius', 0.15, 0.80, 0.40),
            ('경로 추종 가중', 'mppi.weight_path', 5.0, 80.0, 40.0),
        ]
        self._param_vars = {}
        for label, key, lo, hi, default in specs:
            row = tk.Frame(body, bg=BG)
            row.pack(fill='x', pady=6)
            tk.Label(row, text=label, bg=BG, fg=FG, width=28, anchor='w',
                     font=(self.fam, 10)).pack(side='left')
            var = tk.DoubleVar(value=default)
            self._param_vars[key] = var
            val_lbl = tk.Label(row, text=f'{default:.2f}', bg=BG, fg=GOLD,
                               width=6, anchor='e', font=(self.fam, 10, 'bold'))
            val_lbl.pack(side='right')

            def _on(v, k=key, lbl=val_lbl, vv=var):
                try:
                    x = float(v)
                except (TypeError, ValueError):
                    return
                lbl.config(text=f'{x:.2f}')
                self.n.set_mppi_param(k, x)

            sc = tk.Scale(
                row, from_=lo, to=hi, resolution=0.01 if hi - lo < 5 else 0.5,
                orient='horizontal', variable=var, showvalue=0,
                command=_on, bg=BG, fg=FG, troughcolor=PANEL2,
                highlightthickness=0, length=420)
            sc.pack(side='left', fill='x', expand=True, padx=8)
        tk.Label(
            body,
            text='디버그 이미지는 L 구간에서 /tmp/mppi_debug/mppi_*.ppm 에 저장됩니다.\n'
                 '노랑=계획 경로  초록=실제 궤적  시안=MPPI 롤아웃  빨강=장애물',
            bg=BG, fg=DIM, font=(self.fam, 9), justify='left',
        ).pack(anchor='w', pady=16)

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
        if self.page != 'hud':
            self._goto('hud')
            return
        if self._fscreen:
            self.toggle_full()

    def _goto(self, page):
        if str(page).startswith('dbg_'):
            return
        prev = self.page
        self.page = page
        if page == 'params':
            self.cv.pack_forget()
            self.param_frame.pack(fill='both', expand=True)
        elif prev == 'params':
            self.param_frame.pack_forget()
            self.cv.pack(fill='both', expand=True)

    def _ui_button(self, c, x, y, bw, bh, label, page, fill=PANEL2, fg=GOLD):
        tag = 'navbtn_' + page
        c.create_rectangle(x, y, x + bw, y + bh, fill=fill, outline=GOLD,
                           width=1, tags=(tag,))
        c.create_text(x + bw / 2, y + bh / 2, text=label, fill=fg,
                      font=self.font(11, 'bold'), tags=(tag,))
        c.tag_bind(tag, '<Button-1>', lambda _e, p=page: self._goto(p))

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

        if self.page == 'menu':
            self._draw_menu(c, w, h, stale)
            return
        if self.page == 'map':
            self._draw_full_map(c, w, h, stale)
            return
        if self.page == 'camera':
            self._draw_camera_page(c, w, h, stale)
            return
        if self.page == 'debug':
            self._draw_debug_page(c, w, h, stale)
            return

        cam_w = 0

        log_w = int(_clamp(w * 0.22, 220, 320))
        log_x, log_y = 12, 56
        cluster_b = h - 132
        log_h = max(80, cluster_b - log_y)
        self._draw_event_log(c, log_x, log_y, log_w, log_h)

        cluster_l = log_x + log_w + 12
        cluster_r = w - 20
        cluster_t = 56
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

    def _draw_menu(self, c, w, h, stale):
        c.create_text(w / 2, h * 0.22, text='KASA', fill=GOLD,
                      font=self.font(28, 'bold'))
        c.create_text(w / 2, h * 0.22 + 36, text='메인 메뉴', fill=DIM,
                      font=self.font(12))
        bw, bh = int(w * 0.36), int(h * 0.10)
        x = (w - bw) / 2
        self._ui_button(c, x, h * 0.36, bw, bh, '계기판', 'hud',
                        fill='#12331f', fg=GREEN)
        self._ui_button(c, x, h * 0.36 + bh + 14, bw, bh, '전체 맵', 'map',
                        fill='#1a2430', fg=CYAN)
        self._ui_button(c, x, h * 0.36 + 2 * (bh + 14), bw, bh, '파라미터', 'params',
                        fill='#2a2210', fg=GOLD)
        self._ui_button(c, x, h * 0.36 + 3 * (bh + 14), bw, bh, '카메라', 'camera',
                        fill='#1a1520', fg=ORANGE)
        self._ui_button(c, x, h * 0.36 + 4 * (bh + 14), bw, bh, '디버그 이미지', 'debug',
                        fill='#152018', fg=GREEN)
        c.create_text(w / 2, h * 0.78, text='Esc  계기판으로', fill=DIM,
                      font=self.font(10))
        self._draw_topbar(c, w, stale)

    def _draw_camera_page(self, c, w, h, stale):
        self._draw_topbar(c, w, stale)
        self._ui_button(c, 16, 56, 88, 28, '계기판', 'hud')
        self._ui_button(c, 112, 56, 88, 28, '메뉴', 'menu')
        c.create_text(w / 2, 70, text='카메라 / 신호등 디버그', fill=GOLD,
                      font=self.font(13, 'bold'))
        pad = 16
        self._draw_camera(c, pad, 96, w - pad * 2, h - 96 - pad, prefer_tl=True)

    def _scan_debug_files(self):
        d = getattr(self.n, 'mppi_debug_dir', '') or ''
        files = []
        if d and os.path.isdir(d):
            files = sorted(
                glob.glob(os.path.join(d, 'mppi_*.ppm')) +
                glob.glob(os.path.join(d, 'mppi_*.png')),
                reverse=True)
        self._debug_files = files
        if self._debug_idx >= len(files):
            self._debug_idx = max(0, len(files) - 1)
        return d

    def _debug_nav(self, delta):
        self._scan_debug_files()
        n = len(self._debug_files)
        if n <= 0:
            return
        self._debug_idx = (self._debug_idx + delta) % n
        self._debug_shown = ''

    def _draw_debug_page(self, c, w, h, stale):
        self._draw_topbar(c, w, stale)
        self._ui_button(c, 16, 56, 88, 28, '계기판', 'hud')
        self._ui_button(c, 112, 56, 88, 28, '메뉴', 'menu')
        now = time.monotonic()
        if now - self._debug_scan_t > 1.0:
            self._scan_debug_files()
            self._debug_scan_t = now
        d = getattr(self.n, 'mppi_debug_dir', '')
        files = self._debug_files
        c.create_text(w / 2, 70, text='디버그 이미지', fill=GOLD,
                      font=self.font(13, 'bold'))
        c.create_text(w / 2, 90, text=d or '(경로 없음)', fill=DIM,
                      font=self.font(8))
        bx = w / 2 - 140
        self._ui_button(c, bx, 100, 80, 26, '이전', 'dbg_prev')
        self._ui_button(c, bx + 90, 100, 80, 26, '다음', 'dbg_next')
        self._ui_button(c, bx + 180, 100, 100, 26, '새로고침', 'dbg_ref')
        # tag_bind pages: reuse _goto? better custom tags
        c.tag_bind('navbtn_dbg_prev', '<Button-1>',
                   lambda _e: self._debug_nav(-1))
        c.tag_bind('navbtn_dbg_next', '<Button-1>',
                   lambda _e: self._debug_nav(1))
        c.tag_bind('navbtn_dbg_ref', '<Button-1>',
                   lambda _e: (self._scan_debug_files(), setattr(self, '_debug_shown', '')))
        if not files:
            c.create_text(w / 2, h / 2, text='저장된 이미지 없음', fill=DIM,
                          font=self.font(16, 'bold'))
            c.create_text(w / 2, h / 2 + 28,
                          text='L 구간을 달리면 여기에 쌓입니다', fill='#3a4250',
                          font=self.font(10))
            return
        path = files[self._debug_idx]
        cap = os.path.basename(path)
        c.create_text(w / 2, 136, text=f'{self._debug_idx + 1}/{len(files)}  {cap}',
                      fill=FG, font=self.font(10))
        if self._debug_shown != path:
            try:
                self._debug_photo = tk.PhotoImage(file=path)
                self._debug_shown = path
            except tk.TclError:
                self._debug_photo = None
                self._debug_shown = path
        if self._debug_photo is None:
            c.create_text(w / 2, h / 2, text='이미지 로드 실패 (ppm)', fill=RED,
                          font=self.font(12))
            return
        iw = self._debug_photo.width()
        ih = self._debug_photo.height()
        box_w, box_h = w - 32, h - 160
        # PhotoImage subsample if too big
        img = self._debug_photo
        if iw > box_w or ih > box_h:
            fx = max(1, int(math.ceil(iw / max(box_w, 1))))
            fy = max(1, int(math.ceil(ih / max(box_h, 1))))
            f = max(fx, fy)
            try:
                img = self._debug_photo.subsample(f, f)
                self._debug_photo_view = img
            except tk.TclError:
                img = self._debug_photo
        c.create_image(w / 2, (160 + h) / 2, image=img)

    def _draw_full_map(self, c, w, h, stale):
        self._draw_topbar(c, w, stale)
        self._ui_button(c, 16, 56, 88, 28, '계기판', 'hud')
        self._ui_button(c, 112, 56, 88, 28, '메뉴', 'menu')
        c.create_text(w / 2, 70, text='전체 맵', fill=GOLD,
                      font=self.font(13, 'bold'))
        pad = 16
        log_w = int(_clamp(w * 0.22, 220, 320))
        self._draw_nav(c, pad, 96, w - pad * 2 - log_w - 8, h - 96 - pad,
                       stale, full=True)
        self._draw_event_log(c, w - pad - log_w, 96, log_w, h - 96 - pad)

    # ── 상단 필 ────────────────────────────────────────────────────────────
    def _draw_topbar(self, c, w, stale):
        n = self.n
        c.create_rectangle(0, 0, w, 48, fill='#0b0e14', outline='')
        c.create_text(18, 24, text='KASA', fill=GOLD, anchor='w',
                      font=self.font(16, 'bold'))
        c.create_text(78, 24, text='HUD', fill=DIM, anchor='w',
                      font=self.font(11))

        x = 140
        x = self._pill(c, x, 10, *self._as_pill(stale))
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
        self._ui_button(c, w - 108, 10, 88, 28, '메뉴', 'menu')
        c.create_text(w - 122, 24, text=st_s, fill=col, anchor='e',
                      font=self.font(13, 'bold'))

        wait = n.prompt_wait.get(2.5)
        if wait:
            c.create_text(w - 18, 42, text=str(wait), fill=ORANGE, anchor='e',
                          font=self.font(8))

    def _as_pill(self, stale):
        """★AS 상태 필★ [2026-10-07] — 종전 AUTO/MANUAL 필과 E-STOP 필을 하나로 합쳤다
        (AS 상태가 모드를 이미 담고 있다). 판정은 모듈 함수 as_state() 하나다."""
        n = self.n
        s = as_state(n.mode.get(stale), n.estop.get(stale),
                     _b_link(n.boards.get(stale)))
        if s == AS_ON:
            return 'AS-ON · RUN', '#12331f', GREEN
        if s == AS_EMERGENCY:
            flash = int(time.monotonic() * 4) % 2
            return 'AS-EMERGENCY · PAUSE', RED if flash else '#5a1020', '#ffffff'
        if s == AS_OFF:
            return 'AS-OFF', '#33280c', GOLD
        return AS_UNKNOWN, PANEL2, DIM

    def _drive_by_pill(self, stale):
        """지금 /cmd_vel_raw 를 누가 내고 있나. ★GPS / 라이다 / 라이다 없음★

        · GPS      허락이 내려가 있다 = driving 이 낸다 (평소)
        · 라이다   허락이 올라가 있고 mppi 도 살아 있다 = mppi 가 낸다 (L 구간)
        · 라이다?  허락은 올라갔는데 mppi 신고가 없다 — ★이 상태로는 아무도 몰지
                   않는다★. driving 이 곧 정지시키므로 오래 보이지는 않지만,
                   보인다면 use_lidar:=false 로 띄웠거나 mppi 가 죽은 것이다.
        · 라이다 대기  mppi 는 살아 있고 회피 구간을 기다린다 (평소 GPS 와 함께 뜸)
        """
        p = self.n.lstatus.get(stale)
        p = None if p is None else (p == 'L')
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
        # ══════════════════════════════════════════════════════════════════════
        #  ★[2026-09-09] 큰 숫자를 ★GPS 속도★ 로 바꿨다 (종전: 엔코더)★
        # ══════════════════════════════════════════════════════════════════════
        #  ★왜★ 엔코더는 이 차에서 절대속도의 기준이 못 된다:
        #    · A보드 기동 블랭킹 구간에서 ★허수 카운트★ 를 뱉는다 — 실측으로 지령
        #      0~1펄스인데 중앙 16, 최대 34 까지 튀었다(정상 구간 4~5).
        #    · 저속에서 위로 튀는 성질이라 '섰는데 빠르게 보이는' 방향으로 틀린다.
        #  driving.py 도 2026-08-12 에 같은 이유로 속도 1순위를 GPS 로 옮겼고
        #  (measured_kmh), 코너 제동은 아예 "GPS 를 못 읽으면 제동하지 않는다" 다.
        #  ★화면의 큰 숫자만 다른 기준을 쓰고 있을 이유가 없다.★
        #
        #  출처 : /gps_fused[8] = 원시 fix 변위 속도 [km/h] (gps.py 가 계산해 싣는다.
        #         DR 로 메운 구간은 NaN 이 아니라 마지막 원시값이 유지된다)
        #  폴백 순서 : GPS → IMU 적분(/speed) → 엔코더.  ENC 는 맨 아래로 내렸다 —
        #  없애지는 않는다(GPS 두절 구간에서 '아주 없음' 보다는 낫다).
        fused = n.fused.get(stale)
        gps_kmh = None
        if fused is not None and len(fused) > 8:
            v = fused[8]
            if v == v and v >= 0.0:          # NaN 이 아니고 음수가 아니면
                gps_kmh = float(v)
        if gps_kmh is not None:
            sp = gps_kmh
            src = 'GPS'
        elif imu is not None:
            sp = imu
            src = 'IMU'
        elif enc is not None:
            #  ★바퀴 하나 기준으로 접는다★ /encoder 는 좌+우 ★합★ 이다.
            sp = enc * ENC_SUM_TO_PULSE * KMH_PER_PULSE
            src = 'ENC'
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

        # ══════════════════════════════════════════════════════════════════════
        #  ★[2026-09-09] '펄스 · ENC' → ★L_Pulse · R_Pulse★ 로 바꿨다★
        # ══════════════════════════════════════════════════════════════════════
        #  ★종전 두 칸은 사실상 같은 것을 보고 있었다★ 이 차에서 /encoder 는 이름만
        #  encoder 이지 1/5카의 엔코더 카운트가 아니다 — 홀 3상 XOR 합산 ★펄스★ 를
        #  그대로 싣는다. 그래서 '펄스'(/drive_pulse_cmd)와 'ENC'(/encoder)가 같은
        #  눈금의 값이 되어 한 칸이 남았다.
        #
        #  ★대신 좌·우를 나눠 보인다★ 인휠 2개가 ★각자 PID 를 닫는다★ (A보드
        #  좌 2번핀→8번PWM / 우 21번핀→9번PWM, 교차 없음). 그래서 한쪽만 덜 도는
        #  상황이 실제로 생기는데, 합(/encoder)만 보면 '전체가 조금 느리다' 로만
        #  보이고 어느 바퀴인지 알 수 없다. 좌우를 따로 보면 그 자리에서 드러난다.
        #  두 값 모두 ★목표펄스(0~15)와 같은 눈금★ 이라 지령과 바로 비교된다.
        pwm = n.pwm.get(stale)
        el = n.enc_l.get(stale)
        er = n.enc_r.get(stale)
        pwm_s = '—' if pwm is None else str(int(pwm))
        el_s = '—' if el is None else str(int(el))
        er_s = '—' if er is None else str(int(er))
        c.create_text(x, y + 78,
                      text=f'PWM  {pwm_s}    L_Pulse  {el_s}    R_Pulse  {er_s}',
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
        """표시할 경로 점. 선택된 CSV가 있으면 그걸, 없으면 매핑 트레일.

        반환 (pts, zone, name, kind). zone 은 경로일 때만 pts 와 같은 길이.
        """
        n = self.n
        with n._nav_lock:
            wps = list(n.route_wps)
            zone = list(n.route_zone)
            trail = list(n.map_trail)
            name = n.route_name
        if len(wps) >= 2:
            if len(zone) != len(wps):
                zone = [ZONE_GPS] * len(wps)
            return wps, zone, name, 'route'
        if len(trail) >= 2:
            return trail, [], 'mapping', 'trail'
        return [], [], '', 'none'

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

    def _draw_nav_lidar_overlay(self, c, scr, live_xy, heading_deg, grid,
                               mppi_path, pt, mppi_ref, rt, now, stale):
        """ego 코스트맵·롤아웃 → GPS 미니맵 (동=x, 북=y). heading 0=동 CCW."""
        h = math.radians(heading_deg)
        ch, sh = math.cos(h), math.sin(h)
        #  ★ego 원점은 뒷차축(= 라이다)이고 live_xy 는 GPS 안테나(앞차축 위)다★ [2026-09-30]
        #  그대로 얹으면 콘·회피 경로가 안테나 거리(1.25 m)만큼 앞에 그려진다.
        e0 = live_xy[0] - dv.GPS_ANT_X_M * ch
        n0 = live_xy[1] - dv.GPS_ANT_X_M * sh

        def ego_en(ex, ey):
            return e0 + ex * ch - ey * sh, n0 + ex * sh + ey * ch

        if grid is not None:
            gw, gh, res = grid['w'], grid['h'], grid['res']
            ox, oy, data = grid['ox'], grid['oy'], grid['data']
            step = max(1, int(0.30 / max(res, 0.05)))
            r = max(1.6, 0.22 * (scr(e0 + 1.0, n0)[0] - scr(e0, n0)[0]))
            for iy in range(0, gh, step):
                ey = oy + (iy + 0.5) * res
                row = iy * gw
                for ix in range(0, gw, step):
                    v = data[row + ix]
                    if v < 40:
                        continue
                    ex = ox + (ix + 0.5) * res
                    px, py = scr(*ego_en(ex, ey))
                    col = RED if v >= 100 else ORANGE
                    c.create_rectangle(px - r, py - r, px + r, py + r,
                                       fill=col, outline='')

        def polyline(seq, color, width):
            if len(seq) < 2:
                return
            if len(seq) > 80:
                seq = seq[::max(1, len(seq) // 80)] + [seq[-1]]
            flat = []
            for ex, ey in seq:
                flat.extend(scr(*ego_en(ex, ey)))
            c.create_line(*flat, fill=color, width=width, smooth=True)

        if mppi_ref and (now - rt) < stale:
            polyline(mppi_ref, YELLOW, 2)
        if mppi_path and (now - pt) < stale:
            polyline(mppi_path, CYAN, 3)

    # ── NAV 미니맵 (북쪽 위). 경로 없으면 NONE ────────────────────────────
    def _draw_nav(self, c, x, y, w, h, stale, full=False):
        n = self.n
        if n.nav_mode == 'mppi' and not full:
            self._draw_nav_mppi(c, x, y, w, h, stale)
            return
        self._round_rect(c, x, y, x + w, y + h, 12,
                         fill='#0c1016', outline='#1c2430')
        c.create_text(x + 12, y + 12, text='NAV', fill=GOLD, anchor='nw',
                      font=self.font(9, 'bold'))

        pts, zone, name, kind = self._nav_points()
        live_lat, live_lon, heading = self._live_pose(stale)

        if kind == 'none':
            c.create_text(x + w / 2, y + h / 2, text='NONE',
                          fill=DIM, font=self.font(22, 'bold'))
            c.create_text(x + w / 2, y + h / 2 + 26, text='경로 없음',
                          fill='#3a4250', font=self.font(9))
            return

        cap = name if name and name != 'mapping' else '매핑 중'
        if len(cap) > 18:
            cap = cap[:15] + '…'
        c.create_text(x + w - 10, y + 12, text=cap, fill=DIM, anchor='ne',
                      font=self.font(8))

        # 원점: 경로 첫 점. 실시간 GPS 도 같은 평면에 올린다.
        lat0, lon0 = pts[0]
        xy = [_latlon_to_xy(la, lo, lat0, lon0) for la, lo in pts]
        zoned = kind == 'route' and len(zone) == len(xy)

        live_xy = None
        if live_lat is not None:
            live_xy = _latlon_to_xy(live_lat, live_lon, lat0, lon0)

        xs = [p[0] for p in xy]
        ys = [p[1] for p in xy]
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
        grid, gt, mppi_path, pt, mppi_ref, rt = self._mppi_snapshot()
        now = time.monotonic()
        lidar_live = (grid is not None and live_xy is not None and
                      (now - gt) < max(stale, 1.0))
        if lidar_live and not full:
            # 미니맵: 차 주변. 전체 맵 탭은 경로 전체를 그린다.
            span = 40.0
            midx, midy = live_xy
        inner = min(w, h) - 36
        sc = inner / span
        cx = x + w / 2
        cy = y + h / 2 + 8

        def scr(east, north):
            return cx + (east - midx) * sc, cy - (north - midy) * sc

        def polyline(seq, color, width, smooth=True):
            if len(seq) < 2:
                return
            flat = []
            for east, north in seq:
                flat.extend(scr(east, north))
            c.create_line(*flat, fill=color, width=width, smooth=smooth)

        # wp_idx 는 원본 CSV 행 번호다. 간격을 건너뛴 그림에 클램프하면
        # 구간 경계가 밀린다.
        wp_idx = 0
        ego = n.ego.get(stale)
        if ego and len(ego) >= 6:
            wp_idx = max(0, min(int(ego[4]), len(xy) - 1))
        if kind == 'trail':
            wp_idx = max(0, len(xy) - 1)

        if zoned:
            self._draw_zone_path(c, xy, zone, wp_idx, polyline, scr)
        else:
            if len(xy) > NAV_DRAW_MAX:
                step = max(1, len(xy) // NAV_DRAW_MAX)
                xy_draw = xy[::step]
                if xy_draw[-1] != xy[-1]:
                    xy_draw.append(xy[-1])
                draw_idx = max(0, min(wp_idx // step, len(xy_draw) - 1))
            else:
                xy_draw = xy
                draw_idx = wp_idx
            if draw_idx >= 1:
                polyline(xy_draw[:draw_idx + 1], GREEN_DIM, 2)
            polyline(xy_draw[draw_idx:], CYAN, 2)

        sx0, sy0 = scr(*xy[0])
        gx, gy = scr(*xy[-1])
        c.create_oval(sx0 - 4, sy0 - 4, sx0 + 4, sy0 + 4, fill=GREEN, outline='')
        c.create_oval(gx - 5, gy - 5, gx + 5, gy + 5, fill=ORANGE, outline='')

        note_y = y + 28
        if zoned:
            line, col = _zone_status(xy, zone, wp_idx)
            if line:
                c.create_text(x + 12, note_y, text=line, fill=col, anchor='nw',
                              font=self.font(8, 'bold'))
                note_y += 14
            if full:
                self._draw_zone_legend(c, x + 12, y + h - 42)

        if lidar_live and heading is not None:
            self._draw_nav_lidar_overlay(
                c, scr, live_xy, heading, grid, mppi_path, pt, mppi_ref, rt, now,
                max(stale, 1.0))
            c.create_text(x + 12, note_y, text='LIDAR', fill=ORANGE, anchor='nw',
                          font=self.font(7, 'bold'))

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

    def _draw_zone_path(self, c, xy, zone, wp_idx, polyline, scr):
        """terrain 별로 경로를 칠한다. 지나온 구간은 같은 색을 어둡게."""
        idxs = _kept_indices(len(xy), zone, wp_idx)
        if len(idxs) < 2:
            return
        cur = []
        cur_col = None
        cur_w = 2

        def flush():
            nonlocal cur
            if cur_col is not None and len(cur) >= 2:
                polyline(cur, cur_col, cur_w, smooth=False)
            cur = []

        for a, b in zip(idxs, idxs[1:]):
            z = zone[b]
            passed = b <= wp_idx
            col = _mix(ZONE_COLOR[z], '#0c1016', 0.62) if passed else ZONE_COLOR[z]
            width = 2 if passed else 3
            if col != cur_col or width != cur_w:
                flush()
                cur_col, cur_w = col, width
                cur = [xy[a], xy[b]]
            else:
                cur.append(xy[b])
        flush()

        # S 는 한 행이라 선으로는 안 보인다. 마름모로 찍는다.
        for i in idxs:
            if zone[i] != ZONE_STOP:
                continue
            px, py = scr(*xy[i])
            col = _mix(YELLOW, '#0c1016', 0.45) if i <= wp_idx else YELLOW
            r = 5
            c.create_polygon(
                px, py - r, px + r, py, px, py + r, px - r, py,
                fill=col, outline='#1a1408')

    def _draw_zone_legend(self, c, x, y):
        xx = x
        for z, name in ((ZONE_GPS, 'GPS'), (ZONE_LIDAR, 'L'),
                        (ZONE_STOP, 'S'), (ZONE_TL, 'T')):
            c.create_rectangle(xx, y, xx + 10, y + 10,
                               fill=ZONE_COLOR[z], outline='')
            c.create_text(xx + 14, y + 5, text=name, fill=DIM, anchor='w',
                          font=self.font(8))
            xx += 14 + max(18, 8 * len(name)) + 10

    def _draw_event_log(self, c, x, y, w, h):
        """최근 /drive_event. 아래쪽이 최신이다."""
        self._round_rect(c, x, y, x + w, y + h, 12,
                         fill='#0c1016', outline='#1c2430')
        c.create_text(x + 12, y + 14, text='EVENT', fill=GOLD, anchor='w',
                      font=self.font(9, 'bold'))
        with self.n._event_lock:
            rows = list(self.n.event_log)
        c.create_text(x + w - 10, y + 14, text=str(len(rows)), fill=DIM,
                      anchor='e', font=self.font(8))
        if not rows:
            c.create_text(x + w / 2, y + h / 2, text='이벤트 없음',
                          fill=DIM, font=self.font(10))
            return
        px = max(8, int(8 * max(c.winfo_height(), 1) / 720.0))
        line_h = px + 7
        max_lines = max(1, int((h - 36) / line_h))
        show = rows[-max_lines:]
        time_w = int(px * 6.2)
        max_ch = max(4, int((w - 20 - time_w) / max(px * 1.05, 1)))
        yy = y + 30
        for t, text in show:
            stamp = time.strftime('%H:%M:%S', time.localtime(t))
            shown = text if len(text) <= max_ch else text[:max_ch - 1] + '…'
            c.create_text(x + 8, yy, text=stamp, fill=DIM, anchor='nw',
                          font=self.font(8))
            c.create_text(x + 10 + time_w, yy, text=shown,
                          fill=_event_color(text), anchor='nw',
                          font=self.font(8))
            yy += line_h

    def _brake_reason(self, stale):
        n = self.n
        diag = n.diag.get(stale)

        def num(i):
            if not diag or len(diag) <= i:
                return None
            v = float(diag[i])
            return v if math.isfinite(v) else None

        goal_phase = num(18)
        cb_state = num(19)
        goal_need = num(22)
        return _brake_reason_text(
            n.estop.get(stale),
            n.aeb.get(stale),
            n.tl_brake.get(stale),
            n.lstatus.get(stale),
            None if goal_phase is None else int(goal_phase),
            None if cb_state is None else int(cb_state),
            goal_need,
            n.drive_state.get(stale),
            n.brake_lv.get(stale),
        )

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

        reason, reason_col = self._brake_reason(stale)
        c.create_text(20, y0 + 64, text=reason, fill=reason_col,
                      font=self.font(11, 'bold'), anchor='w')

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
            #  ★[2026-09-16] LDR 은 라이다 연결이다 — AEB 노드가 아니다★
            #  종전에는 n.aeb_dist(/cone_lidar_node/obstacle_distance)를 봤는데,
            #  white1/one_launch.py 는 cone_lidar_node 를 ★띄우지 않는다★(6.4⑦).
            #  그래서 라이다가 멀쩡히 돌아도 이 LED 는 ★구조상 절대 켜지지 않았다★.
            #  이제 driving 의 주행 게이트와 같은 것을 본다(아래 lidar_* 계산).
            ('LDR', None),
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
        #  ★LDR — driving.lidar_wait_reason() 과 ★같은 판정★ 이다★
        #  센서(/ouster/imu 가 충분히·신선하게 온다) + 플래너(mppi 의 /lidar_active).
        #  둘 다여야 녹색 = ★이 상태면 주행이 라이다 게이트를 통과한다★.
        #  하나라도 받은 적이 있는데 지금 아니면 빨강(끊긴 것), 아예 없으면 회색.
        lidar_sensor = (n._ouster_n >= dv.LIDAR_SENSOR_MIN_N
                        and n.ouster.fresh(dv.LIDAR_SENSOR_STALE_S))
        lidar_live = lidar_sensor and n.lidar_active.fresh(dv.LIDAR_ACTIVE_STALE_S)
        lidar_ever = (n.ouster.t > 0) or (n.lidar_active.t > 0)

        mppi_live = False
        mppi_ever = False
        if n.nav_mode == 'mppi':
            mppi_ever = n._mppi_grid_t > 0
            mppi_live = mppi_ever and (time.monotonic() - n._mppi_grid_t) < stale
        for name, samp in leds:
            if name == 'LDR':
                live = lidar_live
                ever = lidar_ever
            elif name == 'CAM':
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
    def _draw_camera(self, c, x, y, tw, th, prefer_tl=False):
        n = self.n
        self._round_rect(c, x, y, x + tw, y + th, 10,
                         fill='#05070a', outline='#1c2430')
        if not _HAVE_CV:
            c.create_text(x + tw / 2, y + th / 2, text='CAM off',
                          fill=DIM, font=self.font(10))
            return tw + 8
        frame = None
        src = '/image_raw'
        with n._img_lock:
            if prefer_tl and n._tl_bgr is not None:
                frame = n._tl_bgr
                age = time.monotonic() - n._tl_t if n._tl_t else 1e9
                src = '/tl/debug_image'
            elif n._img_bgr is not None:
                frame = n._img_bgr
                age = time.monotonic() - n._img_t if n._img_t else 1e9
            else:
                age = 1e9
        if frame is None or age > max(n.stale_s, 1.5):
            c.create_text(x + tw / 2, y + th / 2, text=src + ' —',
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
