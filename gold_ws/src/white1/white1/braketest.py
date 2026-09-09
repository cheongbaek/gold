#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
braketest.py ― ★브레이크 제동거리 측정 전용 노드★ [white1 / 2026-09-09]
════════════════════════════════════════════════════════════════════════════════
    ros2 launch white1 braketest.launch.py

  직선(또는 직선에 가까운) 매핑 경로를 GPS 로 추종하면서 ★고정 속도★ 로 달리다가,
  매핑 CSV 의 `terrain` 열에 ★S★ 를 적어 둔 행을 지나가는 즉시 ★리니어 2단★ 을
  물고 완전정지한다. 그 사이의 거리·시간·감속도를 재서 로그와 화면에 남기고,
  런치를 스스로 내린다.

  ★driving.py 를 쓰지 않는다★ 그쪽은 코너 감속·종점 접근·CTE 적분·라이다 이양·
  신호등까지 얹힌 3600줄짜리 주행기이고, 그 장치들이 전부 '제동거리를 재는 일'에
  개입한다(속도를 미리 낮추고, 코너에서 1단을 물고, 종점 5m 앞에서 또 문다).
  ★재려는 것 하나만 남긴 노드★ 가 이 파일이다.

════════════════════════════════════════════════════════════════════════════════
 ⚠️⚠️ 안전 — 읽고 시작할 것 ⚠️⚠️
════════════════════════════════════════════════════════════════════════════════
  기본값 ★10펄스 = 31.8 km/h★ 다. 이 차의 2단 실측 감속도는 2.2~3.8 m/s² 이므로
  (BRAKING.md 1절) 순수 제동거리만:

        v = 10펄스 = 8.84 m/s
        a = 2.2 (하한)  →  17.8 m      a = 3.8 (호조건)  →  10.3 m
        + 판정·행정 지연 0.30s × 8.84 =  2.7 m
        ────────────────────────────────────────────────
        ★S 지점 뒤로 13 ~ 20 m 를 더 간다★

  거기에 ①헤딩 초기화 구간(1~2m) ②10펄스까지 가속하는 구간이 앞에 붙는다.
  ★S 지점 뒤 최소 30 m 는 비어 있어야 한다.★ 경로가 S 에서 끝나면 안 된다 —
  이 노드는 S 를 지나 ★멈출 때까지 계속 달린다★.

  · E-STOP 은 언제든 듣는다(하드웨어. B보드가 직접 문다).
  · D5 스위치를 수동조종으로 내리면 즉시 손을 뗀다.
  · 경로에서 CTE_ABORT_M 이상 벗어나면 스스로 2단을 물고 끝낸다.

════════════════════════════════════════════════════════════════════════════════
 ★★ 속도 지령 — 파일 상단 두 값이 전부다 ★★
════════════════════════════════════════════════════════════════════════════════
    DRIVE_PULSE = 10     ← ★기본★ A보드 목표펄스(0~15). 보드 PID 가 이 속도를 맞춘다
    DRIVE_PWM   = 0      ← 0 이 아니면 ★이쪽이 이긴다★ A보드 직접 PWM(16~255)

  ★왜 둘인가★ 제동거리는 '어떤 속도로 진입했는가' 가 전부인데, 두 경로는 진입
  속도를 만드는 방식이 다르다:
    · 펄스 : 보드 PID 가 목표를 맞춘다. 노면·경사가 달라도 속도가 같아진다.
             ★같은 속도로 반복 측정할 때 쓴다.★
    · PWM  : 듀티를 고정한다. PID·슬루·폭주감지·기동블랭킹이 ★전부 빠진다★
             (A보드 applySide 의 DRIVE_PWM 분기). 속도는 노면에 따라 달라지지만
             ★구동계가 개입하지 않는 순수한 관성 상태★ 로 제동에 들어간다.
             "PID 가 제동 중에 전류를 밀어넣어 거리가 늘어나는가" 를 가릴 때 쓴다.

  ⚠️ PWM 경로는 ★arduino 노드의 auto_direct_pwm 파라미터★ 가 켜져 있어야 한다.
     braketest.launch.py 가 그것을 켜서 띄운다. one_launch.py 는 켜지 않는다.

  ※ 어느 쪽이든 ★리니어가 물리면 A보드 REF 는 0 이 된다★ (arduino compose (4)) —
    제동이 시작되면 구동은 자동으로 끊긴다. 그래서 이 노드는 제동 중에 펄스를
    따로 0 으로 덮지 않는다(덮으면 해제 순간 0→목표로 두 번 전이한다).

════════════════════════════════════════════════════════════════════════════════
 측정하는 것
════════════════════════════════════════════════════════════════════════════════
    체결 시점 속도 v0        GPS 변위속도 (/gps_fused[8]) — ★엔코더를 쓰지 않는다★
    제동거리 d               체결 지점 ~ 완전정지 지점의 GPS 직선거리
    제동시간 t               체결 ~ 완전정지
    평균 감속도 a = v0 / t   그리고 에너지식 v0²/(2d) 도 함께 낸다
    S 지점 초과거리          S 를 얼마나 지나서 섰는가 (실무에서 제일 궁금한 값)

  ★엔코더로 재지 않는 이유★ A보드 기동 블랭킹의 허수 카운트가 저속에서 위로
  튄다(실측 중앙 16, 최대 34 / 정상 4~5). '완전정지' 판정이 그것 때문에 늦어지면
  제동시간이 통째로 틀어진다. 정지 판정만은 엔코더와 GPS 를 ★둘 다★ 요구한다.
"""

import math
import os
import time
from datetime import datetime

import rclpy
import rclpy.executors
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64MultiArray, Int32, String

from white1 import paths
from white1.gps import GPS_FUSED_TOPIC, Q_LABEL, Q_NONE

# ══════════════════════════════════════════════════════════════════════════════
#  ★★ 여기 두 값만 고치면 된다 ★★
# ══════════════════════════════════════════════════════════════════════════════
DRIVE_PULSE = 10          # A보드 목표펄스 0~15. ★10펄스 = 31.8 km/h★
DRIVE_PWM   = 0           # 0 아니면 이쪽이 이긴다. A보드 직접 PWM 16~255
#   ※ 둘 다 런치 인자로도 덮을 수 있다: drive_pulse:=8  /  drive_pwm:=140

# ── 그 밖의 상수 (건드릴 일이 드물다) ──
CONTROL_HZ    = 20.0      # driving.py 와 같은 주기 (A보드 텔레메트리도 20Hz)
HEADING_PULSE = 3         # 헤딩 초기화 구간 속도 ≈ 9.5 km/h
#   ★헤딩이 잡히기 전에는 절대 가속하지 않는다★ 방위를 모르는 채로 31.8 km/h 를
#   내면 순수추종이 아무 방향이나 겨눈다. driving.py 와 같은 절차를 쓴다.
HEAD_MIN_DIST_M   = 1.0
HEAD_MAX_DIST_M   = 5.0
HEAD_MIN_SAMPLES  = 4
HEAD_TARGET_SIGMA = 3.0
HEAD_SIGMA_FLOOR  = 0.02
HEAD_MAX_RESID_M  = 0.15

WHEELBASE_M        = 1.25     # 축거 (순수추종 기하)
STEER_PLANT_GAIN   = 1.26     # pot 지령 / 도로휠각
STEER_UNDERSTEER   = 5.17     # [deg/(m/s²)]
STEER_MAX_DEG      = 40       # B보드 수용 상한

#  ★LFD 는 속도에 비례한다★ LFD = v·√2/ω_n (driving.py lookahead_m 과 같은 설계식).
#  10펄스(8.84 m/s)에서 12.9 m 가 되므로 ★직선 경로여야 성립한다★ — 이 노드가
#  직선 전용인 이유가 여기에도 있다.
LFD_OMEGA_N = 0.97
LFD_MIN_M   = 2.3
LFD_MAX_M   = 14.0

MS_PER_PULSE  = 0.884
KMH_PER_PULSE = 3.182

WP_SEARCH_WINDOW  = 60        # 10펄스는 한 틱에 0.44m 간다 — 창을 넉넉히 둔다
WP_MAX_ADVANCE    = 12
WP_AHEAD_MARGIN_M = 2.0
WP_AHEAD_PENALTY_M = 1.2
CTE_WINDOW_WP     = 60

STOP_ZONE_CHARS = ('S', 's')  # driving.py 와 같은 규약
CTE_ABORT_M     = 3.0         # 이보다 벗어나면 시험을 접고 2단으로 세운다
GPS_TIMEOUT_S   = 2.0
MODE_SETTLE_S   = 0.7

BRAKE_NONE, BRAKE_SOFT, BRAKE_FULL = 0, 1, 2
BRAKE_KEEPALIVE_S = 0.25      # 물고 있는 동안 재발행 (발행자가 여럿인 토픽이다)

#  ★완전정지 판정은 GPS 와 엔코더를 ★둘 다★ 요구한다★ (헤더 '엔코더로 재지 않는
#  이유' 참고). 한쪽만 보면 허수 카운트나 GPS 양자화에 판정이 끌려간다.
STOP_KMH_EPS  = 0.35          # GPS 변위속도가 이 밑
STOP_ENC_EPS  = 0.5           # 엔코더(바퀴 하나 기준)가 이 밑
STOP_HOLD_S   = 0.7           # 그 상태가 이만큼 이어져야 '섰다'
ENC_SUM_TO_PULSE = 0.5
ENC_MEDIAN_N  = 3

BRAKE_MAX_S   = 15.0          # 이 시간 안에 못 서면 굳지 않게 끝낸다(이상 상황)
DONE_LINGER_S = 2.0           # 결과를 찍고 이만큼 뒤에 런치를 내린다

S_WAIT, S_HEADING, S_RUN, S_BRAKE, S_DONE = (
    'WAIT', 'HEADING', 'RUN', 'BRAKE', 'DONE')

EARTH_R = 6378137.0


def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


def latlon_to_xy(lat, lon, lat0, lon0):
    x = EARTH_R * math.radians(lon - lon0) * math.cos(math.radians(lat0))
    y = EARTH_R * math.radians(lat - lat0)
    return x, y


class HeadingEstimator:
    """출발 직진 구간의 GPS 점들에 직선을 맞춰 초기 방위를 낸다.

    ★driving.py 의 같은 이름 클래스와 같은 방법이다★ 자이로는 절대 기준이 없어서
    누군가 한 번은 '지금 북쪽이 어디인가' 를 말해 줘야 하고, 그 답을 곧게 굴러가는
    동안의 GPS 변위가 준다. 여기서는 그 코드를 ★그대로 옮기지 않고 최소한만★ 뒀다.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.pts = []

    def add(self, x, y):
        if self.pts:
            px, py = self.pts[-1]
            if math.hypot(x - px, y - py) < 0.05:
                return
        self.pts.append((x, y))

    def distance(self):
        if len(self.pts) < 2:
            return 0.0
        (x0, y0), (x1, y1) = self.pts[0], self.pts[-1]
        return math.hypot(x1 - x0, y1 - y0)

    def solve(self):
        n = len(self.pts)
        if n < 2:
            return None
        dist = self.distance()
        if dist <= 1e-6:
            return None
        (x0, y0), (x1, y1) = self.pts[0], self.pts[-1]
        heading = math.degrees(math.atan2(y1 - y0, x1 - x0))
        # 직선 잔차 RMS — '곧게 갔는가' 를 본다(곡선이면 방위를 믿을 수 없다)
        ch, sh = math.cos(math.radians(heading)), math.sin(math.radians(heading))
        acc = 0.0
        for (px, py) in self.pts:
            dx, dy = px - x0, py - y0
            acc += (-dx * sh + dy * ch) ** 2
        resid = math.sqrt(acc / n)
        # 방위 표준편차 ≈ 측방오차 / 이동거리
        sigma = math.degrees(math.atan2(max(resid, HEAD_SIGMA_FLOOR), dist))
        return heading, sigma, resid, n, dist


class BrakeTestNode(Node):
    """고정 속도 직진 → S 통과 → 리니어 2단 → 완전정지 → 결과 보고 → 종료."""

    def __init__(self):
        super().__init__('braketest_node')

        self.declare_parameter('data_dir', '')
        self.declare_parameter('route', '')
        self.declare_parameter('drive_pulse', DRIVE_PULSE)
        self.declare_parameter('drive_pwm', DRIVE_PWM)
        self.declare_parameter('heading_pulse', HEADING_PULSE)
        self.declare_parameter('cte_abort_m', CTE_ABORT_M)
        #  ★자동 출발★ 런치를 띄우면 준비되는 대로 곧바로 굴러간다(사용자 지시).
        #    false 로 두면 /braketest_go 에 true 가 올 때까지 기다린다 — 실차에서
        #    "차 앞을 비웠는지" 를 한 번 더 확인하고 싶을 때 쓴다.
        self.declare_parameter('auto_start', True)

        self.data_dir = paths.data_dir(self.get_parameter('data_dir').value or '')
        self.drive_pulse = int(self.get_parameter('drive_pulse').value)
        self.drive_pwm = int(self.get_parameter('drive_pwm').value)
        self.heading_pulse = int(self.get_parameter('heading_pulse').value)
        self.cte_abort = float(self.get_parameter('cte_abort_m').value)
        self.auto_start = bool(self.get_parameter('auto_start').value)

        #  ★지령값을 한 번만 정해 둔다★ 주행 중에 바뀌지 않는다 — 그게 이 시험의 전제다.
        if self.drive_pwm > 0:
            self.cmd_value = float(max(16, min(255, self.drive_pwm)))
            self.cmd_kind = f"직접 PWM {int(self.cmd_value)}"
            self.nominal_ms = float('nan')      # PWM 은 속도를 예측할 수 없다
        else:
            self.cmd_value = float(max(0, min(15, self.drive_pulse)))
            self.cmd_kind = (f"{int(self.cmd_value)}펄스 "
                             f"≈ {self.cmd_value * KMH_PER_PULSE:.1f} km/h")
            self.nominal_ms = self.cmd_value * MS_PER_PULSE

        # ── 발행 ──
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.pub_state = self.create_publisher(Bool, '/control_state', 10)
        self.pub_brake = self.create_publisher(Int32, '/brake_level', 10)
        self.pub_event = self.create_publisher(String, '/drive_event', 10)
        self.pub_dstate = self.create_publisher(String, '/drive_state', 10)
        #  ★record 가 이 시험도 기록하게 한다★ 그쪽은 /drive_state 가 DRIVE_* 일 때만
        #  켜지므로, 이 노드도 같은 이름을 쓴다(BRAKETEST_* 를 새로 만들지 않는다).
        self.pub_done = self.create_publisher(Bool, '/braketest_done', 10)

        # ── 구독 ──
        self.create_subscription(Float64MultiArray, GPS_FUSED_TOPIC,
                                 self.cb_gps, 10)
        self.create_subscription(Imu, '/imu', self.cb_imu, 10)
        self.create_subscription(Int32, '/encoder', self.cb_encoder, 10)
        self.create_subscription(Bool, '/vehicle_mode', self.cb_mode, 10)
        self.create_subscription(Bool, '/estop', self.cb_estop, 10)
        self.create_subscription(Bool, '/braketest_go', self.cb_go, 10)

        # ── 상태 ──
        self.state = S_WAIT
        self.state_t0 = time.time()
        self.lat0 = self.lon0 = None
        self.x = self.y = 0.0
        self.fix_ok = False
        self.fix_time = 0.0
        self.gps_kmh = None
        self.gps_quality = Q_NONE
        self.gps_sigma = float('nan')
        self.heading = None
        self.gyro_z = 0.0
        self.imu_time = 0.0
        self.enc_pulse = 0.0
        self._enc_buf = []
        self.auto_mode = None
        self.estop = False
        self.go = False

        self.head_est = HeadingEstimator()
        self.waypoints = []
        self.wp_zone = []
        self.wp_idx = 0
        self._wp_prev = 0
        self.stop_idx = []
        self.route_name = ''

        self.brake_now = BRAKE_NONE
        self._brake_out = -1
        self._brake_t = 0.0
        self._last_steer = 0.0

        # 측정
        self.s_hit_idx = None
        self.s_hit_xy = None
        self.brake_t0 = 0.0
        self.brake_v0 = float('nan')
        self.brake_xy = None
        self._still_t = 0.0
        self._done_t = 0.0
        self._reported = False

        if not self.load_route():
            #  ★경로가 없거나 S 가 없으면 아예 시작하지 않는다★ 굴린 뒤에 알면 늦다.
            self.state = S_DONE
            self._done_t = time.time()
            self._reported = True

        self.create_timer(1.0 / CONTROL_HZ, self.loop)
        self.event(f"🅑 브레이크 시험 준비 — 지령 {self.cmd_kind}, "
                   f"경로 {self.route_name or '(없음)'}")

    # ══════════════════════════════════════════════════════════════════════════
    #  경로
    # ══════════════════════════════════════════════════════════════════════════
    def load_route(self):
        """route 파라미터가 가리키는 CSV 를 읽는다. 비었으면 ★최신★ route_*.csv.

        ★S 가 없으면 거부한다★ 이 시험은 S 에서만 제동을 건다 — 없으면 차는
        경로 끝까지 10펄스로 달리고 아무도 세우지 않는다. 그건 사고다.
        """
        import csv as _csv
        name = str(self.get_parameter('route').value or '').strip()
        try:
            names = sorted(f for f in os.listdir(self.data_dir)
                           if f.startswith('route_') and f.endswith('.csv'))
        except OSError:
            names = []
        if not name:
            if not names:
                self.event(f"❌ {self.data_dir} 에 route_*.csv 가 없다 — 먼저 매핑할 것")
                return False
            name = names[-1]
            self.event(f"ℹ️ route 를 지정하지 않아 ★최신 경로★ 를 쓴다: {name}")
        path = os.path.join(self.data_dir, name)
        if not os.path.isfile(path):
            self.event(f"❌ 경로 파일 없음: {path}")
            return False

        wps, zone = [], []
        try:
            with open(path, 'r', encoding='utf-8') as f:
                for row in _csv.DictReader(f):
                    try:
                        wps.append((float(row['latitude']), float(row['longitude'])))
                    except (KeyError, ValueError, TypeError):
                        continue
                    zone.append(str(row.get('terrain', '') or '').strip())
        except Exception as e:            # noqa: BLE001
            self.event(f"❌ 경로 읽기 실패: {e}")
            return False
        if len(wps) < 2:
            self.event(f"❌ 웨이포인트 부족({len(wps)}개): {name}")
            return False

        self.raw_wps, self.raw_zone, self.route_name = wps, zone, name
        self.stop_idx = [i for i, z in enumerate(zone) if z in STOP_ZONE_CHARS]
        if not self.stop_idx:
            self.event(
                f"❌ {name} 의 terrain 열에 ★S 가 없다★ — 제동을 걸 지점이 없으므로 "
                f"시작하지 않는다. 제동을 시작할 행의 terrain 을 'S' 로 고칠 것 "
                f"(경로 {len(wps)}점 / S 뒤로 30m 이상 여유가 있어야 한다)")
            return False

        # 직선성 · S 뒤 여유거리를 미리 말해 준다 (굴리기 전에 알아야 하는 값이다)
        self.event(f"📁 경로 {name} — WP {len(wps)}개, S {len(self.stop_idx)}곳 "
                   f"(WP {', '.join(str(i) for i in self.stop_idx)})")
        return True

    def build_waypoints(self):
        if self.lat0 is None or not getattr(self, 'raw_wps', None):
            return False
        self.waypoints = [latlon_to_xy(la, lo, self.lat0, self.lon0)
                          for (la, lo) in self.raw_wps]
        self.wp_zone = list(self.raw_zone[:len(self.waypoints)])
        self.wp_zone += [''] * (len(self.waypoints) - len(self.wp_zone))
        # ★S 뒤 남은 거리를 재서 경고한다★ 헤더 안전절의 그 값이다.
        first_s = self.stop_idx[0]
        run_out = 0.0
        for i in range(first_s, len(self.waypoints) - 1):
            run_out += math.dist(self.waypoints[i], self.waypoints[i + 1])
        need = 30.0
        msg = (f"📏 첫 S(WP {first_s}) 뒤 경로 잔여 {run_out:.1f}m")
        if run_out < need:
            msg += (f" — ⚠️★{need:.0f}m 미만이다★ 10펄스면 S 뒤로 13~20m 를 더 간다. "
                    f"경로가 모자라면 차는 경로 밖에서 선다. 그래도 진행한다")
        self.event(msg)
        self.wp_idx = 0
        self._wp_prev = 0
        return True

    # ══════════════════════════════════════════════════════════════════════════
    #  구독 콜백
    # ══════════════════════════════════════════════════════════════════════════
    def cb_gps(self, msg: Float64MultiArray):
        d = list(msg.data)
        if len(d) < 9:
            return
        lat, lon = d[0], d[1]
        if not (math.isfinite(lat) and math.isfinite(lon)):
            return
        self.gps_quality = int(d[5]) if math.isfinite(d[5]) else Q_NONE
        self.gps_sigma = d[6]
        self.fix_ok = bool(d[7] > 0.5) if len(d) > 7 else True
        v = d[8]
        self.gps_kmh = float(v) if math.isfinite(v) else None
        if self.lat0 is None:
            self.lat0, self.lon0 = lat, lon
        self.x, self.y = latlon_to_xy(lat, lon, self.lat0, self.lon0)
        self.fix_time = time.time()
        if self.state == S_HEADING and self.fix_ok:
            self.head_est.add(self.x, self.y)

    def cb_imu(self, msg: Imu):
        now = time.time()
        if self.imu_time > 0.0 and self.heading is not None:
            dt = now - self.imu_time
            if 0.0 < dt < 0.5:
                self.heading = wrap180(self.heading + math.degrees(self.gyro_z) * dt)
        #  ★직선 시험이라 자이로 축 보정을 두지 않는다★ 이 노드는 사실상 직진만
        #  하므로 요 적분의 누적오차가 문제 되는 구간이 없다(driving.py 는 U턴에서
        #  40° 를 잃어 중력축 투영이 필요했다 — 거기는 그 코드가 들어가 있다).
        self.gyro_z = float(msg.angular_velocity.z)
        self.imu_time = now

    def cb_encoder(self, msg: Int32):
        self._enc_buf.append(float(msg.data))
        if len(self._enc_buf) > ENC_MEDIAN_N:
            del self._enc_buf[0]
        med = sorted(self._enc_buf)[len(self._enc_buf) // 2]
        self.enc_pulse = med * ENC_SUM_TO_PULSE

    def cb_mode(self, msg: Bool):
        self.auto_mode = bool(msg.data)

    def cb_estop(self, msg: Bool):
        self.estop = bool(msg.data)

    def cb_go(self, msg: Bool):
        if bool(msg.data):
            self.go = True

    # ══════════════════════════════════════════════════════════════════════════
    #  출력
    # ══════════════════════════════════════════════════════════════════════════
    def event(self, text):
        self.get_logger().info(text)
        self.pub_event.publish(String(data=text))

    def send(self, value, steer_deg, control):
        """★값을 그대로 낸다★ 펄스든 직접 PWM 이든 arduino 가 대역으로 가른다.

        ★제동 중에도 이 값을 0 으로 덮지 않는다★ arduino 가 브레이크>0 이면 A보드
        REF 를 0 으로 만들기 때문에(compose (4)) 우리가 또 0 을 낼 이유가 없고,
        내면 해제 순간 0→목표로 두 번 전이한다(driving.corner_brake 와 같은 이유).
        """
        msg = Twist()
        msg.linear.x = float(value)
        msg.angular.z = float(steer_deg)
        self.pub_cmd.publish(msg)
        self.pub_state.publish(Bool(data=bool(control)))
        self._last_steer = float(steer_deg)

    def set_brake(self, level):
        if level != self.brake_now:
            self.brake_now = level
            self.publish_brake(force=True)

    def publish_brake(self, force=False):
        """★0 은 재확인하지 않는다★ '놓음' 을 계속 주장하면 남의 정지를 푼다."""
        now = time.time()
        if not force and self.brake_now == self._brake_out:
            if self.brake_now <= 0 or (now - self._brake_t) < BRAKE_KEEPALIVE_S:
                return
        self._brake_out = self.brake_now
        self._brake_t = now
        self.pub_brake.publish(Int32(data=int(self.brake_now)))

    def publish_state(self):
        #  record 가 켜지도록 DRIVE_* 이름을 쓴다(헤더 참고).
        name = {S_WAIT: 'IDLE', S_HEADING: 'DRIVE_HEADING', S_RUN: 'DRIVE_RUN',
                S_BRAKE: 'DRIVE_RUN', S_DONE: 'DRIVE_DONE'}[self.state]
        self.pub_dstate.publish(String(data=name))

    def enter(self, new_state, msg=''):
        self.state = new_state
        self.state_t0 = time.time()
        if msg:
            self.event(msg)

    # ══════════════════════════════════════════════════════════════════════════
    #  기하
    # ══════════════════════════════════════════════════════════════════════════
    def advance_wp(self):
        """진행 포인터를 창 안 최근접점으로. driving.advance_wp_idx 와 같은 규칙."""
        n = len(self.waypoints)
        hi = min(self.wp_idx + WP_SEARCH_WINDOW, n)
        ch = math.cos(math.radians(self.heading))
        sh = math.sin(math.radians(self.heading))
        best_any, d_any = self.wp_idx, float('inf')
        best_ahead, d_ahead = None, float('inf')
        for i in range(self.wp_idx, hi):
            wx, wy = self.waypoints[i]
            dx, dy = wx - self.x, wy - self.y
            d = math.hypot(dx, dy)
            if d < d_any:
                d_any, best_any = d, i
            if dx * ch + dy * sh > -WP_AHEAD_MARGIN_M and d < d_ahead:
                d_ahead, best_ahead = d, i
        best = best_any
        if best_ahead is not None and d_ahead <= d_any + WP_AHEAD_PENALTY_M:
            best = best_ahead
        self.wp_idx = max(self.wp_idx, min(best, self.wp_idx + WP_MAX_ADVANCE))

    def lookahead_m(self):
        """LFD = v·√2/ω_n. ★속도에 비례한다★ (driving.lookahead_m 과 같은 설계식)"""
        v = self.speed_ms()
        if v is None or not math.isfinite(v):
            v = self.nominal_ms if math.isfinite(self.nominal_ms) else 3.0
        lfd = v * math.sqrt(2.0) / LFD_OMEGA_N
        return max(LFD_MIN_M, min(LFD_MAX_M, lfd))

    def pure_pursuit(self, lfd):
        """목표점 = wp_idx 이후 LFD 이상 떨어진 첫 앞쪽 WP → 도로휠각 [deg, +좌]."""
        n = len(self.waypoints)
        ch = math.cos(math.radians(self.heading))
        sh = math.sin(math.radians(self.heading))
        tgt = None
        for i in range(self.wp_idx, n):
            wx, wy = self.waypoints[i]
            dx, dy = wx - self.x, wy - self.y
            if dx * ch + dy * sh <= 0.0:
                continue
            if math.hypot(dx, dy) >= lfd:
                tgt = (dx, dy)
                break
        if tgt is None:
            wx, wy = self.waypoints[-1]
            tgt = (wx - self.x, wy - self.y)
        dx, dy = tgt
        d = math.hypot(dx, dy)
        if d < 1e-3:
            return 0.0
        # 차체기준 방위 (왼쪽 +)
        alpha = math.atan2(-dx * sh + dy * ch, dx * ch + dy * sh)
        return math.degrees(math.atan2(2.0 * WHEELBASE_M * math.sin(alpha), d))

    def steer_command(self, road_deg, v_ms):
        """도로휠각 → B보드 pot 지령. ★부호가 여기서 한 번만 뒤집힌다 (− 좌 / + 우)★"""
        d = abs(road_deg)
        if d < 1e-6:
            return 0.0
        v = v_ms if (v_ms is not None and math.isfinite(v_ms)) else 0.0
        pot = (STEER_PLANT_GAIN * d
               + STEER_UNDERSTEER * v * v * math.tan(math.radians(d)) / WHEELBASE_M)
        pot = min(STEER_MAX_DEG, pot)
        return -math.copysign(pot, road_deg)

    def signed_cte(self):
        """경로에서 벗어난 측방거리 [m]. + = 차가 경로 왼쪽."""
        n = len(self.waypoints)
        if n < 2:
            return float('nan')
        lo = max(0, self.wp_idx - CTE_WINDOW_WP)
        hi = min(n - 1, self.wp_idx + CTE_WINDOW_WP)
        best, bi = float('inf'), lo
        for i in range(lo, hi + 1):
            d = math.dist(self.waypoints[i], (self.x, self.y))
            if d < best:
                best, bi = d, i
        j = min(bi + 1, n - 1)
        k = max(bi - 1, 0)
        ax, ay = self.waypoints[k]
        bx, by = self.waypoints[j]
        seg = math.hypot(bx - ax, by - ay)
        if seg < 1e-6:
            return float('nan')
        ux, uy = (bx - ax) / seg, (by - ay) / seg
        return -(self.x - ax) * uy + (self.y - ay) * ux

    def speed_ms(self):
        return None if self.gps_kmh is None else self.gps_kmh / 3.6

    def stopped(self, now):
        """★GPS 와 엔코더를 둘 다 요구한다★ (헤더 '엔코더로 재지 않는 이유')"""
        v = self.gps_kmh
        slow = (v is not None and v <= STOP_KMH_EPS) and self.enc_pulse <= STOP_ENC_EPS
        if not slow:
            self._still_t = 0.0
            return False
        if self._still_t == 0.0:
            self._still_t = now
            return False
        return (now - self._still_t) >= STOP_HOLD_S

    # ══════════════════════════════════════════════════════════════════════════
    #  제어 루프
    # ══════════════════════════════════════════════════════════════════════════
    def loop(self):
        self.publish_state()
        self.publish_brake()          # 물고 있는 동안 재확인 (발행자가 여럿이다)
        now = time.time()

        if self.state == S_DONE:
            self.send(0, self._last_steer, control=True)
            if not self._reported:
                return
            if self._done_t and (now - self._done_t) >= DONE_LINGER_S:
                #  ★런치를 내리는 신호★ braketest.launch.py 가 이 노드의 종료를
                #  받아 전체를 내린다(OnProcessExit → Shutdown).
                self.pub_done.publish(Bool(data=True))
                self.event("🛑 브레이크 시험 종료 — 런치를 내린다")
                raise SystemExit(0)
            return

        # ── 안전 게이트 : 어느 상태에서든 먼저 본다 ──
        if self.estop:
            self.send(0, self._last_steer, control=True)
            self.throttle("🚨 E-STOP 체결 중 — 해제하면 대기부터 다시 시작한다")
            if self.state in (S_RUN, S_BRAKE):
                self.finish("E-STOP 으로 중단")
            return
        if self.auto_mode is not True:
            self.send(0, self._last_steer, control=False)
            self.throttle("⏸️ D5 가 자율주행이 아니다 — 스위치를 올리면 시작한다"
                          if self.auto_mode is False else
                          "⏸️ /vehicle_mode 미수신 — nxde arduino 가 떠 있는지 확인")
            if self.state in (S_RUN, S_BRAKE):
                self.finish("수동조종 전환으로 중단")
            return
        if now - self.fix_time > GPS_TIMEOUT_S:
            self.send(0, self._last_steer, control=True)
            self.throttle(f"⚠️ GPS 두절 — 원시 fix {now - self.fix_time:.1f}s 없음")
            if self.state in (S_RUN, S_BRAKE):
                self.finish("GPS 두절로 중단")
            return

        if self.state == S_WAIT:
            self.run_wait(now)
        elif self.state == S_HEADING:
            self.run_heading(now)
        elif self.state == S_RUN:
            self.run_follow(now)
        elif self.state == S_BRAKE:
            self.run_brake(now)

    def throttle(self, text, period=2.0):
        t = time.time()
        if t - getattr(self, '_thr_t', 0.0) >= period:
            self._thr_t = t
            self.event(text)

    def run_wait(self, now):
        self.send(0, 0.0, control=True)
        if not (self.auto_start or self.go):
            self.throttle("⏸️ 출발 대기 — `ros2 topic pub -1 /braketest_go "
                          "std_msgs/Bool '{data: true}'` 로 시작")
            return
        if not self.fix_ok:
            self.throttle(f"⏸️ GPS 품질 대기 — {Q_LABEL.get(self.gps_quality, '?')}"
                          f"(σ={self.gps_sigma:.2f}m)")
            return
        if not self.build_waypoints():
            self.throttle("⏸️ GPS 원점 대기")
            return
        self.head_est.reset()
        self.heading = None
        self.enter(S_HEADING,
                   f"▶ 출발 — 헤딩 초기화({self.heading_pulse}펄스로 곧게). "
                   f"확정되면 {self.cmd_kind} 로 가속한다")

    def run_heading(self, now):
        #  ★진입 직후 잠깐은 굴리지 않는다★ (driving.py 와 같은 이유)
        if now - self.state_t0 < MODE_SETTLE_S:
            self.send(0, 0.0, control=True)
            return
        self.send(self.heading_pulse, 0.0, control=True)
        sol = self.head_est.solve()
        if sol is None:
            return
        heading, sigma, resid, n, dist = sol
        if dist < HEAD_MIN_DIST_M:
            return
        good = (n >= HEAD_MIN_SAMPLES and sigma <= HEAD_TARGET_SIGMA
                and resid <= HEAD_MAX_RESID_M)
        if not (good or dist >= HEAD_MAX_DIST_M):
            return
        self.heading = heading
        # 가장 가까운 WP 로 포인터를 맞춘다 (출발점이 경로 중간일 수 있다)
        best = min(range(len(self.waypoints)),
                   key=lambda i: math.dist(self.waypoints[i], (self.x, self.y)))
        self.wp_idx = self._wp_prev = best
        mark = "" if good else "  ⚠️(최대거리 도달 — 정확도 미달)"
        self.enter(S_RUN,
                   f"🧭 헤딩 확정 {heading:+.1f}° (±{sigma:.1f}°, {dist:.2f}m, "
                   f"{n}점){mark} — WP {best}/{len(self.waypoints)} 에서 "
                   f"★{self.cmd_kind}★ 로 주행 시작")

    def run_follow(self, now):
        if self.heading is None or not self.waypoints:
            self.send(0, 0.0, control=True)
            return
        prev = self._wp_prev
        self.advance_wp()
        self._wp_prev = self.wp_idx

        # ── S 통과 판정 ── ★'같은가' 가 아니라 지나온 구간을 훑는다★
        #    10펄스면 한 틱에 0.44m — WP 간격 0.25m 면 두 칸씩 건너뛴다.
        #    단일 행을 '같은가' 로 보면 그 틱에 조용히 사라진다.
        hit = None
        for i in range(prev + 1, self.wp_idx + 1):
            if 0 <= i < len(self.wp_zone) and self.wp_zone[i] in STOP_ZONE_CHARS:
                hit = i
                break
        if hit is not None:
            self.begin_brake(hit, now)
            return

        # ── 경로이탈 ──
        cte = self.signed_cte()
        if math.isfinite(cte) and abs(cte) > self.cte_abort:
            self.set_brake(BRAKE_FULL)
            self.publish_brake(force=True)
            self.send(0, self._last_steer, control=True)
            self.finish(f"경로이탈 {cte:+.2f}m (한계 {self.cte_abort:.1f}m)")
            return

        # ── 경로 끝 ──
        if self.wp_idx >= len(self.waypoints) - 1:
            self.set_brake(BRAKE_FULL)
            self.publish_brake(force=True)
            self.send(0, self._last_steer, control=True)
            self.finish("★S 를 만나기 전에 경로가 끝났다★ — terrain 열 확인")
            return

        lfd = self.lookahead_m()
        road = self.pure_pursuit(lfd)
        v = self.speed_ms()
        steer = self.steer_command(road, v if v is not None else self.nominal_ms)
        self.send(self.cmd_value, steer, control=True)
        self.throttle(
            f"🅑 주행 중 — WP {self.wp_idx}/{len(self.waypoints)}, "
            f"{'?' if self.gps_kmh is None else f'{self.gps_kmh:.1f}'}km/h, "
            f"CTE {cte:+.2f}m, LFD {lfd:.1f}m", period=1.0)

    def begin_brake(self, idx, now):
        """S 통과 — ★즉시 리니어 2단★. 여기서부터가 측정 구간이다."""
        self.s_hit_idx = idx
        self.s_hit_xy = (self.x, self.y)
        self.brake_xy = (self.x, self.y)
        self.brake_t0 = now
        self.brake_v0 = self.speed_ms() if self.speed_ms() is not None else float('nan')
        self._still_t = 0.0
        self.set_brake(BRAKE_FULL)
        self.publish_brake(force=True)
        #  ★조향은 유지한다★ 정지 직전에 앞바퀴를 정면으로 꺾으면 제동 중 거동이
        #  바뀌어 측정이 오염된다. 펄스는 arduino 가 0 으로 덮는다(send docstring).
        self.send(self.cmd_value, self._last_steer, control=True)
        v0kmh = (float('nan') if not math.isfinite(self.brake_v0)
                 else self.brake_v0 * 3.6)
        self.enter(S_BRAKE,
                   f"🛑 S 통과(WP {idx}) — ★리니어 2단 체결★ "
                   f"진입속도 {v0kmh:.2f} km/h ({self.brake_v0:.3f} m/s)")

    def run_brake(self, now):
        self.send(self.cmd_value, self._last_steer, control=True)
        held = now - self.brake_t0
        if self.stopped(now):
            self.report(now, held)
            return
        if held >= BRAKE_MAX_S:
            self.report(now, held, note="★시간 초과★ 완전정지를 확인하지 못했다")
            return
        self.throttle(
            f"🛑 제동 중 {held:.2f}s — "
            f"{'?' if self.gps_kmh is None else f'{self.gps_kmh:.2f}'}km/h, "
            f"ENC {self.enc_pulse:.1f}펄스", period=0.5)

    # ══════════════════════════════════════════════════════════════════════════
    #  결과
    # ══════════════════════════════════════════════════════════════════════════
    def report(self, now, held, note=''):
        d = (math.dist(self.brake_xy, (self.x, self.y))
             if self.brake_xy else float('nan'))
        s_over = (math.dist(self.s_hit_xy, (self.x, self.y))
                  if self.s_hit_xy else float('nan'))
        v0 = self.brake_v0
        #  ★두 가지 방법으로 낸다★ 시간식과 에너지식이 크게 다르면 어딘가 틀렸다
        #  (대개 '완전정지' 판정 시각이 늦은 것이다 — 그때 시간식만 작아진다).
        a_t = v0 / held if (math.isfinite(v0) and held > 0.05) else float('nan')
        a_d = (v0 * v0 / (2.0 * d)) if (math.isfinite(v0) and d > 0.05) else float('nan')
        lines = [
            "═" * 66,
            " 브레이크 제동거리 측정 결과",
            "═" * 66,
            f"  경로            {self.route_name}",
            f"  지령            {self.cmd_kind}",
            f"  S 지점          WP {self.s_hit_idx}",
            f"  체결 시 속도 v0 {v0:.3f} m/s ({v0 * 3.6:.2f} km/h)",
            f"  제동시간 t      {held:.2f} s",
            f"  제동거리 d      {d:.2f} m        ← 체결 지점부터 잰 GPS 직선거리",
            f"  S 초과거리      {s_over:.2f} m        ← S 를 이만큼 지나서 섰다",
            f"  평균 감속도     {a_t:.2f} m/s²  (v0/t)",
            f"                  {a_d:.2f} m/s²  (v0²/2d)",
            "═" * 66,
            "  ※ BRAKING.md 실측 : 2단 2.2~3.8 m/s² / 1단 0.62~1.05 / 코스트 0.29~0.54",
        ]
        if note:
            lines.append(f"  ⚠️ {note}")
        for ln in lines:
            self.get_logger().info(ln)
        self.pub_event.publish(String(data=(
            f"🅑 제동 결과 — v0 {v0 * 3.6:.2f}km/h, d {d:.2f}m, t {held:.2f}s, "
            f"a {a_t:.2f}/{a_d:.2f} m/s², S 초과 {s_over:.2f}m{'  ' + note if note else ''}")))
        self._reported = True
        self._done_t = now
        self.enter(S_DONE)

    def finish(self, why):
        """측정 없이 끝낸다(중단·이상). ★리니어는 물린 채로 두지 않는다★"""
        self.set_brake(BRAKE_FULL)
        self.publish_brake(force=True)
        self._reported = True
        self._done_t = time.time()
        self.enter(S_DONE, f"⛔ 시험 중단 — {why}. 리니어 2단")

    def destroy_node(self):
        try:
            self.send(0, self._last_steer, control=True)
            self.pub_brake.publish(Int32(data=BRAKE_NONE))
        except Exception:            # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BrakeTestNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit,
            rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:            # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
