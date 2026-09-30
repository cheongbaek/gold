#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
follow_sim.py ― driving.py GPS 추종의 ★폐루프 모의실험 하네스★  [2026-09-29]
════════════════════════════════════════════════════════════════════════════════
CHANGELOG.md 의 'TO DO LIST — TODO-1(흔들림 감소 + 10펄스 증속)' 의 수치는 전부
이 파일과 follow_design.py 로 다시 낼 수 있다.

★제어기는 흉내 내지 않는다 — driving.py 의 DrivingNode 를 그대로 만든다★
  loop() → update_trim_estimate / apply_trim / run_follow(advance_wp_idx ·
  lookahead_m · corner_speed · goal_approach · corner_brake · pure_pursuit_steer ·
  apply_cte_integral · steer_command) 가 실차와 한 글자도 다르지 않다.
  · 시간 : 모듈의 `time` 을 가짜 시계로 바꾼다 (driving 은 전부 time.time() 을 쓴다)
  · 입력 : /gps_fused(20 Hz = 5 Hz 원시 + DR) · /imu · /encoder · /steer_angle_measured
           를 ★콜백에 직접★ 넣는다. ROS 통신은 쓰지 않는다
  · 출력 : pub_cmd / pub_brake 를 가로채 arduino.py 규칙(정수 반올림 · 제동 중 펄스 0)을 태운다
  · 격리 : ROS_DOMAIN_ID=77 을 ★import 전에★ 강제한다 — 이 기계에서 실차 스택(도메인 7)이
           돌고 있어도 토픽이 섞이지 않는다. (ROS_LOCALHOST_ONLY=1 은 쓰지 않는다 — CycloneDDS 가
           참가자 번호를 모자라게 잡아 병렬 실행이 'free participant index' 오류로 죽는다)

★라이다 구간은 뺀다 (사용자 지시)★ 경로 CSV 의 terrain 을 전부 '0' 으로 지운 사본을
  임시 폴더에 만들어 쓴다 — L(라이다)·S(일시정지)·T(신호등) 구간이 없다.
  원본 gps_data/ 는 건드리지 않는다.

차량 모델 (전부 9/13 로그 식별값 — CHANGELOG TODO-1 '모의실험' 절)
  · 뒷차축 기준 자전거 모델, 축거 1.25 m, GPS 안테나 = 앞차축 위(뒷차축 앞 ant_x = 1.25 m)
  · 조향 'backlash' : 정수 지령 → 지연 T → 백래시(반폭 b) → 1차 지연 τ → 도로휠 = (pot − trim)/G
    조향 'phys'     : kasa_0909_B.ino updateSteer 모사 (허용오차 3/6 카운트 · 최소 PWM 110 ·
                      정지마찰 pwm0 · 9점 중앙값 + EMA 0.3 · 새 지령/1 s keepalive 마다 ACTIVE)
  · 속도 : A보드 = 지령펄스 × 0.884 로 1차 추종(가속 상한 1.0, 코스트 0.41 m/s²), 1단 +0.9 · 2단 +3.0

실행 (ROS 환경만 source 하면 된다 — 워크스페이스 빌드는 필요 없다):
    source /opt/ros/humble/setup.bash
    python3 follow_design.py compare --cands 현행,A+B+C+D+E --pulses 7,10
"""
import csv
import math
import os
import sys
import tempfile
import time as _real_time

# ★rclpy 를 import 하기 전에★ 도메인을 격리한다 (실차 스택은 도메인 7)
os.environ['ROS_DOMAIN_ID'] = os.environ.get('WHITE1_SIM_DOMAIN', '77')

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.dirname(HERE)                    # gold_ws/src/white1 (소스트리)
sys.path.insert(0, PKG_ROOT)

import numpy as np                                   # noqa: E402
import rclpy                                         # noqa: E402
import rclpy.node                                    # noqa: E402
from sensor_msgs.msg import Imu                      # noqa: E402
from std_msgs.msg import Float64MultiArray, Int32   # noqa: E402

import white1.driving as drv                         # noqa: E402

GPS_DIR = os.path.join(PKG_ROOT, 'gps_data')
BAG_DIR = os.path.join(PKG_ROOT, 'ros2bag')
ROUTE_TMP = os.path.join(tempfile.gettempdir(), 'white1_follow_sim_routes')
MS_PER_PULSE = drv.MS_PER_PULSE
EARTH_R = drv.EARTH_R

# 모의에 쓴 경로 셋 (전부 저장소에 추적돼 있다)
ROUTES = {
    'A': 'route_20200101_111113.csv',    # 745 m 굽은 코스 (9/13 155621·160154)
    'B': 'maincourse.csv',               # 743 m 긴 직선 + 코너 (9/13 163010) — 옛 route_20260913_161721
    'C': 'route_20200202_222200.csv',    # 568 m 굽은 코스 (9/13 170016)
}
# 9/13 최신 주행 4개 (record CSV, 경로)
LOGS = {
    '155621': ('route_20200101_111113-20260913_155621.csv', ROUTES['A']),
    '160154': ('route_20200101_111113-20260913_160154.csv', ROUTES['A']),
    '163010': ('route_20260913_161721-20260913_163010.csv', ROUTES['B']),
    '170016': ('route_20200202_222200-20260913_170016.csv', ROUTES['C']),
}


# ══════════════════════════════════════════════════════════════════════════════
#  가짜 시계
# ══════════════════════════════════════════════════════════════════════════════
class FakeTime:
    def __init__(self):
        self.t = 1.8e9

    def time(self):
        return self.t

    def __getattr__(self, k):
        return getattr(_real_time, k)


CLOCK = FakeTime()
drv.time = CLOCK


def prepare_route(name):
    """terrain 을 전부 '0' 으로 지운 사본 — 라이다(L)·일시정지(S)·신호등(T) 없음."""
    os.makedirs(ROUTE_TMP, exist_ok=True)
    src = os.path.join(GPS_DIR, name)
    dst = os.path.join(ROUTE_TMP, name)
    rows = list(csv.DictReader(open(src, encoding='utf-8-sig')))
    fields = list(rows[0].keys())
    if 'terrain' not in fields:
        fields.append('terrain')
    # 원본이 바뀌었으면 다시 만든다 (낡은 사본으로 조용히 돌지 않게)
    if not os.path.isfile(dst) or os.path.getmtime(src) > os.path.getmtime(dst):
        tmp = f'{dst}.{os.getpid()}.tmp'
        with open(tmp, 'w', encoding='utf-8', newline='') as f:
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            for r in rows:
                r['terrain'] = '0'
                w.writerow(r)
        os.replace(tmp, dst)      # 여러 프로세스가 동시에 써도 반쯤 쓴 파일을 읽지 않는다
    lat = np.array([float(r['latitude']) for r in rows])
    lon = np.array([float(r['longitude']) for r in rows])
    return lat, lon


def xy_to_latlon(x, y, lat0, lon0):
    lat = lat0 + math.degrees(y / EARTH_R)
    lon = lon0 + math.degrees(x / (EARTH_R * math.cos(math.radians(lat0))))
    return lat, lon


def rha(x):
    """arduino.py _round_half_away 와 같은 반올림."""
    return int(x + 0.5) if x >= 0 else -int(-x + 0.5)


class _Capture:
    def __init__(self):
        self.last = None

    def publish(self, msg):
        self.last = msg


class _Null:
    def publish(self, msg):
        pass


# ══════════════════════════════════════════════════════════════════════════════
#  차량 모델 기본값 — ★9/13 로그 식별값★ (CHANGELOG TODO-1 '모의실험' 절)
# ══════════════════════════════════════════════════════════════════════════════
DEFAULT_PLANT = dict(
    G=1.60,            # pot/도로휠 전달비 — 식별 1.50~1.77 (CLAUDE.md 4.1b 큰각 1.42~1.52)
    trim_true=-2.1,    # 직진을 만드는 pot — 식별 −1.95~−2.24
    b=1.0,             # 조향 유효 백래시 반폭 [pot deg] — 식별 0.8~1.2
    tau=0.15,          # 조향 1차 지연 [s] — 식별 0.05~0.25
    T=0.20,            # 지령 → 조향 순수지연 [s] — 식별 0.1~0.3
    slew=70.0,         # [pot deg/s] (CLAUDE.md 1.3)
    L=1.25,
    ant_x=1.25,        # GPS 안테나 = 뒷차축 앞 [m] — 앞차축 위 = 축거. 로그의 코너 안쪽 편향
                       #   +0.19~0.30 이 이 값에서 재현된다(0.6 이면 +0.09~0.13)
    gps_sigma=0.012,   # RTK Fixed 잡음 [m]
    gps_lat=0.10,      # GPS 위치 지연 [s]
    gyro_sigma=1.9,    # [deg/s] 20 Hz 백색잡음 — 9/13 로그 실측 1.8~2.0
    gyro_bias=0.03,    # [deg/s]
    k_over=1.00,       # A보드 정상상태 속도 / 지령 (9/13 braketest 10펄스에서 1.1)
    acc=1.0, tau_v=1.0, coast=0.41,
    us_k=0.0,          # 고속 언더스티어 : G_eff = G·(1 + us_k·v²) (실측 안 됨 — 강건성용)
    int_cmd=True,      # arduino 정수 반올림 (B보드는 atoi 로 정수 도만 받는다)
    servo='backlash',  # 'backlash' | 'phys'
    pwm0=90.0,         # phys : 이 PWM 밑에서는 조향 모터가 안 돈다 (80~95 가 로그를 재현)
    tau_m=0.06, tau_c=0.05, pot_noise=0.7,
    seed=1,
)


class Sim:
    """경로 하나를 drive_pulse 로 끝까지(또는 t_max) 달린다. run() 뒤 self.tr 에 궤적.

    params : driving 노드 ROS 파라미터 (예: {'lfd_omega_n_fast': 0.9})
    plant  : DEFAULT_PLANT 덮어쓰기
    mods   : 노드를 받아 메서드를 감싸는 함수들 (follow_design.py 의 설계 후보)
    """

    def __init__(self, route, drive_pulse=7, params=None, plant=None, mods=(), start_idx=8,
                 t_max=400.0):
        self.p = dict(DEFAULT_PLANT)
        if plant:
            self.p.update(plant)
        self.rng = np.random.default_rng(self.p['seed'])
        self.t0 = CLOCK.t
        self.gps_kmh = 3 * MS_PER_PULSE * 3.6
        lat, lon = prepare_route(route)
        args = ['--ros-args', '--log-level', 'error',
                '-p', f'data_dir:={ROUTE_TMP}', '-p', f'drive_pulse:={int(drive_pulse)}']
        for k, v in (params or {}).items():
            args += ['-p', f'{k}:={v}']
        rclpy.init(args=args)
        n = drv.DrivingNode()
        self.node = n
        n.pub_cmd = _Capture()
        n.pub_brake = _Capture()
        for name in ('pub_state', 'pub_map', 'pub_dstate', 'pub_tl_permit', 'pub_lstatus',
                     'pub_lidar_ref', 'pub_event', 'pub_ego', 'pub_diag'):
            setattr(n, name, _Null())
        self.events = []
        n.event = lambda text: self.events.append((CLOCK.t - self.t0, text))
        n.throttle_event = n.event
        n.throttle_warn = lambda text, period=10.0: None
        for m in mods:
            m(n)
        self.lat0, self.lon0 = lat[0], lon[0]
        n.lat0, n.lon0 = self.lat0, self.lon0
        n.select_route(route)
        if not n.build_waypoints():
            raise RuntimeError('build_waypoints 실패')
        wps = np.array(n.waypoints)
        self.wps = wps
        # 출발 자세 : 헤딩 초기화를 끝낸 직후처럼 — start_idx 에서 경로 방위, 3펄스
        i = start_idx
        hd = math.atan2(wps[i + 3, 1] - wps[i, 1], wps[i + 3, 0] - wps[i, 0])
        self.psi = hd
        self.x = wps[i, 0] - self.p['ant_x'] * math.cos(hd)
        self.y = wps[i, 1] - self.p['ant_x'] * math.sin(hd)
        self.v = 3 * MS_PER_PULSE
        self.delta = 0.0          # 도로휠 [rad] (+좌)
        self.pot = self.p['trim_true']
        self.x_bl = self.pot
        self.cmd_hist = []        # (t, 정수 pot) — 지연 T
        # phys 서보 상태 (raw 카운트, 3.375 카운트/도 = 270 카운트 / 80°)
        self.cnt = self.pot * 3.375
        self.om = 0.0
        self.filt = self.cnt
        self.prev_cur = int(self.filt)
        self.active = False
        self.settle_t = None
        self.last_rx = None
        self.last_rx_t = -99.0
        self.b_t = 0.0
        self.motor = 0.0
        self.pos_hist = []
        self.v_hist = []
        self.brake_level = 0
        self.brake_t = -99.0
        self.cmd_pulse = 3
        self.cmd_angle = rha(self.pot)
        self.cmd_float = self.pot
        self.t_max = t_max
        n.heading = math.degrees(self.psi)
        n.imu_up = (0.0, 0.0, 1.0)     # 중력축 투영은 이미 끝났다고 본다
        n.auto_mode = True
        n.estop = False
        n.reset_trim()
        n.reset_cte_integral()
        n.wp_idx = i
        n._wp_idx_prev = i
        n.enter(drv.S_DRIVE_RUN)
        self.log = []

    # ── 센서 ─────────────────────────────────────────────────────────────
    def ant(self):
        return (self.x + self.p['ant_x'] * math.cos(self.psi),
                self.y + self.p['ant_x'] * math.sin(self.psi))

    def feed_sensors(self, k):
        n = self.node
        t = CLOCK.t
        ax, ay = self.ant()
        self.pos_hist.append((t, ax, ay))
        self.v_hist.append((t, self.v))
        while self.pos_hist and self.pos_hist[0][0] < t - 2.0:
            self.pos_hist.pop(0)
        while self.v_hist and self.v_hist[0][0] < t - 2.0:
            self.v_hist.pop(0)
        tl = t - self.p['gps_lat']
        px, py = ax, ay
        for (tt, xx, yy) in self.pos_hist:
            if tt >= tl:
                px, py = xx, yy
                break
        is_raw = (k % 4 == 0)                       # 5 Hz 원시, 사이는 DR
        s = self.p['gps_sigma'] * (1.0 if is_raw else 0.5)
        px += self.rng.normal(0, s)
        py += self.rng.normal(0, s)
        if is_raw:
            # gps.py [8] : 원시 fix 3점(0.4 s 창) 양끝 변위 속도
            vv = [vv for (tt, vv) in self.v_hist if tl - 0.4 <= tt <= tl]
            self.gps_kmh = (np.mean(vv) if vv else self.v) * 3.6 + self.rng.normal(0, 0.15)
        lat, lon = xy_to_latlon(px, py, self.lat0, self.lon0)
        d = [0.0] * drv.GPS_FUSED_FIELDS
        d[0], d[1] = lat, lon
        d[2], d[3], d[4] = float(drv.Q_FIXED), 0.02, 1.0
        d[5] = 1.0 if is_raw else 0.0
        d[6] = self.p['gps_lat'] + (0.0 if is_raw else 0.05 * (k % 4))
        d[7] = 0.0 if is_raw else self.v * 0.05 * (k % 4)
        d[8] = self.gps_kmh
        n.cb_gps_fused(Float64MultiArray(data=d))
        r = self.v * math.tan(self.delta) / self.p['L']
        m = Imu()
        m.angular_velocity.z = r + math.radians(self.p['gyro_bias']
                                                + self.rng.normal(0, self.p['gyro_sigma']))
        n.cb_imu(m)
        n.cb_encoder(Int32(data=int(round(2 * self.v / MS_PER_PULSE))))
        n.cb_steer_measured(Int32(data=int(round(self.pot))))

    # ── 액추에이터 (arduino.py 규칙) ─────────────────────────────────────
    def read_outputs(self):
        n = self.node
        msg = n.pub_cmd.last
        if msg is not None:
            self.cmd_pulse = max(0, min(15, rha(float(msg.linear.x))))
            self.cmd_float = float(msg.angular.z)
            a = rha(self.cmd_float) if self.p['int_cmd'] else self.cmd_float
            self.cmd_angle = max(-40, min(40, a))
        b = n.pub_brake.last
        if b is not None:
            lvl = int(b.data)
            if lvl != self.brake_level:
                self.brake_level, self.brake_t = lvl, CLOCK.t

    def step_phys(self, c, dt):
        """kasa_0909_B.ino updateSteer 모사. c = B보드가 받은 정수 pot [deg]."""
        p = self.p
        t = CLOCK.t
        K = 3.375
        if (c != self.last_rx and t - self.last_rx_t >= 0.05) or t - self.last_rx_t >= 1.0:
            self.last_rx, self.last_rx_t = c, t        # 새 지령 / keepalive → ACTIVE
            self.active = True
            self.settle_t = None
        target = c * K
        self.b_t += dt
        if self.b_t >= 0.02 - 1e-9:                   # 20 ms 제어창
            self.b_t = 0.0
            med = self.cnt + self.rng.normal(0, p['pot_noise'])
            self.filt += 0.3 * (int(round(med)) - self.filt)
            cur = int(self.filt)
            self.motor = 0.0
            if self.active:
                err = target - cur
                out = 6.0 * err - 0.1 * ((cur - self.prev_cur) / 0.02)
                if abs(err) <= 3:                     # STEER_TOLERANCE_ENTER
                    if self.settle_t is None:
                        self.settle_t = t
                    elif t - self.settle_t >= 0.5:    # SETTLE_MS
                        self.active = False
                elif abs(err) > 6:                    # STEER_TOLERANCE_EXIT
                    self.settle_t = None
                    spd = max(110.0, min(255.0, abs(out)))   # STEER_MIN/MAX_PWM
                    self.motor = math.copysign(spd, out)
            self.prev_cur = cur
        m = self.motor
        if abs(m) > p['pwm0']:
            om_t = math.copysign(236.0 * (abs(m) - p['pwm0']) / (255.0 - p['pwm0']), m)
            self.om += (om_t - self.om) * min(1.0, dt / p['tau_m'])
        else:
            self.om -= self.om * min(1.0, dt / p['tau_c'])
        self.cnt += self.om * dt
        self.pot = self.cnt / K

    def step_plant(self, dt):
        p = self.p
        t = CLOCK.t
        self.cmd_hist.append((t, self.cmd_angle))
        while len(self.cmd_hist) > 2 and self.cmd_hist[1][0] <= t - p['T']:
            self.cmd_hist.pop(0)
        c = self.cmd_hist[0][1]
        if p['servo'] == 'phys':
            self.step_phys(c, dt)
        else:
            if c > self.x_bl + p['b']:
                self.x_bl = c - p['b']
            elif c < self.x_bl - p['b']:
                self.x_bl = c + p['b']
            want = self.pot + (self.x_bl - self.pot) * min(1.0, dt / max(dt, p['tau']))
            dmax = p['slew'] * dt
            self.pot = max(self.pot - dmax, min(self.pot + dmax, want))
        g = p['G'] * (1.0 + p['us_k'] * self.v * self.v)
        self.delta = -math.radians((self.pot - p['trim_true']) / g)    # pot + = 우 → 도로휠 + = 좌
        pulse = 0 if self.brake_level > 0 else self.cmd_pulse           # arduino compose (4)
        vt = pulse * MS_PER_PULSE * p['k_over']
        a = max(-p['coast'], min(p['acc'], (vt - self.v) / p['tau_v']))
        if self.brake_level == 1 and t - self.brake_t > 0.55:
            a -= 0.9
        elif self.brake_level >= 2 and t - self.brake_t > 0.30:
            a -= 3.0
        self.v = max(0.0, self.v + a * dt)
        self.x += self.v * math.cos(self.psi) * dt
        self.y += self.v * math.sin(self.psi) * dt
        self.psi += self.v * math.tan(self.delta) / p['L'] * dt

    # ── 주행 ─────────────────────────────────────────────────────────────
    def run(self):
        n = self.node
        dt = 0.01
        k = 0
        steps = 0
        while CLOCK.t - self.t0 < self.t_max:
            if steps % 5 == 0:                       # 20 Hz — driving CONTROL_HZ
                self.feed_sensors(k)
                n.loop()
                self.read_outputs()
                self.log.append((CLOCK.t - self.t0, self.x, self.y, self.psi, self.v,
                                 self.delta, self.pot, self.cmd_float, self.cmd_pulse,
                                 self.brake_level, n.wp_idx, n._cte_last,
                                 n._diag_target_dist, n._cte_i_term, n.steer_trim))
                k += 1
                if n.state != drv.S_DRIVE_RUN:
                    break
            self.step_plant(dt)
            CLOCK.t += dt
            steps += 1
        self.done_state = n.state
        # ★driving.destroy_node 는 부르지 않는다★ time.time() 으로 0.4 s 를 기다리는데
        #  가짜 시계는 흐르지 않아 영원히 돈다
        rclpy.node.Node.destroy_node(n)
        rclpy.shutdown()
        cols = ('t', 'x', 'y', 'psi', 'v', 'delta', 'pot', 'cmd', 'pulse', 'brake', 'wp',
                'cte', 'lfd', 'iterm', 'trim')
        a = np.array(self.log, dtype=float)
        self.tr = {c: a[:, i] for i, c in enumerate(cols)}
        self.tr['r'] = self.tr['v'] * np.tan(self.tr['delta']) / self.p['L']
        return self


# ══════════════════════════════════════════════════════════════════════════════
#  지표 — 모의 궤적과 실차 record CSV 에 ★같은 잣대★ 를 쓴다
# ══════════════════════════════════════════════════════════════════════════════
def path_kappa(wps, win=4.0):
    """±win 창 방위차로 잰 부호 있는 경로 곡률 [1/m] (+ = 좌회전)."""
    xy = np.asarray(wps, dtype=float)
    ds = np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))
    s = np.concatenate([[0], np.cumsum(ds)])
    n = len(s)
    kap = np.zeros(n)
    for i in range(1, n - 1):
        i0 = max(0, min(np.searchsorted(s, s[i] - win), i - 1))
        i2 = min(n - 1, max(np.searchsorted(s, s[i] + win), i + 1))
        h0 = math.atan2(xy[i, 1] - xy[i0, 1], xy[i, 0] - xy[i0, 0])
        h1 = math.atan2(xy[i2, 1] - xy[i, 1], xy[i2, 0] - xy[i, 0])
        dh = (h1 - h0 + math.pi) % (2 * math.pi) - math.pi
        kap[i] = dh / max(0.5, (s[i2] - s[i0]) / 2.0)
    return s, kap


def _segs(mask, t, min_len):
    out, i, n = [], 0, len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j + 1 < n and mask[j + 1] and t[j + 1] - t[j] < 0.5:
                j += 1
            if t[j] - t[i] >= min_len:
                out.append((i, j))
            i = j + 1
        else:
            i += 1
    return out


def _zc_period(x, t):
    x = x - np.mean(x)
    st, cross = 0, []
    for xi, ti in zip(x, t):
        s = 1 if xi > 0.05 else (-1 if xi < -0.05 else 0)
        if s != 0 and s != st:
            if st != 0:
                cross.append(ti)
            st = s
    return 2 * float(np.median(np.diff(cross))) if len(cross) >= 3 else float('nan')


def _reversals(x, hyst):
    n, ext, dirn = 0, x[0], 0
    for xi in x[1:]:
        if dirn >= 0 and xi < ext - hyst:
            n += dirn == 1
            dirn, ext = -1, xi
        elif dirn <= 0 and xi > ext + hyst:
            n += dirn == -1
            dirn, ext = 1, xi
        else:
            ext = max(ext, xi) if dirn >= 0 else min(ext, xi)
    return n


def stats(tr, wps, skip_s=20.0):
    """★흔들림(wob)★ = RMS(r − v·κ_경로) [deg/s] — 경로가 요구하는 요레이트를 뺀 나머지.
    굽은 경로에서도 쓸 수 있는 지표라 이것을 주 지표로 쓴다(1 s 이동평균 뒤).
    나머지 : |CTE| 평균·최대, 코너(κ>0.04) 최대·안쪽(+) 편향, 직선(R>83 m, 8 s 이상) CTE std·주기,
             횡가속 p99, 조향 서보 누적 이동량 [pot deg/s], 평균속도."""
    s, kap = path_kappa(wps)
    t = tr['t']
    m0 = t > min(skip_s, 0.3 * t[-1])
    wi = np.clip(tr['wp'].astype(int), 0, len(s) - 1)
    kmax = np.array([np.max(np.abs(kap[np.searchsorted(s, s[i] - 6):np.searchsorted(s, s[i] + 14) + 1]))
                     for i in wi])
    v = tr['v']
    cte = tr['cte']
    r = np.convolve(tr['r'], np.ones(20) / 20, mode='same')
    out = dict(v=float(np.mean(v[m0]) * 3.6) if np.any(m0) else 0.0, t_end=float(t[-1]))
    fin = m0 & np.isfinite(cte)
    out['cte_abs'] = float(np.mean(np.abs(cte[fin]))) if np.any(fin) else float('nan')
    out['cte_max'] = float(np.max(np.abs(cte[np.isfinite(cte)]))) if np.any(np.isfinite(cte)) else float('nan')
    curve = fin & (kmax > 0.04)
    if np.any(curve):
        out['c_max'] = float(np.max(np.abs(cte[curve])))
        out['c_in'] = float(np.mean((cte * np.sign(kap[wi]))[curve]))   # + = 안쪽
    straight = fin & (kmax < 0.012) & (v > 2.0)
    rows = []
    for (i, j) in _segs(straight, t, 8.0):
        sl = slice(i, j + 1)
        rows.append((t[j] - t[i], np.std(cte[sl]), _zc_period(cte[sl], t[sl])))
    if rows:
        w = np.array([x[0] for x in rows])
        out['s_std'] = float(np.sum(w * np.array([x[1] for x in rows])) / np.sum(w))
        pers = [x[2] for x in rows if np.isfinite(x[2])]
        out['s_per'] = float(np.median(pers)) if pers else float('nan')
    ks = np.convolve(kap, np.ones(13) / 13, mode='same')
    mm = m0 & (v > 2.0)
    if not np.any(mm):
        mm = v > 0.5
    rex = r - v * ks[wi]
    out['wob'] = float(np.degrees(np.sqrt(np.mean(rex[mm] ** 2))))
    out['wob_ay'] = float(np.sqrt(np.mean((v[mm] * rex[mm]) ** 2)))
    out['ay_p99'] = float(np.percentile(np.abs(v[mm] * r[mm]), 99))
    if 'pot' in tr and np.sum(m0) > 2:
        out['servo'] = float(np.sum(np.abs(np.diff(tr['pot'][m0]))) / max(1.0, t[m0][-1] - t[m0][0]))
    if np.sum(m0) > 2:
        out['rev'] = _reversals(tr['cmd'][m0], 0.5) / max(1.0, t[m0][-1] - t[m0][0]) * 60
    return out


def _f(x):
    if x in ('True', 'true'):
        return 1.0
    if x in ('False', 'false'):
        return 0.0
    try:
        return float(x)
    except (TypeError, ValueError):
        return float('nan')


def log_trace(record_csv, route_csv):
    """실차 record CSV → stats() 가 받는 궤적. ★DRIVE_RUN · lstatus '0' 만★ (라이다 구간 제외)."""
    rp = record_csv if os.path.isabs(record_csv) else os.path.join(BAG_DIR, record_csv)
    rows = [r for r in csv.DictReader(open(rp, encoding='utf-8-sig'))
            if r['drive_state'] == 'DRIVE_RUN' and r.get('lstatus', '0') in ('0', '')
            and _f(r.get('rejoin', '')) != 1.0]
    col = lambda k: np.array([_f(r[k]) for r in rows])   # noqa: E731
    t = col('t_rel')
    tr = dict(t=t - t[0], v=col('gps_kmh') / 3.6, cte=col('cte_m'), cmd=col('cmd_steer_deg'),
              wp=col('ego_wp_idx'), lfd=col('target_dist_m'), r=np.radians(col('gyro_z_dps')),
              pot=col('steer_measured_deg'))
    lat, lon = prepare_route(route_csv)
    wps = [drv.latlon_to_xy(a, b, lat[0], lon[0]) for a, b in zip(lat, lon)]
    return tr, wps


def ident_servo(tr, L=1.25):
    """★조향 식별★ 지령 pot(정수 반올림) → 지연 T → 백래시 b → 1차 τ → 도로휠 = (x − trim)/G.
    관측 도로휠 = atan(r·L/v) (요레이트·GPS 속도). 격자탐색 + 선형회귀. → (잔차, b, τ, T, G, trim)"""
    t = tr['t']
    m = (t > 20) & (tr['v'] > 2.0)
    v = tr['v'][m]
    r = tr['r'][m]
    cmd = np.array([rha(c) for c in tr['cmd'][m]], dtype=float)
    dact = -np.degrees(np.arctan(r * L / np.maximum(v, 1.0)))
    dt = 0.05
    best = None
    for b in (0.0, 0.5, 0.8, 1.0, 1.2, 1.5, 1.78, 2.0):
        y = np.empty_like(cmd)
        x = cmd[0]
        for i, c in enumerate(cmd):
            if c > x + b:
                x = c - b
            elif c < x - b:
                x = c + b
            y[i] = x
        for tau in (0.05, 0.15, 0.25, 0.4):
            z = np.empty_like(y)
            zi = y[0]
            a = min(1.0, dt / tau)
            for i, yi in enumerate(y):
                zi += a * (yi - zi)
                z[i] = zi
            for T in (0.0, 0.1, 0.2, 0.3, 0.4):
                k = int(round(T / dt))
                zz = np.concatenate([np.full(k, z[0]), z[:-k]]) if k else z
                A = np.vstack([zz, np.ones_like(zz)]).T
                sol, *_ = np.linalg.lstsq(A, dact, rcond=None)
                rms = float(np.sqrt(np.mean((dact - A @ sol) ** 2)))
                if best is None or rms < best[0]:
                    best = (rms, b, tau, T, 1.0 / sol[0], -sol[1] / sol[0])
    return best
