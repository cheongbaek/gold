#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
traffic_timer.py ― 본선 코스 신호 타이머 [white1]   [2026-09-30 신설 · 사용자 지시]
════════════════════════════════════════════════════════════════════════════════
 ★하는 일은 시계 하나다★ T1 에서 카메라가 본 ★적색→녹색 전환 시각을 0초★ 로 잡고,
 그 뒤로는 시스템 시계(time.time())만 보고 "지금 T1·T3·T4 가 녹색인가" 를 낸다.
 ★차를 세우고 출발시키는 것은 driving 이다★ — 이 노드는 판단 재료만 준다
 (리니어·펄스·조향 토픽을 하나도 내지 않는다).

 ┌ 신호 체계 (사용자 지시 — 모든 신호등이 0~100초 한 주기로 연동된다) ──────────┐
 │  T1 · T2 : 0 ~ 32초 진행          T3 : 35 ~ 47초 좌회전                      │
 │  T4 : 60 ~ 97초 좌회전            T5 : 0 ~ 57초 직진                         │
 └──────────────────────────────────────────────────────────────────────────┘
   T1     카메라로 적색→녹색 전환을 본다 → ★그 순간이 0초★. 전환을 보기 전이면 driving 이
          ★정지선 1 m 앞에 맞춰 선다★(녹색이어도). 접근 중에 전환을 보면 감속을 풀고 지나간다.
   T2     T1 과 같은 시각표라 ★무시하고 지나간다★ (CSV 에 마킹만 있다)
   T3·T4  ★카메라 없이 이 타이머만★ 본다 — 카메라 인지가 잘 안 되는 곳이다
   T5     카메라가 본다(driving 이 LightFilter 로) — 이 타이머는 관여하지 않는다
   ★T 구간의 끝 = 정지선★ [2026-10-01] 두 코스 모두 좌표로 맞췄다. 정지·감속의 방법은 driving
   상수절 '신호 접근' 이 소유한다.

 ★본선 코스(maincourse.csv)에서만 돈다★ prompt 가 경로를 고를 때 set_route() 로
   알려 준다. 그 밖의 경로를 고르면 ★아무것도 내지 않고 콜백도 무시한다★ — 그래서
   노드가 떠 있어도 종전 경로의 거동은 한 글자도 바뀌지 않는다(driving 은 그 경로에서
   이 토픽을 읽지도 않는다).

 ★prompt 와 한 프로세스로 뜬다★ (사용자 지시) prompt.main() 이 이 노드를 만들어 같은
   실행기에 얹는다 — 따로 띄우는 진입점(console_scripts)을 두지 않은 이유는, 둘을 같이
   띄우면 /traffic_timer 발행자가 둘이 되어 기준 시각이 갈라지기 때문이다.

════════════════════════════════════════════════════════════════════════════════
 전환 판정 — T1 을 '상대하는 동안' 만 본다
════════════════════════════════════════════════════════════════════════════════
 driving 이 /tl_zone 으로 "지금 상대하는 신호" 를 알려 준다 — T 구간 안이거나, 다음
 T 구간 시작까지 WATCH_PRE_M(40 m) 안일 때 그 라벨이다. 그것이 'T1' 인 동안만
 /tl/state(카메라 프레임 판정)를 본다.
   · 40 m 인 이유 — 9/13 기록에서 T1 신호등이 처음 잡힌 것이 T1 시작 35 m 앞(RED_FAR)
     이었다. 그보다 멀리서 잡힌 것은 T1 신호라고 보기 어렵다(같은 기록에 T1 77 m 앞,
     출발 직후의 GREEN 7프레임이 있다 — 무엇이었는지는 확인하지 않았다).
   · T3·T4 의 좌회전 녹색은 35·60초에 켜진다 — 그 전환을 0초로 잘못 잡지 않으려면
     T1 이 아닌 곳에서는 전환을 보면 안 된다.

   적색(RED·RED_FAR)을 RED_MIN_S 이상 이어서 봤다
     → RED_TO_GREEN_MAX_S 안에 녹색(GREEN)이 나왔다       ← 이 첫 녹색 프레임 시각이 0초
     → 그 녹색이 GREEN_MIN_S 이어졌다(도중에 적색이 끼면 무효) → ★초기화 확정★
   스트릭은 GAP_S 안의 끊김(UNKNOWN 프레임)은 봐준다 — 가까워지면 등기구가 화면
   위로 벗어나 한두 프레임씩 빠지는 것이 정상이다(traffic_light._feed_state 와 같은 이유).

 ★주행마다 다시 잰다★ /drive_state 가 DRIVE_HEADING 으로 들어오는 순간(새 주행) 기준
   시각을 지운다. 본선 코스는 T3·T4 전에 반드시 T1 을 지나고, T1 은 전환을 봐야만
   지나가므로 이전 주행의 기준을 들고 있을 이유가 없다.

 ★시스템 시계를 쓴다★ (사용자 지시) time.time() — NTP 가 주행 중에 시계를 옮기면
   위상도 그만큼 옮는다. 보통은 ms 단위라 무시할 만하다.

════════════════════════════════════════════════════════════════════════════════
 /traffic_timer (std_msgs/Float64MultiArray, 20 Hz) — ★이 절이 배열 규약의 소유자다★
════════════════════════════════════════════════════════════════════════════════
   [0] armed        1 = 본선 코스가 골라져 타이머가 켜져 있다 (꺼져 있으면 아예 안 낸다)
   [1] initialized  1 = ★이번 주행에서★ T1 전환을 봐서 0초가 잡혔다
   [2] phase_s      지금 위상 [s] 0~100. 초기화 전이면 NaN
   [3] go_T1        1 = T1 녹색(0~32초)이다      ─┐ 초기화 전이면 전부 0 —
   [4] go_T3        1 = T3 녹색(35~47초)이다      ├ ★모르면 적색으로 본다★
   [5] go_T4        1 = T4 녹색(60~97초)이다     ─┘
   [6] watch        0 = T1 을 안 보는 중 / 1 = 보는 중 / 2 = 적색 확인, 녹색 기다림
   [7] t0_epoch     0초의 UNIX 시각 [s]. 초기화 전이면 NaN
 driving 은 ★신선할 때만★ 믿는다(STALE_S) — prompt 가 죽으면 go 가 끊겨 T1·T3·T4 에서
 서서 기다린다(신호위반보다 정지가 낫다). record 는 이 배열을 그대로 적는다.
"""

import math
import os
import time

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

# ── 경로 ──────────────────────────────────────────────────────────────────────
#  ★본선 코스 파일 이름의 단일 소유자★ prompt(타이머 켜기)와 driving(T1~T5 구분)이
#  둘 다 이 이름을 본다 — 한쪽만 바꾸면 '타이머는 켜졌는데 driving 은 T 로만 도는'
#  상태가 된다.
MAINCOURSE_FILE = 'maincourse.csv'

# ── 신호 체계 (사용자 지시) ───────────────────────────────────────────────────
CYCLE_S = 100.0
#  [시작, 끝) 초 — ★여유를 두지 않는다★ (사용자 결정 '신호 그대로').
#  판단 지점(T 구간 진입)을 지나는 순간과, 서서 기다리다 출발하는 순간 둘 다에 쓴다.
SIGNAL_WINDOWS = {
    'T1': (0.0, 32.0),     # 진행 — ★0초의 기준★
    'T2': (0.0, 32.0),     # T1 과 같다 — 무시한다
    'T3': (35.0, 47.0),    # 좌회전
    'T4': (60.0, 97.0),    # 좌회전
    'T5': (0.0, 57.0),     # 직진 — 카메라가 본다(여기서는 기록용)
}
INIT_SIGNAL = 'T1'                     # 이 신호의 적색→녹색이 0초다
TIMER_SIGNALS = ('T1', 'T3', 'T4')     # driving 이 이 타이머로 세우고 보내는 신호
CAMERA_SIGNALS = ('T5',)               # 종전 T 와 같이 카메라가 개입하는 신호
IGNORED_SIGNALS = ('T2',)              # 마킹만 있다

# ── 토픽 ──────────────────────────────────────────────────────────────────────
TOPIC = '/traffic_timer'               # 이 노드 → driving · record
ZONE_TOPIC = '/tl_zone'                # driving → 이 노드 : 지금 상대하는 신호 라벨
TL_STATE_TOPIC = '/tl/state'           # traffic_light → 이 노드 : 프레임 판정
PUBLISH_HZ = 20.0
STALE_S = 1.0                          # driving 이 이보다 낡은 값은 '없다' 로 본다
ZONE_STALE_S = 1.0                     # /tl_zone 이 이보다 낡으면 T1 을 안 본다

# 배열 인덱스 (위 docstring 표와 1:1)
F_ARMED, F_INIT, F_PHASE, F_GO_T1, F_GO_T3, F_GO_T4, F_WATCH, F_T0 = range(8)
N_FIELDS = 8
GO_FIELD = {'T1': F_GO_T1, 'T3': F_GO_T3, 'T4': F_GO_T4}

# ── 전환 판정 ─────────────────────────────────────────────────────────────────
WATCH_PRE_M = 40.0         # [m] T 구간 시작 이만큼 앞부터 '상대하는 신호' 다 (driving 이 쓴다)
RED_MIN_S = 0.3            # [s] 적색을 이만큼 이어서 봐야 '적색이었다'
GREEN_MIN_S = 0.4          # [s] 녹색이 이만큼 이어져야 전환 확정 (traffic_light green_hold_s 와 같다)
GAP_S = 0.5                # [s] 스트릭 안의 끊김(UNKNOWN)은 이만큼 봐준다
RED_TO_GREEN_MAX_S = 3.0   # [s] 마지막 적색과 첫 녹색 사이가 이보다 멀면 전환이 아니다
RED_STATES = ('RED', 'RED_FAR')
GREEN_STATE = 'GREEN'

WATCH_OFF, WATCH_ON, WATCH_RED = 0, 1, 2

#  ★카메라 구간(T5 · 그 밖 경로의 T) — driving 이 /tl/state 를 직접 읽는다★ [2026-10-01]
#  카메라가 이 시간 넘게 아무 판정도 안 내면 '죽었다' 로 본다 = 신호를 못 봤다 = 통과(fail-open).
#  ★UNKNOWN 프레임은 '살아 있다' 다★ — 정지선 앞에서 등기구가 화면 위로 벗어나 UNKNOWN 이
#  이어져도 빨간불 대기는 풀리지 않는다(녹색을 봐야 풀린다 — LightFilter).
CAMERA_STALE_S = 2.0


class LightFilter:
    """카메라 프레임 판정(/tl/state) → ★적색 확정 · 녹색 확정★.  [2026-10-01]

    driving 의 카메라 구간이 쓴다. 판정 문턱은 위 전환 판정과 같은 값이다(RED_MIN_S ·
    GREEN_MIN_S · GAP_S) — 둘이 갈라지면 같은 신호등을 두 곳이 다르게 읽는다.
      · 적색 = RED·RED_FAR. ★RED_FAR 도 센다★ — T 구간 안(정지선 앞 15~30 m)에서는 신호등
        박스가 아직 작다. 구간 밖의 먼 빨간불은 driving 이 구간 안에서만 물어보므로 안 걸린다.
      · ★적색 프레임은 녹색 스트릭을 죽인다★ — 적색등과 좌회전 녹색이 함께 보이는 등기구는
        적색으로 읽힌다(직진 차에게는 그게 맞다).
    """

    def __init__(self):
        self.last_t = 0.0
        self.reset()

    def reset(self):
        self._red_since = self._red_last = None
        self._green_since = self._green_last = None

    def feed(self, state, now):
        self.last_t = now
        if state in RED_STATES:
            if self._red_last is None or (now - self._red_last) > GAP_S:
                self._red_since = now
            self._red_last = now
            self._green_since = self._green_last = None
        elif state == GREEN_STATE:
            if self._green_last is None or (now - self._green_last) > GAP_S:
                self._green_since = now
            self._green_last = now

    def alive(self, now):
        return self.last_t > 0.0 and (now - self.last_t) <= CAMERA_STALE_S

    def red_confirmed(self, now):
        return (self._red_since is not None and (now - self._red_last) <= GAP_S
                and (self._red_last - self._red_since) >= RED_MIN_S)

    def green_confirmed(self, now):
        return (self._green_since is not None and (now - self._green_last) <= GAP_S
                and (self._green_last - self._green_since) >= GREEN_MIN_S)


def is_maincourse(route_name):
    """이 경로가 본선 코스인가. 경로(폴더 포함)를 받아도 된다."""
    return os.path.basename(str(route_name or '').strip()) == MAINCOURSE_FILE


def phase_of(t0, now=None):
    """기준 시각 t0 로부터의 위상 [s] 0~CYCLE_S. t0 가 없으면 NaN."""
    if t0 is None:
        return float('nan')
    return ((time.time() if now is None else now) - t0) % CYCLE_S


def in_window(label, phase):
    """label 신호가 위상 phase 에서 녹색인가. 모르는 라벨·NaN 이면 False(= 적색)."""
    win = SIGNAL_WINDOWS.get(label)
    if win is None or not math.isfinite(phase):
        return False
    return win[0] <= phase < win[1]


def window_txt(label):
    win = SIGNAL_WINDOWS.get(label)
    return f"{win[0]:.0f}~{win[1]:.0f}초" if win else '?'


class TrafficTimerNode(Node):
    """본선 코스 신호 타이머. prompt 가 만들고 set_route() 로 켜고 끈다."""

    def __init__(self):
        super().__init__('traffic_timer')
        self.armed = False
        self.t0 = None               # 0초의 UNIX 시각 — 이번 주행에서 T1 전환을 본 순간
        self.zone = ''               # driving 이 알려 준 '상대하는 신호'
        self.zone_t = 0.0
        self.drive_state = ''
        self._reset_watch()

        self.pub = self.create_publisher(Float64MultiArray, TOPIC, 10)
        self.create_subscription(String, TL_STATE_TOPIC, self.cb_tl_state, 10)
        self.create_subscription(String, ZONE_TOPIC, self.cb_zone, 10)
        self.create_subscription(String, '/drive_state', self.cb_drive_state, 10)
        self.create_timer(1.0 / PUBLISH_HZ, self.tick)

    # ── prompt 가 부른다 ─────────────────────────────────────────────────────
    def set_route(self, name):
        """경로를 골랐다. ★본선 코스일 때만 켠다★ — 다른 경로면 끄고 기준도 버린다."""
        on = is_maincourse(name)
        if on == self.armed:
            return
        if not on:
            self.armed = False            # 먼저 끈다 — tick 이 반쯤 지운 값을 내지 않게
        self.t0 = None
        self._reset_watch()
        self.armed = on
        self.get_logger().info(
            f"🚦 신호 타이머 {'켜짐 — 본선 코스' if on else '꺼짐'} ({name or '경로 없음'})")

    def status_text(self):
        """prompt 화면에 띄울 한 줄. ★초 단위로 바뀌지 않는다★ — 화면이 바뀔 때마다
        다시 찍으므로, 흐르는 위상을 넣으면 화면이 매초 쏟아진다."""
        if not self.armed:
            return ''
        if self.t0 is None:
            if self._watch_state() == WATCH_RED:
                return "🚦 신호 타이머 — T1 적색 확인, 녹색으로 바뀌는 순간을 0초로 잡는다"
            return "🚦 신호 타이머 — 초기화 전 (T1 에서 적색→녹색 전환을 보면 0초)"
        return (f"🚦 신호 타이머 작동 — 0초 = "
                f"{time.strftime('%H:%M:%S', time.localtime(self.t0))} (T1 녹색 시작)")

    # ── 구독 ────────────────────────────────────────────────────────────────
    def cb_drive_state(self, msg):
        new = str(msg.data).strip()
        #  ★새 주행이면 기준을 버린다★ (docstring '주행마다 다시 잰다')
        if new == 'DRIVE_HEADING' and self.drive_state != 'DRIVE_HEADING':
            if self.armed and self.t0 is not None:
                self.get_logger().info("🚦 새 주행 — 신호 타이머 기준을 지운다(T1 에서 다시 잰다)")
            self.t0 = None
            self._reset_watch()
        self.drive_state = new

    def cb_zone(self, msg):
        self.zone = str(msg.data).strip()
        self.zone_t = time.time()

    def cb_tl_state(self, msg):
        if not self.armed:
            return
        now = time.time()
        if not self._watching(now):
            self._reset_watch()          # T1 밖에서 본 적색을 T1 의 적색으로 세지 않는다
            return
        self._feed(str(msg.data).strip(), now)

    # ── 전환 판정 ────────────────────────────────────────────────────────────
    def _reset_watch(self):
        self._red_since = None       # 적색 스트릭 시작
        self._red_last = None        # 마지막 적색 프레임
        self._green_since = None     # 녹색 후보의 첫 프레임 = 0초 후보
        self._green_last = None

    def _watching(self, now):
        return (self.armed and self.drive_state == 'DRIVE_RUN'
                and self.zone == INIT_SIGNAL
                and (now - self.zone_t) <= ZONE_STALE_S)

    def _red_confirmed(self):
        return (self._red_since is not None
                and (self._red_last - self._red_since) >= RED_MIN_S)

    def _watch_state(self):
        if not self._watching(time.time()):
            return WATCH_OFF
        return WATCH_RED if self._red_confirmed() else WATCH_ON

    def _feed(self, state, now):
        if state in RED_STATES:
            if self._red_last is None or (now - self._red_last) > GAP_S:
                self._red_since = now            # 끊겼으면 새 스트릭
            self._red_last = now
            self._green_since = self._green_last = None   # ★녹색 도중의 적색 = 깜빡임, 무효★
            return
        if state != GREEN_STATE:
            return                               # UNKNOWN — 간격은 시각으로 잰다
        if self._green_since is not None and (now - self._green_last) > GAP_S:
            self._green_since = self._green_last = None   # 녹색이 끊겼다 — 후보를 버린다
        if self._green_since is None:
            if not (self._red_confirmed() and (now - self._red_last) <= RED_TO_GREEN_MAX_S):
                return                           # ★적색을 먼저 봐야 전환이다★
            self._green_since = now
        self._green_last = now
        if (self._green_last - self._green_since) >= GREEN_MIN_S:
            self._init(self._green_since)

    def _init(self, t0):
        again = self.t0 is not None
        self.t0 = t0
        self._reset_watch()
        self.get_logger().info(
            f"🚦 신호 타이머 {'다시 ' if again else ''}초기화 — T1 적색→녹색 전환 "
            f"{time.strftime('%H:%M:%S', time.localtime(t0))}.{int((t0 % 1) * 10)} = 0초")

    # ── 발행 ────────────────────────────────────────────────────────────────
    def tick(self):
        if not self.armed:
            return                               # ★꺼져 있으면 아무것도 내지 않는다★
        phase = phase_of(self.t0)
        d = [0.0] * N_FIELDS
        d[F_ARMED] = 1.0
        d[F_INIT] = 0.0 if self.t0 is None else 1.0
        d[F_PHASE] = phase
        for label, idx in GO_FIELD.items():
            d[idx] = 1.0 if in_window(label, phase) else 0.0
        d[F_WATCH] = float(self._watch_state())
        d[F_T0] = float('nan') if self.t0 is None else float(self.t0)
        self.pub.publish(Float64MultiArray(data=d))
