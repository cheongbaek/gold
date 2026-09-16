#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""joyread.py ― 조이스틱 메가 보드("J,") 해석 [nxde]
════════════════════════════════════════════════════════════════════════════════
★노드도 아니고 포트도 열지 않는다★ — 줄을 받아서 파싱하고, 영점을 잡고, 스틱
위치를 ★차량 지령(펄스·조향·브레이크)★ 으로 환산하는 일만 한다.

  ★[2026-09-16] 포트의 소유자가 `nxde/arduino.py` 로 바뀌었다★
  종전에는 이 파일이 자기 스레드로 포트를 찾아 열었고, 그것을 `master.py` 의
  체크박스가 켰다. 그러면 ① 조이스틱 조종이 master 창을 띄운 자리에서만 되고
  ② 조이스틱과 A/B 보드가 ★같은 포트 풀을 각자 두드려★ 서로를 리셋시킨다.
  이제 A·B·J 세 보드를 arduino 노드 하나가 찾고, 그 노드가 줄을 이 상태기계에
  먹인다(`feed_line`). 그래서 one_launch 든 master.launch 든 ★arduino 가 뜨는
  자리면 어디서나★ 조이스틱으로 몰 수 있다.

    st = JoystickState()
    st.feed_line("J,512,700,1,480,500,1,1,1")   # 시리얼에서 온 줄마다
    snap = st.snapshot()                        # 제어 틱마다 한 번
    pulse, steer, brake = st.command(snap)

════════════════════════════════════════════════════════════════════════════════
 프로토콜 — joyled.ino / joy4.ino (2026-09-16)
════════════════════════════════════════════════════════════════════════════════
    J,x1,y1,k1,x2,y2,k2,swa,swb        ← 9토큰 (접두어 포함)
    J,RESET                            ← 보드가 방금 (재)부팅했다

  · 아날로그는 전부 0~1023 raw, 버튼은 ★active-low★ (0 = 눌림).
  · ★구 joy.ino 의 12토큰(… ,d12,d13,a0) 도 그대로 받는다★ — 앞 8필드가 같아서
    필드 수만 보고 뒤를 버리면 된다. (U 보드(joy2.ino)는 축 규격이 달라 여기서
    안 다룬다 — 그쪽이 필요하면 `nxde/joystick.py` 노드를 쓴다.)
  · ★리셋 버튼이 곧 영점 재보정이다★ 보드가 부팅하며 "J,RESET" 을 한 줄 보내고,
    이 상태기계는 스틱이 제자리로 돌아올 시간(RESET_CALIB_DELAY)을 준 뒤 그때의
    값으로 영점을 다시 잡는다.

════════════════════════════════════════════════════════════════════════════════
 조작 규약 — ★여기가 이 파일의 본론이다★
════════════════════════════════════════════════════════════════════════════════
    L스틱 위쪽   주행 펄스 0~15         (영점 기준, 데드존 밖에서만)
    L스틱 아래쪽 브레이크 0 / 1 / 2 단   (중앙~맨아래를 3등분)
    R스틱 좌우   조향 −40 ~ +40         ★− 좌 / + 우 (ROS·B보드 공통 규약)★
    SWA 짧게     시작 / 일시정지 토글   (쓰는 쪽이 `swa_releases` 로 받는다)
    SWB          ★보드 안에서 스톱워치를 조작한다★ — ROS 는 상태만 중계한다

  ★부호가 뒤집히는 지점은 `command()` 의 조향 한 줄뿐이다★ 보드 내부 규격은
  'x 가 클수록 왼쪽' 이라 한 번 뒤집으면 ROS 규약이 된다. 두 번 뒤집으면 조용히
  좌우가 바뀐다(CLAUDE.md 2절).
"""

import statistics
import threading
import time


ADC_MAX = 1023
DEFAULT_CENTER = ADC_MAX // 2

PULSE_MAX = 15            # A보드 단일값 입력 상한 (TARGET_MAX)
STEER_MAX = 40            # B보드 STEER_ANGLE_MAX
BRAKE_LEVEL_MAX = 2       # 0 = 놓음 / 1 = 약(1/3) / 2 = 풀

DEADZONE_RAW = 120        # 영점 기준 raw 편차가 이 값 미달이면 0 으로 본다
                          #   (보정된 스틱도 이 정도는 흔들린다 — L/R 공통)

CALIBRATION_SAMPLES = 20  # 영점(중앙값)에 쓸 샘플 수
RESET_CALIB_DELAY = 3.0   # "J,RESET" 뒤 몇 초 값을 영점으로 삼을지
                          #   (스틱이 제자리로 돌아올 시간을 준다)
STALE_INPUT_S = 0.6       # 이 시간 이상 줄이 안 오면 '입력 상실'

JOY_PREFIX = 'J,'
# 접두어를 포함한 총 토큰 수. 9 = joyled.ino/joy4.ino(신) / 12 = joy.ino(구, 뒤 3필드는 버린다)
JOY_FIELD_COUNTS = (9, 12)

# ── 버튼 비트 (ROS 로 중계할 때 쓴다 — /joy_buttons) ──────────────────────
#   ★지금은 아무도 구독하지 않는다★ 추후 prompt.py 를 이 스위치로 조작하기 위한
#   자리만 만들어 둔 것이다(2026-09-16 사용자 지시 6항 "보낼 준비까지만").
BTN_K1  = 1 << 0     # L스틱 버튼
BTN_K2  = 1 << 1     # R스틱 버튼
BTN_SWA = 1 << 2     # SWA (조종 시작/일시정지 — 이미 쓰고 있다)
BTN_SWB = 1 << 3     # SWB (보드 안에서 스톱워치를 조작한다)


# ══════════════════════════════════════════════════════════════════════════════
#  파싱 · 스케일링
# ══════════════════════════════════════════════════════════════════════════════
def is_joy_line(text):
    """이 줄이 조이스틱 보드의 줄인가 (RESET 포함)."""
    return text.startswith(JOY_PREFIX)


def parse_joy_line(line):
    """조이스틱 한 줄 → (x1,y1,k1,x2,y2,k2,swa,swb). 아니면 None."""
    parts = line.split(',')
    if len(parts) not in JOY_FIELD_COUNTS or parts[0] != 'J':
        return None
    try:
        return tuple(int(v) for v in parts[1:9])
    except ValueError:
        return None


def normalize(raw, center):
    """center 기준으로 양쪽 구간을 각각 독립 비율로 −1..1 에 매핑.

    영점이 정확히 512 가 아니므로 위/아래(좌/우) 구간 길이가 다르다 — 한쪽
    기준으로 나누면 반대쪽이 끝까지 가도 1 에 못 닿거나 넘친다."""
    center = max(1, min(ADC_MAX - 1, center))
    if raw >= center:
        return (raw - center) / (ADC_MAX - center)
    return (raw - center) / center


def scaled_with_deadzone(raw, center, deadzone_raw, max_out):
    """center 기준 편차가 deadzone_raw 미달이면 0. 그 밖은 (1..max_out) 로 재매핑.

    ★데드존을 raw 로 잡는 이유★ 흔들림은 ADC 눈금의 성질이라 출력 단위(펄스·도)와
    무관하다. raw 로 한 번 자르면 L(펄스)·R(조향) 어느 쪽에도 같은 값을 쓸 수 있다.
    데드존 바로 밖에서 출력이 0 → 1 로 뛰는 것은 의도다 — 0.4펄스 같은 값은
    A보드가 못 받는다(정수 지령)."""
    center = max(1, min(ADC_MAX - 1, center))
    diff = raw - center
    span = (ADC_MAX - center) if diff >= 0 else center
    mag = abs(diff)
    if mag < deadzone_raw:
        return 0.0
    remain = span - deadzone_raw
    magnitude = max_out if remain <= 0 else 1 + (mag - deadzone_raw) / remain * (max_out - 1)
    return magnitude if diff >= 0 else -magnitude


# ══════════════════════════════════════════════════════════════════════════════
#  상태기계
# ══════════════════════════════════════════════════════════════════════════════
class JoystickState:
    """조이스틱 줄을 먹고 상태를 들고 있는다. ★포트는 부르는 쪽이 소유한다★

    ★스레드 경계를 한 곳으로 모은다★ 줄을 먹이는 쪽(arduino 의 RX 타이머)과 값을
    읽는 쪽(TX 타이머·화면)이 다를 수 있으므로, lock 을 잡아 한 번에 복사해 준다.
    """

    def __init__(self, deadzone=DEADZONE_RAW, pulse_max=PULSE_MAX, log=None):
        self.log = log or (lambda m: None)
        self.deadzone = int(deadzone)
        self.pulse_max = max(1, min(PULSE_MAX, int(pulse_max)))

        self._lock = threading.Lock()
        self.data = None            # 마지막 8튜플
        self.data_t = 0.0

        self.calibrated = False
        self.calib_buf = None
        self.center = [DEFAULT_CENTER] * 4      # x1, y1, x2, y2
        self._reset_at = None

        # ★SWA 릴리즈 엣지를 세어 둔다★ 쓰는 쪽(20Hz 틱)이 짧은 누름을 놓치지
        #   않게, 엣지를 '지금 눌려 있나' 가 아니라 ★횟수★ 로 넘긴다.
        self.swa_releases = 0
        self._swa_prev = 1

    # ── 수명 ──────────────────────────────────────────────────────────
    def reset(self, reason=''):
        """연결·재연결·단절에서 부른다. ★영점부터 다시 잡는다★

        보드가 빠졌다 들어오면 그 사이에 스틱이 어디에 있었는지 알 수 없고,
        재연결은 대개 보드 리셋을 동반한다(포트 open 이 DTR 을 흔든다)."""
        with self._lock:
            self.data = None
            self.data_t = 0.0
            self.calibrated = False
            self.calib_buf = {'x1': [], 'y1': [], 'x2': [], 'y2': []}
            self._reset_at = None
            self.swa_releases = 0
            self._swa_prev = 1
        if reason:
            self.log(f"조이스틱 영점을 다시 잡습니다 — {reason} "
                     f"(스틱을 건드리지 말고 잠시 기다리십시오)")

    # ── 입력 ─────────────────────────────────────────────────────────
    def feed_line(self, text):
        """시리얼에서 온 줄 하나. 조이스틱 줄이 아니면 조용히 버린다."""
        if not text:
            return
        if 'RESET' in text:
            # 보드가 방금 (재)부팅했다 — 스틱이 제자리로 돌아올 시간을 준 뒤 재보정.
            # ★리셋 버튼이 곧 영점 재보정 수단이다★ (사용자 지시 2항)
            with self._lock:
                self._reset_at = time.monotonic()
            self.log(f"조이스틱 RESET 감지 — {RESET_CALIB_DELAY:.0f}초 뒤 "
                     f"영점을 다시 잡습니다")
            return

        parsed = parse_joy_line(text)
        if parsed is None:
            return
        with self._lock:
            self.data = parsed
            self.data_t = time.monotonic()
            if self._swa_prev == 0 and parsed[6] == 1:      # active-low 릴리즈
                self.swa_releases += 1
            self._swa_prev = parsed[6]
            self._step_calibration_locked(parsed)

    # ── 영점 ─────────────────────────────────────────────────────────
    def _step_calibration_locked(self, parsed):
        """lock 을 잡은 채 호출된다. 샘플을 모아 ★중앙값★ 을 영점으로 잡는다.

        평균이 아니라 중앙값인 이유는 단발 튐이 영점을 끌고 가지 않게 하기 위한
        것이다(스택 전체가 같은 이유로 median3 을 쓴다)."""
        if self._reset_at is not None:
            if time.monotonic() - self._reset_at < RESET_CALIB_DELAY:
                return
            self._reset_at = None
            self.calibrated = False
            self.calib_buf = {'x1': [], 'y1': [], 'x2': [], 'y2': []}

        if self.calibrated:
            return
        if self.calib_buf is None:
            self.calib_buf = {'x1': [], 'y1': [], 'x2': [], 'y2': []}

        x1, y1, x2, y2 = parsed[0], parsed[1], parsed[3], parsed[4]
        self.calib_buf['x1'].append(x1)
        self.calib_buf['y1'].append(y1)
        self.calib_buf['x2'].append(x2)
        self.calib_buf['y2'].append(y2)
        if len(self.calib_buf['y1']) < CALIBRATION_SAMPLES:
            return

        self.center = [statistics.median(self.calib_buf[k])
                       for k in ('x1', 'y1', 'x2', 'y2')]
        self.calib_buf = None
        self.calibrated = True
        self.log(f"✅ 조이스틱 영점 완료 — L=({self.center[0]:.0f},{self.center[1]:.0f}) "
                 f"R=({self.center[2]:.0f},{self.center[3]:.0f})  "
                 f"★이제 SWA 를 한 번 누르면 시작합니다★")

    # ── 읽기 ─────────────────────────────────────────────────────────
    def snapshot(self):
        """제어·화면이 한 번에 읽어 가는 상태. ★lock 은 여기서만 잡는다★

        `swa_releases` 는 ★소비하면 0 으로 비운다★ — 같은 누름이 두 번 먹지 않게
        하는 규약이고, 그래서 이 함수는 한 틱에 한 번만 불러야 한다.
        """
        with self._lock:
            releases, self.swa_releases = self.swa_releases, 0
            fresh = (self.data is not None
                     and (time.monotonic() - self.data_t) <= STALE_INPUT_S)
            return {
                'calibrated': self.calibrated,
                'calib_n':    len(self.calib_buf['y1']) if self.calib_buf else 0,
                'data':       self.data,
                'fresh':      fresh,
                'center':     tuple(self.center),
                'swa_releases': releases,
            }

    def gate_reason(self, snap):
        """지금 이 입력을 쓸 수 없는 이유. 없으면 None.

        ★차량 쪽 사정(주행모드·E-STOP)은 보지 않는다★ 그것은 이 파일이 알 수 있는
        것이 아니고, 부르는 쪽(arduino)이 자기 상태로 얹는다."""
        if snap['data'] is None:
            return "조이스틱 입력 없음"
        if not snap['fresh']:
            return "조이스틱 입력 끊김"
        if not snap['calibrated']:
            return (f"영점 잡는 중 ({snap['calib_n']}/{CALIBRATION_SAMPLES}) "
                    f"— 스틱을 놓으십시오")
        return None

    def buttons(self, snap):
        """눌린 버튼 비트마스크 (BTN_*). 입력이 없으면 0.

        ★지금은 발행만 하고 아무도 구독하지 않는다★ (사용자 지시 6항)."""
        data = snap['data']
        if data is None or not snap['fresh']:
            return 0
        bits = 0
        if data[2] == 0:
            bits |= BTN_K1
        if data[5] == 0:
            bits |= BTN_K2
        if data[6] == 0:
            bits |= BTN_SWA
        if data[7] == 0:
            bits |= BTN_SWB
        return bits

    def command(self, snap):
        """스냅샷 → (pulse, steer, brake). 쓸 수 없는 상태면 (0, 0, 0).

        ★L스틱 위 = 펄스 / L스틱 아래 = 브레이크 / R스틱 좌우 = 조향★
        한 축이 두 가지 일을 하므로 ★위쪽에서는 브레이크가, 아래쪽에서는 펄스가
        반드시 0 이어야 한다★ — 한 스틱이 '밀면서 밟는' 값을 내면 A보드는 구동을,
        B보드는 제동을 동시에 받는다. `scaled_with_deadzone` 이 위쪽에서 음수를
        내지 않고(max 0), 아래쪽 3등분이 중앙에서 0 을 내므로 그 배타가 성립한다.
        """
        data = snap['data']
        if data is None or not snap['fresh'] or not snap['calibrated']:
            return 0, 0, 0
        y1, x2 = data[1], data[3]               # y1 = L 세로, x2 = R 가로
        cy1, cx2 = snap['center'][1], snap['center'][2]

        # 주행 펄스 — L스틱 ★위쪽만★ (아래쪽은 브레이크가 쓴다)
        pulse = int(round(max(0.0, scaled_with_deadzone(
            y1, cy1, self.deadzone, self.pulse_max))))
        pulse = max(0, min(self.pulse_max, pulse))

        # 브레이크 — L스틱 아래, 중앙~맨아래를 3등분해 0 / 1 / 2
        #   ★맨 아래 1/3 이 데드존 몫이다★ 손을 얹은 정도로는 물리지 않는다.
        down = -normalize(y1, cy1)
        brake = 2 if down >= 2.0 / 3.0 else (1 if down >= 1.0 / 3.0 else 0)

        # 조향 — R스틱 가로. ★부호가 뒤집히는 유일한 지점★ (− 좌 / + 우)
        steer = int(round(-scaled_with_deadzone(x2, cx2, self.deadzone, STEER_MAX)))
        steer = max(-STEER_MAX, min(STEER_MAX, steer))
        return pulse, steer, max(0, min(BRAKE_LEVEL_MAX, brake))
