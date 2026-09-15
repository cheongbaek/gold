#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""joyread.py ― 조이스틱 메가 보드("J,") 읽기 [nxde]
════════════════════════════════════════════════════════════════════════════════
★노드가 아니다★ — 시리얼 포트를 찾아 열고, 줄을 파싱하고, 영점을 잡고, 스틱
위치를 ★차량 지령(펄스·조향·브레이크)★ 으로 환산하는 일만 한다. ROS 도, 화면도
모른다. 지금 쓰는 곳은 `nxde/master.py` 의 '조이스틱으로 조종하기' 체크박스다.

    from nxde.joyread import JoystickReader
    r = JoystickReader(log=node.get_logger().info)
    r.start()
    snap = r.snapshot()          # 매 틱 한 덩어리로 읽어 간다
    r.stop()

════════════════════════════════════════════════════════════════════════════════
 프로토콜 — joy4.ino (2026-09-15)
════════════════════════════════════════════════════════════════════════════════
    J,x1,y1,k1,x2,y2,k2,swa,swb        ← 9토큰 (접두어 포함)
    J,RESET                            ← 보드가 방금 (재)부팅했다

  · 아날로그는 전부 0~1023 raw, 버튼은 ★active-low★ (0 = 눌림).
  · ★구 joy.ino 의 12토큰(… ,d12,d13,a0) 도 그대로 받는다★ — 앞 8필드가 같아서
    필드 수만 보고 뒤를 버리면 된다. 새 보드가 꽂혀 있든 옛 보드가 꽂혀 있든
    같은 창으로 몰 수 있다. (U 보드(joy2.ino)는 축 규격이 달라 여기서 안 다룬다 —
    그쪽이 필요하면 `nxde/joystick.py` 노드를 쓴다.)

════════════════════════════════════════════════════════════════════════════════
 조작 규약 — ★여기가 이 파일의 본론이다★
════════════════════════════════════════════════════════════════════════════════
    L스틱 위쪽   주행 펄스 0~15         (영점 기준, 데드존 밖에서만)
    L스틱 아래쪽 브레이크 0 / 1 / 2 단   (중앙~맨아래를 3등분)
    R스틱 좌우   조향 −40 ~ +40         ★− 좌 / + 우 (ROS·B보드 공통 규약)★
    SWA 짧게     시작 / 일시정지 토글   (소비하는 쪽이 `swa_releases` 로 받는다)

  ★부호가 뒤집히는 지점은 `command()` 의 조향 한 줄뿐이다★ 보드 내부 규격은
  'x 가 클수록 왼쪽' 이라 한 번 뒤집으면 ROS 규약이 된다. 두 번 뒤집으면 조용히
  좌우가 바뀐다(CLAUDE.md 2절).

════════════════════════════════════════════════════════════════════════════════
 Windows 원본에서 바뀐 것 (Ubuntu · 이 스택 규약)
════════════════════════════════════════════════════════════════════════════════
  · 포트 후보를 VID 표로 직접 만들지 않는다 → ★`arduino.candidate_ports()` 하나가
    소유한다★. GPS·IMU 의 VID/PID 와 udev 링크(/dev/gps·/dev/imu)가 그쪽에서 이미
    빠지므로, 남의 장치를 열어 리셋시키는 일이 없다.
  · ★배타 open(exclusive=True)★ 으로 연다 — A/B 보드를 arduino 노드가 이미 쥐고
    있으면 열리지도 않아 그 보드가 리셋되지 않는다.
  · ★A/B 보드로 판명된 포트는 즉시 놓아준다★ + ★실패한 포트는 PORT_COOLDOWN_S
    동안 다시 열지 않는다★ (joystick.py 가 2026-08-14 에 실차로 배운 것 —
    3초마다 전 포트를 두드리면 A/B 보드가 DTR 로 계속 리부팅되어 arduino 노드가
    보드를 잡지 못한다).
  · tkinter 를 들고 있지 않다. 화면은 이 값을 읽어 가는 쪽(master)의 몫이다.
"""

import statistics
import threading
import time

import serial

from nxde.arduino import candidate_ports


BAUD = 115200

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
RECONNECT_S = 3.0         # 못 찾았을 때 재탐색 간격
VERIFY_WINDOW_S = 3.5     # 후보 포트 하나당 "J," 를 기다려볼 최대 시간
PORT_COOLDOWN_S = 30.0    # '조이스틱이 아니다'로 판명된 포트를 다시 열기까지

JOY_PREFIX = 'J,'
# 접두어를 포함한 총 토큰 수. 9 = joy4.ino(신) / 12 = joy.ino(구, 뒤 3필드는 버린다)
JOY_FIELD_COUNTS = (9, 12)
# A보드 "S,좌,우,…" / B보드 "P,각,모드"·"STOP" — 이 줄이 보이면 남의 보드다
ARDUINO_PREFIXES = ('S,', 'P,', 'STOP')

_port_cooldown = {}       # {device: 이 시각 전에는 열지 않는다(monotonic)}


# ══════════════════════════════════════════════════════════════════════════════
#  파싱 · 스케일링
# ══════════════════════════════════════════════════════════════════════════════
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


def find_joystick_port(log, exclude=None):
    """조이스틱 포트 탐색 → (device, serial). 못 찾으면 (None, None).

    ★조이스틱은 A/B 보드와 같은 메가라 VID/PID·설명으로는 구분되지 않는다★
    실제로 열어서 "J," 줄이 오는지 확인한 뒤에만 채택한다. 아니면 즉시 닫는다 —
    포트를 열고 있는 동안 그 보드는 리셋된 채 멎어 있다.
    """
    now = time.monotonic()
    for device in candidate_ports(exclude=exclude):
        if now < _port_cooldown.get(device, 0.0):
            continue
        ser = None
        reason = None
        try:
            ser = serial.Serial(device, BAUD, timeout=1, exclusive=True)
            deadline = time.monotonic() + VERIFY_WINDOW_S
            while time.monotonic() < deadline:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith(JOY_PREFIX):
                    _port_cooldown.pop(device, None)
                    log(f"[조이스틱 연결] {device}")
                    ser.timeout = 0.2
                    return device, ser
                if line.startswith(ARDUINO_PREFIXES):
                    reason = 'A/B 보드'      # 3.5초를 기다리지 않고 즉시 놓아준다
                    break
        except (serial.SerialException, OSError):
            # 열리지 않는다 = 대개 arduino 노드가 배타 open 으로 이미 쥐고 있다.
            # ★이 경우가 가장 좋다★ — 열리지 않았으니 그 보드는 리셋되지 않았다.
            reason = '이미 다른 노드가 사용 중'
        if ser is not None:
            try:
                ser.close()
            except (serial.SerialException, OSError):
                pass
        _port_cooldown[device] = time.monotonic() + PORT_COOLDOWN_S
        log(f"[포트 건너뜀] {device} — {reason or '조이스틱 신호 없음'} "
            f"({PORT_COOLDOWN_S:.0f}초간 다시 열지 않는다)")
    return None, None


# ══════════════════════════════════════════════════════════════════════════════
#  리더
# ══════════════════════════════════════════════════════════════════════════════
class JoystickReader:
    """백그라운드 스레드로 조이스틱을 읽는다. 값은 `snapshot()` 한 덩어리로 준다.

    ★스레드 경계를 한 곳으로 모은 이유★ 소비하는 쪽(tkinter)은 위젯을 메인
    스레드에서만 만질 수 있다. 그래서 여기서 lock 을 잡아 한 번에 복사해 주고,
    화면은 그 복사본만 보고 그린다(master.py 와 joystick.py 가 같은 규약).
    """

    def __init__(self, log=print, exclude=None, deadzone=DEADZONE_RAW,
                 pulse_max=PULSE_MAX):
        self.log = log
        self.exclude = [str(p) for p in (exclude or []) if str(p)]
        self.deadzone = int(deadzone)
        self.pulse_max = max(1, min(PULSE_MAX, int(pulse_max)))

        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self._ser = None

        self.port = None
        self.data = None            # 마지막 8튜플
        self.data_t = 0.0
        self.connected = False

        self.calibrated = False
        self.calib_buf = None
        self.center = [DEFAULT_CENTER] * 4      # x1, y1, x2, y2
        self._reset_at = None

        # ★SWA 릴리즈 엣지를 세어 둔다★ 소비하는 쪽(20Hz 틱)이 짧은 누름을
        #   놓치지 않게, 엣지를 '지금 눌려 있나'가 아니라 ★횟수★ 로 넘긴다.
        self.swa_releases = 0
        self._swa_prev = 1

    # ── 수명 ──────────────────────────────────────────────────────────
    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, name='joyread', daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        thread, self._thread = self._thread, None
        if thread is not None:
            thread.join(timeout=1.5)
        self._close()

    def _close(self):
        with self._lock:
            ser, self._ser = self._ser, None
            self.connected = False
            self.data = None
            self.calibrated = False
            self.calib_buf = None
            self.port = None
        if ser is not None:
            try:
                ser.close()
            except (serial.SerialException, OSError):
                pass

    # ── 리더 스레드 ───────────────────────────────────────────────────
    def _loop(self):
        while self._running:
            if self._ser is None:
                device, ser = find_joystick_port(self.log, exclude=self.exclude)
                if ser is None:
                    self.log(f"조이스틱 보드를 찾지 못했습니다 — {RECONNECT_S:.0f}초 후 "
                             f"재시도 (연결·전원·펌웨어 확인)")
                    for _ in range(int(RECONNECT_S * 10)):
                        if not self._running:
                            return
                        time.sleep(0.1)
                    continue
                with self._lock:
                    self._ser, self.port = ser, device
                    self.connected = True
                    # ★재연결이면 영점을 다시 잡는다★ 보드가 리셋됐을 수 있다
                    self.calibrated = False
                    self.calib_buf = {'x1': [], 'y1': [], 'x2': [], 'y2': []}
                    self._reset_at = None

            ser = self._ser          # ★지역 참조★ 다른 스레드가 None 으로 떨어뜨린다
            if ser is None:
                continue
            try:
                raw = ser.readline()
            except (serial.SerialException, OSError) as e:
                self.log(f"조이스틱 단절: {e} → 재연결 시도")
                self._close()
                continue
            if not raw:
                continue
            text = raw.decode('utf-8', errors='ignore').strip()
            if not text:
                continue

            if 'RESET' in text:
                # 보드가 방금 (재)부팅했다 — 스틱이 제자리로 돌아올 시간을 준 뒤 재보정
                with self._lock:
                    self._reset_at = time.monotonic()
                self.log(f"조이스틱 RESET 감지 — {RESET_CALIB_DELAY:.0f}초 뒤 "
                         f"영점을 다시 잡습니다")
                continue

            parsed = parse_joy_line(text)
            if parsed is None:
                continue
            with self._lock:
                self.data = parsed
                self.data_t = time.monotonic()
                if self._swa_prev == 0 and parsed[6] == 1:      # active-low 릴리즈
                    self.swa_releases += 1
                self._swa_prev = parsed[6]
                self._step_calibration_locked(parsed)

        self._close()

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

        if self.calibrated or self.calib_buf is None:
            return

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
        """화면·제어가 한 번에 읽어 가는 상태. ★lock 은 여기서만 잡는다★

        `swa_releases` 는 ★소비하면 0 으로 비운다★ — 같은 누름이 두 번 먹지 않게
        하는 규약이고, 그래서 이 함수는 한 틱에 한 번만 불러야 한다.
        """
        with self._lock:
            releases, self.swa_releases = self.swa_releases, 0
            fresh = (self.data is not None
                     and (time.monotonic() - self.data_t) <= STALE_INPUT_S)
            return {
                'connected':  self.connected,
                'port':       self.port,
                'calibrated': self.calibrated,
                'calib_n':    len(self.calib_buf['y1']) if self.calib_buf else 0,
                'data':       self.data,
                'fresh':      fresh,
                'center':     tuple(self.center),
                'swa_releases': releases,
            }

    def gate_reason(self, snap):
        """지금 이 입력을 쓸 수 없는 이유. 없으면 None. (차량 쪽 사정은 보지 않는다)"""
        if not snap['connected'] or snap['data'] is None:
            return "조이스틱 연결 중..."
        if not snap['fresh']:
            return "입력 끊김"
        if not snap['calibrated']:
            return f"영점 잡는 중 ({snap['calib_n']}/{CALIBRATION_SAMPLES}) — 스틱을 놓으십시오"
        return None

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
