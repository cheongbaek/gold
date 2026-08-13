#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""joystick — 조이스틱 메가 보드로 차량을 조종한다 (white 규약 토픽 직접 발행)

    ros2 run nxde arduino      ← 먼저 (또는 one_launch.py 가 함께 띄운 것)
    ros2 run nxde joystick     ← 이 노드

╔══════════════════════════════════════════════════════════════════════════════╗
║  ★★ 안전장치 2개 — 반드시 알고 쓸 것 (2026-08-05 지정) ★★                      ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  ① ★주행모드 스위치(B보드 D5)가 '자율주행'일 때만 작동한다★                    ║
║     /vehicle_mode == False(수동조종)면 이 노드는 입력을 무시하고 정지값만 낸다.  ║
║     수동조종에서는 사람이 페달·핸들을 잡고 있고 arduino 노드가 그 경로를 직접    ║
║     넘긴다 — 그때 조이스틱까지 명령을 쏘면 사람과 싸운다.                        ║
║     ⚠️ 수동으로 내려갔다 자율로 돌아오면 ★일시정지 상태로 재무장★ 된다.          ║
║        다시 움직이려면 SWA 를 한 번 더 눌러야 한다(의도치 않은 재출발 방지).      ║
║                                                                              ║
║  ② ★첫 실행 시 영점을 먼저 잡고, SWA 를 한 번 눌러야 입력이 반영된다★           ║
║     · 연결되면 스틱을 건드리지 않은 상태에서 CALIBRATION_SAMPLES 개를 모아        ║
║       중앙값을 영점으로 잡는다 (그동안 어떤 명령도 나가지 않는다).                ║
║     · 영점이 끝나기 전에는 SWA 를 눌러도 일시정지가 풀리지 않는다.                ║
║     · 영점 후 SWA 짧게 누름 = 시작/일시정지 토글.                               ║
║     이유: 스틱이 물리적으로 정확히 중앙(512)에 있지 않다. 영점 없이 시작하면      ║
║     '가만히 둔 스틱'이 이미 펄스를 요구하는 상태일 수 있다.                      ║
╚══════════════════════════════════════════════════════════════════════════════╝

═══════════════════════════════════════════════════════════════════════════════
 조작
═══════════════════════════════════════════════════════════════════════════════
   L스틱 위    주행 펄스 0~15   (영점 기준, 데드존 밖에서만)
   L스틱 아래  브레이크 단계     중앙~맨아래를 3등분 → 0 / 1 / 2 (리니어모터)
   R스틱 좌우  조향각 −40~40    ★− 좌 / + 우 (white·kasa ROS 규약)★
   SWA 짧게    시작 / 일시정지 토글  (영점 완료 후에만)
               ※ U 보드에는 SWA 가 없어 ★L·R 스틱 버튼 동시 누름★ 이 대신한다
   메가 리셋   3초 뒤 그 시점 값으로 영점 재보정

 조이스틱이 끊기면 입력 상실이므로 즉시 정지 + 일시정지로 떨어지고 3초마다 재연결한다.
 종료 시에도 정지값을 발행한다 — A보드에는 무입력 타임아웃이 없어서 마지막 명령을
 arduino 노드가 1초마다 계속 재전송한다(= 안 내면 차가 계속 간다).

═══════════════════════════════════════════════════════════════════════════════
 kasa_ws 원본(nxde/joystick.py)에서 바뀐 점
═══════════════════════════════════════════════════════════════════════════════
  · 통신 양식 : /in (String "주행 조향 리니어") → ★white 규약 토픽★
        /cmd_vel_raw (Twist)  linear.x = 주행펄스 0~15 / angular.z = 조향각 −40~40
        /control_state (Bool) True = 구동 허용 / False = 정지
        /brake_level (Int32)  0 / 1 / 2
  · ★A0 펄스 모드 · PWM 테스트 모드 · 좌/우 독립 출력을 제거했다★
      arduino.py 가 A보드로 항상 '단일값'만 보내기 때문이다. 직접 PWM(16~255)은
      PID·슬루레이트·폭주감지가 전부 빠지는 무보호 경로라 이 스택에서 봉쇄했다.
      (그래서 D13·SWB 에 걸려 있던 기능도 함께 없어졌다. 좌우 차동도 하지 않는다.)
  · [2026-08-14] ★터미널 한 줄 표시 → tkinter GUI★ 로 바꿨다(joystick.real.py 의
    화면을 이 스택에 맞춰 옮겼다). 스틱 위치·스위치·게이트 사유가 한눈에 보인다.
    ★조종 로직은 그대로다★ — GUI 는 값을 읽어 그리기만 하고, 발행은 종전과 같이
    ROS 타이머(20Hz)가 한다. 위젯을 건드리는 것은 메인 스레드의 after() 틱뿐이다
    (tkinter 는 스레드 안전하지 않다 — master.py·prompt_g.py 와 같은 규약).
  · ★SWB 3초 조향 캘리브레이션은 넣지 않았다★ B보드(kasa_0813_B.ino)에 'a' 명령이
    있지만, 그것은 IDE 로 직접 실행하는 절차로 둔다. 화면에 SWB 칸은 남기되
    '미배정'으로 표시한다.
  · 위 안전장치 ①(자율주행 모드 게이트)이 새로 추가되었다.

 지원 보드는 원본과 같다 — 접두어로 구분한다(둘 다 메가라 VID/PID 로는 구분 불가):
   "J," = joy.ino  : x1,y1,k1,x2,y2,k2,swa,swb,d12,d13,a0   (11필드)
   "U," = joy2.ino : lx,ly,lsw,rx,ry,rsw                    (6필드)
     축 규격이 달라(lx/rx=세로, ly/ry=가로) 수신 즉시 J 규격으로 변환한다.
     U 보드에는 SWA 가 없으므로 ★L·R 스틱 버튼 동시 누름★ 이 SWA 를 대신한다.
"""

import math
import signal
import statistics
import threading
import time
import tkinter as tk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Twist

import serial

from nxde.proc_guard import watch_parent
# 포트 후보 목록은 arduino.py 가 소유한다 (같은 패키지 — 표를 두 벌로 만들지 않는다).
# GPS/IMU VID/PID 와 udev 링크(/dev/gps · /dev/imu)는 그쪽에서 이미 제외된다.
from nxde.arduino import candidate_ports


BAUD = 115200

SAMPLE_HZ = 20                # 제어/발행 주기 (arduino 노드의 TX 판정 20Hz 와 맞춘다)
CONTROL_PERIOD_S = 1.0 / SAMPLE_HZ

CALIBRATION_SAMPLES = 20      # 영점(중앙값) 보정에 사용할 샘플 개수
RECONNECT_S = 3.0             # 조이스틱이 끊기면 이 간격으로 재연결 시도
RESET_CALIB_DELAY = 3.0       # 메가 RESET 수신 후 몇 초 뒤 값을 영점으로 삼을지
STALE_INPUT_S = 0.6           # 이 시간 이상 조이스틱 줄이 안 오면 '입력 상실'로 본다
GUI_PERIOD_MS = 50            # 화면 갱신 주기 (제어 20Hz 와 같은 결)

PULSE_MAX = 15                # A보드 단일값 입력 상한
STEER_MAX = 40                # B보드 STEER_ANGLE_MAX
BRAKE_LEVEL_MAX = 2           # 0 = 놓음 / 1 = 약(1/3) / 2 = 풀
ADC_MAX = 1023

DEADZONE_RAW = 120            # 영점 기준 raw ADC 편차가 이 값 미달이면 0으로 처리
DEFAULT_CENTER = ADC_MAX // 2

# ── 화면 (joystick.real.py 의 배치를 그대로 옮겼다) ──
PAD      = 240     # 스틱 패드 한 변
DOT_R    = 9       # 스틱 위치 점
CENTER_R = 12      # 스틱 버튼(누름) 표시 원
SW_R     = 15      # 스위치 원
POT_R    = 28      # A0 다이얼 반지름 (표시 전용)
POT_GAP  = 90      # 다이얼의 '입'(회전 불가 구간) 각도

BG          = '#1e1e1e'
PAD_BG      = '#2b2b2b'
LINE        = '#4a4a4a'
DOT_COLOR   = '#4fc3f7'
PRESS_COLOR = '#ef5350'
IDLE_COLOR  = '#3a3a3a'
OK_COLOR    = '#66bb6a'
WARN_COLOR  = '#ffb454'
TEXT        = '#dddddd'
# U 보드에 물리적으로 없는 칸 — 지우지 않고 회색으로 죽인다(레이아웃 유지)
DEAD_FILL   = '#242424'
DEAD_LINE   = '#333333'
DEAD_TEXT   = '#5a5a5a'

# 접두어 한 글자로 보드를 구분한다 (JOY_FIELD_COUNT 는 접두어 포함 총 필드 수)
JOY_KINDS = ('J', 'U')
JOY_FIELD_COUNT = {'J': 12, 'U': 7}
VERIFY_WINDOW_S = 3.5         # 후보 포트 하나당 "J,"/"U," 줄을 기다려볼 최대 시간


# ══════════════════════════════════════════════════════════════════════════════
#  파싱 · 스케일링 (kasa_ws 원본과 동일 — 검증된 로직이라 손대지 않았다)
# ══════════════════════════════════════════════════════════════════════════════
def joy_kind_of(line):
    """줄의 접두어로 보드 종류('J'/'U')를 판정. 조이스틱 줄이 아니면 None."""
    for kind in JOY_KINDS:
        if line.startswith(kind + ','):
            return kind
    return None


def parse_joy_line(line, kind):
    """조이스틱 한 줄 → 내부 공통 11튜플 (x1,y1,k1,x2,y2,k2,swa,swb,d12,d13,a0).

    해당 보드의 데이터 줄이 아니거나 형식이 깨졌으면 None.

    U 보드(joy2.ino)는 축 규격이 반대라 여기서 J 규격으로 맞춘다:
      joy2 : lx/rx = 세로(1023이 위),   ly/ry = 가로(1023이 오른쪽)
      내부 : y      = 세로(1023이 위),   x     = 가로(★1023이 왼쪽★)
    없는 입력(SWB/D12/D13/A0)은 '안 눌림·0'으로 채운다. 단 SWA 자리에는
    'L·R 스틱 버튼 동시 누름'을 넣는다 — 그러지 않으면 U 보드에서 일시정지를
    풀 수단이 아예 없어진다(안전장치 ② 때문에 SWA 는 필수 입력이다).
    """
    parts = line.split(',')
    if len(parts) != JOY_FIELD_COUNT.get(kind, -1) or parts[0] != kind:
        return None
    try:
        vals = tuple(map(int, parts[1:]))
    except ValueError:
        return None
    if kind == 'J':
        return vals
    lx, ly, lsw, rx, ry, rsw = vals
    both = 0 if (lsw == 0 and rsw == 0) else 1   # active-low: 둘 다 눌렸을 때만 0
    return (ADC_MAX - ly, lx, lsw,
            ADC_MAX - ry, rx, rsw,
            both, 1, 1, 1, 0)


def normalize(raw, center):
    """center 기준으로 양쪽 구간을 각각 독립 비율로 −1..1 에 매핑.

    (영점이 정확히 512가 아니므로 양쪽 구간 길이가 다르다)"""
    center = max(1, min(ADC_MAX - 1, center))
    if raw >= center:
        return (raw - center) / (ADC_MAX - center)
    return (raw - center) / center


def scaled_with_deadzone(raw, center, deadzone_raw, max_out):
    """center 기준 편차가 deadzone_raw 미달이면 0. 그 밖은 (1..max_out)로 재매핑."""
    center = max(1, min(ADC_MAX - 1, center))
    diff = raw - center
    span = (ADC_MAX - center) if diff >= 0 else center
    mag = abs(diff)
    if mag < deadzone_raw:
        return 0.0
    remain = span - deadzone_raw
    magnitude = max_out if remain <= 0 else 1 + (mag - deadzone_raw) / remain * (max_out - 1)
    return magnitude if diff >= 0 else -magnitude


def find_and_verify_joystick_port(logger):
    """조이스틱 포트 탐색. (device, serial, kind) — 못 찾으면 (None, None, None).

    ★조이스틱은 A/B 보드와 같은 메가라 VID/PID·description 으로는 구분이 안 된다★
    후보 포트를 실제로 열어 "J,"/"U," 줄이 오는지 확인한 뒤에만 채택한다.
    아니면 곧바로 닫아준다(포트를 열면 보드가 리셋되므로 오래 붙잡지 않는다).

    A/B 보드는 arduino 노드가 'S,'/'P,' 로 식별해 가져가는데, 그 탐색과 여기 탐색이
    같은 포트를 동시에 두드릴 수 있다. 서로 배타 open 으로 튕기고 각자 재시도하므로
    결국 각자 제 보드를 잡는다(원본에서도 같은 방식으로 공존했다).
    """
    for device in candidate_ports():
        ser = None
        try:
            ser = serial.Serial(device, BAUD, timeout=1, exclusive=True)
            deadline = time.monotonic() + VERIFY_WINDOW_S
            while time.monotonic() < deadline:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                kind = joy_kind_of(line)
                if kind is not None:
                    logger.info(f"[조이스틱 연결] {device} ({kind} 보드)")
                    return device, ser, kind
        except (serial.SerialException, OSError):
            pass
        if ser is not None:
            try:
                ser.close()
            except (serial.SerialException, OSError):
                pass
    return None, None, None


# ══════════════════════════════════════════════════════════════════════════════
#  노드
# ══════════════════════════════════════════════════════════════════════════════
class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick')

        self.deadzone = int(self.declare_parameter('deadzone_raw', DEADZONE_RAW).value)
        # 조이스틱으로 낼 수 있는 최대 펄스. ★초기 시험에서는 3~5 로 낮출 것★
        #   기본 15 는 A보드 상한(≈47km/h)이라 실내·저속 시험에는 과하다.
        self.pulse_max = max(1, min(PULSE_MAX,
                                    int(self.declare_parameter('pulse_max', 5).value)))
        # 자율주행 모드 게이트를 끌 수 있게 해 두지만 ★기본은 켜짐★ 이다.
        #   벤치에서 보드만 놓고 시험할 때를 위한 탈출구다 — 실차에서 끄지 말 것.
        self.require_auto_mode = bool(
            self.declare_parameter('require_auto_mode', True).value)

        # ── 발행 (white 규약) ──
        self.pub_cmd   = self.create_publisher(Twist, '/cmd_vel_raw',    10)
        self.pub_state = self.create_publisher(Bool,  '/control_state',  10)
        self.pub_brake = self.create_publisher(Int32, '/brake_level',    10)

        # ── 구독 : 주행모드 스위치 (안전장치 ①) ──
        self.auto_mode = None            # None = 아직 미수신
        self.create_subscription(Bool, '/vehicle_mode', self._cb_mode, 10)
        self.estop = False
        self.create_subscription(Bool, '/estop', self._cb_estop, 10)

        # ── 조이스틱 상태 ──
        self.lock = threading.Lock()
        self.data = None                 # 최근 파싱된 11튜플
        self.data_t = 0.0                # 최근 수신 시각
        self.connected = False
        self.kind = None
        self.ser = None

        # 영점 (안전장치 ②)
        self.calibrated = False
        self.calib_buf = None
        self.center_x1 = self.center_y1 = DEFAULT_CENTER
        self.center_x2 = self.center_y2 = DEFAULT_CENTER
        self.reset_at = None             # 메가 RESET 수신 시각 (3초 뒤 재보정)

        # 조작 상태
        self.paused = True               # ★시작은 항상 일시정지★
        self.swa_prev = 1                # active-low (1 = 안 눌림)
        self.rearm_reason = "첫 실행"     # 왜 일시정지인지 (터미널 표시용)

        # ★GUI 표시 전용★ 제어틱이 마지막으로 발행한 값을 남긴다(제어에는 쓰지 않는다)
        self.last_pulse = 0
        self.last_steer = 0
        self.last_brake = 0

        self._running = True
        self._reader = threading.Thread(target=self._reader_loop,
                                        name='joy_serial', daemon=True)
        self._reader.start()

        self.create_timer(CONTROL_PERIOD_S, self._control_tick)

        # 부모(런치) 사망 시에도 정지값을 내보내고 끝낸다
        #   ※ 이 콜백 안에서 print 하지 않는다 — proc_guard.py 헤더 경고 참고
        watch_parent(cleanup=self.stop_and_close)

        self.get_logger().info(
            f"joystick 시작 — pulse_max={self.pulse_max} deadzone={self.deadzone} "
            f"자율모드게이트={'ON' if self.require_auto_mode else '★OFF★'}")
        self.get_logger().info(
            "★스틱을 건드리지 말고 기다리십시오 — 영점을 잡습니다. "
            "그 다음 SWA 를 한 번 누르면 시작합니다★")

    # ── 콜백 ──────────────────────────────────────────────────────────
    def _cb_mode(self, msg: Bool):
        """B보드 D5 : True 자율주행 / False 수동조종.

        ★자율 → 수동으로 내려가면 즉시 일시정지로 재무장한다★ 수동에서 사람이 페달을
        잡고 있는데 조이스틱이 명령을 쏘면 서로 싸운다. 자율로 돌아와도 자동으로
        재개하지 않는다 — SWA 를 다시 눌러야 한다(의도치 않은 재출발 방지)."""
        new = bool(msg.data)
        if self.auto_mode is not None and new != self.auto_mode:
            self.get_logger().info(
                f"[주행모드] {'자율주행' if new else '수동조종'} 전환")
        if not new and not self.paused:
            self.paused = True
            self.rearm_reason = "수동조종 전환"
            self.get_logger().warn(
                "수동조종으로 전환됨 → 조이스틱 일시정지. "
                "자율로 돌린 뒤 SWA 를 다시 누르십시오.")
        self.auto_mode = new

    def _cb_estop(self, msg: Bool):
        new = bool(msg.data)
        if new and not self.paused:
            self.paused = True
            self.rearm_reason = "E-STOP"
            self.get_logger().warn("E-STOP 발동 → 조이스틱 일시정지 (해제 후 SWA 재입력)")
        self.estop = new

    # ── 시리얼 리더 스레드 ────────────────────────────────────────────
    def _reader_loop(self):
        while self._running and rclpy.ok():
            if self.ser is None:
                dev, ser, kind = find_and_verify_joystick_port(self.get_logger())
                if ser is None:
                    self.get_logger().warn(
                        f"조이스틱 보드를 찾지 못했습니다 — {RECONNECT_S}s 후 재시도 "
                        f"(연결·전원·펌웨어 확인)", throttle_duration_sec=15.0)
                    time.sleep(RECONNECT_S)
                    continue
                ser.timeout = 0.2
                with self.lock:
                    self.ser, self.kind = ser, kind
                    self.connected = True
                    # 재연결이면 영점을 다시 잡는다 — 보드가 리셋됐을 수 있다
                    self.calibrated = False
                    self.calib_buf = {'x1': [], 'y1': [], 'x2': [], 'y2': []}
                    self.paused = True
                    self.rearm_reason = "재연결"

            # ★지역 참조로 받는다★ _drop_serial 이 다른 스레드에서 self.ser 를 None 으로
            #   떨어뜨릴 수 있어서, 그대로 쓰면 AttributeError 로 리더 스레드가 죽는다.
            ser = self.ser
            if ser is None:
                continue
            try:
                raw = ser.readline()
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f"조이스틱 단절: {e} → 재연결 시도")
                self._drop_serial()
                continue
            if not raw:
                continue

            text = raw.decode('utf-8', errors='ignore').strip()
            if not text:
                continue

            # 메가 리셋 버튼 → 3초 뒤 그 시점 값을 영점으로 재보정
            if 'RESET' in text:
                with self.lock:
                    self.reset_at = time.monotonic()
                self.get_logger().info(
                    f"조이스틱 RESET 감지 — {RESET_CALIB_DELAY:.0f}초 뒤 영점을 다시 잡습니다")
                continue

            kind = joy_kind_of(text)
            if kind is None:
                continue
            parsed = parse_joy_line(text, kind)
            if parsed is None:
                continue

            with self.lock:
                self.kind = kind
                self.data = parsed
                self.data_t = time.monotonic()
                self._step_calibration_locked(parsed)

    def _drop_serial(self):
        with self.lock:
            if self.ser is not None:
                try:
                    self.ser.close()
                except (serial.SerialException, OSError):
                    pass
            self.ser = None
            self.connected = False
            self.data = None
            self.calibrated = False
            self.calib_buf = None
            self.paused = True
            self.rearm_reason = "입력 상실"
        # 입력을 잃었으므로 즉시 정지값을 낸다
        self._publish_stop()

    # ── 영점 (안전장치 ②) ─────────────────────────────────────────────
    def _step_calibration_locked(self, parsed):
        """lock 을 잡은 상태에서 호출된다. 샘플을 모아 중앙값을 영점으로 잡는다."""
        x1, y1, _k1, x2, y2 = parsed[0], parsed[1], parsed[2], parsed[3], parsed[4]

        # RESET 후 지연 재보정 : 지연이 지나면 버퍼를 새로 시작한다
        if self.reset_at is not None:
            if time.monotonic() - self.reset_at < RESET_CALIB_DELAY:
                return
            self.reset_at = None
            self.calibrated = False
            self.calib_buf = {'x1': [], 'y1': [], 'x2': [], 'y2': []}
            self.paused = True
            self.rearm_reason = "RESET 후 재보정"

        if self.calibrated or self.calib_buf is None:
            return

        self.calib_buf['x1'].append(x1)
        self.calib_buf['y1'].append(y1)
        self.calib_buf['x2'].append(x2)
        self.calib_buf['y2'].append(y2)
        if len(self.calib_buf['y1']) < CALIBRATION_SAMPLES:
            return

        self.center_x1 = statistics.median(self.calib_buf['x1'])
        self.center_y1 = statistics.median(self.calib_buf['y1'])
        self.center_x2 = statistics.median(self.calib_buf['x2'])
        self.center_y2 = statistics.median(self.calib_buf['y2'])
        self.calib_buf = None
        self.calibrated = True
        self.get_logger().info(
            f"✅ 영점 완료 — L=({self.center_x1:.0f},{self.center_y1:.0f}) "
            f"R=({self.center_x2:.0f},{self.center_y2:.0f})  "
            f"★이제 SWA 를 한 번 누르면 시작합니다★")

    # ── 제어 ─────────────────────────────────────────────────────────
    def _gate_reason(self):
        """지금 입력을 반영할 수 없는 이유. 없으면 None."""
        if not self.connected or self.data is None:
            return "조이스틱 미연결"
        if (time.monotonic() - self.data_t) > STALE_INPUT_S:
            return "입력 끊김"
        if not self.calibrated:
            return "영점 미완료"
        if self.require_auto_mode and self.auto_mode is not True:
            return ("주행모드 미수신(B보드 확인)" if self.auto_mode is None
                    else "수동조종 모드")
        if self.estop:
            return "E-STOP"
        return None

    def _control_tick(self):
        with self.lock:
            data = self.data
            gate = self._gate_reason()

            # SWA 릴리즈 에지 = 시작/일시정지 토글 (영점 완료 후에만)
            if data is not None:
                swa = data[6]        # active-low : 0 = 눌림
                if swa == 1 and self.swa_prev == 0:      # 릴리즈
                    if not self.calibrated:
                        self.get_logger().warn(
                            "SWA 입력 무시 — 영점이 아직 안 끝났습니다 "
                            "(스틱을 건드리지 말고 잠시 기다리십시오)")
                    elif gate is not None and self.paused:
                        self.get_logger().warn(f"SWA 입력 무시 — {gate}")
                    else:
                        self.paused = not self.paused
                        if self.paused:
                            self.rearm_reason = "SWA 일시정지"
                            self.get_logger().info("⏸ 일시정지")
                        else:
                            self.get_logger().info("▶ 시작 — 입력이 반영됩니다")
                self.swa_prev = swa

            paused = self.paused
            if data is None or gate is not None or paused:
                self._publish_stop()
                return

            y1, x2 = data[1], data[3]
            cy1, cx2 = self.center_y1, self.center_x2

        # ── 주행 펄스 : L스틱 위쪽만 (아래쪽은 브레이크로 쓴다) ──
        pulse = int(round(max(0.0, scaled_with_deadzone(
            y1, cy1, self.deadzone, self.pulse_max))))
        pulse = max(0, min(self.pulse_max, pulse))

        # ── 브레이크 : L스틱 아래, 중앙~맨아래를 3등분 → 0/1/2 ──
        down = -normalize(y1, cy1)          # 아래로 내리면 0..1
        if down >= 2.0 / 3.0:
            brake = 2
        elif down >= 1.0 / 3.0:
            brake = 1
        else:
            brake = 0

        # ── 조향 : R스틱 가로 ──
        #   내부 규격은 'x 가 클수록 왼쪽'이라 부호를 뒤집으면 ROS 규약(− 좌 / + 우)이 된다.
        #   ★부호가 뒤집히는 지점은 이 한 줄뿐이다★ arduino.py 는 그대로 보드에 넘긴다.
        steer = int(round(-scaled_with_deadzone(x2, cx2, self.deadzone, STEER_MAX)))
        steer = max(-STEER_MAX, min(STEER_MAX, steer))
        self.last_steer = steer

        brake = max(0, min(BRAKE_LEVEL_MAX, brake))
        self.last_pulse, self.last_brake = pulse, brake

        msg = Twist()
        msg.linear.x = float(pulse)         # ★m/s 가 아니라 주행 목표펄스★
        msg.angular.z = float(steer)
        self.pub_cmd.publish(msg)
        self.pub_state.publish(Bool(data=True))
        self.pub_brake.publish(Int32(data=brake))

    def _publish_stop(self):
        """정지값 발행. ★발행을 '멈추면' 안 된다★

        A보드 펌웨어에는 무입력 타임아웃이 없다 — arduino 노드가 마지막 명령을
        1초마다 계속 재전송하므로, 발행을 그냥 끊으면 차가 계속 간다.
        조향은 0 을 넣어도 arduino 가 control_state=False 에서 마지막 각도를
        유지한다(주행 중 0도로 급조향하면 위험하므로 그쪽이 맞다).
        """
        self.last_pulse = self.last_brake = 0
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub_cmd.publish(msg)
        self.pub_state.publish(Bool(data=False))
        self.pub_brake.publish(Int32(data=0))

    # ── 화면이 읽어 갈 상태 한 덩어리 ────────────────────────────────
    def snapshot(self):
        """GUI 가 한 번에 읽어 가는 상태. ★lock 은 여기서만 잡는다★

        위젯을 만지는 쪽(메인 스레드)과 값을 바꾸는 쪽(리더 스레드·ROS 타이머)이
        다르므로, 화면은 매 틱 이 한 덩어리를 받아 그리기만 한다.
        """
        with self.lock:
            data = self.data
            gate = self._gate_reason()
            return {
                'connected':  self.connected,
                'kind':       self.kind,
                'calibrated': self.calibrated,
                'calib_n':    len(self.calib_buf['y1']) if self.calib_buf else 0,
                'paused':     self.paused,
                'reason':     self.rearm_reason,
                'gate':       gate,
                'data':       data,
                'auto_mode':  self.auto_mode,
                'estop':      self.estop,
                'center':     (self.center_x1, self.center_y1,
                               self.center_x2, self.center_y2),
                'pulse':      self.last_pulse,
                'steer':      self.last_steer,
                'brake':      self.last_brake,
            }

    # ── 종료 ─────────────────────────────────────────────────────────
    def stop_and_close(self):
        """종료 정리. 창 닫기/부모 사망/Ctrl+C 어디서 와도 정지값을 낸다."""
        if not self._running:
            return
        self._running = False
        try:
            self._publish_stop()
            time.sleep(0.15)        # 정지값이 실제로 DDS 로 나갈 시간
        except Exception:
            pass
        with self.lock:
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None



# ══════════════════════════════════════════════════════════════════════════════
#  화면 (joystick.real.py 의 배치를 이 스택에 맞춰 옮겼다)
#    ★그리기만 한다★ 발행·판단은 전부 노드가 한다. 여기서 조종값을 만들지 않는다.
# ══════════════════════════════════════════════════════════════════════════════
class JoystickGui:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.running = True
        self.kind_applied = None      # 보드 종류 전환을 감지해 칸 활성/비활성을 반영

        root.title("nxde joystick — 조이스틱 조종")
        root.configure(bg=BG)
        root.protocol('WM_DELETE_WINDOW', self.on_close)

        main = tk.Frame(root, bg=BG)
        main.pack(padx=16, pady=(14, 8))

        self.pad_l = self._make_pad(main, 'L  (주행/제동)', column=0, highlight='h')
        self._make_switch_column(main)
        self.pad_r = self._make_pad(main, 'R  (조향)',      column=2, highlight='v')

        # ── 아래쪽 : 상태 3줄 ──
        # ★textvariable 을 쓰지 않는다★ master.py 와 같이 라벨을 직접 config 한다.
        #   textvariable + config 를 섞었더니 이 환경의 Tk 가 'Tcl_Release couldn't find
        #   reference' 로 죽었다(전체 _tick 조합에서만 재현). 굳이 두 경로를 섞을 이유가
        #   없어 검증된 쪽(master.py)으로 통일했다.
        self.state_label = tk.Label(root, text='시작 중…', bg=BG, fg=TEXT,
                                    font=('Consolas', 13, 'bold'), wraplength=820)
        self.state_label.pack(pady=(2, 2))

        self.gate_label = tk.Label(root, text='', bg=BG, fg=TEXT,
                                   font=('Consolas', 11), wraplength=820)
        self.gate_label.pack()

        self.cmd_label = tk.Label(root, text='발행: -', bg=BG, fg=DOT_COLOR,
                                  font=('Consolas', 12, 'bold'))
        self.cmd_label.pack(pady=(4, 12))

        self._tick()

    # ── 위젯 만들기 ───────────────────────────────────────────────────
    def _make_pad(self, parent, label, column, highlight):
        frame = tk.Frame(parent, bg=BG)
        frame.grid(row=0, column=column, padx=10)
        tk.Label(frame, text=label, bg=BG, fg=TEXT,
                 font=('Consolas', 12, 'bold')).pack()

        c = tk.Canvas(frame, width=PAD, height=PAD, bg=PAD_BG,
                      highlightthickness=1, highlightbackground=LINE)
        c.pack()
        # 그 패드가 실제로 쓰는 축만 색을 준다 — L 은 세로(주행/제동), R 은 가로(조향)
        if highlight == 'h':
            c.create_line(PAD / 2, 0, PAD / 2, PAD, fill=LINE)
            hi = c.create_line(0, PAD / 2, PAD, PAD / 2, fill=DOT_COLOR)
        else:
            hi = c.create_line(PAD / 2, 0, PAD / 2, PAD, fill=DOT_COLOR)
            c.create_line(0, PAD / 2, PAD, PAD / 2, fill=LINE)
        k = c.create_oval(PAD / 2 - CENTER_R, PAD / 2 - CENTER_R,
                          PAD / 2 + CENTER_R, PAD / 2 + CENTER_R, fill='', outline=LINE)
        dot = c.create_oval(0, 0, 0, 0, fill=DOT_COLOR, outline='')

        lbl = tk.Label(frame, text='X:---- Y:----', bg=BG, fg=TEXT, font=('Consolas', 10))
        lbl.pack()
        return {'canvas': c, 'dot': dot, 'k': k, 'hi': hi,
                'label': lbl, 'highlight': highlight}

    def _make_switch_column(self, parent):
        frame = tk.Frame(parent, bg=BG)
        frame.grid(row=0, column=1, padx=10, sticky='s')
        self.pot = self._make_pot_dial(frame)
        self.toggle_ids = self._make_switch_row(frame, ('D12', 'D13'))
        self.sw_ids = self._make_switch_row(frame, ('SWA', 'SWB'))

    def _make_switch_row(self, parent, names):
        row = tk.Frame(parent, bg=BG)
        row.pack(pady=(0, 6))
        ids = {}
        for name in names:
            box = tk.Frame(row, bg=BG)
            box.pack(side=tk.LEFT, padx=6)
            c = tk.Canvas(box, width=SW_R * 2 + 6, height=SW_R * 2 + 6,
                          bg=BG, highlightthickness=0)
            c.pack()
            oid = c.create_oval(3, 3, SW_R * 2 + 3, SW_R * 2 + 3,
                                fill=IDLE_COLOR, outline=LINE)
            lbl = tk.Label(box, text=name, bg=BG, fg=TEXT, font=('Consolas', 9))
            lbl.pack()
            ids[name] = (c, oid, lbl)
        return ids

    def _make_pot_dial(self, parent):
        frame = tk.Frame(parent, bg=BG)
        frame.pack(pady=(0, 8))
        size = POT_R * 2 + 10
        cx = cy = size / 2
        c = tk.Canvas(frame, width=size, height=size, bg=BG, highlightthickness=0)
        c.pack()
        arc = c.create_arc((cx - POT_R, cy - POT_R, cx + POT_R, cy + POT_R),
                           start=90 + POT_GAP / 2, extent=360 - POT_GAP,
                           fill=PAD_BG, outline=LINE)
        needle = c.create_line(cx, cy, cx, cy - POT_R, fill=DOT_COLOR, width=3)
        lbl = tk.Label(frame, text='A0:----', bg=BG, fg=TEXT, font=('Consolas', 9))
        lbl.pack()
        return {'canvas': c, 'needle': needle, 'arc': arc, 'label': lbl,
                'cx': cx, 'cy': cy}

    def _apply_board_widgets(self, reduced):
        """U 보드에는 SWA/SWB/D12/D13/A0 가 물리적으로 없다. 칸은 그대로 두고 회색으로
        죽인다 — 지우면 레이아웃이 바뀌어 J 보드와 화면이 달라 보인다."""
        on = not reduced
        for ids in (self.sw_ids, self.toggle_ids):
            for c, oid, lbl in ids.values():
                c.itemconfig(oid, fill=IDLE_COLOR if on else DEAD_FILL,
                             outline=LINE if on else DEAD_LINE)
                lbl.config(fg=TEXT if on else DEAD_TEXT)
        # SWB 는 J 보드에서도 이 노드가 쓰지 않는다(조향 캘리브레이션 미구현)
        c, oid, lbl = self.sw_ids['SWB']
        lbl.config(text='SWB(미배정)', fg=DEAD_TEXT)
        c.itemconfig(oid, fill=DEAD_FILL, outline=DEAD_LINE)
        if not on:
            _c, _o, l = self.sw_ids['SWA']
            l.config(text='L+R 동시', fg=TEXT)   # U 보드의 SWA 대체 입력
            _c.itemconfig(_o, fill=IDLE_COLOR, outline=LINE)
        cv = self.pot['canvas']
        cv.itemconfig(self.pot['arc'], outline=LINE if on else DEAD_LINE)
        cv.itemconfig(self.pot['needle'], fill=DOT_COLOR if on else DEAD_LINE)
        self.pot['label'].config(text='A0:----' if on else 'A0:없음',
                                 fg=TEXT if on else DEAD_TEXT)

    # ── 갱신 ─────────────────────────────────────────────────────────
    def _draw_pad(self, pad, x_raw, y_raw, k, cx, cy):
        sx = (1 - normalize(x_raw, cx)) / 2 * PAD
        sy = (1 - normalize(y_raw, cy)) / 2 * PAD
        c = pad['canvas']
        c.coords(pad['dot'], sx - DOT_R, sy - DOT_R, sx + DOT_R, sy + DOT_R)
        if pad['highlight'] == 'h':
            c.coords(pad['hi'], 0, sy, PAD, sy)
        else:
            c.coords(pad['hi'], sx, 0, sx, PAD)
        c.itemconfig(pad['k'], fill=PRESS_COLOR if k == 0 else '')
        pad['label'].config(text=f'X:{x_raw:4d} Y:{y_raw:4d}')

    def _tick(self):
        if not self.running:
            return
        s = self.node.snapshot()

        # 보드 종류가 바뀌면(재연결 포함) 칸 활성/비활성을 다시 반영
        if s['kind'] != self.kind_applied:
            self.kind_applied = s['kind']
            self._apply_board_widgets(reduced=(s['kind'] == 'U'))

        data = s['data']
        cx1, cy1, cx2, cy2 = s['center']
        if data is not None:
            x1, y1, k1, x2, y2, k2, swa, swb, d12, d13, a0 = data
            self._draw_pad(self.pad_l, x1, y1, k1, cx1, cy1)
            self._draw_pad(self.pad_r, x2, y2, k2, cx2, cy2)
            if s['kind'] == 'J':
                for name, v in (('D12', d12), ('D13', d13)):
                    c, oid, _ = self.toggle_ids[name]
                    c.itemconfig(oid, fill=PRESS_COLOR if v == 0 else IDLE_COLOR)
                theta = math.radians(POT_GAP / 2 + (a0 / ADC_MAX) * (360 - POT_GAP))
                px = self.pot['cx'] - POT_R * math.sin(theta)
                py = self.pot['cy'] - POT_R * math.cos(theta)
                self.pot['canvas'].coords(self.pot['needle'],
                                          self.pot['cx'], self.pot['cy'], px, py)
                self.pot['label'].config(text=f'A0:{a0:4d}')
            # SWA 는 두 보드 모두 표시한다(U 는 L+R 동시 누름이 여기로 들어온다)
            c, oid, _ = self.sw_ids['SWA']
            c.itemconfig(oid, fill=PRESS_COLOR if swa == 0 else IDLE_COLOR)

        # ── 상태 3줄 ──
        if not s['connected']:
            text, color = '조이스틱 미연결 — 재연결 시도 중', WARN_COLOR
        elif not s['calibrated']:
            text = (f"영점 수집 {s['calib_n']}/{CALIBRATION_SAMPLES}"
                    '  — ★스틱을 건드리지 마십시오★')
            color = WARN_COLOR
        elif s['gate'] is not None:
            text, color = f"⛔ 차단 : {s['gate']}", PRESS_COLOR
        elif s['paused']:
            text = f"⏸ 일시정지 ({s['reason']}) — SWA 를 누르면 시작"
            color = TEXT
        else:
            text, color = '▶ 작동 중 — SWA 를 누르면 일시정지', OK_COLOR
        self.state_label.config(text=text, fg=color)

        mode = ('자율주행' if s['auto_mode'] is True else
                '수동조종' if s['auto_mode'] is False else '모드 미수신')
        self.gate_label.config(
            text=f"보드 {s['kind'] or '-'}   |   주행모드 {mode}"
                 f"   |   E-STOP {'발동' if s['estop'] else '정상'}")
        self.cmd_label.config(
            text=f"발행: 펄스 {s['pulse']:2d}   조향 {s['steer']:+3d}"
                 f"   브레이크 {s['brake']}")

        self.root.after(GUI_PERIOD_MS, self._tick)

    # ── 종료 ─────────────────────────────────────────────────────────
    def stop(self):
        self.running = False

    def on_close(self):
        self.running = False
        self.root.quit()      # mainloop 를 빠져나오고, 정리는 main() 의 finally 가 한다


def main(args=None):
    """★master.py 와 같은 결합 방식★ rclpy 는 별 스레드에서 돌고 tkinter 가 메인
    스레드를 잡는다. 조종값 계산·발행은 노드의 ROS 타이머(20Hz)가 하고, 화면은
    snapshot() 을 읽어 그리기만 한다 — 위젯을 다른 스레드에서 만지지 않기 위함이다.

    ★arduino 노드가 떠 있어야 한다★ 이 노드는 토픽만 발행하고 보드에는 직접 쓰지
    않는다(A/B 보드 시리얼은 arduino 노드가 독점한다). 조이스틱 보드만 이 노드가
    직접 연다 — 그쪽은 arduino 가 잡지 않는 별개의 메가다.
    """
    rclpy.init(args=args)
    node = JoystickNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    gui = JoystickGui(root, node)

    # ★Ctrl+C 는 tkinter 가 삼킨다★ (콜백 예외를 report_callback_exception 이 먹는다)
    #   그래서 신호를 받아 창닫기와 같은 경로로 보낸다.
    try:
        signal.signal(signal.SIGINT, lambda *_a: root.after(0, gui.on_close))
    except ValueError:
        pass

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        gui.stop()
        # ★반드시 정지값을 남긴다★ A보드에는 무입력 타임아웃이 없어, arduino 노드가
        #   마지막 명령을 1초마다 재전송한다 — 안 내고 끝내면 차가 계속 간다.
        try:
            node.stop_and_close()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            root.destroy()
        except Exception:
            pass
        print("joystick 종료 — 정지값 발행됨", flush=True)


if __name__ == '__main__':
    main()
