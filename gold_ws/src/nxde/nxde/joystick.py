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
  · tkinter GUI 대신 ★터미널 한 줄 상태표시★ 로 줄였다. 마우스 조종·계측 확인은
    master 노드가 담당한다 (ros2 run nxde master).
  · 위 안전장치 ①(자율주행 모드 게이트)이 새로 추가되었다.

 지원 보드는 원본과 같다 — 접두어로 구분한다(둘 다 메가라 VID/PID 로는 구분 불가):
   "J," = joy.ino  : x1,y1,k1,x2,y2,k2,swa,swb,d12,d13,a0   (11필드)
   "U," = joy2.ino : lx,ly,lsw,rx,ry,rsw                    (6필드)
     축 규격이 달라(lx/rx=세로, ly/ry=가로) 수신 즉시 J 규격으로 변환한다.
     U 보드에는 SWA 가 없으므로 ★L·R 스틱 버튼 동시 누름★ 이 SWA 를 대신한다.
"""

import statistics
import threading
import time

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
STATUS_PERIOD_S = 1.0         # 터미널 상태표시 주기

PULSE_MAX = 15                # A보드 단일값 입력 상한
STEER_MAX = 40                # B보드 STEER_ANGLE_MAX
BRAKE_LEVEL_MAX = 2           # 0 = 놓음 / 1 = 약(1/3) / 2 = 풀
ADC_MAX = 1023

DEADZONE_RAW = 120            # 영점 기준 raw ADC 편차가 이 값 미달이면 0으로 처리
DEFAULT_CENTER = ADC_MAX // 2

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

        self.last_steer = 0
        self._last_status_t = 0.0

        self._running = True
        self._reader = threading.Thread(target=self._reader_loop,
                                        name='joy_serial', daemon=True)
        self._reader.start()

        self.create_timer(CONTROL_PERIOD_S, self._control_tick)
        self.create_timer(STATUS_PERIOD_S, self._print_status)

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

        msg = Twist()
        msg.linear.x = float(pulse)         # ★m/s 가 아니라 주행 목표펄스★
        msg.angular.z = float(steer)
        self.pub_cmd.publish(msg)
        self.pub_state.publish(Bool(data=True))
        self.pub_brake.publish(Int32(data=max(0, min(BRAKE_LEVEL_MAX, brake))))

    def _publish_stop(self):
        """정지값 발행. ★발행을 '멈추면' 안 된다★

        A보드 펌웨어에는 무입력 타임아웃이 없다 — arduino 노드가 마지막 명령을
        1초마다 계속 재전송하므로, 발행을 그냥 끊으면 차가 계속 간다.
        조향은 0 을 넣어도 arduino 가 control_state=False 에서 마지막 각도를
        유지한다(주행 중 0도로 급조향하면 위험하므로 그쪽이 맞다).
        """
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub_cmd.publish(msg)
        self.pub_state.publish(Bool(data=False))
        self.pub_brake.publish(Int32(data=0))

    # ── 터미널 상태표시 ───────────────────────────────────────────────
    def _print_status(self):
        with self.lock:
            conn = self.connected
            kind = self.kind
            calibrated = self.calibrated
            calib_n = len(self.calib_buf['y1']) if self.calib_buf else 0
            paused = self.paused
            data = self.data
            gate = self._gate_reason()
            reason = self.rearm_reason

        if not conn:
            state = "🔌 조이스틱 미연결"
        elif not calibrated:
            state = f"⏳ 영점 수집 {calib_n}/{CALIBRATION_SAMPLES} (스틱을 건드리지 마십시오)"
        elif gate is not None:
            state = f"⛔ 차단: {gate}"
        elif paused:
            state = f"⏸ 일시정지 ({reason}) — SWA 를 누르면 시작"
        else:
            state = "▶ 작동 중"

        mode = ("자율주행" if self.auto_mode is True
                else "수동조종" if self.auto_mode is False else "모드 미수신")
        sticks = ""
        if data is not None:
            sticks = f" | L(Y)={data[1]:4d} R(X)={data[3]:4d} SWA={'눌림' if data[6] == 0 else '  -  '}"
        print(f"[joystick] {state} | 보드={kind or '-'} | 주행모드={mode}{sticks}",
              flush=True)

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


def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_and_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
