#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# master : ★하드웨어 검증용 GUI 조종 노드★ (마우스 + 키보드)
#
#   ros2 run nxde master
#
# ═══════════════════════════════════════════════════════════════════════════════
#  이 파일의 목적
# ═══════════════════════════════════════════════════════════════════════════════
#   arduino 노드만 띄운 상태에서 **차가 실제로 움직이는지** 확인하는 도구다.
#   판단 스택(white one_launch.py)을 올리기 전에
#     · 아두이노 A/B 보드가 붙었는지
#     · 조향 부호가 맞는지 (레버를 왼쪽으로 → 바퀴가 왼쪽으로)
#     · 펄스 명령이 실제 주행 펄스로 돌아오는지
#     · 수동조종 모드에서 페달이 제대로 환산되는지
#     · E-stop 이 감지되는지
#   를 눈으로 확인한다.
#
#   ★ 최소 실행 조합 (nxde 에는 런치파일이 없다 — 2026-08-05) ★
#       터미널 1 :  ros2 run nxde arduino     ← 보드 통신 (이게 없으면 아무것도 안 움직인다)
#       터미널 2 :  ros2 run nxde master      ← 이 창
#     조이스틱으로 조종하려면 master 대신 `ros2 run nxde joystick` 을 띄운다.
#     자율주행은 `ros2 launch white one_launch.py` 가 arduino 를 포함해 함께 띄운다.
#
#   kasa_ws/src/nxde/nxde/master.py 를 옮겨온 것으로 기능·레이아웃이 거의 같다.
#   ★ 차이는 아래 세 가지뿐이다 ★
#     (1) 컨트롤러(조이스틱) 모드 제거 — ★별 노드로 분리했다★: ros2 run nxde joystick
#         (그쪽은 자율주행 모드에서만 작동하고, 영점 후 SWA 를 눌러야 시작한다)
#         차량 자체의 수동조종은 D5 스위치 + 실제 페달·핸들이며 arduino 가 직접 처리한다.
#     (2) 통신 양식이 white 규약 토픽으로 바뀜 — /in·/out String 대신
#         /cmd_vel_raw(Twist) + /control_state(Bool) + /brake_level(Int32) 발행,
#         텔레메트리는 개별 토픽 구독.
#     (3) 디퍼렌셜·PWM모드 체크박스 제거 — nxde/arduino.py 가 A보드로 항상 '단일값'만
#         보내기 때문이다(직접 PWM 16~255 는 PID·슬루레이트·폭주감지가 전부 빠지는
#         무보호 경로라 자율주행 스택에서 봉쇄했다). 좌우 차동도 하지 않는다.
#
#   ⚠️ joystick 노드와 동시에 쓰지 말 것 — /cmd_vel_raw 발행자가 겹친다.
#
# ═══════════════════════════════════════════════════════════════════════════════
#  [2026-08-04 수정] 실차 시험에서 드러난 구버전 잔재 2건
# ═══════════════════════════════════════════════════════════════════════════════
#   ★① 브레이크 : 0~100 슬라이더(표시 전용) → 0/1/2 단계(실제 제어) ★
#     kasa_ws 판을 옮길 때 브레이크 레버를 0~100 표시 전용으로 두었다. 그건 B보드가
#     0~255 열린루프 브레이크만 지원하던 0731 이전 시절의 잔재다. 지금 펌웨어
#     (kasa_0804_B.ino)는 ★단계 0/1/2★ 를 받고 엔코더 실측 시간표로 정확히 밟는다:
#         0 = 기본위치(놓음) / 1 = 행정의 1/3 (83카운트) / 2 = 풀브레이킹 (250카운트)
#     Twist 에는 브레이크 필드가 없으므로 /brake_level (Int32) 로 따로 발행한다.
#     → 이제 레버가 실제로 브레이크를 밟는다.
#
#   ★② 조향 부호 : 레버 방향과 바퀴 방향이 반대였다 ★
#     가로 조향 레버는 왼쪽 끝이 −40, 오른쪽 끝이 +40 이다(당연한 배치). 그런데 예전
#     규약은 ROS 안에서 '+ = 좌회전'(white 부호)이었다 → 레버를 오른쪽으로 밀면 차가
#     왼쪽으로 갔다. 그래서 ROS 토픽 전체를 kasa B보드 부호로 통일했다:
#         ★ 음수 = 좌회전 / 양수 = 우회전 ★  (화면·토픽·시리얼·펌웨어 전부 동일)
#     이 파일은 레버값을 그대로 발행한다 — 부호를 만지는 곳이 없다.
#     (driving.py 제어기 내부만 여전히 '+좌'이고, 그 반전은 driving.publish_cmd 의
#      to_ros_steer() 한 줄에서 끝난다. arduino.py 의 steer_invert 는 기본 False.)
#
# ═══════════════════════════════════════════════════════════════════════════════
#  ⚠️⚠️ one_launch.py / prompt 와 동시에 쓰지 말 것 ⚠️⚠️
# ═══════════════════════════════════════════════════════════════════════════════
#   /cmd_vel_raw 와 /control_state 의 발행자가 겹친다. driving_node 와 이 창이 같이
#   발행하면 두 명령이 20~50ms 간격으로 교대해 차가 떤다.
#   → 이 창은 driving_node 가 떠 있으면 상단에 빨간 경고를 표시한다(_check_conflict).
#   검증이 끝나면 이 창을 닫고 one_launch.py 를 띄운다.
#
# ── 레이아웃 (kasa_ws master 와 동일) ──
#   상단 : 안내문구 (E-stop 발동 중에는 빨간 'E-Stop 발동!!!' 으로 덮인다,
#          발행자 충돌 시에는 그 경고가 최우선)
#   중앙 : [엑셀 레버(세로, 0~15)] [브레이크 레버(세로, UI 전용)]
#          [발행 ON/OFF 토글] [주행모드 박스 + 조향 레버(가로, -40~40)]
#   레버 아래 : 실측값(아두이노 텔레메트리) / 명령값(지금 발행 중인 값) 표
#   최하단   : A보드/B보드 연결 상태 (/board_status)
#
# ── 레버 조작 ──
#   마우스로 레버를 눌러 끌면 그 위치의 값이 되고, 손을 떼도 유지된다(Windows 볼륨
#   슬라이더와 같은 느낌 — 원점 복귀 없음). 키보드 Up/Down=엑셀, Left/Right=조향,
#   PageUp/PageDown=브레이크 단계.
#   ★수동조종 모드에서는 레버가 잠기고 '실측값을 비추는 계기판'이 된다★
#
# ── 발행 ON/OFF 토글 (자율주행 모드 전용) ──
#   OFF 가 기본값. OFF 인 동안은 /control_state=False + 펄스 0 을 보낸다(레버는 자유롭게
#   움직여 미리 값을 맞춰둘 수 있다). ON→OFF 로 바뀌는 순간 엑셀·조향 레버를 0 으로
#   되돌리고 정지값을 1회 강제 발행한다.
#   ※ kasa_ws 판과 달리 '발행 자체를 멈추지' 않는다 — A보드 펌웨어에 무입력 타임아웃이
#     없어서 발행을 끊으면 arduino 노드가 마지막 값을 1초마다 재전송한다(= 차가 계속 간다).
#     그래서 OFF 동안에도 정지값을 계속 내보내는 편이 안전하다.
#
# ── 브레이크 레버 (0 / 1 / 2 단계, 실제 제어) ──
#   /brake_level (Int32) 로 발행한다. 자율주행 모드에서만 반영되며, 아래가 우선한다:
#     · E-stop     → B보드가 스스로 2단 체결 / 해제 시 0단 복귀 (ROS 개입 없음)
#     · 수동조종    → arduino.py 의 래치 (진입 2단 → 페달 밟으면 0단)
#     · /control_state=False → max(stop_brake_level, 이 레버값)
#   즉 이 레버가 실제로 브레이크를 움직이는 것은 '자율주행 + 발행 ON' 일 때다.
#   ★1단은 행정의 1/3, 2단은 풀브레이킹이다 — 정차 중에 2단을 걸면 리니어가 페달을
#     끝까지 밟으므로, 시험 시에는 1단부터 확인하는 것이 안전하다★
#
# ── 자율주행 / 수동조종 모드 (B보드 D5 스위치 + 마우스) ──
#   조향 레버 위 박스에 표시: 자율주행=연두 / 수동조종=노랑 / 모름=회색.
#   표시는 /vehicle_mode(실효 모드)를 따른다.
#
#   ★ [2026-08-04] 이 박스를 클릭해서도 모드를 바꿀 수 있다 ★ 예전에는 물리 스위치가
#     유일한 권한이었는데, B보드 USB 링크가 불안정해 D5 값이 아예 안 들어오는 일이
#     실제로 있었다(실측: 재열거 28회 / urb -32 55회 / 88초간 프레임 0개).
#     그때는 스위치를 만져도 모드가 안 바뀌므로 소프트웨어 경로가 필요하다.
#       좌클릭 : 자율 ↔ 수동 전환   (/vehicle_mode_cmd 1 또는 0)
#       우클릭 : 오버라이드 해제 → 물리 스위치로 복귀   (/vehicle_mode_cmd -1)
#     오버라이드가 걸려 있으면 박스에 [SW] 가 붙고 /board_status 에 SRC:ovr 가 실린다.
#     ★강제는 arduino 노드가 한다★ 이 창은 요청만 보내고 실제 반영은 /vehicle_mode
#     왕복으로 확인한다 — "버튼은 눌렸는데 차는 그대로"를 눈치챌 수 있게.
#
#   ★ 수동조종 모드 동작 ★
#     - 마우스·키보드 입력을 전부 무시한다(레버 잠김).
#     - 엑셀 레버 = /drive_pulse_cmd (arduino 가 페달 raw 를 환산한 목표펄스)
#       조향 레버 = /steer_angle_measured (가변저항 실측 각도)
#       → ★페달을 밟으면 엑셀 레버가 올라가고, 핸들을 돌리면 조향 레버가 움직인다★
#     - 발행값은 '펄스 0 + 실측 조향각'을 유지한다. 자율주행으로 되돌아오는 순간
#       차는 정지 상태이고 조향 목표는 지금 핸들 위치라 급조향이 없다.
#       (수동 중에는 arduino 가 /cmd_vel_raw 를 아예 무시하므로 무엇을 보내도 무해하지만,
#        '전환 직후 적용될 마지막 명령'을 안전하게 두는 것이 목적이다)
#
# ── E-stop 표시 ──
#   /estop 이 True 인 동안 상단이 굵은 빨간 'E-Stop 발동!!!' 으로 바뀐다. **표시 전용이다** —
#   실제 정지는 아두이노(13번 핀)와 B보드 리니어 2단이 이미 수행한다. 화면에서 실측값이
#   굳은 이유를 알 수 있게 하는 목적.

import threading
import time
import tkinter as tk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String
from geometry_msgs.msg import Twist

from nxde.proc_guard import watch_parent

# ── 프로토콜 한계 (kasa_0730_A.ino / kasa_0804_B.ino, arduino.py 와 같은 값) ──
PULSE_MAX = 15          # A보드 단일값 입력 상한
STEER_MAX = 40          # B보드 STEER_ANGLE_MAX
ADC_MAX   = 1023

BRAKE_LEVEL_MAX = 2     # ★브레이크 단계 0/1/2★ (kasa_0804_B.ino — 0~255 PWM 이 아니다)
#   0 = 기본위치(놓음) / 1 = 행정의 1/3 (83카운트) / 2 = 풀브레이킹 (250카운트)
BRAKE_LABELS = {0: "놓음", 1: "약(1/3)", 2: "풀"}
KEYBOARD_BRAKE_STEP = 1     # PageUp/PageDown 1회당 브레이크 단계 증감

# 실차 실측 (kasa_ws master.py / PULSE_SPEED.md 와 동일)
KMH_PER_PULSE = 3.18    # 1펄스 = 3.18 km/h → 15펄스 = 47.7 km/h
# /encoder 1카운트(좌+우 합) 당 속도. white/kasa_units.py 의 MS_PER_ENCODER_COUNT 와 같은 값.
#   ※ nxde 는 white 를 import 하지 않는다(패키지 독립) → 값을 복제하되 출처를 남긴다.
KMH_PER_ENCODER_COUNT = KMH_PER_PULSE / 2.0    # 1.59 km/h

KEYBOARD_PULSE_STEP = 1     # Up/Down 1회(또는 자동반복 1틱)당 엑셀 증감
KEYBOARD_STEER_STEP = 2     # Left/Right 1회당 조향각 증감(도)

KEEPALIVE_S = 0.5           # 값 변화 없어도 이 주기로 재발행
UPDATE_MS   = 50            # GUI/발행 tick 주기
CONFLICT_CHECK_S = 2.0      # driving_node 발행자 충돌 검사 주기

# ================= 다크 테마 팔레트 (kasa_ws master 와 동일 톤) =================
BG = "#1e1e1e"
TRACK_BG = "#2b2b2b"
LINE = "#4a4a4a"
HANDLE_COLOR = "#4fc3f7"
IDLE_COLOR = "#3a3a3a"
OK_COLOR = "#66bb6a"
TEXT = "#dddddd"
DISABLED_TEXT = "#666666"

# 주행모드 박스 색 (조향 레버 위). 밝은 배경이므로 글자는 어두운 색을 얹는다.
AUTO_BOX_BG = "#aed581"     # 자율주행 = 연두색
MANUAL_BOX_BG = "#ffd54f"   # 수동조종 = 노란색
BOX_FG = "#102027"

# 경고 표시
ESTOP_COLOR = "#ff1744"
ESTOP_TEXT = "E-Stop 발동!!!"
CONFLICT_COLOR = "#ff9100"
STATUS_FONT = ("Consolas", 12)
WARN_FONT = ("Consolas", 18, "bold")


def _round_half_away(x):
    """아두이노 round() 매크로와 같은 반올림 (파이썬 내장 round 는 은행가 반올림)."""
    return int(x + 0.5) if x >= 0 else -int(-x + 0.5)


# ================= ROS2 노드 =================
class MasterNode(Node):
    """/cmd_vel_raw·/control_state 발행 + 아두이노 텔레메트리 구독.

    구독 콜백은 spin 스레드에서 실행되므로 tkinter 를 직접 건드리지 않고 값만 저장한다.
    실제 위젯 갱신은 tkinter 메인스레드의 _tick() 이 담당한다."""

    def __init__(self):
        super().__init__('master')

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.pub_state = self.create_publisher(Bool, '/control_state', 10)
        # 브레이크 단계(0/1/2). Twist 에 필드가 없어 별 토픽으로 보낸다.
        self.pub_brake = self.create_publisher(Int32, '/brake_level', 10)
        # 주행모드 소프트웨어 전환 (1 자율 / 0 수동 / -1 물리 스위치 복귀).
        #   arduino 노드가 이 값을 물리 스위치보다 우선 적용한다.
        self.pub_mode_cmd = self.create_publisher(Int32, '/vehicle_mode_cmd', 10)

        self.create_subscription(Int32, '/encoder', self._cb_encoder, 10)
        self.create_subscription(Int32, '/steer_angle_measured', self._cb_steer, 10)
        self.create_subscription(Int32, '/drive_pulse_cmd', self._cb_drive_pulse, 10)
        self.create_subscription(Int32, '/throttle_pedal', self._cb_throttle, 10)
        self.create_subscription(Bool, '/vehicle_mode', self._cb_mode, 10)
        self.create_subscription(Bool, '/estop', self._cb_estop, 10)
        self.create_subscription(String, '/board_status', self._cb_status, 10)

        # ── 텔레메트리 캐시 ──
        self.wheel_pulse = 0        # /encoder (좌+우 합)
        self.steer_measured = 0     # /steer_angle_measured (− 좌 / + 우, 명령과 같은 부호)
        self.drive_pulse_cmd = 0    # /drive_pulse_cmd (자율=계획값 / 수동=페달 환산값)
        self.throttle_raw = 0       # /throttle_pedal (A0 raw)
        # B보드 D5 주행모드. None = /vehicle_mode 를 아직 못 받아 모름.
        #   ★ None 을 '자율'로 가정하지 않는다 ★ 모드를 모르는 상태에서 마우스 명령을
        #     내보내면 수동조종 중인 차에 자율 명령을 쏘는 셈이 된다 → 레버를 잠근다.
        self.auto_mode = None
        self.estop = False
        self.board_status = None     # "A:1,B:1,ESTOP:0,MODE:1"

        self.last_angle_cmd = 0      # 마지막으로 발행한 조향각 (종료 시 참고)

    # ---------- 구독 콜백 ----------
    def _cb_encoder(self, msg):        self.wheel_pulse = int(msg.data)
    def _cb_steer(self, msg):          self.steer_measured = int(msg.data)
    def _cb_drive_pulse(self, msg):    self.drive_pulse_cmd = int(msg.data)
    def _cb_throttle(self, msg):       self.throttle_raw = int(msg.data)
    def _cb_estop(self, msg):          self.estop = bool(msg.data)
    def _cb_status(self, msg):         self.board_status = msg.data

    def _cb_mode(self, msg):
        self.auto_mode = bool(msg.data)

    # ---------- 발행 ----------
    def publish_cmd(self, pulse, angle_deg, brake_level, enabled):
        """/cmd_vel_raw + /control_state + /brake_level 발행.

        linear.x  = 주행 목표펄스 0~15 (★m/s 가 아니다★)
        angular.z = 조향각 -40~40 (★− 좌 / + 우 — 레버값을 그대로, 부호를 만지지 않는다★)
        brake     = 단계 0/1/2 (★0~255 PWM 이 아니다★)"""
        msg = Twist()
        msg.linear.x = float(max(0, min(PULSE_MAX, int(pulse))))
        msg.angular.z = float(max(-STEER_MAX, min(STEER_MAX, int(angle_deg))))
        self.pub_cmd.publish(msg)
        self.pub_state.publish(Bool(data=bool(enabled)))
        self.pub_brake.publish(
            Int32(data=int(max(0, min(BRAKE_LEVEL_MAX, int(brake_level))))))
        self.last_angle_cmd = int(msg.angular.z)

    def publish_mode_cmd(self, value):
        """/vehicle_mode_cmd 발행 — 1 자율 / 0 수동 / -1 물리 스위치 복귀.

        ★강제는 arduino 노드가 한다★ 이 창은 요청만 보내고, 실제 반영은 /vehicle_mode
        로 되돌아온 값으로 확인한다(그래야 "버튼은 눌렸는데 차는 그대로"를 알아챈다)."""
        v = int(value)
        if v not in (-1, 0, 1):
            self.get_logger().warn(f"/vehicle_mode_cmd={v} 는 허용값이 아님 — 무시")
            return
        self.pub_mode_cmd.publish(Int32(data=v))
        self.get_logger().info(
            "[모드 요청] " + {1: "자율주행 강제", 0: "수동조종 강제",
                             -1: "오버라이드 해제(물리 스위치 복귀)"}[v])

    def publish_stop(self):
        """종료 시 정지값. 조향은 마지막 값을 유지한다(정면 급조향 방지).

        ★브레이크는 0(놓음)으로 둔다★ 리니어가 페달을 물고 있으면 사람이 차를 움직일
        수 없다 — 창을 닫은 뒤 수동으로 빼내야 하는 상황을 만들지 않는다."""
        self.publish_cmd(0, self.last_angle_cmd, 0, False)

    def driving_node_present(self):
        """driving_node 가 떠 있으면 True → /cmd_vel_raw 발행자 충돌."""
        try:
            return 'driving_node' in self.get_node_names()
        except Exception:
            return False


# ================= 레버 위젯 (마우스/터치 드래그, 손 떼도 값 유지) =================
class Slider:
    """세로(엑셀/브레이크)·가로(조향) 겸용 페이더. 드래그 즉시 값 반영, 놓아도 스프링 없음."""

    TRACK_LEN = 200
    MARGIN = 20
    TRACK_THICK = 14
    HANDLE_LONG = 50
    HANDLE_SHORT = 20

    def __init__(self, parent, orient, vmin, vmax, initial=0):
        self.orient = orient   # 'v' 또는 'h'
        self.vmin, self.vmax = vmin, vmax
        self.value = initial
        self.readonly = False   # 수동조종 모드일 때 True — 드래그 무시(실측을 비추는 계기판)

        span = self.TRACK_LEN + 2 * self.MARGIN
        thick_span = self.HANDLE_LONG + 2 * self.MARGIN
        w, h = (thick_span, span) if orient == 'v' else (span, thick_span)
        self.canvas = tk.Canvas(parent, width=w, height=h, bg=BG, highlightthickness=0)

        c = thick_span / 2
        if orient == 'v':
            self.canvas.create_rectangle(
                c - self.TRACK_THICK / 2, self.MARGIN,
                c + self.TRACK_THICK / 2, self.MARGIN + self.TRACK_LEN,
                fill=TRACK_BG, outline=LINE)
        else:
            self.canvas.create_rectangle(
                self.MARGIN, c - self.TRACK_THICK / 2,
                self.MARGIN + self.TRACK_LEN, c + self.TRACK_THICK / 2,
                fill=TRACK_BG, outline=LINE)

        self.handle = self.canvas.create_rectangle(0, 0, 0, 0, fill=HANDLE_COLOR, outline="")
        self.value_text = self.canvas.create_text(0, 0, fill="#102027",
                                                  font=("Consolas", 10, "bold"))

        self.canvas.bind('<Button-1>', self._on_drag)
        self.canvas.bind('<B1-Motion>', self._on_drag)
        self._redraw()

    def pack(self, **kw):
        self.canvas.pack(**kw)

    def grid(self, **kw):
        self.canvas.grid(**kw)

    def _on_drag(self, event):
        if self.readonly:
            return
        if self.orient == 'v':
            pos = max(self.MARGIN, min(self.MARGIN + self.TRACK_LEN, event.y))
            frac = 1 - (pos - self.MARGIN) / self.TRACK_LEN
        else:
            pos = max(self.MARGIN, min(self.MARGIN + self.TRACK_LEN, event.x))
            frac = (pos - self.MARGIN) / self.TRACK_LEN
        self.set_value(self.vmin + frac * (self.vmax - self.vmin))

    def set_value(self, v):
        v = max(self.vmin, min(self.vmax, int(round(v))))
        if v != self.value:
            self.value = v
            self._redraw()

    def nudge(self, delta):
        self.set_value(self.value + delta)

    def set_handle_color(self, color):
        self.canvas.itemconfig(self.handle, fill=color)

    def _redraw(self):
        frac = (self.value - self.vmin) / (self.vmax - self.vmin)
        thick_span = self.HANDLE_LONG + 2 * self.MARGIN
        c = thick_span / 2
        if self.orient == 'v':
            y = self.MARGIN + self.TRACK_LEN * (1 - frac)
            self.canvas.coords(self.handle, c - self.HANDLE_LONG / 2, y - self.HANDLE_SHORT / 2,
                               c + self.HANDLE_LONG / 2, y + self.HANDLE_SHORT / 2)
            self.canvas.coords(self.value_text, c, y)
        else:
            x = self.MARGIN + self.TRACK_LEN * frac
            self.canvas.coords(self.handle, x - self.HANDLE_SHORT / 2, c - self.HANDLE_LONG / 2,
                               x + self.HANDLE_SHORT / 2, c + self.HANDLE_LONG / 2)
            self.canvas.coords(self.value_text, x, c)
        self.canvas.itemconfig(self.value_text, text=str(self.value))


# ================= GUI =================
class MasterGui:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.running = True

        self.enabled = False         # 발행 게이트(ON/OFF 토글)
        self._last_pub = None
        self._last_pub_t = 0.0
        self.manual_active = False   # 수동조종 모드인 동안 True
        self.estop_active = False
        self.conflict = False        # driving_node 와 발행자 충돌
        self._last_conflict_check = 0.0

        root.title("nxde master — 하드웨어 검증")
        root.configure(bg=BG)
        root.attributes('-topmost', True)
        root.lift()
        root.after(500, lambda: root.attributes('-topmost', False))

        self.status_label = tk.Label(root, text="", bg=BG, fg=TEXT, font=STATUS_FONT,
                                     wraplength=760, justify='center')
        self.status_label.pack(pady=(14, 6))

        levers = tk.Frame(root, bg=BG)
        levers.pack(padx=16, pady=6)

        throttle_col = tk.Frame(levers, bg=BG)
        throttle_col.grid(row=0, column=0, padx=10)
        self.throttle_label = tk.Label(throttle_col, text="엑셀 (펄스)", bg=BG, fg=TEXT,
                                       font=("Consolas", 13, "bold"))
        self.throttle_label.pack()
        self.throttle = Slider(throttle_col, 'v', 0, PULSE_MAX, 0)
        self.throttle.pack()
        self.throttle_kmh = tk.Label(throttle_col, text="0.0 km/h", bg=BG, fg=TEXT,
                                     font=("Consolas", 9))
        self.throttle_kmh.pack(pady=(2, 0))

        brake_col = tk.Frame(levers, bg=BG)
        brake_col.grid(row=0, column=1, padx=10)
        tk.Label(brake_col, text="브레이크 (단계)", bg=BG, fg=TEXT,
                 font=("Consolas", 13, "bold")).pack()
        # ★0/1/2 단계 — 실제로 리니어를 움직인다★ (구버전의 0~100 표시 전용이 아니다)
        self.brake = Slider(brake_col, 'v', 0, BRAKE_LEVEL_MAX, 0)
        self.brake.pack()
        self.brake_label = tk.Label(brake_col, text="0 놓음", bg=BG, fg=TEXT,
                                    font=("Consolas", 9))
        self.brake_label.pack(pady=(2, 0))

        mid_col = tk.Frame(levers, bg=BG, width=160)
        mid_col.grid(row=0, column=2, padx=16)
        tk.Label(mid_col, text="발행", bg=BG, fg=TEXT, font=("Consolas", 10)).pack()
        self.toggle_btn = tk.Button(mid_col, text="OFF", width=8,
                                    font=("Consolas", 12, "bold"),
                                    bg=IDLE_COLOR, fg=TEXT, activebackground=IDLE_COLOR,
                                    command=self._on_toggle_click)
        self.toggle_btn.pack(pady=6)
        self.pedal_label = tk.Label(mid_col, text="페달 A0:----", bg=BG, fg=TEXT,
                                    font=("Consolas", 9))
        self.pedal_label.pack(pady=(8, 0))

        steer_col = tk.Frame(levers, bg=BG)
        steer_col.grid(row=0, column=3, padx=10)
        # ★ 조향축 그래픽 위 주행모드 박스 — 클릭하면 모드가 바뀐다 ★
        #   [2026-08-04] 예전엔 /vehicle_mode 를 보여주기만 하는 Label 이었다(물리 스위치가
        #   유일한 권한). 이제 마우스로도 전환할 수 있게 Button 으로 바꿨다 —
        #   B보드 USB 링크가 불안정해 D5 값이 아예 안 들어오는 일이 있어서(실측 재열거 28회)
        #   물리 스위치만으로는 모드를 못 바꾸는 상황이 실제로 생긴다.
        self.mode_box = tk.Button(steer_col, text="모드 확인 중...", bg=IDLE_COLOR, fg=TEXT,
                                  font=("Consolas", 13, "bold"), width=18, pady=7,
                                  relief=tk.RAISED, activebackground=IDLE_COLOR,
                                  cursor="hand2", command=self._on_mode_click)
        self.mode_box.pack(pady=(0, 8))
        tk.Label(steer_col, text="클릭: 자율 ↔ 수동 전환 / 우클릭: 스위치로 복귀",
                 bg=BG, fg=DISABLED_TEXT, font=("Consolas", 8)).pack(pady=(0, 6))
        # 우클릭 = 오버라이드 해제(물리 스위치 복귀)
        self.mode_box.bind('<Button-3>', lambda e: self._on_mode_release())
        # ★부호 규약: 왼쪽 끝 −40 = 좌회전 / 오른쪽 끝 +40 = 우회전★
        #   레버를 미는 방향과 바퀴가 도는 방향이 같다(파일 헤더 ② 참고).
        tk.Label(steer_col, text="조향 (도, ←−  +→)", bg=BG, fg=TEXT,
                 font=("Consolas", 13, "bold")).pack()
        self.steering = Slider(steer_col, 'h', -STEER_MAX, STEER_MAX, 0)
        self.steering.pack()
        tk.Label(steer_col, text="− 좌회전   /   + 우회전", bg=BG, fg=DISABLED_TEXT,
                 font=("Consolas", 8)).pack(pady=(2, 0))

        self._build_value_table(root)
        self._build_conn_status(root)

        root.bind('<KeyPress-Up>', lambda e: self._kb_nudge('throttle', KEYBOARD_PULSE_STEP))
        root.bind('<KeyPress-Down>', lambda e: self._kb_nudge('throttle', -KEYBOARD_PULSE_STEP))
        # ← 는 음수(좌회전), → 는 양수(우회전) — 레버 방향과 일치한다
        root.bind('<KeyPress-Left>', lambda e: self._kb_nudge('steering', -KEYBOARD_STEER_STEP))
        root.bind('<KeyPress-Right>', lambda e: self._kb_nudge('steering', KEYBOARD_STEER_STEP))
        root.bind('<KeyPress-Prior>', lambda e: self._kb_nudge('brake', KEYBOARD_BRAKE_STEP))
        root.bind('<KeyPress-Next>', lambda e: self._kb_nudge('brake', -KEYBOARD_BRAKE_STEP))
        root.focus_set()

        root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._update_status_text()
        self._tick()

    # ---------- 위젯 구성 ----------
    def _build_value_table(self, root):
        table = tk.Frame(root, bg=BG)
        table.pack(pady=(10, 6))
        headers = ("", "주행 펄스", "속도", "조향각")
        for col, text in enumerate(headers):
            tk.Label(table, text=text, bg=BG, fg=TEXT,
                     font=("Consolas", 9, "bold")).grid(row=0, column=col, padx=14)

        self.measured_vars = [tk.StringVar(value="0") for _ in range(3)]
        self.command_vars = [tk.StringVar(value="0") for _ in range(3)]
        tk.Label(table, text="실측값", bg=BG, fg=TEXT, font=("Consolas", 9)).grid(row=1, column=0)
        tk.Label(table, text="명령값", bg=BG, fg=TEXT, font=("Consolas", 9)).grid(row=2, column=0)
        for col in range(3):
            tk.Label(table, textvariable=self.measured_vars[col], bg=BG, fg=HANDLE_COLOR,
                     font=("Consolas", 10)).grid(row=1, column=col + 1)
            tk.Label(table, textvariable=self.command_vars[col], bg=BG, fg=OK_COLOR,
                     font=("Consolas", 10)).grid(row=2, column=col + 1)
        tk.Label(table, text="실측 주행펄스는 좌+우 합(1카운트=1.59km/h) · 조향각은 가변저항 실측"
                             " · 부호 − 좌 / + 우",
                 bg=BG, fg=DISABLED_TEXT, font=("Consolas", 8)).grid(
                     row=3, column=0, columnspan=4, pady=(6, 0))

    def _build_conn_status(self, root):
        row = tk.Frame(root, bg=BG)
        row.pack(pady=(0, 12))
        self.conn_a = tk.Label(row, text="A보드: 연결 중...", bg=BG, fg=TEXT, font=("Consolas", 9))
        self.conn_b = tk.Label(row, text="B보드: 연결 중...", bg=BG, fg=TEXT, font=("Consolas", 9))
        for w in (self.conn_a, self.conn_b):
            w.pack(side=tk.LEFT, padx=16)

    # ---------- 토글 ----------
    def _on_toggle_click(self):
        if self.manual_active:
            # 수동조종 중에는 발행 게이트가 의미가 없다(arduino 가 /cmd_vel_raw 를 무시한다).
            return
        self.enabled = not self.enabled
        self.toggle_btn.config(text="ON" if self.enabled else "OFF",
                               bg=OK_COLOR if self.enabled else IDLE_COLOR)
        if not self.enabled:
            # ON→OFF : 레버 위치와 무관하게 즉시 0 으로 되돌리고 정지값을 1회 강제 발행
            #   ★브레이크도 0 으로 내린다★ 리니어가 페달을 물고 있으면 차를 못 움직인다.
            self.throttle.set_value(0)
            self.steering.set_value(0)
            self.brake.set_value(0)
            self.node.publish_cmd(0, 0, 0, False)
            self._last_pub = (0, 0, 0, False)
            self._last_pub_t = time.monotonic()

    # ---------- 주행모드 (마우스) ----------
    def _on_mode_click(self):
        """모드 박스 클릭 — 자율 ↔ 수동 전환.

        현재 '실효 모드'의 반대를 강제한다. 모드를 아직 못 받았으면(None) 수동으로
        간다 — 모르는 상태에서 자율을 켜는 쪽이 더 위험하다."""
        want_auto = not (self.node.auto_mode is True)
        self.node.publish_mode_cmd(1 if want_auto else 0)
        # 화면은 /vehicle_mode 왕복으로 갱신된다(arduino 가 실효 모드를 되돌려준다).
        #   여기서 미리 칠하지 않는 이유: 오버라이드가 반영됐는지 실제 토픽으로 확인해야
        #   "버튼은 눌렸는데 차는 안 바뀐" 상태를 눈치챌 수 있다.

    def _on_mode_release(self):
        """모드 박스 우클릭 — 오버라이드 해제하고 물리 스위치(B보드 D5)로 복귀."""
        self.node.publish_mode_cmd(-1)

    def _kb_nudge(self, which, delta):
        if self.manual_active:
            return   # 수동조종 모드에서는 마우스와 마찬가지로 키보드도 무시
        {'throttle': self.throttle,
         'steering': self.steering,
         'brake':    self.brake}[which].nudge(delta)

    # ---------- 상단 안내문구 ----------
    def _update_status_text(self):
        # ★ 발행자 충돌이 최우선 ★ 이 상태로는 어떤 조작도 신뢰할 수 없다.
        if self.conflict:
            self.status_label.config(
                text="⚠️ driving_node 가 떠 있습니다 — /cmd_vel_raw 발행자 충돌!\n"
                     "one_launch.py 를 내리고 이 창만 쓰세요 (두 명령이 교대해 차가 떱니다)",
                fg=CONFLICT_COLOR, font=WARN_FONT)
            return
        # 그다음 E-stop
        if self.estop_active:
            self.status_label.config(text=ESTOP_TEXT, fg=ESTOP_COLOR, font=WARN_FONT)
            return

        if self.manual_active:
            text = ("수동조종 모드 — 페달과 핸들로 직접 조종하세요. "
                    "레버는 잠기고 실측값을 비춥니다(페달을 밟으면 엑셀 레버가 올라갑니다).")
        elif self.node.auto_mode is None:
            text = ("주행모드 확인 중... B보드가 연결되면 조작할 수 있습니다. "
                    "(`ros2 run nxde arduino` 와 /board_status 를 확인하세요)")
        else:
            text = ("마우스로 엑셀·조향 레버를 움직여보세요 (키보드 ↑↓←→ 도 됩니다). "
                    "발행 토글을 ON 으로 두어야 차가 움직입니다.")
        self.status_label.config(text=text, fg=TEXT, font=STATUS_FONT)

    # ---------- 메인 tick ----------
    def _tick(self):
        if not self.running:
            return
        now = time.monotonic()

        # ── 발행자 충돌 검사 (2초 주기 — get_node_names 가 가볍지 않다) ──
        if now - self._last_conflict_check >= CONFLICT_CHECK_S:
            self._last_conflict_check = now
            conflict = self.node.driving_node_present()
            if conflict != self.conflict:
                self.conflict = conflict
                self._update_status_text()

        # ── E-stop 표시 ──
        if self.node.estop != self.estop_active:
            self.estop_active = self.node.estop
            self._update_status_text()

        # ── 주행모드 판정 ──
        #   ★ None(미수신)도 '수동'처럼 잠근다 ★ 모드를 모르는 상태에서 마우스 명령을
        #     내보내면 수동조종 중인 차에 자율 명령을 쏘는 셈이 된다.
        manual = (self.node.auto_mode is not True)
        if manual != self.manual_active:
            self.manual_active = manual
            self.enabled = False
            self.toggle_btn.config(text="OFF", bg=IDLE_COLOR)
            self._update_status_text()

        for sl in (self.throttle, self.steering, self.brake):
            sl.readonly = manual
            sl.set_handle_color(DISABLED_TEXT if manual else HANDLE_COLOR)

        if manual:
            # ── 수동조종 : 레버가 '실측을 비추는 계기판'이 된다 ──
            #   엑셀 = 페달 환산 목표펄스, 조향 = 가변저항 실측 각도(같은 부호라 그대로)
            #   브레이크는 arduino.py 의 래치가 정하므로 이 창은 값을 모른다 → 0 으로 둔다.
            self.throttle.set_value(self.node.drive_pulse_cmd)
            self.steering.set_value(self.node.steer_measured)
            self.brake.set_value(0)
            pulse_cmd = 0
            angle_cmd = self.node.steer_measured   # 전환 직후 급조향 방지
            brake_cmd = 0
            enabled = False
        else:
            pulse_cmd = self.throttle.value
            angle_cmd = self.steering.value
            brake_cmd = self.brake.value
            enabled = self.enabled
            if not enabled:
                pulse_cmd = 0   # OFF 동안에는 정지값을 계속 내보낸다(헤더 참고)

        # ── 발행 : 값이 바뀌었거나 KEEPALIVE_S 가 지났을 때 ──
        state = (pulse_cmd, angle_cmd, brake_cmd, enabled)
        if state != self._last_pub or (now - self._last_pub_t) >= KEEPALIVE_S:
            self.node.publish_cmd(pulse_cmd, angle_cmd, brake_cmd, enabled)
            self._last_pub = state
            self._last_pub_t = now

        self.brake_label.config(
            text=f"{brake_cmd} {BRAKE_LABELS.get(brake_cmd, '?')}",
            fg=ESTOP_COLOR if brake_cmd >= BRAKE_LEVEL_MAX else TEXT)

        # ── 주행모드 박스 / 페달 표시 ──
        #   /board_status 의 SRC:ovr 로 '소프트웨어가 덮어쓴 상태'를 표시한다 — 오버라이드가
        #   숨어 있으면 "스위치를 돌렸는데 왜 안 바뀌지"로 되돌아온다.
        ovr = 'SRC:ovr' in (self.node.board_status or '')
        suffix = " [SW]" if ovr else ""
        if self.node.auto_mode is None:
            self.mode_box.config(text="모드 확인 중...", bg=IDLE_COLOR, fg=TEXT)
        elif manual:
            self.mode_box.config(text="수동조종 모드" + suffix,
                                 bg=MANUAL_BOX_BG, fg=BOX_FG, activebackground=MANUAL_BOX_BG)
        else:
            self.mode_box.config(text="자율주행 모드" + suffix,
                                 bg=AUTO_BOX_BG, fg=BOX_FG, activebackground=AUTO_BOX_BG)
        self.pedal_label.config(
            text=f"페달 A0:{self.node.throttle_raw:4d} → {self.node.drive_pulse_cmd}펄스")
        self.throttle_kmh.config(text=f"{pulse_cmd * KMH_PER_PULSE:.1f} km/h")

        # ── 값 표 ──
        wp = self.node.wheel_pulse
        self.measured_vars[0].set(str(wp))
        self.measured_vars[1].set(f"{wp * KMH_PER_ENCODER_COUNT:.1f} km/h")
        self.measured_vars[2].set(f"{self.node.steer_measured}°")
        self.command_vars[0].set(str(pulse_cmd))
        self.command_vars[1].set(f"{pulse_cmd * KMH_PER_PULSE:.1f} km/h")
        self.command_vars[2].set(f"{angle_cmd}°")

        # ── 연결 상태 ──
        flags = {}
        if self.node.board_status:
            for field in self.node.board_status.split(','):
                if ':' in field:
                    k, v = field.split(':', 1)
                    flags[k.strip()] = v.strip()
        for label, key, name in ((self.conn_a, 'A', 'A보드'), (self.conn_b, 'B', 'B보드')):
            if not self.node.board_status:
                label.config(text=f"{name}: 상태 미수신", fg=DISABLED_TEXT)
            elif flags.get(key) == '1':
                label.config(text=f"{name}: 연결됨", fg=OK_COLOR)
            else:
                label.config(text=f"{name}: 연결 중...", fg=ESTOP_COLOR)

        self.root.after(UPDATE_MS, self._tick)

    # ---------- 종료 ----------
    def stop(self):
        """종료 정리. 창 닫기와 proc_guard(부모 사망) 양쪽에서 호출된다."""
        self.running = False

    def _on_close(self):
        self.stop()
        # ★ 정지값을 확실히 내보낸다 ★ A보드 펌웨어에 무입력 타임아웃이 없어서, 마지막
        #   명령이 주행값이면 arduino 노드가 그것을 1초마다 계속 재전송한다.
        self.node.publish_stop()
        self.root.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = MasterNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    gui = MasterGui(root, node)
    # 런치로 띄운 경우 이 프로세스가 고아로 남아 /cmd_vel_raw 를 계속 발행하지 않도록
    # 부모 프로세스의 종료를 감시한다 (nxde/proc_guard.py 헤더 참고)
    watch_parent(cleanup=gui.stop)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        gui.running = False
        try:
            node.publish_stop()     # arduino 가 재전송할 마지막 값을 정지값으로
        except Exception:
            pass
        time.sleep(0.1)             # 위 정지값이 실제로 나갈 시간을 준 뒤 컨텍스트를 내린다
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)
        try:
            node.destroy_node()
        except Exception:
            pass
        print("master 종료 — 정지값 발행됨", flush=True)


if __name__ == '__main__':
    main()
