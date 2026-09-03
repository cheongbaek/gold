#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
record.py ― 주행 기록 노드 [white]
─────────────────────────────────────────────────────────────────
매핑된 경로를 자율주행으로 달리는 ★그 구간만★ 골라, 주고받는 토픽 전부를
★CSV 파일 하나★ 에 담는다. one_launch.py 가 다른 노드와 함께 띄우며,
단독 실행은 `ros2 run white record`.

═══════════════════════════════════════════════════════════════════
 수집 조건 (네 가지가 ★동시에★ 참일 때만 기록)
═══════════════════════════════════════════════════════════════════
  ① /drive_cmd     prompt 의 주행 메뉴가 경로 파일명을 하달했다 ("STOP" 이 아님)
  ② /vehicle_mode  True  = 자율주행 모드 (B보드 D5 스위치)
  ③ /control_state True  = driving 이 실제로 제어 중 (도착·정지하면 False 로 내려온다)
  ④ /estop         False = 비상정지가 아님

  ①만으로는 부족하다 — prompt 가 명령을 내려도 driving 이 경로 파일을 못 찾으면
  주행은 시작되지 않는다. ③이 실제 주행 구간의 시작·끝을 정확히 잘라 준다.
  반대로 ③만 보면 master/joystick 수동조종도 /control_state 를 올리므로
  "매핑된 경로 주행"이 아닌 구간까지 섞인다. 그래서 ①과 ③을 함께 본다.

  넷 중 하나라도 깨지면 그 자리에서 세션을 닫는다. 다시 ①~④가 서지면 새 세션이다.
  (도착 → 재출발 = 파일 두 벌. 한 주행이 한 파일이다.)

═══════════════════════════════════════════════════════════════════
 출력 — 한 주행에 ★파일 하나★
═══════════════════════════════════════════════════════════════════
  <white 패키지>/ros2bag/rec_<날짜>_<시각>.csv

      1행  열 이름
      2행~ 데이터

  ★한 행 = 한 시점의 차량 전체 상태★ 다. 토픽마다 발행 주기가 제각각이라
  (driving_debug 20Hz, drive_status 1Hz, /fix 몇 Hz …) 수신할 때마다 한 행씩
  적으면 대부분의 열이 빈 희소 표가 되어 그래프도 못 그린다. 그래서 SAMPLE_HZ
  주기로 스냅샷을 찍고, 각 토픽이 그 순간 갖고 있는 최신값을 한 줄에 나란히 적는다.
  덕분에 열을 그대로 골라 바로 그래프가 된다 (예: ego_speed_ms 와 cmd_linear_x).

  · 숫자 토픽은 다음 값이 올 때까지 ★값을 유지★ 한다(hold) — 그 시점의 상태니까.
  · 문자열·이벤트 토픽(drive_event, drive_status, gps_status …)은 ★새로 온 행에만★
    적고 비운다(one-shot). 20Hz 로 같은 문장을 반복하면 파일만 커지기 때문이다.
    상태로 되살리려면 읽는 쪽에서 앞값 채우기 한 번이면 된다(pandas ffill).
  · 한 주기에 이벤트가 둘 이상 오면 " | " 로 이어 붙여 하나도 버리지 않는다.

  앞 두 열은 항상 이렇다:
      t_wall  UNIX epoch [s] — 다른 기록(SD 로그 등)과 맞출 때 쓰는 절대시각
      t_rel   세션 시작 기준 경과 [s] — 그래프의 x축

  ★콤마 걱정은 하지 않아도 된다★ — /drive_status 처럼 본문에 콤마·파이프가
  들어가는 문자열도 파이썬 csv 모듈이 자동으로 큰따옴표로 감싸(RFC 4180)
  열이 밀리지 않는다. 그래서 xlsx 가 아니라 csv 로 간다. 엑셀에서 그냥 열린다.
  (엑셀이 한글을 깨뜨리지 않도록 UTF-8 BOM 으로 쓴다.)

  주행 중에도 1초에 한 번 디스크로 밀어내므로(flush), 도중에 전원이 끊겨도
  그 직전까지는 파일에 남는다.

═══════════════════════════════════════════════════════════════════
 구독 토픽을 늘리고 싶다면
═══════════════════════════════════════════════════════════════════
  아래 RECORD_TOPICS 에 TopicSpec 을 한 줄 더 넣으면 끝이다. 구독·열 편성·
  헤더·기록이 전부 그 표만 보고 돌아간다. 코드 본문은 건드릴 필요가 없다.
  ★columns 의 이름은 파일 전체에서 유일해야 한다★ (그대로 CSV 열 이름이 된다).

  ※ 수신을 되짚어 확인하는 절차(수신 카운트 검증, 토픽 존재 확인 등)는
    일부러 두지 않았다. 그냥 구독하고, 오는 대로 적는다. 발행자가 없는
    토픽의 열은 내내 빈칸으로 남는다.
"""

import csv
import os
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Callable, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import (Bool, Float32, Float32MultiArray, Float64MultiArray,
                          Int32, String)


# ══════════════════════════════════════════════════════════════════════
#  추출 헬퍼 — 메시지 → 값 리스트
# ══════════════════════════════════════════════════════════════════════
def _scalar(msg) -> List[Any]:
    """Bool / Int32 / Float32 / String 처럼 data 하나짜리 메시지."""
    return [msg.data]


def _twist(msg) -> List[Any]:
    """Twist. 이 스택은 linear.x(속도)와 angular.z(조향)만 쓴다."""
    return [msg.linear.x, msg.angular.z]


def _array(n: int) -> Callable[[Any], List[Any]]:
    """MultiArray → 고정 길이 n 으로 맞춘 리스트.

    발행측이 필드를 늘려도 열이 밀리지 않도록 길이를 강제한다. 짧으면 빈칸,
    길면 잘라낸다(잘리면 최초 1회 경고 로그를 남긴다).
    """
    def _f(msg) -> List[Any]:
        d = list(msg.data)
        if len(d) < n:
            return d + [''] * (n - len(d))
        return d[:n]
    return _f


def _navsat(msg) -> List[Any]:
    return [msg.latitude, msg.longitude, msg.altitude,
            msg.status.status, msg.position_covariance[0]]


def _imu(msg) -> List[Any]:
    q, w, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
    return [q.x, q.y, q.z, q.w, w.x, w.y, w.z, a.x, a.y, a.z]


# ══════════════════════════════════════════════════════════════════════
#  구독 토픽 표
# ══════════════════════════════════════════════════════════════════════
@dataclass(frozen=True)
class TopicSpec:
    """기록할 토픽 하나의 명세.

    topic    : 토픽 이름
    msg_type : 메시지 클래스
    columns  : 이 토픽이 차지할 CSV 열 이름들 ★파일 전체에서 유일할 것★
    extract  : 메시지에서 columns 순서대로 값을 뽑는 함수
    hold     : True  다음 값이 올 때까지 유지 (상태·계측값)
               False 새로 온 행에만 적고 비움 (이벤트·문자열)
    note     : 사람이 읽을 설명
    """
    topic: str
    msg_type: type
    columns: Tuple[str, ...]
    extract: Callable[[Any], List[Any]]
    hold: bool = True
    note: str = ""


# /driving_debug 의 32개 필드 — driving.py control_loop 의 _publish_debug() 인자 순서.
#   ★발행측이 순서를 바꾸면 여기도 바꿔야 한다★ (driving.py 주석: "필드 위치는
#   기존 모니터/분석 호환 유지")
_DBG_COLS = (
    'dbg_wp_idx', 'dbg_wp_total', 'dbg_d2dest_m', 'dbg_cte_m', 'dbg_cte_raw_m',
    'dbg_cte_integral', 'dbg_lfd_m', 'dbg_speed_ratio', 'dbg_raw_steer_deg',
    'dbg_clamped_steer_deg', 'dbg_step_limit', 'dbg_v_target_ms', 'dbg_signed_spd_ms',
    'dbg_speed_ms', 'dbg_near_demand_deg', 'dbg_far_dist_m', 'dbg_far_demand_deg',
    'dbg_p_term', 'dbg_i_term', 'dbg_d_term', 'dbg_d_term_lpf', 'dbg_loop_dt_s',
    'dbg_is_rev', 'dbg_near_peak_deg', 'dbg_approach', 'dbg_spd_reason',
    'dbg_far_peak_deg', 'dbg_cam_lat_err_m', 'dbg_cam_lat_ok', 'dbg_cam_cte_now_m',
    'dbg_cam_cte_map_m', 'dbg_cam_bias_m',
)

RECORD_TOPICS: Tuple[TopicSpec, ...] = (
    # ── 제어 명령 ────────────────────────────────────────────────────
    TopicSpec(
        '/cmd_vel_raw', Twist,
        # [white2] cmd_linear_x 는 항상 m/s(1/5카는 펄스 양자화가 없다).
        # cmd_steer_deg 는 조향각[deg], +좌/−우 규약.
        ('cmd_linear_x', 'cmd_steer_deg'), _twist,
        note='아두이노로 나가는 최종 구동 명령 (카메라 ON 이면 게이트 통과분)'),
    TopicSpec(
        '/cmd_vel_drive', Twist,
        ('drv_linear_x', 'drv_steer_deg'), _twist,
        note='driving 원본 명령 — cmd_* 와 비교하면 신호등 게이트 개입이 보인다'),
    TopicSpec(
        '/drive_pulse_cmd', Int32,
        ('drive_pulse_cmd',), _scalar,
        note='A보드로 실제 送出된 펄스값'),
    TopicSpec(
        '/brake_level', Int32,
        ('brake_level',), _scalar,
        note='리니어 브레이크 단계 0/1/2'),

    # ── 차량 상태 ────────────────────────────────────────────────────
    TopicSpec(
        '/ego_state', Float64MultiArray,
        ('ego_lat', 'ego_lon', 'ego_x_m', 'ego_y_m', 'ego_heading_deg',
         'ego_speed_ms', 'ego_reserved', 'ego_pitch_deg', 'ego_terrain'),
        _array(9),
        note='gps_imu 융합 결과 (위치는 뒷차축 투영)'),
    TopicSpec(
        '/encoder', Int32,
        ('encoder_pulse',), _scalar,
        note='A보드 좌+우 펄스 합 (부호 없음, 20Hz)'),
    TopicSpec(
        '/steer_angle_measured', Int32,
        ('steer_measured_deg',), _scalar,
        note='B보드 포텐셔미터 실측 조향각'),
    TopicSpec(
        '/throttle_pedal', Int32,
        ('throttle_raw',), _scalar,
        note='쓰로틀 페달 raw 0~1023 — 자율주행 중 사람 개입 흔적'),

    # ── 모드 · 안전 (게이트 판정에도 쓰인다) ─────────────────────────
    TopicSpec(
        '/control_state', Bool,
        ('control_state',), _scalar,
        note='제어 활성 여부 ★세션 경계 신호★'),
    TopicSpec(
        '/vehicle_mode', Bool,
        ('vehicle_mode',), _scalar,
        note='True 자율주행 / False 수동조종 (B보드 D5)'),
    TopicSpec(
        '/estop', Bool,
        ('estop',), _scalar,
        note='비상정지'),
    TopicSpec(
        '/drive_cmd', String,
        ('drive_cmd',), _scalar, hold=False,
        note='prompt 하달 — 경로 파일명 또는 STOP ★세션 경계 신호★'),

    # ── 주행 판단 내부값 ─────────────────────────────────────────────
    TopicSpec(
        '/driving_debug', Float64MultiArray,
        _DBG_COLS, _array(len(_DBG_COLS)),
        note='driving 제어 내부값 20Hz — 사후 분석의 핵심'),
    TopicSpec(
        '/drive_status', String,
        ('drive_status',), _scalar, hold=False,
        note='주행 요약 1Hz (본문에 콤마 있음 — csv 가 알아서 감싼다)'),
    TopicSpec(
        '/drive_event', String,
        ('drive_event',), _scalar, hold=False,
        note='도착·정지 등 이벤트'),
    TopicSpec(
        '/gps_status', String,
        ('gps_status',), _scalar, hold=False,
        note='RTK 품질 / 헤딩고정 상태'),

    # ── 카메라 (use_camera:=false 면 발행자가 없어 열이 내내 빈칸이다) ──
    TopicSpec(
        '/lane_metrics', Float32MultiArray,
        ('lane_cte_rear_m', 'lane_cte_near_m', 'lane_theta_deg', 'lane_curv_1pm',
         'lane_conf_eff', 'lane_width_m', 'lane_flags', 'lane_d_near_m',
         'lane_conf_raw', 'lane_seq'),
        _array(10),
        note='camera_judgment 의 미터단위 차선계측'),
    TopicSpec(
        '/judgment_state', String,
        ('judgment_state',), _scalar, hold=False,
        note='신호등 게이트 FSM (TL_STOP 등)'),
    TopicSpec(
        '/tl/state', String,
        ('tl_state',), _scalar, hold=False,
        note='perception 신호등 판정 RED/GREEN/UNKNOWN'),
    TopicSpec(
        '/stop_line_dist', Float32,
        ('stop_line_dist_m',), _scalar,
        note='정지선까지 거리, 미검출 -1'),

    # ── 원시 센서 ────────────────────────────────────────────────────
    TopicSpec(
        '/fix', NavSatFix,
        ('fix_lat', 'fix_lon', 'fix_alt_m', 'fix_status', 'fix_cov_xx'), _navsat,
        note='GPS 원시 (융합 전)'),
    TopicSpec(
        '/imu/data', Imu,
        ('imu_quat_x', 'imu_quat_y', 'imu_quat_z', 'imu_quat_w',
         'imu_gyro_x', 'imu_gyro_y', 'imu_gyro_z',
         'imu_acc_x', 'imu_acc_y', 'imu_acc_z'), _imu,
        note='iAHRS 원시 (융합 전)'),
)

# 세션 경계를 판정하는 토픽 — 위 표에도 들어 있고, 콜백에서 게이트도 갱신한다.
GATE_TOPICS = ('/drive_cmd', '/vehicle_mode', '/control_state', '/estop')

# 공통 앞머리 두 열 + 전 토픽의 열을 이어 붙인 최종 헤더
COMMON_COLUMNS = ('t_wall', 't_rel')
ALL_COLUMNS: Tuple[str, ...] = COMMON_COLUMNS + tuple(
    c for spec in RECORD_TOPICS for c in spec.columns)


# ══════════════════════════════════════════════════════════════════════
#  저장 위치
# ══════════════════════════════════════════════════════════════════════
def resolve_output_dir(explicit: str = "") -> str:
    """ros2bag 디렉터리를 정한다. 우선순위:

      1) 파라미터 output_dir
      2) 환경변수 WHITE_RECORD_DIR
      3) ★소스 트리★ <...>/src/white/ros2bag
         — colcon 을 --symlink-install 로 빌드하면 이 파일이 소스를 가리키는
           심볼릭 링크라 realpath 로 소스 위치를 되찾을 수 있다.
      4) 셋 다 실패하면 ~/ros2bag

    3) 이 실패하는 경우(심볼릭 링크 없이 빌드)에는 설치본 안에 쌓지 않고
    홈으로 보낸다 — install/ 은 재빌드하면 날아가므로 기록을 두면 안 된다.
    """
    if explicit:
        return os.path.abspath(os.path.expanduser(explicit))

    env = os.environ.get('WHITE_RECORD_DIR', '').strip()
    if env:
        return os.path.abspath(os.path.expanduser(env))

    here = os.path.dirname(os.path.realpath(__file__))     # .../src/white/white
    pkg_root = os.path.dirname(here)                       # .../src/white
    if os.path.isfile(os.path.join(pkg_root, 'package.xml')):
        return os.path.join(pkg_root, 'ros2bag')

    return os.path.expanduser('~/ros2bag')


# ══════════════════════════════════════════════════════════════════════
#  노드
# ══════════════════════════════════════════════════════════════════════
class RecordNode(Node):

    STATUS_PERIOD_S = 10.0     # 대기 중 사유를 알려주는 주기
    FLUSH_EVERY_ROWS = 20      # 1초분(20Hz)마다 디스크로 밀어낸다

    def __init__(self):
        super().__init__('record_node')

        self.declare_parameter('output_dir', '')
        self.declare_parameter('require_drive_cmd', True)
        self.declare_parameter('sample_hz', 20.0)

        self.out_root = resolve_output_dir(
            self.get_parameter('output_dir').value or '')
        self.require_drive_cmd = bool(
            self.get_parameter('require_drive_cmd').value)
        # 20Hz = driving_debug·cmd_vel 의 발행주기. 이보다 빠른 토픽(imu 등)은
        # 주기 안에서 마지막 값만 남는다 — 더 촘촘히 남기려면 이 값을 올린다.
        self.sample_hz = max(1.0, float(self.get_parameter('sample_hz').value))

        # ── 게이트 상태 ──────────────────────────────────────────────
        #   None = 아직 한 번도 못 받음. 자율주행 여부를 모르는 채로 기록을
        #   시작하지 않는다(수동조종 구간이 섞이는 것을 막는다).
        self.armed: bool = not self.require_drive_cmd
        self.route_name: str = ''
        self.auto_mode: Optional[bool] = None
        self.control_state: Optional[bool] = None
        self.estop: bool = False

        # ── 스냅샷 버퍼 ──────────────────────────────────────────────
        #   hold  토픽 : 다음 값이 올 때까지 남아 매 행에 다시 찍힌다
        #   1회성 토픽 : _pending 에 모였다가 한 행에 찍히고 지워진다
        self._hold: Dict[str, Any] = {}
        self._pending: Dict[str, str] = {}
        self._rx: Dict[str, int] = {spec.topic: 0 for spec in RECORD_TOPICS}
        self._warned: set = set()

        # ── 세션 상태 ────────────────────────────────────────────────
        self.recording: bool = False
        self.csv_path: str = ''
        self.session_t0: float = 0.0
        self._fp = None
        self._writer = None
        self._rows: int = 0

        self._last_status_log = 0.0

        # ── 구독 (표 하나로 전부) ────────────────────────────────────
        for spec in RECORD_TOPICS:
            self.create_subscription(
                spec.msg_type, spec.topic,
                (lambda msg, s=spec: self._on_msg(s, msg)), 10)

        self.create_timer(1.0 / self.sample_hz, self._tick)
        self.create_timer(1.0, self._status_tick)

        self.get_logger().info(
            f"📼 record 대기 — 기록 위치: {self.out_root}\n"
            f"   토픽 {len(RECORD_TOPICS)}개 → 통합 CSV {len(ALL_COLUMNS)}열, "
            f"{self.sample_hz:.0f}Hz 스냅샷 | 수집 조건: "
            f"{'/drive_cmd(경로) + ' if self.require_drive_cmd else ''}"
            f"/vehicle_mode=True + /control_state=True + /estop=False")

    # ══════════════════════════════════════════════════════════════════
    #  수신
    # ══════════════════════════════════════════════════════════════════
    def _on_msg(self, spec: TopicSpec, msg):
        was = self.recording

        if spec.topic in GATE_TOPICS:
            self._update_gate(spec.topic, msg)

        # 세션 밖에서도 값은 담아 둔다 — 세션 첫 행이 빈칸으로 시작하지 않도록.
        self._stash(spec, msg)

        now_on = self._should_record()
        if now_on and not was:
            self._start_session()
        elif was and not now_on:
            # 종료 사유가 담긴 마지막 스냅샷을 한 줄 남기고 닫는다.
            self._write_row()
            self._stop_session()

    def _stash(self, spec: TopicSpec, msg):
        try:
            values = spec.extract(msg)
        except Exception as e:                      # 발행측 형식이 바뀐 경우
            self.get_logger().warning(f"{spec.topic} 추출 실패: {e}")
            return
        self._rx[spec.topic] += 1
        if len(values) != len(spec.columns) and spec.topic not in self._warned:
            self._warned.add(spec.topic)
            self.get_logger().warning(
                f"{spec.topic} 필드 수가 표와 다르다 — record.py 의 columns 확인 필요")

        if spec.hold:
            for col, val in zip(spec.columns, values):
                self._hold[col] = val
        else:
            # 1회성 — 한 주기에 둘 이상 오면 이어 붙여 하나도 버리지 않는다.
            for col, val in zip(spec.columns, values):
                text = str(val)
                prev = self._pending.get(col)
                self._pending[col] = f"{prev} | {text}" if prev else text

    def _update_gate(self, topic: str, msg):
        if topic == '/drive_cmd':
            cmd = str(msg.data).strip()
            if cmd.upper() == 'STOP':
                self.armed = False
            elif cmd:
                self.armed = True
                self.route_name = cmd
        elif topic == '/vehicle_mode':
            self.auto_mode = bool(msg.data)
        elif topic == '/control_state':
            self.control_state = bool(msg.data)
        elif topic == '/estop':
            self.estop = bool(msg.data)

    def _should_record(self) -> bool:
        if self.require_drive_cmd and not self.armed:
            return False
        return (self.auto_mode is True
                and self.control_state is True
                and not self.estop)

    def _blocking_reason(self) -> str:
        if self.require_drive_cmd and not self.armed:
            return "prompt 주행명령 대기 (/drive_cmd)"
        if self.auto_mode is None:
            return "주행모드 미수신 — nxde arduino 노드 확인 (/vehicle_mode)"
        if not self.auto_mode:
            return "수동조종 모드 (B보드 D5)"
        if self.estop:
            return "E-stop 발동 중"
        if self.control_state is not True:
            return "driving 제어 비활성 (/control_state)"
        return ""

    # ══════════════════════════════════════════════════════════════════
    #  세션
    # ══════════════════════════════════════════════════════════════════
    def _start_session(self):
        os.makedirs(self.out_root, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(self.out_root, f"rec_{stamp}.csv")
        # 같은 초에 두 세션이 시작되는 극단적인 경우에만 뒤에 번호를 붙인다
        # (도착 직후 재출발). 덮어써서 앞 주행을 잃는 것보다 낫다.
        n = 2
        while os.path.exists(self.csv_path):
            self.csv_path = os.path.join(self.out_root, f"rec_{stamp}_{n}.csv")
            n += 1
        # utf-8-sig : 엑셀이 한글 헤더를 깨뜨리지 않게 BOM 을 넣는다.
        self._fp = open(self.csv_path, 'w', newline='', encoding='utf-8-sig')
        self._writer = csv.writer(self._fp)
        self._writer.writerow(ALL_COLUMNS)
        self.session_t0 = time.time()
        self._rows = 0
        self._rx = {spec.topic: 0 for spec in RECORD_TOPICS}
        self.recording = True
        self.get_logger().info(
            f"🔴 기록 시작 [{self.route_name or '경로 미상'}] → {self.csv_path}")

    def _tick(self):
        if self.recording:
            self._write_row()

    def _write_row(self):
        if self._writer is None:
            return
        now = time.time()
        row = [f"{now:.6f}", f"{now - self.session_t0:.3f}"]
        for col in ALL_COLUMNS[len(COMMON_COLUMNS):]:
            if col in self._pending:
                row.append(self._pending.pop(col))     # 1회성 — 찍고 비운다
            else:
                row.append(self._hold.get(col, ''))    # 유지값 — 없으면 빈칸
        self._writer.writerow(row)
        self._rows += 1
        if self._rows % self.FLUSH_EVERY_ROWS == 0:
            try:
                self._fp.flush()
            except Exception:
                pass

    def _stop_session(self):
        if not self.recording:
            return
        self.recording = False
        dur = time.time() - self.session_t0
        reason = self._blocking_reason() or "종료"
        try:
            self._fp.close()
        except Exception:
            pass
        self._fp = None
        self._writer = None
        self.get_logger().info(
            f"⏹️ 기록 종료 ({reason}) — {dur:.1f}초, {self._rows}행 → {self.csv_path}")

    # ══════════════════════════════════════════════════════════════════
    #  주기 로그 — 왜 기록이 안 되는지 알려준다
    # ══════════════════════════════════════════════════════════════════
    def _status_tick(self):
        now = time.time()
        if now - self._last_status_log < self.STATUS_PERIOD_S:
            return
        self._last_status_log = now
        if self.recording:
            live = sum(1 for n in self._rx.values() if n > 0)
            self.get_logger().info(
                f"📼 기록 중 {now - self.session_t0:.0f}초 | "
                f"{self._rows}행 | 수신 토픽 {live}/{len(RECORD_TOPICS)}")
        else:
            self.get_logger().info(f"📼 대기 — {self._blocking_reason()}")

    def destroy_node(self):
        if self.recording:
            self._write_row()
            self._stop_session()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RecordNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 주행 도중 Ctrl+C 로 내려도 세션을 정상 마감한다.
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
