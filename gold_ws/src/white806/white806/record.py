#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
record.py ― 주행 기록 [white806]
════════════════════════════════════════════════════════════════════════════════
자율주행 구간만 골라 주고받는 토픽 전부를 ★CSV 파일 하나★ 에 담는다.
one_launch.py 가 함께 띄우며, 단독 실행은 `ros2 run white806 record`.

════════════════════════════════════════════════════════════════════════════════
 기록 구간 — driving 의 상태를 그대로 따른다
════════════════════════════════════════════════════════════════════════════════
  /drive_state 가 DRIVE_HEADING · DRIVE_RUN · DRIVE_DONE 일 때 기록한다.
  즉 ★스위치를 올려 주행이 시작되는 순간부터, 도착해 선 뒤 스위치를 내릴 때까지★ 다.
  헤딩 초기화 구간을 포함하는 이유는 그것도 주행의 일부이고, 초기 헤딩이 틀렸을 때
  원인을 찾으려면 그 구간의 GPS 가 남아 있어야 하기 때문이다.

  매핑(MAP_*)은 기록하지 않는다 — 그쪽 산출물은 mapping 노드의 경로 CSV 다.
  E-stop 이 걸리면 driving 이 IDLE/ESTOP 으로 빠지므로 자동으로 닫힌다.

════════════════════════════════════════════════════════════════════════════════
 출력 — 한 주행에 ★파일 하나★
════════════════════════════════════════════════════════════════════════════════
  <white806 패키지>/ros2bag/rec_<날짜>_<시각>.csv
      1행  열 이름 / 2행~ 데이터

  ★한 행 = 한 시점의 차량 전체 상태★ 다. 토픽마다 발행 주기가 달라서 수신할 때마다
  한 행씩 적으면 대부분 칸이 빈 희소 표가 된다. SAMPLE_HZ 주기로 스냅샷을 찍어
  각 토픽의 그 순간 최신값을 한 줄에 나란히 적는다 — 열을 골라 바로 그래프가 된다.

  · 숫자 토픽은 다음 값이 올 때까지 ★값을 유지★ 한다(hold).
  · 이벤트·명령 문자열은 ★새로 온 행에만★ 적고 비운다(one-shot). 같은 문장을 20Hz
    로 반복하지 않기 위함이고, 상태로 되살리려면 읽는 쪽에서 ffill 한 번이면 된다.
  · 한 주기에 이벤트가 둘 이상 오면 " | " 로 이어 붙여 하나도 버리지 않는다.

  앞 두 열은 t_wall(UNIX epoch) · t_rel(세션 시작 기준 경과 초)이다.
  콤마가 든 문자열도 csv 모듈이 큰따옴표로 감싸므로 열이 밀리지 않는다.
  1초에 한 번 flush 하므로 도중에 전원이 끊겨도 직전까지는 남는다.

  토픽을 늘리려면 RECORD_TOPICS 에 TopicSpec 한 줄만 더하면 된다
  (columns 이름은 파일 전체에서 유일해야 한다). 수신 여부를 되짚어 확인하는
  절차는 일부러 두지 않았다 — 그냥 구독하고 오는 대로 적는다.

════════════════════════════════════════════════════════════════════════════════
 ★[2026-08-08] '주행이 잘 되었는가' 를 판정하기 위한 열 추가★
════════════════════════════════════════════════════════════════════════════════
  종전 표에는 ★제어의 입력과 출력만★ 있고 제어가 얼마나 잘 됐는지를 말해 주는
  값이 없었다. /cmd_vel_raw(무엇을 시켰나) 와 /ego_state(어디에 있나) 는 있는데,
  '경로에서 얼마나 벗어났나' 가 없어서 로그만 보고는 성패를 판정할 수 없었다.

  · /drive_diag  — driving 이 내놓는 추종 진단 13종. 아래 네 묶음이다.
      추종품질 : cte_m(★부호 있는 경로이탈 — 이 열 하나가 성패 판정의 핵심★),
                 heading_err_deg(제어기 입력 오차), target_dist_m(실효 선행거리)
      진행     : target_idx, goal_dist_m
      헤딩건전 : gps_course_deg, fuse_corr_deg, gyro_z_dps
                 → 융합이 살아 있는지(RTK 가 Fixed 를 벗어나면 조용히 멈춘다),
                   자이로 부호가 맞는지를 사후에 확인할 수 있다
      출발조건 : head_init_* 4종. 확정 시점 값을 그대로 붙들고 있으므로
                 '이 주행은 σ 몇 도짜리 헤딩으로 출발했나' 가 매 행에 남는다
  · /board_status — A/B 보드 링크 상태. B보드 USB 가 끊기면 D5(주행모드)가
      멈춰 상태기계가 굳는데, 그 원인을 로그에서 구별할 수단이 없었다.

  ※ /drive_diag 를 아무도 발행하지 않아도 record 는 그대로 돈다 — 해당 열이
    빈 칸으로 남을 뿐이다. 그래서 driving 쪽 발행 추가와 무관하게 배포해도 된다.
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
from std_msgs.msg import Bool, Float32, Float64MultiArray, Int32, String

from white806 import paths


# 이 상태들에서만 기록한다 (driving.py 의 상태 이름과 같아야 한다)
RECORD_STATES = ('DRIVE_HEADING', 'DRIVE_RUN', 'DRIVE_DONE')


# ══════════════════════════════════════════════════════════════════════════════
#  추출 헬퍼
# ══════════════════════════════════════════════════════════════════════════════
def _scalar(msg) -> List[Any]:
    return [msg.data]


def _twist(msg) -> List[Any]:
    """이 스택은 linear.x(펄스)와 angular.z(조향각)만 쓴다."""
    return [msg.linear.x, msg.angular.z]


def _array(n: int) -> Callable[[Any], List[Any]]:
    """MultiArray → 고정 길이 n. 발행측이 늘려도 열이 밀리지 않게 강제한다."""
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


# ══════════════════════════════════════════════════════════════════════════════
#  구독 토픽 표
# ══════════════════════════════════════════════════════════════════════════════
@dataclass(frozen=True)
class TopicSpec:
    topic: str
    msg_type: type
    columns: Tuple[str, ...]
    extract: Callable[[Any], List[Any]]
    hold: bool = True
    note: str = ""


RECORD_TOPICS: Tuple[TopicSpec, ...] = (
    # ── 제어 명령 ──
    TopicSpec('/cmd_vel_raw', Twist,
              ('cmd_pulse', 'cmd_steer_deg'), _twist,
              note='아두이노로 나가는 최종 명령 (펄스 0~15 / 조향 −좌 +우)'),
    TopicSpec('/drive_pulse_cmd', Int32, ('drive_pulse_cmd',), _scalar,
              note='A보드로 실제 送出된 펄스'),
    TopicSpec('/brake_level', Int32, ('brake_level',), _scalar,
              note='리니어 브레이크 0/1/2'),

    # ── 차량 상태 ──
    #   /ego_state 는 driving 이 만든다: [x, y, heading, enc_pulse, wp_idx, wp_total, fix_ok]
    TopicSpec('/ego_state', Float64MultiArray,
              ('ego_x_m', 'ego_y_m', 'ego_heading_deg', 'ego_pulse',
               'ego_wp_idx', 'ego_wp_total', 'ego_fix_ok'),
              _array(7),
              note='driving 이 직접 만든 위치·헤딩(로컬 평면) + 진행률'),
    # ── 추종 진단 ★주행 성패를 판정하는 열들★ ──
    #   /drive_diag 는 driving 이 만든다(제어에 쓰지 않는 계측 전용 배열):
    #     [cte, head_err, target_idx, target_dist, goal_dist,
    #      gps_course, fuse_corr, gyro_z, brake_latched,
    #      head_init_deg, head_sigma, head_resid, head_dist,
    #      ref_pulse, out_pulse, meas_pulse,        ← [2026-08-12] 저속 보정 3종
    #      cte_integral, cte_i_term_deg]            ← [2026-08-12] CTE 적분항 2종
    TopicSpec('/drive_diag', Float64MultiArray,
              ('cte_m', 'heading_err_deg', 'target_idx', 'target_dist_m',
               'goal_dist_m', 'gps_course_deg', 'fuse_corr_deg', 'gyro_z_dps',
               'brake_latched', 'head_init_deg', 'head_sigma_deg',
               'head_resid_m', 'head_dist_m',
               'ref_pulse', 'out_pulse', 'meas_pulse',
               'cte_integral', 'cte_i_term_deg'),
              _array(18),
              note='★cte_m 이 핵심★ 경로이탈 +왼쪽/−오른쪽. 나머지는 헤딩 융합 '
                   '건전성과 출발 헤딩 품질. ref/out/meas 는 저속 펄스 보정 검증용 — '
                   'out≠ref 인 구간이 보정이 걸린 구간이다. cte_i_term_deg 는 '
                   'CTE 적분이 조향에 더한 도로휠각(pot 기준 ×1.75)'),

    TopicSpec('/encoder', Int32, ('encoder_sum',), _scalar,
              note='A보드 좌+우 펄스 합'),
    TopicSpec('/speed', Float32, ('speed_kmh',), _scalar,
              note='speed.py 의 IMU 적분 속도[km/h]. ★절대값은 못 믿는다★ — '
                   '정지/기동 판정용(speed.py 헤더의 정확도 실측 참고)'),
    TopicSpec('/steer_angle_measured', Int32, ('steer_measured_deg',), _scalar,
              note='B보드 실측 조향각'),
    TopicSpec('/throttle_pedal', Int32, ('throttle_raw',), _scalar,
              note='쓰로틀 페달 raw — 자율주행 중 사람 개입 흔적'),

    # ── 모드 · 안전 ──
    TopicSpec('/control_state', Bool, ('control_state',), _scalar,
              note='구동 허용'),
    TopicSpec('/vehicle_mode', Bool, ('vehicle_mode',), _scalar,
              note='True 자율 / False 수동 (B보드 D5)'),
    TopicSpec('/estop', Bool, ('estop',), _scalar, note='비상정지'),
    TopicSpec('/board_status', String, ('board_status',), _scalar,
              note='A:1,B:1,ESTOP:0,MODE:1 — B보드 링크가 끊기면 D5 가 멈춰 '
                   '상태기계가 굳는다. 그 구간을 로그에서 구별하는 유일한 단서'),
    TopicSpec('/drive_state', String, ('drive_state',), _scalar,
              note='driving 상태기계 ★기록 구간을 정하는 신호★'),
    TopicSpec('/drive_cmd', String, ('drive_cmd',), _scalar, hold=False,
              note='prompt 하달 (경로 선택 / STOP)'),
    TopicSpec('/drive_event', String, ('drive_event',), _scalar, hold=False,
              note='헤딩 확정·도착·브레이크 등 이벤트'),

    # ── 원시 센서 ──
    TopicSpec('/fix', NavSatFix,
              ('fix_lat', 'fix_lon', 'fix_alt_m', 'fix_status', 'fix_cov_xx'),
              _navsat, note='GPS 원시'),
    TopicSpec('/imu', Imu,
              ('imu_quat_x', 'imu_quat_y', 'imu_quat_z', 'imu_quat_w',
               'imu_gyro_x', 'imu_gyro_y', 'imu_gyro_z',
               'imu_acc_x', 'imu_acc_y', 'imu_acc_z'),
              _imu, note='iAHRS 원시 6축'),
)

COMMON_COLUMNS = ('t_wall', 't_rel')
ALL_COLUMNS: Tuple[str, ...] = COMMON_COLUMNS + tuple(
    c for spec in RECORD_TOPICS for c in spec.columns)


# ══════════════════════════════════════════════════════════════════════════════
#  노드
# ══════════════════════════════════════════════════════════════════════════════
class RecordNode(Node):

    STATUS_PERIOD_S = 10.0
    FLUSH_EVERY_ROWS = 20

    def __init__(self):
        super().__init__('record_node')

        self.declare_parameter('output_dir', '')
        self.declare_parameter('sample_hz', 20.0)

        self.out_root = paths.record_dir(self.get_parameter('output_dir').value or '')
        # 20Hz = driving 제어주기. 더 빠른 토픽(imu)은 주기 안에서 마지막 값만 남는다.
        self.sample_hz = max(1.0, float(self.get_parameter('sample_hz').value))

        self.drive_state = 'IDLE'

        self._hold: Dict[str, Any] = {}
        self._pending: Dict[str, str] = {}
        self._rx: Dict[str, int] = {s.topic: 0 for s in RECORD_TOPICS}
        self._warned: set = set()

        self.recording = False
        self.csv_path = ''
        self.session_t0 = 0.0
        self._fp = None
        self._writer = None
        self._rows = 0
        self._last_status_log = 0.0

        for spec in RECORD_TOPICS:
            self.create_subscription(
                spec.msg_type, spec.topic,
                (lambda msg, s=spec: self._on_msg(s, msg)), 10)

        self.create_timer(1.0 / self.sample_hz, self._tick)
        self.create_timer(1.0, self._status_tick)

        self.get_logger().info(
            f"📼 record 대기 — 기록 위치: {self.out_root}\n"
            f"   토픽 {len(RECORD_TOPICS)}개 → 통합 CSV {len(ALL_COLUMNS)}열, "
            f"{self.sample_hz:.0f}Hz 스냅샷 | 기록 구간: {', '.join(RECORD_STATES)}")

    # ── 수신 ───────────────────────────────────────────────────────────────────
    def _on_msg(self, spec: TopicSpec, msg):
        was = self.recording

        if spec.topic == '/drive_state':
            self.drive_state = str(msg.data)

        self._stash(spec, msg)

        now_on = self.drive_state in RECORD_STATES
        if now_on and not was:
            self._start_session()
        elif was and not now_on:
            self._write_row()          # 종료 사유가 담긴 마지막 한 줄
            self._stop_session()

    def _stash(self, spec: TopicSpec, msg):
        try:
            values = spec.extract(msg)
        except Exception as e:
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
            for col, val in zip(spec.columns, values):
                prev = self._pending.get(col)
                text = str(val)
                self._pending[col] = f"{prev} | {text}" if prev else text

    # ── 세션 ───────────────────────────────────────────────────────────────────
    def _start_session(self):
        os.makedirs(self.out_root, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(self.out_root, f"rec_{stamp}.csv")
        n = 2
        while os.path.exists(self.csv_path):   # 같은 초에 두 번 시작한 경우만
            self.csv_path = os.path.join(self.out_root, f"rec_{stamp}_{n}.csv")
            n += 1
        # utf-8-sig : 엑셀이 한글 헤더를 깨뜨리지 않게 BOM
        self._fp = open(self.csv_path, 'w', newline='', encoding='utf-8-sig')
        self._writer = csv.writer(self._fp)
        self._writer.writerow(ALL_COLUMNS)
        self.session_t0 = time.time()
        self._rows = 0
        self._rx = {s.topic: 0 for s in RECORD_TOPICS}
        self.recording = True
        self.get_logger().info(f"🔴 기록 시작 → {self.csv_path}")

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
                row.append(self._pending.pop(col))
            else:
                row.append(self._hold.get(col, ''))
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
        try:
            self._fp.close()
        except Exception:
            pass
        self._fp = self._writer = None
        self.get_logger().info(
            f"⏹️ 기록 종료 ({self.drive_state}) — {dur:.1f}초, {self._rows}행 "
            f"→ {self.csv_path}")

    def _status_tick(self):
        now = time.time()
        if now - self._last_status_log < self.STATUS_PERIOD_S:
            return
        self._last_status_log = now
        if self.recording:
            live = sum(1 for n in self._rx.values() if n > 0)
            self.get_logger().info(
                f"📼 기록 중 {now - self.session_t0:.0f}초 | {self._rows}행 | "
                f"수신 토픽 {live}/{len(RECORD_TOPICS)}")

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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
