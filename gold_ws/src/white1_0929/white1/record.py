#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
record.py ― 주행 기록 [white1]
════════════════════════════════════════════════════════════════════════════════
자율주행 구간만 골라 주고받는 토픽 전부를 ★CSV 파일 하나★ 에 담는다.
one_launch.py 가 함께 띄우며, 단독 실행은 `ros2 run white1 record`.

════════════════════════════════════════════════════════════════════════════════
 기록 구간 — driving 의 상태를 그대로 따른다
════════════════════════════════════════════════════════════════════════════════
  /drive_state 가 DRIVE_HEADING · DRIVE_RUN · DRIVE_DONE 일 때 기록한다.
  즉 ★스위치를 올려 주행이 시작되는 순간부터, 도착해 선 뒤 스위치를 내릴 때까지★ 다.
  헤딩 초기화 구간을 포함하는 이유는 그것도 주행의 일부이고, 초기 헤딩이 틀렸을 때
  원인을 찾으려면 그 구간의 GPS 가 남아 있어야 하기 때문이다.

  매핑(MAP_*)은 기록하지 않는다 — 그쪽 산출물은 mapping 노드의 경로 CSV 다.
  E-STOP(D12)이 걸리면 driving 이 자율주행을 취소하고 IDLE 로 빠지므로 자동으로 닫힌다.

  ★[2026-08-14] force_record — driving 없이 혼자 기록하기★
      ros2 run white1 record --ros-args -p force_record:=true
  조이스틱 수동조종만으로 계측할 때는 /drive_state 가 오지 않아(또는 IDLE 이라)
  위 규칙으로는 ★파일이 열리지 않는다★. 이 스위치를 켜면 상태와 무관하게 노드가
  뜨는 즉시 기록을 시작하고 Ctrl-C 로 내릴 때까지 적는다. 파일명 앞부분은
  manual 이 된다(자율주행 기록의 unknown 과 구별하기 위해서다).
  ※ one_launch.py 는 이 값을 주지 않는다 — 자율주행 기록 규칙은 종전 그대로다.

════════════════════════════════════════════════════════════════════════════════
 출력 — 한 주행에 ★파일 하나★
════════════════════════════════════════════════════════════════════════════════
  <white1 패키지>/ros2bag/<주행한 경로 CSV 이름>-<날짜>_<시각>.csv
      1행  열 이름 / 2행~ 데이터

  ★[2026-08-12] 파일명 앞에 '무엇을 따라 달렸는가' 를 붙인다★ 종전 rec_<시각>.csv
  는 기록 시각만 남아서, 로그를 나중에 열었을 때 어느 경로(gps_data/route_*.csv)로
  달린 주행인지 파일 목록만 보고는 알 수 없었다 — 같은 날 여러 경로를 번갈아
  달리면 특히 그렇다. 경로 이름은 prompt 가 /drive_cmd 로 보내는 파일명을 그대로
  받아 적는다(선택 실패한 이름은 driving 이 거절하므로 여기로 오지 않는다).

      gps_data/route_20260811_160932.csv 로 주행
        → ros2bag/route_20260811_160932-20260812_134501.csv

  경로 이름을 못 들은 채(record 를 주행 도중에 새로 띄운 경우 등) 기록이 시작되면
  앞부분이 unknown 이 된다 — 기록을 거르지는 않는다.

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
import re
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Callable, Dict, List, Optional, Tuple

import rclpy
import rclpy.executors
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Bool, Float32, Float64MultiArray, Int32, String
from nav_msgs.msg import Path as NavPath

from white1 import paths


# 이 상태들에서만 기록한다 (driving.py 의 상태 이름과 같아야 한다)
RECORD_STATES = ('DRIVE_HEADING', 'DRIVE_RUN', 'DRIVE_DONE')

# ── 파일명 앞부분(= 주행한 경로)을 알아내는 두 경로 ──────────────────────────────
#   1) /drive_cmd : prompt 가 경로를 고르는 순간 파일명을 그대로 보낸다. 아래 세
#      단어는 명령이지 파일명이 아니다.
#   2) /drive_event : driving 이 '경로 선택'·'주행 시작' 을 알릴 때 이름을 함께
#      적는다. record 를 나중에 띄워 1) 을 놓쳤을 때의 보조 수단이다 — 이 이벤트는
#      driving 의 enter() 안에서 /drive_state 보다 ★먼저★ 나가므로 세션 시작
#      시점에는 이미 도착해 있다.
ROUTE_CMD_WORDS = ('STOP', 'MAP_START', 'DRIVE_START')
ROUTE_EVENT_HINTS = ('경로 선택', '주행 시작')
ROUTE_IN_TEXT = re.compile(r'([^\s\[\]/\\]+\.csv)')
UNKNOWN_ROUTE = 'unknown'
#  force_record 로 혼자 도는 기록의 파일명 앞부분. 'unknown'(경로 이름을 놓친 자율주행
#  기록)과 구별해야 나중에 폴더만 보고도 '수동 계측'인지 알 수 있다.
MANUAL_ROUTE = 'manual'


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
              note='리니어 브레이크 0/1/2 — ★요청한 단계★ (실제 위치는 brake_pot)'),
    # ★[2026-08-21] 요청(brake_level)과 실측(brake_pot)을 나란히 둔다★
    #   종전에는 제동에 관해 남는 것이 '몇 단을 요청했나' 하나뿐이라, 리니어가 실제로
    #   그만큼 갔는지도 사람이 발로 더 밟았는지도 로그로는 알 수 없었다. 두 열을
    #   같이 놓으면 그 셋이 구별된다:
    #       brake_level 2 + brake_pot 850 → 시킨 대로 갔다
    #       brake_level 2 + brake_pot 620 → ★행정이 덜 나왔다★ (기구·전원 의심)
    #       brake_level 0 + brake_pot 700 → ★사람이 발로 밟았다★ (수동조종 구간)
    #   기준값은 B보드 상수다 — 1단 600 / 2단 850 / 제동등 점등 ★350★
    #   ([2026-09-09 정정] 0821 초판 400 → 실차에서 350. kasa_0909_B.ino
    #    BRAKELIGHT_ON_RAW. 400 으로 읽으면 점등 구간을 좁게 본다)
    TopicSpec('/brake_pot', Int32, ('brake_pot',), _scalar,
              note='B보드 A5 리니어 가변저항 raw 0~1023 — 브레이크 페달 ★실제 위치★. '
                   '1단 목표 600 / 2단 850 / ★350★ 이상이면 제동등 점등(D11)'),

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
    #      cte_integral, cte_i_term_deg,            ← [2026-08-12] CTE 적분항 2종
    #      goal_phase,                              ← [2026-08-12] 종점 접근 단계
    #      cb_state, cb_v0_ms, cb_v_corner_ms,      ← [2026-08-18] 코너 1단 선행제동 3종
    #      goal_need_m,                             ← [2026-08-19] 종점 접근 필요 제동거리
    #      lidar_zone, rejoin]                      ← [2026-09-01] 라이다 구간 이양 2종
    #   ★lidar_zone=1 인 구간의 out_pulse 를 믿지 말 것★ 그 구간에서 driving 은
    #   /cmd_vel_raw 를 아예 내지 않으므로 그 열은 ★직전에 낸 값에 굳어 있다★.
    #   실제로 나간 지령은 /cmd_vel_raw 열(아래에서 따로 받는다)에서 본다 —
    #   그것을 낸 것은 mppi_local_planner 다.
    TopicSpec('/drive_diag', Float64MultiArray,
              ('cte_m', 'heading_err_deg', 'target_idx', 'target_dist_m',
               'goal_dist_m', 'gps_course_deg', 'fuse_corr_deg', 'gyro_z_dps',
               'brake_latched', 'head_init_deg', 'head_sigma_deg',
               'head_resid_m', 'head_dist_m',
               'ref_pulse', 'out_pulse', 'meas_pulse',
               'cte_integral', 'cte_i_term_deg', 'goal_phase',
               'cb_state', 'cb_v0_ms', 'cb_v_corner_ms', 'goal_need_m',
               'lidar_zone', 'rejoin'),
              _array(23),
              note='★cte_m 이 핵심★ 경로이탈 +왼쪽/−오른쪽. 나머지는 헤딩 융합 '
                   '건전성과 출발 헤딩 품질. ref/out/meas 는 저속 펄스 보정 검증용 — '
                   'out≠ref 인 구간이 보정이 걸린 구간이다. cte_i_term_deg 는 '
                   'CTE 적분이 조향에 더한 도로휠각(pot 기준 ×1.75). goal_phase 는 '
                   '종점 접근 ★단계 0없음/1 1단제동/2크립/3 2단백스톱★ [2026-08-19 개편] — '
                   'brake_latched(=DRIVE_DONE 의 2단)만으로는 도착 정지와 접근제동이 '
                   '구별되지 않는다. ★goal_need_m 은 그때 필요했던 제동거리★ — '
                   'goal_dist_m 이 이 값을 아래로 가르는 행이 체결 지점이고, '
                   'goal_phase=1 구간의 gps_kmh 기울기가 ★1단 실측 감속도★ 다'
                   '(물린 뒤 0.6초 이후에서 재라). 그 값으로 런치 goal_brake1_ms2'
                   '(현재 0.47)를 갱신한다. ★goal_phase 3 이 보이면 1단이 안 듣는 것★. '
                   '★cb_state(0없음/1제동/2잠금) 는 코너 1단 선행제동★ — '
                   'cb_state=1 구간의 gps_kmh 기울기가 ★a1 실측(구동차단)★ 이고 '
                   '그 값으로 driving.py 의 A_BRAKE1_MS2(현재 0.88 = 구동이 살아 '
                   '있던 하한)를 갱신한다. 1→2 로 바뀐 행의 gps_kmh 가 '
                   'cb_v_corner_ms 보다 크게 낮으면 해제가 늦은 것 → '
                   'CORNER_BRAKE_RELEASE_LEAD_MS 를 키운다'),

    TopicSpec('/encoder', Int32, ('encoder_sum',), _scalar,
              note='A보드 좌+우 펄스 ★합★ — 바퀴 하나 기준(=양 바퀴 평균)으로 보려면 '
                   '÷2 한다. cmd_pulse 는 바퀴 하나 기준이라 그대로 비교하면 2배 어긋난다'),
    #  ★[2026-09-09] 좌·우를 따로 남긴다★ 합만 있으면 '어느 바퀴가 덜 도는가' 가
    #  로그에서 영영 안 보인다. 인휠 2개가 ★각자 PID 를 닫으므로★ (A보드 좌 2번핀→
    #  8번PWM / 우 21번핀→9번PWM, 교차 없음) 좌우가 갈리는 것이 실제로 일어난다.
    #  ★두 열 모두 cmd_pulse 와 같은 눈금이다★ — ÷2 하지 말 것(합이 아니다).
    TopicSpec('/encoder_l', Int32, ('encoder_l',), _scalar,
              note='A보드 ★왼쪽★ 바퀴 펄스 원값. cmd_pulse 와 같은 눈금(합이 아니다)'),
    TopicSpec('/encoder_r', Int32, ('encoder_r',), _scalar,
              note='A보드 ★오른쪽★ 바퀴 펄스 원값. 좌우 차이가 곧 구동 불균형이다'),
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
    TopicSpec('/estop', Bool, ('estop',), _scalar,
              note='★E-STOP(D12 NC 하드웨어)뿐이다★ A·B 보드가 자체 판정한 것을 '
                   '보고받은 값. 경로이탈·도착·STOP 정지는 여기에 안 잡힌다 — '
                   '그건 drive_state(DRIVE_DONE)·drive_event 로 구별한다'),
    TopicSpec('/board_status', String, ('board_status',), _scalar,
              note='A:1,B:1,ESTOP:0,MODE:1 — B보드 링크가 끊기면 D5 가 멈춰 '
                   '상태기계가 굳는다. 그 구간을 로그에서 구별하는 유일한 단서'),
    TopicSpec('/drive_state', String, ('drive_state',), _scalar,
              note='driving 상태기계 ★기록 구간을 정하는 신호★'),
    #  ★[2026-09-07] /lstatus — 구간 문자 '0'|'L'|'S' 가 곧 조종권이다★
    #  이 열 하나로 "지금 누가 몰고 있었나" 가 로그에서 바로 드러난다. 종전에는
    #  /lidar_permit(Bool) 을 봐야 했는데 그 토픽은 없어졌다.
    # ══════════════════════════════════════════════════════════════════════════
    #  ★라이다 정지 사슬 [2026-09-10]★ — 안 남겨서 한 번 진단이 막혔다
    # ══════════════════════════════════════════════════════════════════════════
    #  2026-09-09 밤 braketest 에서 라바콘을 지나쳤는데 제동이 없었다. 그런데 이
    #  세 토픽이 하나도 기록되지 않아, ★사슬의 어디가 끊겼는지를 로그로 가릴 수
    #  없었다★ (원인은 라이다가 뜨기 전에 출발한 것이었다). 셋의 뜻이 다르므로
    #  셋 다 남긴다 — 이 세 열만 보면 다음부터는 한눈에 갈린다:
    #    · cone_stop 이 비어 있다        → ouster/cone_lidar 가 안 돌았다
    #    · cone_stop 은 오는데 계속 False → 인지는 하는데 못 잡았다(ROI·코리도)
    #    · cone_stop True 인데 aeb False  → pedal_drive_node 의 확정 단계에서 막혔다
    #    · aeb True 인데 brake_pot 그대로 → arduino 가 무시했다(aeb_brake_level=0?)
    TopicSpec('/cone_lidar_node/stop_signal', Bool, ('cone_stop',), _scalar,
              note='cone_lidar_node 의 프레임별 판정. ★열이 통째로 비어 있으면 '
                   '라이다가 한 프레임도 안 돌았다는 뜻이다★ (이 노드는 점군 '
                   '프레임마다 조건 없이 낸다)'),
    TopicSpec('/cone_lidar_node/obstacle_distance', Float32,
              ('cone_dist_m',), _scalar,
              note='코리도 안 최근접 전방거리 [m]. 비었으면 inf 로 나온다 — '
                   '라이다 원점 기준이므로 범퍼까지는 vehicle_front_m(1.2) 를 뺀다'),
    TopicSpec('/aeb_stop', Bool, ('aeb_stop',), _scalar,
              note='pedal_drive_node → arduino. ★이것이 실제로 리니어를 무는 신호★ '
                   '이다. ⚠️ 이 노드는 자기 타이머로 계속 내므로 '
                   '★열이 차 있다는 것은 판단자 생존일 뿐 라이다 생존이 아니다★ — '
                   '라이다 생존은 cone_stop 열로 본다'),
    # ══════════════════════════════════════════════════════════════════════════
    #  ★라이다 회피 진단 [2026-09-11]★ — ★L 구간에서만 채워진다★ (사용자 지시)
    # ══════════════════════════════════════════════════════════════════════════
    #  mppi 는 ★실제로 몰고 있을 때만★ /lidar_diag 를 낸다. 그래서 이 아홉 열은
    #  L 구간에서만 값이 있고 그 밖에서는 빈칸이다 — '어디서부터 라이다가 몰았나'
    #  가 열 모양으로 바로 드러나고, lstatus 열과 교차검증도 된다.
    #  ★hold=False★ 로 둔다 — 값을 유지하면 L 구간이 끝난 뒤에도 마지막 값이
    #  계속 찍혀 '아직 라이다가 몰고 있다' 로 잘못 읽힌다.
    #
    #  ★읽는 법 — 라바콘 사이를 잘 지났는가★
    #    ld_obs_y 의 부호 = 그 라바콘이 중심선 ★어느 쪽★ 에 있었나 (+ 왼쪽)
    #    ld_y      = 그때 차가 중심선에서 ★어디에 있었나★ (+ 왼쪽)
    #    → 둘의 부호가 ★반대★ 여야 비켜 간 것이다. 같으면 라바콘 쪽으로 갔다.
    #    ld_obs_x 가 줄다가 사라지면 그 라바콘을 지나친 것이고, 그 직전의
    #    |ld_y − ld_obs_y| 가 ★실제 통과 여유★ 다(차 반폭 0.55m 와 비교).
    #    ld_gps_ref=0 인 구간은 기준선이 추측항법이라 ld_y 를 믿을 수 없다.
    #  ★[2026-09-12] 11 → 14★ 뒤 3개는 ★검출 진단★ 이다 — '콘이 없었다' 와
    #  '있는데 못 잡았다' 를 가른다(2026-09-12 인계 분석에서 갈리지 않던 지점).
    #    ld_lethal_cells   치사 셀 총수 — 0 이면 ★점군 자체가 없다★
    #    ld_clusters_all   찾은 군집 수 (cone_min_cells 적용 ★전★)
    #    ld_clusters_rej   문턱 미달로 버린 군집 수
    #  → all>0 인데 n_cones=0 이고 rej>0 이면 ★문턱에서 떨어진 것★ 이고,
    #    lethal=0 이면 애초에 아무것도 안 보인 것이다.
    #  ⚠️ mppi_local_planner_node.cpp 의 diag m.data(14개)와 ★짝★ 이다.
    TopicSpec('/lidar_diag', Float64MultiArray,
              ('ld_y', 'ld_yaw_deg', 'ld_road_deg', 'ld_pot_deg', 'ld_pulse',
               'ld_avg_cost', 'ld_gps_ref', 'ld_obs_x', 'ld_obs_y',
               'ld_target_y', 'ld_n_cones',
               'ld_lethal_cells', 'ld_clusters_all', 'ld_clusters_rej'),
              _array(14), hold=False,
              note='mppi 가 ★모는 동안에만★ 낸다 = L 구간 전용. y·yaw 는 '
                   '★기준선(매핑 중심선) 대비★ 이고 + 가 왼쪽. pot 만 보드 규약'
                   '(− 좌 / + 우)이다. obs_x/y 는 ★군집으로 분리한 콘의 중심★ '
                   '(팽창 가장자리가 아니다). ld_target_y 는 지금 겨누는 횡목표, '
                   'ld_n_cones 는 앞에 보이는 콘 개수 — 0 이면 복귀 구간이다. '
                   '★ld_y 가 ld_target_y 를 따라가는지★ 가 회피가 되는지의 판정'),
    TopicSpec('/lstatus', String, ('lstatus',), _scalar,
              note="구간 문자 0=GPS추종 / L=라이다(mppi) / S=일시정지. "
                   "★driving → mppi 조종권★ (terrain 열을 정규화한 값)"),
    TopicSpec('/drive_cmd', String, ('drive_cmd',), _scalar, hold=False,
              note='prompt 하달 (경로 선택 / STOP)'),
    TopicSpec('/drive_event', String, ('drive_event',), _scalar, hold=False,
              note='헤딩 확정·도착·브레이크 등 이벤트'),

    # ── 신호등 인지 (white1/traffic_light.py) [2026-08-14] ──
    #   ★'왜 여기서 섰나 / 왜 멀리서도 섰나' 를 사후에 판정하기 위한 세 열★
    #   brake_level 만으로는 '리니어가 물렸다'는 결과만 남고 그 근거가 남지 않는다.
    #   세 열을 나란히 놓으면 판정 경로가 그대로 드러난다:
    #       tl_state=RED     + tl_near_metric 60  → 가까워서 섰다(정상)
    #       tl_state=RED     + tl_near_metric 26  → ★임계(25px)를 겨우 넘어 멀리서 섰다★
    #       tl_state=RED_FAR + tl_red_far=True    → 빨갛지만 멀다고 보고 안 섰다
    #   ★[2026-09-10] 허락 두 열 — terrain 'T' 구간이 실제로 들었는지를 남긴다★
    #   이 둘이 없어서 '신호등이 왜 개입 안 했나' 를 로그로 가릴 수 없었다.
    #   tl_permit 은 driving 이 내는 허락(TRAFFIC_LIGHT_ENABLE + DRIVE_RUN + ★T 구간★)
    #   이고, tl_req 는 신호등이 그 허락 아래 요구한 단계다. 함께 읽으면 갈린다:
    #       tl_permit=False              → T 구간 밖이다. 신호등은 손을 안 댄다(정상)
    #       tl_permit=True, tl_req=0     → T 구간인데 빨간불이 아니다(통과)
    #       tl_permit=True, tl_req=2     → 빨간불 확정 — brake_level 도 2 여야 한다
    #       tl_req=2 인데 brake_level<2  → ★max() 합성이 안 됐다★ (있을 수 없다)
    TopicSpec('/tl_permit', Bool, ('tl_permit',), _scalar,
              note="driving → traffic_light 개입 허락. ★신선도가 곧 허락★ 이고, "
                   "[2026-09-10] 부터 terrain 열 'T' 구간에서만 True 다"),
    TopicSpec('/tl_brake_req', Int32, ('tl_req',), _scalar,
              note='신호등이 요구한 브레이크 단계. driving 이 자기 요청과 max() 로 '
                   '합쳐 /brake_level 을 낸다 — 두 발행자가 다투지 않게'),
    TopicSpec('/tl/state', String, ('tl_state',), _scalar,
              note='RED / RED_FAR / GREEN / UNKNOWN — 프레임 판정'),
    TopicSpec('/tl/near_metric', Float32, ('tl_near_metric',), _scalar,
              note='근접도 게이트에 걸리는 값(빨간 박스 중 최대). 기본 단위는 '
                   '★박스 높이[px]★ 이고 tl_red_stop_min_area_frac>0 이면 면적비다. '
                   '임계(tl_red_stop_min_height, 기본 25)와 비교해서 읽는다'),
    TopicSpec('/tl/red_far', Bool, ('tl_red_far',), _scalar,
              note='이번 프레임이 RED_FAR 인가 = 빨갛지만 아직 멀다고 본 것'),
    #   ★정지선 네 열 [2026-08-14 → 2026-08-19 sl_px 추가 → 2026-09-08 sl_bev_y 추가]★
    #   위 세 열이 '왜 섰나'라면 이 넷은 ★'어디서 섰나'★ 다.
    #       tl_state=RED + sl_wait=True   → 빨간불은 확정, 정지선을 기다리는 중(안 섰다)
    #       brake_level 이 0→2 로 넘어간 행의 sl_bev_y → ★풀브레이크를 건 지점★
    #         (발화선 sl_trigger_bev_y 와 비교해서 읽는다) 또는 그 행의 tl_near_metric
    #         이 tl_solo_stop_min_height 를 넘겨서 — ★둘 중 먼저 성립한 쪽이 근거다★
    #         (traffic_light.py 헤더: "근거 둘, 단계 하나").
    #       sl_bev_y=−9999(SL_NONE) 인 채 brake_level 2 → 정지선을 못 보고 신호등
    #         단독 문턱만으로 선 것
    TopicSpec('/tl/stop_line_bev_y', Float32, ('sl_bev_y',), _scalar,
              note='★[2026-09-08] 판정값★ BEV 에서 정지선 최근접점의 행. 클수록 '
                   '가깝다(멀면 작고, 사다리꼴 윗변보다 더 멀면 음수). '
                   '−9999 = 미검출(SL_NONE). 발화선(sl_trigger_bev_y, 기본 40)에 '
                   '닿으면 풀브레이크 — sl_trigger_bev_y 와 비교해서 읽는다'),
    TopicSpec('/tl/stop_line_px', Float32, ('sl_px',), _scalar,
              note='★[2026-09-08] 이제 기록 전용★ 판정은 위 sl_bev_y 가 한다. '
                   'BEV 에서 정지선→앞범퍼 픽셀 거리. −1 = 미검출 / 0 = 범퍼선 도달'
                   '(또는 지나침). 값이 작을수록 가깝다 — 종전 1·2단 문턱'
                   '(sl_brake1_px·sl_brake2_px)은 폐지됐고 이 값은 참고로만 남는다'),
    TopicSpec('/tl/stop_line_y', Float32, ('sl_y',), _scalar,
              note='정지선 마스크 최하단 y ÷ 프레임 높이(0~1). −1 = 미검출. '
                   '★판정에는 안 쓴다★ [2026-08-19] — 원근이 남아 거리에 비례하지 '
                   '않는다. 영상과 대조할 때 쓰는 참고값이다'),
    TopicSpec('/tl/stop_line_wait', Bool, ('sl_wait',), _scalar,
              note='RED 확정인데 정지선이 아직 멀어 아무 단계도 안 건 구간'),

    # ── 원시 센서 ──
    TopicSpec('/fix', NavSatFix,
              ('fix_lat', 'fix_lon', 'fix_alt_m', 'fix_status', 'fix_cov_xx'),
              _navsat, note='GPS 원시'),
    # ── GPS 후처리 [2026-08-18 gps.py 신설] ★배열 규약의 소유자는 gps.py 헤더★ ──
    #   ★/fix 와 나란히 기록해야 뜻이 있다★ 이 두 줄을 겹쳐 보면 gps.py 가 한 일이
    #   그대로 드러난다: fix_status 가 2 인 구간이 gps_quality 3(Float)과 4(Fixed)로
    #   갈리는 것이 ①품질 판정이고, gps_is_raw=0 인 행에서 lat/lon 이 /fix 보다
    #   앞서 나가 있는 것이 ②IMU 공백 메움이다.
    #   분석 착안점:
    #     · gps_sigma_m 히스토그램 → 두 무리로 갈린다. 그 골짜기가 Fixed 문턱의 실측근거
    #       (gps.py 의 RTK_FIXED_SIGMA_M 0.30 이 맞는 값인지 여기서 확인한다)
    #     · gps_dr_dist_m 의 최대값 → DR 이 실제로 얼마나 메웠나. 0.2초 공백이면
    #       속도×0.2 근처여야 한다. 그보다 크면 fix 를 놓치고 있다는 뜻이다
    #     · is_raw=1 행에서 직전 DR 예측좌표와의 거리 → ★DR 오차 실측★.
    #       DR_SIGMA_GROWTH_M_PER_S(현재 0.5, 미검증 추정)를 이 값으로 대체할 수 있다
    TopicSpec('/gps_fused', Float64MultiArray,
              ('gps_lat', 'gps_lon', 'gps_quality', 'gps_sigma_m', 'gps_pos_ok',
               'gps_is_raw', 'gps_raw_age_s', 'gps_dr_dist_m', 'gps_kmh',
               # ★'gps_course_deg' 가 아니다★ 그 이름은 위 /drive_diag 가 이미 쓴다
               #   (driving 의 _diag_course = 융합에 실제로 먹인 코스). 같은 이름을 두 번
               #   쓰면 CSV 헤더가 중복되고 ★csv.DictReader 가 뒤엣것만 남겨 앞 열이
               #   조용히 사라진다★ — 실제로 그렇게 냈다가 잡았다. 둘은 다른 값이다:
               #     gps_course_deg     : driving 이 자기 융합에 쓴 코스(0.30m 고정 문턱)
               #     gps_fix_course_deg : gps.py 가 원시 fix 로 낸 코스(σ 비례 문턱)
               #   ★둘을 나란히 보는 것이 진단에 쓸모 있다★ — 갈리면 문턱 차이가 원인이다.
               'gps_fix_course_deg',
               # ── [2026-08-18 (3)] 이상치 게이트 + DEGRADED 융합 3종 ──
               'gps_mode', 'gps_reject_n', 'gps_resid_m'),
              _array(13),
              note='gps.py 후처리 좌표. quality 0없음/1SPS/2DGPS/3RTK_FLOAT/4RTK_FIXED '
                   '— ★status.status 로는 Float 과 Fixed 가 구별되지 않아서 σ 로 '
                   '갈라낸 것이다★. is_raw=0 은 IMU 로 메운 표본이고 그때 '
                   'dr_dist_m 만큼 원시 fix 앞으로 외삽돼 있다. raw_age_s 는 '
                   'driving 의 GPS 두절 판정 기준(DR 로 메운 시간이 판정을 늦추지 '
                   '않도록 원시 fix 의 나이를 그대로 싣는다). '
                   '★gps_reject_n 이 늘어나는 구간이 GPS 가 튀는 구간이다★ — '
                   'fix_lat/fix_lon(원값)과 겹쳐 보면 무엇을 버렸는지 보인다. '
                   'gps_mode 0=NORMAL(fix 스냅) 1=DEGRADED(DR + 잔차 중앙값 융합). '
                   'gps_resid_m 은 DEGRADED 에서 GPS 와 DR 의 불일치 크기 — '
                   '★DEGRADED_RESID_MAX_M(3.0) 근처로 굳으면 DR 이 GPS 를 못 따라가는 '
                   '것이므로 엔코더 환산(ENC_MS_PER_PULSE)이나 자이로를 의심할 것★'),
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
#  ★열 이름 중복 검사 — import 시점에 크게 터뜨린다 [2026-08-18]★
# ══════════════════════════════════════════════════════════════════════════════
#  ★왜 필요한가 — 중복은 조용히 데이터를 지운다★
#  CSV 헤더에 같은 이름이 두 번 들어가도 파일은 정상으로 보이고 행 수도 맞는다.
#  그런데 ★csv.DictReader 는 뒤엣것만 남긴다★ — 앞 열의 값이 분석 단계에서 통째로
#  사라진다. 파일을 열어 봐도 안 보이고, 열 번호로 읽는 도구와 이름으로 읽는 도구가
#  서로 다른 값을 보게 된다. 사후분석이 목적인 이 파일에서 제일 나쁜 실패다.
#  실제로 [2026-08-18] /gps_fused 에 'gps_course_deg' 를 넣어 /drive_diag 의 같은
#  이름과 겹쳤고, 79열이 DictReader 에서 78키로 줄어드는 것으로 발견했다.
#  → 이름을 새로 넣을 때 사람이 기억해서 피하는 것이 아니라 ★기동 자체를 막는다★.
#    노드가 안 뜨면 즉시 알 수 있고, 잘못된 CSV 가 한 줄도 생기지 않는다.
_dups = sorted({c for c in ALL_COLUMNS if ALL_COLUMNS.count(c) > 1})
if _dups:
    raise RuntimeError(
        f"record.py 열 이름 중복: {_dups} — CSV 헤더가 겹치면 csv.DictReader 가 "
        f"뒤엣것만 남겨 앞 열이 조용히 사라진다. RECORD_TOPICS 에서 이름을 바꿀 것 "
        f"(전체 {len(ALL_COLUMNS)}열 중 고유 {len(set(ALL_COLUMNS))}개)")
del _dups


# ══════════════════════════════════════════════════════════════════════════════
#  노드
# ══════════════════════════════════════════════════════════════════════════════
class RecordNode(Node):

    STATUS_PERIOD_S = 10.0
    FLUSH_EVERY_ROWS = 20

    def __init__(self, **kwargs):
        super().__init__('record_node', **kwargs)

        self.declare_parameter('output_dir', '')
        self.declare_parameter('sample_hz', 20.0)
        # ── ★[2026-08-14] force_record : driving 없이 혼자 기록한다★ ──────────────
        #   평소 기록 구간은 /drive_state 가 정한다(RECORD_STATES). 그런데 조이스틱
        #   수동조종만으로 계측할 때는 driving 노드가 아예 없거나 IDLE 이라 그 신호가
        #   영영 오지 않아 ★파일이 열리지 않는다★. 그 경우를 위한 스위치다:
        #       ros2 run white1 record --ros-args -p force_record:=true
        #   켜면 노드가 뜨는 즉시 기록을 시작하고 내려갈 때까지 계속 적는다. 경로
        #   이름을 들은 적이 없으므로 파일명 앞부분은 manual 이 된다.
        #   ★런치(one_launch.py)는 이 값을 주지 않는다★ = 자율주행 기록 규칙은 그대로다.
        self.declare_parameter('force_record', False)
        # ── ★[2026-09-12] 라이다 궤적 기록 (.cones.csv)★ ──────────────────────────
        #  ★무엇을 적나★ 메인 CSV 는 '한 행 = 한 시점' 의 고정 열이라 ★개수가
        #  변하는 콘 목록★ 을 담을 수 없다. 그래서 같은 이름에 확장자만 다른
        #  짝 파일을 연다 :
        #      ros2bag/<경로이름>-<시각>.csv          메인 (87열, 한 행 = 한 시점)
        #      ros2bag/<경로이름>-<시각>.cones.csv    라이다 (한 행 = 한 콘/한 점)
        #  t_rel·t_wall 이 양쪽에 있어 그 열로 조인한다.
        #
        #  ★언제 적나 — 두 모드★
        #    (a) one_launch 와 함께 (기본) : ★lstatus 가 'L' 인 동안만★.
        #        라이다가 실제로 모는 구간만 남으므로 파일이 작고, 메인 CSV 의
        #        lstatus 열과 시각이 그대로 맞는다.
        #    (b) 단독 실행 `ros2 run white1 record --lidar` : ★상시★.
        #        이상적인 회피를 골라 담으려면 구간 판정을 기다릴 수 없다.
        #
        #  ★중복 방지★ (b)가 뜨면 /record_lidar_owner 를 20Hz 로 낸다. (a)는 그것이
        #  신선한 동안 ★자기 .cones.csv 를 닫고 손을 뗀다★ — 메인 CSV 는 그대로
        #  적는다(그쪽은 (b)가 건드리지 않는다). 신선도가 곧 소유권이라,
        #  (b)를 Ctrl-C 로 내리면 (a)가 1초 뒤 자동으로 되받는다.
        self.declare_parameter('lidar_record', False)   # --lidar 가 True 로 바꾼다
        self.declare_parameter('lidar_owner_topic', '/record_lidar_owner')
        self.declare_parameter('lidar_owner_stale_s', 1.0)
        #  계획 경로는 매 틱 수십 점이라 그대로 적으면 메인보다 커진다. 솎아낸다.
        self.declare_parameter('lidar_path_every_n', 20)   # 20틱(1초)마다 한 번만
        self.declare_parameter('lidar_path_stride', 3)     # 그 경로의 매 3번째 점만

        self.out_root = paths.record_dir(self.get_parameter('output_dir').value or '')
        # 20Hz = driving 제어주기. 더 빠른 토픽(imu)은 주기 안에서 마지막 값만 남는다.
        self.sample_hz = max(1.0, float(self.get_parameter('sample_hz').value))
        self.force_record = bool(self.get_parameter('force_record').value)
        self.lidar_record = bool(self.get_parameter('lidar_record').value)
        self.lidar_owner_stale = float(self.get_parameter('lidar_owner_stale_s').value)
        self.lidar_path_every_n = max(1, int(self.get_parameter('lidar_path_every_n').value))
        self.lidar_path_stride = max(1, int(self.get_parameter('lidar_path_stride').value))
        # 라이다 기록 상태
        self._cones_fp = None
        self._cones_writer = None
        self._cones_path = ''
        self._cones_rows = 0
        self._cones_t0 = 0.0
        self._ld_last: Dict[str, Any] = {}   # /lidar_diag 최신값 (cones.csv 전용 사본)
        self._ld_last_t = 0.0
        self._cones: List[float] = []        # 마지막 /lidar_cones (3개씩)
        self._cones_t = 0.0
        self._lpath: List[tuple] = []        # 마지막 계획 경로 [(x,y), ...]
        self._lpath_t = 0.0
        self._lpath_tick = 0
        self._owner_t = 0.0                  # 남이 소유권을 주장한 마지막 시각

        self.drive_state = 'IDLE'
        self.route_name = ''       # 주행 중인 경로 CSV 이름 — 기록 파일명 앞부분

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

        # ── ★라이다 궤적 (.cones.csv)★ — 고정 열 구조와 별개로 직접 구독한다
        self.create_subscription(
            Float64MultiArray, '/lidar_cones',
            lambda m: self._on_cones(m), 10)
        self.create_subscription(
            NavPath, '/mppi_local_planner/local_path',
            lambda m: self._on_lpath(m), 1)
        owner_topic = self.get_parameter('lidar_owner_topic').value
        if self.lidar_record:
            #  ★내가 맡는다★ — 런치 쪽 record 가 이걸 보고 손을 뗀다
            self._owner_pub = self.create_publisher(Bool, owner_topic, 10)
            self.create_timer(0.05, self._owner_tick)     # 20Hz
        else:
            self._owner_pub = None
            self.create_subscription(
                Bool, owner_topic,
                lambda m: setattr(self, '_owner_t', time.time()), 10)

        self.create_timer(1.0 / self.sample_hz, self._tick)
        self.create_timer(1.0, self._status_tick)

        self.get_logger().info(
            f"📼 record 대기 — 기록 위치: {self.out_root}\n"
            f"   토픽 {len(RECORD_TOPICS)}개 → 통합 CSV {len(ALL_COLUMNS)}열, "
            f"{self.sample_hz:.0f}Hz 스냅샷 | 기록 구간: "
            + ('★force_record — 지금부터 계속★' if self.force_record
               else ', '.join(RECORD_STATES)))

        if self.force_record:
            # 구독이 아직 아무것도 못 받은 시점이라 빈 칸이 많은 행부터 시작한다 —
            # 값이 들어오는 순간부터 채워지므로 앞 몇 줄만 비어 있고 문제되지 않는다.
            self.route_name = MANUAL_ROUTE
            self._start_session()

    # ── 수신 ───────────────────────────────────────────────────────────────────
    def _on_msg(self, spec: TopicSpec, msg):
        was = self.recording

        if spec.topic == '/lidar_diag':
            #  ★.cones.csv 는 이 값을 매 행에 적는다★ hold=False 라 _pending 에
            #  들어갔다가 메인 행이 pop 해 가므로, 여기서 따로 붙들어 둔다.
            #  (메인 CSV 의 'L 구간에만 값이 있다' 규칙은 그대로다 — 이건 별도 사본)
            self._ld_last = {
                k: v for k, v in zip(spec.columns, list(msg.data) + [None] * 16)}
            self._ld_last_t = time.time()
        if spec.topic == '/drive_state':
            self.drive_state = str(msg.data)
        elif spec.topic == '/drive_cmd':
            self._note_route_cmd(str(msg.data))
        elif spec.topic == '/drive_event':
            self._note_route_event(str(msg.data))

        self._stash(spec, msg)

        # force_record 는 상태를 보지 않는다 — 시작도 끝도 노드 수명이 정한다
        now_on = self.force_record or self.drive_state in RECORD_STATES
        if now_on and not was:
            self._start_session()
        elif was and not now_on:
            self._write_row()          # 종료 사유가 담긴 마지막 한 줄
            self._stop_session()

    # ── 경로 이름 ──────────────────────────────────────────────────────────────
    def _note_route_cmd(self, text: str):
        """prompt 가 /drive_cmd 로 보낸 것이 경로 파일명이면 붙든다."""
        name = text.strip()
        if not name or name.upper() in ROUTE_CMD_WORDS:
            return
        if name.lower().endswith('.csv'):
            self.route_name = os.path.basename(name)

    def _note_route_event(self, text: str):
        """driving 의 '경로 선택 / 주행 시작' 이벤트에서 이름을 줍는다(보조)."""
        if not any(h in text for h in ROUTE_EVENT_HINTS):
            return
        m = ROUTE_IN_TEXT.search(text)
        if m:
            self.route_name = m.group(1)

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
        # ★파일명 = (주행한 경로 CSV 이름)-(기록 시작 시각).csv★
        base = os.path.splitext(os.path.basename(self.route_name))[0] or UNKNOWN_ROUTE
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(self.out_root, f"{base}-{stamp}.csv")
        n = 2
        while os.path.exists(self.csv_path):   # 같은 초에 두 번 시작한 경우만
            self.csv_path = os.path.join(self.out_root, f"{base}-{stamp}_{n}.csv")
            n += 1
        # utf-8-sig : 엑셀이 한글 헤더를 깨뜨리지 않게 BOM
        self._fp = open(self.csv_path, 'w', newline='', encoding='utf-8-sig')
        self._writer = csv.writer(self._fp)
        self._writer.writerow(ALL_COLUMNS)
        self.session_t0 = time.time()
        self._rows = 0
        self._rx = {s.topic: 0 for s in RECORD_TOPICS}
        self.recording = True
        if not self.route_name:
            self.get_logger().warning(
                "경로 이름을 못 들었다(record 를 주행 도중에 띄웠는가?) — "
                f"파일명 앞부분이 {UNKNOWN_ROUTE} 가 된다")
        self.get_logger().info(f"🔴 기록 시작 → {self.csv_path}")

    def _tick(self):
        if self.recording:
            self._write_row()
        #  ★라이다 궤적은 메인 세션과 ★독립★ 으로 열고 닫는다★
        #  런치 모드에서는 L 구간에만, --lidar 단독 모드에서는 상시.
        want = self._lidar_wanted()
        if want and self._cones_writer is None:
            self._cones_open()
        elif not want and self._cones_writer is not None:
            self._cones_close()
        if self._cones_writer is not None:
            now = time.time()
            #  ★t_rel 기준을 메인 CSV 와 맞춘다★ — 그래야 두 파일이 그 열로 조인된다
            t0 = self.session_t0 if self.recording else self._cones_t0
            self._cones_write(now - t0, now)
            if self._cones_rows % self.FLUSH_EVERY_ROWS == 0:
                try:
                    self._cones_fp.flush()
                except Exception:
                    pass

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

    # ══════════════════════════════════════════════════════════════════════════
    #  라이다 궤적 (.cones.csv)
    # ══════════════════════════════════════════════════════════════════════════
    CONES_COLUMNS = (
        't_rel', 't_wall', 'kind', 'i', 'x_m', 'y_m', 'cells',
        'ego_y', 'ego_yaw_deg', 'target_y', 'n_ahead',
        'lethal_cells', 'clusters_all', 'clusters_rej', 'lstatus')

    def _owner_tick(self):
        """--lidar 모드가 20Hz 로 소유권을 주장한다 (값보다 ★신선도★)."""
        if self._owner_pub is not None:
            self._owner_pub.publish(Bool(data=True))

    def _on_cones(self, msg):
        self._cones = list(msg.data)
        self._cones_t = time.time()

    def _on_lpath(self, msg):
        #  계획 경로는 매 틱 수십 점이다 — every_n 틱에 한 번, stride 점만 남긴다
        self._lpath_tick += 1
        if self._lpath_tick % self.lidar_path_every_n:
            return
        self._lpath = [(p.pose.position.x, p.pose.position.y)
                       for p in msg.poses[::self.lidar_path_stride]]
        self._lpath_t = time.time()

    def _lidar_wanted(self) -> bool:
        """지금 라이다 궤적을 적어야 하는가."""
        if self.lidar_record:
            return True                      # 단독 모드 = 상시
        #  남이 맡고 있으면 손을 뗀다 (신선도가 곧 소유권)
        if time.time() - self._owner_t < self.lidar_owner_stale:
            return False
        #  런치 모드 = lstatus 가 'L' 인 동안만
        return str(self._hold.get('lstatus', '')).strip().upper() == 'L'

    def _cones_open(self):
        if self._cones_writer is not None:
            return
        #  ★메인 CSV 와 ★파일명이 같고 확장자만 다르다★
        if self.csv_path:
            base = self.csv_path[:-4] if self.csv_path.endswith('.csv') else self.csv_path
        else:
            base = os.path.join(
                self.out_root,
                f"{MANUAL_ROUTE}-{datetime.now().strftime('%Y%m%d_%H%M%S')}")
        self._cones_path = base + '.cones.csv'
        os.makedirs(self.out_root, exist_ok=True)
        self._cones_fp = open(self._cones_path, 'w', newline='', encoding='utf-8-sig')
        self._cones_writer = csv.writer(self._cones_fp)
        self._cones_writer.writerow(self.CONES_COLUMNS)
        self._cones_rows = 0
        self._cones_t0 = self.session_t0 if self.recording else time.time()
        self.get_logger().info(
            f"🛞 라이다 궤적 기록 시작 ({'상시(--lidar)' if self.lidar_record else 'L 구간'})"
            f" → {self._cones_path}")

    def _cones_close(self):
        if self._cones_writer is None:
            return
        try:
            self._cones_fp.close()
        except Exception:
            pass
        self.get_logger().info(
            f"🛞 라이다 궤적 기록 종료 — {self._cones_rows}행 → {self._cones_path}")
        self._cones_fp = self._cones_writer = None
        self._cones_rows = 0

    def _cones_write(self, t_rel: float, t_wall: float):
        """한 스냅샷을 적는다. 콘이 없어도 한 행(kind=none)은 남긴다."""
        if self._cones_writer is None:
            return
        h = self._hold
        ld = self._ld_last if (time.time() - self._ld_last_t) < 1.0 else {}
        def g(k):
            v = ld.get(k, None)
            if v is None:
                v = h.get(k, '')
            return '' if v is None else v
        meta = [g('ld_y'), g('ld_yaw_deg'), g('ld_target_y'), g('ld_n_cones'),
                g('ld_lethal_cells'), g('ld_clusters_all'), g('ld_clusters_rej'),
                str(h.get('lstatus', '')).strip()]
        rows = []
        c = self._cones
        for i in range(0, len(c) - 2, 3):
            rows.append([f"{t_rel:.2f}", f"{t_wall:.3f}", 'cone', i // 3,
                         f"{c[i]:.3f}", f"{c[i+1]:.3f}", int(c[i+2])] + meta)
        if not rows:
            #  ★콘 0개도 한 행 남긴다★ — '안 보였다' 가 기록에 있어야
            #  lethal_cells·clusters_rej 와 함께 원인이 갈린다
            rows.append([f"{t_rel:.2f}", f"{t_wall:.3f}", 'none', -1, '', '', ''] + meta)
        #  계획 경로 — 갱신된 것이 있을 때만 (every_n 틱에 한 번)
        if self._lpath:
            for j, (px, py) in enumerate(self._lpath):
                rows.append([f"{t_rel:.2f}", f"{t_wall:.3f}", 'path', j,
                             f"{px:.3f}", f"{py:.3f}", ''] + meta)
            self._lpath = []          # 한 번만 적는다
        for r in rows:
            self._cones_writer.writerow(r)
        self._cones_rows += len(rows)

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
        #  ★런치 모드에서는 메인이 닫히면 라이다도 닫는다★ 다음 주행은 새 파일이다.
        #  --lidar 단독 모드는 노드 수명이 기록 구간이므로 여기서 닫지 않는다.
        if not self.lidar_record:
            self._cones_close()

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
        if self._cones_writer is not None:
            self.get_logger().info(f"🛞 라이다 궤적 {self._cones_rows}행")
        elif not self.lidar_record and time.time() - self._owner_t < self.lidar_owner_stale:
            self.get_logger().info(
                "🛞 라이다 궤적은 --lidar 단독 노드가 맡고 있다 — 손을 뗀 상태")

    def destroy_node(self):
        if self.recording:
            self._write_row()
            self._stop_session()
        self._cones_close()
        super().destroy_node()


def main(args=None):
    # ── ★[2026-09-12] 짧은 플래그를 ROS 파라미터로 옮긴다★ ────────────────────────
    #  `ros2 run white1 record --lidar` 처럼 쓰기 위해서다. rclpy.init 은 모르는
    #  인자를 만나면 실패하므로 ★먼저 꺼내고 argv 에서 지운다★.
    #      --lidar   라이다 궤적을 ★상시★ 기록 (.cones.csv). 런치 쪽 record 는
    #                /record_lidar_owner 를 보고 자기 라이다 기록을 접는다.
    #      --force   메인 CSV 도 상시 기록 (= -p force_record:=true)
    #  ※ --ros-args -p lidar_record:=true 로 줘도 같다. 이건 손맛용 별칭이다.
    import sys
    argv = sys.argv[1:] if args is None else list(args)
    flag_lidar = '--lidar' in argv
    flag_force = '--force' in argv
    argv = [a for a in argv if a not in ('--lidar', '--force')]
    if args is None:
        sys.argv = [sys.argv[0]] + argv
    rclpy.init(args=args)
    #  ★생성자 ★전에★ 정해야 한다★ — __init__ 이 이 값을 보고 구독·발행을
    #  구성하기 때문이다(소유권 발행 타이머는 lidar_record 일 때만 만든다).
    from rclpy.parameter import Parameter
    overrides = []
    if flag_lidar:
        overrides.append(Parameter('lidar_record', Parameter.Type.BOOL, True))
    if flag_force:
        overrides.append(Parameter('force_record', Parameter.Type.BOOL, True))
    node = RecordNode(parameter_overrides=overrides)
    try:
        rclpy.spin(node)
    # ★[2026-09-04] ExternalShutdownException 도 받는다★ launch 가 내려갈 때
    #   rclpy 의 신호 처리기가 컨텍스트를 먼저 닫으면 spin 은 KeyboardInterrupt 가
    #   아니라 이것을 던진다. 안 받으면 노드마다 트레이스백을 십수 줄 쏟아, 정작
    #   봐야 할 종료 로그를 밀어낸다(구독/발행 노드가 종료에 실패할 일은 없으므로
    #   그 트레이스백의 정보량은 0 이다). 원인 로그: gps 의 `rcl_shutdown already
    #   called` RCLError — 그것은 아래 rclpy.ok() 가드가 막는다.
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
