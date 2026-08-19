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
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Bool, Float32, Float64MultiArray, Int32, String

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
    #      cte_integral, cte_i_term_deg,            ← [2026-08-12] CTE 적분항 2종
    #      goal_phase,                              ← [2026-08-12] 종점 접근 단계
    #      cb_state, cb_v0_ms, cb_v_corner_ms,      ← [2026-08-18] 코너 1단 선행제동 3종
    #      goal_need_m]                             ← [2026-08-19] 종점 접근 필요 제동거리
    TopicSpec('/drive_diag', Float64MultiArray,
              ('cte_m', 'heading_err_deg', 'target_idx', 'target_dist_m',
               'goal_dist_m', 'gps_course_deg', 'fuse_corr_deg', 'gyro_z_dps',
               'brake_latched', 'head_init_deg', 'head_sigma_deg',
               'head_resid_m', 'head_dist_m',
               'ref_pulse', 'out_pulse', 'meas_pulse',
               'cte_integral', 'cte_i_term_deg', 'goal_phase',
               'cb_state', 'cb_v0_ms', 'cb_v_corner_ms', 'goal_need_m'),
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
    TopicSpec('/tl/state', String, ('tl_state',), _scalar,
              note='RED / RED_FAR / GREEN / UNKNOWN — 프레임 판정'),
    TopicSpec('/tl/near_metric', Float32, ('tl_near_metric',), _scalar,
              note='근접도 게이트에 걸리는 값(빨간 박스 중 최대). 기본 단위는 '
                   '★박스 높이[px]★ 이고 tl_red_stop_min_area_frac>0 이면 면적비다. '
                   '임계(tl_red_stop_min_height, 기본 25)와 비교해서 읽는다'),
    TopicSpec('/tl/red_far', Bool, ('tl_red_far',), _scalar,
              note='이번 프레임이 RED_FAR 인가 = 빨갛지만 아직 멀다고 본 것'),
    #   ★정지선 세 열 [2026-08-14 → 2026-08-19 sl_px 추가]★ 위 세 열이 '왜 섰나'라면
    #   이 셋은 ★'어디서 섰나'★ 다.
    #       tl_state=RED + sl_wait=True  → 빨간불은 확정, 정지선을 기다리는 중(안 섰다)
    #       brake_level 1 로 넘어간 행의 sl_px → ★1단 예비제동을 건 지점★
    #       brake_level 2 로 넘어간 행의 sl_px → ★2단을 건 지점★ = 실제 정지 지점
    #         두 문턱(sl_brake1_px·sl_brake2_px)을 정하는 근거가 이 값이다.
    #       sl_px=-1 인 채 brake_level 2 → 정지선을 못 보고 그 자리에서 선 것(종전 동작)
    TopicSpec('/tl/stop_line_px', Float32, ('sl_px',), _scalar,
              note='★판정값★ BEV 에서 정지선→앞범퍼 픽셀 거리. −1 = 미검출 / '
                   '0 = 범퍼선 도달(또는 지나침). 값이 작을수록 가깝다 — '
                   'sl_brake1_px(1단)·sl_brake2_px(2단)와 비교해서 읽는다'),
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

    def __init__(self):
        super().__init__('record_node')

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

        self.out_root = paths.record_dir(self.get_parameter('output_dir').value or '')
        # 20Hz = driving 제어주기. 더 빠른 토픽(imu)은 주기 안에서 마지막 값만 남는다.
        self.sample_hz = max(1.0, float(self.get_parameter('sample_hz').value))
        self.force_record = bool(self.get_parameter('force_record').value)

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
