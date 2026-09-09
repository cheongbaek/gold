#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
braketest.py ― ★브레이크 제동거리 측정 전용 노드★ [white1 / 2026-09-09]
════════════════════════════════════════════════════════════════════════════════
    ros2 launch white1 braketest.launch.py

  ★런치 = 출발★ D5 가 자율주행이고 GPS 가 서면 ★그 자리에서 지정속도로 굴러간다★.
  그리고 ★직선 매핑임을 그대로 이용한다★ (아래 '직선 전용 조준' 절):
      ① 출발 직후  : ★조향 무조건 0°★ — 곧게 굴러 GPS 코스로 헤딩을 세운다
      ② 헤딩 확정 후 : ★CSV 의 마지막 점★ 하나만 겨눈다
  헤딩 초기화를 위해 따로 서행하는 구간이 없고, 자이로도 쓰지 않는다.

  직선(또는 직선에 가까운) 매핑 경로를 GPS 로 추종하면서 ★고정 속도★ 로 달리다가,
  ★둘 중 먼저 오는 것★ 에서 리니어 2단을 물고 완전정지한다:

      ① 라이다 사슬이 세웠을 때 (/aeb_stop — arduino 가 직접 문다. 아래 절)
      ② 그런 일 없이 ★종점에 도달했을 때★ (이건 이 노드가 /brake_level 2단으로)

  완전정지하면 그 사이의 거리·시간·감속도를 재서 남기고, ★리니어를 풀고★
  런치를 스스로 내린다.

  ★[2026-09-09] 정지 트리거가 terrain 'S' 에서 라이다로 바뀌었다★
  `terrain` 열은 ★이제 보지 않는다★ — 'S' 가 있든 없든 무시한다.
  사람이 한 칸을 손으로 적어 두는 대신, 실제 장애물을 라이다가 보고 세운다.

════════════════════════════════════════════════════════════════════════════════
 라이다 정지 — ★lidar one_launch.py 의 사슬을 ★그대로★ 쓴다 (사용자 지시)★
════════════════════════════════════════════════════════════════════════════════
  ouster ─/ouster/points─▶ cone_lidar_node ─/…/stop_signal─▶ pedal_drive_node
                              (인지·판정)                      (확정·래치)
                                                                    │ /aeb_stop
                                                                    ▼
                                                          nxde/arduino  (구동차단
                                                                       + 리니어 2단)

  ★이 노드는 그 사슬에 끼어들지 않는다★ 라이다 정지의 판정·래치·제동이 전부
  위 세 노드 안에서 끝난다 — braketest 는 ★한 줄도 관여하지 않는다★.
  반응속도가 `lidar one_launch.py` 와 같아야 한다는 것이 그 이유다(사용자 지시).

  · `braketest.launch.py` 가 `lidar/launch/aeb.launch.py`(= ouster + cone_lidar_node)
    를 include 하고, `pedal_drive_node` 를 그쪽 런치와 ★같은 파라미터★ 로 띄우고,
    `arduino` 에 `aeb_brake_level` · `aeb_stale_s` · `aeb_topic` 을 넘긴다.
    ★그 셋이 비상정지를 '켜는' 유일한 지점이다★ (arduino 기본 aeb_brake_level=0 = 꺼짐).
  · 그래서 `/aeb_stop` 이 서면 arduino 가 ★모드와 무관하게★ A보드 구동을 끊고
    리니어를 물린다 — 이 노드가 `/cmd_vel_raw` 에 무엇을 내고 있든 우선한다
    (arduino compose 우선순위 (1-1)).

  ★이 노드가 `/aeb_stop` 을 구독하는 것은 '계측' 때문이다★ 제동을 걸기 위해서가
  아니다(그건 이미 arduino 가 했다). 언제 물렸는지를 알아야 제동거리·시간을 재고,
  런치를 끝낼 시점을 알 수 있다. ★그 값으로 브레이크를 만지지 않는다.★

  ★driving.py 를 쓰지 않는다★ 그쪽은 코너 감속·종점 접근·CTE 적분·라이다 이양·
  신호등까지 얹힌 3600줄짜리 주행기이고, 그 장치들이 전부 '제동거리를 재는 일'에
  개입한다(속도를 미리 낮추고, 코너에서 1단을 물고, 종점 5m 앞에서 또 문다).
  ★재려는 것 하나만 남긴 노드★ 가 이 파일이다.

════════════════════════════════════════════════════════════════════════════════
 ⚠️⚠️ 안전 — 읽고 시작할 것 ⚠️⚠️
════════════════════════════════════════════════════════════════════════════════
  기본값 ★10펄스 = 31.8 km/h★ 다. 이 차의 2단 실측 감속도는 2.2~3.8 m/s² 이므로
  (BRAKING.md 1절) 순수 제동거리만:

        v = 10펄스 = 8.84 m/s
        a = 2.2 (하한)  →  17.8 m      a = 3.8 (호조건)  →  10.3 m
        + 판정·행정 지연 0.30s × 8.84 =  2.7 m
        ────────────────────────────────────────────────
        ★트리거 지점 뒤로 13 ~ 20 m 를 더 간다★

  거기에 ①헤딩 초기화 구간(1~2m) ②10펄스까지 가속하는 구간이 앞에 붙는다.
  ★차는 종점을 지나 13~20 m 를 더 간다★ — 경로 끝 뒤로 그만큼이 비어 있어야 한다.
  장애물 시험이라면 ★장애물 뒤★ 로 그만큼이 필요하다(차가 못 서면 들이받는다).
  ⚠️ ★라이다 ROI 는 범퍼 기준 0.8~4.8 m 이고, 감지에서 제동력이 서기까지 ≈575 ms 다★
     (ouster 50 + confirm_frames 2×50 + pedal_drive engage + arduino TX 50 +
      2단 행정 300). 10펄스면 그 사이에만 5.1 m 를 가므로 ★보고 나서 설 수 없다★.
     실제로 세워 보려면 3펄스 이하다(3펄스 = 지연 1.5 m + 제동 1.6 m = 3.1 m).
     사람·부술 물건을 장애물로 쓰지 말 것.

  · E-STOP 은 언제든 듣는다(하드웨어. B보드가 직접 문다).
  · D5 스위치를 수동조종으로 내리면 즉시 손을 뗀다.
  · 경로에서 CTE_ABORT_M 이상 벗어나면 스스로 2단을 물고 끝낸다.
  · 출발 전 차가 경로에서 START_MAX_OFFSET_M 이상 떨어져 있으면 출발하지 않는다.

════════════════════════════════════════════════════════════════════════════════
 ★★ 속도 지령 — 파일 상단 두 값이 전부다 ★★
════════════════════════════════════════════════════════════════════════════════
    DRIVE_PULSE = 10     ← ★기본★ A보드 목표펄스(0~15). 보드 PID 가 이 속도를 맞춘다
    DRIVE_PWM   = 0      ← 0 이 아니면 ★이쪽이 이긴다★ A보드 직접 PWM(16~255)
    ROUTE       = '...'  ← 주행할 gps_data 파일명. 비우면 최신 route_*.csv

  ★왜 둘인가★ 제동거리는 '어떤 속도로 진입했는가' 가 전부인데, 두 경로는 진입
  속도를 만드는 방식이 다르다:
    · 펄스 : 보드 PID 가 목표를 맞춘다. 노면·경사가 달라도 속도가 같아진다.
             ★같은 속도로 반복 측정할 때 쓴다.★
    · PWM  : 듀티를 고정한다. PID·슬루·폭주감지·기동블랭킹이 ★전부 빠진다★
             (A보드 applySide 의 DRIVE_PWM 분기). 속도는 노면에 따라 달라지지만
             ★구동계가 개입하지 않는 순수한 관성 상태★ 로 제동에 들어간다.
             "PID 가 제동 중에 전류를 밀어넣어 거리가 늘어나는가" 를 가릴 때 쓴다.

  ⚠️ PWM 경로는 ★arduino 노드의 auto_direct_pwm 파라미터★ 가 켜져 있어야 한다.
     braketest.launch.py 가 그것을 켜서 띄운다. one_launch.py 는 켜지 않는다.

  ※ 어느 쪽이든 ★리니어가 물리면 A보드 REF 는 0 이 된다★ (arduino compose (4)) —
    제동이 시작되면 구동은 자동으로 끊긴다. 그래서 이 노드는 제동 중에 펄스를
    따로 0 으로 덮지 않는다(덮으면 해제 순간 0→목표로 두 번 전이한다).

════════════════════════════════════════════════════════════════════════════════
 측정하는 것
════════════════════════════════════════════════════════════════════════════════
    정지 사유               라이다 장애물 / 종점 도달
    체결 시점 속도 v0        GPS 변위속도 (/gps_fused[8]) — ★엔코더를 쓰지 않는다★
    체결 시 장애물거리       라이다 정지면 그때의 obstacle_distance [m]
    제동거리 d               체결 지점 ~ 완전정지 지점의 GPS 직선거리
    제동시간 t               체결 ~ 완전정지
    평균 감속도 a = v0 / t   그리고 에너지식 v0²/(2d) 도 함께 낸다
    트리거 초과거리          트리거 지점을 얼마나 지나서 섰는가 (제일 궁금한 값)
                            ★라이다 정지면 = 장애물에 얼마나 다가갔는가★

  ★엔코더로 재지 않는 이유★ A보드 기동 블랭킹의 허수 카운트가 저속에서 위로
  튄다(실측 중앙 16, 최대 34 / 정상 4~5). '완전정지' 판정이 그것 때문에 늦어지면
  제동시간이 통째로 틀어진다. 정지 판정만은 엔코더와 GPS 를 ★둘 다★ 요구한다.
"""

import math
import os
import time
from datetime import datetime

import rclpy
import rclpy.executors
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64MultiArray, Int32, String

from white1 import paths
from white1.gps import GPS_FUSED_TOPIC, Q_LABEL, Q_NONE

# ══════════════════════════════════════════════════════════════════════════════
#  ★★ 여기 두 값만 고치면 된다 ★★
# ══════════════════════════════════════════════════════════════════════════════
DRIVE_PULSE = 10       # A보드 목표펄스 0~15. ★10펄스 = 31.8 km/h★
DRIVE_PWM   = 0           # 0 아니면 이쪽이 이긴다. A보드 직접 PWM 16~255
#  ★주행할 경로★ gps_data 안의 파일명. 빈 문자열이면 가장 최신 route_*.csv 를 쓴다.
#  ★직선(또는 직선에 가까운) 경로여야 한다★ — 이 노드는 코너 감속을 하지 않는다.
ROUTE = 'route_20210606_012345.csv'
#   ※ 셋 다 런치 인자로도 덮을 수 있다:
#      drive_pulse:=8  /  drive_pwm:=140  /  route:=route_20260908_203042.csv

# ── 그 밖의 상수 (건드릴 일이 드물다) ──
CONTROL_HZ    = 20.0      # driving.py 와 같은 주기 (A보드 텔레메트리도 20Hz)

# ══════════════════════════════════════════════════════════════════════════════
#  ★★ [2026-09-09] 헤딩 — ★초기화 구간을 없앴다★ (사용자 지시: 바로 출발) ★★
# ══════════════════════════════════════════════════════════════════════════════
#  종전에는 driving.py 처럼 3펄스로 1~5 m 곧게 굴려 GPS 변위로 헤딩을 확정한 뒤에야
#  가속했다. 이 시험에서는 그 구간이 통째로 군더더기다:
#    · 경로가 ★직선★ 이라 출발 방위를 이미 알고 있다 — 경로가 알려준다.
#    · 사람이 차를 경로 위에 올려 두고 시작하므로 그 방위가 곧 차의 방위다.
#    · 자이로 적분도 필요 없다 — 직선에서는 GPS 코스가 훨씬 정확하고 드리프트가 없다
#      (2026-09-08 U턴 사고의 원인이 자이로였다. 여기서는 그 경로를 아예 안 쓴다).
#
#  ★그래서 헤딩은 두 단계로만 만든다★
#    ① 출발 전 : ★경로의 진행방위★ 를 그대로 쓴다 (아래 seed_heading)
#    ② 굴러가면 : ★GPS 코스★ (/gps_fused[9]) 로 갈아탄다. 10펄스면 5Hz fix 사이에
#       1.77 m 를 가므로 RTK 에서 코스 오차가 1° 안쪽이다 — 자이로보다 낫다.
HEADING_COURSE_MIN_KMH = 2.0   # 이 위로 구르면 GPS 코스를 헤딩으로 쓴다
HEADING_LPF = 0.35             # 코스 잡음 완화 (1.0 = 그대로 따라감)
#  ★유효 코스를 이만큼 연속으로 받아야 '헤딩을 잡았다'로 본다★ 5Hz 원시 fix 기준
#  5회 = 1초 = 10펄스에서 8.8 m — 그 거리면 코스가 확실히 선다.
HEADING_LOCK_N = 5

# ══════════════════════════════════════════════════════════════════════════════
#  ★★ [2026-09-09] 직선 전용 조준 — 출발 0° → 그다음 ★CSV 마지막 점★ ★★
# ══════════════════════════════════════════════════════════════════════════════
#  ★왜 웨이포인트 순수추종을 버렸나★ 경로가 직선이면 중간 점들은 아무 정보도 주지
#  않는다. 그런데 그 점들을 차례로 겨누면 ①GPS 잡음이 목표점을 앞뒤로 흔들고
#  ②LFD 가 속도에 따라 변하며 ③목표가 바뀔 때마다 조향이 계단을 만든다 —
#  전부 사행의 재료다. 직선에서는 ★끝점 하나만 겨누면★ 그 셋이 통째로 사라진다.
#
#  ★두 단계로만 간다★
#   ① 출발 직후 : ★조향 무조건 0°★ — 곧게 굴러 GPS 코스로 헤딩을 세운다.
#      경로에서 빌린 방위(seed_heading)는 '대략 맞다' 일 뿐이라, 그것으로 조향을
#      하면 틀린 만큼 그대로 꺾는다. 차라리 안 꺾고 곧게 가는 편이 낫다 —
#      경로 위에 세워 두고 출발하므로 몇 미터는 곧게 가도 벗어나지 않는다.
#   ② 헤딩을 잡은 뒤 : ★CSV 의 마지막 점★ 을 겨눈다. 목표가 고정이라 계단이 없고,
#      멀리 있어서 각도 변화가 완만하다(= 이득이 낮다 = 안정하다).
#
#  ★가까워지면 이득이 커진다 — 그래서 분모에 바닥을 둔다★
#  δ = atan(2·L·sin α / d) 에서 d 가 작아지면 같은 횡오차가 큰 각이 된다.
#  종점 부근에서 그것이 튀지 않게 d 를 AIM_MIN_DIST_M 아래로 내려가지 않게 한다
#  (driving.pure_pursuit_steer 의 `denom = max(lfd·0.5, dist)` 과 같은 장치다).
AIM_MIN_DIST_M = 10.0
#  ★출발 전 위치 확인★ 경로에서 이보다 멀리 떨어져 있으면 출발하지 않는다.
#  방위를 경로에서 빌려 오므로, 차가 경로 위에 있지 않으면 그 전제가 깨진다.
START_MAX_OFFSET_M = 2.0

WHEELBASE_M        = 1.25     # 축거 (순수추종 기하)
STEER_PLANT_GAIN   = 1.26     # pot 지령 / 도로휠각
STEER_UNDERSTEER   = 5.17     # [deg/(m/s²)]
STEER_MAX_DEG      = 40       # B보드 수용 상한

#  ※ LFD(선행거리) 개념이 없다 — ★끝점 하나를 겨누기 때문★ 이다(위 절).
#    속도에 따라 목표가 움직이지 않으므로 driving.py 의 LFD 평활·레이트 제한도
#    필요 없다. 그 대신 AIM_MIN_DIST_M 이 종점 부근의 이득만 막는다.

# ══════════════════════════════════════════════════════════════════════════════
#  ★★ [2026-09-09] 조향 안정화 — 실차에서 사행이 심했다 ★★
# ══════════════════════════════════════════════════════════════════════════════
#  ★원인은 steer_command 의 언더스티어 항이다 — 그 항은 v² 로 자란다★
#      pot = 1.26·δ + 5.17·v²·tan δ / L
#  두 번째 항은 '타이어가 미끄러지는 만큼 더 꺾는다' 는 ★전방향 보정★ 이고,
#  driving.py 가 ★4펄스(3.54 m/s) 이하에서 126표본으로 맞춘 값★ 이다.
#  10펄스(8.84 m/s)는 v² 가 6.25배라 그 보정이 통째로 과해진다:
#
#      δ 요구      4펄스 pot / 실제꺾임       10펄스 pot / 실제꺾임
#       1.0°        2.2° /  1.7°  (×1.7)      6.9° /  5.5°  ★(×5.5)★
#       3.0°        6.5° /  5.2°  (×1.7)     20.7° / 16.4°  ★(×5.5)★
#      δ→0 루프이득  2.16                       6.90   → ★3.2배★
#
#  즉 ★10펄스에서 1° 요구가 실제로는 5.5° 꺾임 = 횡가속도 6 m/s² 급선회★ 가 된다.
#  경로에 붙으려는 작은 보정이 매번 반대편으로 넘겨 버리니 사행할 수밖에 없다.
#
#  ★고치는 방법 세 가지를 겹친다★
#   ① 언더스티어 항의 속도를 ★모델이 검증된 범위★ 로 자른다. 모델을 캘리브레이션
#      바깥으로 외삽하지 않는다 — 그것이 애초의 잘못이다.
#   ② pot 지령을 ±STEER_LIMIT_DEG 로 자른다(사용자 지시 ±5°). 직선 주행이라
#      그 이상이 필요할 일이 없고, 급선회가 ★원천적으로 불가능★ 해진다.
#      pot 5° → 도로휠 3.97° → 10펄스 횡가속도 4.34 m/s² — 72.5m 직선을 잡기에 충분.
#   ③ 저역통과 + 슬루 제한으로 틱 단위 떨림을 없앤다(mppi 의 cmd.steer_* 와 같은 값).
UNDERSTEER_V_MAX_MS = 3.536   # = 4펄스. ★언더스티어 항에 쓰는 속도의 상한★
STEER_LIMIT_DEG     = 5.0     # pot 지령 절대 상한 (런치 steer_limit_deg)
STEER_LPF           = 0.35    # 조향 저역통과 (mppi cmd.steer_lpf_alpha 와 같다)
STEER_SLEW_DEG_S    = 28.0    # 조향 슬루 [pot deg/s] (mppi cmd.steer_slew_deg_s)

MS_PER_PULSE  = 0.884
KMH_PER_PULSE = 3.182

WP_SEARCH_WINDOW  = 60        # 10펄스는 한 틱에 0.44m 간다 — 창을 넉넉히 둔다
WP_MAX_ADVANCE    = 12
WP_AHEAD_MARGIN_M = 2.0
WP_AHEAD_PENALTY_M = 1.2
CTE_WINDOW_WP     = 60

# ══════════════════════════════════════════════════════════════════════════════
#  ★라이다 정지는 ★관찰만★ 한다 — 사슬에 끼어들지 않는다 [2026-09-09]★
# ══════════════════════════════════════════════════════════════════════════════
#  판정·래치·제동은 cone_lidar_node → pedal_drive_node → arduino 안에서 끝난다
#  (헤더 '라이다 정지' 절). 이 노드는 ★언제 물렸는지★ 를 알아야 제동거리를 재고
#  런치를 끝낼 수 있어서 결과 토픽 둘만 본다. ★이 값으로 브레이크를 만지지 않는다.★
AEB_STOP_TOPIC   = '/aeb_stop'                          # pedal_drive_node → arduino
LIDAR_DIST_TOPIC = '/cone_lidar_node/obstacle_distance'  # 기록용 (판정 아님)

#  ★종점 도달 판정★ 10펄스면 한 틱에 0.44 m 를 가므로 반경을 넉넉히 둔다.
#  driving.py 의 WP_REACH_M(0.9) 은 4펄스 기준이라 여기서는 놓칠 수 있다.
GOAL_REACH_M    = 1.5         # 마지막 WP 를 이 안에 들면 도달
GOAL_PASS_MAX_M = 6.0         # '지나쳤다' 를 인정하는 최대 이격

CTE_ABORT_M     = 3.0         # 이보다 벗어나면 시험을 접고 2단으로 세운다
GPS_TIMEOUT_S   = 2.0
MODE_SETTLE_S   = 0.7

BRAKE_NONE, BRAKE_SOFT, BRAKE_FULL = 0, 1, 2
BRAKE_KEEPALIVE_S = 0.25      # 물고 있는 동안 재발행 (발행자가 여럿인 토픽이다)

#  ★완전정지 판정은 GPS 와 엔코더를 ★둘 다★ 요구한다★ (헤더 '엔코더로 재지 않는
#  이유' 참고). 한쪽만 보면 허수 카운트나 GPS 양자화에 판정이 끌려간다.
STOP_KMH_EPS  = 0.35          # GPS 변위속도가 이 밑
STOP_ENC_EPS  = 0.5           # 엔코더(바퀴 하나 기준)가 이 밑
STOP_HOLD_S   = 0.7           # 그 상태가 이만큼 이어져야 '섰다'
ENC_SUM_TO_PULSE = 0.5
ENC_MEDIAN_N  = 3

BRAKE_MAX_S   = 15.0          # 이 시간 안에 못 서면 굳지 않게 끝낸다(이상 상황)
#  ★완전정지 뒤 리니어를 푼다★ (사용자 지시) 물린 채로 런치를 내리면 다음 사람이
#  차를 밀 수 없고, arduino 가 내려간 뒤에는 풀 방법이 D5 를 내리는 것뿐이다.
#  arduino 의 해제유예(BRAKE_RELEASE_HOLD_S 0.5s)와 B보드 0단 복귀(BRAKE_HOME_MS
#  1000ms)가 있으므로, 0 을 내고 ★실제로 빠질 시간★ 을 준 뒤에 종료한다.
BRAKE_RELEASE_WAIT_S = 1.5
#  ★AEB 가 물고 있을 때 기다려 주는 상한★ 장애물을 치우면 arduino 가 스스로
#  푼다(pedal_drive_node 의 release_clear_s → /aeb_stop false). 그때까지 기다리되,
#  안 치우고 가 버릴 수도 있으니 무한정 붙들지는 않는다.
AEB_RELEASE_MAX_S = 20.0
DONE_LINGER_S = 2.0           # 결과를 찍고 이만큼 뒤에 런치를 내린다

#  정지 사유 (결과 표에 그대로 찍는다)
WHY_LIDAR, WHY_GOAL = '라이다 장애물', '종점 도달'

#  ★HEADING 이 없다★ 런치 = 출발이다(위 '헤딩' 절).
S_WAIT, S_RUN, S_BRAKE, S_RELEASE, S_DONE = (
    'WAIT', 'RUN', 'BRAKE', 'RELEASE', 'DONE')

EARTH_R = 6378137.0


def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


def latlon_to_xy(lat, lon, lat0, lon0):
    x = EARTH_R * math.radians(lon - lon0) * math.cos(math.radians(lat0))
    y = EARTH_R * math.radians(lat - lat0)
    return x, y


# ★HeadingEstimator 를 두지 않는다★ 직선 경로에서는 경로 방위 + GPS 코스가
# 자이로 적분보다 정확하고 드리프트도 없다(상단 '헤딩' 절).


class BrakeTestNode(Node):
    """고정 속도 직진 → S 통과 → 리니어 2단 → 완전정지 → 결과 보고 → 종료."""

    def __init__(self):
        super().__init__('braketest_node')

        self.declare_parameter('data_dir', '')
        self.declare_parameter('route', ROUTE)
        self.declare_parameter('drive_pulse', DRIVE_PULSE)
        self.declare_parameter('drive_pwm', DRIVE_PWM)
        self.declare_parameter('cte_abort_m', CTE_ABORT_M)
        self.declare_parameter('steer_limit_deg', STEER_LIMIT_DEG)
        #  ★자동 출발★ 런치를 띄우면 준비되는 대로 곧바로 굴러간다(사용자 지시).
        #    false 로 두면 /braketest_go 에 true 가 올 때까지 기다린다 — 실차에서
        #    "차 앞을 비웠는지" 를 한 번 더 확인하고 싶을 때 쓴다.
        self.declare_parameter('auto_start', True)

        self.data_dir = paths.data_dir(self.get_parameter('data_dir').value or '')
        self.drive_pulse = int(self.get_parameter('drive_pulse').value)
        self.drive_pwm = int(self.get_parameter('drive_pwm').value)
        self.cte_abort = float(self.get_parameter('cte_abort_m').value)
        self.steer_limit = abs(float(self.get_parameter('steer_limit_deg').value))
        self.auto_start = bool(self.get_parameter('auto_start').value)

        #  ★지령값을 한 번만 정해 둔다★ 주행 중에 바뀌지 않는다 — 그게 이 시험의 전제다.
        if self.drive_pwm > 0:
            self.cmd_value = float(max(16, min(255, self.drive_pwm)))
            self.cmd_kind = f"직접 PWM {int(self.cmd_value)}"
            self.nominal_ms = float('nan')      # PWM 은 속도를 예측할 수 없다
        else:
            self.cmd_value = float(max(0, min(15, self.drive_pulse)))
            self.cmd_kind = (f"{int(self.cmd_value)}펄스 "
                             f"≈ {self.cmd_value * KMH_PER_PULSE:.1f} km/h")
            self.nominal_ms = self.cmd_value * MS_PER_PULSE

        # ── 발행 ──
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.pub_state = self.create_publisher(Bool, '/control_state', 10)
        self.pub_brake = self.create_publisher(Int32, '/brake_level', 10)
        self.pub_event = self.create_publisher(String, '/drive_event', 10)
        self.pub_dstate = self.create_publisher(String, '/drive_state', 10)
        #  ★record 가 파일 이름을 여기서 줍는다★ [2026-09-09]
        #  record.py 는 /drive_cmd 로 온 '<파일명>.csv' 를 붙들어 기록 파일명 앞에
        #  붙인다(_note_route_cmd). 그것이 없으면 ros2bag/★unknown★-<시각>.csv 가
        #  되어, 나중에 로그만 보고는 어느 경로로 달렸는지 알 수 없다.
        #  보조로 /drive_event 에도 '주행 시작 [<파일명>]' 을 낸다(_note_route_event).
        self.pub_cmd_name = self.create_publisher(String, '/drive_cmd', 10)
        #  ★record 가 이 시험도 기록하게 한다★ 그쪽은 /drive_state 가 DRIVE_* 일 때만
        #  켜지므로, 이 노드도 같은 이름을 쓴다(BRAKETEST_* 를 새로 만들지 않는다).
        self.pub_done = self.create_publisher(Bool, '/braketest_done', 10)

        # ── 구독 ──
        self.create_subscription(Float64MultiArray, GPS_FUSED_TOPIC,
                                 self.cb_gps, 10)
        self.create_subscription(Int32, '/encoder', self.cb_encoder, 10)
        self.create_subscription(Bool, '/vehicle_mode', self.cb_mode, 10)
        self.create_subscription(Bool, '/estop', self.cb_estop, 10)
        self.create_subscription(Bool, '/braketest_go', self.cb_go, 10)
        #  ★관찰 전용★ 제동은 arduino 가 이미 했다 — 여기서는 시점과 거리만 받는다.
        from std_msgs.msg import Float32 as _F32
        self.create_subscription(Bool, AEB_STOP_TOPIC, self.cb_aeb_stop, 5)
        self.create_subscription(_F32, LIDAR_DIST_TOPIC, self.cb_lidar_dist, 5)

        # ── 상태 ──
        self.state = S_WAIT
        self.state_t0 = time.time()
        self.lat0 = self.lon0 = None
        self.x = self.y = 0.0
        self.fix_ok = False
        self.fix_time = 0.0
        self.gps_kmh = None
        self.gps_quality = Q_NONE
        self.gps_sigma = float('nan')
        self.heading = None          # 출발 시 경로에서 빌리고, 구르면 GPS 코스로 간다
        self.gps_course = float('nan')  # /gps_fused[9]
        self._heading_locked = False    # ★잠그기 전에는 조향 0°★
        self._course_n = 0              # 유효 코스 연속 수신 수
        self.enc_pulse = 0.0
        self._enc_buf = []
        self.auto_mode = None
        self.estop = False
        self.go = False
        #  라이다 사슬 관찰 (제동은 arduino 가 한다)
        self.aeb_stop = False
        self.lidar_dist = float('nan')

        self.waypoints = []
        self.wp_idx = 0
        self._wp_prev = 0
        self.route_name = ''

        #  ★조향 평활 상태 [2026-09-09]★ (상단 '조향 안정화' 절)
        self._steer_out = 0.0
        self._steer_t = 0.0

        self.brake_now = BRAKE_NONE
        self._brake_out = -1
        self._brake_t = 0.0
        self._last_steer = 0.0

        # 측정
        self.why = ''
        self.own_brake = True
        self.hit_dist = float('nan')
        self.s_hit_idx = None
        self.s_hit_xy = None
        self.brake_t0 = 0.0
        self.brake_v0 = float('nan')
        self.brake_xy = None
        self._still_t = 0.0
        self._done_t = 0.0
        self._reported = False

        if not self.load_route():
            #  ★경로가 없거나 S 가 없으면 아예 시작하지 않는다★ 굴린 뒤에 알면 늦다.
            self.state = S_DONE
            self._done_t = time.time()
            self._reported = True

        self.create_timer(1.0 / CONTROL_HZ, self.loop)
        self.event(f"🅑 브레이크 시험 준비 — 지령 {self.cmd_kind}, "
                   f"경로 {self.route_name or '(없음)'}")

    # ══════════════════════════════════════════════════════════════════════════
    #  경로
    # ══════════════════════════════════════════════════════════════════════════
    def load_route(self):
        """route 파라미터가 가리키는 CSV 를 읽는다. 비었으면 ★최신★ route_*.csv.

        ★[2026-09-09] terrain 열은 보지 않는다★ 정지 트리거가 라이다와 종점으로
        바뀌었다. 'S' 가 적혀 있어도 무시하고, 없어도 아무 문제가 없다.
        """
        import csv as _csv
        #  런치가 빈 값을 주면 ★파일 상단 ROUTE★ 로 떨어지고, 그것도 비면 최신이다.
        name = (str(self.get_parameter('route').value or '').strip()
                or str(ROUTE or '').strip())
        try:
            names = sorted(f for f in os.listdir(self.data_dir)
                           if f.startswith('route_') and f.endswith('.csv'))
        except OSError:
            names = []
        if not name:
            if not names:
                self.event(f"❌ {self.data_dir} 에 route_*.csv 가 없다 — 먼저 매핑할 것")
                return False
            name = names[-1]
            self.event(f"ℹ️ route 를 지정하지 않아 ★최신 경로★ 를 쓴다: {name}")
        path = os.path.join(self.data_dir, name)
        if not os.path.isfile(path):
            self.event(f"❌ 경로 파일 없음: {path}")
            return False

        wps = []
        try:
            with open(path, 'r', encoding='utf-8') as f:
                for row in _csv.DictReader(f):
                    try:
                        wps.append((float(row['latitude']), float(row['longitude'])))
                    except (KeyError, ValueError, TypeError):
                        continue
        except Exception as e:            # noqa: BLE001
            self.event(f"❌ 경로 읽기 실패: {e}")
            return False
        if len(wps) < 2:
            self.event(f"❌ 웨이포인트 부족({len(wps)}개): {name}")
            return False

        self.raw_wps, self.route_name = wps, name
        #  ★'경로 선택' 은 record.py 의 ROUTE_EVENT_HINTS 다★ 문구를 바꾸지 말 것 —
        #  바꾸면 기록 파일명이 조용히 unknown 이 된다.
        self.event(f"📁 경로 선택: {name} — WP {len(wps)}개 "
                   f"(terrain 열은 보지 않는다 — 정지는 라이다 또는 종점이다)")
        return True

    def build_waypoints(self):
        if self.lat0 is None or not getattr(self, 'raw_wps', None):
            return False
        self.waypoints = [latlon_to_xy(la, lo, self.lat0, self.lon0)
                          for (la, lo) in self.raw_wps]
        #  경로 총길이를 알려 준다 — 가속 구간이 충분한지 눈으로 보게.
        total = sum(math.dist(self.waypoints[i], self.waypoints[i + 1])
                    for i in range(len(self.waypoints) - 1))
        v = (self.cmd_value * MS_PER_PULSE if self.drive_pwm <= 0
             else float('nan'))
        note = ""
        if math.isfinite(v) and v > 0:
            #  2단 실측 하한 2.2 로 잡은 보수적 값 + 판정·행정 지연 0.30s
            d_stop = v * 0.30 + v * v / (2.0 * 2.2)
            note = (f" / 이 속도의 2단 정지거리 ≈ {d_stop:.1f}m — "
                    f"★종점 뒤로 그만큼이 비어 있어야 한다★")
        self.event(f"📏 경로 총길이 {total:.1f}m{note}")
        self.wp_idx = 0
        self._wp_prev = 0
        return True

    # ══════════════════════════════════════════════════════════════════════════
    #  구독 콜백
    # ══════════════════════════════════════════════════════════════════════════
    def cb_gps(self, msg: Float64MultiArray):
        """/gps_fused 배열. ★인덱스는 gps.py 헤더의 '배열 규약' 이 계약이다★

        ⚠️ [2026-09-09 버그 수정] 종전에 [5][6][7] 을 quality·sigma·pos_ok 로 읽었는데
        그 자리는 ★is_raw · raw_age_s · dr_dist_m★ 이다. 그래서 fix_ok 가 사실상
        `dr_dist_m > 0.5` 가 되어 ★영원히 false★ 였고, braketest 가 출발하지 못한 채
        "GPS 품질 대기 — NO_FIX" 만 찍었다(실차 로그로 잡았다).
            [0] lat  [1] lon  [2] quality  [3] sigma_m  [4] pos_ok
            [5] is_raw  [6] raw_age_s  [7] dr_dist_m  [8] gps_kmh  [9] course_deg
        """
        d = list(msg.data)
        if len(d) < 10:
            return
        lat, lon = d[0], d[1]
        if not (math.isfinite(lat) and math.isfinite(lon)):
            return
        self.gps_quality = int(d[2]) if math.isfinite(d[2]) else Q_NONE
        self.gps_sigma = d[3]
        self.fix_ok = bool(d[4] > 0.5)          # gps.py 의 min_quality 판정 결과
        self.gps_kmh = float(d[8]) if math.isfinite(d[8]) else None
        self.gps_course = float(d[9]) if math.isfinite(d[9]) else float('nan')
        if self.lat0 is None:
            self.lat0, self.lon0 = lat, lon
        self.x, self.y = latlon_to_xy(lat, lon, self.lat0, self.lon0)
        self.fix_time = time.time()

        #  ★굴러가면 헤딩을 GPS 코스로 갈아탄다★ (상단 '헤딩' 절)
        #  10펄스면 5Hz fix 사이 1.77 m — RTK 에서 코스 오차가 1° 안쪽이다.
        moving = (math.isfinite(self.gps_course) and self.gps_kmh is not None
                  and self.gps_kmh >= HEADING_COURSE_MIN_KMH)
        if not moving:
            self._course_n = 0
            return
        self._course_n += 1
        if not self._heading_locked:
            #  ★잠그는 순간 코스를 그대로 취한다★ 경로에서 빌린 값을 버린다 —
            #  차가 실제로 간 방향이 답이고, 그것이 잠금의 뜻이다.
            self.heading = self.gps_course
            if self._course_n >= HEADING_LOCK_N:
                self._heading_locked = True
                self.event(f"🧭 헤딩 확정 {self.heading:+.1f}° "
                           f"(GPS 코스 {self._course_n}회 연속) — "
                           f"이제 ★CSV 마지막 점★ 을 겨눈다")
            return
        self.heading = wrap180(
            self.heading + HEADING_LPF * wrap180(self.gps_course - self.heading))

    def cb_encoder(self, msg: Int32):
        self._enc_buf.append(float(msg.data))
        if len(self._enc_buf) > ENC_MEDIAN_N:
            del self._enc_buf[0]
        med = sorted(self._enc_buf)[len(self._enc_buf) // 2]
        self.enc_pulse = med * ENC_SUM_TO_PULSE

    def cb_mode(self, msg: Bool):
        self.auto_mode = bool(msg.data)

    def cb_estop(self, msg: Bool):
        self.estop = bool(msg.data)

    def cb_go(self, msg: Bool):
        if bool(msg.data):
            self.go = True

    def cb_aeb_stop(self, msg: Bool):
        """pedal_drive_node 의 확정 신호. ★관찰만 한다★ — arduino 가 이미 물었다."""
        self.aeb_stop = bool(msg.data)

    def cb_lidar_dist(self, msg):
        self.lidar_dist = float(msg.data)

    # ══════════════════════════════════════════════════════════════════════════
    #  출력
    # ══════════════════════════════════════════════════════════════════════════
    def event(self, text):
        self.get_logger().info(text)
        self.pub_event.publish(String(data=text))

    def send(self, value, steer_deg, control):
        """★값을 그대로 낸다★ 펄스든 직접 PWM 이든 arduino 가 대역으로 가른다.

        ★제동 중에도 이 값을 0 으로 덮지 않는다★ arduino 가 브레이크>0 이면 A보드
        REF 를 0 으로 만들기 때문에(compose (4)) 우리가 또 0 을 낼 이유가 없고,
        내면 해제 순간 0→목표로 두 번 전이한다(driving.corner_brake 와 같은 이유).
        """
        msg = Twist()
        msg.linear.x = float(value)
        msg.angular.z = float(steer_deg)
        self.pub_cmd.publish(msg)
        self.pub_state.publish(Bool(data=bool(control)))
        self._last_steer = float(steer_deg)

    def set_brake(self, level):
        if level != self.brake_now:
            self.brake_now = level
            self.publish_brake(force=True)

    def publish_brake(self, force=False):
        """★0 은 재확인하지 않는다★ '놓음' 을 계속 주장하면 남의 정지를 푼다."""
        now = time.time()
        if not force and self.brake_now == self._brake_out:
            if self.brake_now <= 0 or (now - self._brake_t) < BRAKE_KEEPALIVE_S:
                return
        self._brake_out = self.brake_now
        self._brake_t = now
        self.pub_brake.publish(Int32(data=int(self.brake_now)))

    def publish_state(self):
        #  record 가 켜지도록 DRIVE_* 이름을 쓴다(헤더 참고).
        name = {S_WAIT: 'IDLE', S_RUN: 'DRIVE_RUN', S_BRAKE: 'DRIVE_RUN',
                S_RELEASE: 'DRIVE_DONE', S_DONE: 'DRIVE_DONE'}[self.state]
        self.pub_dstate.publish(String(data=name))

    def enter(self, new_state, msg=''):
        self.state = new_state
        self.state_t0 = time.time()
        if msg:
            self.event(msg)

    # ══════════════════════════════════════════════════════════════════════════
    #  기하
    # ══════════════════════════════════════════════════════════════════════════
    def advance_wp(self):
        """진행 포인터를 창 안 최근접점으로. driving.advance_wp_idx 와 같은 규칙."""
        n = len(self.waypoints)
        hi = min(self.wp_idx + WP_SEARCH_WINDOW, n)
        ch = math.cos(math.radians(self.heading))
        sh = math.sin(math.radians(self.heading))
        best_any, d_any = self.wp_idx, float('inf')
        best_ahead, d_ahead = None, float('inf')
        for i in range(self.wp_idx, hi):
            wx, wy = self.waypoints[i]
            dx, dy = wx - self.x, wy - self.y
            d = math.hypot(dx, dy)
            if d < d_any:
                d_any, best_any = d, i
            if dx * ch + dy * sh > -WP_AHEAD_MARGIN_M and d < d_ahead:
                d_ahead, best_ahead = d, i
        best = best_any
        if best_ahead is not None and d_ahead <= d_any + WP_AHEAD_PENALTY_M:
            best = best_ahead
        self.wp_idx = max(self.wp_idx, min(best, self.wp_idx + WP_MAX_ADVANCE))

    def aim_at_goal(self):
        """★CSV 의 마지막 점★ 을 겨누는 도로휠각 [deg, +좌]  [2026-09-09]

        직선 경로 전용이다(상단 '직선 전용 조준' 절). 목표가 고정이라 웨이포인트를
        차례로 겨눌 때 생기던 계단·잡음·LFD 변동이 통째로 사라진다.

            δ = atan(2·L·sin α / max(d, AIM_MIN_DIST_M))

        ★분모에 바닥을 두는 이유★ 종점에 가까워지면 d 가 작아져 같은 횡오차가 큰
        각이 된다 — 그대로 두면 마지막 몇 미터에서 조향이 튄다.
        (driving.pure_pursuit_steer 의 denom 하한과 같은 장치다.)
        """
        gx, gy = self.waypoints[-1]
        dx, dy = gx - self.x, gy - self.y
        ch = math.cos(math.radians(self.heading))
        sh = math.sin(math.radians(self.heading))
        lx = dx * ch + dy * sh          # 전방
        ly = -dx * sh + dy * ch         # 왼쪽 +
        d = math.hypot(lx, ly)
        if d < 1e-3 or lx <= 0.0:
            #  종점을 이미 지났거나 등 뒤다 — 겨눌 것이 없다. 곧게 간다.
            return 0.0
        alpha = math.atan2(ly, lx)
        return math.degrees(
            math.atan2(2.0 * WHEELBASE_M * math.sin(alpha), max(d, AIM_MIN_DIST_M)))

    def steer_command(self, road_deg, v_ms):
        """도로휠각 → B보드 pot 지령. ★부호가 여기서 한 번만 뒤집힌다 (− 좌 / + 우)★

        ★언더스티어 항의 속도를 UNDERSTEER_V_MAX_MS 로 자른다★ 그 항은 v² 로 자라는
        전방향 보정이고 ★4펄스 이하에서 맞춘 값★ 이다(상단 '조향 안정화' 절).
        10펄스에 그대로 쓰면 1° 요구가 실제 5.5° 꺾임이 되어 사행한다.
        모델을 캘리브레이션 바깥으로 외삽하지 않는 것이 이 한 줄의 전부다.
        """
        d = abs(road_deg)
        if d < 1e-6:
            return 0.0
        v = v_ms if (v_ms is not None and math.isfinite(v_ms)) else 0.0
        v = min(abs(v), UNDERSTEER_V_MAX_MS)          # ★외삽 금지★
        pot = (STEER_PLANT_GAIN * d
               + STEER_UNDERSTEER * v * v * math.tan(math.radians(d)) / WHEELBASE_M)
        #  ★직선 전용 상한★ B보드 상한(40°)보다 훨씬 낮게 자른다.
        pot = min(self.steer_limit, min(STEER_MAX_DEG, pot))
        return -math.copysign(pot, road_deg)

    def smooth_steer(self, pot, now):
        """조향 지령에 ★저역통과 + 슬루 제한★ 을 건다 [2026-09-09].

        GPS 코스로 만든 헤딩은 5Hz 로 갱신되므로 틱마다 계단이 생긴다. 그 계단을
        그대로 내보내면 B보드 PD 가 매번 새 목표로 달려가 핸들이 떤다
        (mppi 의 cmd.steer_lpf_alpha / cmd.steer_slew_deg_s 와 같은 문제·같은 값).
        """
        dt = (now - self._steer_t) if self._steer_t > 0.0 else (1.0 / CONTROL_HZ)
        dt = max(1e-3, min(0.2, dt))
        self._steer_t = now
        want = self._steer_out + STEER_LPF * (pot - self._steer_out)
        lim = STEER_SLEW_DEG_S * dt
        self._steer_out += max(-lim, min(lim, want - self._steer_out))
        self._steer_out = max(-self.steer_limit, min(self.steer_limit, self._steer_out))
        return self._steer_out

    def signed_cte(self):
        """경로에서 벗어난 측방거리 [m]. + = 차가 경로 왼쪽."""
        n = len(self.waypoints)
        if n < 2:
            return float('nan')
        lo = max(0, self.wp_idx - CTE_WINDOW_WP)
        hi = min(n - 1, self.wp_idx + CTE_WINDOW_WP)
        best, bi = float('inf'), lo
        for i in range(lo, hi + 1):
            d = math.dist(self.waypoints[i], (self.x, self.y))
            if d < best:
                best, bi = d, i
        j = min(bi + 1, n - 1)
        k = max(bi - 1, 0)
        ax, ay = self.waypoints[k]
        bx, by = self.waypoints[j]
        seg = math.hypot(bx - ax, by - ay)
        if seg < 1e-6:
            return float('nan')
        ux, uy = (bx - ax) / seg, (by - ay) / seg
        return -(self.x - ax) * uy + (self.y - ay) * ux

    def speed_ms(self):
        return None if self.gps_kmh is None else self.gps_kmh / 3.6

    def stopped(self, now):
        """★GPS 와 엔코더를 둘 다 요구한다★ (헤더 '엔코더로 재지 않는 이유')"""
        v = self.gps_kmh
        slow = (v is not None and v <= STOP_KMH_EPS) and self.enc_pulse <= STOP_ENC_EPS
        if not slow:
            self._still_t = 0.0
            return False
        if self._still_t == 0.0:
            self._still_t = now
            return False
        return (now - self._still_t) >= STOP_HOLD_S

    # ══════════════════════════════════════════════════════════════════════════
    #  제어 루프
    # ══════════════════════════════════════════════════════════════════════════
    def loop(self):
        self.publish_state()
        self.publish_brake()          # 물고 있는 동안 재확인 (발행자가 여럿이다)
        now = time.time()

        if self.state == S_DONE:
            self.send(0, self._last_steer, control=True)
            if not self._reported:
                return
            if self._done_t and (now - self._done_t) >= DONE_LINGER_S:
                #  ★런치를 내리는 신호★ braketest.launch.py 가 이 노드의 종료를
                #  받아 전체를 내린다(OnProcessExit → Shutdown).
                self.pub_done.publish(Bool(data=True))
                self.event("🛑 브레이크 시험 종료 — 런치를 내린다")
                raise SystemExit(0)
            return

        # ── 안전 게이트 : 어느 상태에서든 먼저 본다 ──
        if self.estop:
            self.send(0, self._last_steer, control=True)
            self.throttle("🚨 E-STOP 체결 중 — 해제하면 대기부터 다시 시작한다")
            if self.state in (S_RUN, S_BRAKE):
                self.finish("E-STOP 으로 중단")
            return
        if self.auto_mode is not True:
            self.send(0, self._last_steer, control=False)
            self.throttle("⏸️ D5 가 자율주행이 아니다 — 스위치를 올리면 시작한다"
                          if self.auto_mode is False else
                          "⏸️ /vehicle_mode 미수신 — nxde arduino 가 떠 있는지 확인")
            if self.state in (S_RUN, S_BRAKE):
                self.finish("수동조종 전환으로 중단")
            return
        if now - self.fix_time > GPS_TIMEOUT_S:
            self.send(0, self._last_steer, control=True)
            self.throttle(f"⚠️ GPS 두절 — 원시 fix {now - self.fix_time:.1f}s 없음")
            if self.state in (S_RUN, S_BRAKE):
                self.finish("GPS 두절로 중단")
            return

        if self.state == S_WAIT:
            self.run_wait(now)
        elif self.state == S_RUN:
            self.run_follow(now)
        elif self.state == S_BRAKE:
            self.run_brake(now)
        elif self.state == S_RELEASE:
            self.run_release(now)

    def throttle(self, text, period=2.0):
        t = time.time()
        if t - getattr(self, '_thr_t', 0.0) >= period:
            self._thr_t = t
            self.event(text)

    def run_wait(self, now):
        """출발 게이트. ★통과하면 그 자리에서 지정속도로 굴러간다★ (S_HEADING 없음)"""
        self.send(0, 0.0, control=True)
        if not (self.auto_start or self.go):
            self.throttle("⏸️ 출발 대기 — `ros2 topic pub -1 /braketest_go "
                          "std_msgs/msg/Bool '{data: true}'` 로 시작")
            return
        if not self.fix_ok:
            #  ★사유를 정확히 말한다★ pos_ok 는 gps 노드의 min_quality 판정 결과다.
            self.throttle(
                f"⏸️ GPS 품질 대기 — {Q_LABEL.get(self.gps_quality, '?')} "
                f"(σ={self.gps_sigma:.2f}m). gps 노드의 min_quality 문턱 미달이다 — "
                f"낮추려면 런치 인자 min_quality:=1")
            return
        if not self.build_waypoints():
            self.throttle("⏸️ GPS 원점 대기")
            return

        # ── 출발 WP + 경로에서 얼마나 떨어져 있나 ──
        best = min(range(len(self.waypoints)),
                   key=lambda i: math.dist(self.waypoints[i], (self.x, self.y)))
        off = math.dist(self.waypoints[best], (self.x, self.y))
        if off > START_MAX_OFFSET_M:
            #  ★방위를 경로에서 빌려 오므로 차가 경로 위에 있어야 한다★
            self.throttle(
                f"⏸️ 차가 경로에서 {off:.2f}m 떨어져 있다 (한계 "
                f"{START_MAX_OFFSET_M:.1f}m) — 경로 위로 옮기고 다시 시작할 것. "
                f"이 노드는 ★출발 방위를 경로에서 빌려 온다★")
            return

        self.wp_idx = self._wp_prev = best
        self.heading = self.seed_heading(best)
        #  ★기록 파일명을 여기서 못 박는다★ load_route 의 이벤트는 __init__ 에서
        #  나가므로 record 가 아직 구독을 붙이기 전일 수 있다. 여기는 출발 직전이라
        #  record 가 확실히 떠 있고, /drive_state 가 DRIVE_RUN 이 되기 ★한 틱 전★
        #  이라 세션이 열릴 때 이름이 이미 잡혀 있다.
        self.pub_cmd_name.publish(String(data=self.route_name))
        self.enter(S_RUN,
                   f"▶ ★주행 시작★ [{self.route_name}] — WP {best}/"
                   f"{len(self.waypoints)} (경로에서 {off:.2f}m), 출발 방위 "
                   f"{self.heading:+.1f}° (경로에서 빌림). 지금부터 ★{self.cmd_kind}★")

    def seed_heading(self, idx):
        """출발 방위를 ★경로의 진행방위★ 에서 빌린다 [2026-09-09].

        ★왜 이래도 되는가★ 이 노드는 직선 경로 전용이고, 사람이 차를 그 경로 위에
        올려 두고 시작한다. 그러면 경로의 방위가 곧 차의 방위다 — 굴려서 재는
        (driving.py 의 HeadingEstimator) 것보다 빠르고, 이 조건에서는 더 정확하다.
        굴러가기 시작하면 곧바로 GPS 코스로 갈아탄다(cb_gps).
        """
        n = len(self.waypoints)
        j = min(idx + 4, n - 1)
        k = max(j - 8, 0)
        ax, ay = self.waypoints[k]
        bx, by = self.waypoints[j]
        if math.hypot(bx - ax, by - ay) < 1e-6:
            return 0.0
        return math.degrees(math.atan2(by - ay, bx - ax))

    def run_follow(self, now):
        if self.heading is None or not self.waypoints:
            self.send(0, 0.0, control=True)
            return
        self.advance_wp()
        self._wp_prev = self.wp_idx

        # ══════════════════════════════════════════════════════════════════════
        #  ① 라이다 사슬이 세웠다 — ★관찰만 한다★
        # ══════════════════════════════════════════════════════════════════════
        #  /aeb_stop 이 서는 순간 arduino 가 ★이미★ 구동을 끊고 리니어를 물었다
        #  (compose 우선순위 (1-1)). 여기서 /brake_level 을 또 내면 발행자만
        #  늘고 아무 이득이 없다 — ★계측 시점만 잡는다.★
        if self.aeb_stop:
            self.begin_brake(WHY_LIDAR, now, own_brake=False)
            return

        # ══════════════════════════════════════════════════════════════════════
        #  ② 종점 도달 — 반경 또는 통과, 둘 중 먼저 되는 쪽
        # ══════════════════════════════════════════════════════════════════════
        #  ★반경만 보면 놓친다★ 10펄스는 한 틱에 0.44m 를 가고 GPS 도 튄다.
        #  driving.py 와 같이 '지나쳤다' 를 독립 조건으로 둔다 — 한 번 지나가면
        #  반드시 성립하므로 종점 질주의 마지막 방벽이 된다.
        gx, gy = self.waypoints[-1]
        d2goal = math.hypot(gx - self.x, gy - self.y)
        if d2goal <= GOAL_REACH_M:
            self.begin_brake(WHY_GOAL, now)
            return
        ch = math.cos(math.radians(self.heading))
        sh = math.sin(math.radians(self.heading))
        if ((gx - self.x) * ch + (gy - self.y) * sh < 0.0
                and d2goal <= GOAL_PASS_MAX_M):
            self.begin_brake(WHY_GOAL, now)
            return

        # ── 경로이탈 ──
        cte = self.signed_cte()
        if math.isfinite(cte) and abs(cte) > self.cte_abort:
            self.set_brake(BRAKE_FULL)
            self.publish_brake(force=True)
            self.send(0, self._last_steer, control=True)
            self.finish(f"경로이탈 {cte:+.2f}m (한계 {self.cte_abort:.1f}m)")
            return

        # ── 포인터가 끝에 닿았는데 위 종점 판정이 안 섰다 (경로가 짧거나 GPS 이상) ──
        if self.wp_idx >= len(self.waypoints) - 1:
            self.begin_brake(WHY_GOAL, now)
            return

        v = self.speed_ms()
        if not self._heading_locked:
            #  ★① 헤딩을 잡기 전에는 무조건 0°★ (상단 '직선 전용 조준' 절)
            #  경로에서 빌린 방위는 '대략 맞다' 일 뿐이라, 그것으로 조향하면 틀린
            #  만큼 그대로 꺾는다. 곧게 굴러 GPS 코스가 서기를 기다리는 편이 낫다.
            self._steer_out = 0.0
            self.send(self.cmd_value, 0.0, control=True)
            self.throttle(
                f"🅑 곧게 가는 중(조향 0°) — 헤딩 확정 대기 "
                f"{self._course_n}/{HEADING_LOCK_N}, "
                f"{'?' if self.gps_kmh is None else f'{self.gps_kmh:.1f}'}km/h, "
                f"CTE {cte:+.2f}m", period=0.5)
            return

        #  ★② 헤딩을 잡았다 — CSV 마지막 점을 겨눈다★
        road = self.aim_at_goal()
        steer = self.smooth_steer(
            self.steer_command(road, v if v is not None else self.nominal_ms), now)
        self.send(self.cmd_value, steer, control=True)
        self.throttle(
            f"🅑 주행 중 — 종점까지 {d2goal:.1f}m, "
            f"{'?' if self.gps_kmh is None else f'{self.gps_kmh:.1f}'}km/h, "
            f"CTE {cte:+.2f}m, 조향 {steer:+.1f}°", period=1.0)

    def begin_brake(self, why, now, own_brake=True):
        """정지 트리거 — 여기서부터가 측정 구간이다.

        why 는 WHY_LIDAR / WHY_GOAL 이다. 결과 표에 그대로 찍힌다.
        ★own_brake★ 는 '리니어를 내가 무는가' 다:
          · 종점 도달  → True  : 이 노드가 /brake_level 2단을 낸다
          · 라이다     → False : ★arduino 가 이미 물었다★ (/aeb_stop 경로).
                                 여기서 또 내면 발행자만 늘고 이득이 없다.
        """
        self.why = why
        self.hit_dist = self.lidar_dist if why == WHY_LIDAR else float('nan')
        self.s_hit_idx = self.wp_idx
        self.s_hit_xy = (self.x, self.y)
        self.brake_xy = (self.x, self.y)
        self.brake_t0 = now
        self.brake_v0 = self.speed_ms() if self.speed_ms() is not None else float('nan')
        self._still_t = 0.0
        self.own_brake = own_brake
        if own_brake:
            self.set_brake(BRAKE_FULL)
            self.publish_brake(force=True)
        #  ★조향은 유지한다★ 정지 직전에 앞바퀴를 정면으로 꺾으면 제동 중 거동이
        #  바뀌어 측정이 오염된다. 펄스는 arduino 가 0 으로 덮는다(send docstring).
        self.send(self.cmd_value, self._last_steer, control=True)
        v0kmh = (float('nan') if not math.isfinite(self.brake_v0)
                 else self.brake_v0 * 3.6)
        extra = (f", 장애물 {self.hit_dist:.2f}m"
                 if math.isfinite(self.hit_dist) else "")
        who = "리니어 2단 체결" if own_brake else "★arduino AEB 가 물었다★"
        self.enter(S_BRAKE,
                   f"🛑 ★{why}★ (WP {self.wp_idx}/{len(self.waypoints)}{extra}) — "
                   f"{who}. 진입속도 {v0kmh:.2f} km/h ({self.brake_v0:.3f} m/s)")

    def run_brake(self, now):
        self.send(self.cmd_value, self._last_steer, control=True)
        held = now - self.brake_t0
        if self.stopped(now):
            self.report(now, held)
            self.begin_release(now)
            return
        if held >= BRAKE_MAX_S:
            self.report(now, held, note="★시간 초과★ 완전정지를 확인하지 못했다")
            self.begin_release(now)
            return
        self.throttle(
            f"🛑 제동 중 {held:.2f}s — "
            f"{'?' if self.gps_kmh is None else f'{self.gps_kmh:.2f}'}km/h, "
            f"ENC {self.enc_pulse:.1f}펄스", period=0.5)

    # ══════════════════════════════════════════════════════════════════════════
    #  완전정지 뒤 리니어 해제 [2026-09-09 사용자 지시]
    # ══════════════════════════════════════════════════════════════════════════
    def begin_release(self, now):
        """★물린 채로 런치를 내리지 않는다★

        arduino 가 내려간 뒤에는 /brake_level 로 풀 방법이 없어져, 차를 밀려면
        D5 를 내렸다 올리는 수밖에 없다. 그래서 0 을 내고 ★실제로 빠질 시간★ 을
        준 뒤에 종료한다 — arduino 해제유예 0.5s + B보드 0단 복귀 BRAKE_HOME_MS
        1000ms 를 합쳐 BRAKE_RELEASE_WAIT_S 로 잡았다.

        ⚠️ ★AEB 가 물고 있으면 이 노드는 못 푼다★ /aeb_stop 이 true 인 동안
        arduino 는 리니어를 aeb_brake_level 로 계속 물고 있고, 우리 /brake_level=0
        은 그 분기보다 우선순위가 낮다(compose (1-1)). 그건 결함이 아니라 설계다 —
        ★장애물이 그대로 있는데 브레이크를 푸는 노드가 있으면 안 된다.★
        그래서 그때는 '장애물을 치우면 풀린다' 를 말해 주고 기다린다.
        """
        self.set_brake(BRAKE_NONE)
        self.publish_brake(force=True)      # ★0 은 평소 재확인하지 않는다★
        if self.aeb_stop:
            self.enter(S_RELEASE,
                       "🟢 완전정지 확인 — 그런데 ★AEB 가 아직 물고 있다★ "
                       "(/aeb_stop = true). 장애물을 치우면 arduino 가 스스로 "
                       "푼다(release_clear_s). 그때까지 기다렸다 런치를 내린다")
        else:
            self.enter(S_RELEASE,
                       f"🟢 완전정지 확인 — 리니어 해제(0단). "
                       f"{BRAKE_RELEASE_WAIT_S:.1f}초 뒤 런치를 내린다")

    def run_release(self, now):
        self.send(0, self._last_steer, control=True)
        held = now - self.state_t0
        if self.aeb_stop:
            #  ★AEB 가 놓을 때까지 기다린다★ 그래야 리니어가 실제로 빠진다.
            #  무한정 기다리지는 않는다 — 장애물을 안 치우고 가 버릴 수도 있다.
            if held < AEB_RELEASE_MAX_S:
                self.throttle(
                    f"⏸️ AEB 가 리니어를 물고 있다 — 장애물을 치우면 풀린다 "
                    f"({held:.0f}/{AEB_RELEASE_MAX_S:.0f}s)", period=2.0)
                return
            self.event(
                f"⚠️ {AEB_RELEASE_MAX_S:.0f}초가 지나도 /aeb_stop 이 서 있다 — "
                f"★리니어가 물린 채로 런치를 내린다★. 차를 밀려면 D5 를 수동조종으로 "
                f"내렸다 올릴 것(모드 전환은 리니어를 반드시 푼다)")
        if held >= BRAKE_RELEASE_WAIT_S:
            self._done_t = now
            self.enter(S_DONE)

    # ══════════════════════════════════════════════════════════════════════════
    #  결과
    # ══════════════════════════════════════════════════════════════════════════
    def report(self, now, held, note=''):
        d = (math.dist(self.brake_xy, (self.x, self.y))
             if self.brake_xy else float('nan'))
        s_over = (math.dist(self.s_hit_xy, (self.x, self.y))
                  if self.s_hit_xy else float('nan'))
        v0 = self.brake_v0
        #  ★두 가지 방법으로 낸다★ 시간식과 에너지식이 크게 다르면 어딘가 틀렸다
        #  (대개 '완전정지' 판정 시각이 늦은 것이다 — 그때 시간식만 작아진다).
        a_t = v0 / held if (math.isfinite(v0) and held > 0.05) else float('nan')
        a_d = (v0 * v0 / (2.0 * d)) if (math.isfinite(v0) and d > 0.05) else float('nan')
        lines = [
            "═" * 66,
            " 브레이크 제동거리 측정 결과",
            "═" * 66,
            f"  경로            {self.route_name}",
            f"  지령            {self.cmd_kind}",
            f"  ★정지 사유★     {self.why}",
            f"  트리거 지점     WP {self.s_hit_idx}/{len(self.waypoints)}",
        ]
        if math.isfinite(self.hit_dist):
            #  ★라이다 정지일 때만★ 그 순간 본 거리. ROI 가 2.0~6.0 m 라 그 안이다.
            lines.append(
                f"  체결 시 장애물  {self.hit_dist:.2f} m      "
                f"← cone_lidar_node 가 그 순간 본 최근접 거리")
        lines += [
            f"  체결 시 속도 v0 {v0:.3f} m/s ({v0 * 3.6:.2f} km/h)",
            f"  제동시간 t      {held:.2f} s",
            f"  제동거리 d      {d:.2f} m        ← 체결 지점부터 잰 GPS 직선거리",
            f"  트리거 초과거리 {s_over:.2f} m        ← 트리거를 이만큼 지나서 섰다",
            f"  평균 감속도     {a_t:.2f} m/s²  (v0/t)",
            f"                  {a_d:.2f} m/s²  (v0²/2d)",
            "═" * 66,
            "  ※ BRAKING.md 실측 : 2단 2.2~3.8 m/s² / 1단 0.62~1.05 / 코스트 0.29~0.54",
        ]
        if self.why == WHY_LIDAR and math.isfinite(self.hit_dist):
            reach = self.hit_dist - s_over
            lines.append(
                f"  ※ 장애물까지 남은 여유 ≈ {reach:+.2f} m  "
                + ("(★들이받았을 거리다★)" if reach < 0 else "(멈춰 섰다)"))
        if note:
            lines.append(f"  ⚠️ {note}")
        for ln in lines:
            self.get_logger().info(ln)
        self.pub_event.publish(String(data=(
            f"🅑 제동 결과 [{self.why}] — v0 {v0 * 3.6:.2f}km/h, d {d:.2f}m, "
            f"t {held:.2f}s, a {a_t:.2f}/{a_d:.2f} m/s², 초과 {s_over:.2f}m"
            f"{'  ' + note if note else ''}")))
        self._reported = True
        #  ★여기서 DONE 으로 가지 않는다★ 해제 단계(begin_release)가 리니어를 풀고
        #  실제로 빠질 시간을 준 뒤에 DONE 으로 넘긴다.

    def finish(self, why):
        """측정 없이 끝낸다(중단·이상). ★리니어는 물린 채로 두지 않는다★

        2단으로 세운 뒤 해제 단계를 거쳐 나간다 — 중단이라고 물린 채로 두면
        차를 밀 수 없다(begin_release 참고).
        """
        self.set_brake(BRAKE_FULL)
        self.publish_brake(force=True)
        self._reported = True
        self.event(f"⛔ 시험 중단 — {why}. 리니어 2단")
        self.begin_release(time.time())

    def destroy_node(self):
        try:
            self.send(0, self._last_steer, control=True)
            self.pub_brake.publish(Int32(data=BRAKE_NONE))
        except Exception:            # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BrakeTestNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit,
            rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:            # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
