#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
braketest.py ― ★브레이크 제동검차 전용 노드★ [white1 / 2026-09-12 전면 개편]
════════════════════════════════════════════════════════════════════════════════
    ros2 launch white1 braketest.launch.py

  ★이 노드의 존재의의는 '제동검차 통과' 하나다★ — 30 km/h 를 넘는 속도에서
  급제동을 걸어야 한다. 그래서 ★처음부터 고속으로 달린다★. 저속 예행이 목적이
  아니므로, 조향은 '고속 직선에서 절대 흔들리지 않는 것' 하나에만 맞춰져 있다.

      ① 사람이 차를 ★목표 좌표를 일직선으로 바라보는 자리★ 에 세운다
      ② 런치를 띄우면 그 자리에서 ★목표 좌표를 향한 직선★ 을 기준으로 질주한다
      ③ 라이다가 라바콘을 보면 arduino 가 ★즉시 리니어 2단★ (사슬 그대로)
      ④ 라이다가 못 보고 ★목표 좌표에 도달하면 그 즉시 리니어 2단★ (FAIL-SAFE)

  ★[2026-09-12] 경로 CSV 추종을 버렸다 — 기준은 '점'이 아니라 '선' 이다★
  종전에는 매핑 CSV 291점을 최근접 포인터로 좇았고, 그 층이 사행의 재료였다
  (목표 계단 · 최근접점 점프 · 포인터 도약). 지금은 ★두 좌표★ 만 쓴다:

      S = 출발 순간 차의 위치(RTK)      G = ★파일 상단 상수★ GOAL_LAT/GOAL_LON
      û = (G − S)/|G − S|               D = |G − S|

      매 틱 스칼라 셋만 만든다:
          s = (P − S)·û    진행거리   ← 종점 FAIL-SAFE 는 s ≥ D 하나다
          e = (P − S)·n̂    횡오차     ← + = 선의 왼쪽 (RTK 로 ★직접★ 관측된다)
          ψ = 헤딩 − 선방위  방위오차  ← 자이로+GPS 로 ★추정★ 하는 값

  ⓐ 기준에 잡음이 없다(두 좌표에서 나온 방위는 주행 내내 변하지 않는다)
  ⓑ e(0) = 0, ψ(0) ≈ 0 이 ★설계상 보장된다★ — S 가 차 자신의 출발점이기 때문이다
  ⓒ 종점 판정이 반경이 아니라 ★단조증가 스칼라 하나의 문턱★ 이라 못 놓친다

════════════════════════════════════════════════════════════════════════════════
 ★★ 조향 — '멀리 본다' 의 정확한 뜻 : 먼 점은 방향을 주고, 이득은 프리뷰가 준다 ★★
════════════════════════════════════════════════════════════════════════════════
  ★끝점을 '직접' 겨누면 이득이 0 이 된다 (2026-09-09 에 실제로 겪었다)★
  순수추종식 δ = atan(2L·sinα/d) 은 d 가 분모에 있다. d = 70 m 면 횡오차 1 m 가
  도로휠 0.029° = pot 0.06° 이고, B보드 조향 불감대(6카운트 = ★pot 1.78°★)의
  28분의 1 이다 — ★핸들이 한 번도 움직이지 않는다.★ 그 로그가 남아 있다
  (CTE −0.26 → −1.17 m 단조증가, 실측 조향각 내내 그대로).

  ★그래서 목표점은 '선의 방위' 를 정하는 데만 쓰고, 되먹임 이득은 프리뷰 거리가
  정한다★ — 선 위 L_p 앞의 점을 겨눈다:

      e_p = e + L_p·sin ψ
      δ   = atan(2·L·e_p / L_p²)        (선형화: δ = 2L/L_p²·e + 2L/L_p·ψ)

  L_p → ∞ 는 끝점 직접 조준(이득 0), L_p → 0 은 지연 영역 진동이다.
  ★L_p 하나가 위치항과 방향항의 비율도, 전체 이득도 동시에 정한다.★

  ★L_p = T_PREVIEW × v (정속 프리뷰)★ 로 두면 ω_n = √2/T_p 가 되어 ★속도가 변해도
  안정도가 같다★ — 가속 구간과 31.8 km/h 순항 구간에 같은 튜닝이 통한다.

  ┌ T_p ┬ L_p(8.84m/s) ┬  ω_n  ┬ 헤딩루프 PM ┬ 위치이득[pot°/m] ┬ 불감대를 넘기는 e ┐
  │ 1.2 │    10.6 m    │ 1.18  │    +37°     │       8.8        │     0.20 m        │
  │★1.5★│  ★13.3 m★   │★0.94★│   ★+48°★   │      ★5.6★      │    ★0.32 m★      │
  │ 2.0 │    17.7 m    │ 0.71  │    +58°     │       3.2        │     0.56 m        │
  └─────┴──────────────┴───────┴─────────────┴──────────────────┴───────────────────┘
  ω_n 0.94 는 driving.py 가 LFD_OMEGA_N = 0.97 로 쓰는 자리와 같다 — ★이 차에서
  검증된 유일한 안정도 기준★ 이고 그것을 그대로 물려받았다.
  마지막 열이 '이만큼 벌어져야 B보드 서보가 깨어난다' = ★정상상태 잔류 횡오차★ 다.
  차로가 더 좁으면 t_preview:=1.2 가 첫 번째 손잡이다(위상여유를 11° 내준다).

════════════════════════════════════════════════════════════════════════════════
 ★★ 종전 사행의 진짜 원인 — 헤딩 이득 8.8배 과다 (기록으로 남긴다) ★★
════════════════════════════════════════════════════════════════════════════════
  종전 식 : δ = HEADING_K(1.0)·ψ + atan(CTE_K(0.4)·e/v) + RATE_K·r
  · 위치/방향 이득비로 환산한 실효 프리뷰는 22 m 로 나쁘지 않았다.
  · 그런데 22 m 프리뷰의 순수추종 이득은 2L/L_p = 0.113 인데 코드는 1.0 을 썼다
    → ★이득 8.8배★. 헤딩루프 교차주파수 ω_c = K_ψ·v/L 로 보면:

        현행(플랜트 손실 포함)  ω_c 2.21 rad/s → 지연 0.55s 에서 위상여유 ★+20°★
        프리뷰 13.3 m (신규)    ω_c 1.33 rad/s → +48° (레이트 댐핑까지 +70°)

  · 지연 0.55 s(불감시간 0.25 + 서보 τ 0.30) 적분루프의 위상 −180° 점은
    ω = (π/2)/0.55 = 2.86 rad/s = ★주기 2.2 s★ 다. 실측 진동이 ★주기 2.4 s,
    진폭 ±3.8°, cmd 는 −5° 에 붙은 채★ 였다 — 숫자가 맞아떨어진다.
    ★조향이 흔들린 게 아니라, 이득 과다가 교차주파수를 지연 영역으로 밀어 넣었다.★

════════════════════════════════════════════════════════════════════════════════
 ★★ 전달비(pot/도로휠) — 속도 클램프가 루프이득을 3.2배 깎고 있었다 ★★
════════════════════════════════════════════════════════════════════════════════
      pot = 1.26·δ + 5.17·v²·tan δ / L            (δ = 도로휠각)

  종전 코드는 이 식의 v 를 4펄스(3.536 m/s)로 ★잘라서★ 썼다("모델을 캘리브레이션
  밖으로 외삽하지 않는다"). 그런데 도로휠 1° 기준으로 값을 비교하면:

        코드가 쓰던 값 (v 클램프 3.536)   2.16
        모델을 실제 속도로 (v = 8.84)     6.90
      ★로그 회귀 실측 (6~9 m/s, 468표본)  5.89★    ← 클램프 없는 모델과 맞는다

  즉 이 속도에서는 모델이 맞았고, 클램프가 ★지령한 도로휠각의 31% 만 실제로
  꺾이게★ 만들고 있었다. ★클램프를 없앤다.★ 대신 저속(2~4 m/s) 회귀값 4.96 이
  모델(1.9)보다 크므로 ★전달비 하한 PLANT_GAIN_MIN★ 을 둔다 — 가속 구간에서
  지령이 불감대 밑으로 죽지 않게 하는 장치다.

════════════════════════════════════════════════════════════════════════════════
 ★★ 상한은 '각도' 가 아니라 '횡가속도' 로 정한다 + 요레이트 가드 ★★
════════════════════════════════════════════════════════════════════════════════
  종전 STEER_LIMIT_DEG = 5.0 은 "pot 5° = 도로휠 3.97° = 횡가속 4.34 m/s²" 라는
  계산으로 정해졌는데, 실측 전달비로 다시 재면 ★도로휠 0.72° = 0.79 m/s²★ 다 —
  의도의 1/5.5 짜리 권한이었고, 그래서 ★항상 포화★ 했다. 포화는 곧 계전기이고
  계전기는 반드시 반대편으로 넘긴다.

  · 상한은 AY_MAX_MS2(횡가속도)에서 유도한다 — 속도가 달라져도 뜻이 변하지 않는다.
  · ★요레이트 가드★ 가 최종 방벽이다. 전달비가 실측보다 예민한 쪽으로 틀려 있어도
    (= 같은 pot 이 더 많이 꺾여도) ★실제로 돌고 있는 속도★ 를 자이로가 지연 없이
    알려 주므로, |r| 이 r_max = AY_MAX/v 를 넘으면 그 방향 지령을 그 자리에서
    비율만큼 깎는다. ★플랜트 이득을 몰라도 횡가속도가 상한을 넘지 못한다.★
    자이로가 없으면(신선하지 않으면) pot 상한을 POT_NOIMU_MAX_DEG 로 조인다 —
    ★모르면 낮게 본다.★

════════════════════════════════════════════════════════════════════════════════
 라이다 정지 — ★lidar one_launch.py 의 사슬을 ★그대로★ 쓴다 (사용자 지시)★
════════════════════════════════════════════════════════════════════════════════
  ouster ─/ouster/points─▶ cone_lidar_node ─/…/stop_signal─▶ pedal_drive_node
                              (인지·판정)                      (확정·래치)
                                                                    │ /aeb_stop
                                                                    ▼
                                                          nxde/arduino  (구동차단
                                                                       + 리니어 2단)

  ★이 노드는 그 사슬에 끼어들지 않는다★ 판정·래치·제동이 전부 저 세 노드 안에서
  끝난다. braketest 가 /aeb_stop 을 구독하는 것은 ★계측★ 때문이다 — 언제 물렸는지를
  알아야 거리·시간을 재고 런치를 끝낼 시점을 안다. ★그 값으로 브레이크를 만지지
  않는다.★ (다만 ★판단자의 생사★ 는 본다 — 주행 중 사슬이 끊기면 그 자리에서 접는다.)

════════════════════════════════════════════════════════════════════════════════
 ⚠️⚠️ 안전 — 읽고 시작할 것 ⚠️⚠️
════════════════════════════════════════════════════════════════════════════════
  기본값 ★10펄스 = 31.8 km/h★ 다. 2단 실측 감속도가 2.2~3.8 m/s² 이므로:

        v = 8.84 m/s → 순수 제동거리 10.3 ~ 17.8 m + 지연 0.30s × 8.84 = 2.7 m
        ────────────────────────────────────────────────────────────────────
        ★트리거 지점 뒤로 13 ~ 20 m 를 더 간다★

  ★목표 좌표는 "여기서 서라" 가 아니라 "여기서 밟기 시작하라" 다★ —
  그 뒤로 ★최소 30 m★ 가 비어 있어야 한다. 앞쪽에는 가속 구간이 더 붙는다.
  ⚠️ 라이다 ROI 는 범퍼 기준 0.8~4.8 m 이고 감지→제동력까지 ≈575 ms 다. 10펄스면
     그 사이에만 5.1 m 를 가므로 ★보고 나서 설 수 없다★ — 이 시험은 '얼마나 못
     서는가' 를 재는 것이다. ★사람·부술 물건을 장애물로 쓰지 말 것.★

  · E-STOP 은 언제든 듣는다(하드웨어. B보드가 직접 문다).
  · D5 스위치를 수동조종으로 내리면 즉시 손을 뗀다.
  · 선에서 CTE_ABORT_M 이상 벗어나거나, 방위가 PSI_ABORT_DEG 이상 틀어지거나,
    ★필터된 횡가속도★ 가 상한의 AY_ABORT_K 배를 넘으면 스스로 2단을 물고 끝낸다.
  · 출발 1초 뒤 ★GPS 코스로 조준을 검산★ 한다 — 목표를 안 보고 있으면 즉시 접는다.

════════════════════════════════════════════════════════════════════════════════
 측정하는 것
════════════════════════════════════════════════════════════════════════════════
    정지 사유               라이다 장애물 / 목표 좌표 도달(FAIL-SAFE)
    체결 시점 속도 v0        GPS 변위속도 (/gps_fused[8]) — ★엔코더를 쓰지 않는다★
    체결 시 장애물거리       라이다 정지일 때 obstacle_distance [m]
    제동거리 d / 제동시간 t  체결 지점 ~ 완전정지
    평균 감속도             v0/t 와 v0²/2d ★두 방법으로★
    트리거 초과거리          트리거를 얼마나 지나서 섰는가 (제일 궁금한 값)

  ★엔코더로 재지 않는 이유★ A보드 기동 블랭킹의 허수 카운트가 저속에서 위로 튄다
  (실측 중앙 16, 최대 34 / 정상 4~5). '완전정지' 판정만은 엔코더와 GPS 를 ★둘 다★
  요구한다.
"""

import math
import os
import time

import rclpy
import rclpy.executors
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, Float64MultiArray, Int32, String

from white1 import paths
from white1.gps import GPS_FUSED_TOPIC, Q_LABEL, Q_NONE

# ══════════════════════════════════════════════════════════════════════════════
#  ★★★ 여기 네 값만 고치면 된다 ★★★
# ══════════════════════════════════════════════════════════════════════════════
#  ★목표 좌표★ — 차를 ★이 점을 일직선으로 바라보는 자리★ 에 세우고 런치를 띄운다.
#  주행의 기준선은 '출발 순간의 차 위치 → 이 점' 이다. 도달하면(라이다가 먼저
#  세우지 않았다면) ★그 즉시 리니어 2단★ 이다.
#    · 0.0 을 그대로 두면 ★출발하지 않는다★ (아래 resolve_goal 이 사유를 말한다).
#    · 런치 인자 goal_lat / goal_lon 이 이 값을 덮는다.
#    · 둘 다 비어 있고 route 가 주어지면 ★그 CSV 의 마지막 점★ 을 목표로 쓴다.
GOAL_LAT = 37.23900890    # ★★ 여기에 목표 위도 ★★  [2026-09-13 설정]
GOAL_LON = 126.77539272   # ★★ 여기에 목표 경도 ★★  [2026-09-13 설정]

# ══════════════════════════════════════════════════════════════════════════════
#  ★★ [2026-09-13 저녁] 순항 10 → 9 펄스 — 33.9 km/h 는 ROI 15 m 의 한계였다 ★★
# ══════════════════════════════════════════════════════════════════════════════
#  ★A보드가 지령보다 ★+1펄스★ 높게 돈다 — 실측으로 확인됐다★
#  2026-09-13 두 주행(111151·111403) 모두 지령 10펄스인데 :
#        엔코더 11.0~11.5펄스   GPS 33.9 / 33.5 km/h
#        11.0펄스 × 3.182 = ★35.0 km/h★ — 엔코더 환산과 GPS 가 정확히 맞는다
#  즉 환산이 틀린 게 아니라 ★A보드 PID 가 목표를 넘어 돌고 있다.★
#  → 지령 9펄스면 엔코더 ≈10펄스 = ★31.8 km/h★ 로, 원하는 31~32 대역에 들어온다.
#
#  ★왜 낮추나 — 33.9 km/h 는 라이다 ROI 의 한계 속도다★
#  실측 2단 감속도는 ★4.0~4.2 m/s²★ 로 아주 좋다(제동력은 문제가 아니다).
#  그래도 필요 감지거리가 v·0.35 + v²/8 이라 :
#        33.9 km/h → 14.4 m   (ROI 상한 15.0 m, ★여유 0.6 m = 4%★)
#        31.8 km/h → 13.0 m   (여유 2.0 m)
#  실제로 두 주행의 첫 감지가 14.65 m 와 ★12.87 m★ 로 갈렸고, 뒤쪽이
#  목표를 0.49 m 넘겼다. 한 프레임(0.47 m)만 늦어도 넘기는 대역이었다.
#  ⚠️ 감지 거리가 흔들리는 이유는 따로 있다 — cone_lidar_node.cpp 주석이
#     "min_point_count=5 기준 유효 검출한계 ≈6.5 m" 라고 적고 있는데 지금 YAML 은
#     stop_distance_threshold 15.0 · min_point_count 1 이다. ★점 1개로 문턱을 낮춰
#     15 m 까지 늘린 상태★ 라 그 거리에서는 리턴 유무가 프레임마다 갈린다.
#     그쪽은 이번에 손대지 않았다(검출률 실측이 없다).
DRIVE_PULSE = 9           # A보드 목표펄스 0~15. ★실측 ≈31.8 km/h★ (순항)
DRIVE_PWM   = 0           # 0 아니면 이쪽이 이긴다. A보드 직접 PWM 16~255
# ══════════════════════════════════════════════════════════════════════════════
#  ★★ [2026-09-13] 2단 속도 루틴 — 출발 15펄스 → 30 km/h 에서 10펄스 ★★
# ══════════════════════════════════════════════════════════════════════════════
#  ★제동검차는 30 km/h 를 ★넘겨야★ 통과한다(사용자 지시).★ 2026-09-13 로그는
#  10펄스 고정으로 달려 최고 33.37 km/h 까지 갔지만, 거기까지 ★9.1 초·48 m★ 가
#  걸렸다(0 → 30 km/h). 목표까지 89 m 밖에 없으니 절반을 가속에 쓴 셈이고,
#  제동을 걸 무렵 속도가 막 30 을 넘긴 참이라 여유가 없다.
#
#      t=0 출발 → t=9.1 s 에 30 km/h (48.2 m 소모) → 제동 시작 t=13.6 s
#
#  ★그래서 출발만 15펄스로 민다★ A보드 FF 테이블에서 15펄스는 PWM ≈147 이라
#  10펄스(≈130)보다 훨씬 세게 밀고, 30 km/h 도달이 눈에 띄게 앞당겨진다.
#  30 km/h 를 ★GPS 로★ 확인하는 순간 순항(10펄스)으로 내리고, 그 뒤에 제동한다.
#  ⚠️ 15펄스는 A보드 TARGET_MAX 와 같은 값이다(그 위는 없다).
#  ⚠️ 내려가기만 한다 — 한 번 순항으로 내려오면 ★다시 올리지 않는다★(래치).
#     올렸다 내렸다 하면 그것이 곧 속도 되먹임이고, 이 시험이 재려는 것이 아니다.
LAUNCH_PULSE      = 15    # 출발 구간 목표펄스 (A보드 상한과 같다)
CRUISE_SWITCH_KMH = 30.0  # ★GPS 속도★ 가 이 값을 넘으면 순항(DRIVE_PULSE)으로 내린다
#  ── 내려오는 방벽 둘 — 어느 쪽이든 ★내리는 방향★ 이라 안전하다 ──
#  ⓐ GPS 가 끝내 안 오면 엔코더로도 본다. 엔코더는 기동 허수로 ★위로★ 튀므로
#     (실측 중앙 16, 최대 34) 이쪽이 먼저 걸리면 조금 일찍 내려가는 것뿐이다.
#  ⓑ 그래도 안 걸리면 시간으로 내린다. 15펄스로 무한정 달리지 않게 하는 마지막 줄.
LAUNCH_SWITCH_ENC_PULSE = 10.0  # 엔코더(바퀴 기준) 이 펄스를 넘어도 내린다
#  ★[2026-09-13 저녁] 엔코더 폴백에 기동 블랭킹을 건다★
#  ⚠️ 이것을 빼먹어서 첫 시험에서 ★출발 1초 만에 순항으로 내려갔다.★
#     로그(111151·111403) :
#         t=1.09s  엔코더 ★19.0펄스★ → 순항 전환   그런데 GPS 는 0.59 km/h
#         t=0.99s  엔코더 ★14.0펄스★ → 순항 전환   그런데 GPS 는 0.69 km/h
#     ★차는 서 있었다.★ 이 19·14 는 CLAUDE.md 1.2절의 ★기동 초반 허수 펄스★ 다
#     ("코일에 힘은 들어갔는데 바퀴가 아직 안 도는" 구간, 실측 중앙 16 · 최대 34).
#     그래서 15펄스 구간이 1초뿐이었고 30 km/h 도달이 전혀 앞당겨지지 않았다.
#  ★고친 방법★ 엔코더 폴백은 ★출발 후 이 시간이 지나야★ 본다. A보드 기동
#  블랭킹의 타임아웃(LAUNCH_MAX_MS = 3000)과 같은 값이라, 그 구간이 끝난 뒤에만
#  엔코더를 믿는다는 뜻이 된다. GPS 본선은 이 블랭킹을 받지 않는다 —
#  GPS 는 허수 펄스와 무관하고, 애초에 그쪽이 사용자가 지시한 판정이다.
LAUNCH_ENC_BLANK_S      = 3.0   # [s] 이 시간 전에는 엔코더 폴백을 보지 않는다
LAUNCH_MAX_S            = 12.0  # [s] 이 시간이 지나면 무조건 순항으로 내린다
#  ★목표 좌표를 CSV 에서 빌릴 때만 쓴다★ (GOAL_LAT/LON 이 0 일 때의 폴백)
ROUTE = ''
# ══════════════════════════════════════════════════════════════════════════════
#  ★★ [2026-09-13 저녁] 기준선을 옆으로 평행이동한다 ★★
# ══════════════════════════════════════════════════════════════════════════════
#  S(출발점)와 G(목표 좌표)로 그은 직선을 ★진행방향 기준 왼쪽★ 으로 이 거리만큼
#  통째로 옮긴다. 차는 원래 선이 아니라 ★옮긴 선★ 을 따라간다.
#      + = 왼쪽   −  = 오른쪽   0 = 종전(옮기지 않는다)
#  ★평행이동이라 진행거리 s 는 바뀌지 않는다★ — 목표 도달 판정(s ≥ D)도, 남은
#  거리도 종전과 같다. 바뀌는 것은 횡오차 e 의 영점 하나뿐이다.
#  ⚠️ 중단 방벽(cte_abort)도 ★옮긴 선 기준★ 으로 잰다 — 그게 맞다. 새 선이
#     추종 대상이므로, 그 선에서 벗어난 양이 판정 대상이어야 한다.
LATERAL_OFFSET_M = 0.20   # [m] ★왼쪽으로 20 cm★ (런치 인자 lateral_offset_m 이 이긴다)
#   ※ 넷 다 런치 인자로도 덮을 수 있다:
#      goal_lat:=37.1 goal_lon:=127.1 / drive_pulse:=8 / drive_pwm:=140 / route:=...

# ── 그 밖의 상수 ──
CONTROL_HZ = 20.0         # driving.py 와 같은 주기 (A보드 텔레메트리도 20Hz)

MS_PER_PULSE  = 0.884
KMH_PER_PULSE = 3.182

WHEELBASE_M      = 1.25   # 축거
STEER_PLANT_GAIN = 1.26   # pot 지령 / 도로휠각 (링키지비)
STEER_UNDERSTEER = 5.17   # [deg/(m/s²)] 언더스티어 보정
# ══════════════════════════════════════════════════════════════════════════════
#  ★★ [2026-09-13] 언더스티어 항의 속도를 다시 묶는다 — 조향이 포화한 채 떨었다 ★★
# ══════════════════════════════════════════════════════════════════════════════
#  ★2026-09-13 로그 실측★ 조향 부호반전 ★129.9 회/분★ (같은 날 driving 7펄스가
#  22.8, 4펄스가 0.6). 요레이트 std 15.97 °/s, 진동 주기 ★1.00 s = 6.28 rad/s★.
#  횡오차는 작았다(max 0.56 m) — ★궤적은 맞는데 조향만 계속 떨었다.★
#
#  ★원인은 포화다★ 이벤트 로그가 그대로 말한다 — t=2.3~9.5 s 내내 :
#      조향 -6.0°/6.0   (조준 -0.92 + 댐핑 -0.80 도로휠)
#      조향 -5.7°/5.7   (조준 -1.14 + 댐핑 -0.22 도로휠)
#  요구는 도로휠 1~2° 인데 pot 지령이 상한에 ★붙박이★ 다. 포화한 제어기는
#  계전기(bang-bang)가 되고, 계전기는 반드시 떤다.
#
#  ★왜 포화하나 — 이 식이 고속에서 pot 을 4~5배로 부풀린다★
#      v=8.3 m/s, 도로휠 1.62° 요구 :
#        pot = 1.26×1.62 + 5.17×69×tan1.62°/1.25 = 2.04 + 8.07 = ★10.1°★
#      그런데 ★이 노드가 주행 중에 스스로 잰 전달비★(update_plant_estimate)는
#      같은 구간에서 ★g_est 2.1 → 3.2 (표본 51)★ 였다. 모델 7.1 의 ★1/2.6★ 이다.
#      즉 pot 10.1 은 실제로 도로휠 ★3.7°★ 를 만든다 — 요구의 2.3배.
#  ⚠️ 헤더 '전달비' 절의 회귀 5.89 는 ★고속 직선의 1~2° 표본★ 이라 조향 불감대
#     아티팩트다(같은 날 driving 21340표본 회귀: 1~2° 에서 G 5.5, 7~12° 에서
#     ★1.4~1.5 로 수렴★ — 물리 전달비라면 각도와 무관해야 한다). 그 값을 근거로
#     [2026-09-12] 에 클램프를 없앤 것이 이번 진동을 만들었다. ★되살린다.★
#
#  ★값의 근거★ 이 노드가 잰 g_est 중앙 ≈2.7 에 모델을 맞춘다:
#      G(v_eff) = 1.26 + 5.17·v_eff²·0.01745/1.25 = 1.26 + 0.0724·v_eff²
#      v_eff = 4.42 (5펄스) → G = ★2.67★   ← 실측 2.7 과 일치
#  4.42 m/s 이하에서는 ★식이 한 글자도 바뀌지 않는다.★
UNDERSTEER_V_CLAMP_MS = 4.42   # [m/s] 언더스티어 항의 v 를 여기서 묶는다(5펄스)
#                                (0 이하면 클램프를 끈다 = [2026-09-12] 거동)
STEER_MAX_DEG    = 40     # B보드 수용 상한 (여기까지 갈 일은 없다)

# ══════════════════════════════════════════════════════════════════════════════
#  ★★ 조향 — 프리뷰 추종 (헤더 '멀리 본다' 절이 근거 전부다) ★★
# ══════════════════════════════════════════════════════════════════════════════
#  ★이득을 정하는 값은 이것 하나다★ L_p = T_PREVIEW_S × v.
#  1.5 s → 8.84 m/s 에서 13.3 m, ω_n 0.94 rad/s, 헤딩루프 위상여유 +48°.
#  ★키우면 매끄럽지만 벌어진 채로 가고, 줄이면 정밀하지만 위상여유를 먹는다.★
T_PREVIEW_S = 1.5
LP_MIN_M    = 5.0         # 출발 직후 v≈0 에서 L_p 가 0 이 되는 것만 막는다
LP_MAX_M    = 25.0        # 그 위로는 이득이 의미 없이 0 에 가까워진다

#  ★요레이트 댐핑 = 예견시간★ 지연 보상이다. 자이로는 ★지연이 없어서★ 오차가 0 을
#  지나기 전에 미리 힘을 뺀다 — 포화가 반대편으로 넘어가는 것을 막는 장치다.
#  이득을 따로 두지 않고 ★프리뷰 이득에 비례★ 시킨다(K_r = 2L/L_p × RATE_T_S) —
#  그래야 T_PREVIEW 를 바꿔도 감쇠비가 그대로 따라온다.
RATE_T_S = 0.30           # ≈ B보드 조향 불감시간 0.250 s

#  ★전달비 하한★ pot/도로휠. 로그 회귀 실측이 저속에서 4.96 인데 모델은 1.9 라,
#  가속 구간에서 지령이 불감대 밑으로 죽는 것을 막는다(헤더 '전달비' 절).
#  ★[2026-09-13] 4.0 → 2.6★ 위 클램프를 되살리면 모델이 전 속도대역에서 2.67 로
#  평평해지는데, 하한이 4.0 이면 ★하한이 늘 이겨★ 클램프가 무의미해진다(4.0/2.7 =
#  1.5배 과조향이 그대로 남는다). 2.6 은 이 노드가 실제로 잰 g_est(2.1~3.2, 중앙
#  2.7)의 아래쪽이고, 전달비 추정의 하한(PLANT_EST_CLAMP 1.5)보다는 위다.
#  ⚠️ 종전 근거였던 '저속 회귀 4.96' 도 작은 각 표본이라 위 불감대 아티팩트다.
PLANT_GAIN_MIN = 2.6

#  ══════════════════════════════════════════════════════════════════════════════
#  ★★ 전달비를 주행 중에 ★잰다★ — 이 시험 최대의 미지수를 스스로 지운다 ★★
#  ══════════════════════════════════════════════════════════════════════════════
#  실측 조향각(/steer_angle_measured = pot)과 투영 요레이트가 둘 다 있으므로
#      δ_실제 = atan(r·L / v)          G_est = |pot실측| / |δ_실제|
#  를 주행 중에 그대로 계산할 수 있다. ★이 값은 두 가지로 쓰인다★
#    ① pot 상한을 ★조이는 쪽으로만★ 쓴다 — 차가 모델보다 예민한 것으로 드러나면
#       권한을 줄이고, 둔한 것으로 드러나면 모델값을 그대로 둔다(한쪽으로만 움직이는
#       보정이라 추정이 튀어도 위험해지지 않는다).
#    ② 결과 표에 찍는다 — ★다음 시험의 튜닝 근거가 이 한 줄에서 확정된다.★
PLANT_EST_MIN_V_MS   = 4.0    # 이 밑에서는 요레이트가 작아 비율이 지저분하다
PLANT_EST_MIN_POT    = 3.0    # 불감대(1.78°)보다 확실히 큰 지령일 때만
PLANT_EST_MIN_DPS    = 2.0    # 실제로 돌고 있을 때만
PLANT_EST_ALPHA      = 0.12   # EMA (≈8표본 시정수)
PLANT_EST_MIN_N      = 15     # 이만큼 모이기 전에는 쓰지 않는다
PLANT_EST_CLAMP      = (1.5, 15.0)
POT_LIMIT_FLOOR_DEG  = 3.0    # 추정이 아무리 작아도 이 밑으로는 권한을 뺏지 않는다

#  ★횡가속도 상한★ 이것이 사람이 정할 값이다. 직선 보정에는 넉넉하고 급선회는
#  원천적으로 불가능하다. 8.84 m/s 에서 pot ≈ 12.6° 에 해당한다.
AY_MAX_MS2 = 2.0
#  ★pot 절대 상한★ AY 유도값이 저속에서 커지는 것을 자른다(런치 steer_limit_deg).
POT_HARD_MAX_DEG = 12.0
#  ★전달비를 재기 전까지의 상한★ — ★모르면 낮게 본다★
#  전달비가 모델만큼 예민할 가능성이 남아 있는 동안에는 권한을 열지 않는다.
#  종전 시험이 실제로 달린 권한(5°)과 같은 수준이고, 출발 직후 몇 초는 e ≈ 0 이라
#  이것으로 충분하다. 차가 스스로 둔하다는 것을 증명하면(PLANT_EST_MIN_N 표본)
#  그때 AY_MAX 가 허락하는 데까지 열린다.
POT_START_MAX_DEG = 6.0
#  ★자이로가 없으면 이만큼만★ 요레이트 가드가 못 도니 권한을 줄인다 — 모르면 낮게.
POT_NOIMU_MAX_DEG = 6.0
#  ★[2026-09-13] 속도를 모를 때도 같은 상한으로 묶는다★ pot_limit 은 δ_max 를
#  atan(ay_max·L/v²) 로 얻는데, v 를 모르면 하한 2.0 m/s 로 보므로 δ_max 가 32° 까지
#  열려 ★상한이 pot_hard_max(12°) 로 최대가 된다.★ 2026-09-13 로그에서 GPS 가
#  게이트에 막힌 t=10.5 s 부터 정확히 그렇게 됐다(상한 5.7 → ★12.0★).
#  '모르면 낮게 본다' 를 여기에도 적용한다 — 속도를 모르는 동안은 권한을 주지 않는다.
POT_NOSPEED_MAX_DEG = 6.0

#  ★조향 저역통과를 쓰지 않는다★ (1.0 = 통과) 기준에서 계단이 사라졌으므로
#  거를 것이 없고, 필터는 곧 지연이다(τ = dt/α). 슬루만 안전용으로 남긴다.
STEER_LPF        = 1.0
STEER_SLEW_DEG_S = 35.0   # B보드 서보 실측 슬루 70°/s 의 절반

# ── CTE 적분 = ★조향 영점 트림★ (수렴용이 아니다) ──────────────────────────────
#  ⚠️ 8초짜리 주행에서 적분이 수렴에 기여할 수는 없다. 그렇게 만들려면(T_i ≈ 1.3s)
#  교차주파수에서 위상을 크게 먹어 오히려 흔들린다. 그래서 ★T_i ≈ 5/ω_c★ 로 느리게
#  두고, 이 항의 일은 ★휠얼라인먼트·조향 영점 같은 진짜 정상편차를 지우는 것★ 으로
#  한정한다. 한 번의 주행 안에서 효과를 기대하지 말 것.
#  ★영점이 실제로 치우쳐 있다면★ 지난 로그의 직진 구간 평균 pot 을 읽어
#  STEER_TRIM_POT_DEG 에 적는 편이 정직하고 빠르다.
CTE_KI            = 0.15   # [deg(도로휠)/(m·s)]
CTE_I_CLAMP       = 3.0    # [m·s]
CTE_I_MAX_DEG     = 0.5    # [deg] 도로휠 기여 상한 (pot 기준 ≈3.5°)
CTE_I_DEADBAND_M  = 0.05
CTE_I_DECAY_PER_S = 0.5
CTE_I_FLIP_SCALE  = 0.35
CTE_I_MIN_PULSE   = 0.3    # 실측이 이 밑이면 ★동결★

#  ★조향 영점 트림 [pot deg]★ + 면 우, − 면 좌. 직진인데 한쪽으로 계속 밀리면
#  지난 로그의 steer_measured_deg 평균을 부호 반대로 적는다. 기본은 0(보정 없음).
STEER_TRIM_POT_DEG = 0.0

# ══════════════════════════════════════════════════════════════════════════════
#  ★★ 헤딩 — 자이로(빠르다) + GPS 코스(안 흘러간다) 상보필터 ★★
# ══════════════════════════════════════════════════════════════════════════════
#  ★출발 방위는 '선의 방위' 에서 빌린다★ 사용자가 차를 목표 좌표를 바라보게 세우기
#  때문이다. 그 전제가 틀리면 ★출발 1초 뒤 GPS 코스가 알려 주고 시험을 접는다★
#  (AIM_CHECK_MAX_DEG). 그래서 틀린 전제로 질주하는 경우가 없다.
HEADING_COURSE_MIN_KMH = 2.0   # 이 위로 구르면 GPS 코스가 유효하다고 본다
HEADING_LOCK_N         = 5     # 연속 이만큼 받으면 헤딩을 코스로 ★스냅★ 한다
HEADING_LPF            = 0.35  # IMU 가 없을 때의 폴백(종전 거동)
HEADING_FUSE_K         = 0.5   # [1/s] 자이로 적분을 GPS 코스로 당기는 비율
AIM_CHECK_MAX_DEG      = 8.0   # 스냅 시점의 조준 오차 한계 — 넘으면 즉시 중단

IMU_TOPIC            = '/imu'
IMU_AXIS_MIN_SAMPLES = 20      # 중력축·자이로 바이어스를 재는 최소 표본
IMU_AXIS_G_TOL       = 1.2     # ||a|−9.81| 이 이보다 크면 그 표본은 버린다
IMU_G                = 9.80665
IMU_FRESH_S          = 0.5

# ══════════════════════════════════════════════════════════════════════════════
#  ★라이다 정지는 ★관찰만★ 한다 — 사슬에 끼어들지 않는다★
# ══════════════════════════════════════════════════════════════════════════════
AEB_STOP_TOPIC     = '/aeb_stop'                         # pedal_drive_node → arduino
LIDAR_DIST_TOPIC   = '/cone_lidar_node/obstacle_distance'  # 기록용 (판정 아님)
LIDAR_SIGNAL_TOPIC = '/cone_lidar_node/stop_signal'        # ★사슬 생존 신호★
LIDAR_READY_N       = 10    # 이만큼 연속으로 받아야 '스트리밍 중' 으로 인정
LIDAR_READY_STALE_S = 0.5
AEB_STALE_S         = 1.0   # ★arduino 의 aeb_stale_s 와 같은 값★

# ══════════════════════════════════════════════════════════════════════════════
#  안전 방벽
# ══════════════════════════════════════════════════════════════════════════════
GOAL_MIN_M  = 20.0     # 목표가 이보다 가까우면 시험이 성립하지 않는다
GOAL_MAX_M  = 400.0    # 이보다 멀면 좌표 오타를 의심한다 (자릿수 하나)
CTE_ABORT_M    = 3.0   # 선에서 이만큼 벗어나면 접는다
PSI_ABORT_DEG  = 15.0  # 방위가 이만큼 틀어지면 접는다
PSI_ABORT_HOLD_S = 0.3
#  ★횡가속도 중단★ 요레이트 하나만 보면 ★계전기 진동을 못 잡는다★ — 매 반주기마다
#  부호가 뒤집혀 '연속 초과' 가 성립하지 않기 때문이다(책상 검증에서 확인했다:
#  전달비가 모델만큼 예민하면 pot 이 ±12° 로 튀며 횡가속 9.2 m/s² 가 나는데도
#  순시 요레이트 판정은 한 번도 걸리지 않았다). 그래서 ★|v·r| 을 저역통과시켜★
#  '차가 얼마나 내둘리고 있는가' 를 본다 — 지속 선회와 진동을 한 문턱으로 잡는다.
AY_FILTER_TAU_S = 0.4
AY_ABORT_K      = 1.5    # 필터된 횡가속이 AY_MAX 의 이 배를 넘고
AY_ABORT_HOLD_S = 0.5    # 이만큼 이어지면 접는다
GPS_TIMEOUT_S  = 2.0
RUN_MARGIN_S   = 15.0  # 예상 주행시간에 이만큼 더 주고, 넘으면 폭주로 본다

BRAKE_NONE, BRAKE_SOFT, BRAKE_FULL = 0, 1, 2
BRAKE_KEEPALIVE_S = 0.25   # 물고 있는 동안 재발행 (발행자가 여럿인 토픽이다)

#  ★완전정지 판정은 GPS 와 엔코더를 ★둘 다★ 요구한다★
STOP_KMH_EPS = 0.35
STOP_ENC_EPS = 0.5
STOP_HOLD_S  = 0.7
ENC_SUM_TO_PULSE = 0.5
ENC_MEDIAN_N = 3

BRAKE_MAX_S          = 15.0   # 이 안에 못 서면 굳지 않게 끝낸다(이상 상황)
BRAKE_RELEASE_WAIT_S = 1.5    # 0단을 내고 ★실제로 빠질 시간★
AEB_RELEASE_MAX_S    = 20.0   # AEB 가 물고 있을 때 기다려 주는 상한
DONE_LINGER_S        = 2.0    # 결과를 찍고 이만큼 뒤에 런치를 내린다

#  정지 사유 (결과 표에 그대로 찍는다)
WHY_LIDAR, WHY_GOAL = '라이다 장애물', '목표 좌표 도달(FAIL-SAFE)'

S_WAIT, S_RUN, S_BRAKE, S_RELEASE, S_DONE = (
    'WAIT', 'RUN', 'BRAKE', 'RELEASE', 'DONE')

EARTH_R = 6378137.0


def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


def latlon_to_xy(lat, lon, lat0, lon0):
    x = EARTH_R * math.radians(lon - lon0) * math.cos(math.radians(lat0))
    y = EARTH_R * math.radians(lat - lat0)
    return x, y


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def goal_is_set(lat, lon):
    """0.0 / NaN 을 '미지정' 으로 본다. 적도·본초자오선에서 시험할 일은 없다."""
    return (lat is not None and lon is not None
            and math.isfinite(lat) and math.isfinite(lon)
            and (abs(lat) > 1e-6 or abs(lon) > 1e-6))


class BrakeTestNode(Node):
    """목표 좌표를 향한 직선 질주 → (라이다 | 도달) → 리니어 2단 → 결과 → 종료."""

    def __init__(self):
        super().__init__('braketest_node')

        self.declare_parameter('data_dir', '')
        self.declare_parameter('route', ROUTE)
        #  ★목표 좌표★ 파일 상단 상수를 런치가 덮을 수 있게만 열어 둔다.
        self.declare_parameter('goal_lat', float('nan'))
        self.declare_parameter('goal_lon', float('nan'))
        self.declare_parameter('drive_pulse', DRIVE_PULSE)
        self.declare_parameter('drive_pwm', DRIVE_PWM)
        #  ★[2026-09-13] 2단 속도 루틴★ 상수절 참고. launch_pulse 를 drive_pulse
        #   이하로 주면 루틴 자체가 꺼진다(= 종전의 단일 속도 주행).
        self.declare_parameter('launch_pulse', LAUNCH_PULSE)
        self.declare_parameter('cruise_switch_kmh', CRUISE_SWITCH_KMH)
        self.declare_parameter('launch_max_s', LAUNCH_MAX_S)
        #  ★[2026-09-13 저녁] 기준선 횡 평행이동★ + = 왼쪽. 상수절 참고.
        self.declare_parameter('lateral_offset_m', LATERAL_OFFSET_M)
        self.declare_parameter('cte_abort_m', CTE_ABORT_M)
        #  ★pot 절대 상한★ (종전 이름 그대로 — 뜻만 '횡가속 유도값의 뚜껑' 이 됐다)
        self.declare_parameter('steer_limit_deg', POT_HARD_MAX_DEG)
        self.declare_parameter('t_preview', T_PREVIEW_S)
        self.declare_parameter('ay_max', AY_MAX_MS2)
        self.declare_parameter('steer_trim_deg', STEER_TRIM_POT_DEG)
        self.declare_parameter('require_lidar', True)
        self.declare_parameter('auto_start', True)

        self.data_dir = paths.data_dir(self.get_parameter('data_dir').value or '')
        self.drive_pulse = int(self.get_parameter('drive_pulse').value)
        self.drive_pwm = int(self.get_parameter('drive_pwm').value)
        self.launch_pulse = int(clamp(
            int(self.get_parameter('launch_pulse').value), 0, 15))
        self.cruise_switch_kmh = float(
            self.get_parameter('cruise_switch_kmh').value)
        self.launch_max_s = float(self.get_parameter('launch_max_s').value)
        self.lateral_offset = float(self.get_parameter('lateral_offset_m').value)
        self.cte_abort = float(self.get_parameter('cte_abort_m').value)
        self.pot_hard_max = abs(float(self.get_parameter('steer_limit_deg').value))
        self.t_preview = max(0.5, float(self.get_parameter('t_preview').value))
        self.ay_max = max(0.2, float(self.get_parameter('ay_max').value))
        self.steer_trim = float(self.get_parameter('steer_trim_deg').value)
        self.require_lidar = bool(self.get_parameter('require_lidar').value)
        self.auto_start = bool(self.get_parameter('auto_start').value)

        #  ★순항 지령값은 한 번만 정한다★ — 주행 중에 이 값이 바뀌지는 않는다.
        #  [2026-09-13] 바뀌는 것은 ★출발 구간에서만★ 이고, 그것도 ★한 번 내려오면
        #  끝★ 이다(래치). 아래 drive_value() 하나가 그 판정의 단일 소유자다.
        if self.drive_pwm > 0:
            self.cmd_value = float(clamp(self.drive_pwm, 16, 255))
            self.cmd_kind = f"직접 PWM {int(self.cmd_value)}"
            self.nominal_ms = float('nan')      # PWM 은 속도를 예측할 수 없다
            self.launch_value = self.cmd_value  # ★PWM 모드에는 2단 루틴이 없다★
        else:
            self.cmd_value = float(clamp(self.drive_pulse, 0, 15))
            self.cmd_kind = (f"{int(self.cmd_value)}펄스 "
                             f"≈ {self.cmd_value * KMH_PER_PULSE:.1f} km/h")
            self.nominal_ms = self.cmd_value * MS_PER_PULSE
            self.launch_value = float(max(self.cmd_value, self.launch_pulse))
            if self.launch_value > self.cmd_value:
                self.cmd_kind += (
                    f" (출발 {int(self.launch_value)}펄스 "
                    f"≈ {self.launch_value * KMH_PER_PULSE:.1f} km/h → "
                    f"GPS {self.cruise_switch_kmh:.0f} km/h 에서 내린다)")
        #  ★래치★ True 가 되면 다시 False 가 되지 않는다 — 올렸다 내렸다 하면
        #  그것이 곧 속도 되먹임이고, 이 시험이 재려는 것이 아니다.
        self._cruise_latched = (self.launch_value <= self.cmd_value)
        self._launch_t0 = 0.0     # 출발 구간 시작 시각 (S_RUN 진입에서 잡는다)

        # ── 발행 ──
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.pub_state = self.create_publisher(Bool, '/control_state', 10)
        self.pub_brake = self.create_publisher(Int32, '/brake_level', 10)
        self.pub_event = self.create_publisher(String, '/drive_event', 10)
        self.pub_dstate = self.create_publisher(String, '/drive_state', 10)
        #  ★record 가 파일 이름을 여기서 줍는다★ '<이름>.csv' 여야 붙든다
        #  (record._note_route_cmd). 없으면 ros2bag/★unknown★-<시각>.csv 가 된다.
        self.pub_cmd_name = self.create_publisher(String, '/drive_cmd', 10)
        self.pub_done = self.create_publisher(Bool, '/braketest_done', 10)
        self.pub_diag = self.create_publisher(Float64MultiArray, '/drive_diag', 10)

        # ── 구독 ──
        self.create_subscription(Float64MultiArray, GPS_FUSED_TOPIC, self.cb_gps, 10)
        self.create_subscription(Int32, '/encoder', self.cb_encoder, 10)
        self.create_subscription(Bool, '/vehicle_mode', self.cb_mode, 10)
        self.create_subscription(Bool, '/estop', self.cb_estop, 10)
        #  ★실측 조향각(pot)★ 전달비를 주행 중에 재는 데 쓴다(상단 '전달비를 잰다')
        self.create_subscription(Int32, '/steer_angle_measured',
                                 self.cb_steer_meas, 10)
        self.create_subscription(Bool, '/braketest_go', self.cb_go, 10)
        #  ★관찰 전용★ 제동은 arduino 가 이미 했다 — 여기서는 시점과 거리만 받는다.
        self.create_subscription(Bool, AEB_STOP_TOPIC, self.cb_aeb_stop, 5)
        self.create_subscription(Float32, LIDAR_DIST_TOPIC, self.cb_lidar_dist, 5)
        self.create_subscription(Imu, IMU_TOPIC, self.cb_imu, 20)
        self.create_subscription(Bool, LIDAR_SIGNAL_TOPIC, self.cb_lidar_signal, 5)

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
        self.gps_course = float('nan')
        self.heading = None
        self._heading_locked = False
        self._course_n = 0
        self._aim_err = None          # 헤딩 스냅 순간의 조준 오차 (run_follow 가 검산)
        self._fix_prev_t = 0.0
        self.enc_pulse = 0.0
        self._enc_buf = []
        self.auto_mode = None
        self.estop = False
        self.go = False

        #  라이다 사슬 관찰
        self.aeb_stop = False
        self.lidar_dist = float('nan')
        self.lidar_frames = 0
        self.lidar_t = 0.0
        self.aeb_t = 0.0
        self._lidar_was_ready = False

        #  ★기준선★ (출발 순간에 확정한다)
        self.goal_lat = self.goal_lon = None
        self.goal_src = ''
        self.route_name = ''
        self.sx = self.sy = 0.0          # 출발점 S (로컬 xy)
        self.gx = self.gy = 0.0          # 목표점 G (로컬 xy)
        self.ux, self.uy = 1.0, 0.0      # 진행 단위벡터 û
        self.line_brg = 0.0              # 선의 절대방위 [deg]
        self.dist_total = 0.0            # D = |G − S|
        self.run_max_s = 120.0

        #  조향
        self._steer_out = 0.0
        self._steer_t = 0.0
        self._steer_sat = False
        self.pot_meas = float('nan')      # /steer_angle_measured [pot deg]
        self.pot_meas_t = 0.0
        self.g_est = float('nan')         # 전달비 pot/도로휠 (주행 중 추정)
        self._g_n = 0
        self._ay_f = 0.0                  # 저역통과된 |v·r| [m/s²]
        self._ay_bad_t = 0.0
        self._cte_i = 0.0
        self._cte_i_term = 0.0
        self._cte_prev = 0.0
        self._lp_now = LP_MIN_M

        #  IMU
        self.imu_up = None
        self._acc_sum = [0.0, 0.0, 0.0]
        self._acc_n = 0
        self._gyr_sum = [0.0, 0.0, 0.0]
        self._gyr_n = 0
        self.yaw_bias_dps = 0.0
        self.yaw_rate = 0.0              # 투영·바이어스 보정된 요레이트 [deg/s, + = 좌]
        self.imu_t = 0.0
        self._gyro_t = 0.0

        #  중단 판정 타이머
        self._psi_bad_t = 0.0

        self.brake_now = BRAKE_NONE
        self._brake_out = -1
        self._brake_t = 0.0
        self._last_steer = 0.0

        # 측정
        self.why = ''
        self.own_brake = True
        self.hit_dist = float('nan')
        self.hit_xy = None
        self.hit_s = float('nan')
        self.hit_e = float('nan')
        self.hit_psi = float('nan')
        self.brake_t0 = 0.0
        self.brake_v0 = float('nan')
        self.brake_xy = None
        self._still_t = 0.0
        self._done_t = 0.0
        self._reported = False

        if not self.resolve_goal():
            #  ★목표를 모르면 아예 시작하지 않는다★ 굴린 뒤에 알면 늦다.
            self.state = S_DONE
            self._done_t = time.time()
            self._reported = True

        self.create_timer(1.0 / CONTROL_HZ, self.loop)
        self.event(f"🅑 제동검차 준비 — 지령 {self.cmd_kind}, "
                   f"프리뷰 {self.t_preview:.1f}s, 횡가속 상한 {self.ay_max:.1f}m/s², "
                   f"pot 상한 {self.pot_hard_max:.1f}°")

    # ══════════════════════════════════════════════════════════════════════════
    #  목표 좌표
    # ══════════════════════════════════════════════════════════════════════════
    def resolve_goal(self):
        """목표 좌표를 정한다. ★런치 인자 > 파일 상단 상수 > route CSV 마지막 점★"""
        p_lat = float(self.get_parameter('goal_lat').value)
        p_lon = float(self.get_parameter('goal_lon').value)
        if goal_is_set(p_lat, p_lon):
            self.goal_lat, self.goal_lon, self.goal_src = p_lat, p_lon, '런치 인자'
        elif goal_is_set(GOAL_LAT, GOAL_LON):
            self.goal_lat, self.goal_lon = GOAL_LAT, GOAL_LON
            self.goal_src = '파일 상단 상수'
        else:
            ll = self.goal_from_route()
            if ll is None:
                self.event(
                    "❌ ★목표 좌표가 없다★ — braketest.py 상단의 "
                    "GOAL_LAT / GOAL_LON 을 채우거나, 런치 인자 "
                    "goal_lat:=<위도> goal_lon:=<경도> 로 주거나, "
                    "route:=<파일>.csv 로 경로의 마지막 점을 쓰게 할 것. "
                    "★출발하지 않는다★")
                return False
            self.goal_lat, self.goal_lon = ll
            self.goal_src = f'경로 CSV 마지막 점 [{self.route_name}]'
        if not self.route_name:
            #  record 가 붙들 수 있는 이름을 만들어 준다('.csv' 로 끝나야 한다).
            self.route_name = (f"goal_{self.goal_lat:.7f}_{self.goal_lon:.7f}.csv")
        #  ★'경로 선택' 은 record.py 의 ROUTE_EVENT_HINTS 다★ 문구를 바꾸지 말 것 —
        #  바꾸면 기록 파일명이 조용히 unknown 이 된다.
        self.event(f"📍 경로 선택: 목표 좌표 "
                   f"({self.goal_lat:.7f}, {self.goal_lon:.7f}) — {self.goal_src}")
        return True

    def goal_from_route(self):
        """폴백 — route CSV 의 ★마지막 점★ 을 목표로 쓴다. 없으면 None."""
        import csv as _csv
        name = str(self.get_parameter('route').value or '').strip() or str(ROUTE or '')
        name = name.strip()
        if not name:
            return None
        path = os.path.join(self.data_dir, name)
        if not os.path.isfile(path):
            self.event(f"❌ 경로 파일 없음: {path}")
            return None
        last = None
        try:
            with open(path, 'r', encoding='utf-8') as f:
                for row in _csv.DictReader(f):
                    try:
                        last = (float(row['latitude']), float(row['longitude']))
                    except (KeyError, ValueError, TypeError):
                        continue
        except Exception as e:            # noqa: BLE001
            self.event(f"❌ 경로 읽기 실패: {e}")
            return None
        if last is None:
            self.event(f"❌ 경로에 좌표가 없다: {name}")
            return None
        self.route_name = os.path.basename(name)
        return last

    def build_line(self):
        """★기준선 확정★ S = 지금 차의 위치, G = 목표 좌표. 출발 순간 한 번만 부른다.

        ★S 를 차 자신으로 잡는 이유★ 그래야 e(0) = 0 이 설계상 보장된다. 사람이
        차를 목표 좌표를 바라보는 자리에 세운다는 전제와 합치면 ψ(0) ≈ 0 도 함께
        보장되어, ★초기조건이 최적★ 인 상태로 출발한다.
        """
        self.gx, self.gy = latlon_to_xy(self.goal_lat, self.goal_lon,
                                        self.lat0, self.lon0)
        self.sx, self.sy = self.x, self.y
        dx, dy = self.gx - self.sx, self.gy - self.sy
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return False
        self.ux, self.uy = dx / d, dy / d
        self.dist_total = d
        self.line_brg = math.degrees(math.atan2(dy, dx))
        #  ★폭주 방벽★ 예상 주행시간의 두 배 + 여유. GPS 가 이상해져 s 가 자라지
        #  않아도 이 시간이 지나면 세운다(런치 = 출발이라 사람이 없을 수 있다).
        v = self.nominal_ms if math.isfinite(self.nominal_ms) and self.nominal_ms > 0.5 \
            else 4.0
        self.run_max_s = 2.0 * (d / v) + RUN_MARGIN_S
        return True

    def line_state(self):
        """선 기준 스칼라 셋. s = 진행거리, e = 횡오차(+ = 선의 왼쪽).

        ★[2026-09-13 저녁] lateral_offset 만큼 기준선을 평행이동한다★
        e 에서 오프셋을 빼는 것이 곧 '선을 왼쪽으로 옮기는' 것이다 — 차가 원래 선의
        왼쪽 0.20 m 에 있을 때 e_eff = 0 이 되므로, 제어기는 그 자리를 붙든다.
        ★s 는 건드리지 않는다★ — 평행이동은 진행거리를 바꾸지 않는다(목표 도달
        판정 s ≥ D 도 종전 그대로다). 이 함수 하나만 고치면 조준·적분·중단 방벽·
        진단 출력이 전부 같은 새 기준을 쓴다 — ★영점의 단일 소유자다.★
        """
        dx, dy = self.x - self.sx, self.y - self.sy
        s = dx * self.ux + dy * self.uy
        e = -dx * self.uy + dy * self.ux
        return s, e - self.lateral_offset

    # ══════════════════════════════════════════════════════════════════════════
    #  구독 콜백
    # ══════════════════════════════════════════════════════════════════════════
    def cb_gps(self, msg: Float64MultiArray):
        """/gps_fused 배열. ★인덱스는 gps.py 헤더의 '배열 규약' 이 계약이다★
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
        self.fix_ok = bool(d[4] > 0.5)
        self.gps_kmh = float(d[8]) if math.isfinite(d[8]) else None
        self.gps_course = float(d[9]) if math.isfinite(d[9]) else float('nan')
        if self.lat0 is None:
            self.lat0, self.lon0 = lat, lon
        self.x, self.y = latlon_to_xy(lat, lon, self.lat0, self.lon0)
        self.fix_time = time.time()

        moving = (math.isfinite(self.gps_course) and self.gps_kmh is not None
                  and self.gps_kmh >= HEADING_COURSE_MIN_KMH)
        if not moving or self.heading is None:
            if not moving:
                self._course_n = 0
            return

        if not self._heading_locked:
            #  ★잠글 때까지는 선에서 빌린 방위를 자이로가 들고 간다★ 초반의 거친
            #  코스로 헤딩을 흔들지 않는다. 대신 연속 N 회를 받으면 ★한 번에 스냅★
            #  하고, 그 순간의 차이가 곧 ★조준 오차★ 다(run_follow 가 검산한다).
            self._course_n += 1
            if self._course_n >= HEADING_LOCK_N:
                err = wrap180(self.gps_course - self.line_brg)
                self.heading = self.gps_course
                self._heading_locked = True
                self._aim_err = err
                self.event(f"🧭 헤딩 확정 {self.heading:+.1f}° (GPS 코스 "
                           f"{self._course_n}회 연속) — ★조준 오차 {err:+.2f}°★ "
                           f"(한계 {AIM_CHECK_MAX_DEG:.1f}°)")
            return

        #  ★상보필터★ 헤딩의 주인은 자이로다 — GPS 는 천천히 당길 뿐이다.
        #  ⚠️ 당기는 양은 ★새 fix 가 왔을 때만★ 넣는다 — 같은 값을 20 Hz 로 계속
        #     당기면 3.4 Hz 짜리 낡은 값에 헤딩이 그대로 끌려간다.
        if self.imu_fresh(time.time()):
            self.heading = wrap180(
                self.heading + HEADING_FUSE_K * self._gps_dt()
                * wrap180(self.gps_course - self.heading))
        else:
            self.heading = wrap180(
                self.heading + HEADING_LPF * wrap180(self.gps_course - self.heading))

    def _gps_dt(self):
        """직전 fix 로부터 흐른 시간 [s]. 코스 갱신이 3.4 Hz 라 고정값을 쓰면
        보정량이 실제와 어긋난다 — 실제 간격으로 잰다."""
        t = time.time()
        dt = (t - self._fix_prev_t) if self._fix_prev_t > 0.0 else 0.2
        self._fix_prev_t = t
        return clamp(dt, 0.02, 1.0)

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
        self.aeb_t = time.time()

    def cb_steer_meas(self, msg: Int32):
        """B보드가 실제로 물고 있는 pot 각 [deg]. ★전달비 추정의 절반★ 이다."""
        self.pot_meas = float(msg.data)
        self.pot_meas_t = time.time()

    def cb_lidar_dist(self, msg):
        self.lidar_dist = float(msg.data)

    def cb_lidar_signal(self, msg: Bool):
        """★사슬 생존 신호★ 값은 보지 않는다 — ★온다는 사실★ 만 본다."""
        self.lidar_frames += 1
        self.lidar_t = time.time()

    def cb_imu(self, msg: Imu):
        """요레이트를 ★중력축에 투영하고 바이어스를 뺀 값★ 으로 만든다 [deg/s, + 좌].

        driving.py 4.7절과 같은 방법이다 — 이 차의 IMU 는 z축이 ★58° 기울어★
        있어서 gyro.z 를 그대로 쓰면 요레이트의 53% 만 잡힌다. 레이트 댐핑과
        요레이트 가드가 그만큼 약해지므로 반드시 투영해서 쓴다.

        ★바이어스도 같은 자리에서 잰다★ 출발 직전 정지 구간이 길어서(실측 88~109
        표본) 재기에 좋다. 바이어스는 헤딩 적분과 댐핑 ★양쪽에 동시에★ 실린다 —
        mppi 가 OS1 자이로 −30°/s 바이어스로 인계 직후 좌로 꺾였던 그 성질이다.
        """
        a = msg.linear_acceleration
        w = msg.angular_velocity
        if self.state == S_WAIT:
            #  ★가·감속이 섞인 표본은 버린다★ 중력만 보고 있을 때만 축을 잰다
            mag = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
            if abs(mag - IMU_G) <= IMU_AXIS_G_TOL and self.enc_pulse <= STOP_ENC_EPS:
                self._acc_sum[0] += a.x
                self._acc_sum[1] += a.y
                self._acc_sum[2] += a.z
                self._acc_n += 1
                self._gyr_sum[0] += w.x
                self._gyr_sum[1] += w.y
                self._gyr_sum[2] += w.z
                self._gyr_n += 1
        if self.imu_up is None:
            self.yaw_rate = math.degrees(w.z)          # 아직 z축 단독
        else:
            u = self.imu_up
            self.yaw_rate = math.degrees(
                w.x * u[0] + w.y * u[1] + w.z * u[2]) - self.yaw_bias_dps
        self.imu_t = time.time()

    def solve_imu_axis(self):
        """모아 둔 가속도 평균에서 차량 '위' 단위벡터와 자이로 바이어스를 낸다."""
        if self._acc_n < IMU_AXIS_MIN_SAMPLES:
            return False
        g = [v / self._acc_n for v in self._acc_sum]
        n = math.sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2])
        if n < 1.0:
            return False
        self.imu_up = (g[0] / n, g[1] / n, g[2] / n)
        if self._gyr_n >= IMU_AXIS_MIN_SAMPLES:
            b = [v / self._gyr_n for v in self._gyr_sum]
            self.yaw_bias_dps = math.degrees(
                b[0] * self.imu_up[0] + b[1] * self.imu_up[1] + b[2] * self.imu_up[2])
        return True

    def imu_tilt_deg(self):
        """IMU z축이 차량 수직축에서 얼마나 기울어져 있나 [deg] (진단·로그용)."""
        if self.imu_up is None:
            return float('nan')
        return math.degrees(math.acos(clamp(abs(self.imu_up[2]), -1.0, 1.0)))

    def imu_fresh(self, now):
        return self.imu_t > 0.0 and (now - self.imu_t) <= IMU_FRESH_S

    def integrate_gyro(self, now):
        """헤딩을 자이로로 20 Hz 전진시킨다. ★상보필터의 빠른 쪽★

        ★출발 순간부터 돈다★ 헤딩의 씨앗(선의 방위)이 이미 있기 때문이다.
        느린 쪽(GPS 코스로 당기기)은 cb_gps 가 새 fix 마다 넣는다.
        """
        prev = self._gyro_t
        self._gyro_t = now
        if self.heading is None or not self.imu_fresh(now) or prev <= 0.0:
            return
        dt = now - prev
        if dt <= 0.0 or dt > 0.5:
            return
        self.heading = wrap180(self.heading + self.yaw_rate * dt)

    def lidar_ready(self, now):
        """라이다 정지 사슬이 ★실제로 돌고 있는가★ — 두 가지를 따로 본다.
          ① 인지  : stop_signal 이 프레임마다 오는가 (ouster + cone_lidar 생존)
          ② 판단자: /aeb_stop 이 신선한가 (pedal_drive_node 생존 — arduino 를 문다)
        ①만 있으면 아무리 봐도 안 서고, ②만 있으면 눈이 없다.
        """
        chain = (self.lidar_frames >= LIDAR_READY_N
                 and (now - self.lidar_t) <= LIDAR_READY_STALE_S)
        judge = self.aeb_t > 0.0 and (now - self.aeb_t) <= AEB_STALE_S
        return chain, judge

    def lidar_wait_reason(self, now):
        """왜 아직 못 나가는지 한 줄로. ★진행이 보이게 프레임 수를 함께 찍는다★"""
        chain, judge = self.lidar_ready(now)
        if not chain and self.lidar_frames == 0:
            return ("⏳ ★라이다 대기★ — cone_lidar_node 의 stop_signal 이 아직 한 번도 "
                    "오지 않았다. OS1-32 드라이버는 첫 패킷까지 수십 초가 걸린다. "
                    "안 뜨면 : ip -br addr show eno1 (192.168.6.100/24) / "
                    "ping -c2 192.168.6.11")
        if not chain and (now - self.lidar_t) > LIDAR_READY_STALE_S:
            return (f"⏳ ★라이다 대기★ — stop_signal 이 {now - self.lidar_t:.1f}s 째 "
                    f"끊겼다 (누적 {self.lidar_frames}프레임). 점군이 끊긴다")
        if not chain:
            return (f"⏳ ★라이다 대기★ — 스트리밍 시작됨, 안정화 확인 중 "
                    f"{self.lidar_frames}/{LIDAR_READY_N} 프레임")
        if not judge:
            return ("⏳ ★라이다 대기★ — 인지는 살아 있는데 pedal_drive_node 의 "
                    "/aeb_stop 이 안 온다. 그 노드가 떠 있는지 확인 "
                    "(use_lidar:=true 인가)")
        return ''

    # ══════════════════════════════════════════════════════════════════════════
    #  출력
    # ══════════════════════════════════════════════════════════════════════════
    def event(self, text):
        self.get_logger().info(text)
        self.pub_event.publish(String(data=text))

    def throttle(self, text, period=2.0):
        t = time.time()
        if t - getattr(self, '_thr_t', 0.0) >= period:
            self._thr_t = t
            self.event(text)

    def send(self, value, steer_deg, control):
        """★값을 그대로 낸다★ 펄스든 직접 PWM 이든 arduino 가 대역으로 가른다.

        ★제동 중에도 이 값을 0 으로 덮지 않는다★ arduino 가 브레이크>0 이면 A보드
        REF 를 0 으로 만들기 때문에(compose (4)) 우리가 또 0 을 낼 이유가 없고,
        내면 해제 순간 0→목표로 두 번 전이한다.
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
        #  record 가 켜지도록 DRIVE_* 이름을 쓴다(그쪽은 DRIVE_* 일 때만 기록한다).
        name = {S_WAIT: 'IDLE', S_RUN: 'DRIVE_RUN', S_BRAKE: 'DRIVE_RUN',
                S_RELEASE: 'DRIVE_DONE', S_DONE: 'DRIVE_DONE'}[self.state]
        self.pub_dstate.publish(String(data=name))

    def enter(self, new_state, msg=''):
        self.state = new_state
        self.state_t0 = time.time()
        if msg:
            self.event(msg)

    def pub_diag_now(self, cte, psi_err, d2goal, lp):
        """/drive_diag 를 record.py 의 열 순서대로 낸다.

        ★앞 18개만 채운다★ 뒤쪽(goal_phase·cb_state·lidar_zone 등)은 driving.py
        전용 상태라 이 노드에 없다. record._array 가 모자란 만큼 빈칸으로 채우므로
        ★열이 밀리지 않는다★ — 없는 값을 0 으로 채워 '있는 척' 하는 것보다 낫다.
        ※ target_idx 는 웨이포인트가 없어 NaN, target_dist_m 은 ★프리뷰 거리★ 다
          (프리뷰 조준에서는 그것이 실제로 겨누는 점까지의 거리다).
        """
        n = float('nan')
        self.pub_diag.publish(Float64MultiArray(data=[
            float(cte),                                   # cte_m
            float(psi_err),                               # heading_err_deg
            n,                                            # target_idx (WP 없음)
            float(lp),                                    # target_dist_m = L_p
            float(d2goal),                                # goal_dist_m
            float(self.gps_course),                       # gps_course_deg
            n,                                            # fuse_corr_deg
            float(self.yaw_rate),                         # gyro_z_dps (★투영·보정★)
            float(self.brake_now),                        # brake_latched
            float(self.heading if self.heading is not None else n),  # head_init_deg
            n, n, n,                                      # head_sigma/resid/dist
            float(self.cmd_value),                        # ref_pulse
            float(self.cmd_value),                        # out_pulse (감속 로직 없음)
            float(self.enc_pulse),                        # meas_pulse
            float(self._cte_i),                           # cte_integral [m·s]
            float(self._cte_i_term),                      # cte_i_term_deg [도로휠]
        ]))

    def drive_value(self, now):
        """지금 A보드로 낼 구동 지령. ★출발 15펄스 → 순항 10펄스★ [2026-09-13]

        ★판정의 단일 소유자다★ — 이 함수 밖에서 cmd_value/launch_value 를 직접
        고르지 않는다. 내려가는 조건이 셋이지만 ★전부 '내리는 방향'★ 이라,
        어느 것이 먼저 걸려도 위험해지지 않는다:

          ⓐ ★GPS 속도 ≥ cruise_switch_kmh★  — 사용자 지시의 본선
          ⓑ 엔코더 ≥ LAUNCH_SWITCH_ENC_PULSE — GPS 가 안 올 때의 폴백. 엔코더는
            기동 허수로 ★위로★ 튀므로(실측 중앙 16, 최대 34) 조금 일찍 내릴 뿐이다.
          ⓒ 시간 ≥ launch_max_s              — 15펄스로 무한정 달리지 않게 하는 줄

        ★한 번 내려오면 다시 올리지 않는다★(_cruise_latched). 올렸다 내렸다 하면
        그것이 곧 속도 되먹임이고, 제동거리를 재려는 이 시험이 재려는 것이 아니다.
        """
        if self._cruise_latched:
            return self.cmd_value
        why = None
        if self.gps_kmh is not None and self.gps_kmh >= self.cruise_switch_kmh:
            why = f"GPS {self.gps_kmh:.1f} km/h ≥ {self.cruise_switch_kmh:.0f}"
        elif (self._launch_t0 > 0.0
              and (now - self._launch_t0) >= LAUNCH_ENC_BLANK_S
              and self.enc_pulse >= LAUNCH_SWITCH_ENC_PULSE):
            #  ★기동 블랭킹을 지난 뒤에만 엔코더를 믿는다★ 출발 직후의 허수 펄스
            #  (실측 중앙 16 · 최대 34)가 이 문턱을 그냥 넘긴다 — 상수절 참고.
            why = (f"엔코더 {self.enc_pulse:.1f}펄스 ≥ "
                   f"{LAUNCH_SWITCH_ENC_PULSE:.0f} (GPS 폴백, "
                   f"기동 블랭킹 {LAUNCH_ENC_BLANK_S:.0f}s 경과)")
        elif self._launch_t0 > 0.0 and (now - self._launch_t0) >= self.launch_max_s:
            why = f"출발 {now - self._launch_t0:.1f}s 경과 ≥ {self.launch_max_s:.0f}s"
        if why is None:
            return self.launch_value
        self._cruise_latched = True
        self.event(f"⏬ 순항 전환 — {why} → "
                   f"{int(self.launch_value)}펄스에서 ★{int(self.cmd_value)}펄스"
                   f"({self.cmd_value * KMH_PER_PULSE:.1f} km/h)★ 로 내린다. "
                   f"여기서부터가 제동 대상 속도다")
        return self.cmd_value

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
    #  ★★ 조향 — 프리뷰 추종 (헤더 '멀리 본다' 절이 근거 전부다) ★★
    # ══════════════════════════════════════════════════════════════════════════
    def preview_len(self, v_ms):
        """★L_p = T_PREVIEW × v★ 정속 프리뷰. 속도가 변해도 안정도가 같다."""
        v = v_ms if (v_ms is not None and math.isfinite(v_ms)) else 0.0
        return clamp(self.t_preview * abs(v), LP_MIN_M, LP_MAX_M)

    def aim_deg(self, e, psi_deg, lp):
        """선 위 L_p 앞의 점을 겨눈 도로휠각 [deg]. ★− 좌 / + 우★

            e_p = e + L_p·sin ψ        δ = atan(2·L·e_p / L_p²)

        ★부호★ e + = 차가 선의 왼쪽 → 오른쪽으로 꺾어야 한다 → +.
                psi + = 차가 선방위보다 왼쪽을 향한다 → 오른쪽 → +.
        둘 다 뒤집지 않고 그대로 쓴다(steer_command 가 부호를 다시 뒤집지 않는다).
        """
        if not math.isfinite(e):
            return 0.0
        e_p = e + lp * math.sin(math.radians(psi_deg))
        return math.degrees(math.atan2(2.0 * WHEELBASE_M * e_p, lp * lp))

    def rate_damp_deg(self, lp, now):
        """★요레이트 댐핑★ 지금 실제로 돌고 있는 속도를 조향에서 뺀다 [도로휠 deg].

        ★부호★ yaw_rate + = 좌회전 중 → 오른쪽으로 되돌려야 한다 → + (그대로 더한다).
        ★이득을 프리뷰에 비례시킨다★ K_r = (2L/L_p)·RATE_T_S — T_PREVIEW 를 바꿔도
        감쇠비가 그대로 따라오게 하려는 것이다. 자이로는 ★지연이 없어서★ 오차가 0 을
        지나기 전에 미리 힘을 뺀다 — 포화가 반대편으로 넘어가는 것을 막는 유일한 항이다.
        """
        if not self.imu_fresh(now):
            return 0.0
        return (2.0 * WHEELBASE_M / lp) * RATE_T_S * self.yaw_rate

    def plant_pot(self, road_deg, v_ms):
        """도로휠각 → pot 지령의 크기 [deg]. ★[2026-09-13] 속도를 다시 자른다★

        pot = 1.26·δ + 5.17·v_eff²·tan δ / L,  v_eff = min(v, UNDERSTEER_V_CLAMP_MS)
        단 ★PLANT_GAIN_MIN·δ 를 하한★ 으로 둔다.

        ★[2026-09-12] 에 이 클램프를 없앴다가 [2026-09-13] 에 되살렸다★ — 없앤
        근거였던 '회귀 실측 5.89' 가 ★고속 직선의 1~2° 표본★ 이라 조향 불감대
        아티팩트였기 때문이다(상수절 UNDERSTEER_V_CLAMP_MS 주석에 전말). 클램프
        없는 식은 v=8.3 m/s 에서 도로휠 1.62° 요구에 pot 10.1° 를 내는데, 이 노드가
        같은 구간에서 스스로 잰 전달비는 g_est 2.7 이라 실제로 도로휠 3.7° 가 된다 —
        요구의 2.3배. 그 결과가 ★조향 부호반전 129.9회/분★ 이었다.
        """
        d = abs(road_deg)
        if d < 1e-9:
            return 0.0
        v = abs(v_ms) if (v_ms is not None and math.isfinite(v_ms)) else 0.0
        if UNDERSTEER_V_CLAMP_MS > 0.0:
            v = min(v, UNDERSTEER_V_CLAMP_MS)
        pot_model = (STEER_PLANT_GAIN * d
                     + STEER_UNDERSTEER * v * v * math.tan(math.radians(d))
                     / WHEELBASE_M)
        return max(pot_model, PLANT_GAIN_MIN * d)

    def update_plant_estimate(self, v_ms, now):
        """★전달비를 주행 중에 잰다★ G = |pot실측| / |atan(r·L/v)|  (상단 절)

        ★조건을 좁게 건다★ 빠르고 확실히 돌고 있을 때의 표본만 쓴다 — 불감대
        근처의 작은 지령이나 정지 근처의 작은 요레이트로 나눈 비율은 잡음이다.
        ★부호가 맞을 때만★ 센다(pot + = 우조향 → 요레이트 −). 안 맞으면 서보가
        아직 따라오는 중이거나 외란이라, 그 표본은 전달비를 말해 주지 않는다.
        """
        if not (self.imu_fresh(now) and math.isfinite(self.pot_meas)
                and self.pot_meas_t > 0.0 and (now - self.pot_meas_t) <= 1.0):
            return
        v = abs(v_ms) if (v_ms is not None and math.isfinite(v_ms)) else 0.0
        if v < PLANT_EST_MIN_V_MS or abs(self.pot_meas) < PLANT_EST_MIN_POT:
            return
        r_dps = self.yaw_rate
        if abs(r_dps) < PLANT_EST_MIN_DPS or self.pot_meas * r_dps >= 0.0:
            return
        road = math.degrees(math.atan(math.radians(abs(r_dps)) * WHEELBASE_M / v))
        if road < 1e-3:
            return
        g = clamp(abs(self.pot_meas) / road, *PLANT_EST_CLAMP)
        if not math.isfinite(self.g_est):
            self.g_est = g
        else:
            self.g_est += PLANT_EST_ALPHA * (g - self.g_est)
        self._g_n += 1

    def pot_limit(self, v_ms, now):
        """★상한을 '각도' 가 아니라 '횡가속도' 로 정한다★ [pot deg]

        δ_max = atan(AY_MAX·L / v²) 를 pot 으로 환산한 값과 pot_hard_max 중 작은 것.
        자이로가 없으면 요레이트 가드가 못 도니 POT_NOIMU_MAX_DEG 로 더 조인다.
        """
        known = v_ms is not None and math.isfinite(v_ms) and abs(v_ms) > 1e-6
        v = abs(v_ms) if known else 0.0
        v = max(v, 2.0)                       # 저속에서 δ_max 가 발산하는 것만 막는다
        d_max = math.degrees(math.atan(self.ay_max * WHEELBASE_M / (v * v)))
        lim = min(self.plant_pot(d_max, v), self.pot_hard_max, float(STEER_MAX_DEG))
        #  ★잰 전달비는 조이는 쪽으로만 쓴다★ 차가 모델보다 예민한 것으로 드러나면
        #  같은 pot 이 더 많이 꺾인다는 뜻이므로 권한을 줄인다. 둔한 쪽으로 드러나면
        #  아무것도 하지 않는다(min). 추정이 튀어도 위험해지지 않는 유일한 쓰임새다.
        if math.isfinite(self.g_est) and self._g_n >= PLANT_EST_MIN_N:
            lim = min(lim, max(self.g_est * d_max, POT_LIMIT_FLOOR_DEG))
        else:
            #  ★아직 못 쟀다 — 모르면 낮게 본다★ (상단 POT_START_MAX_DEG)
            lim = min(lim, POT_START_MAX_DEG)
        if not self.imu_fresh(now):
            lim = min(lim, POT_NOIMU_MAX_DEG)
        #  ★[2026-09-13] 속도를 모르면 권한을 주지 않는다★ 상수절 POT_NOSPEED 참고.
        #  v 를 모르면 위 하한 2.0 m/s 때문에 δ_max 가 32° 로 열려 상한이 최대가
        #  된다 — 2026-09-13 로그에서 GPS 가 게이트에 막힌 뒤 5.7° → ★12.0°★ 가
        #  그것이다. '모르면 낮게 본다' 를 여기에도 적용한다.
        if not known:
            lim = min(lim, POT_NOSPEED_MAX_DEG)
        return lim

    def yaw_rate_max(self, v_ms):
        """지금 속도에서 AY_MAX 를 내는 요레이트 [deg/s]."""
        v = abs(v_ms) if (v_ms is not None and math.isfinite(v_ms)) else 0.0
        v = max(v, 2.0)
        return min(math.degrees(self.ay_max / v), 40.0)

    def yaw_guard(self, pot, v_ms, now):
        """★최종 방벽★ 실제로 도는 속도가 상한을 넘으면 그 방향 지령을 깎는다.

        ★전달비를 몰라도 횡가속도가 상한을 넘지 못한다★ 는 것이 이 함수의 전부다.
        모델이 실측보다 예민한 쪽으로 틀려 있어도(= 같은 pot 이 더 많이 꺾여도)
        자이로는 ★실제로 돌고 있는 값★ 을 지연 없이 알려 준다.

        ★부호★ pot − = 좌조향, yaw_rate + = 좌회전. 즉 ★pot·r < 0 이면 지령이
        지금의 회전을 더 키우는 방향★ 이다 — 그때만 깎는다. 되돌리는 쪽(카운터
        스티어)은 절대 막지 않는다.
        """
        if not self.imu_fresh(now):
            return pot
        r = abs(self.yaw_rate)
        r_max = self.yaw_rate_max(v_ms)
        if r <= r_max or r < 1e-6:
            return pot
        if pot * self.yaw_rate < 0.0:
            return pot * (r_max / r)
        return pot

    def apply_cte_integral(self, road_deg, cte, saturated):
        """도로휠각에 ★CTE 적분항★ 을 더한다 (Ki 단독, P·D 없음).

        ★이 항의 일은 수렴이 아니라 '조향 영점 트림' 이다★ (상단 상수절 참고).
        driving.apply_cte_integral 의 와인드업 방어를 같은 순서로 옮겼다:
          ① 부호반전 → 소프트 감쇠  ② 불감대 안 → 감쇠 / 밖 → 적분
          ③ 적분값 클램프  ④ 기여 클램프  ⑤ 차가 안 구르면 동결
          ⑥ ★포화 중 같은 방향 적분 금지★ (이 시험은 상한이 낮아 실질 포화가 있다)
        """
        if not math.isfinite(cte) or CTE_KI <= 0.0:
            self._cte_i_term = 0.0
            return road_deg

        dt = 1.0 / CONTROL_HZ
        moving = self.enc_pulse > CTE_I_MIN_PULSE

        if (cte * self._cte_prev) < 0.0 \
                and abs(cte) > CTE_I_DEADBAND_M \
                and abs(self._cte_prev) > CTE_I_DEADBAND_M:
            self._cte_i *= CTE_I_FLIP_SCALE
        self._cte_prev = cte

        blocked = saturated and (cte * self._cte_i >= 0.0)
        if moving:
            if abs(cte) < CTE_I_DEADBAND_M:
                self._cte_i -= self._cte_i * min(1.0, CTE_I_DECAY_PER_S * dt)
            elif not blocked:
                self._cte_i += cte * dt
            self._cte_i = clamp(self._cte_i, -CTE_I_CLAMP, CTE_I_CLAMP)

        i_term = clamp(CTE_KI * self._cte_i, -CTE_I_MAX_DEG, CTE_I_MAX_DEG)
        self._cte_i_term = i_term
        return road_deg + i_term

    def reset_cte_integral(self):
        self._cte_i = 0.0
        self._cte_i_term = 0.0
        self._cte_prev = 0.0

    def smooth_steer(self, pot, now, lim):
        """슬루 제한. ★저역통과는 쓰지 않는다★ (STEER_LPF = 1.0)

        기준이 선이라 지령에 계단이 없다 — 거를 것이 없고, 필터는 곧 지연이다.
        슬루는 평활이 아니라 ★안전용★ 이다(지령이 한 틱에 튀는 것을 막는다).
        """
        dt = (now - self._steer_t) if self._steer_t > 0.0 else (1.0 / CONTROL_HZ)
        dt = clamp(dt, 1e-3, 0.2)
        self._steer_t = now
        want = self._steer_out + STEER_LPF * (pot - self._steer_out)
        slew = STEER_SLEW_DEG_S * dt
        self._steer_out += clamp(want - self._steer_out, -slew, slew)
        self._steer_out = clamp(self._steer_out, -lim, lim)
        return self._steer_out

    # ══════════════════════════════════════════════════════════════════════════
    #  제어 루프
    # ══════════════════════════════════════════════════════════════════════════
    def loop(self):
        self.publish_state()
        self.publish_brake()          # 물고 있는 동안 재확인 (발행자가 여럿이다)
        now = time.time()
        self.integrate_gyro(now)

        if self.state == S_DONE:
            self.send(0, self._last_steer, control=True)
            if not self._reported:
                return
            if self._done_t and (now - self._done_t) >= DONE_LINGER_S:
                #  ★런치를 내리는 신호★ braketest.launch.py 가 이 노드의 종료를
                #  받아 전체를 내린다(OnProcessExit → Shutdown).
                self.pub_done.publish(Bool(data=True))
                self.event("🛑 제동검차 종료 — 런치를 내린다")
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
        #  ★신선도 = 상대의 생존★ 출발할 때 살아 있던 사슬이 주행 중에 끊기면,
        #  이 차는 ★아무도 안 보는 채로 32 km/h 로 계속 간다★. 정지 트리거가
        #  라이다인 시험에서 그 상태로 달릴 이유가 없다 — 그 자리에서 접는다.
        #  (여기서 보는 것은 장애물 유무가 아니라 ★판단자의 생사★ 다.)
        if self._lidar_was_ready and self.state == S_RUN:
            chain, judge = self.lidar_ready(now)
            if not (chain and judge):
                self.set_brake(BRAKE_FULL)
                self.publish_brake(force=True)
                self.send(0, self._last_steer, control=True)
                self.finish(
                    "★라이다 사슬 두절★ — "
                    + ("인지 끊김" if not chain else "판단자(/aeb_stop) 끊김")
                    + f" (stop_signal {now - self.lidar_t:.1f}s 전, "
                      f"/aeb_stop {now - self.aeb_t:.1f}s 전)")
                return

        if self.state == S_WAIT:
            self.run_wait(now)
        elif self.state == S_RUN:
            self.run_follow(now)
        elif self.state == S_BRAKE:
            self.run_brake(now)
        elif self.state == S_RELEASE:
            self.run_release(now)

    # ══════════════════════════════════════════════════════════════════════════
    #  출발 게이트 — ★통과하면 그 자리에서 지정속도로 굴러간다★
    # ══════════════════════════════════════════════════════════════════════════
    def run_wait(self, now):
        self.send(0, 0.0, control=True)
        if not (self.auto_start or self.go):
            self.throttle("⏸️ 출발 대기 — `ros2 topic pub -1 /braketest_go "
                          "std_msgs/msg/Bool '{data: true}'` 로 시작")
            return
        if not self.fix_ok:
            self.throttle(
                f"⏸️ GPS 품질 대기 — {Q_LABEL.get(self.gps_quality, '?')} "
                f"(σ={self.gps_sigma:.2f}m). gps 노드의 min_quality 문턱 미달이다 — "
                f"낮추려면 런치 인자 min_quality:=1")
            return
        if self.lat0 is None or not self.build_line():
            self.throttle("⏸️ GPS 원점 대기")
            return

        # ── 목표까지의 거리가 시험으로 성립하는가 ──
        d = self.dist_total
        if d < GOAL_MIN_M or d > GOAL_MAX_M:
            self.throttle(
                f"⛔ ★목표 좌표까지 {d:.1f}m★ — 허용 {GOAL_MIN_M:.0f}~{GOAL_MAX_M:.0f}m "
                f"밖이다. 좌표를 확인할 것(자릿수 하나가 틀리면 딱 이렇게 된다). "
                f"목표 = ({self.goal_lat:.7f}, {self.goal_lon:.7f}) [{self.goal_src}]",
                period=5.0)
            return

        # ══════════════════════════════════════════════════════════════════════
        #  ★라이다가 살아난 뒤에 출발한다★ — 사람이 손쓸 게이트를 다 지난 뒤에 본다
        # ══════════════════════════════════════════════════════════════════════
        #  ★이 시험에서 라이다는 선택이 아니라 시험 대상 그 자체다★ 라이다 없이
        #  출발하면 목표 좌표까지 그냥 달릴 뿐이고, 그것이 2026-09-09 밤에 실제로
        #  벌어진 일이다.
        if self.require_lidar:
            why = self.lidar_wait_reason(now)
            if why:
                self.throttle(why, period=3.0)
                return

        # ── 출발 ──
        self.heading = self.line_brg        # ★씨앗★ 차는 목표를 바라보고 서 있다
        self._heading_locked = False
        self._course_n = 0
        self._aim_err = None
        self._gyro_t = 0.0
        self._steer_out = 0.0
        self._steer_t = 0.0
        self._steer_sat = False
        self._psi_bad_t = 0.0
        self._ay_bad_t = 0.0
        self._ay_f = 0.0
        self.reset_cte_integral()
        #  ★기록 파일명을 여기서 못 박는다★ resolve_goal 의 이벤트는 __init__ 에서
        #  나가므로 record 가 아직 구독을 붙이기 전일 수 있다. 여기는 /drive_state 가
        #  DRIVE_RUN 이 되기 ★한 틱 전★ 이라 세션이 열릴 때 이름이 이미 잡혀 있다.
        self.pub_cmd_name.publish(String(data=self.route_name))
        #  ★IMU 축·자이로 바이어스를 여기서 확정한다★ 바로 위 정지 대기 동안
        #  cb_imu 가 표본을 모아 두었다 — 출발 직전이 ★차가 확실히 서 있던 마지막
        #  순간★ 이라 재기에 가장 좋다.
        if self.solve_imu_axis():
            self.event(f"🧭 IMU 축 확정 — 표본 {self._acc_n}개, "
                       f"z축 기울기 {self.imu_tilt_deg():.1f}°, "
                       f"자이로 바이어스 {self.yaw_bias_dps:+.3f}°/s "
                       f"(표본 {self._gyr_n}개) — 요레이트를 이 축에 투영하고 뺀다")
        else:
            self.event(f"⚠️ IMU 축을 못 쟀다 (표본 {self._acc_n}개 < "
                       f"{IMU_AXIS_MIN_SAMPLES}) — ★자이로 z축 단독으로 떨어진다★. "
                       f"이 차는 z축이 58° 기울어 있어 요레이트가 절반쯤으로 잡히고, "
                       f"요레이트 가드도 그만큼 약해진다. /imu 가 오는지 확인할 것")
        self._lidar_was_ready = self.require_lidar
        lid = (f"라이다 사슬 살아 있음({self.lidar_frames}프레임)"
               if self.require_lidar else "★라이다 없음 — 목표 도달 정지만★")
        v = self.nominal_ms
        note = ""
        if math.isfinite(v) and v > 0:
            #  2단 실측 하한 2.2 로 잡은 보수적 값 + 판정·행정 지연 0.30s
            d_stop = v * 0.30 + v * v / (2.0 * 2.2)
            note = (f" 이 속도의 2단 정지거리 ≈ {d_stop:.1f}m — "
                    f"★목표 좌표 뒤로 그만큼이 비어 있어야 한다★.")
        self._launch_t0 = time.time()      # ★[2026-09-13] 출발 구간 시계 시작★
        self.enter(S_RUN,
                   f"▶ ★질주 시작★ 목표까지 {d:.1f}m, 선 방위 {self.line_brg:+.1f}° "
                   f"(이 방위를 출발 헤딩으로 쓴다 — 1초 뒤 GPS 코스로 검산한다). "
                   f"지령 ★{self.cmd_kind}★.{note} {lid}")

    # ══════════════════════════════════════════════════════════════════════════
    #  주행 — 직선 한 줄을 프리뷰로 추종한다
    # ══════════════════════════════════════════════════════════════════════════
    def run_follow(self, now):
        if self.heading is None:
            self.send(0, 0.0, control=True)
            return

        s, e = self.line_state()
        d2goal = self.dist_total - s
        psi = wrap180(self.heading - self.line_brg)
        v = self.speed_ms()
        vv = v if (v is not None and math.isfinite(v)) else 0.0

        # ══════════════════════════════════════════════════════════════════════
        #  ① 라이다 사슬이 세웠다 — ★우리도 2단을 낸다★ [2026-09-13 변경]
        # ══════════════════════════════════════════════════════════════════════
        #  ★종전★ own_brake=False 로 두고 '계측 시점만 잡았다' — arduino 가
        #  /aeb_stop 으로 이미 물었으니 발행자만 늘 뿐이라는 판단이었다.
        #  ★그런데 2026-09-13 로그에서 `/brake_level` 이 ★전 구간 0★ 이었다.★
        #  arduino 의 (4) 정상자율 분기에는
        #       pulse = 0 if brake > 0 else self.cmd_pulse
        #  라는 ★구동 무력화★ 가 있는데, 그 brake 는 `/brake_level` 이다.
        #  AEB 분기 (1-1) 이 따로 구동을 끊어 주긴 하지만, 그것은 ★/aeb_stop 이
        #  신선할 때만★ 이다(aeb_stale_s 1.0 s). 판단 노드가 한 틱이라도 끊기면
        #  그 순간 (4) 로 떨어지고, `/brake_level`=0 이면 ★구동이 되살아난다★ —
        #  리니어는 물려 있는데 인휠이 다시 미는, 제일 나쁜 조합이다.
        #  → ★둘 다 건다.★ 같은 값을 두 경로로 내는 것은 발행자 경합이 아니다
        #    (arduino 가 max 로 합치지 않고 각 분기에서 그대로 쓴다).
        if self.aeb_stop:
            self.begin_brake(WHY_LIDAR, now, s, e, psi)
            return

        # ══════════════════════════════════════════════════════════════════════
        #  ② ★FAIL-SAFE★ 목표 좌표 도달 — 그 즉시 리니어 2단 (사용자 지시)
        # ══════════════════════════════════════════════════════════════════════
        #  ★진행거리 투영 s 로 판정한다★ 반경으로 보면 한 틱에 0.44m 를 가는
        #  이 속도에서 놓칠 여지가 남지만, s 는 ★단조증가 스칼라 하나★ 라
        #  문턱을 반드시 넘는다. 횡오차가 남아 있어도 판정이 흔들리지 않는다.
        if s >= self.dist_total:
            self.begin_brake(WHY_GOAL, now, s, e, psi)
            return

        # ── 조준 검산 : "목표를 바라보고 서 있다" 는 전제를 GPS 코스로 확인한다 ──
        if self._aim_err is not None:
            err, self._aim_err = self._aim_err, None
            if abs(err) > AIM_CHECK_MAX_DEG:
                self.set_brake(BRAKE_FULL)
                self.publish_brake(force=True)
                self.send(0, self._last_steer, control=True)
                self.finish(
                    f"★조준 오차 {err:+.1f}°★ (한계 {AIM_CHECK_MAX_DEG:.1f}°) — "
                    f"차가 목표 좌표를 바라보고 있지 않다. 차를 다시 세우거나 "
                    f"목표 좌표를 확인할 것")
                return

        # ── 중단 방벽 ──
        if math.isfinite(e) and abs(e) > self.cte_abort:
            self.set_brake(BRAKE_FULL)
            self.publish_brake(force=True)
            self.send(0, self._last_steer, control=True)
            self.finish(f"선에서 {e:+.2f}m 이탈 (한계 {self.cte_abort:.1f}m)")
            return
        if abs(psi) > PSI_ABORT_DEG:
            if self._psi_bad_t == 0.0:
                self._psi_bad_t = now
            elif now - self._psi_bad_t >= PSI_ABORT_HOLD_S:
                self.set_brake(BRAKE_FULL)
                self.publish_brake(force=True)
                self.send(0, self._last_steer, control=True)
                self.finish(f"방위 {psi:+.1f}° 틀어짐 (한계 {PSI_ABORT_DEG:.0f}°)")
                return
        else:
            self._psi_bad_t = 0.0
        #  ★필터된 횡가속도★ 지속 선회와 계전기 진동을 ★한 문턱으로★ 잡는다
        #  (순시 요레이트로는 진동을 못 잡는다 — 상단 AY_ABORT_K 주석의 검증 참고)
        if self.imu_fresh(now):
            ay = abs(vv * math.radians(self.yaw_rate))
            a = (1.0 / CONTROL_HZ) / (AY_FILTER_TAU_S + 1.0 / CONTROL_HZ)
            self._ay_f += a * (ay - self._ay_f)
            if self._ay_f > AY_ABORT_K * self.ay_max:
                if self._ay_bad_t == 0.0:
                    self._ay_bad_t = now
                elif now - self._ay_bad_t >= AY_ABORT_HOLD_S:
                    self.set_brake(BRAKE_FULL)
                    self.publish_brake(force=True)
                    self.send(0, self._last_steer, control=True)
                    self.finish(
                        f"★횡가속 {self._ay_f:.2f} m/s²★ (상한 {self.ay_max:.1f} 의 "
                        f"{AY_ABORT_K:.1f}배 초과가 {AY_ABORT_HOLD_S:.1f}s 이어졌다) — "
                        f"차가 내둘리고 있다. 전달비 추정 "
                        f"{self.g_est:.1f}(표본 {self._g_n}) · 조향 영점을 볼 것")
                    return
            else:
                self._ay_bad_t = 0.0
        if now - self.state_t0 > self.run_max_s:
            self.set_brake(BRAKE_FULL)
            self.publish_brake(force=True)
            self.send(0, self._last_steer, control=True)
            self.finish(f"주행시간 {now - self.state_t0:.0f}s 초과 "
                        f"(상한 {self.run_max_s:.0f}s) — 진행거리 {s:.1f}/"
                        f"{self.dist_total:.1f}m. GPS 나 구동을 의심할 것")
            return

        # ══════════════════════════════════════════════════════════════════════
        #  ★조향★ 프리뷰 조준 + 요레이트 댐핑 + 영점 트림 → pot → 가드 → 슬루
        # ══════════════════════════════════════════════════════════════════════
        self.update_plant_estimate(vv, now)
        lp = self.preview_len(vv if vv > 0.1 else self.nominal_ms)
        self._lp_now = lp
        aim = self.aim_deg(e, psi, lp)
        damp = self.rate_damp_deg(lp, now)
        road = self.apply_cte_integral(aim + damp, e, self._steer_sat)
        lim = self.pot_limit(vv, now)
        pot = math.copysign(self.plant_pot(road, vv), road) + self.steer_trim
        pot = clamp(pot, -lim, lim)
        pot = self.yaw_guard(pot, vv, now)
        #  다음 틱의 적분이 볼 포화 여부
        self._steer_sat = abs(pot) >= lim - 1e-6
        steer = self.smooth_steer(pot, now, lim)

        self.pub_diag_now(e, psi, d2goal, lp)
        self.send(self.drive_value(now), steer, control=True)
        self.throttle(
            f"🅑 질주 중 — 남은 {d2goal:.1f}m, "
            f"{'?' if self.gps_kmh is None else f'{self.gps_kmh:.1f}'}km/h, "
            f"e {e:+.2f}m, ψ {psi:+.2f}°, L_p {lp:.1f}m, "
            f"조향 {steer:+.1f}°/{lim:.1f} "
            f"(조준 {aim:+.2f} + 댐핑 {damp:+.2f} + 적분 {self._cte_i_term:+.2f} 도로휠, "
            f"요레이트 {self.yaw_rate:+.1f}°/s, 횡가속 {self._ay_f:.2f}, "
            f"전달비 {self.g_est:.1f}×{self._g_n})", period=1.0)

    # ══════════════════════════════════════════════════════════════════════════
    #  제동 — 여기서부터가 측정 구간이다
    # ══════════════════════════════════════════════════════════════════════════
    def begin_brake(self, why, now, s, e, psi, own_brake=True):
        """정지 트리거.

        ★own_brake★ 는 '리니어를 내가 무는가' 다. [2026-09-13] ★기본이 True 이고
        두 트리거 모두 True 를 쓴다★ — 목표 좌표든 라이다든 이 노드가 직접
        /brake_level 2단을 낸다. 라이다에서 False 로 두었던 종전 판단이
        `/brake_level` 을 ★전 구간 0★ 으로 만들었고, 그러면 arduino (4) 분기의
        `pulse = 0 if brake > 0` 구동 무력화가 ★한 번도 타지 않는다★
        (run_follow ① 주석에 그 사고 경위가 있다). False 는 남겨 두지만 쓰지 않는다.
        """
        self.why = why
        self.hit_dist = self.lidar_dist if why == WHY_LIDAR else float('nan')
        self.hit_xy = (self.x, self.y)
        self.hit_s, self.hit_e, self.hit_psi = s, e, psi
        self.brake_xy = (self.x, self.y)
        self.brake_t0 = now
        v0 = self.speed_ms()
        self.brake_v0 = v0 if v0 is not None else float('nan')
        self._still_t = 0.0
        self.own_brake = own_brake
        if own_brake:
            self.set_brake(BRAKE_FULL)
            self.publish_brake(force=True)
        #  ★조향은 유지한다★ 정지 직전에 앞바퀴를 정면으로 꺾으면 제동 중 거동이
        #  바뀌어 측정이 오염된다. 펄스는 arduino 가 0 으로 덮는다(send docstring).
        self.send(self.drive_value(now), self._last_steer, control=True)
        v0kmh = (float('nan') if not math.isfinite(self.brake_v0)
                 else self.brake_v0 * 3.6)
        extra = (f", 장애물 {self.hit_dist:.2f}m"
                 if math.isfinite(self.hit_dist) else "")
        who = "리니어 2단 체결" if own_brake else "★arduino AEB 가 물었다★"
        self.enter(S_BRAKE,
                   f"🛑 ★{why}★ (진행 {s:.1f}/{self.dist_total:.1f}m, e {e:+.2f}m, "
                   f"ψ {psi:+.1f}°{extra}) — {who}. "
                   f"진입속도 {v0kmh:.2f} km/h ({self.brake_v0:.3f} m/s)")

    def run_brake(self, now):
        self.send(self.drive_value(now), self._last_steer, control=True)
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
    #  완전정지 뒤 리니어 해제
    # ══════════════════════════════════════════════════════════════════════════
    def begin_release(self, now):
        """★물린 채로 런치를 내리지 않는다★

        arduino 가 내려간 뒤에는 /brake_level 로 풀 방법이 없어져, 차를 밀려면
        D5 를 내렸다 올리는 수밖에 없다. 그래서 0 을 내고 ★실제로 빠질 시간★ 을
        준 뒤에 종료한다(arduino 해제유예 0.5s + B보드 0단 복귀 1000ms).

        ⚠️ ★AEB 가 물고 있으면 이 노드는 못 푼다★ /aeb_stop 이 true 인 동안
        arduino 는 리니어를 계속 물고 있고, 우리 /brake_level=0 은 그 분기보다
        우선순위가 낮다(compose (1-1)). 그건 결함이 아니라 설계다 —
        ★장애물이 그대로 있는데 브레이크를 푸는 노드가 있으면 안 된다.★
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
        over = (math.dist(self.hit_xy, (self.x, self.y))
                if self.hit_xy else float('nan'))
        v0 = self.brake_v0
        #  ★두 가지 방법으로 낸다★ 시간식과 에너지식이 크게 다르면 어딘가 틀렸다
        #  (대개 '완전정지' 판정 시각이 늦은 것이다 — 그때 시간식만 작아진다).
        a_t = v0 / held if (math.isfinite(v0) and held > 0.05) else float('nan')
        a_d = (v0 * v0 / (2.0 * d)) if (math.isfinite(v0) and d > 0.05) else float('nan')
        s_now, e_now = self.line_state()
        lines = [
            "═" * 66,
            " 제동검차 — 브레이크 제동거리 측정 결과",
            "═" * 66,
            f"  목표 좌표       ({self.goal_lat:.7f}, {self.goal_lon:.7f})  "
            f"[{self.goal_src}]",
            f"  기준선 길이 D   {self.dist_total:.1f} m   방위 {self.line_brg:+.1f}°",
            f"  지령            {self.cmd_kind}",
            f"  ★정지 사유★     {self.why}",
            f"  트리거 지점     진행 {self.hit_s:.1f} m / D {self.dist_total:.1f} m  "
            f"(횡오차 {self.hit_e:+.2f} m, 방위오차 {self.hit_psi:+.1f}°)",
        ]
        if math.isfinite(self.hit_dist):
            lines.append(
                f"  체결 시 장애물  {self.hit_dist:.2f} m      "
                f"← cone_lidar_node 가 그 순간 본 최근접 거리")
        lines += [
            f"  체결 시 속도 v0 {v0:.3f} m/s ({v0 * 3.6:.2f} km/h)",
            f"  제동시간 t      {held:.2f} s",
            f"  제동거리 d      {d:.2f} m        ← 체결 지점부터 잰 GPS 직선거리",
            f"  트리거 초과거리 {over:.2f} m        ← 트리거를 이만큼 지나서 섰다",
            f"  정지 위치       진행 {s_now:.1f} m (목표 대비 {s_now - self.dist_total:+.1f} m), "
            f"횡오차 {e_now:+.2f} m",
            f"  평균 감속도     {a_t:.2f} m/s²  (v0/t)",
            f"                  {a_d:.2f} m/s²  (v0²/2d)",
            f"  ★전달비 실측★    pot/도로휠 = {self.g_est:.2f}  (표본 {self._g_n}개)"
            f"   ← 모델 {self.plant_pot(1.0, v0 if math.isfinite(v0) else 8.84):.2f}",
            "═" * 66,
            "  ※ BRAKING.md 실측 : 2단 2.2~3.8 m/s² / 1단 0.62~1.05 / 코스트 0.29~0.54",
        ]
        if self.why == WHY_LIDAR and math.isfinite(self.hit_dist):
            reach = self.hit_dist - over
            lines.append(
                f"  ※ 장애물까지 남은 여유 ≈ {reach:+.2f} m  "
                + ("(★들이받았을 거리다★)" if reach < 0 else "(멈춰 섰다)"))
        if note:
            lines.append(f"  ⚠️ {note}")
        for ln in lines:
            self.get_logger().info(ln)
        self.pub_event.publish(String(data=(
            f"🅑 제동 결과 [{self.why}] — v0 {v0 * 3.6:.2f}km/h, d {d:.2f}m, "
            f"t {held:.2f}s, a {a_t:.2f}/{a_d:.2f} m/s², 초과 {over:.2f}m"
            f"{'  ' + note if note else ''}")))
        self._reported = True
        #  ★여기서 DONE 으로 가지 않는다★ 해제 단계(begin_release)가 리니어를 풀고
        #  실제로 빠질 시간을 준 뒤에 DONE 으로 넘긴다.

    def finish(self, why):
        """측정 없이 끝낸다(중단·이상). ★리니어는 물린 채로 두지 않는다★"""
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
