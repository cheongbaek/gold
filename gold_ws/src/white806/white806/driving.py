#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
driving.py ― kasa 자율주행 [white806 / GPS+IMU 최소 추종판]
════════════════════════════════════════════════════════════════════════════════
구 white/driving.py(136KB)를 걷어내고 다시 쓴 것이다. 하드웨어가 kasa A/B 2보드로
바뀌어 기존 제어 파라미터가 맞지 않으므로, ★튜닝의 출발점★ 이 될 만큼 단순하게
되돌렸다. 아직 없는 것 — CTE 적분, 지연보상 예측, 곡률 선행제동, 속도 PID,
조향 슬루제한, 후진, 카메라 융합, 지형/피치 보정.
  ※ 후진·카메라·지형피치는 ★되살릴 계획이 없다★ — 후진은 매핑이 아예 남기지 않고
    (direction 항상 +1), 카메라는 white806 이 안 쓰는 노드·하드웨어가 필요하며,
    지형피치는 구 코드에도 센싱만 남고 보정 알고리즘이 이미 삭제돼 있었다.

남은 제어는 이것뿐이다:

    목표점 = wp_idx 이후 ★LFD 이상 떨어진 첫 앞쪽 WP★
    alpha  = 그 점의 차체기준 방위(왼쪽 +)
    조향   = clamp(−atan(2·L·sin α / 거리), ±40°)   ★− 좌 / + 우 (B보드 규약)★
    속도   = 고정 펄스

  ★[2026-08-11] 순수추종(Pure Pursuit)+가변 LFD 를 구 white 에서 되살렸다★
  그전까지는 wp_idx 가 가리키는 점의 방위를 그대로 겨눴다(steer = −Kp·오차). 맵
  간격이 0.25m 라 그 점은 늘 코앞이고, 그래서 차가 경로에서 옆으로 0.8m 만 떨어져
  출발해도 방위오차가 90° 에 가까워졌다 — 조향이 ±40° 에 포화된 채 최소회전반경
  (L/tan40° ≈ 1.49m)으로 돌기만 하고, 매 틱 새로 닿는 점의 기하가 똑같아 빠져나오지
  못했다(실차 재현: rec_20260811_165756, heading_err 67°→148°, 조향 −40° 5.7초 고정).
  자세한 근거는 pure_pursuit_steer() · lookahead_m() 주석에 있다.

════════════════════════════════════════════════════════════════════════════════
 이 노드가 혼자 맡는 것 (구 white 와 다른 점)
════════════════════════════════════════════════════════════════════════════════
  · GPS+IMU 융합 — 구 gps_imu.py 노드는 white806 에 없다. /fix 와 /imu 를 직접 받아
    위치·헤딩을 여기서 만든다.
  · 단위 환산 없음 — /cmd_vel_raw 는 이미 ★펄스(0~15)·도(degree)★ 단위다
    (nxde/arduino.py 헤더 규약). 그래서 kasa_units 를 거치지 않는다.
  · 모드 스위치는 이제 아무것도 ★시작★시키지 않는다 [2026-08-11] — 매핑·주행의
    시작은 오직 prompt 의 1)매핑/2)주행 메뉴뿐이다(/drive_cmd 의 'MAP_START'·
    'DRIVE_START', 아래 cb_drive_cmd 참고). prompt 가 /vehicle_mode 를 스스로
    지켜보다가 스위치가 목표 위치가 되는 순간 그 명령을 보낸다 — 이 노드는 엣지를
    보고 있지 않다. ★B보드 D5 스위치가 여기서 하는 일은 이제 '취소·정리'뿐이다★
    (아래 on_mode_edge). 예전엔 스위치 엣지 자체가 매핑·주행을 시작시켰는데, 그
    경로가 prompt 의 폴링+명령 경로와 중복이라 없앴다 — 시작 트리거가 두 곳에
    있으면 "지금 뭐가 방금 시작을 시켰는지" 추적이 어려워진다.

════════════════════════════════════════════════════════════════════════════════
 상태기계 — 시작은 prompt 명령, 스위치는 취소·정리만
════════════════════════════════════════════════════════════════════════════════
      IDLE ──(prompt MAP_START)──▶ MAP_HEADING ──(헤딩확정)──▶ MAP_RUN
        ▲                              │                        │
        │                     (자율→수동, 상승: 매핑취소)   (수동→자율, 상승: 경로 저장)
        └──────────────────────────────┴────────────────────────┘

      IDLE ──(prompt DRIVE_START)──▶ DRIVE_HEADING ──(헤딩확정)──▶ DRIVE_RUN
        ▲                                    │                        │
        │                    (수동→자율, 하강: 주행취소)   (마지막 WP 도달 / CTE_DEVIATION_M
        │                                                    초과: 경로이탈 — 둘 다 같은 처리)
        │                                                              ▼
        └──────────(자율→수동, 하강: 주행 중단)──────────────── DRIVE_DONE
                                                       펄스0 + 리니어 2단 → 완전정지
                                                       2초 뒤 자동으로 IDLE(메뉴 복귀)
                                                       (또는 하강 엣지로 즉시 IDLE)

  on_mode_edge() 가 하는 일은 이제 둘뿐이다 — ① 진행 중인 것이 스위치 위치와
  안 맞게 되면 즉시 취소한다(MAP_HEADING/DRIVE_HEADING → IDLE), ② 실제로 끝난
  것을 정리한다(MAP_RUN → 저장 / DRIVE_RUN·DRIVE_DONE → 중단·해제). IDLE 에서는
  ★어떤 엣지도 아무 일을 하지 않는다★ — 시작은 위에서 말했듯 prompt 뿐이다.

  E-stop 은 어느 상태에서든 즉시 IDLE 로 되돌린다. 정지 자체는 아두이노가 이미
  하고 있으므로(A·B 보드가 자체 판정) 여기서 따로 정지 명령을 내지 않는다.

════════════════════════════════════════════════════════════════════════════════
 헤딩 초기화 — 왜 앞으로 굴러야 하는가
════════════════════════════════════════════════════════════════════════════════
  GPS 는 위치만 주고 방향을 주지 않는다. IMU 자이로는 '변화량'만 주므로 시작
  기준각이 없으면 절대 방위를 만들 수 없다. 그래서 출발할 때 ★직진 변위 벡터를
  정면으로 삼는다★ — 그 직진을 누가 만드느냐가 주행/매핑에서 다르다.

    · DRIVE_HEADING(자율) : 이 노드가 ★조향 0 으로 곧게★ 굴린다(heading_pulse).
      단 [2026-08-11] 브레이크 정책은 여기서도 끈다 — A보드가 목표펄스 0→양수
      전환마다 개루프 '기동 블랭킹'에 들어가는데(kasa_0804_A.ino), 브레이크가
      물릴 때마다 target_pulse 를 0으로 보냈다가 되돌리는 게 매번 그걸 재트리거해
      목표보다 훨씬 높은 속도로 다시 settle → 다시 과속 판정 → 다시 브레이크,
      리니어가 무한히 물렸다 풀렸다 하는 되먹임 루프가 됐다. send() 참고.
    · MAP_HEADING(매핑)   : ★사람이 페달을 밟아★ 곧게 굴린다. 이 노드는 GPS 변위를
      관찰만 하고 펄스를 내지 않는다 [2026-08-11] — 수동조종은 사람 발이 속도를
      정하는 구간이라, 여기서 이 노드가 대신 펄스를 내거나 브레이크로 개입하면
      사람의 가속과 다툰다(리니어 2단/해제가 계속 반복되는 버그였다).
      send() 가 상태를 보고 MAP_HEADING·MAP_RUN·DRIVE_HEADING 에서는 브레이크
      정책을 건너뛰는 이유, run_heading_init() 이 두 경우를 분기하는 이유도 이것.

  거리를 미리 정해 두지 않는다 — ★추정 오차가 목표치 아래로 떨어지는 순간 멈춘다★.
  오차는 이동거리에 반비례하므로(σ ≈ atan(위치노이즈 / 거리)) RTK 가 좋으면 1m
  남짓에서 끝나고, 흔들리면 더 간다. DRIVE_HEADING 은 확정 즉시 목표펄스를 0 으로
  준다 — MAP_HEADING 은 원래 펄스를 낸 적이 없으니 그대로 MAP_RUN 으로 넘어간다.

  ┌ 이동거리별 헤딩 오차 (두 점 기준, σ_pos 는 수평 위치 노이즈) ─────────────┐
  │  거리      RTK Fixed(2cm)     RTK Float(30cm)                            │
  │  0.5 m        3.2°               40°                                     │
  │  1.0 m        1.6°               23°     ← Fixed 면 여기서 이미 충분       │
  │  2.0 m        0.8°               16°                                     │
  └──────────────────────────────────────────────────────────────────────────┘
  Float 에서 초기 헤딩이 20° 틀리면 차가 엉뚱하게 조향하고, 그러면 GPS 코스헤딩도
  그 엉뚱한 방향을 가리켜 필터가 '자기 말이 맞다'고 수렴해 버린다 — 경로를 벗어난
  채로. 그래서 ★RTK Fixed 를 요구★ 하고(require_rtk), 직진성(잔차)까지 본다.

  ★단 수동조종에서 조향은 언제나 힘빼기('x')다★ 그러니 MAP_HEADING 구간에서도
  ★사람이 핸들을 잡고 일자를 유지해야 한다★(페달은 밟되 핸들은 곧게). 굽으면
  잔차가 커져 헤딩이 확정되지 않고 최대거리까지 굴러간다(그 경우 경고와 함께
  확정한다).

════════════════════════════════════════════════════════════════════════════════
 리니어 브레이크 — ★'감속 정책'은 완전히 없앴다 (2026-08-11)★
════════════════════════════════════════════════════════════════════════════════
  리니어를 쓰는 곳은 이제 ★DRIVE_DONE 하나뿐★ 이다(도착·정지명령·경로이탈). 거기서는
  조건과 무관하게 2단을 물고, 푸는 것은 ①엔코더 완전정지 확인 후 2초(_check_done_
  release) ②수동조종으로 스위치를 내릴 때 ③이 노드가 내려갈 때 셋뿐이다.

  ★왜 '현재펄스 > 목표펄스+3 이면 2단' 정책을 버렸는가 — 세 번 물려서 세 번 다 실패★
  그 정책은 매번 target_pulse 를 0 으로 덮었고, 그 0→양수 복귀가 A보드
  (kasa_0804_A.ino)의 ★기동 블랭킹(LAUNCH)★ 을 재트리거했다. LAUNCH 는 피드백을 보지
  않는 개루프 구간이고, 그 구간의 홀센서에는 펌웨어 헤더가 직접 적어 둔 대로
  "코일에 힘은 들어갔는데 바퀴가 아직 안 도는" 허수 카운트가 쏟아진다. 정책은 그
  허수를 '과속'으로 읽고 또 물었다 — 자기가 만든 노이즈와 싸우는 되먹임이다.

    1차(수동조종) : 사람이 페달을 떼면 목표가 0 이 되는데 관성 실측이 그보다 크다고
                    물었다. → IDLE·MAP_* 에서 제외
    2차(DRIVE_HEADING) : 헤딩용 저속 목표를 계속 0 으로 덮어 LAUNCH 를 반복 유발.
                    → DRIVE_HEADING 에서도 제외
    3차(DRIVE_RUN) : 곡률 감속으로 목표가 1펄스까지 내려가자 같은 루프가 코너 한복판
                    에서 재현됐다. ★rec_20260811_212858 실측★ — 11.7초 동안 wp_idx 68
                    에서 전혀 전진하지 못하고(자이로 평균 0.33dps = 회전도 없음)
                    브레이크가 3.8초간 물려 있었다. 엔코더 허수(≥8카운트)는 71틱 전부
                    ★지령펄스 0 또는 1★ 일 때만 나왔고(4·3·2펄스에서는 최대 4~5),
                    지령 0펄스 구간의 엔코더 중앙값은 16, 최대 34 였다.
                    → ★DRIVE_RUN 에서도 제외 = 정책 자체를 삭제★

  즉 자율주행 중 감속은 ★전부 곡률 선행제동(목표펄스를 미리 낮춤) + 자연감속(코스트)★
  이 담당한다. 급제동이 필요한 상황은 도착·이탈·정지명령뿐이고 그건 DRIVE_DONE 이다.
  안전망은 A보드 자체 폭주감지(RUNAWAY_CONFIRM_CYCLES=50, 1초)와 E-stop 이다.
"""

import csv
import math
import os
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Bool, Float64MultiArray, Int32, String

from white806 import paths


# ══════════════════════════════════════════════════════════════════════════════
#  튜닝 상수 — 실차에서 만지는 곳은 전부 여기다
# ══════════════════════════════════════════════════════════════════════════════
CONTROL_HZ = 20.0            # 제어 주기. A보드 텔레메트리(20Hz)와 맞췄다

# ── 속도 (펄스. 1펄스 ≈ 0.884 m/s ≈ 3.18 km/h) ──
DRIVE_PULSE   = 4            # ★주행 고정 속도★ ≈ 12.7 km/h
HEADING_PULSE = 3            # 헤딩 초기화 중 속도 ≈ 9.5 km/h

# ══════════════════════════════════════════════════════════════════════════════
#  ★★ 차량 제원 — 순수추종 기하의 전제. 틀리면 전 구간이 어긋난다 ★★
# ══════════════════════════════════════════════════════════════════════════════
#  출처 : 구 white/kasa_units.py(환산상수의 단일 소유자) + white/driving.py:473.
#  ★구 white 의 튜닝값을 읽을 때 반드시 알아야 할 것 — 차가 두 대다★
#
#    구 white 차량 "1/5카" (헤네스 브룬 T870)   금색차 kasa (지금 이 차)
#      휠베이스   0.73 m                          ★1.25 m★ (실측 축거 1250mm)
#      조향 최대  ±21°                            ★±40°★  (B보드 STEER_ANGLE_MAX)
#      최소회전반경 L/tan(δmax) = 1.90 m           ★1.49 m★
#      최고속도   8 km/h (2.22 m/s)               47.7 km/h (13.26 m/s, 인휠 2개)
#      구동       Dual 24V DC 240W                QSWP72V5000W 인휠 × 2
#
#  구 white/driving.py 의 LFD_TABLE·GAIN_TABLE 중 ★v ≤ 1.8 행은 1/5카 실측값★ 이고
#  (2.2 행 주석에 "1/5카에서는 도달 불가였던 구간"이라고 적혀 있다), v ≥ 2.65 행은
#  2026-08-05 에 금색차용으로 새로 만든 ★실차 미검증★ 추정값이다. 그래서 이 파일은
#  표를 베끼지 않고 설계식만 옮겼다(lookahead_m 주석).
#  ※ 휠베이스는 구 white 도 kasa 이식 때 이미 1.25 로 교체했다 — 순수추종 조향각이
#    휠베이스에 정비례하므로 그 값만은 차량을 바꿀 때 반드시 갈아야 한다.
WHEELBASE_M   = 1.25         # [m] 축거 1250mm 실측 (kasa_ws master.py 디퍼렌셜 계산값)
STEER_MAX_DEG = 40           # [deg] B보드 수용 상한 = STEER_ANGLE_MAX (kasa_0804_B.ino)

# ══════════════════════════════════════════════════════════════════════════════
#  ★★ 조향 전달계 실측 보정 — ±40° 는 '도로휠각'이 아니다 (2026-08-11) ★★
# ══════════════════════════════════════════════════════════════════════════════
#  kasa_0804_B.ino 의 ±40° 는 ★조향 가변저항 하드리밋 사이 전체 행정에 붙인 이름★ 일
#  뿐이고, 그것이 도로휠각이라는 보장은 어디에도 없다. 실측해 보니 실제로 달랐다.
#
#  ┌ 측정 : rec_20260811_205127 · 205438 + 두 경로의 매핑 구간, 126 표본 최소자승 ─┐
#  │   pot_지령[deg] = 1.75 · δ_도로휠[deg]  +  7.2 · v²/R      (잔차 RMS 4.2°)    │
#  │   두 독립 방법이 일치 : 자이로+엔코더 곡률 43~46% / ★GPS 궤적★ 곡률 36~40%    │
#  └──────────────────────────────────────────────────────────────────────────────┘
#  · 첫 항 = 링키지비. 도로휠각은 pot 지령의 1/1.75 = 57% 뿐이다.
#  · 둘째 항 = 언더스티어. 같은 반경이라도 속도가 오르면 더 꺾어야 한다(타이어 슬립).
#
#  ★그래서 순수추종이 낸 조향각을 그대로 발행하면 안 된다★ 순수추종 공식이 내는 것은
#  '이 곡률을 만들려면 도로휠을 몇 도 꺾어야 하는가' 이므로, 위 역모델로 pot 지령으로
#  환산해야 한다(steer_command). 이 보정이 없던 2026-08-11 주행에서 코너에 34° 가
#  필요한데 21° 만 나가 두 경로 모두 경로이탈로 정지했다.
#    ※ 구 white 에도 같은 자리에 STEER_PLANT_GAIN_L/R 가 있었지만 kasa 이식 때
#      '미실측'으로 1.0 에 방치돼 있었다 — 그 빈칸을 이제 실측으로 채운 것이다.
STEER_PLANT_GAIN   = 1.75    # pot 지령 / 도로휠각
STEER_UNDERSTEER   = 7.2     # [deg/(m/s²)] 언더스티어 계수
#  도로휠각 상한 = 40 / 1.75 = 22.8°  →  최소회전반경 L/tan(22.8°) = ★2.97 m★
#  (예전에 1.49m 로 적어 두었던 값은 pot 지령을 도로휠각으로 오인한 결과였다)
STEER_ROAD_MAX_DEG = STEER_MAX_DEG / STEER_PLANT_GAIN

# ── 선행거리 LFD (Look-Forward Distance) [2026-08-11 구 white 에서 이식] ──
#   구 white 는 15행짜리 LFD_TABLE 을 썼는데, 그 표의 v≥2.2 구간은 전부 ★ω_n 을
#   일정하게 유지한다★ 는 하나의 설계식에서 나온 값이다 — LFD = v·√2/ω_n.
#   표를 옮기지 않고 식을 옮긴 이유는 lookahead_m() 주석에 적었다.
LFD_OMEGA_N  = 0.97         # [rad/s] 목표 고유진동수. ★낮추면 LFD 가 길어진다★
#  ★LFD 하한의 의미 — 조향 포화가 시작되는 α★
#    순수추종은 δ = atan(2·L·sinα / d) 이므로, d 가 짧으면 조금만 옆을 봐도 δ 가
#    ±40° 에 포화되고 그때부터 '기하'가 아니라 '한계'가 조향을 정한다. 금색차에서
#    포화가 시작되는 α 는 :  LFD 2.2m → 47.6° / 2.3m → 50.5° / 2.98m 이상 → ★없음★
#    즉 ★LFD ≥ 2·최소회전반경(2.98m) 이면 어떤 α 에서도 포화하지 않는다★.
#    현 운용속도 4펄스(3.54 m/s)의 LFD 는 5.16m 라 포화가 기하적으로 불가능하다 —
#    하한 2.3m 이 실제로 쓰이는 구간은 저속(v≲1.6)과 종점 접근뿐이다.
#  ★2.3 은 1/5카 실측값인데 금색차에서 오히려 여유가 늘었다★ LFD/최소회전반경 이
#    1/5카 2.3/1.90 = 1.21 → 금색차 2.3/1.49 = 1.54. 금색차가 더 타이트하게 돌기
#    때문이다(조향 ±40° vs ±21°). 그래서 이 값을 그대로 물려받아도 안전 방향이다.
#    ⚠️ 반대로 ★낮추면 곧바로 포화 영역으로 들어간다★ — 함부로 내리지 말 것.
LFD_MIN_M    = 2.3          # [m] 하한 (구 white min_lfd — 진동유발 영역 탈출 실측값)
LFD_MAX_M    = 6.4          # [m] 상한 (구 표의 마지막 행 = 5펄스 15.9km/h)
LFD_GOAL_A   = 0.45         # 종점 접근 캡 : max(LFD_GOAL_MIN, 남은거리·A + B)
LFD_GOAL_B   = 1.30
LFD_GOAL_MIN = 2.2          # 이 구간만은 포화 임계(2.98m) 아래다 — 위 ★ 참고

# ── 웨이포인트 ──
WP_REACH_M      = 0.2        # ★도착 허용반경★ 마지막 WP 를 이 안에 들면 도착
#   진행 포인터(wp_idx)는 ★창 안의 최근접점★ 으로 옮긴다(advance_wp_idx).
#   맵 간격 0.25m 기준 : 창 45개 = 11.25m, 한 주기 상한 5개 = 1.25m
#   (20Hz·3.5m/s 면 실제로는 한 주기에 0.7개씩 나아간다 — 7배 여유)
WP_SEARCH_WINDOW   = 45
WP_MAX_ADVANCE     = 5
WP_AHEAD_MARGIN_M  = 2.0     # 차체기준 전방거리가 −이것 보다 크면 '앞쪽'으로 본다
WP_AHEAD_PENALTY_M = 1.2     # 앞쪽 최근접이 전체 최근접보다 이보다 나쁘면 후자를 쓴다

# ── 속도 환산 ──
#   1펄스(20ms 창, 바퀴 하나 기준) 당 속도. 구 kasa_units.MS_PER_PULSE 와 같은 값이다.
#     타이어 175/60R13 → 외경 0.5402m → 둘레 1.697147m
#     인휠 홀센서 96펄스/회전 (3상 XOR 6에지 × 16극쌍), A보드 계측창 0.020s
#     1.697147 / 96 / 0.020 = 0.88393 m/s = 3.182 km/h  ← master.py 실측 3.18 과 일치
#   (mapping.py 의 MS_PER_ENC_COUNT 0.442 는 좌+우 합 기준이라 정확히 이 값의 절반)
MS_PER_PULSE = 0.884

# ══════════════════════════════════════════════════════════════════════════════
#  ★★ 곡률 스캔 + 곡률 선행제동 — 구 white/driving.py 에서 이식 (2026-08-11) ★★
# ══════════════════════════════════════════════════════════════════════════════
#  구 white 의 scan_curve_demand() 와 '판단 2. 목표속도' 블록을 값까지 그대로 옮겼다.
#  ★값을 새로 발명하지 않았다★ — 아래 상수는 전부 구 white 의 동명 상수와 같다.
#
#  왜 필요한가 (2026-08-11 실차) : 사람은 매핑할 때 코너를 ★0.44~0.66 m/s 에 풀 락★
#  으로 돌았다. 그 곡률은 그 속도에서만 성립한다 — 언더스티어(위 STEER_UNDERSTEER)
#  때문에 같은 코너를 1.8~2.2 m/s 로 진입하면 조향을 다 써도 돌지 못한다. 그래서
#  코너를 ★미리 보고 미리 줄이는★ 것이 조향 보정과 짝을 이뤄야 한다.
#
#  두 창으로 스캔한다(구 white v6.7.3 과 같은 이유):
#    · WINDOW_M(3.0m)  = 평활된 주 신호. 지속 곡률을 본다.
#    · WINDOW_PEAK_M(1.2m) = 3점 외접원(Menger). 짧고 급한 필렛을 잡는다 —
#      3m 창은 R≈2m 코너를 11° 로 과소평가하는 것이 구 실측으로 확인돼 추가됐다.
CURVE_PREVIEW_NEAR_K  = 1.6    # 근거리 스캔거리 = LFD · 이것 (현재 코너 속도용)
CURVE_PREVIEW_FAR_MAX = 14.0   # [m] 원거리 스캔 상한 (다가오는 코너 조기 인지)
CURVE_WINDOW_M        = 3.0    # [m] 곡률 창 (평활 주신호)
CURVE_WINDOW_PEAK_M   = 1.2    # [m] 짧은 창 (급코너 피크)
CURVE_SCAN_STEP_M     = 0.4    # [m] 스캔 간격

STEER_FULL_SLOWDOWN_DEG = 15.0 # 요구 도로휠각이 이 이상이면 최저속도까지 내린다
PEAK_SPEED_BLEND        = 0.50 # 피크를 속도에 반영하는 비율(0=무시 1=완전). 곡률캡이
                               #   이미 컷을 막으므로 절반만 반영해 순항속도를 보존한다
BRAKE_GATE_DECEL        = 2.0  # [m/s²] 제동거리 가정 감속도 (조기 감속 위해 보수적)
BRAKE_GATE_MARGIN       = 1.5  # [m] 코너 이만큼 전에 목표속도에 도달해 있도록
OMEGA_N_MAX             = 0.95 # 코너에서 LFD 가 짧아질 때 ω_n 을 이 밑으로 묶는다
MIN_SPEED_RATIO         = 1.0 / 2.8   # 최저속도 = 상한 × 이것 (구 white 와 동일)
MIN_SPEED_FLOOR         = 0.9  # [m/s] 그래도 이 밑으로는 내리지 않는다
#  ★코너 최저 펄스 = 2 (1.77 m/s)★ [2026-08-11 실측으로 정한 값]
#  구 white 는 m/s 로 연속 제어했지만 이 차는 정수 펄스라 1펄스 = 0.884 m/s 로 거칠다.
#  rec_20260811_212858 에서 ★1펄스로 내려간 순간 차가 코너에서 아예 멈춰 버렸다★ —
#  10.8초를 1펄스로 지령했는데 엔코더 중앙값이 1, 이어서 11.7초간 wp_idx 68 에서
#  전진 0. 저속 + 코너 타이어 스크럽 부하에서 인휠이 차를 못 굴린 것이다. 그리고
#  멈춘 상태의 코일 통전이 기동 블랭킹 허수 카운트를 만들어 브레이크 채터까지 불렀다.
#  ★2펄스 구간이 오히려 추종이 가장 좋았다★(10.9초, max|CTE| 0.17m) — 그래서 2.
CORNER_MIN_PULSE        = 2

#  ── 곡률 LFD 캡 ── 코너컷 오차 ≈ LFD²/(8R) 이므로 LFD 를 √(K·R) 로 눌러
#     CTE ≤ 약 K/8 = 0.19m 를 보장한다. ★코너에서 조향 권한을 되찾는 장치이기도 하다★
#     (LFD 가 짧아지면 순수추종이 낼 수 있는 최대 도로휠각 atan(2L/LFD) 가 커진다)
LFD_CURVE_CAP_K = 1.5
LFD_LPF_ALPHA   = 0.40         # LFD 급변 완화 (시상수 ≈95ms @20Hz)
LFD_RATE_UP     = 0.9          # [m/s] LFD 증가 상한 — 코너 탈출 후 완만하게
LFD_RATE_DOWN   = 2.0          # [m/s] LFD 감소 상한 — 코너 보호는 즉각

# ── 헤딩 초기화 ──
HEAD_MIN_DIST_M   = 1.0      # 이보다 짧으면 판정하지 않는다
HEAD_MAX_DIST_M   = 5.0      # 여기까지 가면 최선의 추정으로 확정하고 넘어간다
HEAD_MIN_SAMPLES  = 4        # 직진성을 보려면 최소 4점
HEAD_TARGET_SIGMA = 3.0      # [deg] 추정 오차가 이 밑이면 확정
HEAD_SIGMA_FLOOR  = 0.02     # [m] RTK Fixed 수평 노이즈 하한 — 낙관을 막는 바닥값
HEAD_MAX_RESID_M  = 0.15     # 직선 잔차 RMS 상한(곧게 갔는가)
MODE_SETTLE_S     = 0.7      # 스위치 엣지 직후 이만큼은 굴리지 않는다(빠른 토글 대비)

# ── GPS/IMU 융합 ──
FUSE_GAIN          = 0.05    # GPS 코스헤딩으로 끌어당기는 비율(상보필터)
FUSE_MIN_STEP_M    = 0.30    # 이만큼 움직였을 때만 GPS 코스헤딩을 신뢰한다
GPS_TIMEOUT_S      = 2.0     # 이 시간 넘게 /fix 가 없으면 정지
RTK_FIXED_STATUS   = 2       # NavSatFix.status.status (nmea_navsat_driver: GGA q4 → 2)

# ── 브레이크 ──
#   ★[2026-08-11] BRAKE_TRIGGER_DIFF / BRAKE_RELEASE_DIFF 를 삭제했다★
#   '현재펄스 > 목표펄스+3 이면 2단' 감속 정책 자체를 없앴다 — 파일 헤더의
#   '리니어 브레이크' 절에 세 번의 실패 기록과 함께 이유를 적었다. 지금 리니어를
#   쓰는 곳은 DRIVE_DONE 하나뿐이고, 거기서는 조건 없이 2단을 문다.
BRAKE_FULL         = 2
BRAKE_NONE         = 0
ENC_STOP_EPS       = 0.5     # 이 밑이면 '완전정지'로 본다(엔코더 반펄스 노이즈 바닥)
DRIVE_DONE_RELEASE_S = 2.0   # 완전정지 확인 후 이만큼 지나면 자동으로 리니어 해제 + IDLE 복귀

# ── 엔코더 허수 카운트 필터 [2026-08-11] ──
#   A보드는 기동 블랭킹(LAUNCH) 구간에서 "코일에 힘은 들어갔는데 바퀴가 아직 안 도는"
#   허수 홀 카운트를 뱉는다(kasa_0804_A.ino 헤더 [0730-2]). 펌웨어도 PULSE_SANITY_MAX
#   =40 으로 걸러 주지만 그 아래 값은 그대로 나온다 — 실측(rec_20260811_212858)에서
#   지령 0~1펄스 구간의 엔코더가 중앙 16, 최대 34 까지 튀었다(정상 구간은 4~5).
#   ★중앙값 3점★ 으로 단발 스파이크를 죽인다. 실제 가감속은 20Hz 에서 3틱(0.15s)
#   지연되지만, 이 값을 쓰는 곳은 완전정지 판정뿐이라 문제되지 않는다.
ENC_MEDIAN_N = 3

# ── 엔코더 ──
#   /encoder 는 A보드 좌+우 펄스의 ★합★ 이므로 바퀴 하나 기준으로 보려면 2로 나눈다
#   (/cmd_vel_raw 의 펄스는 바퀴 하나 기준 0~15).
ENC_SUM_TO_PULSE = 0.5

# ── 진단(/drive_diag) ──
#   CTE 탐색은 wp_idx 주변 ±이 창만 훑는다. 경로가 몇 천 점이 되어도 비용이 일정하고,
#   되짚어 보면 지금 위치에서 먼 구간의 최근접점은 어차피 의미가 없다.
CTE_WINDOW_WP = 40

# ── 경로이탈 안전정지 [2026-08-11] ──
#   ★유일하게 CTE 를 제어에 쓰는 곳★ 이 이상 벗어나면 즉시 정지 + 리니어 2단
#   (S_DRIVE_DONE 재사용 — 도착과 같은 취급). 그 외의 CTE 보정(조향에 반영 등)은
#   두지 않는다 — 지금 추종기는 여전히 목표 WP 방위만 보고 조향한다.
CTE_DEVIATION_M = 2.0

EARTH_R = 6378137.0


def wrap180(deg):
    """각도를 [-180, 180) 로 접는다. ★이게 없으면 359°와 1°의 차이가 358°로
    계산되어 차가 반대로 꺾는다★"""
    return (deg + 180.0) % 360.0 - 180.0


def latlon_to_xy(lat, lon, lat0, lon0):
    """위경도 → 원점 기준 로컬 평면 [m]. x=동, y=북. 수백 m 범위에서 오차 무시."""
    x = EARTH_R * math.radians(lon - lon0) * math.cos(math.radians(lat0))
    y = EARTH_R * math.radians(lat - lat0)
    return x, y


# ══════════════════════════════════════════════════════════════════════════════
#  헤딩 초기화 추정기
# ══════════════════════════════════════════════════════════════════════════════
class HeadingEstimator:
    """곧게 굴러간 구간의 GPS 점들로 진행 방위를 낸다.

    방향은 ★첫 점 → 마지막 점★ 벡터로 낸다. 최소자승 회귀를 쓰지 않는 이유는
    표본이 적기 때문이다 — 등간격 N점 회귀의 방향 오차는 두 끝점 방식의 √(6/N)
    배라서, N ≤ 6 이면 오히려 회귀가 더 나쁘다. GPS 가 10Hz 이고 3펄스로 1m 를
    지나는 데 1초가 안 걸리니 표본은 대개 그 언저리다.

    대신 중간 점들은 ★직진성 검사★ 에 쓴다(두 끝점을 잇는 직선으로부터의 수직
    거리 RMS). 차가 휘었는데 헤딩을 확정해 버리는 것을 막는 장치다.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.pts = []          # [(x, y)]

    def add(self, x, y):
        # 같은 자리에서 여러 번 받는 것은 의미가 없다(정지 중 GPS 노이즈).
        if self.pts:
            px, py = self.pts[-1]
            if math.hypot(x - px, y - py) < 0.02:
                return
        self.pts.append((x, y))

    def distance(self):
        if len(self.pts) < 2:
            return 0.0
        (x0, y0), (x1, y1) = self.pts[0], self.pts[-1]
        return math.hypot(x1 - x0, y1 - y0)

    def solve(self):
        """→ (heading_deg, sigma_deg, resid_rms, n, dist). 표본 부족이면 None."""
        n = len(self.pts)
        if n < 2:
            return None
        (x0, y0), (x1, y1) = self.pts[0], self.pts[-1]
        dx, dy = x1 - x0, y1 - y0
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return None
        heading = math.degrees(math.atan2(dy, dx))

        # 두 끝점을 잇는 직선에서 각 점까지의 수직거리
        ux, uy = dx / d, dy / d          # 진행방향 단위벡터
        resid_sq = 0.0
        for (px, py) in self.pts:
            ex, ey = px - x0, py - y0
            cross = ex * uy - ey * ux    # 법선 방향 성분 = 수직거리(부호 있음)
            resid_sq += cross * cross
        resid_rms = math.sqrt(resid_sq / n)

        # 방향 오차 추정. 잔차가 0 이어도 GPS 노이즈 바닥값(2cm)은 깔고 본다 —
        # 표본이 두 점뿐이면 잔차가 구조적으로 0 이라 그대로 믿으면 안 된다.
        sigma = math.degrees(math.atan2(max(resid_rms, HEAD_SIGMA_FLOOR), d))
        return heading, sigma, resid_rms, n, d


# ══════════════════════════════════════════════════════════════════════════════
#  노드
# ══════════════════════════════════════════════════════════════════════════════
S_IDLE          = 'IDLE'
S_MAP_HEADING   = 'MAP_HEADING'
S_MAP_RUN       = 'MAP_RUN'
S_DRIVE_HEADING = 'DRIVE_HEADING'
S_DRIVE_RUN     = 'DRIVE_RUN'
S_DRIVE_DONE    = 'DRIVE_DONE'
S_ESTOP         = 'ESTOP'


class DrivingNode(Node):

    def __init__(self):
        super().__init__('driving_node')

        self.declare_parameter('data_dir', '')
        self.declare_parameter('drive_pulse', DRIVE_PULSE)
        self.declare_parameter('heading_pulse', HEADING_PULSE)
        self.declare_parameter('wp_reach_m', WP_REACH_M)
        self.declare_parameter('require_rtk', True)
        # ★[2026-08-11] steer_kp 가 사라졌다★ 순수추종으로 바뀌어 비례게인이 없다.
        #   조향 세기를 만지는 곳은 이제 lfd_omega_n 이다(낮추면 LFD↑ → 조향 완만).
        #   CTE-PID 트림은 아직 이식하지 않았으므로 Kp 가 들어갈 자리 자체가 없다.
        self.declare_parameter('lfd_omega_n', LFD_OMEGA_N)
        self.declare_parameter('lfd_min_m', LFD_MIN_M)
        self.declare_parameter('wheelbase_m', WHEELBASE_M)
        # ★조향 전달계 실측 보정★ 상단 상수 주석의 126표본 최소자승 결과.
        #   실차에서 코너를 여전히 크게 돌면 steer_plant_gain 을 올린다(더 꺾는다).
        self.declare_parameter('steer_plant_gain', STEER_PLANT_GAIN)
        self.declare_parameter('steer_understeer', STEER_UNDERSTEER)

        self.data_dir = paths.data_dir(self.get_parameter('data_dir').value or '')
        self.drive_pulse = int(self.get_parameter('drive_pulse').value)
        self.heading_pulse = int(self.get_parameter('heading_pulse').value)
        self.wp_reach = float(self.get_parameter('wp_reach_m').value)
        self.require_rtk = bool(self.get_parameter('require_rtk').value)
        self.lfd_omega_n = max(0.05, float(self.get_parameter('lfd_omega_n').value))
        self.lfd_min = float(self.get_parameter('lfd_min_m').value)
        self.wheelbase = float(self.get_parameter('wheelbase_m').value)
        self.plant_gain = max(0.1, float(self.get_parameter('steer_plant_gain').value))
        self.understeer = float(self.get_parameter('steer_understeer').value)
        self.road_max = STEER_MAX_DEG / self.plant_gain

        # ── 속도 대역 (구 white 의 max_speed_ms / min_speed_ms 와 같은 역할) ──
        #   white806 은 정수 펄스로 명령하므로 상한은 drive_pulse 가 정하고, 하한은
        #   구 white 와 같은 규칙(상한 × 1/2.8, 바닥 0.9m/s)으로 뽑는다.
        self.max_speed_ms = self.drive_pulse * MS_PER_PULSE
        self.min_speed_ms = min(self.max_speed_ms,
                                max(MIN_SPEED_FLOOR, self.max_speed_ms * MIN_SPEED_RATIO))

        # ── 곡률 프로파일 (경로가 정적이므로 build_waypoints 에서 한 번만 계산) ──
        self.wp_s = []          # 누적 호길이 [m]
        self.wp_req_win = []    # 3.0m 창 요구 도로휠각 [deg]
        self.wp_req_peak = []   # 1.2m 창(Menger) 요구 도로휠각 [deg]
        self._lfd_lpf = LFD_MAX_M
        self._lfd_out = LFD_MAX_M
        self._warned_infeasible = False

        # ── 센서 상태 ──
        self.lat0 = self.lon0 = None      # 로컬 평면 원점(첫 fix)
        self.x = self.y = 0.0
        self.fix_ok = False               # RTK 품질 만족
        self.fix_time = 0.0
        self._last_fuse_pt = None         # 코스헤딩 계산용 직전 점

        self.heading = None               # [deg] 확정 전에는 None
        self.gyro_z = 0.0                 # [rad/s] CCW +
        self.imu_time = 0.0

        self.enc_pulse = 0.0              # 바퀴 하나 기준 현재 펄스(중앙값 필터 후)
        self._enc_buf = []                # 허수 카운트 제거용 (ENC_MEDIAN_N)
        self.auto_mode = None             # /vehicle_mode. None = 미수신
        self.estop = False

        # ── 상태기계 ──
        self.state = S_IDLE
        self.state_t0 = time.time()
        self.head_est = HeadingEstimator()
        self.waypoints = []
        self.wp_idx = 0
        self.route_name = ''
        self.route_path = ''

        # ── 출력 상태 ──
        self.brake_now = BRAKE_NONE
        self._done_zero_t = None      # DRIVE_DONE 에서 완전정지를 확인한 시각

        # ── 진단 계측 (/drive_diag) ★제어 판단에 절대 쓰지 않는다★ ──
        #   여기 있는 값이 제어로 새어 들어가면 '계측을 위해 거동이 바뀌는' 상태가
        #   된다. 전부 쓰기 전용이고, 읽는 곳은 publish_state_topics() 하나뿐이다.
        self._diag_course = float('nan')   # 직전 GPS 변위 방위 [deg]
        self._diag_fuse = 0.0              # 직전 융합이 헤딩을 당긴 양 [deg]
        self._diag_target_idx = 0          # 조향이 겨눈 WP
        self._diag_target_dist = 0.0       # 그 WP 까지 거리 = 실효 선행거리 [m]
        self._diag_head_err = 0.0          # 제어기 입력 오차 [deg]
        self._diag_init = (0.0, 0.0, 0.0, 0.0)   # 확정 헤딩/σ/잔차/기선

        # ── 퍼블리셔 ──
        self.pub_cmd   = self.create_publisher(Twist,  '/cmd_vel_raw',      10)
        self.pub_state = self.create_publisher(Bool,   '/control_state',    10)
        self.pub_brake = self.create_publisher(Int32,  '/brake_level',      10)
        # [2026-08-07] /vehicle_mode_cmd 발행이 사라졌다 — 주행모드는 물리 스위치
        #   전용이고, 수동조종에서의 펄스는 arduino 가 직접 받아 준다.
        self.pub_map   = self.create_publisher(Bool,   '/mapping_cmd',      10)
        self.pub_dstate = self.create_publisher(String, '/drive_state',     10)
        self.pub_event = self.create_publisher(String, '/drive_event',      10)
        self.pub_ego   = self.create_publisher(Float64MultiArray, '/ego_state', 10)
        # ★[2026-08-08] 추종 진단 — record 전용, 제어는 이 값을 읽지 않는다★
        #   /ego_state 는 '차가 어디 있나', 이쪽은 '얼마나 잘 따라가고 있나' 다.
        #   둘을 합치지 않은 이유는 /ego_state 의 열 배치가 이미 소비처를 가지고
        #   있어서다(record 의 _array(7)). 늘리면 그쪽이 조용히 밀린다.
        self.pub_diag  = self.create_publisher(Float64MultiArray, '/drive_diag', 10)

        # ── 구독 ──
        self.create_subscription(NavSatFix, '/fix',          self.cb_fix,     10)
        self.create_subscription(Imu,       '/imu',          self.cb_imu,     10)
        self.create_subscription(Int32,     '/encoder',      self.cb_encoder, 10)
        self.create_subscription(Bool,      '/vehicle_mode', self.cb_mode,    10)
        self.create_subscription(Bool,      '/estop',        self.cb_estop,   10)
        self.create_subscription(String,    '/drive_cmd',    self.cb_drive_cmd, 10)

        self.create_timer(1.0 / CONTROL_HZ, self.loop)

        self.event(f"white806 driving 준비 — 경로 폴더 {self.data_dir}")
        self.event("prompt 에서 1)매핑 또는 2)주행 선택 — 스위치는 취소·정리만 한다")

    # ══════════════════════════════════════════════════════════════════════════
    #  수신
    # ══════════════════════════════════════════════════════════════════════════
    def cb_fix(self, msg: NavSatFix):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        if self.lat0 is None:
            self.lat0, self.lon0 = msg.latitude, msg.longitude
        self.x, self.y = latlon_to_xy(msg.latitude, msg.longitude,
                                      self.lat0, self.lon0)
        self.fix_ok = (not self.require_rtk) or (msg.status.status >= RTK_FIXED_STATUS)
        self.fix_time = time.time()

        if self.state in (S_MAP_HEADING, S_DRIVE_HEADING) and self.fix_ok:
            self.head_est.add(self.x, self.y)
        self._fuse_gps_course()

    def cb_imu(self, msg: Imu):
        now = time.time()
        if self.imu_time > 0.0 and self.heading is not None:
            dt = now - self.imu_time
            if 0.0 < dt < 0.5:
                # 자이로 z(CCW +)를 그대로 적분한다. 절대 기준은 초기화가 잡아준다.
                self.heading = wrap180(self.heading + math.degrees(self.gyro_z) * dt)
        self.gyro_z = float(msg.angular_velocity.z)
        self.imu_time = now

    def cb_encoder(self, msg: Int32):
        """★중앙값 3점 필터★ A보드 기동 블랭킹 구간의 허수 카운트를 죽인다 —
        상단 ENC_MEDIAN_N 주석 참고(정상 4~5 인데 34 까지 튄 실측이 있다)."""
        self._enc_buf.append(float(msg.data))
        if len(self._enc_buf) > ENC_MEDIAN_N:
            del self._enc_buf[0]
        med = sorted(self._enc_buf)[len(self._enc_buf) // 2]
        self.enc_pulse = med * ENC_SUM_TO_PULSE

    def cb_mode(self, msg: Bool):
        new = bool(msg.data)
        if self.auto_mode is None:
            self.auto_mode = new            # 첫 수신은 엣지로 치지 않는다
            self.event(f"주행모드 최초 인식: {'자율' if new else '수동조종'}")
            return
        if new == self.auto_mode:
            return
        self.auto_mode = new
        self.on_mode_edge(rising=new)

    def cb_estop(self, msg: Bool):
        new = bool(msg.data)
        if new and not self.estop:
            self.estop = True
            self.enter(S_ESTOP, "🚨 E-STOP — 모든 동작 중지, 메인화면 복귀")
        elif self.estop and not new:
            self.estop = False
            self.enter(S_IDLE, "✅ E-stop 해제 — 처음부터 다시")
        self.estop = new

    def cb_drive_cmd(self, msg: String):
        """prompt 하달. 'STOP' = 즉시 정지+리니어 2단 / 'MAP_START'·'DRIVE_START' =
        스위치가 이미 목표 위치일 때 엣지 없이 직접 시작 / 그 외 = 경로 선택.

        ★E-stop 중에는 전부 무시한다★ on_mode_edge() 의 같은 가드와 이유가 같다 —
        E-stop 해제는 오직 /estop 하드웨어 신호로만 나가야 한다. 이 가드가 없으면
        prompt 의 '아무 키나 눌러 중단'(STOP 전송)이 E-STOP 화면에서도 먹혀
        소프트웨어로 E-stop을 빠져나가는 구멍이 생긴다."""
        if self.state == S_ESTOP:
            return
        cmd = str(msg.data).strip()
        if not cmd:
            return
        upper = cmd.upper()
        if upper == 'STOP':
            if self.state in (S_DRIVE_HEADING, S_DRIVE_RUN):
                self.enter(S_DRIVE_DONE, "🛑 정지 명령 — 리니어 2단 체결")
            else:
                self.enter(S_IDLE, "🛑 정지 명령")
            return
        if upper == 'MAP_START':
            # ★스위치가 이미 수동조종이라 엣지가 없다★ prompt 가 대신 눌러 준다 —
            #   IDLE·수동조종일 때만 받아준다(엣지가 만드는 전이와 동일하게).
            if self.state != S_IDLE:
                self.event(f"⚠️ 매핑 시작 실패 — 현재 상태 {self.state}(IDLE 에서만 가능)")
            elif self.auto_mode is not False:
                self.event("⚠️ 매핑 시작 실패 — 스위치가 수동조종이어야 한다")
            else:
                self.enter(S_MAP_HEADING, "🗺️ 매핑 시작(prompt) — 페달로 곧게 굴려 헤딩을 잡을 것")
            return
        if upper == 'DRIVE_START':
            # ★스위치가 이미 자율주행이라 엣지가 없다★ 위와 대칭.
            if self.state != S_IDLE:
                self.event(f"⚠️ 주행 시작 실패 — 현재 상태 {self.state}(IDLE 에서만 가능)")
            elif self.auto_mode is not True:
                self.event("⚠️ 주행 시작 실패 — 스위치가 자율주행이어야 한다")
            elif not getattr(self, 'raw_wps', None):
                self.event("⚠️ 주행 시작 실패 — 경로가 선택되지 않았다")
            else:
                self.enter(S_DRIVE_HEADING, f"▶ 주행 시작(prompt) [{self.route_name}]")
            return
        self.select_route(cmd)

    # ══════════════════════════════════════════════════════════════════════════
    #  경로
    # ══════════════════════════════════════════════════════════════════════════
    def select_route(self, name):
        path = os.path.join(self.data_dir, name)
        if not os.path.isfile(path):
            self.event(f"❌ 경로 파일 없음: {path}")
            return
        wps = []
        try:
            with open(path, 'r', encoding='utf-8') as f:
                for row in csv.DictReader(f):
                    try:
                        wps.append((float(row['latitude']), float(row['longitude'])))
                    except (KeyError, ValueError, TypeError):
                        continue
        except Exception as e:
            self.event(f"❌ 경로 읽기 실패: {e}")
            return
        if len(wps) < 2:
            self.event(f"❌ 웨이포인트가 부족하다({len(wps)}개): {name}")
            return
        self.route_name, self.route_path, self.raw_wps = name, path, wps
        self.event(f"📁 경로 선택: {name} (WP {len(wps)}개) — 스위치를 자율로 올리면 출발")

    def build_waypoints(self):
        """선택된 경로를 현재 원점 기준 로컬 좌표로 변환 + 곡률 프로파일 사전계산."""
        if self.lat0 is None or not getattr(self, 'raw_wps', None):
            return False
        self.waypoints = [latlon_to_xy(la, lo, self.lat0, self.lon0)
                          for (la, lo) in self.raw_wps]
        self.wp_idx = 0
        self._lfd_lpf = self._lfd_out = LFD_MAX_M
        self._warned_infeasible = False
        self.build_curve_profile()
        return True

    def build_curve_profile(self):
        """경로의 요구 도로휠각 프로파일을 ★한 번만★ 계산한다.

        구 white 는 이것을 매 주기(20Hz) scan_curve_demand() 로 다시 계산했는데,
        ★경로는 정적이므로 다시 계산할 이유가 없다★ — 여기서 한 번 만들어 두고
        주행 중에는 창 안의 최대값만 꺼내 쓴다(scan_curve_demand). 계산식과 두 창의
        의미는 구 white 와 동일하다:
          · win  (CURVE_WINDOW_M 3.0m) : 앞뒤 창의 진행방위 차 → κ=|Δθ|/win
          · peak (CURVE_WINDOW_PEAK_M 1.2m) : 3점 외접원(Menger) → κ=1/R
        요구 도로휠각은 둘 다 δ = atan(L·κ) 다.
        """
        p = self.waypoints
        n = len(p)
        s = [0.0] * n
        for i in range(1, n):
            s[i] = s[i - 1] + math.hypot(p[i][0] - p[i - 1][0], p[i][1] - p[i - 1][1])
        self.wp_s = s

        def idx_at(dist):
            """호길이 dist 에 가장 먼저 닿는 인덱스(구 white idx_at 과 같은 규칙)."""
            lo, hi = 0, n - 1
            while lo < hi:
                mid = (lo + hi) // 2
                if s[mid] >= dist:
                    hi = mid
                else:
                    lo = mid + 1
            return lo

        win, wpk = CURVE_WINDOW_M, CURVE_WINDOW_PEAK_M
        req_win = [0.0] * n
        req_peak = [0.0] * n
        for i in range(n):
            d = s[i]
            # ── 평활 창 : 앞뒤 방위차 ──
            i0 = idx_at(max(0.0, d - win))
            i1 = i
            i2 = idx_at(min(d + win, s[-1]))
            if i1 > i0 and i2 > i1:
                h0 = math.atan2(p[i1][1] - p[i0][1], p[i1][0] - p[i0][0])
                h1 = math.atan2(p[i2][1] - p[i1][1], p[i2][0] - p[i1][0])
                diff = abs(wrap180(math.degrees(h1 - h0)))
                req_win[i] = math.degrees(math.atan(
                    self.wheelbase * math.radians(diff) / win))
            # ── 짧은 창 : 3점 외접원 ──
            j0 = idx_at(max(0.0, d - wpk))
            j2 = idx_at(min(d + wpk, s[-1]))
            if j0 < i < j2:
                ax, ay = p[j0]; bx, by = p[i]; cx, cy = p[j2]
                la = math.hypot(bx - ax, by - ay)
                lb = math.hypot(cx - bx, cy - by)
                lc = math.hypot(cx - ax, cy - ay)
                area2 = abs((bx - ax) * (cy - ay) - (cx - ax) * (by - ay))
                if area2 > 1e-6 and la * lb * lc > 1e-9:
                    r_menger = (la * lb * lc) / (2.0 * area2)
                    req_peak[i] = math.degrees(math.atan(self.wheelbase / r_menger))
        self.wp_req_win = req_win
        self.wp_req_peak = req_peak

        # 이 경로가 애초에 따라갈 수 있는 것인지 먼저 알려 준다 — 도로휠각 상한을
        # 넘는 코너는 ★속도를 0 으로 줄여도 기하적으로 불가능★ 하다.
        worst = max(req_peak) if req_peak else 0.0
        if worst > self.road_max:
            r_min = self.wheelbase / math.tan(math.radians(worst))
            self.event(f"⚠️ 이 경로에는 차량 한계보다 급한 코너가 있다 — 요구 "
                       f"{worst:.1f}° > 가능 {self.road_max:.1f}° (R≈{r_min:.2f}m < "
                       f"{self.wheelbase / math.tan(math.radians(self.road_max)):.2f}m). "
                       f"더 넓게 다시 매핑할 것을 권한다")

    # ══════════════════════════════════════════════════════════════════════════
    #  상태 전이
    # ══════════════════════════════════════════════════════════════════════════
    def on_mode_edge(self, rising: bool):
        """스위치 엣지 → 상태 전이. [2026-08-11] ★더 이상 아무것도 시작시키지
        않는다★ — 매핑·주행의 시작은 prompt 의 MAP_START/DRIVE_START(cb_drive_cmd)
        뿐이다. 이 함수는 '진행 중인 것이 스위치와 안 맞게 되면 취소', '실제로
        끝난 것을 정리' 두 가지만 한다.

        ┌ 현재상태      │ 하강(자율→수동)        │ 상승(수동→자율)  ┐
        │ IDLE          │ —                      │ —                │
        │ MAP_HEADING   │ —                      │ IDLE (매핑 취소) │
        │ MAP_RUN       │ —                      │ IDLE (경로 저장) │
        │ DRIVE_HEADING │ IDLE (주행 취소)       │ —                │
        │ DRIVE_RUN     │ IDLE (주행 중단)       │ —                │
        │ DRIVE_DONE    │ IDLE (기록저장·해제)   │ —                │
        └───────────────┴────────────────────────┴──────────────────┘

        IDLE 행이 전부 '—' 인 게 핵심이다 — 아무것도 진행 중이지 않으니 취소·정리
        할 대상이 없다. 시작하려면 prompt 에서 1)매핑 또는 2)주행을 눌러야 한다.
        """
        if self.state == S_ESTOP:
            return
        if rising:                                   # 수동 → 자율
            if self.state == S_MAP_HEADING:
                self.enter(S_IDLE, "🗺️ 매핑 취소 — 스위치가 자율로 전환됨")
            elif self.state == S_MAP_RUN:
                self.enter(S_IDLE, "🗺️ 매핑 종료 — 경로 저장")
        else:                                        # 자율 → 수동
            if self.state == S_DRIVE_HEADING:
                self.enter(S_IDLE, "▶ 주행 취소 — 스위치가 수동으로 전환됨")
            elif self.state == S_DRIVE_RUN:
                self.enter(S_IDLE, "■ 주행 중단")
            elif self.state == S_DRIVE_DONE:
                self.enter(S_IDLE, "■ 주행 종료 — 기록 저장, 리니어 해제")

    def enter(self, new_state, msg=""):
        old = self.state
        self.state = new_state
        self.state_t0 = time.time()
        if msg:
            self.event(msg)

        # 매핑 수집은 ★헤딩이 잡히기 전부터(MAP_HEADING)★ 시작한다. [2026-08-11]
        # 예전엔 헤딩이 잡힌 뒤(MAP_RUN)부터였는데, 사람이 페달로 곧게 굴리는
        # 헤딩용 구간도 경로의 시작이므로 그때부터 기록한다(mapping.py 참고).
        # MAP_HEADING↔MAP_RUN 전이 사이에는 계속 '매핑 중'이므로 다시 쏘지 않는다.
        if new_state == S_MAP_HEADING:
            self.pub_map.publish(Bool(data=True))
        elif old in (S_MAP_HEADING, S_MAP_RUN) and new_state not in (S_MAP_HEADING, S_MAP_RUN):
            self.pub_map.publish(Bool(data=False))

        if new_state in (S_MAP_HEADING, S_DRIVE_HEADING):
            self.head_est.reset()
            self.heading = None
            if new_state == S_DRIVE_HEADING and not self.build_waypoints():
                self.enter(S_IDLE, "❌ GPS 원점이 없어 경로를 세울 수 없다")
                return

        # 브레이크
        if new_state == S_DRIVE_DONE:
            self._done_zero_t = None    # loop() 의 _check_done_release() 가 새로 잰다
            self.set_brake(BRAKE_FULL)
        else:
            # ★DRIVE_DONE 이 아닌 모든 상태에서는 반드시 풀어 준다★ 리니어를 물리는
            # 곳이 DRIVE_DONE 하나뿐이므로, 거기서 나오면 무조건 해제가 맞다(수동조종
            # 진입 = 사람이 차를 넘겨받는 순간이라 특히 중요하다).
            self.set_brake(BRAKE_NONE)

    # ══════════════════════════════════════════════════════════════════════════
    #  융합
    # ══════════════════════════════════════════════════════════════════════════
    def _fuse_gps_course(self):
        """GPS 변위 방위로 헤딩을 천천히 끌어당긴다(상보필터).

        ★정지·저속에서는 쓰지 않는다★ — 1cm 노이즈가 방향을 180° 뒤집는다.
        FUSE_MIN_STEP_M 이상 움직였을 때만 한 번 반영한다.
        """
        if self.heading is None or not self.fix_ok:
            self._last_fuse_pt = (self.x, self.y)
            return
        if self._last_fuse_pt is None:
            self._last_fuse_pt = (self.x, self.y)
            return
        px, py = self._last_fuse_pt
        dx, dy = self.x - px, self.y - py
        if math.hypot(dx, dy) < FUSE_MIN_STEP_M:
            return
        course = math.degrees(math.atan2(dy, dx))
        corr = FUSE_GAIN * wrap180(course - self.heading)
        self.heading = wrap180(self.heading + corr)
        self._last_fuse_pt = (self.x, self.y)
        # 진단 : 융합이 실제로 돌고 있는지 / 얼마나 당기고 있는지를 남긴다.
        #   RTK 가 Fixed 를 벗어나면 이 함수가 위에서 조용히 리턴하므로,
        #   fuse_corr 이 0 으로 굳은 구간 = 헤딩이 자이로 단독으로 흘러간 구간이다.
        self._diag_course, self._diag_fuse = course, corr

    # ══════════════════════════════════════════════════════════════════════════
    #  제어 루프
    # ══════════════════════════════════════════════════════════════════════════
    def loop(self):
        self.publish_state_topics()

        if self.state in (S_IDLE, S_ESTOP):
            self.send(0, 0.0, control=False)
            return

        # GPS 두절 — 위치를 모르면 어떤 판단도 신뢰할 수 없다
        if time.time() - self.fix_time > GPS_TIMEOUT_S:
            self.send(0, 0.0, control=True)
            self.throttle_event("⚠️ GPS 두절 — 정지 유지")
            return

        if self.state == S_MAP_HEADING:
            self.run_heading_init(next_state=S_MAP_RUN)
        elif self.state == S_DRIVE_HEADING:
            self.run_heading_init(next_state=S_DRIVE_RUN)
        elif self.state == S_MAP_RUN:
            # 사람이 페달로 몬다. 이 노드는 아무 명령도 내지 않는다
            # (nxde 가 수동조종에서 /cmd_vel_raw 를 무시하기도 한다).
            self.send(0, 0.0, control=False)
        elif self.state == S_DRIVE_RUN:
            self.run_follow()
        elif self.state == S_DRIVE_DONE:
            self.send(0, 0.0, control=True)     # 브레이크 2단은 enter() 에서 물렸다
            self._check_done_release()

    def _check_done_release(self):
        """DRIVE_DONE 에서 완전정지를 확인하고, 확인된 지 DRIVE_DONE_RELEASE_S 뒤
        자동으로 리니어를 풀고 IDLE 로 돌아간다(= prompt 화면도 메뉴로 자동 복귀).

        ★스위치를 내려도 여전히 즉시 IDLE 로 갈 수 있다★(on_mode_edge, 파일 헤더
        상태기계 참고) — 이건 그 대안 경로일 뿐, 사람이 넘겨받는 경로를 없애지 않는다.
        """
        if self.enc_pulse > ENC_STOP_EPS:
            self._done_zero_t = None
            return
        if self._done_zero_t is None:
            self._done_zero_t = time.time()
        elif time.time() - self._done_zero_t >= DRIVE_DONE_RELEASE_S:
            self.enter(S_IDLE, "🏁 완전정지 확인 — 리니어 해제, 메뉴로 복귀")

    # ── 헤딩 초기화 ────────────────────────────────────────────────────────────
    def run_heading_init(self, next_state):
        manual = (next_state == S_MAP_RUN)

        if manual:
            # ★매핑 헤딩은 사람이 페달+핸들로 곧게 굴린다★ 이 노드는 GPS 변위를
            #   관찰만 하고 펄스를 내지 않는다 — 헤딩용 펄스와 브레이크 정책이
            #   사람의 가속과 다투던 문제를 근본적으로 없앤다.
            self.send(0, 0.0, control=False)
        else:
            # ★진입 직후 잠깐은 굴리지 않는다★ 요령상 스위치를 빠르게 토글하는 경우가
            #   있는데(자율↔수동 왕복), 그 찰나마다 차가 튀어나가면 위험하다.
            if time.time() - self.state_t0 < MODE_SETTLE_S:
                self.send(0, 0.0, control=True)
                return
            self.send(self.heading_pulse, 0.0, control=True)   # 조향 0 으로 곧게

        sol = self.head_est.solve()
        if sol is None:
            return
        heading, sigma, resid, n, dist = sol

        if dist < HEAD_MIN_DIST_M:
            return

        good = (n >= HEAD_MIN_SAMPLES
                and sigma <= HEAD_TARGET_SIGMA
                and resid <= HEAD_MAX_RESID_M)
        forced = dist >= HEAD_MAX_DIST_M

        if not (good or forced):
            return

        self.heading = heading
        self._last_fuse_pt = (self.x, self.y)
        # 진단 : 확정 조건을 숫자로 남긴다. 이후 매 행에 그대로 붙으므로
        #   "이 주행은 σ 몇 도짜리 헤딩으로 출발했나" 를 로그만 보고 알 수 있다.
        self._diag_init = (heading, sigma, resid, dist)
        if next_state == S_DRIVE_RUN:
            self.align_start_wp()
            self.send(0, 0.0, control=True)        # ★확정 즉시 목표펄스 0★
        mark = "" if good else "  ⚠️(최대거리 도달 — 정확도 미달인 채 확정)"
        self.event(f"🧭 헤딩 확정 {heading:+.1f}° "
                   f"(±{sigma:.1f}°, {dist:.2f}m, {n}점, 잔차 {resid*100:.1f}cm){mark}")
        self.enter(next_state)

    def align_start_wp(self):
        """출발 WP 를 현재 위치에 맞춘다.

        ★없으면 차가 유턴한다★ 헤딩 초기화로 1~2m 앞으로 나온 뒤 wp_idx=0 을
        그대로 쓰면, 이미 지나온 첫 WP 가 등 뒤에 있어 목표 방위가 180° 로 나온다.
        조향이 포화된 채 경로를 벗어나 원을 그린다(시뮬레이션에서 그대로 재현됐다).

        최근접 WP 를 찾고, 그것이 등 뒤(오차 90° 초과)면 하나 앞을 잡는다.
        """
        if not self.waypoints:
            return
        best_i, best_d = 0, float('inf')
        for i, (wx, wy) in enumerate(self.waypoints):
            d = math.hypot(wx - self.x, wy - self.y)
            if d < best_d:
                best_i, best_d = i, d
        if self.heading is not None and best_i < len(self.waypoints) - 1:
            wx, wy = self.waypoints[best_i]
            ang = wrap180(math.degrees(math.atan2(wy - self.y, wx - self.x))
                          - self.heading)
            if abs(ang) > 90.0:
                best_i += 1
        self.wp_idx = best_i
        self.event(f"📍 출발 WP {best_i}/{len(self.waypoints)} "
                   f"(최근접 {best_d:.2f}m)")

    # ── 경로 추종 ──────────────────────────────────────────────────────────────
    def run_follow(self):
        if self.heading is None or not self.waypoints:
            self.send(0, 0.0, control=True)
            return

        # ★진행 포인터를 먼저 옮긴다★ CTE 창(signed_cte)과 목표점 탐색 시작점이
        #   둘 다 wp_idx 기준이므로, 낡은 포인터로 판정하면 둘 다 어긋난다.
        self.advance_wp_idx()

        # ★경로이탈 안전정지★ 매핑 경로에서 CTE_DEVIATION_M 이상 벗어나면 조향을
        #   더 계산하지 않고 곧바로 도착과 같은 정지 절차로 넘긴다.
        cte = self.signed_cte()
        if not math.isnan(cte) and abs(cte) > CTE_DEVIATION_M:
            self.enter(S_DRIVE_DONE,
                       f"🚨 경로이탈 — CTE {cte:+.2f}m, 정지 + 리니어 2단")
            return

        # 종점 판정 — 마지막 WP 에 도달하면 즉시 정지 + 리니어 2단
        gx, gy = self.waypoints[-1]
        d2goal = math.hypot(gx - self.x, gy - self.y)
        if self.wp_idx >= len(self.waypoints) - 1 and d2goal <= self.wp_reach:
            self.enter(S_DRIVE_DONE,
                       f"🎯 도착 — 마지막 WP {d2goal:.2f}m, 정지 + 리니어 2단")
            return

        # ── 인지 : 전방 곡률 (근거리=현재 코너 / 원거리=다가오는 코너) ──
        #   구 white 와 같은 2단 스캔. 근거리 스캔거리는 LFD 에 비례하므로 직전
        #   주기의 LFD(_lfd_out)를 쓴다 — 구 white 도 같은 이유로 저역값을 썼다.
        near_preview = max(self.lfd_min, self._lfd_out) * CURVE_PREVIEW_NEAR_K
        near_win, _, near_peak, _ = self.scan_curve_demand(near_preview)
        far_win, far_dist, far_peak, far_peak_dist = \
            self.scan_curve_demand(CURVE_PREVIEW_FAR_MAX)

        # ── 판단 1. LFD = min(속도표, 곡률캡) ──
        lfd, lfd_win_only, lfd_speed = self.lookahead_m(d2goal, near_win, near_peak)

        # ── 판단 2. 목표속도 = 곡률 선행제동 ──
        pulse = self.corner_speed(near_win, near_peak, far_win, far_dist,
                                  far_peak, far_peak_dist, lfd_win_only, lfd_speed)

        # ── 제어 : 순수추종(도로휠각) → 전달계 보정 → pot 지령 ──
        road = self.pure_pursuit_steer(lfd)
        steer = self.steer_command(road, pulse * MS_PER_PULSE)
        self.send(pulse, steer, control=True)

    def scan_curve_demand(self, preview_dist):
        """wp_idx 앞 preview_dist 구간의 요구 도로휠각 최대값을 꺼낸다.
        → (win_max, win_dist, peak_max, peak_dist)   거리는 차 기준 전방거리 [m]

        구 white scan_curve_demand() 와 결과가 같지만, 곡률 자체는
        build_curve_profile() 이 미리 계산해 두었으므로 여기서는 창 안을 훑기만 한다.
        """
        n = len(self.waypoints)
        if n < 3 or not self.wp_req_win:
            return 0.0, float('inf'), 0.0, float('inf')
        s0 = self.wp_s[min(self.wp_idx, n - 1)]
        win_max = peak_max = 0.0
        win_d = peak_d = float('inf')
        for i in range(self.wp_idx, n):
            d = self.wp_s[i] - s0
            if d > preview_dist:
                break
            if self.wp_req_win[i] > win_max:
                win_max, win_d = self.wp_req_win[i], d
            if self.wp_req_peak[i] > peak_max:
                peak_max, peak_d = self.wp_req_peak[i], d
        return win_max, win_d, peak_max, peak_d

    def corner_speed(self, near_win, near_peak, far_win, far_dist,
                     far_peak, far_peak_dist, lfd_win_only, lfd_speed):
        """곡률 선행제동 → 목표 주행펄스.
        [2026-08-11 구 white/driving.py '판단 2. 목표 속도' 이식 — 상수까지 동일]

        (a) 근거리 곡률 비례 감속 : 지금 코너에 맞는 속도
        (b) 원거리 제동거리 게이팅 : ★다가오는 코너에 딱 필요한 만큼만 미리 감속★
        (b2) ω_n 결속 : 곡률캡이 LFD 를 눌렀으면 그 LFD 에 맞는 속도로 묶는다

        ★왜 (b) 가 핵심인가★ 코너에 들어선 뒤 줄이면 이미 늦다 — 언더스티어 때문에
        진입속도가 높으면 조향을 다 써도 못 돈다(2026-08-11 실차). 그래서 코너를
        BRAKE_GATE_MARGIN(1.5m) 앞에서 이미 목표속도에 도달하도록 거꾸로 계산한다:
            v_brake = √(v_corner² + 2·a·(게이트거리 − 1.5))   a = 2.0 m/s²
        """
        def demand_to_speed(demand_deg):
            r = max(0.0, min(1.0, demand_deg / STEER_FULL_SLOWDOWN_DEG))
            return self.max_speed_ms - (self.max_speed_ms - self.min_speed_ms) * r

        # (a) 현재 코너 — 피크는 절반만 반영(곡률캡이 이미 컷을 막으므로)
        near_for_speed = near_win + PEAK_SPEED_BLEND * max(0.0, near_peak - near_win)
        v_target = demand_to_speed(near_for_speed)

        # (b) 다가오는 코너 제동거리 게이팅
        far_for_gate = far_win + PEAK_SPEED_BLEND * max(0.0, far_peak - far_win)
        gate_dist = far_dist
        if far_peak_dist != float('inf') and far_peak > far_win + 1.0:
            gate_dist = min(gate_dist, far_peak_dist)   # 급피크가 더 가까우면 그쪽 기준
        if gate_dist != float('inf') and far_for_gate > near_win + 1.0:
            v_corner = demand_to_speed(far_for_gate)
            if far_peak > 2.5:
                # 코너 도착 시점의 LFD(곡률캡)와 ω_n 정합 속도로 '미리' 낮춘다 —
                # 진입 순간 LFD 급감 + 속도 잔존으로 ω_n 이 튀는 것을 예방한다.
                r_far = self.wheelbase / math.tan(math.radians(min(far_peak, self.road_max)))
                cap_far = min(LFD_MAX_M, max(self.lfd_min,
                                             math.sqrt(LFD_CURVE_CAP_K * r_far)))
                v_corner = min(v_corner, OMEGA_N_MAX * cap_far / math.sqrt(2.0))
            brake_d = max(0.0, gate_dist - BRAKE_GATE_MARGIN)
            v_brake = math.sqrt(max(0.0, v_corner * v_corner
                                    + 2.0 * BRAKE_GATE_DECEL * brake_d))
            v_target = min(v_target, v_brake)

        # (b2) ω_n 결속 — 곡률캡이 속도표 LFD 보다 짧게 눌렀을 때만
        if lfd_win_only < lfd_speed - 0.05:
            v_target = min(v_target, OMEGA_N_MAX * lfd_win_only / math.sqrt(2.0))

        v_target = max(self.min_speed_ms, min(self.max_speed_ms, v_target))
        # 정수 펄스로 환산(구 kasa_units.ms_to_pulse 와 같은 반올림).
        # ★하한이 CORNER_MIN_PULSE(2)★ 인 이유는 그 상수 주석에 있다 — 1펄스로
        #   내려가면 이 차는 코너에서 아예 못 움직여 그 자리에 선다(실측).
        pulse = int(v_target / MS_PER_PULSE + 0.5)
        return max(min(CORNER_MIN_PULSE, self.drive_pulse),
                   min(self.drive_pulse, pulse))

    def steer_command(self, road_deg, v_ms):
        """도로휠각 → B보드 pot 지령. ★실측 역모델★ (상단 STEER_PLANT_GAIN 주석)

            pot = plant_gain · δ_road  +  understeer · v²/R,   R = L/tan(δ_road)

        순수추종 공식이 내는 것은 '이 곡률을 만들려면 도로휠을 몇 도 꺾어야 하나'
        이고, B보드가 받는 것은 가변저항 행정 기준 값이다. 둘이 1:1 이 아니라서
        (실측 링키지비 1.75) 이 변환이 없으면 늘 덜 꺾는다 — 2026-08-11 두 주행이
        코너에서 34° 가 필요한데 21° 만 내고 이탈한 원인이다.
        """
        d = abs(float(road_deg))
        if d < 1e-6:
            return 0.0
        pot = self.plant_gain * d + self.understeer * v_ms * v_ms \
            * math.tan(math.radians(d)) / self.wheelbase
        return math.copysign(min(STEER_MAX_DEG, pot), road_deg)

    def advance_wp_idx(self):
        """진행 포인터를 ★창 안의 최근접점★ 으로 옮긴다.
        [2026-08-11 구 white/driving.py 의 'WP 탐색' 이식]

        ★반경 방식을 버린 이유★ 예전에는 "현재 WP 를 반경 0.2m 안에서 지나쳤는가"로
        전진시켰다. 그런데 차가 경로에서 옆으로 0.2m 넘게 떨어져 지나가면 그 조건이
        영원히 성립하지 않아 ★포인터가 그 자리에 굳는다★ — 순수추종이 조향은 알아서
        하더라도 도착 판정과 CTE 창이 함께 멈춰 버린다(경로이탈 감지도 못 하게 된다).
        위치 기반 최근접 탐색은 옆으로 벗어난 채 지나가도 정상적으로 따라 올라간다.

        ★앞쪽을 우선하되 맹신하지 않는다★ 최근접점이 등 뒤일 수 있어서(코스가 되짚는
        구간) '앞쪽 최근접'을 먼저 보지만, 그게 전체 최근접보다 WP_AHEAD_PENALTY_M
        이상 나쁘면 전체 최근접을 쓴다 — 앞쪽만 고집하면 순환코스에서 반대편 구간으로
        건너뛴다.

        ★한 주기 전진 상한★ GPS 가 한 번 튀어도 포인터가 코스를 훌쩍 건너뛰지 않게
        막는다. 되돌아가지도 않는다(max 로 단조 증가).
        """
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

    def lookahead_m(self, d2goal, near_win=0.0, near_peak=0.0):
        """선행거리 LFD [m] = min(속도표, 곡률캡) → (lfd, lfd_win_only, lfd_speed)
        [2026-08-11 구 white/driving.py 의 LFD_TABLE ★설계식★ + 곡률캡 이식]

        ★표가 아니라 식을 옮긴 이유★ 구 white 의 LFD_TABLE 은 15행이지만, v≥2.2
        구간은 전부 "ω_n 을 0.97 로 유지" 라는 하나의 설계식(LFD = v·√2/ω_n)에서
        나온 값이다. 게다가 그 고속행들은 원본 주석에 ★'전부 미검증 추정값'★ 이라고
        적혀 있다. 식으로 두면 drive_pulse 를 바꿀 때 자동으로 따라오고, 표를 베껴
        오면서 미검증 숫자에 권위를 부여하지 않는다.
          검산 : v=2.65→3.86 / 3.54→5.16 / 4.42→6.44   (원표 3.85 / 5.15 / 6.40)
          저속(v≲1.6)은 식이 LFD_MIN 밑으로 내려가 어차피 하한에 클램프된다.

        ★왜 ω_n 을 묶는가★ 순수추종의 고유진동수는 ω_n = v·√2/LFD 다. 이게 커지면
        루프지연(조향 지령→실제 요레이트) 때문에 유효감쇠가 음(−)이 되어 조향이
        발산한다. ★속도만 올리고 LFD 를 고정하면 이 임계를 넘는다★ — 4펄스
        (3.54m/s)에서 LFD 를 2.3 으로 두면 ω_n 이 2.2 까지 올라간다.

        ══════════════════════════════════════════════════════════════════════
         ★ω_n 0.97 의 근거 — 금색차 실측으로 재확인했다 (2026-08-11)★
        ══════════════════════════════════════════════════════════════════════
        ω_n 0.97 과 "임계 1.2" 는 원래 ★1/5카(휠베이스 0.73m, 조향 ±21°)★ 의 로스백
        실측값이고 금색차 값이 아니었다. 그래서 record CSV 로 금색차의 지연을 직접
        재봤다 — 구 코드 변경로그가 "재시도하려면 조향각 실측 피드백을 추가해 τ 를
        추정이 아니라 직접 측정부터 하라"고 요구했던 그 측정이다. white806 의
        record 는 /steer_angle_measured(B보드 가변저항 실측각)를 남기므로 가능하다.

          ┌ rec_20260811_165756.csv, t=3.754s 의 조향 스텝(0 → −40°) ────────┐
          │  지령 → 실측 조향각 :  불감시간 0.250s / 63% 0.550s / 완료 0.750s │
          │                        슬루레이트 70°/s                          │
          │  지령 → 요레이트    :  10% 0.250s / ★63% τ=0.500s★ / 90% 0.800s │
          └──────────────────────────────────────────────────────────────────┘

        위상여유(PM) 판정식 : 2차계(ζ=0.707)의 지연 없는 PM 은 65° 이고, 순수지연은
        crossover(≈ω_n)에서 ω_n·τ [rad] 만큼 깎는다 →  PM ≈ 65° − ω_n·τ·(180/π).
          검증 : 이 식에 1/5카의 실측 임계 ω_n=1.2 를 넣으면 τ=0.95s 가 나온다.
                 구 코드가 3방법 교차검증으로 채택한 τ=0.93s 와 일치 → 식이 타당하다.

          금색차(τ=0.500s) 적용 :
            ω_n 0.97 → LFD 5.16m → PM ★+37°★   (1/5카였다면 +13° — 간신히)
            ω_n 1.22 → LFD 4.10m → PM  +30°
            ω_n 1.50 → LFD 3.33m → PM  +22°
            ω_n 2.26 → LFD 2.21m → PM   ★0°★ = 발산임계
          ★즉 금색차의 발산임계는 1.2 가 아니라 ≈2.3 이다★ 조향이 약 2배 빠르기
          때문(τ 0.93→0.50). 1/5카의 만성 사행이 바로 이 값에서 났던 것도 설명된다.
          그래서 물려받은 0.97 은 금색차에서 ★넉넉히 안전한 쪽★ 이다.

        ★τ=0.500s 은 보수적인 값이다★ 두 가지 이유로 실제 운용 τ 는 더 작다:
          ① 40° 풀스윙 측정치라 대부분이 슬루 시간이다(70°/s). 불감시간 0.250s 만
             진폭과 무관하고, 순수추종이 경로상 실제로 내는 조향은 5° 이내이므로
             τ_eff ≈ 0.29s → PM30° 허용 ω_n 이 2.0 을 넘는다.
          ② 그 스텝은 차가 거의 멈춘 상태(0.66 m/s, 직후 정지)에서 났다. 정지에
             가까울수록 조향 부하가 크므로 주행 중에는 더 빠를 것이다.

        ⚠️ 아직 못 재본 것 : ★닫힌루프 진동★ 이다. 그 로스백의 DRIVE_RUN 구간은
           수정 전 인덱스 조준으로 조향이 −40° 에 5.7초 포화된 채 제자리를 돈
           기록이라, 사행 주기·|CTE|·조향 반전율 같은 닫힌루프 통계는 전부 무의미
           하다(하드웨어 스텝응답만 건져 쓴 것이다). 실차 확인 절차는 아래 참고.

        ── 실차 튜닝 절차 ────────────────────────────────────────────────────
          1. 0.97 그대로 직선+완만코너를 한 번 달리고 record 를 본다.
          2. 조향이 0 주위로 왕복하면 ω_n 을 낮춘다. ★예상 사행주기는
             2π/ω_n = 6.5초(0.15Hz)★ 니 그 근처 주기가 보이면 그게 이 진동이다.
          3. 진동이 없고 |CTE| 만 크면(아래 불감대 하한 참고) 1.22 까지 올려도 된다
             — PM 30° 가 남는다. 1.5 를 넘길 때는 반드시 로스백으로 재확인할 것.
          4. ω_n 은 ★직선 구간만★ 정한다 — 코너에서는 아래 곡률캡이 LFD 를 더
             짧게 눌러 버리므로 이 값이 지배하지 않는다.

        ── ★조향 불감대가 만드는 추종정확도 하한 (B보드 하드웨어 제약)★ ──────
          kasa_0804_B.ino 의 조향 PD 는 STEER_TOLERANCE_EXIT=6 counts 이내면 모터를
          돌리지 않는다. 가변저항 유효범위가 194 counts/80° 이므로 ★2.47° 불감대★ 다
          (SETTLE_MS=500 이면 아예 ST_SETTLED 로 모터가 꺼진다).
            LFD 5.16m → 2.47° 에 해당하는 측방오차 ★0.46m★
            LFD 4.10m → 0.29m      LFD 3.00m → 0.16m
          ★이 값 이하의 CTE 에는 조향이 물리적으로 반응하지 않는다★ 즉 |CTE| 를
          0.5m 밑으로 줄이고 싶으면 ω_n 을 올려 LFD 를 줄이는 것이 유일한 수단이고,
          그것이 이 차의 정확도 ↔ 안정성 실제 trade-off 다. (경로이탈 정지 임계
          CTE_DEVIATION_M=2.0m 는 이 하한보다 훨씬 크므로 오작동하지 않는다.)

        ★종점 접근 캡★ 남은 거리보다 멀리 겨누면 목표점이 마지막 WP 에 붙박이가 되고,
        가까워질수록 alpha 가 커져 도착 직전에 조향이 크게 흔들린다. 남은 거리에 맞춰
        함께 줄인다(구 white 의 APPROACH_LFD 캡과 같은 식).

        ★측정속도가 아니라 지령속도를 쓴다★ 엔코더 실측을 넣으면 LFD 가 노이즈로
        떨리고, 출발 램프 구간에서 속도가 낮다는 이유로 LFD 가 짧아져 바로 이 이식이
        고치려던 '목표가 너무 가까운' 상황을 되살린다. 구 white 도 지령값을 썼다.

        ══════════════════════════════════════════════════════════════════════
         ★곡률캡★ [2026-08-11 구 white 이식] — 코너에서 조향 권한을 되찾는 장치
        ══════════════════════════════════════════════════════════════════════
        코너컷 오차 ≈ LFD²/(8R) 이므로 LFD 를 √(K·R) 로 누르면 CTE ≤ K/8 ≈ 0.19m 다.
        ★그런데 금색차에서는 그보다 더 중요한 효과가 있다★ 순수추종이 낼 수 있는
        최대 도로휠각은 atan(2L/LFD) 이므로, LFD 가 길면 아무리 벗어나도 조향을 다
        쓰지 못한다:
              LFD 5.16m → 최대 25.9°  |  LFD 2.98m → 40.0°(pot 상한 전부)
        2026-08-11 두 주행이 코너에서 −25.6°/−21.0° 에서 멈춘 것이 이 상한이었다.
        코너에서 LFD 를 줄이면 그 상한이 함께 올라가 큰 곡률을 요구할 수 있게 된다.

        반환값이 3개인 이유 : (사용할 LFD, 평활곡률만 반영한 LFD, 속도표 LFD).
        뒤의 둘은 corner_speed() 의 ω_n 결속 판정에 쓴다 — 피크(짧은 필렛)로 인한
        순간 단축까지 속도결속에 넣으면 매 필렛마다 과잉 감속하므로, 결속은 지속
        곡률(평활 창)만 본다. 구 white 의 lfd_win_only 와 같은 이유·같은 구조다.
        """
        v = max(0.1, self.drive_pulse * MS_PER_PULSE)
        lfd_speed = v * math.sqrt(2.0) / self.lfd_omega_n

        def curve_cap(demand_deg):
            if demand_deg <= 2.5:
                return LFD_MAX_M
            r = self.wheelbase / math.tan(math.radians(min(demand_deg, self.road_max)))
            return math.sqrt(LFD_CURVE_CAP_K * r)

        lfd_win_only = min(LFD_MAX_M, max(self.lfd_min,
                                          min(lfd_speed, curve_cap(near_win))))
        target = min(LFD_MAX_M, max(self.lfd_min,
                                    min(lfd_speed, curve_cap(near_peak))))

        # 급변 완화 + 비대칭 슬루(감소는 신속 = 코너 보호, 증가는 완만 = 탈출 후
        # 게인 점프 방지). 구 white 와 같은 값.
        dt = 1.0 / CONTROL_HZ
        self._lfd_lpf += LFD_LPF_ALPHA * (target - self._lfd_lpf)
        self._lfd_out = min(self._lfd_out + LFD_RATE_UP * dt,
                            max(self._lfd_out - LFD_RATE_DOWN * dt, self._lfd_lpf))
        lfd = max(self.lfd_min, min(LFD_MAX_M, self._lfd_out))

        # 종점 접근 캡은 마지막에 — 위 슬루 상태를 오염시키지 않는다
        lfd = min(lfd, max(LFD_GOAL_MIN, d2goal * LFD_GOAL_A + LFD_GOAL_B))
        return max(self.lfd_min, lfd), lfd_win_only, lfd_speed

    def pure_pursuit_steer(self, lfd):
        """순수추종이 요구하는 ★도로휠각★ [deg]. ★− 좌 / + 우 (B보드 규약)★
        [2026-08-11 구 white/driving.py 의 '제어 1. Pure Pursuit' 이식]

        ★반환값은 pot 지령이 아니라 도로휠각이다★ 발행 전에 steer_command() 로
        전달계 보정(링키지 1.75 + 언더스티어)을 거쳐야 한다. 그래서 여기서 클램프하는
        상한도 40° 가 아니라 도로휠각 상한 self.road_max(=40/1.75≈22.8°) 다.

        ★인덱스 조준을 이걸로 바꾼 이유 (파일 헤더의 실차 기록과 같은 사건)★
        예전에는 wp_idx 가 가리키는 점의 방위를 그대로 겨눴다(steer = −Kp·오차).
        맵 간격이 0.3m 라 그 점은 늘 코앞이고, 그래서 차가 경로에서 옆으로 0.8m 만
        떨어져 출발해도 방위오차가 90° 에 가까워진다 — 조향이 포화된 채 최소회전
        반경으로 돌기만 하고, 매 틱 새로 닿는 점의 기하가 똑같아서 빠져나오지 못한다.

        순수추종은 목표를 ★항상 LFD 이상 떨어진 앞쪽 점★ 으로 잡아 그 상황 자체를
        만들지 않는다. 같은 0.8m 오프셋도 5m 앞 점 기준이면 alpha ≈ atan(0.8/5) ≈ 9°
        다. 조향식 δ = atan(2·L·sin α / d) 는 alpha 가 커져도 완만하게 포화한다.

        ★LFD 가 길면 조향을 다 쓰지 못한다★ 이 공식의 최대값은 atan(2L/LFD) 이므로
        LFD 5.16m 에서는 아무리 벗어나도 25.9° 밖에 못 낸다(도로휠 상한 22.8° 는
        넘으니 직선·완만구간에서는 문제가 안 된다). 급코너에서 부족해지는 것은
        lookahead_m() 의 곡률캡이 LFD 를 줄여서 해결한다 — 그 주석 참고.
        """
        n = len(self.waypoints)
        ch = math.cos(math.radians(self.heading))
        sh = math.sin(math.radians(self.heading))

        def to_body(i):
            """WP → 차체기준 (전방 lx, 왼쪽 ly)."""
            wx, wy = self.waypoints[i]
            dx, dy = wx - self.x, wy - self.y
            return dx * ch + dy * sh, -dx * sh + dy * ch

        # ① LFD 이상 떨어진 첫 '앞쪽' 점. 없으면(종점 근처) 마지막 점을 쓴다.
        tgt = n - 1
        for i in range(self.wp_idx, n):
            lx, ly = to_body(i)
            if lx > 0.0 and math.hypot(lx, ly) >= lfd:
                tgt = i
                break

        lx, ly = to_body(tgt)

        # ② 목표가 뒤/옆이면 한 점 앞으로 대체한다 — alpha 가 ±180° 로 튀어
        #    조향이 한 틱만에 반전하는 것을 막는다.
        if lx <= 0.1:
            tgt = min(self.wp_idx + 1, n - 1)
            lx, ly = to_body(tgt)
            if lx <= 0.05:
                self._diag_target_idx = tgt
                self._diag_target_dist = math.hypot(lx, ly)
                self._diag_head_err = 0.0
                return 0.0

        dist = math.hypot(lx, ly)
        # 분모는 ★실제 목표까지 거리★ 다(정석 순수추종 κ = 2·sin α / 현). 종점
        # 근처에서 목표가 LFD 보다 가까이 붙을 때 0 으로 내려가지 않게만 막는다.
        denom = max(lfd * 0.5, dist)

        alpha = math.atan2(ly, lx)
        # ★부호★ alpha>0 은 목표가 왼쪽이라는 뜻이고 B보드는 −가 좌회전이므로 뒤집는다.
        steer = -math.degrees(math.atan2(2.0 * self.wheelbase * math.sin(alpha), denom))

        # 진단 : 조향이 무엇을 겨눴고(target_idx), 얼마나 앞이었고(target_dist =
        #   실효 선행거리), 입력 오차가 얼마였는지. ★head_err 의 의미가 바뀌었다★ —
        #   예전엔 '인덱스 WP 방위오차', 지금은 '선행 목표점의 차체기준 방위(alpha)'.
        self._diag_target_idx = tgt
        self._diag_target_dist = dist
        self._diag_head_err = math.degrees(alpha)
        # ★도로휠각 상한으로 클램프★ pot 상한(40°)이 아니다 — steer_command() 가
        #   여기 값에 링키지비를 곱해 pot 지령을 만들고 거기서 40° 로 클램프한다.
        return max(-self.road_max, min(self.road_max, steer))

    # ══════════════════════════════════════════════════════════════════════════
    #  출력
    # ══════════════════════════════════════════════════════════════════════════
    def send(self, target_pulse, steer_deg, control):
        """목표펄스·조향을 그대로 내보낸다.

        ★[2026-08-11] 여기서 브레이크를 만지지 않는다★ 예전에는 상태에 따라
        apply_brake_policy() 를 태워 '현재펄스가 목표보다 3 이상 크면 리니어 2단 +
        목표를 0 으로 덮기'를 했는데, 그 정책 자체를 삭제했다 — 세 번 물려서 세 번
        다 A보드 기동 블랭킹과 싸우는 되먹임이 됐다(파일 헤더 '리니어 브레이크' 절에
        실측 근거와 함께 기록). 자율주행 중 감속은 곡률 선행제동(corner_speed 가
        목표펄스를 미리 낮춘다) + 자연감속(코스트)이 전담하고, 리니어는 DRIVE_DONE
        에서만 물린다(enter() 가 직접 set_brake(BRAKE_FULL))."""
        target_pulse = int(max(0, min(15, target_pulse)))
        if self.state == S_DRIVE_DONE:
            target_pulse = 0                       # 도착 후에는 무조건 0

        msg = Twist()
        msg.linear.x = float(target_pulse)         # ★펄스 그대로 (m/s 아님)★
        msg.angular.z = float(steer_deg)           # ★− 좌 / + 우★
        self.pub_cmd.publish(msg)
        self.pub_state.publish(Bool(data=bool(control)))

    def set_brake(self, level):
        if level == self.brake_now:
            return
        self.brake_now = level
        self.pub_brake.publish(Int32(data=int(level)))

    def signed_cte(self):
        """경로에서 얼마나 벗어나 있는가. ★+ 왼쪽 / − 오른쪽★ (경로 진행방향 기준)

        [2026-08-11] ★제어가 읽는 유일한 CTE 용도는 안전정지뿐이다★ run_follow()
        가 |CTE| > CTE_DEVIATION_M 이면 조향과 무관하게 즉시 정지한다. 그 밖의
        조향 계산은 여전히 목표점 방위만 보고 하므로(순수추종 기하 없음), 이 값이
        경로를 따라가는 데 직접 쓰이지는 않는다 — '너무 벗어났는가'만 본다.
        /drive_diag 로도 계속 나가므로 사후 진단에도 쓴다.

        wp_idx 주변 창만 훑는다(CTE_WINDOW_WP). 경로 전체를 매 주기 훑으면 점 수에
        비례해 느려지는데, 지금 위치에서 먼 구간의 최근접점은 어차피 의미가 없다.
        ★순환 코스에서 반대편 구간을 최근접으로 집는 것도 이 창이 막아 준다★
        """
        n = len(self.waypoints)
        if n < 2:
            return float('nan')
        lo = max(0, self.wp_idx - CTE_WINDOW_WP)
        hi = min(n - 1, self.wp_idx + CTE_WINDOW_WP)
        best_d, best_signed = float('inf'), float('nan')
        for i in range(lo, hi):
            ax, ay = self.waypoints[i]
            bx, by = self.waypoints[i + 1]
            vx, vy = bx - ax, by - ay
            L2 = vx * vx + vy * vy
            if L2 <= 0.0:
                continue
            # 선분에 내린 수선의 발(구간 밖이면 끝점으로 잘라낸다)
            u = max(0.0, min(1.0, ((self.x - ax) * vx + (self.y - ay) * vy) / L2))
            ex, ey = self.x - (ax + u * vx), self.y - (ay + u * vy)
            d = math.hypot(ex, ey)
            if d < best_d:
                # 외적 z = v × e. 진행방향 v 기준으로 e 가 반시계(왼쪽)면 +
                best_d = d
                best_signed = math.copysign(d, vx * ey - vy * ex)
        return best_signed

    def publish_state_topics(self):
        self.pub_dstate.publish(String(data=self.state))
        ego = Float64MultiArray()
        ego.data = [
            float(self.x), float(self.y),
            float(self.heading if self.heading is not None else 0.0),
            float(self.enc_pulse),
            float(self.wp_idx), float(len(self.waypoints)),
            1.0 if self.fix_ok else 0.0,
        ]
        self.pub_ego.publish(ego)

        # ── 추종 진단 (/drive_diag) — record 가 CSV 로 받아 적는다 ──
        goal_d = float('nan')
        if self.waypoints:
            gx, gy = self.waypoints[-1]
            goal_d = math.hypot(gx - self.x, gy - self.y)
        hd0, sg0, rs0, ds0 = self._diag_init
        diag = Float64MultiArray()
        diag.data = [
            float(self.signed_cte()),          # cte_m           ★핵심★
            float(self._diag_head_err),        # heading_err_deg
            float(self._diag_target_idx),      # target_idx
            float(self._diag_target_dist),     # target_dist_m
            float(goal_d),                     # goal_dist_m
            float(self._diag_course),          # gps_course_deg
            float(self._diag_fuse),            # fuse_corr_deg
            float(math.degrees(self.gyro_z)),  # gyro_z_dps
            # brake_latched — [2026-08-11] 래치 개념이 사라져(감속 정책 삭제) 지금은
            #   '리니어가 물려 있나(=DRIVE_DONE)'를 싣는다. record.py 의 열 이름과
            #   의미가 호환되고 열 개수(13)도 그대로다.
            1.0 if self.brake_now == BRAKE_FULL else 0.0,
            float(hd0), float(sg0), float(rs0), float(ds0),   # head_init_*
        ]
        self.pub_diag.publish(diag)

    def event(self, text):
        self.pub_event.publish(String(data=text))
        self.get_logger().info(text)

    def throttle_event(self, text):
        self.get_logger().warning(text, throttle_duration_sec=3.0)

    # ══════════════════════════════════════════════════════════════════════════
    def destroy_node(self):
        # ★내려갈 때 리니어를 풀어 준다★ 물린 채로 죽으면 사람이 차를 못 민다.
        try:
            self.pub_brake.publish(Int32(data=BRAKE_NONE))
            self.pub_state.publish(Bool(data=False))
            self.pub_cmd.publish(Twist())
            # 시리얼로 실제 나갈 시간을 준다(arduino 노드가 아직 살아 있을 때).
            end = time.time() + 0.4
            while time.time() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DrivingNode()
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
