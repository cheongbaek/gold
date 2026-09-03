#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""1/5카(헤네스 브룬 T870) 차량 단위 환산 — ★이 파일이 환산 상수의 단일 소유자다★

white2 는 white 패키지를 kasa(금색차) 이식 이전, ★1/5카 전용★ 규약으로 되돌린 판이다.
white/kasa_units.py 가 gps_imu.py / sensor_monitor.py / driving.py 세 곳에 흩어져 있던
kasa 환산 상수를 한 곳으로 모았던 것처럼, 이 파일은 그 구조는 그대로 두고 값만
1/5카 원본(white/motor.py — protocol 참고용으로 남아있던 파일)으로 되돌린다.

═══════════════════════════════════════════════════════════════════════════════
 1. 속도 피드백 : /encoder (Int32)
═══════════════════════════════════════════════════════════════════════════════
 motor.py 가 아두이노 Mega의 "E,d_val[,dt_us]\n" 를 그대로 실어 보낸다.

   · d_val 은 ★부호 있는 단일값★(좌우 합이 아니다) — 10ms 창(ENC_DT)당 틱수다.
     kasa 처럼 좌+우를 더하지 않는다. 후진도 이 부호로 그대로 표현된다
     (kasa 는 후진이 수동조종 전용이라 부호 없는 카운트를 썼지만, 1/5카는 원래
      자율주행 중에도 부호 있는 카운트를 받는다).
   · 300틱/회전 광학·자기 엔코더 — 홀센서 XOR 합산 방식이 아니다.

═══════════════════════════════════════════════════════════════════════════════
 2. 주행 명령 : /cmd_vel_raw linear.x = ★m/s (펄스가 아니다)★
═══════════════════════════════════════════════════════════════════════════════
 motor.py 의 cb_cmd_vel 은 "linear.x = m/s, angular.z = deg" 를 그대로 받아
 자기 안에서만 tick/10ms 로 변환해 아두이노에 보낸다("C,tick,steer\n"). 즉
 ROS 계층(=driving.py 가 발행하는 값)에는 펄스 양자화가 없다 — kasa 이식 때
 도입된 ms_to_pulse()/pulse_to_ms() 는 이 차량에는 없는 개념이라 이 파일에는
 없다. 속도 상한은 motor.py 의 MAX_SPEED_MS(5.0 m/s, 프로토콜 안전판)이고,
 실제 이 차량이 낼 수 있는 최고속도는 8km/h(2.22 m/s) 뿐이다 — GAIN_TABLE /
 LFD_TABLE 이 2.2 m/s 행에서 끝나는 이유이기도 하다(그 이상은 물리적으로 도달
 불가라 실측 자체가 없다).

═══════════════════════════════════════════════════════════════════════════════
 3. 조향 부호 : ★ROS 토픽은 1/5카 규약 = 양수 좌회전 / 음수 우회전★
═══════════════════════════════════════════════════════════════════════════════
 kasa 이식 때(2026-08-04) "화면·토픽·시리얼·펌웨어를 전부 kasa B보드 부호(−좌/+우)로
 통일"하며 발행 직전 부호를 뒤집는 to_ros_steer() 가 생겼다. 1/5카는 애초에 그럴
 필요가 없다 — motor.py 의 tx_loop 는 cmd_steer 를 받은 그대로("C,tick,steer\n")
 시리얼에 실어 보내고 어디서도 반전하지 않는다. 즉:

   ROS 토픽 (/cmd_vel_raw.angular.z, /steer_angle_measured) : ★+ 좌 / − 우★
   motor.py 시리얼 ("C,tick,steer")                          : + 좌 / − 우  (동일, 반전 없음)

 driving.py 의 순수추종·PID·조향게인(STEER_PLANT_GAIN_L/R)·트림도 전부 이 '+좌'
 기준(δ_ctrl)으로 실측·튜닝된 값이므로, 내부 계산과 발행값이 애초부터 같은 부호다.
 그래서 이 파일에는 부호를 뒤집는 함수(to_ros_steer)가 없다 — clamp_steer_deg() 가
 부호는 건드리지 않고 범위만 ±STEER_MAX_DEG 로 자른다.
"""

import math

# ═══════════════════════════════════════════════════════════════════════════
#  차량 제원 (white/motor.py — 1/5카 원본 프로토콜 상수)
# ═══════════════════════════════════════════════════════════════════════════
WHEEL_CIRCUMFERENCE_M = 0.8482    # [m] 바퀴 둘레 (motor.py WHEEL_CIRC 실측값)
WHEEL_DIAMETER_M      = WHEEL_CIRCUMFERENCE_M / math.pi     # ≈ 0.270 m (참고용 역산값)

# 엔코더 : 광학/자기식, 1회전 300틱. /encoder 는 좌우 합이 아니라 부호 있는 단일값이다.
ENCODER_COUNTS_PER_REV = 300

# 아두이노 속도 PID 주기 = 엔코더 계측 창 (motor.py ENC_DT)
PULSE_WINDOW_S        = 0.01     # 10ms

# 축거 (구 white/driving.py 실측값, kasa 이식 전)
WHEELBASE_M           = 0.73     # [m] 휠베이스 730mm

# ═══════════════════════════════════════════════════════════════════════════
#  프로토콜 한계 (white/motor.py)
# ═══════════════════════════════════════════════════════════════════════════
STEER_MAX_DEG       = 21          # [deg] motor.py MAX_STEER_DEG
MAX_SPEED_MS_LIMIT  = 5.0         # [m/s] motor.py MAX_SPEED_MS — 시리얼 프로토콜 안전판.
#   ★실차 상한은 이보다 훨씬 낮다★ 이 차량이 실제로 낼 수 있는 최고속도는
#   8km/h(2.22 m/s)뿐이다(GAIN_TABLE/LFD_TABLE 이 2.2 행에서 끝나는 이유). 5.0 은
#   "아두이노가 받아줄 수 있는 값의 상한"이지 "차가 낼 수 있는 속도"가 아니다.

# ═══════════════════════════════════════════════════════════════════════════
#  파생 환산값
# ═══════════════════════════════════════════════════════════════════════════
# /encoder 1틱(10ms 창) 당 속도 — 피드백 단위이자 명령 단위(1/5카는 펄스 양자화가
# 없으므로 명령측 MS_PER_PULSE 같은 상수가 따로 필요 없다).
MS_PER_ENCODER_COUNT = WHEEL_CIRCUMFERENCE_M / ENCODER_COUNTS_PER_REV / PULSE_WINDOW_S  # ≈0.28273


# ═══════════════════════════════════════════════════════════════════════════
#  변환 함수
# ═══════════════════════════════════════════════════════════════════════════
def encoder_count_to_ms(counts: float) -> float:
    """/encoder 틱(부호 있는 단일값) → m/s. 부호가 있으면 그대로 유지한다."""
    return float(counts) * MS_PER_ENCODER_COUNT


def ms_to_encoder_count(speed_ms: float) -> float:
    """m/s → /encoder 틱 (실수). 디버그 표시·역환산용."""
    return float(speed_ms) / MS_PER_ENCODER_COUNT


def clamp_steer_deg(deg: float) -> float:
    """조향각을 motor.py 수용 범위(±STEER_MAX_DEG)로 클램프. ★부호는 건드리지 않는다★

    1/5카는 ROS 규약과 내부 제어 규약이 이미 같은 부호(+좌/−우)라 뒤집을 필요가
    없다 — kasa 판의 to_ros_steer() 에 대응하는 부호반전 함수가 이 파일에는 없다."""
    return max(-float(STEER_MAX_DEG), min(float(STEER_MAX_DEG), float(deg)))
