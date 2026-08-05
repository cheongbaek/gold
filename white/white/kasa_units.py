#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""kasa 차량 단위 환산 — ★이 파일이 환산 상수의 단일 소유자다★

이전(white 차량, 300틱/회전 엔코더)에는 같은 환산식이 driving.py / gps_imu.py /
sensor_monitor.py 세 곳에 각자 하드코딩되어 있었다. kasa 차량으로 옮기면서 세 곳을
전부 여기로 모았다 — 한 곳만 고치고 나머지를 놓치면 표시값만 틀리거나(sensor_monitor),
더 나쁘게는 DR 위치가 조용히 어긋난다(gps_imu).

═══════════════════════════════════════════════════════════════════════════════
 1. 속도 피드백 : /encoder (Int32)
═══════════════════════════════════════════════════════════════════════════════
 nxde/arduino.py 가 A보드 텔레메트리 "S,<좌펄스>,<우펄스>,<쓰로틀>" 의
 ★좌 + 우 (합)★ 을 실어 보낸다. 평균이 아니다.

   · 왜 합인가 : /encoder 가 Int32 라서 평균은 (0,1) → 0.5 → 정수화로 정보가 깨진다.
     합은 그 손실이 없고 양자화 눈금도 절반이 된다. 그리고 아래 ENCODER_COUNTS_PER_REV
     를 2배(192)로 잡으면 결과 m/s 는 평균과 **완전히 동일**하다.
   · 부호 없음 : 후진은 수동조종으로만 하기로 했고, A보드 펌웨어가 부호 없는 카운트를
     보낸다. 따라서 /encoder 는 항상 0 이상이다(white 차량에서는 부호가 있었다).

 ★★ 홀센서 PPR = 96 (64 가 아니다) ★★
   인휠모터는 3상 홀센서 3개를 SN74HC86N(XOR)으로 합산한 신호 1개를 낸다.
     · 3상 홀 조합은 전기적 1회전당 6개 상태를 가진다
     · 상태가 바뀔 때마다 정확히 한 신호만 반전되므로 XOR 출력도 매 전이마다 토글
       → 전기적 1회전당 6 에지
     · CHANGE 인터럽트는 상승·하강을 모두 세므로 6 카운트
     · 16극쌍 → 기계적 1회전당 6 × 16 = 96 카운트
   CLAUDE.md 는 오래 '4엣지 × 16 = 64' 로 적혀 있었는데, 4엣지는 XOR 대상이 2신호일
   때의 값이다. 96 이 맞는 근거 — 실측과 일치한다:
     1.697147 m / 96 / 0.020 s = 0.8839 m/s = 3.182 km/h  per 펄스
     kasa_ws master.py 의 실측 상수 PULSE_TO_KMH = 3.18 과 소수 둘째자리까지 같다.
     (64 로 계산하면 4.77 km/h 가 되어 실측의 정확히 1.5배 = 96/64 만큼 어긋난다)

═══════════════════════════════════════════════════════════════════════════════
 2. 주행 명령 : /cmd_vel_raw linear.x = ★펄스 목표값 0~15 (m/s 가 아니다)★
═══════════════════════════════════════════════════════════════════════════════
 A보드는 정수 펄스 목표를 받아 자체 PID 로 그 속도를 유지한다. 그래서 명령 분해능이
 1펄스 = 0.884 m/s (3.18 km/h) 로 거칠다. driving.py 내부 제어는 계속 m/s 로 돌고,
 발행 직전에 ms_to_pulse() 로 환산한다 — 튜닝 자산(GAIN_TABLE/LFD_TABLE 등)을 모두
 물리 단위로 남기기 위함이다.

   운용 상한별 사용 가능한 단계 수 (min = 1펄스 고정):
     max_speed_ms 1.77 → 펄스 1~2   ( 6.4 km/h)  2단계
     max_speed_ms 2.65 → 펄스 1~3   ( 9.5 km/h)  3단계  ← 현재 기본값
     max_speed_ms 4.42 → 펄스 1~5   (15.9 km/h)  5단계
     max_speed_ms 8.84 → 펄스 1~10  (31.8 km/h) 10단계
   ★ 상한을 2.2 m/s 이상으로 올릴 때 주의 ★ driving.py 의 GAIN_TABLE / LFD_TABLE 은
   최상단 행이 2.2 m/s 다. 그 위 속도는 최상단 행 값으로 클램프되어 스케줄이 끊긴다.
   본격적으로 속도를 올릴 때는 실차 로스백으로 그 위 행을 채워야 한다.

═══════════════════════════════════════════════════════════════════════════════
 3. 조향 부호 : ★ROS 토픽은 kasa 규약 = 음수 좌회전 / 양수 우회전★
═══════════════════════════════════════════════════════════════════════════════
 ── [2026-08-04 개정] 규약을 뒤집었다 ──
 이전 판은 "ROS 안은 white 부호(+좌)로 두고, 시리얼 전송 직전에 arduino.py 가 반전"
 이었다. 그런데 그러면 **사람이 보는 화면과 부호가 반대가 된다**:
   · GUI 의 가로 조향 레버는 왼쪽 끝이 −40, 오른쪽 끝이 +40 이다(당연한 배치).
   · 그 레버를 오른쪽(+)으로 밀면 white 부호에서는 '좌회전' 명령이 된다.
   → 실제로 nxde master 로 시험했을 때 레버 방향과 바퀴 방향이 반대로 나왔다.
 그래서 **ROS 토픽 전체를 kasa B보드 부호로 통일**한다. 화면·토픽·시리얼·펌웨어가
 전부 같은 부호를 쓰므로 더 이상 어디서 뒤집히는지 추적할 필요가 없다.

   ROS 토픽 (/cmd_vel_raw.angular.z, /steer_angle_measured) : ★− 좌 / + 우★
   kasa B보드 시리얼                                        : − 좌 / + 우  (동일)
     (kasa_0804_B.ino angleToPot: −40 → RAW_LEFT_LIMIT(576) 쪽 = 왼쪽 끝)
   → nxde/arduino.py 의 steer_invert 기본값은 **False**(반전 없음)가 되었다.

 ⚠️ 단 하나 예외가 있다 : ★driving.py 의 제어기 내부 계산★
   순수추종·PID·조향게인(STEER_PLANT_GAIN_L/R)·트림은 전부 **+좌 기준(δ_ctrl)** 으로
   튜닝되어 있다. 그 수치들의 의미를 보존하기 위해 내부는 그대로 두고,
   **발행 직전 딱 한 번** to_ros_steer() 로 뒤집는다(driving.publish_cmd).
   즉 부호가 뒤집히는 지점은 코드 전체에서 그 한 줄뿐이다.
"""

import math

# ═══════════════════════════════════════════════════════════════════════════
#  차량 제원 (실측)
# ═══════════════════════════════════════════════════════════════════════════
# 타이어 175/60R13 : 림 13인치 = 330.2mm, 사이드월 175 × 0.60 = 105mm (양쪽)
#   → 외경 330.2 + 2×105 = 540.2mm
WHEEL_DIAMETER_M      = 0.5402
WHEEL_CIRCUMFERENCE_M = math.pi * WHEEL_DIAMETER_M      # 1.697147 m

# 인휠모터 : QSWP72V5000W(QS260 계열), 16극쌍, 허브 직결(감속기 없음)
POLE_PAIRS            = 16
XOR_EDGES_PER_E_REV   = 6                               # 3상 홀 XOR + CHANGE 인터럽트
HALL_PULSES_PER_REV   = XOR_EDGES_PER_E_REV * POLE_PAIRS   # 96 (바퀴 하나당)

# /encoder 는 좌+우 합이라 유효 분해능이 2배다
ENCODER_COUNTS_PER_REV = HALL_PULSES_PER_REV * 2        # 192

# A보드 인휠 제어주기 = 펄스 계측 창
PULSE_WINDOW_S        = 0.020

# 축거·윤거 (kasa_ws master.py 의 디퍼렌셜 계산에 쓰인 실측값)
WHEELBASE_M           = 1.25    # 축거 1250mm
TRACK_WIDTH_M         = 1.10    # 윤거 1100mm

# ═══════════════════════════════════════════════════════════════════════════
#  프로토콜 한계 (kasa_0730_A.ino / kasa_0804_B.ino)
# ═══════════════════════════════════════════════════════════════════════════
PULSE_MAX      = 15     # A보드 단일값 입력 상한 (16~255 는 '직접 PWM' 무보호 경로)
STEER_MAX_DEG  = 40     # B보드 STEER_ANGLE_MAX

# ═══════════════════════════════════════════════════════════════════════════
#  파생 환산값
# ═══════════════════════════════════════════════════════════════════════════
# 바퀴 하나의 1펄스(20ms 창) 당 속도 — /cmd_vel_raw 명령 단위
MS_PER_PULSE = WHEEL_CIRCUMFERENCE_M / HALL_PULSES_PER_REV / PULSE_WINDOW_S    # 0.88393
# /encoder 1카운트(좌+우 합) 당 속도 — 피드백 단위
MS_PER_ENCODER_COUNT = (WHEEL_CIRCUMFERENCE_M
                        / ENCODER_COUNTS_PER_REV / PULSE_WINDOW_S)             # 0.44197

KMH_PER_PULSE = MS_PER_PULSE * 3.6            # 3.182 — master.py 실측 3.18 과 일치
MAX_SPEED_MS_LIMIT = MS_PER_PULSE * PULSE_MAX  # 13.26 m/s = 47.7 km/h (차량 상한)


# ═══════════════════════════════════════════════════════════════════════════
#  변환 함수
# ═══════════════════════════════════════════════════════════════════════════
def encoder_count_to_ms(counts: float) -> float:
    """/encoder 카운트(좌+우 합) → m/s. 부호가 있으면 그대로 유지한다."""
    return float(counts) * MS_PER_ENCODER_COUNT


def ms_to_encoder_count(speed_ms: float) -> float:
    """m/s → /encoder 카운트 (실수). 디버그 표시·역환산용."""
    return float(speed_ms) / MS_PER_ENCODER_COUNT


def ms_to_pulse(speed_ms: float) -> int:
    """m/s → A보드 주행 목표펄스 (정수 0~PULSE_MAX). /cmd_vel_raw 발행 직전에 쓴다.

    ★ 단순 반올림이다 ★ white/motor.py 는 "v>0 인데 환산값이 1 미만이면 1로 올린다"는
    최소 구동 보정을 했지만, 여기서는 하지 않는다 — 1펄스가 0.88 m/s(3.2 km/h)나 되므로
    0.2 m/s 를 원하는 계획값을 1펄스로 올리면 4배 과속이 된다. 반올림 결과가 0 이면
    '정지'가 맞다. 최저 주행속도 보장은 driving.py 의 min_speed_ms 가 담당한다
    (min_speed_ms 를 MS_PER_PULSE 이상으로 두면 0 으로 떨어지지 않는다).

    후진은 없다 — 음수는 0 으로 클램프한다(A보드가 음수를 받지 않는다).
    """
    if speed_ms <= 0.0:
        return 0
    pulse = int(speed_ms / MS_PER_PULSE + 0.5)      # 아두이노 round() 와 같은 반올림
    return max(0, min(PULSE_MAX, pulse))


def pulse_to_ms(pulse: float) -> float:
    """A보드 주행 목표펄스 → m/s. 디버그 표시·상한 산출용."""
    return float(pulse) * MS_PER_PULSE


def clamp_steer_deg(deg: float) -> float:
    """조향각을 B보드 수용 범위(±STEER_MAX_DEG)로 클램프. ★부호는 건드리지 않는다★

    이미 ROS 규약(− 좌 / + 우)인 값에 쓴다 — 예: camera_judgment 가 driving 의
    angular.z 를 그대로 통과시킬 때."""
    return max(-float(STEER_MAX_DEG), min(float(STEER_MAX_DEG), float(deg)))


def to_ros_steer(ctrl_deg: float) -> float:
    """★제어기 내부 부호(+좌) → ROS 토픽 부호(− 좌 / + 우)★ 로 변환하고 클램프한다.

    ★★ 코드 전체에서 조향 부호가 뒤집히는 지점은 여기 하나뿐이다 ★★
    호출처도 하나여야 한다 — driving.publish_cmd 의 msg.angular.z 대입.
    두 번 호출하면 이중 반전으로 조용히 좌우가 바뀐다.

    왜 driving 내부만 다른 부호를 쓰는가 :
      순수추종 기하(atan2 기반), PID, STEER_PLANT_GAIN_L/R, STEER_TRIM_DEG 가 모두
      '+ = 좌회전' 전제로 실측·튜닝된 값이다. 내부를 뒤집으면 그 수치들의 의미가
      전부 반대가 되어 로스백 분석 기록과 대조할 수 없게 된다.
      그래서 내부는 보존하고 출력단에서만 규약을 맞춘다. 파일 헤더 3절 참고.
    """
    return clamp_steer_deg(-float(ctrl_deg))
