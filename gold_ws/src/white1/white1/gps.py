#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gps.py ― white1 GPS 후처리 [원시 /fix → 품질 판정 + 이상치 게이트 + DR 융합 → /gps_fused]
════════════════════════════════════════════════════════════════════════════════
    ros2 run white1 gps          (one_launch.py 가 띄운다)

이 노드가 하는 일은 네 가지다.

  ① ★품질 판정★ — 지금 이 fix 가 RTK Fixed 인가 Float 인가 DGPS 인가.
     `status.status` 만으로는 ★Fixed 와 Float 이 구별되지 않는다★(아래 ①절).
  ② ★이상치 게이트★ — 말이 안 되는 fix 를 버린다. 물리·방향·품질 세 가지로 본다(②절).
  ③ ★공백 메움★ — fix 사이의 빈 시간을 IMU 로 외삽해 제어 주기(20Hz)에 맞춘다(③절).
  ④ ★품질 저하 구간 융합★ — status 가 계속 0/1(비RTK)이면 fix 하나하나를 그대로
     믿지 않고, ★DR 로 굴러가면서 GPS 잔차의 중앙값만 따라간다★(④절).

★설계의 한 줄 요약 — 상태에 따라 GPS 를 믿는 방식이 다르다★
    NORMAL   (RTK Fixed) : fix 가 오면 ★그 값으로 스냅★ 한다. 상시 보정 없음.
    DEGRADED (비RTK 지속) : fix 로 ★DR 을 천천히 끌어당긴다★(잔차 중앙값 × 게인).
어느 쪽이든 fix 사이는 IMU 가 메운다. 그래서 나가는 좌표는 언제나
`앵커 + 그 뒤 흐른 시간만큼의 추측항법` 이고, 두 성분이 [is_raw]·[dr_dist_m] 로
분리되어 나간다. ★상시 칼만필터가 아니다★ — 구 white 의 gps_imu.py 처럼 항상
융합하지 않는다. Fixed 일 때는 GPS 가 이기고, 그 판단이 코드 한 줄로 드러난다.

────────────────────────────────────────────────────────────────────────────────
 ★① status.status 로는 RTK Fixed 와 Float 을 구별할 수 없다 (실제 드라이버 확인)★
────────────────────────────────────────────────────────────────────────────────
/fix 를 만드는 것은 외부 패키지 nmea_navsat_driver(nmea_serial_driver)다. 그
소스(libnmea_navsat_driver/driver.py)의 GGA quality → NavSatStatus 매핑은 이렇다:

      GGA q=1 (SPS)       → STATUS_FIX      (0)   기본 EPE 4.0 m
      GGA q=2 (DGPS)      → STATUS_SBAS_FIX (1)   기본 EPE 0.1 m
      GGA q=4 (RTK Fixed) → STATUS_GBAS_FIX (2)   기본 EPE 0.02 m
      GGA q=5 (RTK Float) → STATUS_GBAS_FIX (2)   기본 EPE 4.0 m   ★같은 2★
      GGA q=9 (WAAS)      → STATUS_GBAS_FIX (2)   기본 EPE 3.0 m   ★같은 2★

즉 `status.status >= 2` 는 "Fixed 이상"이 아니라 ★"Float 이상"★ 이다. 종전
driving.py 의 `RTK_FIXED_STATUS = 2` 판정은 이름과 달리 오차 수 m 짜리 Float 을
2cm 짜리 Fixed 와 똑같이 통과시키고 있었고, 로그에도 그 차이가 남지 않았다.

★구별하는 값은 같은 메시지 안에 이미 있다★ 같은 드라이버가
      position_covariance[0] = (HDOP × lon_std_dev)²      ← 동(x)
      position_covariance[4] = (HDOP × lat_std_dev)²      ← 북(y)
를 채운다. std_dev 의 기본값이 위 표의 EPE 이므로 σ = √cov 는:
      Fixed(q4) : HDOP 1.0 → 0.02 m,  HDOP 5.0(악조건) → 0.10 m
      Float(q5) : HDOP 0.5(호조건) → 2.00 m,  HDOP 1.0 → 4.00 m
      WAAS(q9)  : HDOP 1.0 → 3.00 m
0.10 과 2.00 사이가 비어 있으므로 그 사이에 문턱을 두면 양쪽 다 3배 이상 여유가
있다 → RTK_FIXED_SIGMA_M = 0.30.

★σ 를 1차 기준으로 삼는 이유(단순히 q4 를 알아내려는 게 아니다)★ 수신기가
$GST 문장을 내면 드라이버가 위 고정 표 대신 ★수신기가 실시간 계산한 오차추정★ 을
그대로 covariance 에 넣는다(driver.py 의 using_receiver_epe). 그러면 같은 q5 라도
수렴 중인 Float 은 σ 가 작게 나온다. 우리가 알고 싶은 것은 'GGA 숫자가 몇인가'가
아니라 ★지금 이 좌표를 몇 cm 로 믿을 수 있나★ 이므로, σ 를 기준으로 두는 편이
질문에 더 정확히 답한다. status.status 는 그 위의 ★상한 게이트★ 로만 쓴다
(status 가 0/1 이면 σ 가 아무리 작아도 RTK 로 올리지 않는다).

⚠️ covariance 가 비어 있는 경우(COVARIANCE_TYPE_UNKNOWN — RMC-only 모드 등)에는
   σ 를 알 수 없으므로 ★Fixed 로 올리지 않고 FLOAT 로 둔다★. σ=0 을 '아주 정확'
   으로 읽으면 최악의 입력을 최선으로 오해하게 된다 — 모르면 낮게 본다.

────────────────────────────────────────────────────────────────────────────────
 ★② 이상치 게이트 — "말이 안 되는 값은 버린다"★
────────────────────────────────────────────────────────────────────────────────
RTK 가 흔들리면 오차가 백색잡음처럼 커지는 게 아니라 ★한두 표본이 수 m 밖으로
튀는★ 형태로 나타난다(멀티패스·위성 편입/이탈). 그건 평균으로 못 지우고 버려야 한다.
통계 용어로 innovation gating 이고, 칼만필터에도 기본으로 들어가는 기능이다.

  ★게이트 1 — 물리적으로 불가능한가 (제일 확실하고 제일 싸다)★
    이 차는 0.2초에 5m 를 갈 수 없다. 직전 위치에서 ★갈 수 있었던 최대 거리★ 를
    계산해 그것을 넘으면 버린다:
        허용 = v_직전·Δt + ½·a_max·Δt² + max(3σ, 바닥)
    앞의 두 항은 운동학이고 뒤 항은 ★측정 잡음 몫★ 이다. σ 를 넣는 이유가 중요하다 —
    Fixed(σ 0.02)에서는 게이트가 타이트하고 Float(σ 2~4)에서는 느슨해진다. 즉
    ★수신기가 스스로 밝힌 불확실성만큼만 관대해진다★. 다만 Float 의 σ 를 그대로
    쓰면 게이트가 12m 까지 열려 무력해지므로 GATE_SIGMA_CAP_M 으로 자른다.

  ★게이트 2 — 방향이 말이 되는가 (자동차는 옆으로 못 간다)★
    변위 방위와 차량 진행축(코스 + 자이로 적분)의 차가 거의 90° 면 그건 차의 운동이
    아니다. ★전진·후진 양쪽을 다 허용하고 '횡방향'만 버린다★ — 후진을 금지하면
    수동 조작에서 정상 주행을 버리게 된다. 즉 이 게이트가 잡는 것은 물리적으로
    불가능한 ★옆걸음★ 뿐이다. 그 대신 오검출이 사실상 없다.

  ★게이트 3 — 수신기가 스스로 못 믿겠다고 하는가★
    quality == Q_NONE 이면 좌표 자체를 내보내지 않는다(아래 '발행하지 않는다' 절).

  ★★ 게이트에는 반드시 탈출구가 있어야 한다 ★★
  RTK 가 재수렴하면 ★진짜 위치가 실제로 몇 m 점프한다★(직전 해가 틀렸던 것이다).
  게이트만 있으면 그 옳은 값을 영원히 거부하고 낡은 DR 로 달린다 — 게이트 없는
  것보다 더 위험하다. 그래서 연속 GATE_MAX_REJECT_N 회 기각하면 ★그 다음 것은
  받아들이고★ 상태를 그 위치로 재설정한다(로그로 크게 남긴다).

────────────────────────────────────────────────────────────────────────────────
 ★③ 공백 메움(DR) — 무엇을 어디까지 메우는가★
────────────────────────────────────────────────────────────────────────────────
fix 는 5Hz(0.2초)로 오고 제어는 20Hz(0.05초)다. 그 사이 3틱은 종전에 ★같은 좌표를
그대로 다시 쓰고 있었다★ — 4.42 m/s 라면 마지막 틱에서 위치가 최대 0.88m 낡는다.

      ψ(t) = (마지막 유효 GPS 코스) + ∫자이로z dt        ← 방향은 IMU 가 만든다
      v    = 엔코더 속도(우선) 또는 GPS 변위 속도        ← 크기
      가상좌표 = 앵커 + ∫ v·[cosψ, sinψ] dt

★절대 헤딩을 driving 에서 받아오지 않는다★ 그러면 driving(헤딩 추정) →
gps(위치) → driving(헤딩 추정) 순환이 생긴다. 대신 ★GPS 코스★(연속한 두 fix 의
변위 방위)를 절대 기준으로 삼고, 그 뒤의 회전만 자이로로 얹는다.

★속도는 엔코더를 먼저 쓴다 [2026-08-18 (3)]★ 종전에는 GPS 변위 속도만 썼는데,
품질이 나쁠 때는 그 값이 ★잡음 때문에 부풀어 오른다★ — σ 2m 짜리 fix 두 개의
변위를 0.4초로 나누면 속도 오차가 ±5 m/s 급이다. 그걸로 외삽하면 DR 이 날아간다.
엔코더는 바퀴가 실제로 돈 만큼이라 품질과 무관하고 지연도 없다.
  ⚠️ 엔코더의 약점은 ★정지 중 허수 카운트★ 다(A보드 기동 블랭킹, 실측 최대 77카운트).
     그래서 ⓐ driving 과 같은 3점 중앙값 필터를 쓰고 ⓑ 물리 상한으로 자르고
     ⓒ 어차피 DR 누적 거리 상한(DR_MAX_DIST_M)이 마지막 방벽으로 남는다.

★한도 3개 — 넘으면 메우지 않는다★
  · DR_MAX_S      : 마지막 fix 로부터 이 시간까지만 메운다. 넘으면 ★발행을 멈춘다★
  · DR_MAX_DIST_M : 누적 외삽 거리 상한(속도 오독으로 가상좌표가 날아가는 것 차단)
  · IMU 신선도    : IMU 가 낡았으면 메우지 않는다. '방향은 IMU 가 만든다'는 전제가
                    깨진 상태에서 코스 방향으로 직진 외삽하면 코너에서 크게 틀린다.

★DR_MAX_S 를 넘으면 왜 '발행 중지'인가 — driving 의 두절 감지를 살려 두기 위해서다★
driving 은 `now − fix_time > GPS_TIMEOUT_S(2.0s)` 로 GPS 두절을 잡아 차를 세운다.
이 노드가 DR 로 무한히 발행하면 그 타이머가 영원히 리셋되어 ★두절을 못 잡는다★.
그래서 한도를 넘으면 조용히 멈춘다. 더불어 [raw_age_s] 를 함께 실어 보내므로
driving 은 '이 배열이 도착한 시각'이 아니라 ★원시 fix 의 나이★ 로 두절을 판정한다.

★품질이 Q_NONE 이면 발행하지 않는다 (종전 동작을 의도적으로 바꿨다)★
종전 driving.cb_fix 는 status 를 위치 수용에 쓰지 않아서, NO FIX 해(解)가 와도
x/y 를 갱신하고 fix_time 까지 리셋했다 → 두절도 아니고 좌표는 쓰레기인 채로 추종이
계속됐다. 여기서는 아예 내보내지 않는다 → driving 은 2초 뒤 두절로 판단해 선다.

────────────────────────────────────────────────────────────────────────────────
 ★④ DEGRADED 모드 — 비RTK 가 지속될 때 (2026-08-18 (3) 신설)★
────────────────────────────────────────────────────────────────────────────────
status 가 계속 0(SPS)·1(DGPS)·또는 Float 이면, fix 하나하나는 수십 cm~수 m 로 튄다.
그때 ★스냅은 최악의 선택★ 이다 — 튐이 그대로 조향에 실린다. 반대로 ★순수 DR 만★
쓰는 것도 안 된다 — 자이로 드리프트와 바퀴 지름 오차가 몇 초면 쌓인다.

그래서 둘을 나눠 맡긴다:
    ★단기 운동(어디로 얼마나 갔나)★  → DR (IMU 방향 + 엔코더 거리). 매끄럽다.
    ★장기 위치(절대 어디인가)★      → GPS. 단, 개별 fix 가 아니라 ★평균★ 으로.

구체적으로, 매 fix 마다 ★잔차★ 를 본다 :
        잔차 = fix 좌표 − (앵커 + 그때까지의 DR)
그리고 최근 DEGRADED_RESID_WIN 개 잔차의 ★중앙값★ 을 DEGRADED_RESID_GAIN 만큼만
따라간다. 상한은 DEGRADED_RESID_MAX_M.

  ★왜 좌표를 직접 평균하지 않고 '잔차'를 평균하는가 — 차가 움직이기 때문이다★
  3.5 m/s 로 달리는 중에 1초치 좌표를 평균하면 3.5m 뒤처진 점이 나온다. 뜻이 없다.
  DR 이 그 운동을 이미 설명하므로, 남는 잔차는 ★운동이 빠진 순수 위치오차★ 다.
  그것만 평균하면 '차가 움직이는 것'과 'GPS 가 튀는 것'이 깔끔히 분리된다.
  사용자가 말한 "IMU 가 바라보는 방향이 있으니 GPS 위치를 찍기 편하다"가 이 구조다.

  ★평균이 아니라 중앙값이다★ 평균은 한 표본이 5m 튀면 그 1/N 만큼 끌려간다.
  중앙값은 소수의 극단값에 전혀 반응하지 않는다 — 지금 막으려는 것이 정확히
  '한두 표본의 큰 튐'이므로 중앙값이 맞는 도구다.

  ★게인의 뜻★ 0.30 · 5Hz 면 시상수 ≈ 0.67초다. 느리게 따라가는 게 아니라,
  ★한 표본이 전부를 끌고 가지 못하게★ 하는 것이 목적이다(중앙값 창 2초 + 게인).

진입·복귀는 ★양방향 모두 지속 조건★ 을 요구한다(채터 방지):
    NORMAL → DEGRADED : 품질 미달이 DEGRADED_ENTER_S 지속
    DEGRADED → NORMAL : Fixed 가 DEGRADED_EXIT_S 지속 → ★그 fix 로 스냅★

⚠️ ★경로에 붙이는 보정(map matching)은 여기서 하지 않는다★ 이 노드는 경로를 모르고,
  알아야 할 이유도 없다. 그리고 그건 driving 의 경로이탈 감지(CTE 2m)를 무력화한다 —
  진짜로 벗어났는데 계산상 경로에 붙어 있게 되기 때문이다. 하려면 driving 쪽에서,
  이탈 감지에는 보정 전 원값을 쓰도록 분리해서 해야 한다.

⚠️ ★카메라 융합은 아직 없다★ GPS 가 완전히 끊긴 구간은 지금도 '발행 중지 → driving
  이 정지'로 끝난다. 카메라(차선 기반 횡보정)를 붙이면 그 구간을 이어 갈 수 있고,
  그때 손댈 곳은 ④절의 잔차 소스뿐이다 — DR 에 GPS 잔차를 먹이는 자리에 카메라
  잔차를 같은 방식으로 얹으면 된다(구 white 의 CAM_LAT_* 가 그 구조였다).
  ★그래서 이번 구조를 '잔차를 먹인다'로 만들어 두었다★ — 소스를 바꿔 끼울 수 있다.

────────────────────────────────────────────────────────────────────────────────
 매핑은 이 노드를 거치지 않는다
────────────────────────────────────────────────────────────────────────────────
mapping.py 는 ★그대로 /fix 원값★ 을 받는다(사용자 결정). 경로는 '차가 실제로 지나간
자리'를 남기는 것이고, DR 로 메운 가상좌표를 지도에 굽는 것은 ★추정을 사실로 굳히는
일★ 이라 나중에 되짚을 수 없게 된다.

────────────────────────────────────────────────────────────────────────────────
 /gps_fused 배열 규약  ★이 목록이 계약이다 — 순서를 바꾸면 record 열이 밀린다★
────────────────────────────────────────────────────────────────────────────────
  [0] lat_deg      가상좌표 위도  (앵커 + DR, DEGRADED 면 잔차보정 포함)
  [1] lon_deg      가상좌표 경도
  [2] quality      0=NONE 1=SPS 2=DGPS 3=RTK_FLOAT 4=RTK_FIXED
  [3] sigma_m      수평 1σ [m]. DR 중에는 경과시간만큼 팽창시킨다(★추정값★)
  [4] pos_ok       1=min_quality 파라미터를 만족 (driving 의 fix_ok 가 이걸 쓴다)
  [5] is_raw       1=이 표본이 갓 온 원시 fix / 0=DR 로 메운 표본
  [6] raw_age_s    마지막 ★수용된★ fix 로부터 경과 [s]  ★두절 판정은 이 값으로★
  [7] dr_dist_m    이 표본에서 DR 이 외삽한 거리 [m]  (0=메우지 않았다)
  [8] gps_kmh      원시 fix 변위 속도 [km/h]. NaN=아직 없음
  [9] course_deg   마지막 유효 GPS 코스 [deg]. NaN=아직 없음
  [10] mode        0=NORMAL(스냅) 1=DEGRADED(잔차 중앙값 융합)     ← [2026-08-18 (3)]
  [11] reject_n    누적 게이트 기각 수. ★늘어나는 구간이 튀는 구간이다★
  [12] resid_m     DEGRADED 잔차 중앙값의 크기 [m] = GPS 와 DR 의 불일치

값이 없을 때는 ★전부 NaN★ 으로 통일한다(-1 같은 파수꾼 값을 쓰지 않는다 — 각
소비처가 그 규약을 따로 기억해야 하고, 한 곳이 잊으면 -1 이 실측값처럼 흘러간다).

────────────────────────────────────────────────────────────────────────────────
 ★남은 일 — DEGRADED 융합을 살리려면 '방향의 출처'를 바꿔야 한다★
────────────────────────────────────────────────────────────────────────────────
지금 DR 의 절대 방향은 ★GPS 변위(코스)★ 다. 비RTK 구간에서는 잡음이 한 샘플 이동보다
커서 그 변위에 방향 정보가 없고, 그래서 융합이 케이스마다 흔들린다(DEGRADED_ENABLE
주석의 실측). 문턱 조정으로는 해결되지 않는다 — ★없는 정보를 짜낼 수는 없다★.
방향의 출처를 바꾸는 세 가지 길이 있고, 어느 하나가 생기면 융합을 그대로 켜면 된다:

  ① ★driving 의 확정 헤딩을 받는다★ (작업량 작음, 순환 주의)
     driving 은 출발 직진 구간에서 직진성 잔차까지 검사해 헤딩을 확정하고
     (HeadingEstimator) 이후 자이로 적분 + 코스 융합으로 유지한다 — 이미 우리보다
     좋은 값을 들고 있다. 그것을 토픽으로 받으면 된다.
     ⚠️ 단 driving 의 헤딩은 ★이 노드가 준 위치★ 로 만들어진다 → 순환이다. 끊는 법:
        헤딩은 DEGRADED 진입 ★직전(Fixed 구간)★ 값만 쓰고 그 뒤로는 자이로만 얹는다.
        즉 '지금 헤딩'이 아니라 '마지막으로 믿을 수 있었던 헤딩'을 받는 것이다.

  ② ★자이로 바이어스 보정★ (작업량 중간)
     ①이든 지금 구조든, 몇 초 이상 자이로만 쓰면 바이어스가 각도로 쌓인다. 정지 중
     (엔코더 0 · GPS 변위 0)에 gyro_z 를 저역통과로 모아 두면 그 바이어스를 뺄 수 있다
     — 구 white/gps_imu.py 가 하던 것이고 여기엔 아직 없다.

  ③ ★듀얼 안테나 (moving baseline / GNSS heading)★ (하드웨어, 근본 해결)
     안테나 두 개면 수신기가 방향을 직접 준다. 헤딩 초기화·자이로 드리프트·이 절의
     문제가 통째로 사라진다. 자율주행 농기계가 거의 다 이 방식이다.

★게이트는 이 문제와 무관하게 이미 값을 낸다★ 물리 게이트는 방향을 쓰지 않고, 방향
게이트는 σ 문턱으로 Fixed 전용이 되어 있다. 그래서 게이트만 ON 인 현재 설정에서도
단발 튐은 잡힌다(폐루프 시뮬: 8m 튐 4회 섞인 구간의 최대오차 8.95m → 1.80m).
"""

import math
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64MultiArray, Int32, String


# ══════════════════════════════════════════════════════════════════════════════
#  ★공개 계약★ — driving.py 가 이 상수들을 import 한다 (단일 소유자)
#    리터럴을 양쪽에 각자 적으면 반드시 어긋난다. paths.py 와 같은 이유·같은 방식.
# ══════════════════════════════════════════════════════════════════════════════
GPS_FUSED_TOPIC   = '/gps_fused'
GPS_QUALITY_TOPIC = '/gps_quality'
GPS_FUSED_FIELDS  = 13          # record.py 의 _array(n) 와 맞춘다

Q_NONE  = 0     # fix 없음 / 좌표 무효 — ★이 값은 발행되지 않는다★(위 헤더 참고)
Q_SPS   = 1     # GGA q1. 보정 없는 단독측위. 오차 수 m
Q_DGPS  = 2     # GGA q2. 오차 ~1m
Q_FLOAT = 3     # GGA q4/q5/q9 인데 σ 가 문턱보다 크다 = Float/WAAS. 오차 수십cm~수m
Q_FIXED = 4     # GGA q4 급. σ 가 문턱 이하 = 정수해 확정. 오차 ~2cm

Q_LABEL = {Q_NONE:  'NO_FIX',
           Q_SPS:   'SPS',
           Q_DGPS:  'DGPS',
           Q_FLOAT: 'RTK_FLOAT',
           Q_FIXED: 'RTK_FIXED'}
Q_EMOJI = {Q_NONE: '❌', Q_SPS: '🟡', Q_DGPS: '🟠', Q_FLOAT: '🔵', Q_FIXED: '🟢'}

M_NORMAL, M_DEGRADED = 0, 1     # [10] mode
M_LABEL = {M_NORMAL: 'NORMAL', M_DEGRADED: 'DEGRADED'}


# ══════════════════════════════════════════════════════════════════════════════
#  튜닝 상수
# ══════════════════════════════════════════════════════════════════════════════
PUB_HZ = 20.0        # 발행 주기 [Hz] — driving 의 CONTROL_HZ 와 맞춘다
#   ★원시 fix 는 이 타이머를 기다리지 않는다★ cb_fix 에서 즉시 내보낸다.
#   타이머는 그 사이의 빈 틱만 DR 로 메운다 → 원시 fix 에 지연이 붙지 않는다.

# ── 품질 판정 ──
RTK_FIXED_SIGMA_M = 0.30
#   σ 가 이 값 이하이고 status 가 GBAS(2) 면 Fixed 로 본다. 근거는 파일 헤더의
#   ①절 계산(Fixed 최악 0.10m / Float 최선 2.00m 사이 → 양쪽 3배 이상 여유).
#   ⚠️ 내리면 HDOP 나쁜 진짜 Fixed 를 Float 로 떨어뜨리고, 올리면 수렴 중인 Float 을
#     Fixed 로 오인한다. 실차에서 이 값을 만지기 전에 record 의 gps_sigma_m 분포를
#     먼저 볼 것 — 두 무리로 갈려 보일 것이고 그 골짜기가 이 문턱이다.

# ── ★이상치 게이트★ [2026-08-18 (3) 신설] 근거는 파일 헤더 ②절 ────────────────────
GATE_ENABLE       = True
GATE_SPEED_MAX_MS = 8.0    # [m/s] 이 차가 낼 수 없는 속도(5펄스 4.42 의 1.8배)
#   ★자율주행 상한(4펄스 3.54)이 아니라 차량 쪽에 여유를 두고 잡는다★ 이 노드는
#   수동 계측 런치에서도 돌고, 사람이 몰면 더 빠를 수 있다. 게이트가 정상 주행을
#   버리는 것이 튐을 놓치는 것보다 나쁘다 — 넉넉하게 두고 아래 두 항이 실제로 조인다.
GATE_A_MAX_MS2    = 6.0    # [m/s²] 이 차가 낼 수 없는 가·감속 (2단 실측 3.8 의 1.6배)
GATE_SIGMA_K      = 3.5
#  ★3σ 가 아니라 3.5σ 인 이유 — '3σ=99.7%' 는 정규분포 이야기다★
#  게이트가 보는 것은 ★2차원 벡터 차의 크기★ 다. 각 축 잡음이 ±a 균일(σ=a/√3)이면
#  두 표본 차는 축마다 ±2a 이고 2D 크기의 상한은 2a√2 = ★4.9σ★ 다. 3σ 로 자르면
#  정상 표본의 꼬리를 버린다 — 실측(폐루프 시뮬)에서 σ=0.5 구간의 기각률이 3.4%
#  (41/1200)였고, 기각된 틱은 위치가 낡아 ★최대오차가 오히려 77% 악화★ 됐다.
#  3.5 × √2 = 4.95σ 로 두면 그 꼬리를 통과시키면서 8m 급 튐은 그대로 잡는다.
GATE_SIGMA_SQRT2  = 1.41421356
#   ★잡음을 √2 배로 세는 이유 — 게이트가 보는 것은 '두 fix 사이의 거리'다★
#   그 거리에는 ★직전 fix 의 잡음과 지금 fix 의 잡음이 모두★ 들어간다. 독립이므로
#   합의 표준편차는 σ√2 다. 이걸 빼먹으면 게이트가 실제 산포보다 좁아져 ★정상 fix 를
#   버린다★ — 폐루프 시뮬에서 σ=1 짜리 DGPS 구간의 fix 를 절반(28/58) 기각했고,
#   그 결과 DEGRADED 융합이 데이터를 굶어 오차가 오히려 81% 악화됐다(수정 전 실측).
GATE_SIGMA_CAP_M  = 5.00   # [m] 그 잡음 몫의 상한 — ★없으면 Float 에서 게이트가 죽는다★
#   σ 가 4m 까지 나오는 Float 에서 3σ√2 를 그대로 쓰면 17m 까지 열려 무력해진다.
#   5.0 은 'DGPS(σ≈1) 산포는 통과시키고 그보다 큰 것은 잡는다'로 정한 값이다:
#       σ=0.02(Fixed) → 0.09m  ★타이트★   σ=1.0(DGPS) → 4.24m  ★산포 통과★
#       σ=4.0(Float) → 5.00m(캡)          50m 야생해 → 어느 σ 에서도 기각
GATE_FLOOR_M      = 0.12   # [m] 정지 중 지터 허용 바닥 (RTK ±2cm 의 몇 배 여유)
GATE_MAX_REJECT_N = 5
#   ★연속 이만큼 기각하면 그 다음은 받아들이고 상태를 재설정한다★ RTK 재수렴 때
#   진짜 위치가 몇 m 점프하는 것을 영원히 거부하지 않기 위한 ★탈출구★ 다.
#   5회 = 5Hz 에서 1초. driving 의 GPS_TIMEOUT_S(2.0s) 보다 작아서, 게이트가 붙들고
#   있는 동안 차가 먼저 서 버리는 일이 없다.
GATE_DIR_MIN_STEP_M = 0.25  # [m] 이만큼 움직였을 때만 방향을 본다(정지 중엔 방위가 무의미)
GATE_DIR_MAX_DEG    = 60.0
#   변위 방위와 차량 진행축의 차. ★전진·후진 양쪽을 다 허용하고 '횡방향'만 버린다★
#   즉 |Δθ| 와 |180−Δθ| 중 작은 쪽이 이 값을 넘을 때만 기각한다. 60° 는 관대하지만,
#   잡으려는 것이 '옆걸음'(≈90°)이라 그것으로 충분하고 오검출이 사실상 없다.
DIR_SIGMA_K = 6.0
#  ★★ 변위가 잡음보다 충분히 커야 방위를 믿을 수 있다 ★★
#  ★이걸 빼먹으면 방향 게이트가 정상 fix 를 대량 기각한다 (폐루프 시뮬 실측)★
#  DGPS(σ 0.5~1.0)에서 한 샘플의 이동은 0.4m 인데 잡음차는 ±2m 급이다. 즉 측정된
#  변위 방향은 ★거의 전부 잡음이고 방위가 사실상 난수★ 다. 그런데 '진행축과 60° 이상'
#  으로 판정하면 난수 방위의 3분의 1 이상이 걸린다 — 실측 기각률 44%(21/48) 였다.
#  그래서 방위를 쓰는 두 곳 모두 문턱을 σ 에 비례시킨다:
#      필요 변위 = max(고정 문턱, DIR_SIGMA_K · σ)
#  ★계수를 3.0 → 6.0 으로 올린 근거(실측)★ 3.0 으로는 σ=0.5 구간에서 여전히 2.7%
#  (5/186) 를 기각했고 ★전부 방향 게이트였다★(물리 게이트는 0회). 기각된 표본들의
#  jump 가 1.5~2.6m 인데 실제 이동은 0.4m 였다 — 잡음이 4~6배라 방위가 난수인 것이
#  숫자로 드러난다. 6.0 이면 그 구간이 전부 '판정 보류'가 된다.
#  ★그 결과 방향 게이트는 사실상 Fixed 전용이 된다 — 그게 맞다★
#      σ=0.02(Fixed) → 0.30m : 0.2초에 0.4m 가니 ★판정된다★(잡음차 0.08m → 방위오차 11°)
#      σ=0.50        → 3.0m  : 한 샘플로는 판정하지 않는다
#      σ=1.00        → 6.0m  : 그만큼 큰 한 샘플 점프는 어차피 물리 게이트가 잡는다
#  ★'잡음이 크면 방위 판정을 포기한다'가 맞는 답이다★ — 억지로 판정하면 정상을 버리고,
#  그 대가로 위치가 낡아 ★최대오차가 오히려 커진다★(실측 -77%).
COURSE_SIGMA_K = 6.0
#  ★★ 코스 문턱은 게이트 문턱과 ★반드시 분리★ 해야 한다 ★★
#  둘 다 '방위를 믿을 수 있나'를 묻지만 ★틀렸을 때의 대가가 정반대★ 다:
#     · 게이트 : 판정을 포기하면 그 fix 를 받아들인다 = ★안전한 실패★ → 문턱을 크게(6.0)
#     · 코스   : 갱신을 포기하면 방향이 낡는다. COURSE_STALE_S(3초)를 넘으면 ★DR 이
#                아예 멈추고★, DR 이 멈추면 DEGRADED 잔차에서 운동이 제거되지 않아
#                ★융합이 통째로 무너진다★ = ★위험한 실패★ → 문턱을 작게(2.0)
#  ★두 값을 하나로 묶었다가 실측으로 잡았다★ 6.0 을 코스에도 쓰자 σ=1.0~1.5 에서
#  코스 갱신에 6~9m 이동이 필요해졌고, 그 사이 코스가 낡아 DR 이 죽었다. 그 결과
#  DEGRADED 융합이 케이스에 따라 ★-140% ~ -206% 로 악화★ 됐다(그 전 실행에서는
#  같은 케이스가 +22~35% 였다). 문턱 하나를 공유한 것이 원인이었다.
#  ★★ 그런데 이 값에는 '좋은 값'이 없다 — 그것이 DEGRADED 융합을 끈 이유다 ★★
#  2.0 으로 내려 코스를 살리자 이번엔 ★코스가 잡음으로 갱신돼 DR 방향이 배회★ 했고,
#  게이트까지 함께 나빠졌다(백색 σ=0.5·1.0 에서 최대오차 -20%/-12%). 6.0 으로 올리면
#  게이트는 좋아지지만 코스가 낡아 융합이 무너진다. ★양쪽을 동시에 만족하는 값이 없다★
#      COURSE_SIGMA_K   게이트 최대오차     융합 최대오차
#            2.0          -20% / -12%      -116% / -105%
#            6.0          +12% /  +3%      (코스 낡음 → 케이스별 -140~-206%)
#  근본 원인은 ★비RTK 구간에서 GPS 변위로 절대 방향을 얻으려는 것 자체★ 다. 잡음이
#  이동보다 크면 그 안에 방향 정보가 없다 — 문턱을 어디에 두든 마찬가지다.
#  → 지금은 ★게이트를 살리는 쪽(6.0)★ 으로 두고 융합을 끈다. 융합을 되살리려면
#    방향의 출처를 바꿔야 한다(맨 아래 '남은 일' 참고). 그때 만질 곳이 여기다.

# ── ★DEGRADED 모드★ [2026-08-18 (3) 신설] 근거는 파일 헤더 ④절 ────────────────────
DEGRADED_ENABLE      = False
#  ★★ 기본 OFF — ★폐루프 시뮬에서 케이스마다 결과가 갈렸다★ ★★
#  잔차 중앙값 융합 자체는 의도대로 동작한다(상관 편향에서 평균 +24~36%, p95 +15~21%).
#  그런데 ★최대오차★ 가 케이스에 따라 +84% ~ −116% 로 흔들린다. 추종 제어에서 중요한
#  것은 최대오차다 — 그것이 CTE 스파이크와 조향 급변을 만든다. 평균을 얻고 최대를
#  잃는 거래는 이 차에 맞지 않는다.
#  ★원인은 이 모듈의 구조적 한계다 (COURSE_SIGMA_K 주석의 표 참고)★
#  DR 의 방향 기준을 ★GPS 변위(코스)★ 에서 얻는데, 비RTK 구간에서는 잡음이 한 샘플
#  이동보다 커서 그 변위에 방향 정보가 없다. 방향이 틀리면 DR 이 엉뚱하게 굴러가고,
#  잔차 융합은 그 위에 얹히므로 같이 틀어진다. 문턱을 어떻게 잡아도 해결되지 않는다.
#  ★그래도 코드를 남겨 둔 이유★ ⓐ 게이트만으로 이미 큰 튐은 잡힌다(단발 8m 기준
#  최대오차 +80%) ⓑ 방향의 출처가 좋아지면 이 융합은 그대로 살아난다 — 파일 맨 아래
#  '남은 일' 의 세 가지 중 하나가 되면 켜면 된다.
#  켜 보려면 : ros2 param set /gps_node degraded_enable true  (로그의 gps_resid_m 확인)
DEGRADED_ENTER_S     = 1.0   # [s] 품질 미달이 이만큼 지속되면 진입
DEGRADED_EXIT_S      = 2.0   # [s] Fixed 가 이만큼 지속되면 복귀(복귀는 더 신중하게)
DEGRADED_RESID_WIN   = 10    # 잔차 중앙값 표본 수 (5Hz 면 2초 창)
DEGRADED_RESID_GAIN  = 0.30  # 중앙값의 이 비율만 따라간다 (5Hz 에서 시상수 ≈0.67s)
DEGRADED_RESID_MAX_M = 3.0   # [m] 한 번에 적용하는 보정 상한(폭주 방지)
#   ⚠️ 상한에 계속 붙어 있으면 DR 이 GPS 를 못 따라가고 있다는 뜻이다 — 로그의
#     resid_m 이 이 값 근처로 굳으면 엔코더 환산(MS_PER_PULSE)이나 자이로를 의심할 것.

# ── 추측항법(DR) ──
DR_ENABLE       = True
DR_MAX_S        = 1.0    # [s] 이 시간까지만 메운다. 넘으면 발행 중지(헤더 ③절 참고)
#   5Hz 정상이면 공백은 0.2초다. 1.0 은 fix 를 4번 연속 놓친 경우까지 버티는 값이고,
#   driving 의 GPS_TIMEOUT_S(2.0) 보다 충분히 작아서 두절 감지를 가리지 않는다.
DR_MAX_DIST_M   = 3.0    # [m] 누적 외삽 거리 상한. 4.42m/s × 1.0s = 4.4m 라 이쪽이 먼저 걸린다
DR_MIN_MS       = 0.14   # [m/s] 이 밑이면 '안 움직인다'로 보고 적분하지 않는다(0.5km/h)
DR_MAX_STEP_S   = 0.5    # [s] 한 번의 적분 스텝이 이보다 크면 버린다(타이머 스톨 후 튐 방지)
DR_IMU_FRESH_S  = 0.3    # [s] IMU 가 이보다 낡았으면 메우지 않는다
DR_SIGMA_GROWTH_M_PER_S = 0.5
#   ★미검증 추정값★ DR 로 메운 시간에 비례해 σ 를 부풀린다. 0.5 m/s 는 '1초 메우면
#   오차가 0.5m 늘어난다'는 뜻이고, 실측 근거가 아니라 보수적으로 고른 값이다.
#   실측 방법 : record 의 gps_is_raw=1 행에서 직전 DR 예측좌표와의 거리를 모으면 된다.

# ── 엔코더 (DR 거리 성분) [2026-08-18 (3) 신설] ────────────────────────────────────
#   ★값은 driving.py 와 같아야 한다★ 두 노드가 같은 /encoder 를 다르게 환산하면
#   로그에서 두 속도가 갈려 어느 쪽이 맞는지 알 수 없게 된다.
ENC_SUM_TO_PULSE = 0.5    # A보드는 좌+우 합을 보낸다 → 바퀴 하나 기준
ENC_MEDIAN_N     = 3      # 기동 블랭킹 허수 카운트 제거(driving 과 동일)
ENC_MS_PER_PULSE = 0.884  # 1펄스 = 0.884 m/s
ENC_FRESH_S      = 0.4    # [s] 이보다 낡으면 안 믿고 GPS 변위 속도로 내려간다

# ── GPS 변위 속도 ★[2026-08-12] driving 에서 이 노드로 옮겨 온 로직★ ──────────────
#   원시 fix 3점(0.4초 창)의 ★양 끝점 직선거리★ ÷ 시간. 구간마다 잘라 더하지 않는
#   이유는 RTK 지터(±2cm)가 매 구간 양수로 쌓여 정지 중에도 속도가 뜨기 때문이다.
#   ★여기로 옮긴 이유★ 이 계산은 '원시 5Hz fix' 위에서만 뜻이 있다. driving 이
#   /gps_fused(20Hz, DR 포함)를 받아서 같은 3점 창을 쓰면 창이 0.1초로 줄고 DR
#   톱니가 섞여 값이 망가진다. 원시 fix 를 보는 쪽이 계산해서 실어 보내는 것이 맞다.
GPS_SPEED_WIN    = 3     # 표본 수(5Hz 3점 = 0.4초 창)
GPS_SPEED_MAX_DT = 1.0   # 창이 이보다 벌어지면 두절로 보고 버린다
GPS_SPEED_STALE_S = 0.6  # 마지막 갱신이 이보다 오래되면 NaN 으로 낸다

# ── GPS 코스(변위 방위) — ★DR 방향의 절대 기준★ ────────────────────────────────
COURSE_MIN_STEP_M = 0.30
#   이만큼 움직였을 때만 코스를 갱신한다. 정지 중 1~2cm 노이즈는 방위를 180° 뒤집는다.
COURSE_FIRST_MIN_STEP_M = 1.00
#  ★★ 첫 코스는 더 긴 기선으로 잡는다 ★★
#  주행은 ★출발 직후 조향 0 으로 곧게 굴러가는 구간★ 에서 시작한다(driving 의
#  DRIVE_HEADING). 그 구간이 방위를 잡기에 가장 좋은 조건이고, 여기서 잡은 코스가
#  이후 DR 방향의 ★절대 기준★ 이 되므로 정확도가 그대로 따라 내려간다.
#  기선 길이에 따른 방위 오차(RTK 2cm, 양 끝점이므로 √2배):
#        0.30m → atan(0.028/0.30) = ★5.4°★      1.00m → ★1.6°★
#  driving 의 HEAD_MIN_DIST_M 도 같은 이유로 1.0m 다 — 두 값을 맞춰 둔다.
#  ⚠️ 이후 갱신은 COURSE_MIN_STEP_M(0.30m)로 짧게 한다 — 코너에서 방위를 따라가야
#     하기 때문이다. 첫 값만 길게 잡는 이유는 그때만 '곧게 간다'가 보장되기 때문이다.
#  ※ ★엄밀한 헤딩 초기화는 driving 이 소유한다★(HeadingEstimator: 직진성 잔차 검사와
#    σ 판정까지 한다). 여기 코스는 DR 의 방향 기준일 뿐이고 GPS 로 계속 교정된다.
COURSE_STALE_S    = 3.0  # 이보다 오래된 코스는 DR·방향게이트에 쓰지 않는다

# ── 로그 ──
QUALITY_LOG_PERIOD_S = 5.0   # 변화가 없어도 이 주기로 한 번은 상태를 남긴다

EARTH_R = 6378137.0


# ══════════════════════════════════════════════════════════════════════════════
#  로컬 평면 변환
#    ★이 노드의 xy 프레임은 밖으로 나가지 않는다★ 출력은 위경도(lat/lon)뿐이므로
#    driving 의 원점(lat0/lon0)과 일치할 필요가 없다 — 두 프레임이 만나지 않아서
#    '원점이 어긋났다'는 종류의 버그가 원리적으로 생기지 않는다.
# ══════════════════════════════════════════════════════════════════════════════
def latlon_to_xy(lat, lon, lat0, lon0):
    x = EARTH_R * math.radians(lon - lon0) * math.cos(math.radians(lat0))
    y = EARTH_R * math.radians(lat - lat0)
    return x, y


def xy_to_latlon(x, y, lat0, lon0):
    lat = lat0 + math.degrees(y / EARTH_R)
    lon = lon0 + math.degrees(x / (EARTH_R * math.cos(math.radians(lat0))))
    return lat, lon


def _median(vals):
    s = sorted(vals)
    n = len(s)
    if n == 0:
        return 0.0
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


# ══════════════════════════════════════════════════════════════════════════════
#  노드
# ══════════════════════════════════════════════════════════════════════════════
class GpsNode(Node):

    def __init__(self):
        super().__init__('gps_node')

        # ── 파라미터 ──
        #  ★★ min_quality 기본값 = Q_DGPS(2) — 2026-08-18 실차 로그로 내렸다 ★★
        #  이 값은 driving 의 fix_ok 가 되고, fix_ok 는 ① 헤딩 초기화 표본 수집
        #  (head_est.add) ② 코스 융합(_fuse_gps_course) 두 곳을 막는 문턱이다.
        #  ★위치 추종에는 쓰이지 않는다★ — 좌표는 품질과 무관하게 그대로 나간다.
        #
        #  ★왜 내렸나 — 실차 3주행이 DGPS 라 헤딩을 못 잡고 전부 실패했다★
        #  ros2bag/route_*-2026081820{1804,2118,2531}.csv : fix_status 1(GGA q2=DGPS)
        #  100%, gps_pos_ok 0%, 그래서 head_est 에 표본이 ★한 개도★ 안 들어가
        #  DRIVE_HEADING 에서 못 나왔다(차는 실제로 12km/h 로 굴렀다).
        #  종전 기본값 3(FLOAT)이 DGPS(2)를 막은 것이다.
        #  ※ 이건 이번에 생긴 회귀가 아니다 — 종전 driving 도 `status >= 2` 로 막았고
        #    DGPS 는 status=1 이라 똑같이 막혔다. 달라진 것은 ★이유가 로그에 보인다★ 는 점뿐이다.
        #
        #  ★DGPS 로 헤딩을 잡아도 되는가 — 그 로그로 실제 추정기를 재생해 확인했다★
        #    (HeadingEstimator 를 그대로 돌려 지도 시작방위와 대조)
        #        DGPS  +173.1° vs 지도 +173.9°  → 차이 −0.8°   σ=1.24° 잔차 2.9cm
        #        DGPS  −15.1° vs 지도 −10.7°   → 차이 −4.4°   σ=1.08° 잔차 1.8cm
        #        DGPS  −14.0° vs 지도 −12.8°   → 차이 −1.3°   σ=1.22° 잔차 2.3cm
        #        FIXED −10.8° vs 지도 −12.8°   → 차이 +2.0°   σ=1.02° 잔차 0.9cm
        #    ★DGPS 의 헤딩 오차가 RTK Fixed 와 같은 급이다★. 게다가 이 '차이'는
        #    헤딩 오차 + 사람이 차를 놓은 각도 오차의 합이므로 실제 오차는 더 작다.
        #    확정 문턱(σ 3.0° / 잔차 15cm)에도 여유가 크다 → 문턱을 손댈 필요가 없었다.
        #
        #  ★그래도 SPS(1)는 계속 막는다★ HeadingEstimator 에는 자체 품질검사가 있지만
        #    `forced = dist >= HEAD_MAX_DIST_M(5.0)` 경로가 있어 ★5m 를 가면 정확도
        #    미달이어도 확정한다★. SPS(기본 EPE 4m)면 그 경로로 엉뚱한 헤딩이 박힌다.
        #    2 로 두면 DGPS 만 통과하고 그 구멍이 막힌다 — 그래서 1 이 아니라 2 다.
        #
        #  ⚠️ 잔차가 작은 것이 헤딩이 정확하다는 ★증거는 아니다★. DGPS 오차는 백색잡음이
        #    아니라 서행 편향이고, 편향이 매끄럽게 흐르면 직선처럼 보여 잔차에 안 나타난다
        #    (1.1m 기선에서 10cm 횡편향 드리프트 = 5.2° 헤딩 오차인데 잔차는 작다).
        #    위 −0.8~−4.4° 는 ★결과가 좋았다는 관측★ 이고 σ 가 보증한 값이 아니다.
        #    σ 계산의 HEAD_SIGMA_FLOOR(0.02m)도 주석에 'RTK Fixed 기준'이라 적혀 있다.
        #    → 출발 헤딩이 몇 도 틀어질 수 있고, 그 뒤는 코스 융합이 당겨서 맞춘다.
        #      그 융합이 DGPS 에서 안전하도록 driving 의 FUSE_SIGMA_K 를 함께 넣었다.
        #     Fixed 만 쓰려면 : ros2 param set /gps_node min_quality 4
        self.declare_parameter('min_quality', Q_DGPS)
        self.declare_parameter('dr_enable', DR_ENABLE)
        self.declare_parameter('dr_max_s', DR_MAX_S)
        self.declare_parameter('pub_hz', PUB_HZ)
        self.declare_parameter('rtk_fixed_sigma_m', RTK_FIXED_SIGMA_M)
        self.declare_parameter('gate_enable', GATE_ENABLE)
        self.declare_parameter('degraded_enable', DEGRADED_ENABLE)
        #   ★degraded_below : 이 품질 미달이 지속되면 DEGRADED 로 간다★
        #   기본 Q_FIXED = 'Fixed 가 아니면 전부'(Float·DGPS·SPS). 사용자가 말한
        #   "status 1 과 0" 만 잡고 싶으면 Q_FLOAT(3) 으로 내린다 — 그러면 Float 은
        #   종전처럼 스냅한다. ⚠️ Float 은 σ 가 수 m 라 스냅하면 그 튐이 조향에 실린다.
        self.declare_parameter('degraded_below', Q_FIXED)

        self.min_quality = int(self.get_parameter('min_quality').value)
        self.dr_enable   = bool(self.get_parameter('dr_enable').value)
        self.dr_max_s    = max(0.0, float(self.get_parameter('dr_max_s').value))
        self.pub_hz      = max(1.0, float(self.get_parameter('pub_hz').value))
        self.fixed_sigma = max(0.0, float(self.get_parameter('rtk_fixed_sigma_m').value))
        self.gate_enable = bool(self.get_parameter('gate_enable').value)
        self.deg_enable  = bool(self.get_parameter('degraded_enable').value)
        self.deg_below   = int(self.get_parameter('degraded_below').value)

        # ── 앵커 (발행 좌표의 기준점) ★[2026-08-18 (3)] fix 좌표에서 앵커로 바뀌었다★
        #   NORMAL   : 마지막으로 수용한 fix 좌표 그대로(스냅)
        #   DEGRADED : DR 예측 + 잔차 중앙값 × 게인 (잔차 융합)
        #   이 구조 덕분에 잔차 소스를 바꿔 끼울 수 있다(헤더 ④절의 카메라 융합 메모).
        self.lat0 = self.lon0 = None    # 내부 평면 원점(첫 유효 fix)
        self._ax = self._ay = 0.0        # 앵커 좌표
        self._fix_t = 0.0                # 마지막 ★수용된★ fix 시각. 0 = 아직 없음
        self._quality = Q_NONE
        self._sigma = float('nan')

        # ── 게이트 상태 ──
        self._last_ok_x = self._last_ok_y = 0.0   # 마지막 수용 fix (물리 게이트 기준)
        self._last_ok_t = 0.0
        self._reject_n = 0               # 누적 기각 수(진단)
        self._reject_run = 0             # ★연속★ 기각 수(탈출구 판정)

        # ── 모드 상태 ──
        self._mode = M_NORMAL
        self._q_good_since = 0.0         # 품질이 충족된 이후 경과 기준시각
        self._q_bad_since = 0.0
        self._resid = []                 # [(dx, dy)] 최근 잔차
        self._resid_m = 0.0              # 잔차 중앙값 크기(진단)

        # ── GPS 변위 속도 ──
        self._trail = []                 # [(t, x, y)] 최근 GPS_SPEED_WIN 표본
        self._kmh = None
        self._kmh_t = 0.0

        # ── GPS 코스 ──
        self._course = None              # [rad] 마지막 유효 변위 방위
        self._course_t = 0.0
        self._course_pt = None           # 코스 계산 기준점 (x, y)

        # ── IMU ──
        self._gyro_z = 0.0               # [rad/s] CCW +
        self._imu_t = 0.0

        # ── 엔코더 ──
        self._enc_buf = []
        self._enc_ms = None              # [m/s] 중앙값 필터 후 속도
        self._enc_t = 0.0

        # ── DR 누적 (앵커가 갱신될 때마다 0 으로 리셋) ──
        self._dr_x = self._dr_y = 0.0    # [m] 앵커로부터의 외삽 변위
        self._dr_t = 0.0                 # 마지막 적분 시각
        # ★[2026-08-18 (3)] '코스 이후' 자이로 누적 — ★fix 마다 리셋하지 않는다★
        #   리셋은 코스가 갱신될 때만(_update_course). 이유는 _advance_dr 주석.
        self._yaw_ref = 0.0              # [rad]

        # ── 발행/로그 상태 ──
        self._last_pub_t = 0.0
        self._last_q_log_t = 0.0
        self._logged_quality = None
        self._is_raw_pending = False

        self.pub_fused = self.create_publisher(Float64MultiArray, GPS_FUSED_TOPIC, 10)
        self.pub_qual  = self.create_publisher(String, GPS_QUALITY_TOPIC, 10)

        self.create_subscription(NavSatFix, '/fix',     self.cb_fix,     10)
        self.create_subscription(Imu,       '/imu',     self.cb_imu,     10)
        self.create_subscription(Int32,     '/encoder', self.cb_encoder, 10)

        self.create_timer(1.0 / self.pub_hz, self.on_timer)

        self.get_logger().info(
            f"white1 gps 준비 — {GPS_FUSED_TOPIC} {self.pub_hz:.0f}Hz | "
            f"min_quality={Q_LABEL.get(self.min_quality, '?')} | "
            f"게이트={'ON' if self.gate_enable else 'OFF'} | "
            f"DEGRADED={'ON' if self.deg_enable else 'OFF'}"
            f"(<{Q_LABEL.get(self.deg_below, '?')}) | "
            f"DR={'ON' if self.dr_enable else 'OFF'}(≤{self.dr_max_s:.1f}s) | "
            f"Fixed 문턱 σ≤{self.fixed_sigma:.2f}m")

    # ══════════════════════════════════════════════════════════════════════════
    #  수신
    # ══════════════════════════════════════════════════════════════════════════
    def cb_fix(self, msg: NavSatFix):
        """원시 fix. ★타이머를 기다리지 않고 즉시 발행한다★ (지연 0 유지)"""
        now = time.time()
        quality, sigma = self._classify(msg)

        if quality == Q_NONE:
            # 위치를 내보내지 않는다 — 그러면 driving 이 두절로 판단해 선다.
            self._quality, self._sigma = quality, sigma
            self._log_quality(now, force=False)
            return

        if self.lat0 is None:
            self.lat0, self.lon0 = msg.latitude, msg.longitude
            self.get_logger().info(
                f"📍 로컬 원점 확정 {self.lat0:.7f}, {self.lon0:.7f} "
                f"({Q_EMOJI.get(quality,'')} {Q_LABEL.get(quality)})")

        x, y = latlon_to_xy(msg.latitude, msg.longitude, self.lat0, self.lon0)

        # ── ★게이트★ 말이 안 되는 fix 는 버린다 (헤더 ②절) ─────────────────────
        ok, why = self._gate(x, y, now, sigma)
        forced = False
        if not ok and self._reject_run >= GATE_MAX_REJECT_N:
            # ★★ 탈출구 ★★ 연속으로 이만큼 버렸다면 '게이트가 틀렸다'고 본다.
            #   RTK 가 재수렴하면 진짜 위치가 실제로 몇 m 점프하는데(직전 해가 틀렸던
            #   것이다), 그것을 영원히 거부하면 낡은 DR 로 달리게 되어 ★게이트가 없는
            #   것보다 더 위험하다★. 그래서 받아들이고 상태를 재설정한다.
            ok, forced = True, True
        if not ok:
            self._reject_n += 1
            self._reject_run += 1
            self._quality, self._sigma = quality, sigma
            self.get_logger().warning(
                f"🚫 fix 기각 {self._reject_run}/{GATE_MAX_REJECT_N} — {why} "
                f"({Q_LABEL.get(quality)} σ={sigma:.2f}m)")
            self._log_quality(now, force=False)
            return
        if self._reject_run > 0:
            # ★연속 기각 뒤의 첫 수용은 '진짜 위치가 점프했다'로 보는 것이 맞다★
            #   그러니 그 점프를 운동으로 오해할 이력을 전부 버린다 — 안 버리면
            #   ⓐ 변위 속도가 점프 크기만큼 폭등하고 ⓑ 코스가 점프 방향으로 뒤집히고
            #   ⓒ DEGRADED 잔차 창이 점프 전 기준으로 계산된 값을 계속 물고 있다.
            self.get_logger().warning(
                f"↩️ fix {'★탈출구★ 강제수용' if forced else '재수용'} — 연속 "
                f"{self._reject_run}회 기각 뒤 복귀({why or '게이트 통과'}). "
                f"코스·속도·잔차 이력을 버리고 이 좌표로 재설정한다")
            self._trail = []
            self._kmh = None
            self._course = None
            self._course_pt = None
            self._resid = []
            self._resid_m = 0.0
            self._reject_run = 0
            # ★강제수용은 스냅이다★ 점프한 좌표에 DEGRADED 잔차 융합을 걸면 게인
            #   0.3 으로 몇 초에 걸쳐 기어가게 된다 — 그동안 좌표가 틀린 채로 달린다.
            #   게이트가 졌다고 인정한 순간에는 GPS 를 그대로 믿는 편이 맞다.
            if forced:
                self._accept_normal(x, y, now)
                self._last_ok_x, self._last_ok_y, self._last_ok_t = x, y, now
                self._fix_t = now
                self._quality, self._sigma = quality, sigma
                self._update_mode(now, quality)
                self._publish(now, is_raw=True)
                self._log_quality(now, force=True)
                return

        self._update_speed(now, x, y)
        self._update_course(x, y, now, sigma)

        # ── 모드 판정 → 앵커 갱신 ──────────────────────────────────────────────
        self._update_mode(now, quality)
        if self._mode == M_DEGRADED:
            self._accept_degraded(x, y, now)
        else:
            self._accept_normal(x, y, now)

        self._last_ok_x, self._last_ok_y, self._last_ok_t = x, y, now
        self._fix_t = now
        self._quality, self._sigma = quality, sigma

        self._publish(now, is_raw=True)
        self._log_quality(now, force=False)

    def cb_imu(self, msg: Imu):
        """자이로 z 만 쓴다. ★적분은 여기서 하지 않는다★ — DR 스텝과 시각을 한
        곳(_advance_dr)에서 맞추지 않으면 '적분은 됐는데 위치는 안 옮겨진' 틱이
        생긴다. 여기서는 최신 각속도와 도착 시각만 남긴다."""
        self._gyro_z = float(msg.angular_velocity.z)
        self._imu_t = time.time()

    def cb_encoder(self, msg: Int32):
        """A보드 좌+우 펄스 합 → 바퀴 하나 기준 속도 [m/s].
        ★3점 중앙값★ 은 기동 블랭킹 허수 카운트를 죽이기 위한 것이고 driving 과 같다
        (그쪽 ENC_MEDIAN_N 주석에 '정상 4~5 인데 34 까지 튄' 실측이 있다).
        ★이 값을 DR 거리 성분으로 쓴다★ 이유는 헤더 ③절 '속도는 엔코더를 먼저 쓴다'.
        """
        self._enc_buf.append(float(msg.data))
        if len(self._enc_buf) > ENC_MEDIAN_N:
            del self._enc_buf[0]
        med = sorted(self._enc_buf)[len(self._enc_buf) // 2]
        v = med * ENC_SUM_TO_PULSE * ENC_MS_PER_PULSE
        # 물리 상한으로 자른다 — 허수 카운트가 DR 을 밀어내지 못하게 하는 1차 방어
        self._enc_ms = max(0.0, min(GATE_SPEED_MAX_MS, v))
        self._enc_t = time.time()

    # ══════════════════════════════════════════════════════════════════════════
    #  판정
    # ══════════════════════════════════════════════════════════════════════════
    def _classify(self, msg: NavSatFix):
        """→ (quality, sigma_m). 판정 근거는 파일 헤더 ①절 전체."""
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            return Q_NONE, float('nan')

        st = int(msg.status.status)
        if st < 0:                       # STATUS_NO_FIX
            return Q_NONE, float('nan')

        # 수평 σ — 두 축 중 ★큰 쪽★ 을 쓴다(보수적: σ 를 크게 봐서 Fixed 판정을 아낀다)
        sigma = float('nan')
        if msg.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            cxx = msg.position_covariance[0]
            cyy = msg.position_covariance[4]
            worst = max(cxx, cyy)
            if math.isfinite(worst) and worst > 0.0:
                sigma = math.sqrt(worst)

        if st == 0:                      # STATUS_FIX      = GGA q1
            return Q_SPS, sigma
        if st == 1:                      # STATUS_SBAS_FIX = GGA q2
            return Q_DGPS, sigma
        # st >= 2 : STATUS_GBAS_FIX = GGA q4 / q5 / q9 → σ 로 갈라낸다.
        #   σ 를 모르면(covariance 없음) Fixed 로 올리지 않는다 — 모르면 낮게 본다.
        if math.isfinite(sigma) and sigma <= self.fixed_sigma:
            return Q_FIXED, sigma
        return Q_FLOAT, sigma

    def _gate(self, x, y, now, sigma):
        """→ (수용할까, 기각 사유). 근거는 파일 헤더 ②절 전체.

        ★기각은 '아무것도 안 하는 것'과 다르다★ 기각하면 앵커도 fix_t 도 갱신되지
        않으므로 raw_age_s 가 계속 흐른다 → 튐이 오래 이어지면 driving 이 두절로
        판단해 스스로 선다. 즉 게이트가 차를 위험하게 만드는 방향으로는 못 간다.
        """
        if not self.gate_enable or self._last_ok_t <= 0.0:
            return True, ''
        dt = now - self._last_ok_t
        if dt <= 0.0:
            return True, ''

        dx, dy = x - self._last_ok_x, y - self._last_ok_y
        jump = math.hypot(dx, dy)

        # ── 게이트 1 : 물리적으로 갈 수 있었던 거리인가 ────────────────────────
        v_prev = self._dr_speed_raw()
        if v_prev is None:
            v_prev = GATE_SPEED_MAX_MS      # 속도를 모르면 관대하게(오검출 방지)
        # 잡음 몫 : ★두 fix 의 잡음이 모두 들어가므로 σ√2★ (상수 GATE_SIGMA_SQRT2 주석)
        noise = GATE_FLOOR_M
        if math.isfinite(sigma):
            noise = max(GATE_FLOOR_M,
                        min(GATE_SIGMA_CAP_M,
                            GATE_SIGMA_K * GATE_SIGMA_SQRT2 * sigma))
        d_max = min(v_prev * dt + 0.5 * GATE_A_MAX_MS2 * dt * dt,
                    GATE_SPEED_MAX_MS * dt) + noise
        if jump > d_max:
            return False, (f"물리 불가 — {dt:.2f}초에 {jump:.2f}m "
                           f"(허용 {d_max:.2f}m = v{v_prev:.1f}·dt + 잡음 {noise:.2f})")

        # ── 게이트 2 : 방향이 말이 되는가 (자동차는 옆으로 못 간다) ─────────────
        #   ★변위가 잡음보다 충분히 클 때만 본다★ 아니면 방위가 난수라 정상을 버린다
        #   (상수 DIR_SIGMA_K 주석의 실측 기각률 44% 참고)
        dir_min = GATE_DIR_MIN_STEP_M
        if math.isfinite(sigma):
            dir_min = max(dir_min, DIR_SIGMA_K * sigma)
        if (jump >= dir_min and self._course is not None
                and now - self._course_t <= COURSE_STALE_S):
            psi = self._course + self._yaw_ref         # 코스 + 자이로 = 진행축
            d_ang = abs(math.degrees(math.atan2(dy, dx) - psi)) % 360.0
            if d_ang > 180.0:
                d_ang = 360.0 - d_ang
            lateral = min(d_ang, abs(180.0 - d_ang))   # 전진·후진 양쪽 허용
            if lateral > GATE_DIR_MAX_DEG:
                return False, (f"횡방향 변위 — 진행축과 {lateral:.0f}° "
                               f"({jump:.2f}m 이동)")

        return True, ''

    def _update_mode(self, now, quality):
        """품질 지속시간으로 NORMAL ↔ DEGRADED 를 전환한다. 근거는 헤더 ④절."""
        good = quality >= self.deg_below
        if good:
            self._q_bad_since = 0.0
            if self._q_good_since == 0.0:
                self._q_good_since = now
        else:
            self._q_good_since = 0.0
            if self._q_bad_since == 0.0:
                self._q_bad_since = now

        if not self.deg_enable:
            self._mode = M_NORMAL
            return

        if self._mode == M_NORMAL:
            if (not good and self._q_bad_since > 0.0
                    and now - self._q_bad_since >= DEGRADED_ENTER_S):
                self._mode = M_DEGRADED
                self._resid = []
                self._resid_m = 0.0
                self.get_logger().warning(
                    f"🔶 DEGRADED 진입 — {Q_LABEL.get(quality)} 가 "
                    f"{now - self._q_bad_since:.1f}초 지속. fix 스냅을 멈추고 "
                    f"DR + 잔차 중앙값 융합으로 간다")
        else:
            if (good and self._q_good_since > 0.0
                    and now - self._q_good_since >= DEGRADED_EXIT_S):
                self._mode = M_NORMAL
                self._resid = []
                self._resid_m = 0.0
                self.get_logger().info(
                    f"🟢 NORMAL 복귀 — {Q_LABEL.get(quality)} 가 "
                    f"{now - self._q_good_since:.1f}초 지속. fix 스냅으로 되돌린다")

    # ══════════════════════════════════════════════════════════════════════════
    #  앵커 갱신
    # ══════════════════════════════════════════════════════════════════════════
    def _accept_normal(self, x, y, now):
        """NORMAL — ★fix 로 그냥 스냅한다★ (블렌딩 없음)

        이 한 줄이 '상시 보정을 하지 않는다'는 설계가 코드로 드러나는 지점이다.
        RTK Fixed 는 2cm 라, 그것보다 나은 추정을 우리가 만들 수 없다.
        """
        self._ax, self._ay = x, y
        self._reset_dr(now)

    def _accept_degraded(self, x, y, now):
        """DEGRADED — ★DR 예측을 GPS 잔차 중앙값으로 조금씩 끌어당긴다★ (헤더 ④절)

        잔차 = fix − (앵커 + DR).  DR 이 차의 운동을 이미 설명하므로 남는 잔차는
        ★운동이 빠진 순수 위치오차★ 다. 그것의 중앙값만 게인만큼 따라간다 —
        중앙값이라 한두 표본이 크게 튀어도 끌려가지 않는다.
        """
        self._advance_dr(now)                     # 이 fix 시점까지 DR 을 밀어 둔다
        px, py = self._ax + self._dr_x, self._ay + self._dr_y

        self._resid.append((x - px, y - py))
        if len(self._resid) > DEGRADED_RESID_WIN:
            del self._resid[0]
        mx = _median([r[0] for r in self._resid])
        my = _median([r[1] for r in self._resid])
        self._resid_m = math.hypot(mx, my)

        # 게인 + 상한. 상한은 '한 번에 이만큼 이상 옮기지 않는다'는 뜻이다.
        cx, cy = mx * DEGRADED_RESID_GAIN, my * DEGRADED_RESID_GAIN
        c = math.hypot(cx, cy)
        if c > DEGRADED_RESID_MAX_M:
            k = DEGRADED_RESID_MAX_M / c
            cx, cy = cx * k, cy * k

        # 새 앵커 = 지금 추정 + 보정.  DR 은 여기서 0 으로 되돌린다.
        self._ax, self._ay = px + cx, py + cy
        self._reset_dr(now)

        # ★★ 창을 새 기준으로 다시 맞춘다 (rebase) ★★
        # ★이게 없으면 필터가 수렴하지 않는다 — 폐루프 시뮬로 잡았다★
        # 창에 남아 있는 잔차들은 ★보정 전 앵커★ 를 기준으로 측정된 값이다. 앵커를
        # cx 만큼 옮겼으면 그 값들도 그만큼 옛것이 된다. 그대로 두면 다음 틱의 중앙값이
        # ★이미 처리한 오차를 다시 보고★ 또 같은 방향으로 보정하게 된다(이중 계산).
        # 실측: rebase 없이는 잔차가 0.9~1.3m 에서 굳어 융합이 스냅보다 27% 나빴다.
        self._resid = [(rx - cx, ry - cy) for rx, ry in self._resid]

    def _reset_dr(self, now):
        """앵커가 바뀌었으니 그로부터의 외삽 변위를 0 으로.
        ★_yaw_ref 는 건드리지 않는다★ 그것은 '앵커 이후'가 아니라 ★'코스 이후'★
        누적이고, 코스가 갱신될 때만 리셋한다(_advance_dr 의 ★★ 주석 참고)."""
        self._dr_x = self._dr_y = 0.0
        self._dr_t = now

    # ══════════════════════════════════════════════════════════════════════════
    #  속도 · 코스
    # ══════════════════════════════════════════════════════════════════════════
    def _update_speed(self, now, x, y):
        """원시 fix 변위 속도 [km/h]. 상수 GPS_SPEED_* 절에 근거가 있다."""
        self._trail.append((now, x, y))
        if len(self._trail) > GPS_SPEED_WIN:
            del self._trail[0]
        if len(self._trail) < 2:
            return
        t0, x0, y0 = self._trail[0]
        t1, x1, y1 = self._trail[-1]
        dt = t1 - t0
        if dt <= 0.0 or dt > GPS_SPEED_MAX_DT:
            # 두절 뒤 첫 표본 — 낡은 점과 이어 붙이면 엉뚱한 속도가 나온다
            self._trail = [self._trail[-1]]
            return
        self._kmh = 3.6 * math.hypot(x1 - x0, y1 - y0) / dt
        self._kmh_t = now

    def _update_course(self, x, y, now, sigma):
        """연속한 원시 fix 의 변위 방위. ★DR 과 방향 게이트의 절대 기준★

        ★문턱이 σ 에 비례한다 (상수 DIR_SIGMA_K)★ 잡음보다 작은 변위로 방위를 내면
        그 방위는 난수이고, 그것이 DR 의 방향이 되므로 ★위치가 배회한다★. 품질이
        나쁠 때는 차가 그만큼 더 움직인 뒤에야 코스를 갱신하는 것이 맞다 — 그 사이는
        자이로가 방향을 이어 간다(그게 DR 의 원래 설계다).
        기준점(_course_pt)을 갱신하지 않고 그대로 두므로, 문턱을 넘을 때까지 변위가
        계속 누적된다 — 즉 ★기다리면 반드시 문턱을 넘는다★(정지 중이 아니라면).
        """
        if self._course_pt is None:
            self._course_pt = (x, y)
            return
        # 첫 코스만 긴 기선을 요구한다(상수 COURSE_FIRST_MIN_STEP_M 주석 참고)
        need = (COURSE_FIRST_MIN_STEP_M if self._course is None
                else COURSE_MIN_STEP_M)
        if math.isfinite(sigma):
            # ★게이트의 DIR_SIGMA_K 가 아니라 COURSE_SIGMA_K 를 쓴다★ 둘을 공유하면
            #   코스가 낡아 DR 이 죽는다 — 그 상수 주석의 실측(-206%) 참고.
            need = max(need, COURSE_SIGMA_K * sigma)
        px, py = self._course_pt
        dx, dy = x - px, y - py
        if math.hypot(dx, dy) < need:
            return                       # 아직 덜 움직였다 — 기준점을 그대로 둔다
        first = self._course is None
        self._course = math.atan2(dy, dx)
        self._course_t = now
        self._course_pt = (x, y)
        # ★코스가 갱신되면 자이로 적분을 0 으로 되돌린다★ _yaw_ref 는 '이 코스 이후
        #   얼마나 돌았나'이므로 기준이 바뀌면 함께 리셋해야 한다(_advance_dr 참고).
        self._yaw_ref = 0.0
        if first:
            self.get_logger().info(
                f"🧭 GPS 코스 확정 {math.degrees(self._course):+.1f}° "
                f"(기선 {math.hypot(dx, dy):.2f}m) — DR 방향의 절대 기준")

    def _dr_speed_raw(self):
        """DR·게이트가 쓸 속도 [m/s]. ★엔코더 우선, 없으면 GPS 변위★ (헤더 ③절)
        둘 다 없으면 None."""
        now = time.time()
        if self._enc_ms is not None and now - self._enc_t <= ENC_FRESH_S:
            return self._enc_ms
        if self._kmh is not None and now - self._kmh_t <= GPS_SPEED_STALE_S:
            return self._kmh / 3.6
        return None

    # ══════════════════════════════════════════════════════════════════════════
    #  발행
    # ══════════════════════════════════════════════════════════════════════════
    def on_timer(self):
        """원시 fix 사이의 빈 틱을 DR 로 메운다."""
        now = time.time()

        if self._fix_t <= 0.0:
            if now - self._last_q_log_t > QUALITY_LOG_PERIOD_S:
                self._last_q_log_t = now
                self.get_logger().warning(
                    "⏳ 유효 fix 대기 — /fix 가 오는지, GGA quality 가 0/무효는 아닌지 확인")
            return

        if now - self._last_pub_t < 1.0 / self.pub_hz * 0.5:
            return                       # 방금 원시 fix 로 냈다 — 중복 발행 방지

        # ★DR 한도를 넘으면 발행을 멈춘다★ driving 의 두절 타임아웃을 살려 두기
        #   위한 것이다(헤더 ③절). 여기서 계속 내보내면 차가 낡은 추정으로 달린다.
        if now - self._fix_t > self.dr_max_s:
            self._log_quality(now, force=False)
            return

        self._publish(now, is_raw=False)
        self._log_quality(now, force=False)

    def _publish(self, now, is_raw: bool):
        if not is_raw:
            self._advance_dr(now)
        x = self._ax + self._dr_x
        y = self._ay + self._dr_y
        dr_dist = math.hypot(self._dr_x, self._dr_y)

        lat, lon = xy_to_latlon(x, y, self.lat0, self.lon0)
        raw_age = max(0.0, now - self._fix_t)

        # σ : DR 로 메운 시간만큼 부풀린다(상수 절의 ★미검증★ 표시 참고)
        sigma = self._sigma
        if math.isfinite(sigma):
            sigma += DR_SIGMA_GROWTH_M_PER_S * raw_age

        kmh = self._kmh if (self._kmh is not None
                            and now - self._kmh_t <= GPS_SPEED_STALE_S) else float('nan')
        course = math.degrees(self._course) if self._course is not None else float('nan')

        msg = Float64MultiArray()
        msg.data = [
            float(lat),                                  # [0]
            float(lon),                                  # [1]
            float(self._quality),                        # [2]
            float(sigma),                                # [3]
            1.0 if self._quality >= self.min_quality else 0.0,   # [4] pos_ok
            1.0 if is_raw else 0.0,                      # [5]
            float(raw_age),                              # [6]
            float(dr_dist),                              # [7]
            float(kmh),                                  # [8]
            float(course),                               # [9]
            float(self._mode),                           # [10]
            float(self._reject_n),                       # [11]
            float(self._resid_m),                        # [12]
        ]
        self.pub_fused.publish(msg)
        self._last_pub_t = now

    def _advance_dr(self, now):
        """DR 누적을 now 까지 한 스텝 전진시킨다.

        메우지 않는(=누적을 그대로 두는) 조건이 여럿이고, 전부 '틀리게 메우는
        것보다 안 메우는 게 낫다'는 같은 이유다. 안 메우면 좌표가 앵커 시점에
        멈춰 있을 뿐이고 그 오차는 v·Δt 로 상한이 뻔하지만, 잘못 메우면 오차의
        크기도 방향도 알 수 없게 된다.
        """
        dt = now - self._dr_t
        self._dr_t = now

        if dt <= 0.0 or dt > DR_MAX_STEP_S:
            return                       # 타이머 스톨 뒤의 큰 스텝은 버린다
        if now - self._imu_t > DR_IMU_FRESH_S:
            return                       # 방향을 만들 IMU 가 없다

        # ★★ 자이로는 '코스 이후 누적'이고, 여기서 먼저 적분한다 ★★
        # ★[2026-08-18 (3)] 이걸 fix 마다 리셋하던 것이 버그였다★ 종전에는 앵커를
        #   갱신할 때(=매 fix) 같이 0 으로 되돌렸다. NORMAL(0.2초 간격)에서는 차이가
        #   없지만, 품질이 나빠 코스가 몇 초간 갱신되지 않는 DEGRADED 에서는
        #   ★그 사이에 돈 각도가 통째로 사라진다★ — 코스는 낡고 자이로는 매번 0 이
        #   되므로 DR 이 '몇 초 전 방향'으로 직진해 버린다. 코너에서 크게 틀린다.
        #   기준은 ★코스가 갱신될 때만★ 리셋한다(_update_course).
        #   ※ 아래 조기 리턴들보다 앞에 두는 것도 같은 이유다 — 속도를 몰라 위치를
        #     못 옮기는 틱에도 ★방향은 계속 따라가야★ 한다.
        self._yaw_ref += self._gyro_z * dt

        if not self.dr_enable:
            return
        if self._course is None or now - self._course_t > COURSE_STALE_S:
            return                       # 절대 방향 기준이 없거나 낡았다
        v = self._dr_speed_raw()
        if v is None or v < DR_MIN_MS:
            return                       # 속도를 모르거나 서 있다

        # 방향 : GPS 코스(절대) + 자이로 적분(상대). 스텝 중간값을 쓴다.
        psi = self._course + self._yaw_ref - self._gyro_z * dt * 0.5

        self._dr_x += v * math.cos(psi) * dt
        self._dr_y += v * math.sin(psi) * dt

        # 누적 거리 상한 — 속도 오독으로 가상좌표가 날아가는 것을 막는 마지막 방벽
        d = math.hypot(self._dr_x, self._dr_y)
        if d > DR_MAX_DIST_M:
            k = DR_MAX_DIST_M / d
            self._dr_x *= k
            self._dr_y *= k

    # ══════════════════════════════════════════════════════════════════════════
    #  상태 로그 / /gps_quality
    # ══════════════════════════════════════════════════════════════════════════
    def _log_quality(self, now, force: bool):
        """★품질이 바뀌면 즉시, 안 바뀌어도 주기적으로 남긴다★
        종전에는 Fixed 인지 Float 인지가 로그에 전혀 남지 않아서, 사후에
        '그 주행이 몇 cm 짜리 좌표로 달린 것인가'를 되짚을 수 없었다.
        """
        changed = (self._quality != self._logged_quality)
        if not (force or changed or now - self._last_q_log_t >= QUALITY_LOG_PERIOD_S):
            return
        self._last_q_log_t = now
        self._logged_quality = self._quality

        sig = f"{self._sigma:.3f}m" if math.isfinite(self._sigma) else "σ미제공"
        age = max(0.0, now - self._fix_t) if self._fix_t > 0.0 else float('inf')
        kmh = self._kmh if self._kmh is not None else float('nan')
        text = (f"{Q_EMOJI.get(self._quality, '?')} {Q_LABEL.get(self._quality, '?')} "
                f"σ={sig} age={age:.2f}s v={kmh:.2f}km/h "
                f"dr={math.hypot(self._dr_x, self._dr_y):.2f}m "
                f"[{M_LABEL.get(self._mode, '?')}] "
                f"기각={self._reject_n} 잔차={self._resid_m:.2f}m "
                f"{'ok' if self._quality >= self.min_quality else '★게이트 미달★'}")
        self.pub_qual.publish(String(data=text))
        if changed:
            self.get_logger().info(f"📶 GPS 품질 변화 → {text}")


def main(args=None):
    rclpy.init(args=args)
    node = GpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
