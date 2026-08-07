# arduino : kasa A/B 2보드 아두이노 시리얼 브리지 (Ubuntu 22.04 / ROS2 Humble 전용)
#
# ★ 이 파일의 위치 ★
#   kasa_ws/src/nxde/nxde/arduino.py 에서 통신 로직만 가져와, 토픽 계약을 white 패키지
#   규약으로 바꾼 것이다. **kasa_ws 쪽은 수정하지 않았다** — 저쪽은 /in·/out String
#   프로토콜을 그대로 쓰고, 이쪽은 white 의 Twist/Bool/Int32 토픽을 직접 주고받는다.
#   아두이노 펌웨어(kasa_0730_A.ino / kasa_0804_B.ino)도 무수정 전제다.
#
# ★ 역할 ★
#   ROS → 보드 :  /cmd_vel_raw (Twist)    linear.x = 주행 목표펄스 0~15
#                                          angular.z = 조향각 -40~40 (★− 좌 / + 우★)
#                 /control_state (Bool)   True = 구동 허용 / False = 정지
#                 /brake_level (Int32)    브레이크 단계 0 / 1 / 2 (선택 — 안 오면 0)
#   보드 → ROS :  /encoder (Int32)              A보드 좌+우 펄스의 ★합★
#                 /steer_angle_measured (Int32) B보드 실측 조향각 (− 좌 / + 우, 그대로 중계)
#                 /vehicle_mode (Bool)          B보드 D5 : True = 자율 / False = 수동조종
#                 /throttle_pedal (Int32)       A보드 A0 쓰로틀 페달 raw 0~1023
#                 /drive_pulse_cmd (Int32)      ★A보드로 실제 보낸 주행 목표펄스★
#                                               자율=계획값 / 수동조종=페달 환산값
#                                               → mapping 노드의 수집 라벨(①)로 쓰인다
#                 /estop (Bool)                 A·B 중 한쪽이라도 STOP 이면 True
#                 /board_status (String)        "A:1,B:1,ESTOP:0,MODE:1" (진단·로스백용)
#
#   ※ /motor_pwm, /steer_pwm 은 발행하지 않는다 — kasa 펌웨어가 PWM 을 텔레메트리로
#     내보내지 않는다. white 쪽 구독자도 없었으므로(로스백 진단 전용이었다) 그냥 사라진다.
#
# ══════════════════════════════════════════════════════════════════════════════
#  ★★ 단위·부호 규약 (이식에서 제일 많이 틀리는 곳) ★★
# ══════════════════════════════════════════════════════════════════════════════
#  1) /cmd_vel_raw 의 linear.x 는 **m/s 가 아니라 펄스**다.
#     m/s ↔ 펄스 환산은 발행하는 쪽(white/kasa_units.py)이 하고, 이 노드는 정수 클램프만
#     한다. 그래야 이 노드가 차량 제원(타이어·PPR)을 몰라도 되고, 제원이 바뀌어도
#     이 파일을 안 고친다.
#
#  2) ★조향 부호는 ROS 와 보드가 같다 (− 좌 / + 우) → 이 노드는 뒤집지 않는다★
#     [2026-08-04 개정] 예전에는 "ROS 안은 white 부호(+좌), 여기서 반전"이었다. 그런데
#     GUI 의 가로 조향 레버는 왼쪽 끝이 −40 인데 +가 좌회전이면 **레버 방향과 바퀴 방향이
#     반대**가 된다(nxde master 실차 시험에서 확인). 그래서 ROS 토픽 전체를 kasa B보드
#     부호로 통일했다 — 화면·토픽·시리얼·펌웨어가 전부 같은 부호를 쓴다.
#       kasa B보드 : − 좌 / + 우 (kasa_0804_B.ino angleToPot: −40 → RAW_LEFT_LIMIT(576))
#     → steer_invert 기본값은 **False**. 배선/펌웨어를 뒤집었을 때만 True 로 쓴다.
#     ※ driving.py 제어기 내부는 여전히 '+좌'로 튜닝되어 있고, 그 반전은
#       driving.publish_cmd 의 to_ros_steer() 한 줄에서만 일어난다(거기서 이미 끝났다).
#
#  2-1) 브레이크 단계는 /brake_level (Int32) 로 받는다.
#     Twist 에는 브레이크 필드가 없어 별 토픽을 쓴다. 값은 ★0 / 1 / 2 단계★ 이며
#     0~255 PWM 이 아니다(kasa_0804_B.ino). 안 오면 0(놓음)으로 둔다.
#     자율주행 경로에서만 반영된다 — E-stop 과 수동조종에서는 아래 상태판단이 우선한다.
#
#  3) /encoder 는 좌·우 펄스의 **합**이다 (평균이 아니다).
#     합/평균은 어차피 상수배 차이이고, 소비측(white)에서 TICKS_PER_REV 를 2배(192)로
#     잡으면 결과 m/s 는 평균과 완전히 동일하다. 그런데 /encoder 는 Int32 라서 평균을
#     쓰면 (0,1) → 0.5 → 정수화로 정보가 깨진다. 합은 그 손실이 없고 양자화 눈금도
#     절반(0.884 → 0.442 m/s)이 된다. → 이 파일은 합, white 는 192.
#
#  4) 후진은 없다. A보드가 음수를 받지 않으므로 주행값은 항상 0 이상으로 클램프한다.
#     (후진이 필요하면 사람이 수동조종 모드에서 한다는 전제)
#
#  5) 브레이크는 0~255 PWM 이 아니라 **단계 0/1/2** 다 (kasa_0804_B.ino).
#       0 = 놓음 / 1 = 약한 브레이킹(행정 1/3) / 2 = 풀브레이킹
#     범위 밖 값은 B보드가 브레이크 필드만 무시한다.
#
# ══════════════════════════════════════════════════════════════════════════════
#  ★★ 주행 상태 판단 (우선순위 순서 그대로) ★★
# ══════════════════════════════════════════════════════════════════════════════
#  ★★ 리니어(브레이크)는 ROS 가 보낸 브레이크 필드만큼만 움직인다 ★★
#     펌웨어는 시킨 대로만 작동한다 — 스스로 체결하거나 복귀하지 않는다.
#     따라서 "E-STOP 도 아닌데 리니어가 튀어나왔다" 면 원인은 100% 이 파일이 보낸 값이다.
#     [2026-08-04] 실제로 그랬다 — 아래 (2) 에 '자율→수동 전환 엣지에서 2단 체결' 래치가
#     있었고 스위치를 내리는 순간 리니어가 브레이크 페달을 밟았다.
#     ★그 로직을 완전히 제거했다★ (파라미터 manual_brake_level·manual_release_raw 도 삭제).
#     지금 ROS 가 리니어를 움직이는 경로는 둘뿐이고 전부 '명시적 지시'다:
#       · /brake_level (GUI 레버 · camera_judgment) — 사람/판단이 직접 요청한 값
#       · stop_brake_level  (자율 정지 시, 기본 0)
#
#  (1) E-stop 중        : A="0", B="x,0"
#      브레이크 0 을 보낸다 — 모드 전환이나 e-stop 자체가 제동 지시는 아니다.
#      "x,0" 의 뜻은 **해제 직후에 적용될 마지막 명령을 안전한 값으로 두는 것**이다
#      (조향 힘빼기 = 사람이 핸들을 잡고 있어도 급조향이 없다).
#
#  (2) 수동조종 모드    : A=페달 펄스 ★또는★ ROS 지정펄스, B="x,0"
#      D5 스위치가 개방(모드 0)인 동안. 사람이 핸들과 페달을 직접 잡으므로
#        - 조향은 'x'(힘빼기) — DC모터에 힘이 들어가면 사람이 핸들을 못 돌린다
#        - 브레이크는 ★항상 0★ — 제동은 사람 발이 한다. ROS 가 개입하지 않는다.
#        - 주행 펄스는 ★쓰로틀 우선★ 아래 순서로 정해진다:
#            ① 페달을 밟고 있으면(환산 펄스 > 0) → 무조건 페달값
#            ② 발을 뗐고 /control_state=True 면 → /cmd_vel_raw 의 지정 펄스
#            ③ 그 외 → 0
#      ★[2026-08-07] ②가 새로 생겼다★ 예전에는 수동에서 ROS 명령을 통째로 무시했다.
#        그래서 '수동조종으로 매핑을 시작할 때 페달 없이 곧게 굴려 초기 헤딩을 잡는'
#        일(white806)을 하려면 모드를 자율로 속이는 수밖에 없었는데, 그 오버라이드
#        (/vehicle_mode_cmd)를 없애는 대신 이 경로를 열었다.
#        ①이 ②보다 앞서므로 ★사람이 밟는 순간 소프트웨어 값은 즉시 밀려난다★.
#        ②에 /control_state 게이트를 둔 이유는, 없으면 아무 노드가 남긴 낡은
#        /cmd_vel_raw 하나로 수동 중인 차가 밀려 나가기 때문이다.
#
#  (3) /control_state=False : A="0", B="<마지막 조향각>,<stop_brake_level>"
#      driving.py 가 정지를 지시한 상태(instant_stop / 경로 미로드 / STOP 명령).
#      조향각을 0 으로 리셋하지 않고 마지막 값을 유지한다 — 정지 순간에 바퀴가 정면으로
#      튀는 것을 막는다(white/motor.py 가 S,0 만 보내고 조향을 건드리지 않았던 것과 같은 태도).
#
#  (4) 정상 자율주행    : A="<펄스>", B="<조향각>,<stop_brake_level 아님, 0>"
#
# ══════════════════════════════════════════════════════════════════════════════
#  ★★ 전송 정책 ★★
#   - TX 는 TX_PERIOD_S(0.05s) 타이머에서 돌고, **값이 바뀌었을 때 또는 KEEPALIVE_S 마다**
#     만 실제로 시리얼에 쓴다. 매 주기 무조건 쓰면 B보드 handleLine 이 매번
#     steer_state=ST_ACTIVE 로 되돌려 조향 도달판정(SETTLE_MS=500ms)이 영구히 성립하지
#     않고, PD 가 목표 근처에서 계속 힘을 준다.
#   - A보드 펌웨어에는 무입력 타임아웃이 없다(0713에서 제거). 마지막 명령을 계속 물고
#     있으므로 ★종료 시 정지값이 반드시 시리얼까지 나가야 한다★ → stop_and_close.
#   - A보드로는 항상 **단일값**을 보낸다. 콤마 2값은 16~255 를 '직접 PWM(무보호 경로)'
#     으로 해석하는데, 자율주행에서 그 경로를 쓸 이유가 없고 오발동만 위험하다.
#
#  ★★ 연결 정책 (2026-08-04 개편) ★★
#   - ★생성자는 블로킹하지 않는다★ 예전에는 두 보드를 다 찾을 때까지 __init__ 안에서
#     돌았다. 그러면 보드가 안 꽂혀 있는 동안 노드가 spin 조차 못 해서 /board_status 도,
#     구독도 살아나지 않았다. 이제 탐색·재연결은 전부 데몬 스레드가 담당하고, 노드는
#     즉시 뜬다 → 보드가 없어도 나머지 스택(GPS/IMU/카메라/판단)이 정상 기동한다.
#   - ★도중에 끊겨도 재연결한다★ read/write 에서 SerialException·OSError 가 나면 그 보드만
#     닫고 None 으로 떨어뜨린다(_drop_board). 같은 스레드가 그것을 보고 다시 스캔한다.
#     한쪽만 빠지면 나머지 한쪽은 계속 정상 동작한다.
#   - 재연결 중에는 그 보드로 나가는 write 가 조용히 버려진다(send_line 의 ser is None).
#     A보드가 복귀하면 다음 TX 주기에 최신 명령이 즉시 나간다(변경감지 캐시를 비운다).
#
#  ★★ RX 정책 ★★
#   - SERIAL_POLL_S = 0.05 (A·B 텔레메트리 주기 50ms 와 일치).
#     kasa_ws 원본은 0.1 이었는데, poll 마다 최신 줄 하나만 쓰므로(latest = texts[-1])
#     보드가 보낸 텔레메트리의 절반을 버리고 있었다. 0.05 로 맞추면 버리지 않는다.
#   - 그래도 근본적인 측정 공백은 남는다: 펄스 필드는 '직전 20ms 창'의 카운트인데 보고는
#     50ms 마다다 → 50ms 중 30ms 는 계측되지 않는다. gps_imu 의 DR 거리적분이 그만큼
#     거칠다는 뜻이다(펌웨어에 누적 카운터를 넣지 않기로 결정했으므로 감수한다).
#
# ══════════════════════════════════════════════════════════════════════════════
#  실행 : ros2 launch white one_launch.py     ← ★이 노드를 함께 띄운다★
#         ros2 run nxde arduino --ros-args -p baud:=115200   (단독 실행도 가능)
#  정상 동작 중에는 로그를 내지 않는다 (보드감지 / estop·모드 전환 / 오류 시에만).
#
#  ★★ [2026-08-05] 이 노드는 자립형이다 — 어떤 런치도, 다른 패키지도 필요하지 않다 ★★
#    nxde 에는 런치파일이 없다. 차량을 움직이는 최소 단위가 이 노드 하나이므로
#    의존성을 늘리지 않는다:
#      · 포트 탐색표(아두이노 VID + GPS/IMU 제외목록)를 ★이 파일이 직접 소유★한다.
#        예전에는 nxde/ports.py 에 있었고 g.launch.py 가 GPS/IMU 경로를 확정해
#        exclude_ports 로 넘겨줬는데, 런치가 갈라지면서 그 전달 경로가 끊겼다.
#        → 지금은 이 노드가 GPS/IMU VID/PID 를 스스로 보고 그 포트를 제외한다.
#      · white 패키지를 import 하지 않는다(자율주행 스택이 없어도 차는 움직여야 한다).
#    실행 조합:
#      차량 구동만        : ros2 run nxde arduino  +  ros2 run nxde master
#      조이스틱 조종      : ros2 run nxde arduino  +  ros2 run nxde joystick
#      자율주행          : ros2 launch white one_launch.py (이 노드를 포함해 함께 뜬다)

import os
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String
from geometry_msgs.msg import Twist

import serial
try:
    from serial.tools import list_ports
except Exception:      # pyserial 이 없는 환경에서도 import 는 되게
    list_ports = None

from nxde.proc_guard import watch_parent


BAUD_RATE = 115200

# ── A보드 프로토콜 상한 (kasa_0730_A.ino) ──
# 단일값 입력은 0~PULSE_MAX 만 유효하고 그 외는 펌웨어가 줄 통째로 무시한다.
PULSE_MIN, PULSE_MAX = 0, 15

# ── B보드 프로토콜 (kasa_0804_B.ino) ──
STEER_DEG_MAX = 40           # 입력 조향각 클램프 (STEER_ANGLE_MAX 와 동일해야 한다)
BRAKE_LEVEL_MAX = 2          # 0 = 놓음 / 1 = 약 / 2 = 풀
STEER_RELEASE_TOKEN = 'x'    # 조향 힘빼기 ([0730-2])

# ── 쓰로틀 페달 raw → 펄스 환산 (실측 2026-07-30, master.py 와 동일 값) ──
THROTTLE_RAW_MIN = 177       # 페달을 완전히 놓았을 때
THROTTLE_RAW_MAX = 800       # 끝까지 밟았을 때
ADC_MAX = 1023

# ── 주기 ──
SERIAL_POLL_S = 0.05         # 시리얼 수신 폴링 + 텔레메트리 발행 (보드 50ms 와 일치)
TX_PERIOD_S   = 0.05         # 전송 판정 주기 (실제 write 는 변경/keepalive 시에만)
KEEPALIVE_S   = 1.0          # 값이 안 바뀌어도 이 간격으로는 한 번 재전송

# ── 보드 탐색 ──
DETECT_READ_S  = 5.0         # 포트 하나를 A/B 로 식별하기 위해 읽어보는 시간
DETECT_RETRY_S = 3.0         # 두 보드를 아직 못 찾았을 때 재스캔 간격
DETECT_OPEN_RETRY   = 5      # open 간헐 실패 시 재시도 횟수
DETECT_OPEN_DELAY_S = 1.0
PORT_SETTLE_S       = 0.5    # 한 보드를 이미 연 상태에서 다음 포트를 열기 전 USB 안정화 대기
STOP_FLUSH_S        = 0.15   # 종료 직전 정지값이 실제로 시리얼로 나갈 시간

# 포트는 항상 배타적으로 연다. POSIX 는 기본이 비배타적이라 두 프로세스가 같은
# /dev/ttyACM* 을 동시에 열 수 있고, 그러면 잔재 노드와 새 노드가 같은 보드에 명령을
# 섞어 쓴다. flock(pyserial 3.3+)으로 막는다.
SERIAL_EXCLUSIVE = True

BUSY_HINT = ("재시도로는 풀리지 않습니다 — 이전 런치의 잔재라면 "
             "`pkill -f nxde.arduino` 또는 `fuser -k /dev/ttyACM*` 로 정리하고, "
             "시리얼 모니터가 열려 있으면 닫으세요. "
             "(권한 오류라면 dialout 그룹 확인: sudo usermod -aG dialout $USER)")


def _round_half_away(x):
    """아두이노 round() 매크로와 같은 반올림 (파이썬 내장 round 는 은행가 반올림이라 다름)."""
    return int(x + 0.5) if x >= 0 else -int(-x + 0.5)


def is_busy_error(exc):
    """포트가 이미 점유되어(또는 권한이 없어) 열리지 않은 상황인지.

    POSIX: exclusive(flock) 충돌은 EAGAIN(11), 권한 부족은 EACCES(13)."""
    lowered = str(exc).lower()
    return ('permission denied' in lowered
            or 'resource temporarily unavailable' in lowered
            or 'errno 11' in lowered or 'errno 13' in lowered)


# ══════════════════════════════════════════════════════════════════════════════
#  포트 탐색 — ★이 파일이 소유한다★ (구 nxde/ports.py 에서 흡수, 2026-08-05)
# ══════════════════════════════════════════════════════════════════════════════
#  아두이노 계열 USB-serial VID: Arduino 정품(2341) / CH340 클론(1A86) / Arduino LLC(2A03)
#  ★ A/B 두 대가 같은 VID/PID 라서 VID/PID 로는 역할을 구분할 수 없다 ★ 실제 식별은
#  포트를 열어 첫 텔레메트리 접두어('S,'=A / 'P,'=B)로 한다 — identify_port().
ARDUINO_VIDS = {0x2341, 0x1A86, 0x2A03}

#  ★★ GPS·IMU 를 스스로 제외한다 ★★
#    GPS(u-blox)·IMU(CP210x)도 같은 /dev/ttyACM*·/dev/ttyUSB* 대역에 있다. 제외하지 않으면
#      (1) 탐색이 포트당 DETECT_READ_S(5초)씩 느려지고
#      (2) ★배타 open 이 충돌해 GPS/IMU 드라이버가 자기 포트를 못 잡는다★
#         — RTK 가 안 붙는 증상의 유력한 원인이었다. nmea 드라이버는 respawn 으로 계속
#           다시 뜨는데, 그 사이 이 노드가 GPS 포트를 5초씩 물면 서로 밀어낸다.
#    예전에는 g.launch.py 가 GPS/IMU 경로를 확정해 exclude_ports 로 넘겨줬다. 런치가
#    갈라진 뒤로는 그 전달이 끊기므로 ★여기서 직접 VID/PID 를 보고 건너뛴다★.
#    (one_launch.py 가 exclude_ports 로 실제 경로를 넘겨주면 그것도 함께 반영한다 —
#     둘은 배타가 아니라 합집합이다. 같은 VID 장치가 여러 개일 때 런치가 확정한 경로가
#     더 정확하므로 둘 다 받는다.)
NON_ARDUINO_VIDPID = [
    (0x1546, 0x01A9),   # u-blox 9 계열 GPS
    (0x1546, 0x01A8),   # u-blox 8 계열 GPS
    (0x10C4, 0xEA60),   # iAHRS / CP210x IMU
]
#  udev 심볼릭링크(white/ports.py·99-white.rules 와 이름 일치) — 실제 경로로 풀어 제외한다
NON_ARDUINO_SYMLINKS = ('/dev/gps', '/dev/imu')


def _comports():
    if list_ports is None:
        return []
    try:
        return sorted(list_ports.comports(), key=lambda p: p.device)
    except Exception:
        return []


def looks_like_arduino(port):
    """VID(정품/CH340/Arduino LLC) 또는 설명으로 아두이노 계열 여부 판정."""
    if port.vid in ARDUINO_VIDS:
        return True
    desc = (port.description or '').lower()
    return ('arduino' in desc) or ('ch340' in desc)


def is_known_non_arduino(port):
    """GPS/IMU 로 알려진 VID/PID 인지. True 면 열어보지 않는다."""
    return any(port.vid == vid and port.pid == pid for vid, pid in NON_ARDUINO_VIDPID)


def candidate_ports(exclude=None):
    """아두이노 A/B 탐색 대상 포트 목록 (/dev/ttyACM* · /dev/ttyUSB*).

    아두이노로 추정되는 포트(VID/설명 일치)를 앞에, 그 외 USB-serial 포트를 뒤에 둔다.
    GPS/IMU 로 알려진 VID/PID 와 udev 링크(/dev/gps, /dev/imu)는 항상 제외한다.

    exclude : 추가로 제외할 경로(one_launch.py 가 넘겨주는 확정 경로). 심볼릭링크로
      들어와도 realpath 까지 함께 막는다."""
    resolved_exclude = set()
    for path in list(exclude or ()) + list(NON_ARDUINO_SYMLINKS):
        if not path:
            continue
        resolved_exclude.add(path)
        try:
            resolved_exclude.add(os.path.realpath(path))
        except OSError:
            pass

    likely, others = [], []
    for p in _comports():
        dev = p.device
        if not (('ACM' in dev) or ('USB' in dev)):
            continue
        if dev in resolved_exclude:
            continue
        try:
            if os.path.realpath(dev) in resolved_exclude:
                continue
        except OSError:
            pass
        if is_known_non_arduino(p):
            continue          # GPS/IMU — 열면 그쪽 드라이버가 포트를 못 잡는다
        (likely if looks_like_arduino(p) else others).append(dev)
    return likely + others


def open_serial(port, baud, logger):
    """포트를 재시도하며 연다. 끝내 실패하면 None (예외를 밖으로 던지지 않는다 —
    한 포트의 open 실패가 노드 전체를 죽이지 않도록)."""
    for attempt in range(1, DETECT_OPEN_RETRY + 1):
        try:
            return serial.Serial(port, baud, timeout=0.2, exclusive=SERIAL_EXCLUSIVE)
        except (serial.SerialException, OSError) as e:
            if attempt < DETECT_OPEN_RETRY:
                logger.warn(f"{port} 열기 실패({attempt}/{DETECT_OPEN_RETRY}), "
                            f"{DETECT_OPEN_DELAY_S}s 후 재시도: {e}")
                time.sleep(DETECT_OPEN_DELAY_S)
            else:
                logger.warn(f"{port} 열기 최종 실패({DETECT_OPEN_RETRY}회 시도): {e}")
                if is_busy_error(e):
                    logger.error(f"{port}를 열 수 없습니다. {BUSY_HINT}")
    return None


def identify_port(port, baud, logger):
    """포트를 열어 DETECT_READ_S 동안 읽으며 첫 'S,'/'P,' 줄로 보드를 식별.
       반환: ('A'|'B'|None, serial.Serial 또는 None(실패 시))"""
    ser = open_serial(port, baud, logger)
    if ser is None:
        return None, None

    buf = b''
    deadline = time.monotonic() + DETECT_READ_S
    while time.monotonic() < deadline:
        try:
            data = ser.read(256)
        except (serial.SerialException, OSError):
            try:
                ser.close()
            except (serial.SerialException, OSError):
                pass
            return None, None
        if data:
            buf += data
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                text = line.decode('ascii', errors='ignore').strip()
                if text.startswith('S,'):
                    return 'A', ser
                if text.startswith('P,'):
                    return 'B', ser

    ser.close()
    return None, None


class Arduino(Node):
    def __init__(self):
        super().__init__('arduino')

        # ── 파라미터 ──
        self.baud = int(self.declare_parameter('baud', BAUD_RATE).value)
        # ★ 조향 부호 반전 — 기본 False(반전 없음) ★ ROS 토픽과 B보드가 같은 규약
        #   (− 좌 / + 우)을 쓴다. 배선이나 펌웨어를 뒤집었을 때만 True 로 둔다.
        #   자세한 경위는 파일 헤더 규약 2) 참고.
        self.steer_invert = bool(self.declare_parameter('steer_invert', False).value)
        # /control_state=False 일 때 걸 브레이크 단계.
        #   0 = 코스트(white/motor.py 의 'S,0' 과 같은 동작, 기본)
        #   1 = 약한 브레이킹으로 더 빨리 세우고 싶을 때
        self.stop_brake_level = int(
            self.declare_parameter('stop_brake_level', 0).value)
        # ★[2026-08-04] manual_brake_level / manual_release_raw 를 삭제했다★
        #   '수동조종 진입 시 브레이크 2단 체결 → 페달을 밟으면 해제' 로직 자체를 없앴다.
        #   모드 전환은 제동 지시가 아니고, 실차에서 스위치를 수동으로 내리는 순간 리니어가
        #   브레이크 페달을 밟고 튀어나왔다(E-STOP 도 아닌데). 수동에서는 브레이크를 항상
        #   0 으로 보낸다 — 제동은 사람 발이 한다. compose() (2) 분기 참고.
        # 수동조종에서 페달 최대치가 대응할 펄스. 기본은 A보드 상한(15 ≈ 47km/h)으로
        # kasa_ws master.py 와 동일하게 두었다. 초기 시험에서는 낮춰 두는 것이 안전하다.
        self.manual_pulse_max = int(
            self.declare_parameter('manual_pulse_max', PULSE_MAX).value)
        self.throttle_raw_min = int(
            self.declare_parameter('throttle_raw_min', THROTTLE_RAW_MIN).value)
        self.throttle_raw_max = int(
            self.declare_parameter('throttle_raw_max', THROTTLE_RAW_MAX).value)
        # ★ 아두이노가 아닌 장치 경로 — 탐색에서 추가로 제외한다 ★
        #   ※ 안 넘겨도 된다 — candidate_ports() 가 GPS/IMU VID/PID(NON_ARDUINO_VIDPID)와
        #     udev 링크(/dev/gps · /dev/imu)를 이미 스스로 걸러낸다. 이 파라미터는 같은
        #     VID 장치가 여러 개 꽂혀 있어 런치가 확정한 경로가 더 정확할 때 쓴다
        #     (one_launch.py 가 GPS/IMU 실경로를 넘겨준다).
        self.exclude_ports = [
            str(p) for p in self.declare_parameter('exclude_ports', ['']).value if str(p)]

        # ── 시리얼 ──
        # ★ _running / _link_thread 를 여기서 먼저 만든다 ★ watch_parent 가 등록하는
        #   stop_and_close 는 언제든 불릴 수 있는데, 그 안에서 이 속성들을 읽는다.
        self._running = False
        self._link_thread = None
        self.ser_a = None
        self.ser_b = None
        self.rx_buf_a = b''
        self.rx_buf_b = b''
        self.last_line_a = None
        self.last_line_b = None

        # ── 보드 → ROS 최신 상태 (STOP·형식오류 시 마지막 값 유지) ──
        self.pulse_l = 0
        self.pulse_r = 0
        self.angle_board = 0       # B보드 실측 조향각 (− 좌 / + 우 = ROS 규약과 동일)
        self.throttle_raw = 0
        # B보드 D5 주행모드. ★페일세이프로 수동(False)에서 시작한다★ — 첫 텔레메트리를
        # 받기 전에 '자율'로 오인해 자동 명령이 나가는 것보다 수동으로 보는 편이 안전하다.
        self.switch_mode = False   # ← B보드 D5 원값 (물리 스위치가 말하는 것)
        self.estop_active = False

        # [2026-08-07] 소프트웨어 모드 오버라이드(mode_override / /vehicle_mode_cmd)를
        #   삭제했다. 주행모드의 소유자는 물리 스위치 하나다 — auto_mode 주석 참고.

        # 수동조종에서 '지금 페달을 밟고 있다'고 볼 최소 펄스. 로그용 상태이기도 하다.
        self._manual_src = None    # 'pedal' / 'ros' — 바뀔 때만 로그를 남긴다

        # ── ROS → 보드 명령 캐시 ──
        self.cmd_pulse = 0         # /cmd_vel_raw linear.x (펄스, 0~15)
        self.cmd_angle = 0         # /cmd_vel_raw angular.z (− 좌 / + 우, -40~40)
        self.control_enabled = False   # /control_state. 시작은 False(정지)
        self.cmd_brake = 0         # /brake_level (0/1/2). 안 오면 0(놓음)

        # ★[2026-08-04] 수동조종 브레이크 래치 상태(_manual_brake / _manual_released /
        #   _prev_auto_mode)를 삭제했다★ 래치 로직 자체가 없어져 전환 엣지를 볼 이유가 없다.
        #   수동에서는 브레이크가 항상 0 이다.

        # ── 전송 변경 감지 ──
        self._last_a = None
        self._last_b = None
        self._last_a_t = 0.0
        self._last_b_t = 0.0

        # ── 퍼블리셔 ──
        self.pub_encoder = self.create_publisher(Int32, '/encoder', 10)
        self.pub_steer_angle = self.create_publisher(Int32, '/steer_angle_measured', 10)
        self.pub_mode = self.create_publisher(Bool, '/vehicle_mode', 10)
        self.pub_throttle = self.create_publisher(Int32, '/throttle_pedal', 10)
        # ★ A보드로 실제 보낸 주행 목표펄스 ★ 자율=계획값 / 수동조종=페달 환산값.
        #   수동조종 수집(mapping)의 라벨 ①이 이 값이다 — 환산 규칙(throttle_raw_min/max,
        #   manual_pulse_max)이 이 노드에만 있으므로, 여기서 발행해야 소비측이 규칙을
        #   복제하지 않는다(복제하면 파라미터를 바꿀 때 조용히 어긋난다).
        self.pub_drive_pulse = self.create_publisher(Int32, '/drive_pulse_cmd', 10)
        self.pub_estop = self.create_publisher(Bool, '/estop', 10)
        self.pub_status = self.create_publisher(String, '/board_status', 10)

        # ── 서브스크라이버 ──
        self.create_subscription(Twist, '/cmd_vel_raw', self.cb_cmd_vel, 10)
        self.create_subscription(Bool, '/control_state', self.cb_control_state, 10)
        # [2026-08-07] /vehicle_mode_cmd 구독을 삭제했다 — 주행모드는 물리 스위치만
        #   바꿀 수 있다. 수동조종에서 ROS 펄스가 필요한 경우는 compose() (2) 가
        #   직접 처리하므로 모드를 속일 이유가 없어졌다.
        # 브레이크 단계(0/1/2). Twist 에 필드가 없어 별 토픽으로 받는다. 선택 입력 —
        # 아무도 발행하지 않으면 0(놓음)으로 유지된다(white 의 driving 은 발행하지 않는다).
        self.create_subscription(Int32, '/brake_level', self.cb_brake_level, 10)

        # ★ 포트를 열기 전에 부모 감시를 걸어둔다 ★
        #   탐색 스레드가 포트를 여는 도중에 런치가 내려가면 이 프로세스가 고아로 남아
        #   포트를 물고 있게 된다. 감시를 탐색보다 먼저 걸어야 그 구간까지 커버된다.
        watch_parent(cleanup=self.stop_and_close)

        # ★ 생성자는 블로킹하지 않는다 ★ 보드가 안 꽂혀 있어도 노드는 즉시 뜬다.
        #   탐색과 재연결을 같은 데몬 스레드가 담당한다(_link_loop).
        self._running = True
        self._link_thread = threading.Thread(
            target=self._link_loop, name='board_link', daemon=True)
        self._link_thread.start()

        self.create_timer(SERIAL_POLL_S, self.on_rx_timer)
        self.create_timer(TX_PERIOD_S, self.on_tx_timer)

        self.get_logger().info(
            "아두이노 브리지 시작 — A/B 보드 탐색은 백그라운드에서 진행합니다 "
            "(연결 전에도 노드는 정상 동작하며, 꽂는 순간 자동으로 붙습니다)")

    # ═══════════════════════════════════════════════════════════════
    #  보드 링크 관리 : 최초 탐색 + 도중 단절 재연결 (전용 스레드)
    # ═══════════════════════════════════════════════════════════════
    def _link_loop(self):
        """A/B 보드가 둘 다 붙어 있을 때까지(그리고 빠질 때마다 다시) 스캔한다.

        ★ 이 루프가 최초 연결과 재연결을 겸한다 ★ read/write 오류로 _drop_board 가
        포트를 None 으로 떨어뜨리면 다음 사이클이 그것을 보고 다시 찾는다. 한쪽만
        빠지면 나머지 한쪽은 계속 정상 동작한다(그쪽 포트는 건드리지 않는다).

        rclpy 콜백이 아니라 전용 스레드이므로 여기서 time.sleep / 블로킹 read 를 해도
        제어 타이머(on_tx_timer/on_rx_timer)를 막지 않는다."""
        first_round = True
        while self._running and rclpy.ok():
            if self.ser_a is not None and self.ser_b is not None:
                time.sleep(DETECT_RETRY_S)
                first_round = False
                continue

            missing = [n for n, s in (('A', self.ser_a), ('B', self.ser_b)) if s is None]
            if first_round:
                self.get_logger().info(f"{'/'.join(missing)}보드 탐색 시작...")
            self.publish_status()

            self._scan_once()

            if self.ser_a is None or self.ser_b is None:
                missing = [n for n, s in (('A', self.ser_a), ('B', self.ser_b)) if s is None]
                self.get_logger().warn(
                    f"{'/'.join(missing)}보드 미발견, {DETECT_RETRY_S}s 후 재스캔 "
                    f"(연결·전원·USB 케이블 확인)", throttle_duration_sec=15.0)
                self.publish_status()
                time.sleep(DETECT_RETRY_S)
            elif not first_round:
                self.get_logger().info("A/B 보드 모두 연결됨")
            first_round = False

    def _scan_once(self):
        """후보 포트를 한 바퀴 돌며 아직 못 찾은 보드를 채운다."""
        owned = {s.port for s in (self.ser_a, self.ser_b) if s is not None}
        exclude = list(self.exclude_ports) + list(owned)

        for port in candidate_ports(exclude=exclude):
            if not (self._running and rclpy.ok()):
                return
            if self.ser_a is not None and self.ser_b is not None:
                return

            # 이미 한 보드를 연 상태면 USB 컨트롤러가 안정되도록 잠깐 쉰다
            if self.ser_a is not None or self.ser_b is not None:
                time.sleep(PORT_SETTLE_S)

            try:
                role, ser = identify_port(port, self.baud, self.get_logger())
            except Exception as e:   # 한 포트의 예기치 못한 오류가 스레드를 죽이지 않도록
                self.get_logger().warn(f"{port} 감지 중 오류(건너뜀): {e}")
                continue

            if role == 'A' and self.ser_a is None:
                ser.timeout = 0            # 이후 폴링은 논블로킹
                self.rx_buf_a = b''        # 재연결이면 이전 버퍼 잔재를 버린다
                self._last_a = None        # 변경감지 캐시 초기화 → 다음 TX 에서 즉시 재전송
                self.ser_a = ser
                self.get_logger().info(f"[A보드 연결] {port} (인휠 PID + 주행펄스)")
                self.publish_status()
            elif role == 'B' and self.ser_b is None:
                ser.timeout = 0
                self.rx_buf_b = b''
                self._last_b = None
                self.ser_b = ser
                self.get_logger().info(f"[B보드 연결] {port} (조향 + 제동 + 모드)")
                self.publish_status()
            elif ser is not None:
                ser.close()

    def _drop_board(self, which, reason):
        """단절된 보드의 포트를 닫고 None 으로 떨어뜨린다 → _link_loop 가 다시 찾는다.

        ★ 텔레메트리 캐시(펄스/각도/모드)는 지우지 않는다 ★ 마지막 정상값을 유지하는
        것이 STOP·형식오류 처리와 같은 태도이고, 재연결 직후 값이 0 으로 튀는 것보다 낫다.
        단, 최신 줄(last_line_*)은 지운다 — 'STOP' 이 남아 있으면 보드가 빠진 뒤에도
        e-stop 이 걸린 것으로 오판한다."""
        ser = self.ser_a if which == 'a' else self.ser_b
        if ser is None:
            return
        self.get_logger().error(
            f"[{which.upper()}보드 단절] {getattr(ser, 'port', '?')}: {reason} → 재연결 시도")
        try:
            ser.close()
        except (serial.SerialException, OSError):
            pass
        if which == 'a':
            self.ser_a = None
            self.rx_buf_a = b''
            self.last_line_a = None
        else:
            self.ser_b = None
            self.rx_buf_b = b''
            self.last_line_b = None
        self.publish_status()

    # ═══════════════════════════════════════════════════════════════
    #  ROS 콜백
    # ═══════════════════════════════════════════════════════════════
    def cb_cmd_vel(self, msg: Twist):
        """/cmd_vel_raw — linear.x = 주행 목표펄스(0~15), angular.z = 조향각(− 좌 / + 우).

        ★ linear.x 는 m/s 가 아니다 ★ 환산은 white/kasa_units.py 가 발행 전에 끝낸다.
        ★ angular.z 부호는 이미 보드 규약과 같다 ★ 여기서 뒤집지 않는다(헤더 규약 2).
        후진은 없으므로 음수는 0 으로 클램프한다(A보드가 음수를 받지 않는다)."""
        pulse = _round_half_away(float(msg.linear.x))
        self.cmd_pulse = max(PULSE_MIN, min(PULSE_MAX, pulse))

        angle = _round_half_away(float(msg.angular.z))
        self.cmd_angle = max(-STEER_DEG_MAX, min(STEER_DEG_MAX, angle))

    def cb_control_state(self, msg: Bool):
        new_state = bool(msg.data)
        if new_state != self.control_enabled:
            self.get_logger().info(
                f"/control_state → {'구동 허용' if new_state else '정지'}")
        self.control_enabled = new_state

    def cb_brake_level(self, msg: Int32):
        """/brake_level — 브레이크 ★단계 0/1/2★ (0~255 PWM 이 아니다).

        범위 밖 값은 무시한다. B보드도 범위 밖이면 브레이크 필드만 버리지만, 여기서
        먼저 걸러 '왜 안 걸리는지' 로그로 드러나게 한다."""
        level = int(msg.data)
        if not (0 <= level <= BRAKE_LEVEL_MAX):
            self.get_logger().warn(
                f"/brake_level={level} 은 허용범위(0~{BRAKE_LEVEL_MAX}) 밖 — 무시. "
                f"★0~255 PWM 이 아니라 단계값이다★ (0 놓음 / 1 약 / 2 풀)",
                throttle_duration_sec=5.0)
            return
        if level != self.cmd_brake:
            self.get_logger().info(f"/brake_level → {level}단")
        self.cmd_brake = level

    # ═══════════════════════════════════════════════════════════════
    #  ROS → 보드 : 명령 조립 + 전송
    # ═══════════════════════════════════════════════════════════════
    def throttle_to_pulse(self, raw):
        """쓰로틀 페달 raw(0~1023) → 주행펄스. 수동조종 모드 전용.

        throttle_raw_min~max 구간을 0~manual_pulse_max 에 선형 대응시킨다(데드존 없음 —
        페달을 놓은 상태의 잔노이즈는 throttle_raw_min 을 실측값으로 올려 잡아 흡수한다)."""
        raw = max(0, min(ADC_MAX, int(raw)))
        lo, hi = self.throttle_raw_min, self.throttle_raw_max
        if hi <= lo or raw <= lo:
            return 0
        frac = (raw - lo) / (hi - lo)
        return max(0, min(PULSE_MAX,
                          _round_half_away(frac * self.manual_pulse_max)))

    def to_board_angle(self, ros_deg):
        """ROS 조향각 → B보드로 보낼 값. ★기본은 그대로 통과다★ (같은 규약 − 좌 / + 우)

        steer_invert=True 일 때만 뒤집는다 — 배선이나 펌웨어를 바꿔 방향이 반대가 된
        경우의 탈출구다. 규약 2 참고."""
        deg = -ros_deg if self.steer_invert else ros_deg
        return max(-STEER_DEG_MAX, min(STEER_DEG_MAX, int(deg)))

    # ═══════════════════════════════════════════════════════════════
    #  주행모드 = ★물리 스위치(B보드 D5) 하나뿐★
    # ═══════════════════════════════════════════════════════════════
    @property
    def auto_mode(self):
        """실효 주행모드. ★물리 스위치가 유일한 소유자다★

        [2026-08-07] 소프트웨어 오버라이드(/vehicle_mode_cmd)를 삭제했다.
          예전에는 GUI 버튼으로 자율/수동을 덮어쓸 수 있었다. 그런데 모드는
          '사람이 핸들과 페달을 잡고 있는가'를 뜻하는 물리적 사실이라, 화면 클릭
          한 번으로 뒤집을 수 있으면 안 된다 — 사람이 운전대를 잡은 채 소프트웨어가
          자율로 바꾸면 조향모터에 힘이 들어간다.

          오버라이드가 필요했던 실제 이유는 '수동조종에서도 ROS 가 지정한 펄스를
          내보내고 싶다'였는데(white806 의 헤딩 초기화), 그건 compose() (2) 분기가
          직접 지원하도록 바꿔서 더 이상 모드를 속일 이유가 없다.
        """
        return self.switch_mode

    def compose(self):
        """현재 상태에서 A/B 보드로 보낼 페이로드 두 개를 만든다.

        반환: (a_payload, b_payload) — 둘 다 개행 없는 문자열.
        우선순위는 파일 헤더의 '주행 상태 판단' 순서와 같다."""

        # (1) E-stop — B보드가 리니어 2단 체결과 0단 복귀를 스스로 한다([0804-3]).
        #     ★ 여기서 수동 래치를 건드리지 않는다 ★ 해제 시 B보드가 이미 HOME(0단)으로
        #     돌아가는데 우리가 2단을 다시 물리면 그 복귀와 싸운다.
        if self.estop_active:
            return '0', f'{STEER_RELEASE_TOKEN},0'

        # (2) 수동조종 (D5 개방) — /control_state 와 무관하게 항상 이 경로
        #     ★[2026-08-04] '진입 시 브레이크 체결' 로직을 완전히 제거했다★
        #     예전에는 자율→수동 전환 엣지에서 manual_brake_level(2단)을 물고,
        #     쓰로틀 raw >= manual_release_raw 가 되면 풀었다. 그런데 모드 전환은 그 자체가
        #     제동 지시가 아니고, 실차에서 스위치를 수동으로 내리는 순간 리니어가 브레이크
        #     페달을 밟고 튀어나왔다(E-STOP 아닌데도). 사람이 넘겨받는 순간 페달이 물려
        #     있으면 오히려 출발도 못 한다.
        #     → 수동에서는 브레이크를 ★항상 0★ 으로 보낸다. 제동은 사람 발이 한다.
        #
        #     ★[2026-08-07] 수동에서도 ROS 지정 펄스를 받는다 — 단 쓰로틀이 최우선★
        #       페달을 밟고 있으면 무조건 페달값이다. 발을 뗀 동안에만 /cmd_vel_raw 의
        #       펄스를 쓴다. 사람이 운전대를 잡은 채 소프트웨어가 가속하는 일은 없고,
        #       사람이 개입하는 즉시(밟는 즉시) 소프트웨어 값은 밀려난다.
        #
        #       쓰임새 : white806 의 헤딩 초기화. 수동조종으로 매핑을 시작할 때 사람이
        #       페달을 밟지 않아도 차가 곧게 굴러가 초기 방위를 잡아야 한다.
        #
        #       ★/control_state 가 True 일 때만 유효하다★ 이 게이트가 없으면 아무
        #       노드나 발행한 낡은 /cmd_vel_raw 하나로 수동 중인 차가 밀려 나간다.
        #       조향은 그대로 힘빼기다 — 수동에서 핸들은 사람 것이다.
        if not self.auto_mode:
            pedal = self.throttle_to_pulse(self.throttle_raw)
            if pedal > 0:
                pulse, src = pedal, 'pedal'
            elif self.control_enabled:
                pulse, src = self.cmd_pulse, 'ros'
            else:
                pulse, src = 0, None
            if src != self._manual_src:
                self._manual_src = src
                if src == 'ros':
                    self.get_logger().info(
                        f"[수동조종] 페달 유휴 → ROS 지정펄스 사용 ({pulse})")
                elif src == 'pedal':
                    self.get_logger().info("[수동조종] 페달 입력 감지 → 페달 우선")
            return str(pulse), f'{STEER_RELEASE_TOKEN},0'

        # (3) ROS 가 정지를 지시한 상태. 조향각은 마지막 값을 유지한다(정면 급조향 방지).
        #     브레이크는 stop_brake_level 이 /brake_level 보다 우선한다 — '정지 지시'가
        #     더 강한 의도이므로, 그때 0 을 받고 있었다고 브레이크를 풀면 안 된다.
        if not self.control_enabled:
            brake = max(self.stop_brake_level, self.cmd_brake)
            brake = max(0, min(BRAKE_LEVEL_MAX, brake))
            return '0', f'{self.to_board_angle(self.cmd_angle)},{brake}'

        # (4) 정상 자율주행 — /brake_level 을 그대로 반영(안 오면 0)
        brake = max(0, min(BRAKE_LEVEL_MAX, self.cmd_brake))
        return str(self.cmd_pulse), f'{self.to_board_angle(self.cmd_angle)},{brake}'

    def on_tx_timer(self):
        """값이 바뀌었거나 KEEPALIVE_S 가 지났을 때만 실제로 시리얼에 쓴다.

        ★ 매 주기 무조건 쓰지 않는 이유 ★ B보드 handleLine 은 줄을 받을 때마다
        steer_state 를 ST_ACTIVE 로 되돌린다. 20Hz 로 계속 보내면 도달판정
        (SETTLE_MS=500ms)이 영구히 성립하지 않아 PD 가 목표 근처에서 계속 힘을 준다."""
        a_payload, b_payload = self.compose()
        now = time.monotonic()

        if a_payload != self._last_a or (now - self._last_a_t) >= KEEPALIVE_S:
            if self.send_line('a', a_payload):
                self._last_a = a_payload
                self._last_a_t = now

        if b_payload != self._last_b or (now - self._last_b_t) >= KEEPALIVE_S:
            if self.send_line('b', b_payload):
                self._last_b = b_payload
                self._last_b_t = now

        # ★ A보드로 실제 나간 주행 목표펄스를 그대로 발행한다 ★ 수동조종에서는 페달
        #   환산값이므로 mapping 의 수집 라벨 ①이 된다. 보드가 단절되어 전송이 안 된
        #   주기에도 '이번에 내려던 값'을 발행한다 — 라벨은 명령의 기록이기 때문이다.
        try:
            self.pub_drive_pulse.publish(Int32(data=int(a_payload)))
        except ValueError:
            pass   # a_payload 는 항상 정수 문자열이지만 방어적으로 둔다

    def send_line(self, which, text):
        """한 보드에 한 줄 전송. 성공하면 True.

        전송 실패는 곧 포트 단절이므로 _drop_board 로 넘겨 재연결 대상으로 만든다.
        False 를 돌려주면 호출측이 변경감지 캐시를 갱신하지 않아, 재연결 직후 같은 값이
        다시 전송된다(= 명령이 유실되지 않는다)."""
        ser = self.ser_a if which == 'a' else self.ser_b
        if ser is None:
            return False
        try:
            ser.write((text + '\n').encode('ascii'))
            return True
        except (serial.SerialException, OSError) as e:
            self._drop_board(which, f"전송 실패: {e}")
            return False

    # ═══════════════════════════════════════════════════════════════
    #  보드 → ROS : 수신 + 발행
    # ═══════════════════════════════════════════════════════════════
    def on_rx_timer(self):
        self.poll_port('a')
        self.poll_port('b')
        self.update_estop()
        self.publish_telemetry()
        self.publish_status()

    def poll_port(self, which):
        ser = self.ser_a if which == 'a' else self.ser_b
        if ser is None:
            return
        buf_attr = 'rx_buf_a' if which == 'a' else 'rx_buf_b'
        buf = getattr(self, buf_attr)

        try:
            data = ser.read(4096)
        except (serial.SerialException, OSError) as e:
            # 포트가 죽었다(USB 분리 등) → 그 보드만 떨어뜨리고 _link_loop 가 재연결한다
            self._drop_board(which, f"수신 실패: {e}")
            return
        if data:
            buf += data
        if b'\n' not in buf:
            setattr(self, buf_attr, buf)
            return

        lines = buf.split(b'\n')
        setattr(self, buf_attr, lines[-1])          # 미완성 줄은 버퍼에 보존
        texts = [t for t in
                 (line.decode('ascii', errors='ignore').strip() for line in lines[:-1])
                 if t]
        if not texts:
            return

        # ★ 진단 줄('#'로 시작, kasa_0804_B.ino DEBUG_LINEAR)은 걸러낸다 ★
        #   운용 시엔 DEBUG_LINEAR=false 라 나오지 않지만, 표 보정 중에 켜 두면
        #   그 줄이 최신 줄로 잡혀 텔레메트리를 덮어버린다.
        texts = [t for t in texts if not t.startswith('#')]
        if not texts:
            return

        latest = texts[-1]                          # 가장 최신 완성 줄 하나만 사용
        if which == 'a':
            self.last_line_a = latest
            self.parse_a(latest)
        else:
            self.last_line_b = latest
            self.parse_b(latest)

    def parse_a(self, text):
        """"S,<왼쪽펄스>,<오른쪽펄스>,<쓰로틀raw>" (kasa_0730_A.ino).
           STOP/형식오류 시 마지막 값 유지. 쓰로틀 필드가 없는 구버전(3필드)도 받아준다."""
        if not text.startswith('S,'):
            return
        fields = text.split(',')
        if len(fields) not in (3, 4):
            return
        try:
            self.pulse_l = int(fields[1])
            self.pulse_r = int(fields[2])
            if len(fields) == 4:
                self.throttle_raw = int(fields[3])
        except ValueError:
            pass

    def parse_b(self, text):
        """"P,<조향각>,<모드>" (kasa_0804_B.ino). 조향각 부호는 ★− 좌 / + 우★ 로
           ROS 규약과 같다(그대로 발행한다).
           STOP/형식오류 시 마지막 값 유지. 모드 필드가 없는 구버전(2필드)도 받아준다."""
        if not text.startswith('P,'):
            return
        fields = text.split(',')
        if len(fields) not in (2, 3):
            return
        try:
            self.angle_board = int(fields[1])
            if len(fields) == 3:
                # ★값 규약: 1 = 자율주행 / 0 = 수동조종★
                new_mode = bool(int(fields[2]))
                if new_mode != self.switch_mode:
                    self.switch_mode = new_mode
                    self.get_logger().info(
                        f"[주행모드 전환] {'자율주행' if new_mode else '수동조종'} (B보드 D5)")
        except ValueError:
            pass

    def update_estop(self):
        """A·B 중 한쪽이라도 최신 줄이 STOP 이면 e-stop (OR 판정). 전환 시점만 로그."""
        active = (self.last_line_a == 'STOP') or (self.last_line_b == 'STOP')
        if active and not self.estop_active:
            self.get_logger().warn(
                "[E-STOP 발동] A/B 보드 중 하나 이상이 STOP 신호를 보냄 "
                "(B보드가 리니어 2단 체결, 인휠 정지)")
        elif not active and self.estop_active:
            self.get_logger().info("[E-STOP 해제] 정상 텔레메트리 재개 (리니어 0단 복귀)")
        self.estop_active = active

    def publish_telemetry(self):
        """white 규약 토픽으로 보드 상태를 발행.

        ★ /encoder = 좌 + 우 (합) ★ 규약 3 참고. 소비측(white)의 TICKS_PER_REV=192 와
        짝을 이뤄야 m/s 가 맞는다 — 한쪽만 바꾸면 속도가 2배/절반으로 조용히 어긋난다.
        후진이 없으므로 음수는 나오지 않는다(펌웨어가 부호 없는 카운트를 보낸다)."""
        enc = Int32()
        enc.data = max(0, int(self.pulse_l) + int(self.pulse_r))
        self.pub_encoder.publish(enc)

        # 실측 조향각. ★기본은 보드 값 그대로★ (ROS 와 보드가 같은 규약 − 좌 / + 우)
        #   steer_invert=True 일 때만 뒤집어, 명령과 실측이 항상 같은 부호로 보이게 한다.
        ang = Int32()
        ang.data = -int(self.angle_board) if self.steer_invert else int(self.angle_board)
        self.pub_steer_angle.publish(ang)

        self.pub_mode.publish(Bool(data=bool(self.auto_mode)))
        self.pub_throttle.publish(Int32(data=int(self.throttle_raw)))
        self.pub_estop.publish(Bool(data=bool(self.estop_active)))

    def publish_status(self):
        """"A:0|1,B:0|1,ESTOP:0|1,MODE:0|1[,SRC:sw|ovr,SW:0|1]" — 진단·로스백용.
           보드 탐색 중에도 발행하므로 '연결 중'인지 바로 알 수 있다.

           MODE 는 ★실효 모드★ 다. 소프트웨어 오버라이드가 걸려 있으면 SRC:ovr 와 함께
           물리 스위치 원값(SW)도 붙인다 — 오버라이드가 조용히 숨어 있으면 "스위치를
           돌렸는데 왜 안 바뀌지"로 되돌아온다."""
        msg = String()
        # [2026-08-07] SRC:ovr / SW: 필드가 사라졌다 — 오버라이드가 없으니 MODE 가
        #   곧 물리 스위치 값이고, 둘이 어긋날 방법이 없다.
        msg.data = (f"A:{1 if self.ser_a else 0},B:{1 if self.ser_b else 0},"
                    f"ESTOP:{1 if self.estop_active else 0},"
                    f"MODE:{1 if self.auto_mode else 0}")
        self.pub_status.publish(msg)

    # ═══════════════════════════════════════════════════════════════
    #  종료 정리 (정상 종료 / 부모 사망 공용)
    # ═══════════════════════════════════════════════════════════════
    def stop_and_close(self):
        """차를 세우는 정지값을 두 보드에 직접 쓰고 포트를 닫는다.

        ★ A보드 펌웨어에는 무입력 타임아웃이 없다 ★ (0713에서 제거) 마지막 수신 명령을
        계속 물고 있으므로, 이 프로세스가 끝나기 전에 정지값이 반드시 시리얼까지 나가야
        한다. 종료 직전에는 타이머 콜백 경로를 믿을 수 없으므로 여기서 직접 쓴다.

        조향은 0 이 아니라 'x'(힘빼기)를 보낸다 — 수동조종 중이었다면 사람이 핸들을 잡고
        있어서 0도로 급조향하면 위험하다. 브레이크도 0 으로 둔다(리니어가 페달을 물고
        있으면 사람이 차를 움직일 수 없다).

        정상 종료(destroy_node)와 부모 사망 감지(proc_guard) 양쪽에서 호출되며, 두 번
        불려도 안전하다(닫힌 포트는 건너뜀)."""
        # 재연결 스레드를 먼저 멈춘다 — 아래에서 닫은 포트를 그 스레드가 다시 열면
        # 정지값을 보낸 의미가 없어지고 포트가 물린 채로 프로세스가 끝난다.
        self._running = False
        wrote = False
        for ser, text in ((self.ser_a, '0'), (self.ser_b, f'{STEER_RELEASE_TOKEN},0')):
            if ser is None:
                continue
            try:
                if not ser.is_open:
                    continue
                ser.write((text + '\n').encode('ascii'))
                ser.flush()
                wrote = True
            except (serial.SerialException, OSError):
                pass
        if wrote:
            time.sleep(STOP_FLUSH_S)   # 정지값이 실제로 나갈 시간을 준 뒤 닫는다

        for ser in (self.ser_a, self.ser_b):
            try:
                if ser is not None and ser.is_open:
                    ser.close()
            except (serial.SerialException, OSError):
                pass

    def destroy_node(self):
        self.stop_and_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = Arduino()
    except KeyboardInterrupt:
        # 보드 탐색 중 Ctrl+C 면 rclpy 의 SIGINT 핸들러가 이미 컨텍스트를 내린 뒤일 수
        # 있다. 그 상태에서 또 shutdown 하면 RCLError 로 exit 1 이 되므로 가드한다.
        if rclpy.ok():
            rclpy.shutdown()
        return
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
