#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
traffic_light.py ― 신호등 인지·정지 [white1]
════════════════════════════════════════════════════════════════════════════════
하는 일은 하나뿐이다 — ★카메라로 신호등을 보고, 빨간불이 확정되면 차를 세운다★.
그리고 ★빨간불이 1초 이상 안 보이거나 초록불이 확정되면 놓는다★ (white806 판과
다른 점 — 아래 '정지 래치' 절).

두 곳에서 쓴다. 어느 쪽이든 이 노드가 하는 일은 /brake_level 하나뿐이다:
  · 자율주행 : driving 이 경로추종 중(/drive_state=DRIVE_RUN)일 때 자동으로 개입한다.
               해제되면 driving 이 20Hz 로 계속 내던 목표펄스가 그대로 다시 통해
               ★스스로 재출발★ 한다(이 노드는 펄스를 기억하지도, 복원하지도 않는다).
  · 수동 조종 : nxde master 의 ★'신호등 인지' 체크박스★ 가 /tl_enable=True 를 낼 때만
               개입한다. 해제되면 arduino 가 캐시하고 있던 master 의 레버값이 그대로
               되살아난다 — ★E-STOP 이 풀릴 때와 같은 성질★ 이고, 그것을 얻으려고
               이 노드는 /cmd_vel_raw 를 ★내지 않는다★(publish_cmd_vel 기본 false).

구 white(white_cam_ws) 에서 신호등에 해당하는 부분만 떼어 왔다:
  · 인지 = perception.py 의 신호등 파이프라인
      YOLO 박스 → 크기·종횡비·conf 필터 → HSV 색 교정 → RED 적색픽셀 관문
      → RED / RED_FAR / GREEN / UNKNOWN
  · 판단 = camera_judgment.py 의 RED 확정 필터
      tl_hold_s 연속 목격 + tl_gap_grace_s 이내의 끊김은 봐주기(깜빡임·오검출 흡수)

★가져오지 않은 것★ 차선(BEV·슬라이딩윈도우·/lane_metrics)·정지선·횡단보도.
  white1 에는 그 발행자도 소비자도 없다. 그래서 정지 판정은 구 white 의
  tl_nodist_stop=True 경로 하나뿐이다 — ★RED 확정이면 정지선을 못 잡아도 선다★.
  (실측에서 stop_line_dist 는 100% 미검출이었고, 그 값을 기다리면 영영 안 섰다.)

════════════════════════════════════════════════════════════════════════════════
 발행 / 구독
════════════════════════════════════════════════════════════════════════════════
발행:
  /brake_level  std_msgs/Int32       ★리니어 2단 — 정지의 본체다(아래 참고)★
  /cmd_vel_raw  geometry_msgs/Twist  ★기본으로는 내지 않는다★ (publish_cmd_vel:=true 일 때만)
  /tl/state     std_msgs/String      RED / RED_FAR / GREEN / UNKNOWN (기록·디버그·master 표시)
구독:
  /image_raw    sensor_msgs/Image    usb_cam
  /drive_state  std_msgs/String      ← driving.py (DRIVE_DONE 이면 브레이크 소유권 양보)
  /tl_enable    std_msgs/Bool        ← nxde master 의 '신호등 인지' 체크박스(상시)
  /tl_permit    std_msgs/Bool        ← driving.py 의 허락(상수 + DRIVE_RUN)

════════════════════════════════════════════════════════════════════════════════
 ★왜 /brake_level 하나가 정지의 본체인가★
════════════════════════════════════════════════════════════════════════════════
  nxde/arduino.py compose() 의 (4) 정상 자율주행 분기에 이 줄이 있다:

      pulse = 0 if brake > 0 else self.cmd_pulse

  브레이크가 0 이 아니면 A보드로 나가는 목표펄스가 ★강제로 0★ 이 된다. 즉
  /brake_level=2 를 내는 것만으로 ★리니어 2단 체결 + 인휠 구동 차단★ 이 동시에
  성립한다. 그 줄의 주석이 말하는 "주행 중에 브레이크를 요청하는 다른 발행자"가
  바로 이 노드다(구 white 에서는 camera_judgment 였다).

 ★왜 /cmd_vel_raw 를 내지 않는가 [white1 에서 기본값을 뒤집었다]★
  펄스는 위에서 이미 0 이 되므로 이 토픽이 실제로 할 일은 ★조향각 0(일직선)★ 뿐인데,
  그 대가가 크다:
    ① arduino 의 명령 캐시(cmd_pulse·cmd_angle)를 0 으로 덮어쓴다. 그러면 브레이크를
       푸는 순간 되살아날 값이 ★0★ 이 되어, master 로 몰던 사람은 레버를 그대로 두고도
       차가 안 나가게 된다. ★E-STOP 이 풀릴 때처럼 '있던 명령이 그대로 돌아오는' 성질★
       을 원하므로 캐시를 건드리지 않는다.
    ② /cmd_vel_raw 의 주인은 driving(20Hz)·master 다. 발행자가 겹치면 캐시가 번갈아
       덮여 '누가 지금 명령의 주인인가'를 추적할 수 없게 된다.
  → 그래서 이 노드는 ★/brake_level 하나만★ 낸다. 정지 성립에 그것으로 충분하다(위 절).
    정지 중 바퀴를 일직선으로 두고 싶으면 publish_cmd_vel:=true 로 켠다 — 그때는 위
    ①②를 감수하는 것이다(자율주행 전용으로만 쓸 것을 권한다).

════════════════════════════════════════════════════════════════════════════════
 안전 규약 — 이 노드가 지키는 세 가지
════════════════════════════════════════════════════════════════════════════════
 ① fail-open : 모델을 못 읽었거나(가중치 경로 오류) 영상이 끊기면 ★아무 개입도 하지
    않는다★. 신선하고 확정된 RED 가 있을 때만 브레이크를 건다. 카메라가 죽었다고
    차가 영영 못 가는 일은 없어야 한다.
 ② ★허락받은 구간에서만 개입★ 둘 중 하나가 성립해야 브레이크를 건다:
      · /tl_enable == True   master 의 '신호등 인지' 체크박스 — ★체크되어 있으면 상시★
      · /tl_permit == True   driving 의 TRAFFIC_LIGHT_ENABLE(코드 내부 상수)이면서
                             ★DRIVE_RUN 일 때만★. 매핑(MAP_*)에서는 절대 안 온다 —
                             사람이 페달로 모는 구간에 리니어가 끼어들면 안 된다.
                             그 상수를 False 로 두면 ★카메라 없이 종전대로 돈다★.
    아무 때나 리니어가 튀어나오는 것은 nxde/arduino.py 가 2026-08-04 부터 지켜 온
    불변식("모드 전환은 절대로 리니어를 체결하지 않는다")을 밖에서 깨는 짓이다.
    ★체크박스는 사람이 직접 켠 것이므로 '허락'이다★ — 스스로 켜지지 않는다.
    체크를 끄면 그 순간 걸어 둔 것을 풀고 손을 뗀다. 벤치에서 판정만 보고 싶으면
    require_permission:=false (그때도 브레이크는 걸리니 차를 세워 두고 할 것).
    ⚠️ 수동조종 모드(D5 내림)에서는 arduino 가 브레이크를 항상 0 으로 보내므로
      체크를 켜도 리니어는 물리지 않는다 — 그 불변식은 이 노드가 깨지 않는다.
 ③ 남의 브레이크는 풀지 않는다 : 0단은 ★이 노드가 직접 2단을 걸었을 때만★ 낸다.
    driving 이 DRIVE_DONE(도착·경로이탈)으로 스스로 2단을 물고 있는 구간에서는
    해제를 아예 발행하지 않는다 — 그 구간의 해제는 driving 의 몫이다.

 ★정지 래치(stop_latch) — white1 은 기본 OFF 다★
  white806 판은 기본 ON 이었다. 근거는 이랬다: 신호등에 다가갈수록 등기구가 화면 위로
  올라가 tl_roi 를 벗어나므로, 정지선 앞에 서면 신호등이 시야에서 사라지는 것이
  정상이고 → 그 순간 UNKNOWN → 해제 → 차가 다시 굴러간다.
  ★white1 은 '빨간불을 보는 동안만 잡는다'를 택했다★ (지시사항):
      RED 확정(tl_hold_s 0.4s)      → 2단  ★즉시★
      RED 를 red_release_hold_s(1.0s) 동안 못 봄, 또는 GREEN 확정 → 0단
  ★[2026-08-14] 놓는 쪽에만 1초 유예를 뒀다 — 실차에서 리니어가 들락날락했다★
  종전에는 스트릭이 끊기는 순간(0.3s) 바로 놓았다. 인지가 한두 번 흔들리면
      RED 확정 → 0.3초 놓침 → 해제 → 다시 0.4초 연속 → 체결 …
  이 1초 주기로 반복된다. ★리니어는 물리적으로 왕복하는 장치라 이 왕복이 제일 나쁘다★
  (B보드 1회 구동 최대 1초). 무는 쪽은 그대로 두어 반응이 늦어지지 않게 했다.
  ★소비자 쪽에도 같은 유예가 한 겹 더 있다★ master·driving 이 /tl_brake_req 를 받아
  자기 값과 max 로 합치는데, 요구가 0 이 되어도 1초는 유지한다(TL_REQ_RELEASE_HOLD_S)
  — 토픽이 잠깐 밀리는 경우까지 함께 막는다.
    · tl_gap_grace_s(0.3s) 는 그대로다 — 그것은 '연속 목격 스트릭'의 유예이고,
      해제 유예(위)와는 다른 값이다.
    · ⚠️ 신호등이 시야를 완전히 벗어나면 1초 뒤 차는 다시 굴러간다. 정지선에 바짝
      붙기 전에 서도록 근접도 게이트(tl_red_stop_min_height)를 크게 잡을 것.
  래치가 필요하면 stop_latch:=true — 그때는 GREEN 을 봐야만 놓는다.
"""

import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Int32, String
from cv_bridge import CvBridge
from ultralytics import YOLO

# HSV 가 YOLO 라벨을 ★위험한 방향(RED→GREEN)★ 으로 뒤집는 것을 허용하는 conf 상한.
# 이보다 자신 있는 박스는 색 몇 픽셀로 뒤집지 않는다. (perception.py 와 같은 값)
HSV_FALLBACK_CONF = 0.55

# driving.py 의 상태 문자열. 값이 바뀌면 여기도 바꿔야 한다(driving.py S_* 상수).
DRIVE_RUN_STATE  = 'DRIVE_RUN'
DRIVE_DONE_STATE = 'DRIVE_DONE'
DRIVE_STATE_STALE_S = 2.0    # 이보다 오래된 /drive_state 는 '모른다'로 친다

# ★신호등 가중치 — 경로를 하드코딩한다 [2026-08-14 지시]★
#   .engine(TensorRT)은 ★빌드한 GPU·드라이버에 묶인다★ — 다른 기계로 옮기면 로드가
#   실패하고, 그때 이 노드는 fail-open 으로 아무 개입도 하지 않는다(안전한 쪽).
#   그 기계에서 다시 export 하거나 .pt 를 tl_weights 파라미터로 주면 된다.
TL_WEIGHTS = '/home/mad2/runs2/runs/detect/combined_light/weights/best.engine'

# master 의 '신호등 인지' 체크박스가 이 주기보다 오래 끊기면 '허락 없음'으로 본다.
#   (master 창이 죽었는데 체크가 켜진 채로 굳어 있는 상태를 막는다)
TL_ENABLE_STALE_S = 2.0

# 물고 있는 동안 브레이크를 다시 주장하는 주기 [s]. master 의 KEEPALIVE_S(0.5)보다
# 짧아야 한다 — 그래야 남이 덮어도 곧바로 되돌아온다(_apply_brake 주석 참고).
BRAKE_KEEPALIVE_S = 0.25


class TrafficLight(Node):
    def __init__(self):
        super().__init__('traffic_light')

        # ── 입력 ───────────────────────────────────────────────────────────
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('device',      'cuda:0')   # GPU 없으면 'cpu'

        # ── 인지(perception.py 신호등 절에서 그대로 이식) ───────────────────
        #   ★가중치 경로★ 구 white 기본값(/home/mad2/...)은 이 PC에 없다.
        #   .engine(TensorRT)은 빌드한 GPU에 묶이므로, 다른 기계면 .pt 를 주거나
        #   그 기계에서 다시 export 할 것.
        self.declare_parameter('tl_weights', TL_WEIGHTS)
        self.declare_parameter('tl_conf',      0.35)
        self.declare_parameter('tl_imgsz',     640)
        # 몇 프레임에 한 번 추론할 것인가. ★1 을 유지할 것★ — 2 로 올리면 판정 기회가
        # 절반이 되어 tl_hold_s(연속 RED)를 채우지 못하고 신호등에서 안 선다.
        self.declare_parameter('tl_interval',  1)
        # 신호등 탐색 ROI(원본 픽셀). 기본은 1920x1080 카메라의 위쪽 절반.
        # 프레임이 더 작으면 _clamp_roi 가 알아서 프레임 크기로 잘라 준다.
        self.declare_parameter('tl_roi_xmin',  0)
        self.declare_parameter('tl_roi_ymin',  0)
        self.declare_parameter('tl_roi_xmax',  1920)
        self.declare_parameter('tl_roi_ymax',  540)
        self.declare_parameter('tl_min_area',  20)
        # 상한은 사실상 해제 상태다 — 근접하면 박스가 커지는데 상한에 걸리면
        # ★정지해야 할 바로 그 순간 신호등이 UNKNOWN 으로 사라진다★.
        self.declare_parameter('tl_max_area',  1000000)
        self.declare_parameter('tl_min_aspect', 0.2)
        # 가로형 4구 신호등은 bw/bh ≈ 3.5~4.5 이고, ROI 상단에 잘리면 더 커진다.
        self.declare_parameter('tl_max_aspect', 6.0)
        # RED(정지 대상) 와 RED_FAR(아직 멀다) 를 가르는 근접도 게이트.
        #   ▸ tl_red_stop_min_area_frac > 0 → 박스 면적비 기준(해상도·화각 독립)
        #   ▸ 0(기본)                      → 박스 높이(px) 기준
        # 면적비는 항상 계산·표시되므로 실주행 화면으로 값을 정한 뒤 파라미터만 주면
        # 코드 수정 없이 기준이 바뀐다. 정하는 법 — 원거리 오탐의 최댓값과 실제 정지
        # 지점의 최솟값을 읽어 그 기하평균.
        self.declare_parameter('tl_red_stop_min_height',    25)
        self.declare_parameter('tl_red_stop_min_area_frac', 0.0)
        self.declare_parameter('use_hsv_refine',      True)
        # RED 박스는 HSV 적색 픽셀 확인을 통과해야 채택한다(오탐 관문).
        # 붉지 않은 것이 RED 로 나가면 도로 한복판에서 리니어 2단이 물린다.
        # ⚠️ 이 관문의 실패 모드는 '진짜 RED 를 버리는 것'이다 — 현장에서 RED 가 안
        #    잡히면 이 값을 false 로 내려 먼저 격리하고 로그의 drop 수를 볼 것.
        self.declare_parameter('tl_red_require_hsv',  True)
        self.declare_parameter('hsv_min_color_pixels', 15)
        self.declare_parameter('hsv_crop_center_ratio', 0.85)
        self.declare_parameter('hsv_red_h1_low',   0)
        self.declare_parameter('hsv_red_h1_high',  10)
        self.declare_parameter('hsv_red_h2_low',   170)
        self.declare_parameter('hsv_red_h2_high',  180)
        self.declare_parameter('hsv_green_h_low',  45)
        self.declare_parameter('hsv_green_h_high', 90)
        # 실측 기반 하향값(80/70 → 55/60). 실제 신호등 크롭에서 관찰된 최저가
        # S=62, V=66 이었다 — 그보다 낮게 잡아야 RED 가 검출된다.
        self.declare_parameter('hsv_sat_low',      55)
        self.declare_parameter('hsv_val_low',      60)

        # ── 판단(camera_judgment.py RED 확정 필터에서 이식) ───ㄴ──────────────
        self.declare_parameter('tl_hold_s',        0.4)   # 이만큼 연속으로 봐야 확정
        self.declare_parameter('tl_gap_grace_s',   0.3)   # 이 이내의 끊김은 봐준다
        self.declare_parameter('tl_state_max_age', 3.0)   # 이보다 낡은 판정은 무시
        self.declare_parameter('green_hold_s',     0.4)   # 재출발용 GREEN 확정 시간
        # ★해제 유예 [2026-08-14]★ 빨간불을 이만큼 못 봐야 놓는다. 무는 쪽(tl_hold_s)은
        #   그대로 두고 놓는 쪽만 늦춘 것이다 — 인지가 한두 번 흔들릴 때 리니어가
        #   들락날락하던 실차 증상을 막는다(_red_gone 주석에 근거).
        #   ★[2026-08-14 조정] 1.0 → 0.5★ 체감 해제가 너무 늦었다. 전체 해제 지연은
        #   ★여기 0.5초 + arduino 의 마지막 유예 0.5초 = 1.0초★ 로 맞춘다.
        #   (그 배분을 왜 이렇게 나눴는지는 arduino.BRAKE_RELEASE_HOLD_S 주석에 있다)
        self.declare_parameter('red_release_hold_s', 0.5)

        # ── 정지 동작 ──────────────────────────────────────────────────────
        self.declare_parameter('brake_level',         2)     # 0 놓음 / 1 약 / 2 풀
        self.declare_parameter('brake_release_level', 0)
        # ★[2026-08-14] publish_cmd_vel 은 true 로 둔다(지시)★ 정지 중 펄스 0·조향 0 을
        #   함께 낸다. 캐시를 덮는 부작용은 남지만, master 도 driving 도 자기 명령을
        #   주기적으로 재발행하므로(master KEEPALIVE_S=0.5s / driving 20Hz) 해제 뒤
        #   ★0.5초 안에 원래 명령값이 되돌아온다★ — 실질 손해가 작다.
        #   조향 다툼이 싫으면 false 로 끄면 되고, 그때도 정지는 그대로 성립한다.
        self.declare_parameter('publish_cmd_vel',     True)
        self.declare_parameter('stop_cmd_hz',         30.0)  # 판단 틱 주기
        self.declare_parameter('stop_latch',          False)
        # 개입 허락 : /drive_state==DRIVE_RUN 또는 /tl_enable(master 체크박스)
        self.declare_parameter('require_permission',  True)

        # ── 표시 ───────────────────────────────────────────────────────────
        #  ★창은 기본으로 띄운다 [2026-08-14]★ 신호등 인지는 '지금 뭘 보고 있나'를
        #  눈으로 확인하지 않으면 튜닝이 불가능하다(ROI·근접도·색 임계). 화면 없는
        #  터미널(ssh)에서는 cv2 가 창을 못 열므로 show_window:=false 로 끈다.
        self.declare_parameter('show_window', True)
        self.declare_parameter('draw_roi',    True)
        #  ★창 가로폭 [2026-08-14]★ 원본(1920)을 그대로 띄우면 화면을 덮는다. 이 폭으로
        #  줄여서 띄운다(비율 유지). 0 이면 원본 크기. 판정은 원본 해상도로 하므로
        #  이 값을 줄여도 인지 성능에는 영향이 없다 — 보이는 크기만 달라진다.
        self.declare_parameter('window_width', 640)

        g = lambda k: self.get_parameter(k).value
        self.image_topic = str(g('image_topic'))
        self.device      = str(g('device'))
        self.tl_conf     = float(g('tl_conf'))
        self.tl_imgsz    = int(g('tl_imgsz'))
        self.tl_interval = max(1, int(g('tl_interval')))
        self.tl_roi = (int(g('tl_roi_xmin')), int(g('tl_roi_ymin')),
                       int(g('tl_roi_xmax')), int(g('tl_roi_ymax')))
        self.tl_min_area   = int(g('tl_min_area'))
        self.tl_max_area   = int(g('tl_max_area'))
        self.tl_min_aspect = float(g('tl_min_aspect'))
        self.tl_max_aspect = float(g('tl_max_aspect'))
        self.tl_red_stop_min_height    = int(g('tl_red_stop_min_height'))
        self.tl_red_stop_min_area_frac = float(g('tl_red_stop_min_area_frac'))
        self.use_hsv_refine       = bool(g('use_hsv_refine'))
        self.tl_red_require_hsv   = bool(g('tl_red_require_hsv'))
        self.hsv_min_color_pixels = int(g('hsv_min_color_pixels'))
        self.hsv_crop_center_ratio = float(g('hsv_crop_center_ratio'))
        self.hsv_red_h1_low   = int(g('hsv_red_h1_low'))
        self.hsv_red_h1_high  = int(g('hsv_red_h1_high'))
        self.hsv_red_h2_low   = int(g('hsv_red_h2_low'))
        self.hsv_red_h2_high  = int(g('hsv_red_h2_high'))
        self.hsv_green_h_low  = int(g('hsv_green_h_low'))
        self.hsv_green_h_high = int(g('hsv_green_h_high'))
        self.hsv_sat_low = int(g('hsv_sat_low'))
        self.hsv_val_low = int(g('hsv_val_low'))
        self.tl_hold_s        = float(g('tl_hold_s'))
        self.tl_gap_grace_s   = float(g('tl_gap_grace_s'))
        self.tl_state_max_age = float(g('tl_state_max_age'))
        self.green_hold_s     = float(g('green_hold_s'))
        self.red_release_hold_s = max(0.0, float(g('red_release_hold_s')))
        self.brake_level         = max(0, min(2, int(g('brake_level'))))
        self.brake_release_level = max(0, min(2, int(g('brake_release_level'))))
        self.publish_cmd_vel  = bool(g('publish_cmd_vel'))
        self.stop_cmd_hz      = max(1.0, float(g('stop_cmd_hz')))
        self.stop_latch       = bool(g('stop_latch'))
        self.require_permission = bool(g('require_permission'))
        self.show_window = bool(g('show_window'))
        self.draw_roi    = bool(g('draw_roi'))
        self.window_width = max(0, int(g('window_width')))
        # 창 표시는 ★메인 스레드★ 가 한다(show_pending) — _draw 주석에 근거.
        self._show_lock = threading.Lock()
        self._show_frame = None
        self._window_ready = False

        # ── 상태 ───────────────────────────────────────────────────────────
        self.tl_state   = 'UNKNOWN'   # 마지막 프레임 판정
        self.tl_time    = 0.0         # 그 판정 시각(신선도 판단용)
        self.red_since  = None        # 연속 RED 스트릭의 시작 시각
        self.red_last_seen  = None    # 마지막으로 RED/RED_FAR 를 본 시각(끊김 유예)
        self.green_since    = None
        self.green_last_seen = None
        self.stopping   = False       # 지금 이 노드가 차를 세우고 있는가
        self._brake_t   = 0.0         # 브레이크를 마지막으로 발행한 시각(재확인용)
        # 우리가 마지막으로 ★발행한★ 브레이크 단계. None = 한 번도 건 적 없다.
        #   → 이 값이 None 이면 0단도 내지 않는다(남의 브레이크를 풀지 않기 위해).
        self.brake_now  = None
        self.drive_state   = ''
        self.drive_state_t = 0.0
        self.tl_enable     = False    # master 의 '신호등 인지' 체크박스
        self.tl_enable_t   = 0.0
        self.tl_permit     = False    # driving 의 허락(TRAFFIC_LIGHT_ENABLE + DRIVE_RUN)
        self.tl_permit_t   = 0.0
        self.img_time      = 0.0
        self.frame_count   = 0
        self.last_boxes    = []
        self.last_raw      = 0        # 필터 전 박스 수 (-1 = 추론 예외)
        self.last_red_drop = 0        # HSV 적색 관문에 걸려 버려진 RED 수
        self.fps           = 0.0
        self.fps_t         = time.monotonic()

        # ── 모델 ───────────────────────────────────────────────────────────
        self.model = None
        self.TL_LABEL = {}
        self.TL_ALLOW = set()
        self._load_model(str(g('tl_weights')))

        # ── ROS 인터페이스 ─────────────────────────────────────────────────
        qos_img = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self.bridge    = CvBridge()
        self.pub_brake = self.create_publisher(Int32,  '/brake_level', qos)
        # ★[2026-08-14] 내가 지금 요구하는 브레이크 단계를 따로 알린다★
        #   /brake_level 은 '명령'이라 마지막 발행자가 이긴다. master 는 자기 레버값을
        #   KEEPALIVE_S(0.5s)마다 재발행하므로, 신호등이 2단을 걸어도 0.5초 뒤 0 으로
        #   덮여 리니어가 나왔다 들어간다(실측 로그로 확인). 그래서 '요구'를 별도로
        #   내보내고 master 가 그것을 자기 값과 max 로 합쳐서 낸다 — 두 발행자가
        #   ★같은 값★ 을 내게 되어 다툼이 사라진다.
        self.pub_req   = self.create_publisher(Int32,  '/tl_brake_req', qos)
        self.pub_cmd   = self.create_publisher(Twist,  '/cmd_vel_raw', qos)
        self.pub_state = self.create_publisher(String, '/tl/state',    qos)
        # ★진단 전용 [2026-08-14]★ 제어에는 쓰지 않는다 — record 가 CSV 로 받아 적어
        #   '왜 여기서 섰나 / 왜 멀리서도 섰나' 를 사후에 판정할 수 있게 한다.
        self.pub_near    = self.create_publisher(Float32, '/tl/near_metric', qos)
        self.pub_red_far = self.create_publisher(Bool,    '/tl/red_far',     qos)

        # ★콜백 그룹을 둘로 나눈다 [2026-08-14]★ 영상(추론·그리기)과 제어(브레이크
        #   유지·해제, 요구 발행)를 다른 그룹에 두고 MultiThreadedExecutor 로 돌린다.
        #   한 프레임 처리가 길어져도 제어 틱이 그만큼 밀리지 않는다 — main() 참고.
        self.cg_image = MutuallyExclusiveCallbackGroup()
        self.cg_ctrl  = MutuallyExclusiveCallbackGroup()

        self.create_subscription(Image, self.image_topic, self.cb_image, qos_img,
                                 callback_group=self.cg_image)
        self.create_subscription(String, '/drive_state', self.cb_drive_state, qos,
                                 callback_group=self.cg_ctrl)
        # 허락 두 갈래. 안 오면 False = 허락 없음(fail-safe 방향)
        #   /tl_enable : master 의 '신호등 인지' 체크박스 — ★체크되어 있으면 상시★
        #   /tl_permit : driving 의 TRAFFIC_LIGHT_ENABLE + DRIVE_RUN — ★자율주행 중에만★
        self.create_subscription(Bool, '/tl_enable', self.cb_tl_enable, qos,
                                 callback_group=self.cg_ctrl)
        self.create_subscription(Bool, '/tl_permit', self.cb_tl_permit, qos,
                                 callback_group=self.cg_ctrl)

        # 정지 지령 타이머 — ★영상 콜백이 아니라 여기서 낸다★ 추론이 늦어져
        # 프레임이 띄엄띄엄 들어와도 정지 지령의 주기는 흔들리면 안 된다.
        self.create_timer(1.0 / self.stop_cmd_hz, self.tick,
                          callback_group=self.cg_ctrl)
        self.create_timer(1.0, self.status_tick, callback_group=self.cg_ctrl)

        self.get_logger().info(
            f"🚦 traffic_light | img={self.image_topic} dev={self.device} "
            f"conf={self.tl_conf} interval={self.tl_interval} "
            f"ROI={self.tl_roi} | 근접도 게이트="
            + (f"면적비≥{100.0 * self.tl_red_stop_min_area_frac:.3f}%"
               if self.tl_red_stop_min_area_frac > 0.0
               else f"박스높이≥{self.tl_red_stop_min_height}px")
            + f" | 확정 hold={self.tl_hold_s}s grace={self.tl_gap_grace_s}s "
            f"age≤{self.tl_state_max_age}s | 정지=리니어 {self.brake_level}단"
            + (f" + /cmd_vel_raw 0/0 @{self.stop_cmd_hz:.0f}Hz"
               if self.publish_cmd_vel else " (조향 개입 없음)")
            + f" | latch={'ON(GREEN 까지)' if self.stop_latch else 'OFF(빨간불 동안만)'} "
            + ("허락=/tl_enable(master 체크박스) 또는 /tl_permit(driving DRIVE_RUN)"
               if self.require_permission else "허락=★없음(벤치 모드)★"))

    def _load_model(self, weights):
        """가중치를 읽고 라벨 표를 세운다. 실패해도 노드는 살아 있는다(fail-open).

        ★라벨은 엔진의 클래스 이름에서 유도한다★ 하드코딩({0:GREEN, 1:RED})은
        가중치를 재학습·교체하는 순간 ★조용히 적/녹이 뒤집힌다★ — 그 실패 모드는
        '빨간불에 가속'이다. 아래 로그가 실제 로드된 클래스를 찍으므로 기동할 때
        반드시 눈으로 확인할 것.
        """
        try:
            self.model = YOLO(weights)
        except Exception as e:
            self.model = None
            self.get_logger().error(
                f"⛔ 신호등 가중치 로드 실패 — 이 노드는 아무것도 하지 않는다: {weights} ({e})")
            return

        try:
            names = self.model.names
            self.TL_LABEL = {int(k): str(v).upper() for k, v in
                             (names.items() if isinstance(names, dict) else enumerate(names))}
        except Exception as e:
            self.TL_LABEL = {}
            self.get_logger().error(f"신호등 엔진 클래스 이름 조회 실패: {e}")

        if not {'RED', 'GREEN'} <= set(self.TL_LABEL.values()):
            # 엔진에 메타데이터가 없으면 ultralytics 가 'class0/class1' 을 돌려준다.
            # 그대로 두면 RED/GREEN 이 영영 안 나와 ★신호등이 조용히 무력화★된다.
            self.get_logger().error(
                f"⛔ 엔진 클래스에 RED/GREEN 이 없다: {self.TL_LABEL} → "
                f"옛 하드코딩({{0:GREEN, 1:RED}})으로 폴백한다. "
                f"★클래스 순서가 이와 다르면 적/녹이 뒤집힌다 — 반드시 확인할 것★")
            self.TL_LABEL = {0: 'GREEN', 1: 'RED'}
        # 화이트리스트는 ★RED/GREEN 만★ 이다. set(TL_LABEL) 로 두면 모델의 전 클래스가
        # 통과해 필터가 no-op 이 된다.
        self.TL_ALLOW = {k for k, v in self.TL_LABEL.items() if v in ('RED', 'GREEN')}
        self.get_logger().info(f"신호등 엔진 클래스 = {self.TL_LABEL} (허용 {sorted(self.TL_ALLOW)})")

    # ══════════════════════════════════════════════════════════════════════════
    #  인지 — perception.py 이식
    # ══════════════════════════════════════════════════════════════════════════
    @staticmethod
    def _clamp_roi(w, h, xmin, ymin, xmax, ymax):
        xmin = max(0, min(int(xmin), w - 1))
        xmax = max(0, min(int(xmax), w))
        ymin = max(0, min(int(ymin), h - 1))
        ymax = max(0, min(int(ymax), h))
        if xmax <= xmin or ymax <= ymin:
            return 0, 0, w, h
        return xmin, ymin, xmax, ymax

    @staticmethod
    def _central_crop(img, ratio=0.8):
        if img is None or img.size == 0:
            return img
        h, w = img.shape[:2]
        ratio = max(0.2, min(1.0, ratio))
        nw, nh = int(w * ratio), int(h * ratio)
        x1, y1 = max(0, (w - nw) // 2), max(0, (h - nh) // 2)
        return img[y1:y1 + nh, x1:x1 + nw]

    def _hsv_state(self, crop_bgr):
        """박스 크롭의 적/녹 픽셀 수를 세어 색으로 판정한다."""
        if crop_bgr is None or crop_bgr.size == 0:
            return 'UNKNOWN', 0, 0
        crop = self._central_crop(crop_bgr, self.hsv_crop_center_ratio)
        hsv  = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)

        lo_r1 = np.array([self.hsv_red_h1_low,  self.hsv_sat_low, self.hsv_val_low], dtype=np.uint8)
        hi_r1 = np.array([self.hsv_red_h1_high, 255, 255], dtype=np.uint8)
        lo_r2 = np.array([self.hsv_red_h2_low,  self.hsv_sat_low, self.hsv_val_low], dtype=np.uint8)
        hi_r2 = np.array([self.hsv_red_h2_high, 255, 255], dtype=np.uint8)
        lo_g  = np.array([self.hsv_green_h_low, self.hsv_sat_low, self.hsv_val_low], dtype=np.uint8)
        hi_g  = np.array([self.hsv_green_h_high, 255, 255], dtype=np.uint8)

        mask_r = cv2.bitwise_or(cv2.inRange(hsv, lo_r1, hi_r1),
                                cv2.inRange(hsv, lo_r2, hi_r2))
        mask_g = cv2.inRange(hsv, lo_g, hi_g)

        kernel = np.ones((3, 3), np.uint8)
        mask_r = cv2.morphologyEx(mask_r, cv2.MORPH_OPEN, kernel)
        mask_g = cv2.morphologyEx(mask_g, cv2.MORPH_OPEN, kernel)

        r = cv2.countNonZero(mask_r)
        gr = cv2.countNonZero(mask_g)
        if r > gr and r >= self.hsv_min_color_pixels:
            return 'RED', r, gr
        if gr > r and gr >= self.hsv_min_color_pixels:
            return 'GREEN', r, gr
        return 'UNKNOWN', r, gr

    def _all_tl_boxes(self, result, roi_img, frame_area):
        """YOLO 결과 → 필터링·색보정된 신호등 박스 목록.

        frame_area 는 ★원본 프레임 전체 면적★ 이다(ROI 면적이 아니다) — ROI 를 바꿔도
        면적비(area_frac)의 의미가 흔들리지 않게 한다.
        """
        out = []
        self.last_red_drop = 0
        if result is None or result.boxes is None or len(result.boxes) == 0:
            return out
        for b in result.boxes:
            conf   = float(b.conf[0].item()) if hasattr(b, 'conf') else 0.0
            cls_id = int(b.cls[0].item())    if hasattr(b, 'cls')  else -1
            if cls_id not in self.TL_ALLOW or conf < self.tl_conf:
                continue
            x1, y1, x2, y2 = [int(v) for v in b.xyxy[0].tolist()]
            bw = max(1, x2 - x1)
            bh = max(1, y2 - y1)
            area   = bw * bh
            aspect = bw / float(bh)
            if area   < self.tl_min_area   or area   > self.tl_max_area:   continue
            if aspect < self.tl_min_aspect or aspect > self.tl_max_aspect: continue

            label = self.TL_LABEL.get(cls_id, str(cls_id))
            hsv_red = hsv_green = 0
            if self.use_hsv_refine:
                # HSV 는 conf 와 무관하게 ★항상★ 돌리되, 라벨을 뒤집는 조건은
                # 방향에 따라 다르다 — 신호등에서 오류 비용은 극단적으로 비대칭이다.
                crop = roi_img[max(0, y1):max(0, y2), max(0, x1):max(0, x2)]
                hsv_label, hsv_red, hsv_green = self._hsv_state(crop)
                if label == 'GREEN' and hsv_label == 'RED':
                    label = 'RED'                     # 안전 방향 → 항상 교정
                elif conf < HSV_FALLBACK_CONF and hsv_label in ('RED', 'GREEN'):
                    label = hsv_label                 # 위험 방향 → 저신뢰에서만 위임

            # RED 오탐 관문 — 붉지 않은 것이 RED 로 나가면 도로 한복판에서 선다.
            if (self.use_hsv_refine and self.tl_red_require_hsv
                    and label == 'RED' and hsv_red < self.hsv_min_color_pixels):
                self.last_red_drop += 1
                continue

            out.append({
                'label': label, 'conf': conf, 'box': (x1, y1, x2, y2), 'box_h': bh,
                # 근접도 지표. 해상도·화각에 묶이지 않아 box_h(px)보다 이식성이 좋다.
                'area_frac': area / float(frame_area) if frame_area else 0.0,
                'hsv_red': hsv_red, 'hsv_green': hsv_green,
            })
        return out

    def _near_metric(self, boxes):
        """지금 근접도 게이트에 걸리는 값 — ★빨간 박스 중 최대★. 빨간 박스가 없으면 0.

        단위는 ★쓰이고 있는 게이트의 단위★ 다:
          · tl_red_stop_min_area_frac > 0 → 면적비(0~1, 화면 대비)
          · 그 외(기본)                   → 박스 높이[px]
        임계와 이 값을 나란히 보면 RED / RED_FAR 가 갈린 이유가 그대로 드러난다.
        ★제어에는 쓰지 않는다★ — 판정은 _resolve_tl_state 가 이미 했고, 이건 기록용이다.
        """
        red = [b for b in boxes if b['label'] == 'RED']
        if not red:
            return 0.0
        if self.tl_red_stop_min_area_frac > 0.0:
            return max(b.get('area_frac', 0.0) for b in red)
        return float(max(b.get('box_h', 0) for b in red))

    def _resolve_tl_state(self, boxes):
        if not boxes:
            return 'UNKNOWN'
        red_boxes = [b for b in boxes if b['label'] == 'RED']
        if red_boxes:
            if self.tl_red_stop_min_area_frac > 0.0:
                near = any(b.get('area_frac', 0.0) >= self.tl_red_stop_min_area_frac
                           for b in red_boxes)
            else:
                near = any(b.get('box_h', 0) >= self.tl_red_stop_min_height
                           for b in red_boxes)
            return 'RED' if near else 'RED_FAR'
        if any(b['label'] == 'GREEN' for b in boxes):
            return 'GREEN'
        return 'UNKNOWN'

    def cb_image(self, msg: Image):
        if self.model is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}", throttle_duration_sec=5.0)
            return

        self.frame_count += 1
        run = (self.frame_count % self.tl_interval == 0)
        h, w = frame.shape[:2]
        xmin, ymin, xmax, ymax = self._clamp_roi(w, h, *self.tl_roi)
        roi_img = frame[ymin:ymax, xmin:xmax]

        boxes = []
        if roi_img.size != 0:
            if run:
                try:
                    res = self.model.predict(source=roi_img.copy(), conf=self.tl_conf,
                                             imgsz=self.tl_imgsz, device=self.device,
                                             verbose=False)[0]
                    boxes = self._all_tl_boxes(res, roi_img, w * h)
                    self.last_boxes = boxes
                    # "YOLO 가 못 봤다" vs "필터가 먹었다" 를 사후에 가르기 위한 값.
                    self.last_raw = 0 if res.boxes is None else len(res.boxes)
                except Exception as e:
                    self.get_logger().error(f"YOLO TL error: {e}", throttle_duration_sec=5.0)
                    self.last_boxes = []
                    self.last_raw = -1
                    self.last_red_drop = 0
            else:
                boxes = self.last_boxes

        state = self._resolve_tl_state(boxes)
        self.pub_state.publish(String(data=state))
        # ★[2026-08-14] '신호등을 어떻게 보고 있나' 를 기록으로 남긴다★
        #   near_metric : 지금 근접도 게이트에 걸리는 값(빨간 박스 중 최대). 임계와
        #                 나란히 보면 RED / RED_FAR 가 갈린 이유가 그대로 드러난다.
        #   red_far     : 이번 프레임 판정이 RED_FAR 인가 = '빨갛지만 아직 멀다'.
        #   둘 다 record 가 CSV 로 받아 적는다(white1/record.py).
        self.pub_near.publish(Float32(data=float(self._near_metric(boxes))))
        self.pub_red_far.publish(Bool(data=(state == 'RED_FAR')))
        self._feed_state(state)
        self.img_time = time.time()

        if run:
            now = time.monotonic()
            inst = 1.0 / max(1e-6, now - self.fps_t)
            self.fps = 0.9 * self.fps + 0.1 * inst if self.fps > 0 else inst
            self.fps_t = now

        if self.show_window:
            self._draw(frame, boxes, state, (xmin, ymin, xmax, ymax))

    def _draw(self, frame, boxes, state, roi):
        xmin, ymin, xmax, ymax = roi
        dbg = frame.copy()
        if self.draw_roi:
            cv2.rectangle(dbg, (xmin, ymin), (xmax, ymax), (0, 255, 255), 2)
        for it in boxes:
            x1, y1, x2, y2 = it['box']
            color = ((0, 0, 255) if it['label'] == 'RED'
                     else (0, 255, 0) if it['label'] == 'GREEN' else (150, 150, 150))
            cv2.rectangle(dbg, (x1 + xmin, y1 + ymin), (x2 + xmin, y2 + ymin), color, 2)
            cv2.putText(dbg, f"{it['label']} {it['conf']:.2f} "
                             f"{100.0 * it.get('area_frac', 0.0):.3f}%",
                        (x1 + xmin, max(0, y1 + ymin - 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        # HUD 는 ROI 밖(하단)에 그린다 — 좌상단은 tl_roi 한복판이라 글자가 램프를 덮는다.
        hud_y = dbg.shape[0] - 15
        res_color = ((0, 0, 255) if 'RED' in state
                     else (0, 255, 0) if state == 'GREEN' else (150, 150, 150))
        thr = (f"area>={100.0 * self.tl_red_stop_min_area_frac:.3f}%"
               if self.tl_red_stop_min_area_frac > 0.0
               else f"h>={self.tl_red_stop_min_height}px")
        cv2.putText(dbg, f"FPS: {self.fps:.1f}", (20, hud_y - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(dbg, f"STATE: {state}  raw:{self.last_raw}  drop:{self.last_red_drop}  "
                         f"{thr}  {'STOPPING' if self.stopping else ''}",
                    (20, hud_y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, res_color, 2)
        # ★여기서 imshow 를 부르지 않는다 [2026-08-14]★ 이 함수는 영상 콜백(워커
        #   스레드)에서 돈다. OpenCV HighGUI(GTK)는 ★스레드 안전하지 않고★, 워커
        #   스레드에서 imshow/waitKey 를 부르면 그 스레드가 GTK 안에서 멎는다 —
        #   그러면 영상 콜백이 다시 돌지 못해 ★'/image_raw 두절'★ 이 된다(실측:
        #   단일 스레드 판은 멀쩡, MultiThreadedExecutor 로 바꾼 판은 0.9초 만에 두절).
        #   그래서 그린 프레임만 넘겨 두고, 실제 표시는 ★메인 스레드★ 가 한다(main).
        #   ※ 표시용으로 미리 줄여서 넘긴다 — 원본 1920x1080 을 그대로 띄우면 창이
        #     화면을 덮고, 복사·렌더 비용도 그만큼 크다(구 white 는 WINDOW_NORMAL 로
        #     띄웠는데, 우리는 아예 작게 만들어 넘긴다).
        if self.window_width > 0 and dbg.shape[1] > self.window_width:
            s = self.window_width / float(dbg.shape[1])
            dbg = cv2.resize(dbg, (self.window_width, int(round(dbg.shape[0] * s))),
                             interpolation=cv2.INTER_AREA)
        with self._show_lock:
            self._show_frame = dbg

    def show_pending(self):
        """★메인 스레드에서만 부른다★ 마지막으로 그려 둔 프레임을 창에 띄운다."""
        if not self.show_window:
            return
        with self._show_lock:
            frame, self._show_frame = self._show_frame, None
        if frame is None:
            return
        if not self._window_ready:
            # 구 white/perception.py 와 같은 방식 — 크기를 사람이 바꿀 수 있게 두고,
            # 처음 크기만 정해 준다(WINDOW_AUTOSIZE 면 원본 크기로 고정되어 거대해진다).
            cv2.namedWindow('Traffic Light', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Traffic Light', frame.shape[1], frame.shape[0])
            self._window_ready = True
        cv2.imshow('Traffic Light', frame)
        cv2.waitKey(1)

    # ══════════════════════════════════════════════════════════════════════════
    #  판단 — camera_judgment.py 의 확정 필터 이식
    # ══════════════════════════════════════════════════════════════════════════
    def _feed_state(self, state):
        """프레임 판정 하나를 스트릭에 먹인다.

        ★끊김 유예가 이 필터의 핵심이다★ 실측 rosbag 의 패턴은
        RED,RED,UNKNOWN,UNKNOWN,RED,RED,RED,RED,UNKNOWN (프레임간 0.04~0.15s) 였다.
        RED 아닌 프레임 하나에 스트릭을 리셋하면 빨간불을 6프레임이나 정확히 보고도
        tl_hold_s(0.4s) 연속을 못 채운다 → 신호등에서 안 선다. 그래서 마지막 목격
        시각으로부터 tl_gap_grace_s 를 넘겨서야 스트릭을 죽인다.
        """
        now = time.time()
        if state in ('RED', 'RED_FAR'):
            if self.red_since is None:
                self.red_since = now
            self.red_last_seen = now
        elif (self.red_last_seen is None
              or (now - self.red_last_seen) > self.tl_gap_grace_s):
            # ★스트릭(red_since)만 죽인다 [2026-08-14]★ red_last_seen 은 지우지 않는다 —
            #   '언제 마지막으로 빨간불을 봤나'는 ★해제 판정(_red_gone)의 근거★ 라서
            #   여기서 None 으로 지우면 그 순간 '아주 오래전부터 못 봤다'가 되어
            #   해제 유예(red_release_hold_s)가 통째로 무력화된다.
            self.red_since = None

        if state == 'GREEN':
            if self.green_since is None:
                self.green_since = now
            self.green_last_seen = now
        elif (self.green_last_seen is None
              or (now - self.green_last_seen) > self.tl_gap_grace_s):
            self.green_since = None
            self.green_last_seen = None

        self.tl_state = state
        self.tl_time  = now

    def _fresh(self, now):
        """마지막 판정이 아직 믿을 만한가. 낡았으면 개입하지 않는다(fail-open)."""
        return self.tl_time > 0.0 and (now - self.tl_time) <= self.tl_state_max_age

    def _red_confirmed(self, now):
        """★RED(근접) 만 정지시킨다★ RED_FAR 는 스트릭만 유지하고 세우지 않는다.

        구 white 는 정지선 거리가 있으면 RED_FAR 에서 서행(tl_far_cap)을 했지만,
        white1 에는 속도를 부드럽게 깎을 경로가 없다(리니어는 물리거나 풀리거나
        둘뿐이다). 교차로 한참 전에 2단을 물면 그게 곧 급제동이므로, 근접도 게이트를
        통과한 RED 에서만 선다.
        """
        if self.red_since is None or not self._fresh(now):
            return False
        if self.tl_state != 'RED':
            return False
        return (now - self.red_since) >= self.tl_hold_s

    def _red_gone(self, now):
        """빨간불이 ★red_release_hold_s 이상★ 안 보였는가 = 이제 놓아도 되는가.

        ★[2026-08-14] 해제에 유예를 뒀다 — 실차에서 리니어가 들락날락했다★
        종전 해제 조건은 '_red_confirmed 가 거짓'이었고, 그것은 스트릭이 끊기는
        순간(tl_gap_grace_s 0.3초) 곧바로 참이 된다. 인지가 한두 번 흔들리면
            RED 확정 → 0.3초 놓침 → 해제 → 다시 0.4초 연속 → 체결 …
        이 1초 주기로 반복된다. 리니어는 물리적으로 왕복하는 장치라 이 왕복이 가장
        나쁘다(B보드 1회 구동 최대 1초). 그래서 ★놓는 쪽에만★ 유예를 준다 —
        무는 쪽(tl_hold_s)은 그대로 두어 반응이 늦어지지 않게 한다.
        """
        if self.red_last_seen is None:
            return True
        return (now - self.red_last_seen) >= self.red_release_hold_s

    def _green_confirmed(self, now):
        if self.green_since is None or not self._fresh(now):
            return False
        return (now - self.green_since) >= self.green_hold_s

    # ══════════════════════════════════════════════════════════════════════════
    #  작동 — 정지 지령
    # ══════════════════════════════════════════════════════════════════════════
    def cb_drive_state(self, msg: String):
        self.drive_state   = str(msg.data).strip()
        self.drive_state_t = time.time()

    def cb_tl_enable(self, msg: Bool):
        """master 의 '신호등 인지' 체크박스. ★사람이 직접 켠 허락★ 이다."""
        self.tl_enable   = bool(msg.data)
        self.tl_enable_t = time.time()

    def cb_tl_permit(self, msg: Bool):
        """driving 의 허락. ★코드 내부 상수(TRAFFIC_LIGHT_ENABLE) + DRIVE_RUN★ 이다.
        매핑(MAP_*)·IDLE 에서는 driving 이 False 를 보낸다 — 그쪽에서 이미 걸러진다."""
        self.tl_permit   = bool(msg.data)
        self.tl_permit_t = time.time()

    def _permitted(self, now):
        """지금 이 차에 손을 대도 되는가 (안전 규약 ②) — 둘 중 하나면 된다.

          · 사람이 master 에서 '신호등 인지' 를 켰다 (/tl_enable)  ★체크되어 있으면 상시★
          · driving 이 허락했다 (/tl_permit) = TRAFFIC_LIGHT_ENABLE 이면서 DRIVE_RUN

        둘 다 ★신선할 때만★ 인정한다 — 발행하던 노드가 죽어 마지막 True 가 굳어 있는
        것을 허락으로 읽으면, 카메라만 살아 있는 상태에서 차가 영영 물려 있게 된다.
        """
        if not self.require_permission:
            return True
        if self.tl_enable and (now - self.tl_enable_t) <= TL_ENABLE_STALE_S:
            return True
        if self.tl_permit and (now - self.tl_permit_t) <= TL_ENABLE_STALE_S:
            return True
        return False

    def tick(self):
        now = time.time()

        if not self._permitted(now):
            # 허락이 사라졌다(체크 해제·주행 종료) — 걸어 둔 것이 있으면 풀고 손을 뗀다.
            if self.stopping:
                self.get_logger().warn(
                    "신호등 정지 해제 — 개입 허락이 없다"
                    f"(tl_enable={self.tl_enable} tl_permit={self.tl_permit} "
                    f"drive_state={self.drive_state or '없음'})")
            self._set_stopping(False)
            self._apply_brake(self.brake_release_level)
            # ★요구도 반드시 0 으로 내린다★ 여기서 그냥 return 하면 /tl_brake_req 가
            #   끊길 뿐이라, master 는 낡음 판정(TL_REQ_STALE_S)까지 ★마지막 2단을 계속
            #   주장★ 한다 — 체크를 껐는데 1초 더 물려 있는 꼴이 된다.
            self.pub_req.publish(Int32(data=0))
            return

        if not self.stopping:
            if self._red_confirmed(now):
                self._set_stopping(True)
        elif self.stop_latch:
            # 래치 ON(선택) — 초록불을 확정해야 놓는다. 근접하면 신호등이 ROI 를 벗어나
            # UNKNOWN 이 되는 것이 정상이라, RED 가 안 보인다고 풀면 차가 굴러간다.
            if self._green_confirmed(now):
                self.get_logger().info("🟢 초록불 확정 — 리니어 해제, 주행 재개")
                self._set_stopping(False)
        elif self._red_gone(now):
            # ★기본 동작 [2026-08-14 지시]★ 빨간불을 보는 동안만 잡는다 —
            #   RED 스트릭이 끊기거나(끊김 유예 tl_gap_grace_s 초과) GREEN 이 확정되면
            #   곧바로 놓는다. 놓는 순간 하는 일은 /brake_level=0 하나뿐이고,
            #   그 다음은 원래 명령의 주인(driving 20Hz / arduino 캐시의 master 레버)이
            #   알아서 이어간다 — 이 노드는 펄스를 기억하지도 복원하지도 않는다.
            # ★[2026-08-14] 해제 조건에서 '초록불 확정' 을 뺐다 — 이것이 왕복의 정체다★
            #   로스백(manual-20260814_151206) 에서 브레이크가 ★0.10초 주기로 2↔0★ 을
            #   50번 반복했다. mode·board·estop 은 전부 정상이었다.
            #   원인 : 해제 조건이 (RED 1초 미감지) ★또는★ (GREEN 확정) 이었다.
            #   RED 와 GREEN 이 프레임마다 번갈아 잡히면(가로형 4구에서 적색등과
            #   좌회전 녹색등이 함께 보이거나, 모델이 두 클래스를 오갈 때) ★두 스트릭이
            #   동시에 살아 있어★ 매 틱 '물어라(RED 확정)' 와 '놓아라(GREEN 확정)' 가
            #   같이 참이 된다 → 틱 주기로 flip-flop.
            #   ★이제 해제 근거는 하나뿐이다 — 빨간불을 red_release_hold_s 동안 못 봄★
            #   초록불이 보인다는 것은 곧 빨간불이 없다는 뜻이므로, 그 1초가 지나면
            #   어차피 풀린다. 판정 근거를 하나로 줄이면 왕복이 구조적으로 불가능해진다.
            self.get_logger().info(
                f"⚪ 빨간불 {self.red_release_hold_s:.1f}초 미감지 — 리니어 해제"
                + ("  (초록불 확정 상태)" if self._green_confirmed(now) else ""))
            self._set_stopping(False)

        # 브레이크 요청은 매 틱 같은 값을 내도 안전하다(_apply_brake 가 변화분만 발행).
        # /cmd_vel_raw 는 ★정지 중에만★ 낸다 — 풀리는 즉시 토픽의 주인을 driving 에게
        # 돌려주기 위해서다(파일 헤더의 발행자 충돌 설명 참고).
        # ★내 '요구'를 따로 알린다★ master 가 이것을 자기 레버값과 max 로 합쳐서
        #   /brake_level 을 내므로, 두 발행자가 같은 값을 내게 되어 다툼이 사라진다.
        self.pub_req.publish(Int32(data=self.brake_level if self.stopping else 0))

        if self.stopping:
            self._apply_brake(self.brake_level)
            if self.publish_cmd_vel:
                # ★펄스 0 · 조향 0★ — 펄스는 arduino 가 브레이크 때문에 어차피 0 으로
                #   덮지만, 여기서도 명시적으로 0 을 내야 브레이크를 푸는 순간
                #   직전 주행값이 되살아나지 않는다. 조향 0 은 정지 중 바퀴를 일직선
                #   으로 두기 위한 것이다(driving 이 종점 정지에서 하는 것과 같다).
                out = Twist()
                out.linear.x  = 0.0
                out.angular.z = 0.0
                self.pub_cmd.publish(out)
        else:
            self._apply_brake(self.brake_release_level)

    def _set_stopping(self, on):
        """정지 상태 플래그. 실제 발행은 tick() 과 _apply_brake() 가 한다."""
        if on == self.stopping:
            return
        self.stopping = on
        if on:
            self.get_logger().warn(
                f"🚦🛑 빨간불 확정 — 정지 (리니어 {self.brake_level}단"
                + (" + /cmd_vel_raw 0/0)" if self.publish_cmd_vel else ")"))

    def _apply_brake(self, level):
        """리니어 단계를 요청한다 — ★값이 바뀔 때만★ 발행한다.

        (안전 규약 ③) 해제(0단)는 우리가 직접 건 적이 있을 때만 낸다. brake_now 가
        None 이면 이 노드는 브레이크의 주인이 아니므로 아무것도 내지 않는다 —
        driving 이 DRIVE_DONE 에서 물고 있는 2단을 우리가 푸는 사고를 막는다.

        ★[2026-08-14] 이 가드만으로는 부족한 구간이 하나 더 있다★ white1 의 종점
        접근제동(goal_phase=BRAKE)은 ★DRIVE_RUN 상태에서★ 리니어를 문다 — 여기서는
        DRIVE_DONE 이 아니라 걸러지지 않는다. 그래서 driving 쪽에 ★자기 브레이크
        재확인(keepalive)★ 을 두었다: driving 은 0 이 아닌 단계를 물고 있는 동안
        0.5초마다 같은 값을 다시 발행한다(driving.set_brake). 우리가 낸 0 이 그 구간에
        섞여도 driving 이 곧바로 되돌린다.
        """
        level = max(0, min(2, int(level)))
        if level == 0:
            if self.brake_now in (None, 0):
                return          # 건 적이 없다 = 풀 것도 없다
            if self.drive_state == DRIVE_DONE_STATE:
                # driving 이 도착·경로이탈로 스스로 2단을 물고 있는 구간이다.
                # 여기서 0 을 내면 남의 정지를 푸는 것이 된다 — 소유권만 넘긴다.
                self.get_logger().warn("리니어 해제 보류 — driving 이 DRIVE_DONE 으로 물고 있다")
                self.brake_now = None
                return
        if level == self.brake_now:
            # ★재확인 [2026-08-14]★ 값이 같아도 물고 있는 동안은 주기적으로 다시 낸다.
            #   /brake_level 은 마지막 발행자가 이기는 '명령' 토픽인데 master 가
            #   KEEPALIVE_S(0.5s)마다 자기 레버값(0단)을 재발행한다 — 그대로 두면
            #   2단을 건 0.5초 뒤 리니어가 도로 풀린다(실측 로그).
            if level > 0 and (time.time() - self._brake_t) >= BRAKE_KEEPALIVE_S:
                self._brake_t = time.time()
                self.pub_brake.publish(Int32(data=level))
            return
        self.brake_now = level
        self._brake_t = time.time()
        self.pub_brake.publish(Int32(data=level))
        self.get_logger().info(
            f"🛑 리니어 브레이크 {level}단 "
            f"({'체결 — 신호등 정지' if level > 0 else '해제'})")

    def status_tick(self):
        now = time.time()
        if self.model is None:
            self.get_logger().error("⛔ 신호등 모델이 없다 — 이 노드는 정지를 걸지 않는다",
                                    throttle_duration_sec=10.0)
            return
        if self.img_time == 0.0:
            self.get_logger().warn(f"⏳ {self.image_topic} 수신 대기 (usb_cam 미기동?)",
                                   throttle_duration_sec=5.0)
        elif (now - self.img_time) > 2.0:
            self.get_logger().warn(
                f"⛔ {self.image_topic} {now - self.img_time:.1f}s 두절 — 카메라 확인!"
                + ("  ※ 정지 상태는 래치로 유지된다(수동조종으로 내리면 풀린다)"
                   if self.stopping else "  ※ 신호등 개입은 하지 않는다"),
                throttle_duration_sec=5.0)
        if self.stopping:
            self.get_logger().info(
                f"🛑 신호등 정지 유지 중 | tl={self.tl_state} raw={self.last_raw} "
                f"drop={self.last_red_drop} fps={self.fps:.1f}",
                throttle_duration_sec=2.0)

    def shutdown_release(self):
        """내려갈 때 리니어를 반드시 풀고 간다.

        arduino 는 마지막 /brake_level 을 캐시로 물고 있다 — 이 노드가 2단을 건 채
        죽으면 ★차가 영영 못 움직인다★(수동조종으로 D5 를 내리면 arduino 가 캐시를
        지워 주는 것이 유일한 탈출구다). 발행 직후 프로세스가 끝나면 DDS 가 아직
        못 내보냈을 수 있으므로 잠깐 스핀해서 실제로 나가게 한다.

        ★이게 되려면 rclpy 컨텍스트가 아직 살아 있어야 한다★ — main() 이 rclpy 의
        기본 시그널 핸들러를 끄는 이유가 이것이다(아래 참고).
        """
        try:
            if self.brake_now not in (None, 0):
                self.pub_brake.publish(Int32(data=0))
                self.get_logger().info("🛑 종료 — 리니어 해제(0단) 발행")
                for _ in range(10):
                    rclpy.spin_once(self, timeout_sec=0.03)
        except Exception as e:
            # 여기까지 왔는데 실패하면 리니어가 물린 채 남는다 — 조용히 넘기지 않는다.
            print(f"⛔ 종료 시 리니어 해제 발행 실패: {e} "
                  f"— 수동조종(D5)으로 내리면 arduino 가 브레이크 요청을 지운다")
        if self.show_window:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass


def _install_shutdown_signals():
    """SIGINT/SIGTERM 을 ★파이썬 예외★ 로 받는다.

    ★왜 rclpy 기본 핸들러를 쓰지 않는가★ rclpy 의 기본 SIGINT 핸들러는 컨텍스트를
    먼저 내려버린다. 그러면 spin() 이 풀린 뒤 finally 에서 /brake_level=0 을 내려 해도
    'publisher's context is invalid' 로 실패한다 — ★리니어가 2단으로 물린 채 노드만
    사라진다★(실측으로 확인했다: 종료 로그에 해제가 찍히지 않았다).
    그래서 rclpy 핸들러를 끄고(SignalHandlerOptions.NO) 파이썬 기본 동작
    (SIGINT → KeyboardInterrupt)을 쓴다. launch 가 보내는 SIGTERM 도 같은 예외로
    바꿔 준다 — 그래야 두 경로 모두 shutdown_release() 를 지나간다.
    """
    import signal

    def _term(_sig, _frm):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _term)


def main(args=None):
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init(args=args)      # 구버전 rclpy 폴백
    _install_shutdown_signals()

    node = TrafficLight()
    # ★[2026-08-14] 단일 스레드 spin 을 버렸다 — 제어 틱이 추론에 굶으면 안 된다★
    #   영상 콜백은 한 프레임에 YOLO 추론 + 그리기 + imshow 를 한다(1920x1080).
    #   단일 스레드 실행기에서는 그동안 tick() 이 대기하는데, tick 은 ★브레이크를
    #   유지·해제하는 곳★ 이다. 30fps 를 겨우 맞추는 상태에서 GPU 나 창 합성이 한 번
    #   밀리면 브레이크 재확인과 /tl_brake_req 발행이 함께 늦어지고, 그것을 소비자가
    #   '요구 없음'으로 읽으면 리니어가 풀린다 — 실차에서 본 왕복의 한 경로다.
    #   ★두 콜백 그룹을 나눠 서로를 막지 못하게 한다★(생성자에서 그룹을 지정했다).
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    #  ★spin 은 배경 스레드, 창은 메인 스레드★ [2026-08-14]
    #    OpenCV HighGUI 는 스레드 안전하지 않다. 실행기 워커에서 imshow 를 부르면 그
    #    스레드가 GTK 안에서 멎고, 그 스레드가 맡은 ★영상 콜백이 영영 안 돌아★
    #    '/image_raw 두절' 로 나타난다(실측으로 확인한 회귀다). 그래서 표시만 메인
    #    스레드로 뺀다 — master.py·prompt_g.py 가 지키는 규약과 같다.
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        while rclpy.ok():
            node.show_pending()
            time.sleep(0.02)          # 표시 주기 ≈50Hz 상한(그릴 것이 없으면 즉시 반환)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.shutdown_release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
