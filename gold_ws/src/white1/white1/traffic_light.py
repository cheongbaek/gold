#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
traffic_light.py ― 신호등 인지·정지 [white1]
════════════════════════════════════════════════════════════════════════════════
카메라로 신호등을 보고 ★빨간불이 확정되면 리니어 2단(/brake_level=2)★ 으로 세운다.
빨간불이 red_release_hold_s 동안 안 보이고 초록이 확정되면 놓는다 — 초록도 못 보면 red_release_hold_lost_s 까지
기다린다 [2026-10-08](stop_latch:=true 면 초록 확정까지 문다).
변경 이력과 각 값의 실측 근거는 white1/CHANGELOG.md 에 있다.
[2026-10-07] 인지 쪽은 white2 판(10/5 ROI·1280 엔진·구간 게이트 · 10/6 정리)을 그대로 옮겨 왔다 —
두 패키지의 이 파일은 머리말·import 줄만 다르다.

⚠️ ★[2026-10-01] white1 의 자율주행에서는 이 노드가 제동하지 않는다★ driving 이 /tl_permit 을 항상
   False 로 낸다 — 신호 정지는 driving 이 이 노드의 /tl/state(색)만 읽어 ★정지선(T 구간 끝) 1 m 앞★ 에
   맞춰 한다(driving.py 상수절 '신호 접근', CLAUDE.md 4.10). 아래 '자율주행' 줄과 정지선 판단은
   white2(카메라가 세우는 판)의 방식이고, white1 에서는 master 체크박스(/tl_enable)로 켜는 수동조종
   쪽에서만 쓰인다. ★white1 에서 이 노드가 주행에 주는 것은 /tl/state 하나다.★
   · white1 의 /tl_zone 은 'T'·'T1'~'T5'(구간 시작 40 m 앞부터)다 — cb_tl_zone 이 첫 글자만 봐서
     'T' 로 접힌다. 'A'(좌회전)는 white1 driving 이 내지 않는다.

개입은 허락받은 구간에서만 한다:
  · 자율주행 : /tl_permit (driving 이 DRIVE_RUN 에서 낸다). 놓으면 driving 의 목표펄스가
               그대로 다시 통해 스스로 재출발한다(이 노드는 펄스를 기억하지 않는다).
  · 수동 조종 : /tl_enable (nxde master 의 '신호등 인지' 체크박스).

════════════════════════════════════════════════════════════════════════════════
 인지 (프레임마다, cb_image)
════════════════════════════════════════════════════════════════════════════════
  보정(camera_model.undistort) → ROI → YOLO → 크기·종횡비·conf 필터 → HSV 색 교정
  → RED 적색픽셀 관문 → RED / RED_FAR / GREEN / UNKNOWN
  · RED 와 RED_FAR 는 박스 높이로 가른다 — tl_red_stop_min_height(25px), 신호 구간(/tl_zone
    'T'·'A')에서는 tl_zone_red_min_height(9px). 'A' 에서는 빨강+좌회전 화살표가 진행이다.
  · 정지선 seg 는 빨간 박스가 보이는 동안(sl_gate_red_s)만 같은 프레임에 한 번 더 돈다.

════════════════════════════════════════════════════════════════════════════════
 판단 (30Hz, tick) — 단계는 0 과 2 뿐, 올라가기만 한다
════════════════════════════════════════════════════════════════════════════════
  RED 확정 = 근접 RED 가 tl_hold_s 이어짐(tl_gap_grace_s 이내 끊김은 봐준다).
  확정 뒤에는 마지막 RED 로부터 tl_arm_hold_s 동안 확정이 이어진 것으로 본다(_red_armed).
     ├ 정지선이 확정됐고 BEV 행 ≥ sl_trigger_bev_y          → 2단 [정지선 앞]
     ├ 확정했던 정지선을 놓쳤다(sl_lost_min_bev_y 까지 온 것) → 2단 [정지선 놓침]
     ├ 빨간 박스 높이 ≥ tl_solo_stop_min_height            → 2단 [신호등 단독]
     └ 그 밖                                                  → 무개입 (기다린다)
  두 근거는 OR 다 — 가까워지면 등기구가 ROI 위로 벗어나 정지선보다 먼저 사라지기 때문이다.
  0 으로 돌아가는 길은 해제뿐이다 — 빨간불 사라짐 · 허락 없음 · (래치면) 초록 확정.
  ⚠️ 남은 구멍 : 신호등이 단독 문턱까지 안 크고 정지선도 못 보면 안 선다.
  ⚠️ 사다리꼴(camera_model)을 바꾸면 sl_trigger_bev_y 의 뜻도 바뀐다.

════════════════════════════════════════════════════════════════════════════════
 발행 / 구독
════════════════════════════════════════════════════════════════════════════════
발행:
  /brake_level        Int32    ★정지의 본체★ — arduino.compose() 가 brake>0 이면 구동펄스를 0 으로 덮는다
  /tl_brake_req       Int32    지금 요구하는 단계 — master·driving 이 자기 값과 max 로 합친다
  /cmd_vel_raw        Twist    2단 동안 펄스 0·조향 0 (publish_cmd_vel; 런치 기본 false)
  /tl/state           String   RED / RED_FAR / GREEN / UNKNOWN
  /tl/near_metric     Float32  빨간 박스 높이 최대 [px] (기록용)
  /tl/red_far         Bool     이번 프레임이 RED_FAR 인가 (기록용)
  /tl/stop_line_bev_y Float32  정지선 최근접점의 BEV 행 = 판정값. SL_NONE(−9999) = 미검출
  /tl/stop_line_px    Float32  정지선 → 앞범퍼 [px]. −1 = 미검출 (기록·HUD 용)
  /tl/stop_line_y     Float32  정지선 최하단 y / 프레임 높이. −1 = 미검출 (기록용)
  /tl/stop_line_wait  Bool     RED 확정인데 아직 0단으로 기다리는 중인가
  /tl/debug_image     Image    디버그 화면 (tl_publish_debug)
  /tl/boxes           String   [진단] 이 프레임 판단에 쓴 박스 JSON — [[x1,y1,x2,y2,색,conf,모델색],…] 원본 좌표
구독:
  /image_raw  /drive_state  /tl_enable  /tl_permit  /tl_zone  /tl/fake_box_h(시험용)

════════════════════════════════════════════════════════════════════════════════
 안전 규약
════════════════════════════════════════════════════════════════════════════════
 ① fail-open : 모델이 없거나 영상이 끊기면(tl_state_max_age) 아무 개입도 하지 않는다.
 ② 허락받은 구간에서만 문다(위). 수동조종(D5)에서는 arduino 가 브레이크를 0 으로 보낸다.
 ③ 남의 브레이크는 풀지 않는다 — 0단은 이 노드가 2단을 건 적이 있을 때만 내고,
    driving 이 DRIVE_DONE 으로 물고 있으면 해제를 내지 않는다.
 ⚠️ 신호등이 시야를 완전히 벗어나면 red_release_hold_lost_s 뒤 차는 다시 굴러간다 —
    더 가까이 세우려면 근접 게이트가 아니라 카메라를 위로 틸트한다.
"""

import json
import os
import threading
import time

import cv2
import numpy as np
#  ★PIL 은 반드시 별명으로 받는다★ 아래에서 sensor_msgs 의 Image 를 그대로 받으므로,
#  별명 없이 `from PIL import Image` 를 쓰면 ROS 메시지 타입이 PIL 을 덮어써
#  ★cb_image 의 타입 힌트가 조용히 PIL 을 가리킨다★. 실제로 한 번 밟았다.
try:
    from PIL import Image as PILImage, ImageDraw, ImageFont
except ImportError:                     # fail-open — 아래 TextRenderer 주석 참고
    PILImage = ImageDraw = ImageFont = None
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

# 카메라 기하(어안 보정·BEV)의 단일 소유자. 보정 계수도 사다리꼴도 이 파일에는 없다.
from white1 import camera_model

# HSV 가 YOLO 라벨을 ★위험한 방향(RED→GREEN)★ 으로 뒤집는 것을 허용하는 conf 상한.
# 이보다 자신 있는 박스는 색 몇 픽셀로 뒤집지 않는다. (perception.py 와 같은 값)
HSV_FALLBACK_CONF = 0.55

# 정지선 미검출 표식(BEV 행). −1 은 못 쓴다 — 먼 정지선은 BEV 행이 실제로 음수다(−185 까지 실측).
SL_NONE = -9999.0

# driving.py 의 상태 문자열(driving.py S_* 상수와 같아야 한다).
DRIVE_DONE_STATE = 'DRIVE_DONE'

# 신호등 가중치(실차 PC 경로). .engine 은 빌드한 GPU·드라이버에 묶인다 — 다른 기계면
#   그 기계에서 export 하거나 tl_weights 로 .pt 를 준다. 1280 엔진은 ROI(640x560)를 2배로
#   넣는다(원거리 검출률 36→68%). 실차 PC 에서는 tools/export_tl_1280.sh 를 한 번 돌린다.
#   1280 엔진이 없으면 640 엔진(FALLBACK)으로 내려간다(_load_model).
TL_WEIGHTS = '/home/mad1/runs/detect/combined_light/weights_1280/best.engine'
TL_WEIGHTS_FALLBACK = '/home/mad1/runs/detect/combined_light/weights/best.engine'

# 정지선 가중치 — 구 white 의 차선 seg 모델. 클래스 {0: crosswalk, 1: lain_lines,
#   2: stop-line} 중 이름에 'stop' 이 든 것을 쓰고, 없으면 SL_CLASS_FALLBACK.
SL_WEIGHTS = '/home/mad1/runs/segment/lane_line_new2/weights/best.engine'
SL_CLASS_FALLBACK = 2

# master 의 '신호등 인지' 체크박스가 이 주기보다 오래 끊기면 '허락 없음'으로 본다.
#   (master 창이 죽었는데 체크가 켜진 채로 굳어 있는 상태를 막는다)
TL_ENABLE_STALE_S = 2.0

# 물고 있는 동안 브레이크를 다시 주장하는 주기 [s]. master 의 KEEPALIVE_S(0.5)보다
# 짧아야 한다 — 그래야 남이 덮어도 곧바로 되돌아온다(_apply_brake 주석 참고).
BRAKE_KEEPALIVE_S = 0.25



# ══════════════════════════════════════════════════════════════════════════════
#  창에 글자를 그리는 도구 — 한글 HUD
# ══════════════════════════════════════════════════════════════════════════════
#  cv2.putText 는 ASCII 만 그려서 한글은 PIL 로 그린다(ultralytics 가 이미 pillow 를 요구한다).
#  폰트가 없으면 경고만 남기고 cv2 로 물러난다(fail-open). PIL 스트립 하나가 3ms 라
#  폰트·글자폭·스트립(내용이 같으면 지난 그림)을 캐시한다.
#  폰트 후보는 앞에서부터 — 1순위 NanumGothicCoding 은 고정폭이라 HUD 열이 맞는다.
FONT_CANDIDATES = (
    '/usr/share/fonts/truetype/nanum/NanumGothicCoding.ttf',
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/truetype/nanum/NanumGothic.ttf',
    '/usr/share/fonts/truetype/unfonts-core/UnDotum.ttf',
)

#  ── 색 (BGR) ─────────────────────────────────────────────────────────────
#  신호등 창이 쓰는 색은 ★화면의 의미와 1:1★ 이다. 여기가 그 정본이다.
C_BG     = (26, 26, 28)        # 패널 바닥
C_HUD    = (15, 15, 17)        # 하단 HUD 띠 바닥
C_TXT    = (228, 228, 232)     # 보통 글자
C_DIM    = (138, 138, 144)     # 참고값 글자
C_FRAME  = (92, 92, 96)        # 패널 테두리
C_RED    = (60, 60, 255)       # RED · 2단
C_GREEN  = (80, 220, 90)       # GREEN · 범퍼선 · 허락 있음
C_AMBER  = (0, 180, 255)       # 경고 · 단독 문턱 눈금
C_CYAN   = (235, 225, 70)      # BEV 사다리꼴
C_MAGENTA = (255, 0, 255)      # 정지선
C_GRAY   = (150, 150, 150)     # UNKNOWN 박스


class TextRenderer:
    """한글 스트립 렌더러. ★상태는 캐시뿐이다★ (그림을 기억하지 않는다).

        tr = TextRenderer(log=node.get_logger())
        strip = tr.strip(w, h, [(x, y, '제동 2단', C_RED, 15, True)], C_HUD)

    items 의 한 원소 = (x, y, 글자, BGR색, 크기, 굵게). y 는 ★글자 상단★ 이다
    (cv2.putText 의 baseline 과 다르다 — 줄 배치를 위에서부터 세는 것이 편하다).
    """

    def __init__(self, log=None, font_path=''):
        self._log = log
        self.path = ''
        if PILImage is None:
            self._warn("Pillow 가 없다 — HUD 한글이 물음표로 나온다(그 외는 정상)")
        else:
            cands = ((font_path,) if font_path else ()) + FONT_CANDIDATES
            self.path = next((p for p in cands if p and os.path.exists(p)), '')
            if not self.path:
                self._warn("한글 폰트를 못 찾았다 — HUD 한글이 물음표로 나온다. "
                           "`sudo apt install fonts-nanum-coding` 로 해결된다")
        self.ok = bool(self.path)
        self._font, self._width, self._cell, self._patch = {}, {}, {}, {}

    def _warn(self, msg):
        if self._log is not None:
            self._log.warn(f"🖋 {msg}")

    # ── 폰트·치수 ────────────────────────────────────────────────────────
    def font(self, size, bold=False):
        key = (size, bold)
        if key not in self._font:
            path = self.path
            if bold:
                # 같은 계열의 Bold 가 옆에 있으면 그것을 쓴다(없으면 보통 굵기).
                alt = path.replace('Coding.ttf', 'CodingBold.ttf').replace(
                    'Gothic.ttf', 'GothicBold.ttf').replace('Regular', 'Bold')
                if alt != path and os.path.exists(alt):
                    path = alt
            self._font[key] = ImageFont.truetype(path, size)
        return self._font[key]

    def width(self, txt, size, bold=False):
        """글자 폭[px]. ★매 프레임 열 배치에 쓰므로 캐시한다★.

        ⚠️ 폰트가 없을 때도 ★실제로 재야 한다★ — 글자 수 × 상수로 어림하면 물러난
           경로에서 열 계산이 틀려 ★두 칸이 겹쳐 찍힌다★(실제로 그렇게 나왔다).
           cv2 로 그릴 것이므로 cv2 의 자로 잰다.
        """
        key = (txt, size, bold)
        if key not in self._width:
            self._width[key] = (
                float(self.font(size, bold).getlength(txt)) if self.ok
                else float(cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX,
                                           size / 32.0, 1)[0][0]))
        return self._width[key]

    def cell(self, size):
        """한 칸(=ASCII 한 글자) 폭. HUD 의 열 좌표 단위다."""
        if size not in self._cell:
            self._cell[size] = self.width('A', size) if self.ok else size * 0.55
        return self._cell[size]

    def line_h(self, size):
        """줄 높이 — 크기에 여백을 더한 값. 줄 간격의 정본이다."""
        return size + 6

    # ── 그리기 ──────────────────────────────────────────────────────────
    def strip(self, w, h, items, bg):
        """단색 바닥 위에 글자만 있는 스트립을 새로 그려 돌려준다."""
        img = np.full((h, w, 3), bg, np.uint8)
        self.onto(img, items)
        return img

    def onto(self, img, items):
        """이미 있는 그림 위에 글자를 얹는다(PIL 변환 1회로 몰아 그린다).

        ⚠️ img 는 ★연속 배열★ 이어야 한다(PILImage.fromarray 의 전제다). 그래서
           캔버스 슬라이스에 직접 쓰지 않고, strip() 으로 새 배열에 그린 뒤 blit 한다.
        """
        if not items:
            return img
        if not self.ok:
            # fail-open — 한글은 물음표가 되지만 숫자·영문은 그대로 읽힌다.
            for x, y, txt, col, size, _bold in items:
                cv2.putText(img, txt, (int(x), int(y) + size - 2),
                            cv2.FONT_HERSHEY_SIMPLEX, size / 32.0, col, 1, cv2.LINE_AA)
            return img
        pim = PILImage.fromarray(img)
        draw = ImageDraw.Draw(pim)
        for x, y, txt, col, size, bold in items:
            # 색을 뒤집지 않는다 — img 는 BGR 배열이고 PIL 은 그것을 변환 없이
            #   채널 순서 그대로 쓴다 — 그러니 BGR 색을 그대로 넘겨야 BGR 로 들어간다.
            draw.text((x, y), txt, font=self.font(size, bold), fill=tuple(col))
        # np.asarray 는 PIL 버퍼를 감싸기만 하므로, 원본 img 로 되돌려 복사한다.
        img[:] = np.asarray(pim)
        return img

    def patch(self, txt, col, size=12, bold=False, bg=C_BG):
        """짧은 ★고정 라벨★ — 한 번 그려 두고 blit 한다(패널 라벨용)."""
        key = (txt, col, size, bold, bg)
        if key not in self._patch:
            w = int(self.width(txt, size, bold)) + 5
            self._patch[key] = self.strip(w, size + 6, [(2, 1, txt, col, size, bold)], bg)
        return self._patch[key]


class StripCache:
    """스트립 하나를 ★내용이 바뀔 때만★ 다시 그린다.

    HUD 는 대부분의 프레임에서 글자가 그대로다(FPS 를 정수로 찍는 이유이기도 하다).
    key 가 같으면 지난 그림을 그대로 돌려주므로 PIL 이 아예 돌지 않는다.
    """

    def __init__(self, tr):
        self.tr = tr
        self._key = None
        self._img = None

    def get(self, w, h, items, bg):
        key = (w, h, bg, tuple((round(i[0]), round(i[1]), i[2], i[3], i[4], i[5])
                               for i in items))
        if key != self._key:
            self._key = key
            self._img = self.tr.strip(w, h, items, bg)
        return self._img


class CanvasPool:
    """표시용 캔버스를 ★두 장 만들어 돌려 쓴다★.

    ★왜 매 프레임 만들지 않는가★ np.full + np.vstack 으로 1248x610 캔버스를 매
    프레임 새로 만들면 그것만 3.6ms 다(실측). 그리기 전체 예산보다 크다.

    ★왜 한 장이 아니라 두 장인가★ 그리는 것은 워커 스레드고 imshow 는 메인
    스레드다(traffic_light.main 의 규약). 한 장만 돌려 쓰면 메인이 창에 올리는 사이
    워커가 같은 버퍼를 덮어써 ★화면이 찢어진다★. 두 장이면 다음 덮어쓰기가 두
    프레임 뒤(30fps 에서 66ms)라 겹치지 않는다.
    """

    def __init__(self):
        self._key = None
        self._buf = []
        self._i = 0

    def get(self, w, h, bg=C_BG):
        if self._key != (w, h, bg):
            self._key = (w, h, bg)
            self._buf = [np.full((h, w, 3), bg, np.uint8) for _ in range(2)]
            self._i = 0
        self._i ^= 1
        return self._buf[self._i]


# ══════════════════════════════════════════════════════════════════════════════
#  작은 그리기 도구 — 전부 ASCII 전용(cv2)이라 비용이 사실상 0 이다
# ══════════════════════════════════════════════════════════════════════════════
def atxt(img, x, y, txt, col, scale=0.4, thick=1):
    """ASCII 글자. y 는 baseline (cv2.putText 그대로)."""
    cv2.putText(img, txt, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX,
                scale, col, thick, cv2.LINE_AA)


def chip(img, x, y, txt, col, scale=0.36):
    """★배경칩을 깐 라벨★ — 밝은 하늘·노면 위에서도 읽히게 하는 유일한 방법이다.

    y 는 칩의 ★위쪽★ 이다. 돌려주는 것은 칩의 높이(다음 줄을 쌓을 때 쓴다).
    """
    (tw, th), bl = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    h = th + bl + 4
    x, y = int(x), int(y)
    cv2.rectangle(img, (x, y), (x + tw + 6, y + h), col, -1)
    atxt(img, x + 3, y + th + 2, txt, (255, 255, 255), scale)
    return h


def gauge(img, x, y, w, h, frac, col, ticks=()):
    """★지금값을 문턱과 나란히 보여주는 막대★ 숫자보다 눈이 빠르다.

    frac  = 0.0~1.0 로 정규화된 지금값 (범위를 넘으면 꽉 찬다)
    ticks = ((위치 0~1, BGR), …) 문턱 눈금. 막대가 눈금을 넘으면 그 문턱을 넘은 것이다.
    """
    x, y, w, h = int(x), int(y), int(w), int(h)
    cv2.rectangle(img, (x, y), (x + w, y + h), (60, 60, 64), -1)
    fw = int(round(w * min(1.0, max(0.0, frac))))
    if fw > 0:
        cv2.rectangle(img, (x, y), (x + fw, y + h), col, -1)
    for tf, tc in ticks:
        tx = x + int(round(w * min(1.0, max(0.0, tf))))
        cv2.line(img, (tx, y - 3), (tx, y + h + 3), tc, 2)
    cv2.rectangle(img, (x, y), (x + w, y + h), (105, 105, 110), 1)


def blit(dst, patch, x, y):
    """patch 를 dst 의 (x, y) 에 붙인다. ★화면 밖은 잘라 낸다★ (예외를 내지 않는다)."""
    x, y = int(x), int(y)
    ph, pw = patch.shape[:2]
    H, W = dst.shape[:2]
    if x >= W or y >= H:
        return
    w, h = min(pw, W - x), min(ph, H - y)
    if w > 0 and h > 0:
        dst[y:y + h, x:x + w] = patch[:h, :w]


def dim_outside(view, rect, factor):
    """rect(사각형) ★바깥만★ 어둡게 한다 — 선 하나 없이 ROI 를 말하는 방법이다.

    ★사각형 슬라이스 네 장만 만진다★ 불리언 마스크(view[mask==0] = …)로 쓰면 같은
    일에 4배가 든다(팬시 인덱싱이 배열을 두 번 훑는다).
    """
    if factor >= 1.0:
        return
    h, w = view.shape[:2]
    x0, y0, x1, y1 = (max(0, min(w, rect[0])), max(0, min(h, rect[1])),
                      max(0, min(w, rect[2])), max(0, min(h, rect[3])))
    for a, b, c, d in ((0, y0, 0, w), (y1, h, 0, w),
                       (y0, y1, 0, x0), (y0, y1, x1, w)):
        if b > a and d > c:
            band = view[a:b, c:d]
            cv2.addWeighted(band, factor, band, 0.0, 0.0, dst=band)

# ══════════════════════════════════════════════════════════════════════════════
#  ★#400 버스 신호 몸체 찾기★ [2026-10-07] — 구간 'B' 에서만 돈다 (모델 없는 영상처리)
# ══════════════════════════════════════════════════════════════════════════════
#  #400 교차로 신호 팔에는 왼쪽 = 버스 신호(버스 그림 등화), 오른쪽 = 차량 신호(가로 4구) 가 나란하다.
#  모델이 버스 신호를 거의 못 찾는다(차량 박스 ≥15px 프레임 중 36.3%, 144031 은 1.4%) — 학습 데이터에
#  둥근 등화만 있어서다. 그래서 차량 박스를 기준으로 버스가 있어야 할 자리(중심 dx −4.75h, dy −0.45h)에서
#  ★하늘보다 어두운 직사각형 몸체★ 를 찾는다(오프라인 97.8% — 근거는 CHANGELOG 2026-10-07 밤 '#400 두 박스' 항목).
#    기준 밝기 : 몸체 = 차량 박스 안쪽 30 백분위, 하늘 = 탐색창 70 백분위 → 그 사이 dark_t 에서 자른다
#    켜진 등화색(HSV)을 몸체에 더한다 → 닫힘 0.15h → 열림 open_k·h(신호 팔·전선·기둥을 지운다)
#    연결요소 → 모양(높이비·가로세로비·채움·자리) → 넓으면 세로 투영으로 '키 큰 열' 만 다시 본다
#    고립도 : 몸체 아래·왼쪽이 하늘이어야 한다(전주 위 변압기·기기함을 거른다)
#  ⚠️ 낮·흐린 하늘에서만 검증했다 — '몸체가 하늘보다 어둡다' 에 기대므로 역광·야간·건물 배경은 미검증.
#  ⚠️ 모양·자리만으로는 다른 교차로의 나란한 신호(k-city 2:04~2:12)와 못 가른다 — ★구간 'B' 밖에서 쓰지 않는다★.
BUS_BODY = dict(
    dark_t=0.5, close_k=0.15, open_k=0.40,
    hr=(0.60, 1.30), ar=(1.25, 2.50), fill=0.62,
    exp_dx=-4.75, exp_dy=-0.45, win_dx=-7.3, win_dy=(-2.3, 1.7),
    max_dx_dev=1.6, dy_rng=(-1.6, 0.8),
    iso_below=0.12, iso_left=0.20, min_contrast=25.0,
)


def bus_slot(box, veh, tol_dx=1.6, dy_rng=(-1.6, 0.8)):
    """box 의 중심이 veh(차량 박스) 기준 버스 자리 안에 있는가 — 둘 다 [x1,y1,x2,y2] 같은 좌표계."""
    h = max(1.0, veh[3] - veh[1])
    dx = ((box[0] + box[2]) / 2 - (veh[0] + veh[2]) / 2) / h
    dy = ((box[1] + box[3]) / 2 - (veh[1] + veh[3]) / 2) / h
    return abs(dx - BUS_BODY['exp_dx']) <= tol_dx and dy_rng[0] <= dy <= dy_rng[1]


def find_bus_body(img, veh, hsv_ranges, P=BUS_BODY):
    """img: 보정된 원본 BGR, veh: 차량 박스 [x1,y1,x2,y2](같은 좌표). → 버스 몸체 (x1,y1,x2,y2) 또는 None."""
    vx1, vy1, vx2, vy2 = veh
    h = max(1.0, vy2 - vy1); cx = (vx1 + vx2) / 2; cy = (vy1 + vy2) / 2
    H, W = img.shape[:2]
    wx1 = int(max(0, np.floor(cx + P['win_dx'] * h))); wx2 = int(min(W, np.ceil(vx1 - 0.4 * h)))
    wy1 = int(max(0, np.floor(cy + P['win_dy'][0] * h))); wy2 = int(min(H, np.ceil(cy + P['win_dy'][1] * h)))
    if wx2 - wx1 < 0.8 * h or wy2 - wy1 < 0.8 * h:
        return None
    crop = img[wy1:wy2, wx1:wx2]
    win = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    ix1, iy1 = int(round(vx1 + 0.1 * h)), int(round(vy1 + 0.15 * h))
    ix2, iy2 = int(round(vx2 - 0.1 * h)), int(round(vy2 - 0.15 * h))
    cb = img[max(0, iy1):max(iy1 + 1, iy2), max(0, ix1):max(ix1 + 1, ix2)]
    body = (float(np.percentile(cv2.cvtColor(cb, cv2.COLOR_BGR2GRAY), 30)) if cb.size
            else float(np.percentile(win, 5)))
    sky = float(np.percentile(win, 70))
    if sky - body < P['min_contrast']:
        return None
    m = (win < body + P['dark_t'] * (sky - body)).astype(np.uint8) * 255
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    for lo, hi in hsv_ranges:                      # 켜진 등화가 몸체에 구멍을 내지 않게
        m |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    m0 = m.copy()
    for op, k in ((cv2.MORPH_CLOSE, P['close_k']), (cv2.MORPH_OPEN, P['open_k'])):
        k = max(1, int(round(k * h)))
        if k > 1:
            m = cv2.morphologyEx(m, op, cv2.getStructuringElement(cv2.MORPH_RECT, (k, k)))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
    cands = []
    for i in range(1, n):
        x, y, w, hh, a = st[i]
        cands += _bus_judge(x + wx1, y + wy1, w, hh, a, cx, cy, h, P)
        if hh >= P['hr'][0] * h and w / max(1, hh) > P['ar'][1]:      # 팔·이웃과 붙은 넓은 덩어리
            cands += _bus_split(lab[y:y + hh, x:x + w] == i, x + wx1, y + wy1, cx, cy, h, P)
    best = None
    for c in sorted(cands):
        _, x, y, w, hh = c
        below, left = _bus_isolation(m0, x - wx1, y - wy1, w, hh, h)
        if below <= P['iso_below'] and left <= P['iso_left']:
            best = c
            break
    if best is None:
        return None
    _, x, y, w, hh = best
    return (int(x), int(y), int(x + w), int(y + hh))


def _bus_isolation(m0, x, y, w, hh, h):
    H, W = m0.shape
    def frac(ya, yb, xa, xb):
        ya, yb, xa, xb = max(0, int(ya)), min(H, int(yb)), max(0, int(xa)), min(W, int(xb))
        return float((m0[ya:yb, xa:xb] > 0).mean()) if (yb > ya and xb > xa) else 0.0
    return (frac(y + hh + 0.2 * h, y + hh + 1.0 * h, x, x + w),
            frac(y, y + hh, x - 0.8 * h, x - 0.2 * h))


def _bus_judge(x, y, w, hh, area, cx, cy, h, P):
    if hh <= 0 or w <= 0:
        return []
    hr, ar, fill = hh / h, w / hh, area / float(w * hh)
    if not (P['hr'][0] <= hr <= P['hr'][1] and P['ar'][0] <= ar <= P['ar'][1] and fill >= P['fill']):
        return []
    dx, dy = (x + w / 2 - cx) / h, (y + hh / 2 - cy) / h
    if abs(dx - P['exp_dx']) > P['max_dx_dev'] or not (P['dy_rng'][0] <= dy <= P['dy_rng'][1]):
        return []
    cost = ((dx - P['exp_dx']) ** 2 + ((dy - P['exp_dy']) / 0.8) ** 2 + ((hr - 0.9) / 0.3) ** 2
            + ((ar - 1.8) / 0.5) ** 2 + (1 - fill) * 2)
    return [(cost, x, y, w, hh)]


def _bus_split(mask, x0, y0, cx, cy, h, P):
    col = mask.sum(0)
    tall = col >= 0.6 * h
    out, i, n = [], 0, len(tall)
    while i < n:
        if not tall[i]:
            i += 1
            continue
        j = i
        while j < n and tall[j]:
            j += 1
        sub = mask[:, i:j]
        rows = np.where(sub.sum(1) >= 0.5 * (j - i))[0]
        if len(rows):
            r1, r2 = rows[0], rows[-1] + 1
            out += _bus_judge(x0 + i, y0 + r1, j - i, r2 - r1, int(sub[r1:r2].sum()), cx, cy, h, P)
        i = j
    return out


#  ★#400 좌회전 화살표 읽기★ [2026-10-08] — 구간 'B' 판단(2단계)
#  #400 차량 신호는 가로 4구 [빨강 · 황색 · ←좌회전 화살표 · 녹색 원등] 이고 모델 박스가 등기구를 거의 꼭 맞게
#  덮는다(가로세로비 중앙 2.0) — 그래서 박스를 4등분한 셋째 칸이 화살표다. 그 칸 가운데의 녹색 화소 비율을 잰다.
#    칸 x 0.50~0.66 · y 0.15~0.85 — 오른쪽(0.66~0.75)은 원등 번짐이 들어와서 뺐다
#    녹색 H 45~100 · S ≥45 · V ≥40 — 가까우면(≥30px) 등화가 청록으로 어둡게 찍혀(V 60~90) 노드의 HSV(S55·V60)로는 놓친다
#  근거(144031 1:46~ 화살표+원등 · 105807/144031 원등만·빨강 — 노드 박스, 차량 박스 ≥20px):
#    화살표 켜짐 최소 0.42(85프레임) / 꺼짐 최대 0.14(146프레임) → 문턱 0.30(tl_b_arrow_frac).
#    ⚠️ 20px 밑은 원등 번짐이 셋째 칸까지 퍼져 꺼짐이 0.60 까지 나온다 → 읽지 않는다(모름 = 정지, tl_b_arrow_min_h).
#    ⚠️ 빨강+화살표 현시 녹화는 없다 — 원등과 무관하게 셋째 칸만 보므로 같은 식으로 읽힐 것으로 본다(미검증).
ARROW = dict(x=(0.50, 0.66), y=(0.15, 0.85), lo=(45, 45, 40), hi=(100, 255, 255))


def arrow_frac(img, box, P=ARROW):
    """img: 보정된 원본 BGR, box: #400 차량 박스 [x1,y1,x2,y2](같은 좌표) → 셋째 칸(화살표) 녹색 화소 비율 0~1."""
    x1, y1, x2, y2 = [int(round(v)) for v in box[:4]]
    c = img[max(0, y1):max(0, y2), max(0, x1):max(0, x2)]
    if c.size == 0:
        return 0.0
    h, w = c.shape[:2]
    ya = int(round(P['y'][0] * h)); yb = max(ya + 1, int(round(P['y'][1] * h)))
    xa = int(round(P['x'][0] * w)); xb = max(xa + 1, int(round(P['x'][1] * w)))
    m = cv2.inRange(cv2.cvtColor(c[ya:yb, xa:xb], cv2.COLOR_BGR2HSV),
                    np.array(P['lo'], np.uint8), np.array(P['hi'], np.uint8))
    return float(cv2.countNonZero(m)) / max(1, m.size)


class TrafficLight(Node):
    def __init__(self):
        super().__init__('traffic_light')

        # ── 입력 ───────────────────────────────────────────────────────────
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('device',      'cuda:0')   # GPU 없으면 'cpu'

        # ── 신호등 인지 ────────────────────────────────────────────────────
        self.declare_parameter('tl_weights', TL_WEIGHTS)
        self.declare_parameter('tl_conf',      0.35)
        # 정적 TensorRT 엔진의 입력 크기와 짝이다 — 다르면 _load_model 이 엔진 값으로 맞춘다.
        self.declare_parameter('tl_imgsz',     1280)
        # 신호등 탐색 ROI(보정 후 원본 픽셀). 맞은편 신호가 지평선 근처(y≈380~450)라 ymax 560.
        #   640x560 이라 imgsz 1280 이면 정확히 ×2. 좌우·아래로 더 넓히면 교차로 안에서
        #   커진 옆 신호·차량 위 가짜 RED 에 선다(2026-10-05 E2·E3 런).
        self.declare_parameter('tl_roi_xmin',  640)
        self.declare_parameter('tl_roi_ymin',  0)
        self.declare_parameter('tl_roi_xmax',  1280)
        self.declare_parameter('tl_roi_ymax',  560)
        self.declare_parameter('tl_min_area',  20)
        # 박스 높이가 ROI 높이의 이 비율을 넘으면 버린다 — 야간에 ROI 를 통째로 덮는 가짜
        #   박스(ROI 의 85~99%)가 RED 로 읽혀 급정지하던 것을 막는다. 0 = 끔.
        self.declare_parameter('tl_max_height_frac', 0.5)
        self.declare_parameter('tl_min_aspect', 0.2)
        self.declare_parameter('tl_max_aspect', 6.0)   # 가로형 4구 ≈3.5~4.5, 위가 잘리면 더 크다
        # RED(정지 대상) / RED_FAR(아직 멀다) 를 가르는 근접 게이트 [박스 높이 px].
        self.declare_parameter('tl_red_stop_min_height',    25)
        # ★정지선 없이도 설 만큼 가까운가★ — 이 높이 이상이면 정지선과 무관하게 선다
        #   (_stop_plan 의 OR). K-City 에서 25px 첫 도달 때 정지선이 3.1~3.9 m 앞이었다
        #   = 정지선 발화와 거의 같은 자리. 0 이면 정지선을 못 볼 때 안 선다(실차 금지).
        self.declare_parameter('tl_solo_stop_min_height',   25.0)
        # 물고 있는 동안 근접 게이트를 이 비율로 낮춘다 — 박스가 몇 px 줄었다고
        #   RED_FAR 로 떨어져 해제 타이머가 도는 것을 막는다. 1.0 = 히스테리시스 없음.
        self.declare_parameter('tl_near_release_ratio',     0.7)
        # HSV 색 교정. RED 박스는 적색 픽셀이 hsv_min_color_pixels 이상이어야 채택한다
        #   (붉지 않은 RED 오검출 관문). 램프 실측 최저 S=62·V=66, 적색 hue 는 160~169 까지 나온다.
        self.declare_parameter('hsv_min_color_pixels', 15)
        self.declare_parameter('hsv_crop_center_ratio', 0.85)
        self.declare_parameter('hsv_red_h1_low',   0)
        self.declare_parameter('hsv_red_h1_high',  10)
        self.declare_parameter('hsv_red_h2_low',   160)
        self.declare_parameter('hsv_red_h2_high',  180)
        self.declare_parameter('hsv_green_h_low',  45)
        self.declare_parameter('hsv_green_h_high', 90)
        self.declare_parameter('hsv_sat_low',      55)
        self.declare_parameter('hsv_val_low',      60)

        # ── 판단 ───────────────────────────────────────────────────────────
        self.declare_parameter('tl_hold_s',        0.1)   # 근접 RED 가 이만큼 이어지면 확정
        # 이 이내의 끊김은 봐준다. 8~10px 먼 신호는 RED 가 2~3프레임씩만 이어진다(0.05 면 확정이
        #   매번 리셋돼 제동이 0.8초 늦었다 — CHANGELOG 2026-10-06).
        self.declare_parameter('tl_gap_grace_s',   0.15)
        self.declare_parameter('tl_state_max_age', 3.0)   # 이보다 낡은 판정은 무시(fail-open)
        self.declare_parameter('green_hold_s',     0.4)   # stop_latch 일 때 GREEN 확정 시간
        # 빨간불을 이만큼 못 봐야 놓는다(무는 쪽은 그대로) — 검출이 흔들려 리니어가 왕복하지
        #   않게. 0.55초 공백에 풀렸다 다시 물린 실측 때문에 0.7. 전체 해제 지연은 여기 +
        #   arduino BRAKE_RELEASE_HOLD_S(0.5).
        self.declare_parameter('red_release_hold_s', 0.7)
        # [2026-10-08] 그런데 ★초록도 못 봤다면★ 이만큼 더 기다린다 — 서 있는 동안 작은 RED 가 1.0 s 끊겨
        #   0.7 s 에 풀렸다 0.6 s 뒤 다시 물린 실측(k-city 1280 실시간, 인계 미결 1). lockstep 회귀 3영상·k-city 에서
        #   RED 끊김 최장 0.42 s. 초록을 보면 종전처럼 red_release_hold_s 에 놓는다(출발 지연 없음).
        #   red_release_hold_s 이하면 종전과 같다(끔).
        self.declare_parameter('red_release_hold_lost_s', 1.5)
        # 이번 접근에서 확정했으면 마지막 RED 로부터 이만큼은 정지선 계획을 잇는다(_red_armed).
        #   0 = 끔.
        self.declare_parameter('tl_arm_hold_s', 1.0)

        # ── 정지선 ─────────────────────────────────────────────────────────
        #   정지선이 보이는 동안 '아직 멀다 → 기다린다' 를 끼워 넣는 것이 전부다.
        self.declare_parameter('sl_enable',  True)
        self.declare_parameter('sl_weights', SL_WEIGHTS)
        self.declare_parameter('sl_conf',    0.30)
        self.declare_parameter('sl_imgsz',   640)
        # ★발화선★ 정지선 최근접점의 BEV 행이 이 값에 닿으면 2단. BEV 는 아래가 차 쪽이라
        #   클수록 가깝다. 40 = 정지선이 BEV 위쪽에 들어온 직후(≈3.6 m) — 전수 시험 접근 17회 중
        #   정지선 앞 12회·단독 4회·안 섬 1회(440 일 때는 정지선 앞 1회뿐이었다).
        #   ⚠️ 사다리꼴(bev_src_pts)을 바꾸면 이 값도 다시 잡는다.
        self.declare_parameter('sl_trigger_bev_y', 40.0)
        # 정지선은 가로로 긴 물체다 — 폭·면적이 이보다 작으면 노면 잡티로 버린다.
        self.declare_parameter('sl_min_width_frac', 0.12)
        self.declare_parameter('sl_min_area_frac',  0.0004)
        self.declare_parameter('sl_hold_s',  0.2)    # 이만큼 이어서 봐야 확정
        self.declare_parameter('sl_stale_s', 0.5)    # 이보다 오래 못 보면 '놓쳤다'
        self.declare_parameter('sl_gate_red_s', 1.0) # 빨간 박스를 이 안에 봤을 때만 추론(성능)

        # ── 정지 동작 ──────────────────────────────────────────────────────
        self.declare_parameter('brake_level',         2)     # 무는 단계(풀브레이크)
        # 2단 동안 펄스 0·조향 0 도 낸다. arduino 의 명령 캐시를 덮지만 master·driving 이
        #   주기적으로 재발행해 0.5초 안에 돌아온다. 런치(tl_publish_cmd_vel)는 false 를 넘긴다.
        self.declare_parameter('publish_cmd_vel',     True)
        self.declare_parameter('stop_cmd_hz',         30.0)  # 판단 틱 주기
        self.declare_parameter('stop_latch',          False) # true 면 GREEN 확정까지 문다
        self.declare_parameter('require_permission',  True)  # false = 벤치 모드(항상 허락)

        # ── 표시 ───────────────────────────────────────────────────────────
        self.declare_parameter('show_window', True)   # 화면 없는 ssh 면 false
        # _draw() 가 그린 같은 그림을 /tl/debug_image 로 낸다. 실차는 camera_launch 의
        #   tl_record_video 가 이것과 nxde video 노드를 함께 켜서 mp4 로 적는다.
        self.declare_parameter('tl_publish_debug', False)
        self.declare_parameter('draw_roi',    True)
        # 카메라 뷰 폭(우측 패널·HUD 는 밖에 붙는다). 960 = 1920 의 절반이라 리사이즈가
        #   0.28ms(임의 배율은 2.5ms). 0 이면 원본 크기.
        self.declare_parameter('window_width', 960)
        # [2026-10-08] 디버그 화면은 '최소 자원·판단에 필요한 것만'(사용자) — BEV 패널·ROI 확대 열은 기본 끔.
        #   본화면(박스·정지선·판단 박스 확대 1장) + HUD 2줄 = 960x584(종전 1728x616 의 53%).
        self.declare_parameter('show_bev', True)      # 우측 BEV 패널(발화선 썸네일 + 근접·정지선 게이지)
        # ROI 밖 밝기. 너무 낮추면 ROI 밖 노면의 정지선 마스크를 눈으로 못 본다.
        self.declare_parameter('roi_dim', 0.6)
        self.declare_parameter('tl_roi_zoom', True)   # 원본에서 자른 ROI 확대 열 + 박스 타일 4장(판단 박스부터)
        self.declare_parameter('hud_font', '')        # 빈 값 = FONT_CANDIDATES 순서대로

        # ── 시험용 신호등 주입 — ★실차에서는 반드시 0★ ─────────────────────
        #  0 보다 크면 그 높이(px)의 빨간 박스를 봤다고 친다(YOLO 결과를 통째로 대신한다).
        #  신호등이 안 찍힌 영상으로 정지선을 시험하려고 둔다(15 = RED_FAR, 40 = RED).
        #  켜져 있으면 배너와 status_tick 이 계속 경고하고, cam_testbed 계약이 그 로그로 막는다.
        self.declare_parameter('tl_fake_box_h', 0.0)

        # ── 신호 구간 · 작은 맞은편 신호 대응 ───────────────────────────────
        #  /tl_zone (driving 이 경로 CSV terrain 에서 낸다):
        #    'T' 신호 정지 구간 — RED 게이트가 tl_zone_red_min_height 로 내려간다
        #    'A' 좌회전 구간   — 위와 같고, 모델이 GREEN 으로 본 등기구(빨강+좌회전 화살표)는 진행
        #    그 밖·끊김(tl_zone_stale_s) = tl_red_stop_min_height
        #  9 = 본선 정지선 앞 맞은편 신호 9~12px. 0 이면 구간 신호를 무시한다.
        self.declare_parameter('tl_zone_red_min_height', 9)
        self.declare_parameter('tl_zone_stale_s', 1.0)
        #    'B' #400 교차로 구간(좌회전) — 위와 같고, 차량 박스 왼쪽의 ★버스 신호★ 를 찾아(1단계, [2026-10-07])
        #        ★판단에서 뺀다★ — 그리고 차량 박스 하나만 좌회전 화살표로 읽는다(2단계, [2026-10-08]):
        #        화살표 켜짐 = GREEN · 그 밖(원등만·빨강·황색·못 읽음) = RED(높이로 RED/RED_FAR) — _decision_boxes
        self.declare_parameter('tl_bus_detect', True)        # 끄면 'B' 도 'T' 와 똑같다(버스 찾기·화살표 둘 다 꺼진다)
        self.declare_parameter('tl_bus_min_veh_h', 10.0)     # 이보다 작은 차량 박스로는 버스 몸체를 영상처리로 찾지 않는다 [px]
        self.declare_parameter('tl_b_need_arrow', True)      # false = 버스만 빼고 원등에도 간다(직진 경로일 때)
        self.declare_parameter('tl_b_arrow_min_h', 20.0)     # 이보다 작은 차량 박스는 화살표를 안 읽는다(모름 = 정지) [px]
        self.declare_parameter('tl_b_arrow_frac', 0.30)      # 셋째 칸 녹색 비율이 이 이상이면 화살표 켜짐(ARROW 근거)
        self.declare_parameter('tl_b_veh_min_ar', 1.75)      # 화살표를 읽을 박스의 최소 가로세로비 — 버스 박스(1.47~1.80)를 거른다
        self.declare_parameter('tl_bus_dark_t', BUS_BODY['dark_t'])
        self.declare_parameter('tl_bus_open_k', BUS_BODY['open_k'])
        self.declare_parameter('tl_bus_iso_below', BUS_BODY['iso_below'])
        self.declare_parameter('tl_bus_iso_left', BUS_BODY['iso_left'])
        # ① 정지선 폴리곤이 BEV 중심열 ± 이 거리[m]를 모두 덮어야 인정(길가 노면표시 거름).
        #    0.5 는 중앙선에 붙어 달릴 때 진짜 정지선을 버렸다. 0 = 끔.
        self.declare_parameter('sl_lane_half_m', 0.1)
        # ② '정지선 놓침 → 즉시 정지' 는 이 BEV 행까지 내려온 적이 있는 정지선만. 음수 = 끔.
        self.declare_parameter('sl_lost_min_bev_y', 40.0)

        # ── 카메라 기하 (어안 왜곡보정·BEV) ─────────────────────────────────
        #  ★파라미터 이름·기본값의 주인은 camera_model.py 다★ 차선 인지가 붙어도
        #  같은 이름을 쓰게 하려고 선언을 그쪽에 두었다(camera_launch.camera_params()
        #  한 벌을 두 노드에 그대로 먹일 수 있다).
        camera_model.declare_params(self)

        g = lambda k: self.get_parameter(k).value
        self.image_topic = str(g('image_topic'))
        self.device      = str(g('device'))
        self.tl_conf     = float(g('tl_conf'))
        self.tl_imgsz    = int(g('tl_imgsz'))
        self.tl_roi = (int(g('tl_roi_xmin')), int(g('tl_roi_ymin')),
                       int(g('tl_roi_xmax')), int(g('tl_roi_ymax')))
        self.tl_min_area   = int(g('tl_min_area'))
        self.tl_max_h_frac = max(0.0, float(g('tl_max_height_frac')))
        #  시험용 주입. /tl/fake_box_h 가 오면 그쪽이 이긴다(런 중에 바꾸기 위해서다).
        self.fake_box_h = max(0.0, float(g('tl_fake_box_h')))
        self.tl_zone_red_min_height = max(0, int(g('tl_zone_red_min_height')))
        self.tl_zone_stale_s   = max(0.1, float(g('tl_zone_stale_s')))
        self.bus_detect   = bool(g('tl_bus_detect'))
        self.bus_min_veh_h = float(g('tl_bus_min_veh_h'))
        self.bus_P = dict(BUS_BODY, dark_t=float(g('tl_bus_dark_t')), open_k=float(g('tl_bus_open_k')),
                          iso_below=float(g('tl_bus_iso_below')), iso_left=float(g('tl_bus_iso_left')))
        self.b_need_arrow  = bool(g('tl_b_need_arrow'))
        self.b_arrow_min_h = float(g('tl_b_arrow_min_h'))
        self.b_arrow_frac  = float(g('tl_b_arrow_frac'))
        self.b_veh_min_ar  = float(g('tl_b_veh_min_ar'))
        self.last_bus = []            # 이번 프레임 버스 신호 [{'box':원본좌표,'src':'model'|'cv','color':…}]
        self.last_veh = None          # 이번 프레임 #400 차량 박스(boxes 의 원소) — 구간 'B' 에서만
        self.last_arrow = None        # 그 박스의 화살표 판독 (셋째 칸 비율 | None=못 읽음, 켜짐?) — 구간 'B' 에서만
        self.arrow_lit_prev = None    # 로그용 — 화살표 판독이 바뀔 때만 한 줄
        self.last_dec = []            # 이번 프레임 ★판단에 쓴★ 박스 — 'B' 밖에서는 boxes 그대로
        self.sl_lane_half_m    = max(0.0, float(g('sl_lane_half_m')))
        self.sl_lost_min_bev_y = float(g('sl_lost_min_bev_y'))
        self.tl_min_aspect = float(g('tl_min_aspect'))
        self.tl_max_aspect = float(g('tl_max_aspect'))
        self.tl_red_stop_min_height    = int(g('tl_red_stop_min_height'))
        self.tl_near_release_ratio = min(1.0, max(0.1, float(g('tl_near_release_ratio'))))
        # 단독 문턱이 인지 게이트보다 작으면 'RED 확정 = 즉시 정지' 와 같아진다.
        self.tl_solo_stop_min_height = max(0.0, float(g('tl_solo_stop_min_height')))
        if 0.0 < self.tl_solo_stop_min_height < self._near_gate_base():
            self.get_logger().warn(
                f"tl_solo_stop_min_height({self.tl_solo_stop_min_height:.0f}) < "
                f"인지 게이트({self._near_gate_base():.0f}) — 단독 정지 문턱이 "
                "RED 인정 문턱보다 낮다. 사실상 'RED 확정 = 즉시 정지' 로 동작한다")
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
        self.red_release_hold_lost_s = max(self.red_release_hold_s, float(g('red_release_hold_lost_s')))
        self.tl_arm_hold_s = max(0.0, float(g('tl_arm_hold_s')))
        self.sl_enable  = bool(g('sl_enable'))
        self.sl_conf    = float(g('sl_conf'))
        self.sl_imgsz   = int(g('sl_imgsz'))
        # 발화선이 BEV 밖이면 영영 안 닿는다 — 조용히 두지 않고 bev_h 로 낮춘다.
        self.sl_trigger_bev_y = max(0.0, float(g('sl_trigger_bev_y')))
        _bev_h = float(int(g('bev_h')))     # camera_model.declare_params 가 선언한다
        if self.sl_trigger_bev_y > _bev_h:
            self.get_logger().warn(
                f"sl_trigger_bev_y({self.sl_trigger_bev_y:.0f}) > bev_h"
                f"({_bev_h:.0f}) — BEV 밖이라 영영 발화하지 않는다. bev_h 로 낮춘다")
            self.sl_trigger_bev_y = _bev_h
        self.sl_min_width_frac = float(g('sl_min_width_frac'))
        self.sl_min_area_frac  = float(g('sl_min_area_frac'))
        self.sl_hold_s   = max(0.0, float(g('sl_hold_s')))
        self.sl_stale_s  = max(0.05, float(g('sl_stale_s')))
        self.sl_gate_red_s = max(0.0, float(g('sl_gate_red_s')))
        self.brake_level         = max(0, min(2, int(g('brake_level'))))
        self.publish_cmd_vel  = bool(g('publish_cmd_vel'))
        self.stop_cmd_hz      = max(1.0, float(g('stop_cmd_hz')))
        self.stop_latch       = bool(g('stop_latch'))
        self.require_permission = bool(g('require_permission'))
        self.show_window = bool(g('show_window'))
        self.publish_debug = bool(g('tl_publish_debug'))
        self.draw_roi    = bool(g('draw_roi'))
        self.window_width = max(0, int(g('window_width')))
        self.show_bev    = bool(g('show_bev')) and (self.show_window or self.publish_debug)
        self.roi_dim     = min(1.0, max(0.0, float(g('roi_dim'))))
        self.roi_zoom    = bool(g('tl_roi_zoom'))
        # 캘리브 로드 실패는 경고로 끝나고 보정만 꺼진다(fail-open).
        self.cam = camera_model.CameraModel.from_node(self)
        # 창 표시는 메인 스레드가 한다(show_pending) — HighGUI 는 스레드 안전하지 않다.
        self._show_lock = threading.Lock()
        self._show_frame = None
        self._window_ready = False
        # 그리기 전용 스레드 — cb_image 는 재료만 넘긴다(창·녹화가 인지 루프에 0ms).
        #   그리기가 밀리면 최신 것만 그린다.
        self._draw_lock = threading.Lock()
        self._draw_job = None
        self._draw_evt = threading.Event()
        if self.show_window or self.publish_debug:
            threading.Thread(target=self._draw_loop, daemon=True).start()
        # ── 그리기 도구 — 전부 캐시다(매 프레임 새로 만드는 것이 없다) ──────────
        self.tr = TextRenderer(log=self.get_logger() if self.show_window else None,
                               font_path=str(g('hud_font')))
        self._pool = CanvasPool()          # 표시 캔버스 두 장(더블버퍼)
        self._hud_strip = StripCache(self.tr)   # 하단 HUD — 글자가 바뀔 때만 다시 그린다
        self._bev_m = None                 # 썸네일 크기로 바로 펴는 호모그래피
        self._bev_m_key = None

        # ── 상태 ───────────────────────────────────────────────────────────
        self.tl_state   = 'UNKNOWN'   # 마지막 프레임 판정
        self.tl_time    = 0.0         # 그 판정 시각(신선도 판단용)
        self.red_since  = None        # 연속 RED 스트릭의 시작 시각
        self.red_last_seen  = None    # 마지막으로 근접 RED 를 본 시각(RED_FAR 는 안 친다)
        self.green_since    = None
        self.green_last_seen = None
        self.stop_level = 0           # 0 또는 brake_level. 해제 경로에서만 0 으로 돌아간다
        self.stop_why   = ''          # 지금 단계를 고른 근거(로그·HUD 용)
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

        # ── 정지선 상태 ────────────────────────────────────────────────────
        self.sl_bev_y   = SL_NONE     # ★판정값★ BEV 에서 정지선 최근접점의 행(SL_NONE=없음)
        self.sl_px      = -1.0        # 정지선 → 앞범퍼 [px] (기록·HUD 전용). −1 = 미검출
        self.sl_y       = -1.0        # 마지막 관측(최하단 y / 프레임 높이). −1 = 미검출
        self.sl_seen_t  = 0.0         # 마지막으로 정지선을 본 시각 = 신선도의 근거
        self.sl_since   = None        # 연속 목격 스트릭의 시작 시각(확정 판정용)
        self.sl_engaged = False       # 이번 접근에서 정지선을 ★확정한 적★ 이 있는가
        self.sl_engaged_max_y = SL_NONE  # 확정한 정지선이 내려온 최대 BEV 행(sl_lost_min_bev_y)
        self.tl_zone   = ''           # driving 이 알려 준 지금 구간('T'/'A'/'')
        self.tl_zone_t = 0.0
        self.sl_wait    = False       # 지금 정지선 때문에 브레이크를 참고 있는가(아직 0단)
        self.sl_poly    = None        # HUD 용 마스크 폴리곤(보정된 원본 좌표)
        self.sl_poly_bev = None       # 같은 폴리곤의 BEV 좌표(HUD 용)
        self.red_seen_t = 0.0         # 마지막으로 ★빨간 박스★ 를 본 시각(추론 게이팅)
        self.red_conf_t = None        # 이번 접근에서 RED 를 확정한 시각(_red_armed·HUD). 해제 때 지운다

        # ── 모델 ───────────────────────────────────────────────────────────
        self.model = None
        self.TL_LABEL = {}
        self.TL_ALLOW = set()
        self._load_model(str(g('tl_weights')))
        self.sl_model = None
        self.sl_class_id = SL_CLASS_FALLBACK
        if self.sl_enable:
            self._load_sl_model(str(g('sl_weights')))

        # ── ROS 인터페이스 ─────────────────────────────────────────────────
        qos_img = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self.bridge    = CvBridge()
        self.pub_brake = self.create_publisher(Int32,  '/brake_level', qos)
        # 내 '요구' 를 따로 낸다 — master·driving 이 자기 값과 max 로 합쳐 /brake_level 을
        #   내므로 두 발행자가 같은 값을 내게 되어 서로 덮지 않는다.
        self.pub_req   = self.create_publisher(Int32,  '/tl_brake_req', qos)
        self.pub_cmd   = self.create_publisher(Twist,  '/cmd_vel_raw', qos)
        self.pub_state = self.create_publisher(String, '/tl/state',    qos)
        # [진단 2026-10-07] 판단에 쓴 박스 목록 — 테스트베드 채점용, 동작에는 쓰지 않는다.
        self.pub_boxes = self.create_publisher(String, '/tl/boxes',    qos)
        self.pub_debug_img = self.create_publisher(Image, '/tl/debug_image', qos_img)
        # 아래는 기록 전용(record 가 CSV 로 적는다) — 제어에는 쓰지 않는다.
        self.pub_near    = self.create_publisher(Float32, '/tl/near_metric', qos)
        self.pub_red_far = self.create_publisher(Bool,    '/tl/red_far',     qos)
        self.pub_sl_bev_y = self.create_publisher(Float32, '/tl/stop_line_bev_y', qos)
        self.pub_sl_y    = self.create_publisher(Float32, '/tl/stop_line_y',    qos)
        self.pub_sl_wait = self.create_publisher(Bool,    '/tl/stop_line_wait', qos)
        self.pub_sl_px   = self.create_publisher(Float32, '/tl/stop_line_px',   qos)

        # 영상과 제어를 다른 콜백 그룹에 둔다 — 추론이 길어져도 제어 틱이 밀리지 않는다.
        self.cg_image = MutuallyExclusiveCallbackGroup()
        self.cg_ctrl  = MutuallyExclusiveCallbackGroup()

        self.create_subscription(Image, self.image_topic, self.cb_image, qos_img,
                                 callback_group=self.cg_image)
        self.create_subscription(String, '/drive_state', self.cb_drive_state, qos,
                                 callback_group=self.cg_ctrl)
        # 허락 두 갈래(헤더). 안 오면 허락 없음.
        self.create_subscription(Bool, '/tl_enable', self.cb_tl_enable, qos,
                                 callback_group=self.cg_ctrl)
        self.create_subscription(Bool, '/tl_permit', self.cb_tl_permit, qos,
                                 callback_group=self.cg_ctrl)
        # 시험용 주입을 런 도중에 바꾸는 통로(실차에서는 아무도 안 낸다).
        self.create_subscription(Float32, '/tl/fake_box_h', self.cb_fake_box_h, qos,
                                 callback_group=self.cg_ctrl)
        self.create_subscription(String, '/tl_zone', self.cb_tl_zone, qos,
                                 callback_group=self.cg_ctrl)

        # 판단·발행은 영상 콜백이 아니라 타이머에서 — 프레임이 늦어도 주기가 흔들리지 않게.
        self.create_timer(1.0 / self.stop_cmd_hz, self.tick,
                          callback_group=self.cg_ctrl)
        self.create_timer(1.0, self.status_tick, callback_group=self.cg_ctrl)

        self.get_logger().info(
            f"🚦 traffic_light | img={self.image_topic} dev={self.device} "
            f"conf={self.tl_conf} ROI={self.tl_roi} | 근접도 게이트="
            f"박스높이≥{self.tl_red_stop_min_height}px"
            f"(물고 있는 동안 ×{self.tl_near_release_ratio:.2f})"
            + f" | 확정 hold={self.tl_hold_s}s grace={self.tl_gap_grace_s}s "
            f"age≤{self.tl_state_max_age}s | 정지=리니어 {self.brake_level}단"
            + (f" + /cmd_vel_raw 0/0 @{self.stop_cmd_hz:.0f}Hz"
               if self.publish_cmd_vel else " (조향 개입 없음)")
            + f" | latch={'ON(GREEN 까지)' if self.stop_latch else 'OFF(빨간불 동안만)'} "
            + ("허락=/tl_enable(master 체크박스) 또는 /tl_permit(driving DRIVE_RUN)"
               if self.require_permission else "허락=★없음(벤치 모드)★")
            + (f"\n   🛑 정지선: ★BEV 발화선★ y≥{self.sl_trigger_bev_y:.0f} → "
               f"{self.brake_level}단(풀브레이크) | "
               f"conf={self.sl_conf} "
               f"확정 {self.sl_hold_s}s / 신선도 {self.sl_stale_s}s "
               f"| ★대기 상한 없음★ — 정지선이 보이는 동안은 발화선까지 기다린다"
               if (self.sl_enable and self.sl_model is not None)
               else "\n   🛑 정지선: ★꺼져 있다★ — 신호등 단독 문턱만 남는다")
            + (f"\n   🚦 신호등 단독 정지: ★정지선을 못 볼 때만★ 박스높이 ≥"
               f"{self.tl_solo_stop_min_height:.0f}px → {self.brake_level}단"
               if self.tl_solo_stop_min_height > 0.0
               else "\n   🚦 신호등 단독 정지: ★꺼져 있다★ — 정지선을 못 보면 안 선다")
            + ("\n   🚌 구간 'B'(#400): 버스 신호를 판단에서 뺀다 · "
               + (f"★좌회전 화살표만 진행★ (차량 박스 ≥{self.b_arrow_min_h:.0f}px · "
                  f"셋째 칸 녹색 ≥{self.b_arrow_frac:.2f} — 못 읽으면 정지)"
                  if self.b_need_arrow else "원등에도 진행(화살표를 읽지 않는다)")
               if self.bus_detect else "\n   🚌 구간 'B': ★꺼져 있다★ — 'T' 와 같다")
            + "\n   " + self.cam.describe()
            + (f"\n   🧪 ★신호등 주입 모드★ 박스높이 {self.fake_box_h:.0f}px 를 봤다고 "
               f"친다 — ★신호등 인지는 시험되지 않는다★ (실차 금지)"
               if self.fake_box_h > 0.0 else ""))

    def _load_model(self, weights):
        """가중치를 읽고 라벨 표를 세운다. 실패해도 노드는 살아 있는다(fail-open).

        ★라벨은 엔진의 클래스 이름에서 유도한다★ 하드코딩({0:GREEN, 1:RED})은
        가중치를 재학습·교체하는 순간 ★조용히 적/녹이 뒤집힌다★ — 그 실패 모드는
        '빨간불에 가속'이다. 아래 로그가 실제 로드된 클래스를 찍으므로 기동할 때
        반드시 눈으로 확인할 것.
        """
        # 1280 엔진이 없으면 640 엔진으로 — 기본 경로일 때만(직접 준 경로는 그 뜻대로).
        if weights == TL_WEIGHTS and not os.path.exists(weights) \
                and os.path.exists(TL_WEIGHTS_FALLBACK):
            self.get_logger().warn(
                f"⚠️ 1280 신호등 엔진이 없다: {weights} → 종전 640 엔진으로 돈다 "
                f"({TL_WEIGHTS_FALLBACK}). 원거리 신호 검출률이 절반이다 — "
                f"tools/export_tl_1280.sh 를 이 기계에서 한 번 돌릴 것")
            weights = TL_WEIGHTS_FALLBACK
        # 정적 엔진에 다른 크기를 주면 매 프레임 예외 → 박스 0 개로 조용히 신호등이 꺼진다.
        #   엔진 쪽 크기로 맞춘다.
        eng = self._engine_imgsz(weights)
        if eng and eng != self.tl_imgsz:
            self.get_logger().warn(
                f"⚠️ tl_imgsz {self.tl_imgsz} ≠ 엔진 고정 입력 {eng} ({weights}) "
                f"→ {eng} 로 맞춘다")
            self.tl_imgsz = eng
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

    @staticmethod
    def _engine_imgsz(path):
        """ultralytics 가 .engine 머리에 붙인 메타데이터(4바이트 길이 + JSON)에서 고정
        입력 크기를 읽는다. .engine 이 아니거나 못 읽으면 None(검사 생략)."""
        if not str(path).endswith('.engine') or not os.path.exists(path):
            return None
        try:
            with open(path, 'rb') as f:
                n = int.from_bytes(f.read(4), 'little')
                meta = json.loads(f.read(n).decode('utf-8'))
            sz = meta.get('imgsz')
            return int(sz[0] if isinstance(sz, (list, tuple)) else sz)
        except Exception:
            return None

    def _load_sl_model(self, weights):
        """정지선 seg 가중치를 읽는다. 실패하면 정지선 없이(신호등 단독 문턱만으로) 돈다.
        클래스는 이름에서 찾는다 — 하드코딩은 가중치를 바꾸면 엉뚱한 것 앞에서 서게 한다."""
        try:
            self.sl_model = YOLO(weights, task='segment')
        except Exception as e:
            self.sl_model = None
            self.get_logger().error(
                f"⛔ 정지선 가중치 로드 실패 — 정지선 없이 신호등 단독 문턱으로만 선다: "
                f"{weights} ({e})")
            return

        names = {}
        try:
            n = self.sl_model.names
            names = {int(k): str(v) for k, v in
                     (n.items() if isinstance(n, dict) else enumerate(n))}
        except Exception as e:
            self.get_logger().error(f"정지선 엔진 클래스 이름 조회 실패: {e}")

        found = [k for k, v in names.items() if 'STOP' in v.upper()]
        if found:
            self.sl_class_id = found[0]
        else:
            self.sl_class_id = SL_CLASS_FALLBACK
            self.get_logger().error(
                f"⛔ 정지선 엔진 클래스에 'stop' 이 없다: {names} → "
                f"id={SL_CLASS_FALLBACK} 로 폴백한다. ★다른 클래스를 정지선으로 읽으면 "
                f"엉뚱한 곳에서 선다 — 반드시 확인할 것★")
        self.get_logger().info(
            f"정지선 엔진 클래스 = {names} (사용 id={self.sl_class_id})")

    # ══════════════════════════════════════════════════════════════════════════
    #  인지
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

    def _all_tl_boxes(self, result, roi_img):
        """YOLO 결과 → 필터링·색보정된 신호등 박스 목록."""
        out = []
        self.last_red_drop = 0
        if result is None or result.boxes is None or len(result.boxes) == 0:
            return out
        # ROI 를 통째로 덮는 박스를 거르는 자 — ROI 높이 기준이라 ROI 를 바꿔도 산다.
        max_bh = (roi_img.shape[0] * self.tl_max_h_frac) if self.tl_max_h_frac > 0 else 0
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
            if area   < self.tl_min_area:                                  continue
            if aspect < self.tl_min_aspect or aspect > self.tl_max_aspect: continue
            if max_bh and bh > max_bh:                                     continue

            label = self.TL_LABEL.get(cls_id, str(cls_id))
            # HSV 는 항상 돌리되 라벨을 뒤집는 조건은 방향에 따라 다르다(오류 비용이 비대칭).
            crop = roi_img[max(0, y1):max(0, y2), max(0, x1):max(0, x2)]
            hsv_label, hsv_red, hsv_green = self._hsv_state(crop)
            if label == 'GREEN' and hsv_label == 'RED':
                label = 'RED'                     # 안전 방향 → 항상 교정
            elif conf < HSV_FALLBACK_CONF and hsv_label in ('RED', 'GREEN'):
                label = hsv_label                 # 위험 방향 → 저신뢰에서만 위임

            # RED 오탐 관문 — 붉지 않은 것이 RED 로 나가면 도로 한복판에서 선다.
            if label == 'RED' and hsv_red < self.hsv_min_color_pixels:
                self.last_red_drop += 1
                continue

            out.append({
                'label': label, 'conf': conf, 'box': (x1, y1, x2, y2), 'box_h': bh,
                # 모델 자신의 판정(HSV 교정 전). 좌회전 구간에서 '빨강+화살표' 를 가른다.
                'cls_label': self.TL_LABEL.get(cls_id, str(cls_id)),
                'hsv_red': hsv_red, 'hsv_green': hsv_green,
            })
        return out

    def _b_active(self):
        """구간 'B' 처리(버스 찾기·버스 빼기·화살표)를 지금 하는가 — 'B' 가 신선하고 tl_bus_detect 가 켜져 있다."""
        return self.bus_detect and self._zone_active() and self.tl_zone == 'B'

    def _find_bus(self, frame, boxes, xmin, ymin):
        """구간 'B' 에서만 — #400 차량 박스(기준)와 그 왼쪽의 버스 신호를 찾는다.
        기준 = ★남의 버스 자리에 있지 않은★ 박스 중 가장 큰 것 — 크기와 무관하다(판단도 이것 하나를 본다 → last_veh).
        그 버스 자리에 모델 박스가 있으면 그것이 버스(판단에서 뺀다 — _decision_boxes), 없고 기준이
        tl_bus_min_veh_h 이상이면 몸체를 영상처리로 찾는다(≈1 ms, 표시용 — 모델 박스가 아니라 판단에는 원래 없다)."""
        self.last_bus = []
        self.last_veh = None
        if not self._b_active() or not boxes:
            return []
        g = [(b, (b['box'][0] + xmin, b['box'][1] + ymin, b['box'][2] + xmin, b['box'][3] + ymin))
             for b in boxes]
        veh = None
        for b, gb in sorted(g, key=lambda t: -t[0].get('box_h', 0)):
            if not any(o is not b and bus_slot(gb, go) for o, go in g):
                self.last_veh, veh = b, gb
                break
        if veh is None:
            return []
        for b, gb in g:
            if b is not self.last_veh and bus_slot(gb, veh):
                self.last_bus = [{'box': gb, 'src': 'model', 'color': b['label'], '_ref': b}]
                return self.last_bus
        if self.last_veh.get('box_h', 0) < self.bus_min_veh_h:
            return []
        try:
            body = find_bus_body(frame, veh, (
                ((self.hsv_red_h1_low, self.hsv_sat_low, self.hsv_val_low), (self.hsv_red_h1_high, 255, 255)),
                ((self.hsv_red_h2_low, self.hsv_sat_low, self.hsv_val_low), (self.hsv_red_h2_high, 255, 255)),
                ((self.hsv_green_h_low, self.hsv_sat_low, self.hsv_val_low), (self.hsv_green_h_high, 255, 255))),
                self.bus_P)
        except Exception as e:                       # 표시용이다 — 실패해도 판단 경로는 그대로
            self.get_logger().error(f"버스 몸체 찾기 실패: {e}", throttle_duration_sec=5.0)
            return []
        if body is None:
            return []
        color = self._hsv_state(frame[body[1]:body[3], body[0]:body[2]])[0]
        self.last_bus = [{'box': body, 'src': 'cv', 'color': color}]
        return self.last_bus

    def _decision_boxes(self, frame, boxes, bus, xmin, ymin):
        """★판단에 쓸 박스★ — 구간 'B' 밖(그리고 주입 중)이면 boxes 그대로다(종전과 한 글자도 같은 판단).
        'B' 에서는 ① 버스 신호(모델 박스)를 뺀다 ② tl_b_need_arrow 면 차량 박스(last_veh) 하나만 남겨
        좌회전 화살표로 읽는다 — 켜짐 = GREEN, 그 밖(원등만·빨강·황색·작아서 못 읽음·버스 모양) = RED.
        RED 는 높이로 RED/RED_FAR 가 갈리고(_resolve_tl_state) 근접 높이·정지선 추론도 이 박스를 본다.
        ★원본 박스는 건드리지 않고 사본을 낸다★ — 그림과 /tl/boxes 의 라벨은 모델·HSV 가 본 그대로다."""
        self.last_arrow = None
        if not self._b_active() or self.fake_box_h > 0.0:
            return boxes
        drop = {id(b['_ref']) for b in bus if b['src'] == 'model'}
        rest = [b for b in boxes if id(b) not in drop]
        veh = self.last_veh
        if not self.b_need_arrow or veh is None:
            return rest
        x1, y1, x2, y2 = veh['box']
        bh = veh.get('box_h', y2 - y1)
        frac = None
        if bh >= self.b_arrow_min_h and (x2 - x1) / max(1.0, bh) >= self.b_veh_min_ar:
            frac = arrow_frac(frame, (x1 + xmin, y1 + ymin, x2 + xmin, y2 + ymin))
        lit = frac is not None and frac >= self.b_arrow_frac
        self.last_arrow = (frac, lit)
        if frac is not None and lit != self.arrow_lit_prev:
            self.get_logger().info(
                f"{'⬅️🟢 좌회전 화살표 켜짐 — 진행' if lit else '⬅️⚫ 좌회전 화살표 없음 — 정지 신호'} "
                f"(#400 차량 박스 {bh}px · 셋째 칸 녹색 {frac:.2f}/{self.b_arrow_frac:.2f} · 모델 {veh['label']})")
            self.arrow_lit_prev = lit
        return [dict(veh, label='GREEN' if lit else 'RED', b_arrow=frac)]

    def _near_metric(self, boxes):
        """빨간 박스 높이의 최대 [px]. 없으면 0. 단독 문턱 비교와 기록에 쓴다."""
        red = [b for b in boxes if b['label'] == 'RED']
        if not red:
            return 0.0
        return float(max(b.get('box_h', 0) for b in red))

    def _near_gate_base(self):
        """히스테리시스를 뺀 근접 게이트[px] — 게이지의 자도 여기에 맞춘다(무는 순간 안 튀게)."""
        if self._zone_active():
            return float(self.tl_zone_red_min_height)     # 신호 구간 안에서만 낮춘다
        return float(self.tl_red_stop_min_height)

    def _zone_active(self):
        """driving 이 '지금 신호 정지 구간(T/A)' 이라고 ★신선하게★ 알려 주고 있는가.
        __init__ 의 경고 검사가 상태 변수보다 먼저 부르므로 getattr 로 받는다."""
        if getattr(self, 'tl_zone_red_min_height', 0) <= 0:
            return False
        if getattr(self, 'tl_zone', '') not in ('T', 'A', 'B'):
            return False
        return (time.time() - self.tl_zone_t) <= self.tl_zone_stale_s

    def _near_gate(self):
        """지금 '가깝다' 로 인정할 높이[px] — 물고 있는 동안은 낮춘다(히스테리시스)."""
        base = self._near_gate_base()
        return base * self.tl_near_release_ratio if self.stopping else base

    def _resolve_tl_state(self, boxes):
        if not boxes:
            return 'UNKNOWN'
        red_boxes = [b for b in boxes if b['label'] == 'RED']
        if red_boxes and self._zone_active() and self.tl_zone == 'A':
            # ★좌회전 구간★ 모델이 GREEN 이라 했는데 HSV 가 붉다고 교정한 등기구 =
            #   빨강 + 좌회전 화살표(2026-09-13 144031 실측: 모델이 101/103 GREEN).
            #   좌회전하는 차에게 그건 진행 신호다. 모델도 RED 라 한 것(순수 빨강)은 그대로 선다.
            red_boxes = [b for b in red_boxes if b.get('cls_label') != 'GREEN']
            if not red_boxes:
                return 'GREEN'
        if red_boxes:
            gate = self._near_gate()
            near = any(b.get('box_h', 0) >= gate for b in red_boxes)
            return 'RED' if near else 'RED_FAR'
        if any(b['label'] == 'GREEN' for b in boxes):
            return 'GREEN'
        return 'UNKNOWN'

    # ══════════════════════════════════════════════════════════════════════════
    #  인지 — 정지선
    # ══════════════════════════════════════════════════════════════════════════
    def _detect_stop_line(self, frame):
        """가장 가까운 정지선(BEV 행이 가장 큰 것)을 찾는다.

        돌려주는 것 : (y_bev, dist_px, y_frac, poly, poly_bev) — 못 찾으면 (None, None, −1, None, None)
          · y_bev    ★판정값★ BEV 최근접점의 행. 클수록 가깝다
          · dist_px  범퍼까지 픽셀 거리 · y_frac 원본 최하단 y / 높이 (둘 다 기록·HUD 용)
          · poly / poly_bev  보정된 원본 / BEV 좌표의 마스크 폴리곤 (HUD)
        프레임 전체로 추론한다 — 이 모델은 전체 화면으로 학습돼 잘라 넣으면 나빠진다.
        """
        none = (None, None, -1.0, None, None)
        try:
            res = self.sl_model.predict(source=frame, conf=self.sl_conf,
                                        imgsz=self.sl_imgsz, device=self.device,
                                        verbose=False)[0]
        except Exception as e:
            self.get_logger().error(f"YOLO stop-line error: {e}", throttle_duration_sec=5.0)
            return none
        if res.masks is None or res.boxes is None:
            return none

        h, w = frame.shape[:2]
        best = none
        for i, pts in enumerate(res.masks.xy):
            if i >= len(res.boxes):
                break
            try:
                cls_id = int(res.boxes[i].cls[0].item())
            except Exception:
                continue
            if cls_id != self.sl_class_id:
                continue
            pts = np.asarray(pts, dtype=np.float32)
            if len(pts) < 3:
                continue
            xs, ys = pts[:, 0], pts[:, 1]
            # ★정지선은 가로로 긴 물체다★ 폭·면적이 안 되면 노면 잡티·차선 조각이다.
            if (float(xs.max() - xs.min()) / w) < self.sl_min_width_frac:
                continue
            if (abs(cv2.contourArea(pts)) / float(w * h)) < self.sl_min_area_frac:
                continue
            y_bev, dist, poly_bev = self.cam.nearest_bev_y(pts)
            if y_bev is None:
                # 폴리곤이 통째로 소실선 너머다 = BEV 좌표가 무의미하다. 버린다.
                continue
            if self.sl_lane_half_m > 0.0 and not self._sl_in_lane(pts):
                # ① 차가 가는 자리를 가로지르지 않는 선 — 길가·옆 차로 노면표시다.
                continue
            # 가장 가까운 것 = BEV 행이 가장 큰 것.
            if best[0] is None or y_bev > best[0]:
                best = (y_bev, dist, float(ys.max()) / float(h), pts, poly_bev)
        return best

    def _sl_in_lane(self, pts):
        """① 정지선 폴리곤이 BEV 중심열(차량 중심선) ± sl_lane_half_m 를 모두 덮는가.
        BEV 사다리꼴은 차 중심에 대해 좌우 대칭이라 BEV 가운데 열이 곧 차가 가는 자리다.
        px→m 을 모르면(0) 판정할 근거가 없으므로 통과시킨다."""
        if self.cam.px_to_m <= 0.0:
            return True
        bev, ok = self.cam.poly_to_bev(pts)
        if not np.any(ok):
            return False
        xs = bev[ok, 0]
        cx = self.cam.bev_w / 2.0
        half = self.sl_lane_half_m / self.cam.px_to_m
        return float(xs.min()) <= cx - half and float(xs.max()) >= cx + half

    def _sl_should_run(self, now):
        """정지선 추론은 빨간 박스가 보이는 동안과 서 있는 동안(HUD)만 돌린다."""
        if not self.sl_enable or self.sl_model is None:
            return False
        if self.stopping:
            return True
        return self.red_seen_t > 0.0 and (now - self.red_seen_t) <= self.sl_gate_red_s

    def _update_stop_line(self, frame, boxes):
        """프레임 하나로 정지선 상태를 갱신한다(스트릭·신선도는 신호등과 같은 방식)."""
        now = time.time()
        if any(b['label'] == 'RED' for b in boxes):
            # RED_FAR 도 라벨은 RED 다 — 멀리서 보이기 시작하면 정지선을 찾기 시작한다.
            self.red_seen_t = now

        if not self._sl_should_run(now):
            return

        y_bev, dist, y, poly, poly_bev = self._detect_stop_line(frame)
        if y_bev is not None:
            if self.sl_since is None:
                self.sl_since = now
            self.sl_seen_t = now
            self.sl_bev_y = y_bev
            self.sl_px = dist
            self.sl_y = y
            self.sl_poly = poly
            self.sl_poly_bev = poly_bev
        elif (now - self.sl_seen_t) > self.sl_stale_s:
            # 신선도가 끊긴 뒤에야 지운다('놓쳤다' 의 판정은 _stop_plan 이 한다).
            self.sl_since = None
            self.sl_bev_y = SL_NONE
            self.sl_px = -1.0
            self.sl_y = -1.0
            self.sl_poly = None
            self.sl_poly_bev = None

    def cb_image(self, msg: Image):
        if self.model is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}", throttle_duration_sec=5.0)
            return

        # 어안 보정 — 이 아래는 전부 보정된 그림을 본다(못 읽었으면 원본 그대로).
        #   /image_raw 자체는 건드리지 않는다(원본 녹화가 카메라 그림이어야 해서).
        frame = self.cam.undistort(frame)

        self.frame_count += 1
        h, w = frame.shape[:2]
        xmin, ymin, xmax, ymax = self._clamp_roi(w, h, *self.tl_roi)
        roi_img = frame[ymin:ymax, xmin:xmax]

        boxes = []
        if roi_img.size != 0:
            try:
                res = self.model.predict(source=roi_img.copy(), conf=self.tl_conf,
                                         imgsz=self.tl_imgsz, device=self.device,
                                         verbose=False)[0]
                boxes = self._all_tl_boxes(res, roi_img)
                self.last_boxes = boxes
                # "YOLO 가 못 봤다" vs "필터가 먹었다" 를 사후에 가르기 위한 값.
                self.last_raw = 0 if res.boxes is None else len(res.boxes)
            except Exception as e:
                self.get_logger().error(f"YOLO TL error: {e}", throttle_duration_sec=5.0)
                self.last_boxes = []
                self.last_raw = -1
                self.last_red_drop = 0

        # 시험용 주입 — 추론은 그대로 돌리고(지연 통계가 실제와 같게) 결론만 바꾼다.
        if self.fake_box_h > 0.0:
            boxes = [self._fake_red_box(self.fake_box_h)]
            self.last_boxes = boxes

        # 구간 'B' — 버스·차량 박스를 가르고(_find_bus), 판단에 쓸 박스를 따로 만든다(_decision_boxes).
        #   'B' 밖에서는 dec 가 boxes 그 자체라 판단이 종전과 같다.
        bus = self._find_bus(frame, boxes, xmin, ymin)
        dec = self._decision_boxes(frame, boxes, bus, xmin, ymin)
        self.last_dec = dec
        # [진단] 이 프레임 박스(원본 좌표). ★/tl/state 보다 먼저★ — 테스트베드는 그것을
        #   동기 신호로 다음 프레임을 민다. 주입 박스(fake)는 좌표가 HUD 용이라 그대로 낸다.
        #   8번째 칸 = 역할('' | 'BUS' | 'VEH'), 9번째 = 화살표 판독(셋째 칸 녹색 비율, 안 읽었으면 -1).
        #   'BUS'·'VEH' 는 구간 'B' 에서만 붙는다 — BUS 는 판단에서 빠지고 VEH 하나가 판단한다.
        bus_model = {id(b['_ref']) for b in bus if b['src'] == 'model'}
        veh, arrow = self.last_veh, self.last_arrow
        self.pub_boxes.publish(String(data=json.dumps(
            [[b['box'][0] + xmin, b['box'][1] + ymin, b['box'][2] + xmin, b['box'][3] + ymin,
              b['label'], round(float(b['conf']), 3), b.get('cls_label', ''),
              'BUS' if id(b) in bus_model else ('VEH' if b is veh else ''),
              round(arrow[0], 3) if (b is veh and arrow and arrow[0] is not None) else -1] for b in boxes]
            + [[*b['box'], b['color'], 0.0, 'cv', 'BUS', -1] for b in bus if b['src'] == 'cv'],
            separators=(',', ':'))))
        state = self._resolve_tl_state(dec)
        self.pub_state.publish(String(data=state))
        self.pub_near.publish(Float32(data=float(self._near_metric(dec))))
        self.pub_red_far.publish(Bool(data=(state == 'RED_FAR')))
        self._feed_state(state)
        # 정지선은 신호등 판정 뒤에 — 빨간 박스를 봤는지가 추론 조건이라 순서가 바뀌면 한 프레임 늦는다.
        self._update_stop_line(frame, dec)
        self.img_time = time.time()

        now = time.monotonic()
        inst = 1.0 / max(1e-6, now - self.fps_t)
        self.fps = 0.9 * self.fps + 0.1 * inst if self.fps > 0 else inst
        self.fps_t = now

        if self.show_window or self.publish_debug:
            with self._draw_lock:       # 그리기 스레드에 넘기기만 한다(_draw_loop)
                self._draw_job = (frame, boxes, state, (xmin, ymin, xmax, ymax), bus, dec, veh, arrow)
            self._draw_evt.set()

    def _draw_loop(self):
        """그리기 전용 스레드 — 밀린 재료는 버리고 최신 것만 그린다."""
        while rclpy.ok():
            if not self._draw_evt.wait(0.5):
                continue
            self._draw_evt.clear()
            with self._draw_lock:
                job, self._draw_job = self._draw_job, None
            if job is None:
                continue
            try:
                self._draw(*job)
            except Exception as e:
                self.get_logger().error(f"디버그 그리기 실패: {e}",
                                         throttle_duration_sec=5.0)

    # ══════════════════════════════════════════════════════════════════════════
    #  표시 — 계기판 (뷰 + 우측 BEV 패널 show_bev + ROI 확대 열 tl_roi_zoom + 하단 HUD 2줄)
    #  [2026-10-08 오후] 그림은 그대로, 글자는 판단에 필요한 것만 — 고정 숫자(파라미터 요약·ROI 크기·'BEV'·'BUMPER')를 지웠고,
    #  패널이 있으면 HUD 는 패널 게이지와 겹치는 숫자(근접·정지선)를 싣지 않는다.
    # ══════════════════════════════════════════════════════════════════════════
    #  먼저 표시 크기로 줄이고 그 좌표계에서 그린다(글자가 창 크기에 맞고 싸다).
    #  글자는 그림 위가 아니라 하단 띠·우측 패널에 둔다. 대부분의 프레임 2.4ms.

    #  하단 HUD 의 글자 크기·줄 수. 열 좌표는 이 크기의 '칸' 단위로 적는다.
    HUD_SIZE  = 14
    HUD_LINES = 2
    #  우측 패널이 뷰 폭에서 차지하는 비율. BEV 썸네일(4:3)이 이 폭을 채운다.
    PANEL_RATIO = 0.30
    #  ROI 확대 열 폭과 그 아래 박스 확대 타일 개수.
    ZOOM_W = 480
    ZOOM_TILES = 4

    def _draw(self, frame, boxes, state, roi, bus=(), dec=None, veh=None, arrow=None):
        """dec = 판단에 쓴 박스(구간 'B' 면 차량 박스 사본 하나) · veh = 'B' 의 차량 박스 · arrow = (비율|None, 켜짐)."""
        dec = boxes if dec is None else dec
        now = time.time()
        sl_live = self.sl_enable and self.sl_model is not None
        sl_fresh = self._sl_present(now)

        # ── 뷰 — ★먼저 줄인다★ ──────────────────────────────────────────
        h0, w0 = frame.shape[:2]
        vw = w0 if self.window_width <= 0 else min(self.window_width, w0)
        sc = vw / float(w0)
        vh = int(round(h0 * sc))
        pw = int(round(vw * self.PANEL_RATIO)) if (self.show_bev and sl_live) else 0
        zw = self.ZOOM_W if self.roi_zoom else 0
        hh = self.HUD_LINES * self.tr.line_h(self.HUD_SIZE) + 10
        canvas = self._pool.get(vw + pw + zw, vh + hh)
        view = canvas[0:vh, 0:vw]
        #  정확히 절반일 때만 INTER_AREA(0.28ms) — 임의 배율이면 2.5ms 라 LINEAR.
        cv2.resize(frame, (vw, vh), dst=view,
                   interpolation=(cv2.INTER_AREA if abs(sc - 0.5) < 1e-6
                                  else cv2.INTER_LINEAR))

        # ── ROI — 밖을 어둡게(정지선은 ROI 밖 노면에 있으니 너무 어둡게 하지 않는다) ──
        rect = tuple(int(round(v * sc)) for v in roi)
        if self.draw_roi:
            dim_outside(view, rect, self.roi_dim)
            cv2.rectangle(view, (rect[0], rect[1]),
                          (max(0, rect[2] - 1), max(0, rect[3] - 1)), (120, 120, 124), 1)

        # ── BEV 사다리꼴·정지선 (사다리꼴을 노면에 맞추는 것이 STOPLINE_TEST 단계 2) ──
        if sl_live:
            cv2.polylines(view, [(self.cam.src_pts * sc).astype(np.int32)], True,
                          C_CYAN, 1)
            if self.sl_poly is not None and sl_fresh:
                poly = (self.sl_poly * sc).astype(np.int32)
                cv2.polylines(view, [poly], True, C_MAGENTA, 2)
                bi = int(np.argmax(poly[:, 1]))
                cv2.circle(view, (int(poly[bi, 0]), int(poly[bi, 1])), 4,
                           C_MAGENTA, -1)

        # ── 박스 [2026-10-08] — 판단에 쓴 박스만 굵게. 칩은 '색 conf 높이' 만.
        #   구간 'B' : 차량 박스(VEH)는 ★판단 색★(화살표 켜짐 = 녹, 그 밖 = 적), 버스는 회색 얇게 'skip'.
        bus_ids = {id(b['_ref']) for b in bus if b['src'] == 'model'}
        dec_ids = {id(b) for b in dec}
        col_of = lambda lab: C_RED if lab == 'RED' else C_GREEN if lab == 'GREEN' else C_GRAY
        for it in boxes:
            x1, y1, x2, y2 = it['box']
            bx1, by1 = int(round((x1 + roi[0]) * sc)), int(round((y1 + roi[1]) * sc))
            bx2, by2 = int(round((x2 + roi[0]) * sc)), int(round((y2 + roi[1]) * sc))
            ty = (by1 - 16) if by1 > 17 else by2
            if id(it) in bus_ids:
                cv2.rectangle(view, (bx1, by1), (bx2, by2), C_GRAY, 1)
                chip(view, bx1, by2 + 2, f"BUS {it['label'][0]} skip", C_GRAY)   # 아래 — 차량 칩을 덮지 않게
            elif it is veh and len(dec) == 1 and 'b_arrow' in dec[0]:
                col = col_of(dec[0]['label'])
                cv2.rectangle(view, (bx1, by1), (bx2, by2), col, 3)
                a = (f" <{arrow[0]:.2f}" if (arrow and arrow[0] is not None) else " <--")
                chip(view, bx1, ty, f"VEH {dec[0]['label'][0]} {it.get('box_h', 0)}px{a}", col)
            else:
                col = col_of(it['label'])
                cv2.rectangle(view, (bx1, by1), (bx2, by2), col, 2 if id(it) in dec_ids else 1)
                chip(view, bx1, ty, f"{it['label'][0]} {it['conf']:.2f} {it.get('box_h', 0)}px", col)
        for it in bus:                                    # 영상처리로 찾은 버스 몸체
            if it['src'] != 'cv':
                continue
            bx1, by1, bx2, by2 = (int(round(v * sc)) for v in it['box'])
            cv2.rectangle(view, (bx1, by1), (bx2, by2), C_GRAY, 1)
            chip(view, bx1, by2 + 2, f"BUS {it['color'][0]} cv skip", C_GRAY)

        # ── 물고 있으면 빨간 테두리 — 곁눈으로도 보이게 ────────────────
        if self.stopping:
            cv2.rectangle(view, (0, 0), (vw - 1, vh - 1), C_RED, 6)

        if pw:
            self._draw_panel(canvas, frame, vw, vh, pw, sl_fresh, dec)
        if zw:
            self._draw_zoom(canvas, frame, boxes, roi, vw + pw, zw, vh, bus_ids, dec, veh, arrow)
        self._draw_hud(canvas, vw + pw + zw, vh, hh, state, dec, now, sl_live, sl_fresh, arrow,
                       numbers=not pw)

        # imshow 는 여기서 부르지 않는다 — 워커 스레드에서 HighGUI 를 부르면 그 스레드가
        #   GTK 안에서 멎는다. 캔버스만 넘기고 표시는 메인 스레드가 한다(창이 켜졌을 때만).
        if self.show_window:
            with self._show_lock:
                self._show_frame = canvas

        if self.publish_debug:
            try:
                msg = self.bridge.cv2_to_imgmsg(canvas, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'traffic_light'
                self.pub_debug_img.publish(msg)
            except Exception as e:
                self.get_logger().error(f"디버그 이미지 발행 실패: {e}",
                                         throttle_duration_sec=5.0)

    def _draw_panel(self, canvas, frame, vw, vh, pw, sl_fresh, dec):
        """우측 패널 — BEV 썸네일(발화선·범퍼선) + 게이지 두 개(근접 = ★판단에 쓴 박스★ dec 기준).
        발화선은 BEV 행이라 원본 화면만으로는 어디인지 알 수 없어 썸네일에 그린다."""
        px, tw = vw, pw - 12
        th = int(round(tw * self.cam.bev_h / float(self.cam.bev_w)))
        thumb = canvas[6:6 + th, px + 6:px + 6 + tw]
        # 오버레이 없는 원본에서 썸네일 크기로 바로 편다(0.9ms).
        if self._bev_m_key != (tw, th):
            self._bev_m_key = (tw, th)
            self._bev_m = self.cam.bev_matrix(tw, th)
        cv2.warpPerspective(frame, self._bev_m, (tw, th), dst=thumb)

        k = tw / float(self.cam.bev_w)
        #  범퍼선은 기록값 sl_px 의 원점이라 선만 남긴다(판정에는 안 쓴다).
        for row, col, lab in ((self.cam.bumper_y, C_GREEN, ''),
                              (self.sl_trigger_bev_y, C_RED,
                               f"STOP y{self.sl_trigger_bev_y:.0f}")):
            y = int(round(row * k))
            if not (-4 <= y <= th + 4):
                continue
            y = max(0, min(th - 1, y))       # 범퍼행 = bev_h 여도 선이 보이게 클램프
            cv2.line(thumb, (0, y), (tw, y), col, 1)
            if lab:
                atxt(thumb, 3, max(10, y - 3), lab, col, 0.33)
        if self.sl_poly_bev is not None and sl_fresh:
            pts = (np.asarray(self.sl_poly_bev) * k).astype(np.int32)
            cv2.polylines(thumb, [pts], True, C_MAGENTA, 2)
            bi = int(np.argmax(pts[:, 1]))
            cv2.circle(thumb, (int(pts[bi, 0]), int(pts[bi, 1])), 3, C_MAGENTA, -1)
        cv2.rectangle(canvas, (px + 6, 6), (px + 6 + tw, 6 + th), C_FRAME, 1)
        cv2.line(canvas, (px, 0), (px, vh), C_FRAME, 1)

        # ── 게이지 — 지금값을 문턱과 나란히 ───────────────────────────
        yy = 6 + th + 12
        cv2.rectangle(canvas, (px + 1, yy - 4), (px + pw, yy + 80), C_BG, -1)
        gate, near = self._near_gate(), self._near_metric(dec)
        base = max(1e-6, self._near_gate_base())
        solo = self.tl_solo_stop_min_height
        span = max(base, solo) * 1.3
        blit(canvas, self.tr.patch('근접', C_DIM, 12), px + 7, yy)
        col_near = C_RED if near >= gate else C_TXT
        #  숫자는 atxt(ASCII), 한글 '단독' 만 고정 패치로(매 프레임 PIL 을 안 돌린다).
        head = f"{near:.0f} | RED {gate:.0f} "
        atxt(canvas, px + 48, yy + 12, head, col_near, 0.40)
        x = px + 48 + cv2.getTextSize(head, cv2.FONT_HERSHEY_SIMPLEX, 0.40, 1)[0][0]
        lab = self.tr.patch('단독', col_near, 12)
        blit(canvas, lab, x, yy)
        atxt(canvas, x + lab.shape[1], yy + 12, f"{solo:.0f}px", col_near, 0.40)
        gauge(canvas, px + 8, yy + 20, pw - 22, 8, near / span,
                  C_RED if near >= gate else (108, 108, 114),
                  ((gate / span, (255, 255, 255)),
                   (solo / span, C_AMBER)))

        yy += 40
        #  왼쪽 = BEV 윗변(멀다), 오른쪽 = 밑변(가깝다). 빨간 눈금(발화선)을 넘으면 2단.
        rng = max(1.0, float(self.cam.bev_h))
        cur = self.sl_bev_y if sl_fresh else SL_NONE
        live = cur > SL_NONE
        hit = live and cur >= self.sl_trigger_bev_y
        blit(canvas, self.tr.patch('정지선', C_DIM, 12), px + 7, yy)
        atxt(canvas, px + 62, yy + 12,
                 (f"y{cur:.0f}/{self.sl_trigger_bev_y:.0f}" if live else '--'),
                 (C_RED if hit else C_MAGENTA) if live else C_DIM, 0.40)
        #  먼 정지선은 행이 음수라 막대는 0 에서 자른다(숫자는 그대로).
        gauge(canvas, px + 8, yy + 20, pw - 22, 8,
                  max(0.0, cur / rng) if live else 0.0, C_RED if hit else C_MAGENTA,
                  ((self.sl_trigger_bev_y / rng, C_RED),))


    def _draw_zoom(self, canvas, frame, boxes, roi, x0, zw, vh, bus_ids=(), dec=None, veh=None, arrow=None):
        """ROI 확대 열 — 축소된 뷰가 아니라 원본에서 ROI 를 잘라 키운다.

        위: ROI 전체 + 박스(라벨·높이 px, 근접 게이트를 넘으면 '*').
        아래: 박스 확대 타일(★판단 박스 먼저★, 그다음 높이 큰 순, NEAREST — 픽셀을 지어내지 않는다).
        구간 'B' 면 차량 박스는 판단 색·화살표 비율(VEH), 버스는 회색 'BUS skip', 차량 타일에는
        4칸 경계와 셋째 칸 판독 창(노랑)을 긋는다.
        """
        dec = boxes if dec is None else dec
        col_area = canvas[0:vh, x0:x0 + zw]
        col_area[:] = C_BG                      # 타일 수가 바뀌므로 매번 지운다
        cv2.line(canvas, (x0, 0), (x0, vh), C_FRAME, 1)
        xmin, ymin, xmax, ymax = roi
        rw, rh = xmax - xmin, ymax - ymin
        if rw <= 0 or rh <= 0:
            return
        n = self.ZOOM_TILES
        t = (zw - 6 * (n + 1)) // n                         # 타일 한 변
        k = min((zw - 12) / float(rw), (vh - t - 20) / float(rh))
        w, h = int(rw * k), int(rh * k)
        x, y = x0 + 6, 6
        cv2.resize(frame[ymin:ymax, xmin:xmax], (w, h), dst=canvas[y:y + h, x:x + w],
                   interpolation=cv2.INTER_LINEAR)
        cv2.rectangle(canvas, (x - 1, y - 1), (x + w, y + h), C_FRAME, 1)

        gate = self._near_gate()
        b_veh = veh is not None and len(dec) == 1 and 'b_arrow' in dec[0]
        dec_ids = {id(b) for b in dec}
        is_dec = lambda b: id(b) in dec_ids or (b_veh and b is veh)
        boxes = sorted(boxes, key=lambda b: (not is_dec(b), -b.get('box_h', 0)))
        H, W = frame.shape[:2]
        for i, it in enumerate(boxes):
            x1, y1, x2, y2 = it['box']
            if id(it) in bus_ids:
                lab_c, lab = 'BUS', f"BUS {it['label'][0]} skip"
            elif b_veh and it is veh:
                lab_c = dec[0]['label']
                a = f" <{arrow[0]:.2f}" if (arrow and arrow[0] is not None) else " <--"
                lab = f"VEH {lab_c[0]} {it.get('box_h', 0)}px{a}"
            else:
                lab_c, lab = it['label'], f"{it['label'][0]} {it.get('box_h', 0)}px"
            col = (C_RED if lab_c == 'RED' else C_GREEN if lab_c == 'GREEN' else C_GRAY)
            hit = lab_c == 'RED' and is_dec(it) and it.get('box_h', 0) >= gate
            lab += '*' if hit else ''
            bx1, by1 = x + int(x1 * k), y + int(y1 * k)
            bx2, by2 = x + int(x2 * k), y + int(y2 * k)
            cv2.rectangle(canvas, (bx1, by1), (bx2, by2), col, 2 if hit else 1)
            chip(canvas, bx1, (by1 - 16) if by1 - 16 > y else by2 + 1, lab, col)
            if i >= n:
                continue
            # 타일 — 박스 긴 변의 1.6배 정사각형(최소 16px)을 화면 안으로 밀어 넣는다.
            s = min(H, W, max(16, int(1.6 * max(x2 - x1, y2 - y1))))
            cx, cy = xmin + (x1 + x2) // 2, ymin + (y1 + y2) // 2
            sx = max(0, min(W - s, cx - s // 2))
            sy = max(0, min(H - s, cy - s // 2))
            tx, ty = x0 + 6 + i * (t + 6), vh - t - 6
            cv2.resize(frame[sy:sy + s, sx:sx + s], (t, t),
                       dst=canvas[ty:ty + t, tx:tx + t], interpolation=cv2.INTER_NEAREST)
            if b_veh and it is veh:              # 'B' — 4칸(빨강·황색·화살표·원등) 경계 + 셋째 칸 판독 창
                q = t / float(s)
                X = lambda v: int(round(tx + (v + xmin - sx) * q))
                Y = lambda v: int(round(ty + (v + ymin - sy) * q))
                bw, bh = x2 - x1, y2 - y1
                for f in (0.25, 0.5, 0.75):
                    cv2.line(canvas, (X(x1 + f * bw), Y(y1)), (X(x1 + f * bw), Y(y2)), C_DIM, 1)
                cv2.rectangle(canvas, (X(x1 + ARROW['x'][0] * bw), Y(y1 + ARROW['y'][0] * bh)),
                              (X(x1 + ARROW['x'][1] * bw), Y(y1 + ARROW['y'][1] * bh)), C_AMBER, 1)
            cv2.rectangle(canvas, (tx, ty), (tx + t - 1, ty + t - 1), col, 2 if hit else 1)
            chip(canvas, tx + 2, ty + 2, lab, col)

    def _permit_txt(self, now):
        """지금 개입 허락이 어디서 오는가 — HUD 용. 허락이 없으면 빨간불을 봐도 안 문다."""
        if not self.require_permission:
            return '항상'
        if self.tl_enable and (now - self.tl_enable_t) <= TL_ENABLE_STALE_S:
            return '체크박스'
        if self.tl_permit and (now - self.tl_permit_t) <= TL_ENABLE_STALE_S:
            return f"주행({self.drive_state or '?'})"
        return ''

    def _draw_hud(self, canvas, cw, vh, hh, state, boxes, now, sl_live, sl_fresh, arrow=None, numbers=True):
        """하단 HUD ★2줄★ [2026-10-08] — 판단과 그 근거만. boxes = ★판단에 쓴 박스★(근접 높이가 실제 판단과 같다).
        numbers=False(BEV 패널이 떠 있다)면 근접·정지선 숫자는 패널 게이지에 있으므로 싣지 않는다.
        ★열은 고정폭 '칸' 으로 적는다★ — 줄이 달라도 열이 맞는다.

        한 줄의 원소 = (칸, 글자, 색[, 굵게]). 앞 원소와 겹치려 하면 밀어 낸다
        (겹쳐 찍혀 못 읽는 것보다 열이 조금 어긋나는 편이 낫다).
        """
        sz, cell = self.HUD_SIZE, self.tr.cell(self.HUD_SIZE)
        scol = (C_RED if 'RED' in state
                else C_GREEN if state == 'GREEN' else C_DIM)
        permit = self._permit_txt(now)
        lvl = self.stop_level if self.stopping else 0
        lvl_txt = f"제동 {lvl}단" if lvl else '제동 없음'
        gate, near = self._near_gate(), self._near_metric(boxes)
        solo = self.tl_solo_stop_min_height
        near_txt = f"근접 {near:.0f}  RED≥{gate:.0f} 단독≥{solo:.0f}px"
        # 구간 — 'B' 면 화살표 판독이 곧 근거다.
        z = self.tl_zone if self._zone_active() else ''
        zcol = C_TXT
        if z == 'B' and self.bus_detect and self.b_need_arrow:
            if not arrow:
                ztxt, zcol = 'B 차량박스 없음', C_DIM
            elif arrow[0] is None:
                ztxt, zcol = f"B 화살표 판독 전(<{self.b_arrow_min_h:.0f}px) 정지", C_AMBER
            else:
                ztxt = f"B 화살표 {arrow[0]:.2f}/{self.b_arrow_frac:.2f} {'진행' if arrow[1] else '없음'}"
                zcol = C_GREEN if arrow[1] else C_RED
        else:
            ztxt = f"구간 {z or '-'}"
        cur = self.sl_bev_y if sl_fresh else SL_NONE
        if not sl_live:
            sl_txt, sl_col = '정지선 OFF', C_DIM
        elif cur <= SL_NONE:
            sl_txt, sl_col = '정지선 --', C_DIM
        else:
            sl_txt = (f"정지선 y{cur:.0f}/{self.sl_trigger_bev_y:.0f}"
                      f" {self.cam.m_txt(self.sl_px)}".rstrip()
                      + (' 확정' if self._sl_confirmed(now) else ''))
            sl_col = C_RED if cur >= self.sl_trigger_bev_y else C_MAGENTA
        waited = (0.0 if self.red_conf_t is None else max(0.0, now - self.red_conf_t))

        lines = [
            [(0, state, scol, True),
             (10, ztxt, zcol),
             (38, near_txt if numbers else '', C_RED if near >= gate else C_TXT),
             (66, f"FPS {self.fps:.0f}", C_DIM)],
            [(0, lvl_txt, C_RED if lvl else C_DIM, True),
             (10, (f"{self.stop_why}" if (self.stop_why and lvl) else
                   (f"대기 {waited:.1f}s" if self.sl_wait else '')), C_AMBER if self.sl_wait and not lvl else C_TXT),
             (38, sl_txt if numbers else '', sl_col),
             (66, f"허락 {permit or '없음'}", C_GREEN if permit else C_RED)],
        ]
        items = []
        for li, segs in enumerate(lines):
            y, xend = 5 + li * self.tr.line_h(sz), 0.0
            for seg in segs:
                col_cell, txt, col = seg[0], seg[1], seg[2]
                if not txt:
                    continue
                bold = len(seg) > 3 and seg[3]
                size = sz + (3 if bold else 0)
                x = max(col_cell * cell, xend)
                items.append((x, y - (2 if bold else 0), txt, col, size, bold))
                xend = x + self.tr.width(txt, size, bold) + 2 * cell
        blit(canvas, self._hud_strip.get(cw, hh, items, C_HUD), 0, vh)

    def show_pending(self):
        """★메인 스레드에서만 부른다★ 마지막으로 그려 둔 캔버스를 창에 띄운다."""
        if not self.show_window:
            return
        with self._show_lock:
            frame, self._show_frame = self._show_frame, None
        if frame is None:
            return
        if not self._window_ready:
            # 크기를 사람이 바꿀 수 있게 두고, 처음 크기만 정해 준다
            # (WINDOW_AUTOSIZE 면 원본 크기로 고정되어 창을 못 줄인다).
            cv2.namedWindow('Traffic Light', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Traffic Light', frame.shape[1], frame.shape[0])
            self._window_ready = True
        cv2.imshow('Traffic Light', frame)
        cv2.waitKey(1)

    # ══════════════════════════════════════════════════════════════════════════
    #  판단 — RED 확정 필터
    # ══════════════════════════════════════════════════════════════════════════
    def _feed_state(self, state):
        """프레임 판정 하나를 스트릭에 먹인다.

        마지막 RED 로부터 tl_gap_grace_s 를 넘겨서야 스트릭을 죽인다 — 검출은
        RED,RED,UNKNOWN,RED,… 처럼 끊기므로 한 프레임에 리셋하면 확정이 안 된다.
        RED_FAR 는 스트릭에 먹이지 않는다(관측 전용) — 먹이면 먼 오검출이 쌓아 둔 스트릭으로
        한 프레임 RED 에 즉시 물리고, 다른 교차로의 먼 빨간불이 해제를 막는다.
        """
        now = time.time()
        if state == 'RED':
            if self.red_since is None:
                self.red_since = now
            self.red_last_seen = now
        elif (self.red_last_seen is None
              or (now - self.red_last_seen) > self.tl_gap_grace_s):
            # 스트릭만 죽인다 — red_last_seen 은 해제 판정(_red_gone)의 근거라 남긴다.
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
        """근접 RED 가 지금 보이고 tl_hold_s 이상 이어졌는가. RED_FAR 로는 서지 않는다
        (리니어는 물거나 풀거나 둘뿐이라 멀리서 물면 곧 급제동이다)."""
        if self.red_since is None or not self._fresh(now):
            return False
        if self.tl_state != 'RED':
            return False
        return (now - self.red_since) >= self.tl_hold_s

    def _red_armed(self, now):
        """이번 접근에서 이미 확정했고(red_conf_t) 근접 RED 를 tl_arm_hold_s 안에 봤는가.
        확정하는 조건은 그대로 _red_confirmed 이고, 이것은 짧은 끊김 동안 대기를 잇기만 한다.
        해제 때 _reset_stop_line_wait 가 red_conf_t 를 지우므로 풀린 뒤 다시 물지는 않는다."""
        return (self.tl_arm_hold_s > 0.0 and self.red_conf_t is not None
                and self.red_last_seen is not None and self._fresh(now)
                and (now - self.red_last_seen) <= self.tl_arm_hold_s)

    def _red_gone(self, now, hold=None):
        """근접 RED 를 hold(기본 red_release_hold_s) 이상 못 봤는가.
        놓는 쪽에만 유예를 둔다 — 검출이 흔들릴 때 리니어가 왕복하지 않게(무는 쪽은 그대로)."""
        if self.red_last_seen is None:
            return True
        return (now - self.red_last_seen) >= (self.red_release_hold_s if hold is None else hold)

    def _release_ready(self, now):
        """서 있다가 놓아도 되는가 [2026-10-08] — RED 가 red_release_hold_s 사라졌고 ★초록이 확정됐거나★,
        초록도 못 본 채 RED 가 red_release_hold_lost_s 사라졌다(신호가 시야를 벗어남·검출 끊김).
        초록은 'RED 가 이미 사라진 뒤' 에만 보므로 RED·GREEN 이 번갈아 잡혀도 2↔0 왕복이 생기지 않는다(08-14 사례)."""
        if not self._red_gone(now):
            return False
        return self._green_confirmed(now) or self._red_gone(now, self.red_release_hold_lost_s)

    def _green_confirmed(self, now):
        if self.green_since is None or not self._fresh(now):
            return False
        return (now - self.green_since) >= self.green_hold_s

    def _sl_present(self, now):
        """정지선을 ★지금 보고 있는가★ (sl_stale_s 안의 관측이 있는가)."""
        return self.sl_seen_t > 0.0 and (now - self.sl_seen_t) <= self.sl_stale_s

    def _sl_confirmed(self, now):
        """sl_hold_s 이상 이어서 본 정지선인가(스친 오검출로 브레이크를 미루지 않게)."""
        return (self.sl_since is not None and self._sl_present(now)
                and (now - self.sl_since) >= self.sl_hold_s)

    def _stop_plan(self, now):
        """RED 는 확정됐다 — 지금 서야 하는가. (level, 근거) 를 돌려준다(규칙은 파일 헤더).

        근거는 OR 이고 먼저 성립한 것으로 문다 — 가까워지면 등기구가 ROI 위로 벗어나 정지선이
        발화선에 닿기 0.6~1.6초 전에 사라지기 때문이다(2026-09-08 전수 시험에서 안 서던 2회).
        정지선 쪽을 먼저 보는 것은 로그 때문이다('정지선 앞' 이 사후 판정에 더 쓸모 있다).
        여기서 0 을 돌려줘도 이미 문 것을 내리지 않는다(단조 증가).
        """
        full = self.brake_level
        if self.red_conf_t is None:
            self.red_conf_t = now          # 이번 접근의 확정 시각(_red_armed·HUD)
        sl_live = self.sl_enable and self.sl_model is not None
        sl_seen = False
        if sl_live:
            if self._sl_confirmed(now):
                self.sl_engaged = True
                self.sl_engaged_max_y = max(self.sl_engaged_max_y, self.sl_bev_y)
                sl_seen = True
                if self.sl_bev_y >= self.sl_trigger_bev_y:
                    return full, '정지선 앞'
            elif (self.sl_engaged and not self._sl_present(now)
                  and (self.sl_lost_min_bev_y < 0.0
                       or self.sl_engaged_max_y >= self.sl_lost_min_bev_y)):
                # 확정했던 정지선이 사라졌다 = 이미 선 위다 → 즉시.
                self.get_logger().info(f"🛑 정지선을 놓쳤다 — 이미 선 위다, 즉시 {full}단")
                return full, '정지선 놓침'
        # OR — 정지선을 보고 있든 아니든 신호등이 단독 문턱을 넘으면 선다.
        if self.tl_solo_stop_min_height > 0.0:
            if self._near_metric(self.last_dec) >= self.tl_solo_stop_min_height:
                return full, '신호등 단독'
        return 0, ('대기' if sl_seen else '신호등 대기')

    def _sl_px_txt(self):
        """로그·HUD 용 정지선 문자열 — 판정값(BEV 행)/발화선 (참고 거리). 미검출 '--'."""
        if self.sl_bev_y <= SL_NONE:
            return '--'
        return (f"y{self.sl_bev_y:.0f}/{self.sl_trigger_bev_y:.0f}"
                f" ({self.sl_px:.0f}px{self.cam.m_txt(self.sl_px)})")

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
        """driving 의 허락(TRAFFIC_LIGHT_ENABLE + DRIVE_RUN). 매핑·IDLE 에서는 False 다."""
        self.tl_permit   = bool(msg.data)
        self.tl_permit_t = time.time()

    def cb_tl_zone(self, msg: String):
        """driving 의 신호 구간. 대문자로 접어 'T'/'A'/'B' 외는 전부 '' (구간 밖).
        ★첫 글자만 본다★ — white1 의 본선 코스 라벨 'T1'~'T5' 도 'T' 가 된다."""
        z = str(msg.data).strip().upper()[:1]
        z = z if z in ('T', 'A', 'B') else ''
        if z != self.tl_zone:
            self.get_logger().info(f"🗺 신호 구간 {self.tl_zone or '-'} → {z or '-'}"
                                   + ("  (🚌 버스 신호 빼고 ⬅️ 좌회전 화살표만 진행)"
                                      if z == 'B' and self.bus_detect and self.b_need_arrow else ""))
            self.arrow_lit_prev = None
        self.tl_zone   = z
        self.tl_zone_t = time.time()

    def cb_fake_box_h(self, msg: Float32):
        """★시험용★ 빨간 박스 높이를 바깥에서 준다. 0 이면 주입을 끈다."""
        new = max(0.0, float(msg.data))
        if (new > 0.0) != (self.fake_box_h > 0.0):
            self.get_logger().warn(
                f"🧪 신호등 주입 {'ON' if new > 0.0 else 'OFF'} "
                f"(박스높이 {new:.0f}px) — ★인지는 시험되지 않는다★")
        self.fake_box_h = new

    def _fake_red_box(self, bh):
        """주입용 빨간 박스 — _all_tl_boxes 가 내는 것과 같은 모양(좌표는 HUD 용)."""
        bh = max(1, int(round(bh)))
        bw = max(1, bh * 4)
        x1, y1 = 40, 20
        return {'label': 'RED', 'conf': 1.0, 'box': (x1, y1, x1 + bw, y1 + bh),
                'box_h': bh, 'hsv_red': 9999, 'hsv_green': 0}

    def _permitted(self, now):
        """지금 손을 대도 되는가(안전 규약 ②) — /tl_enable 또는 /tl_permit 이 ★신선한★ True.
        신선도를 보는 이유: 발행 노드가 죽어 마지막 True 가 굳으면 차가 영영 물려 있게 된다."""
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
            self._set_stop_level(0, '허락 없음')
            self._apply_brake(0)
            # 요구도 0 으로 내린다 — 그냥 return 하면 master 는 낡음 판정까지 마지막 2단을 주장한다.
            self.pub_req.publish(Int32(data=0))
            self._reset_stop_line_wait()
            self._publish_sl(now)
            return

        # ① 해제 — _red_confirmed(지금 RED)와 _red_gone(유예 동안 RED 없음)은 동시에 참일 수
        #    없으므로 순서가 판정을 바꾸지 않는다.
        if self.stopping:
            if self.stop_latch:
                # 래치 ON — 초록불을 확정해야 놓는다.
                if self._green_confirmed(now):
                    self.get_logger().info("🟢 초록불 확정 — 리니어 해제, 주행 재개")
                    self._set_stop_level(0, '초록불')
            elif self._release_ready(now):
                self._release_on_red_gone(now)

        # ② 계획 — 물기 전까지 매 틱 다시 본다(정지선이 발화선으로, 신호등이 단독 문턱으로 다가온다).
        if self.stop_level < self.brake_level:
            if self._red_confirmed(now) or self._red_armed(now):
                level, why = self._stop_plan(now)
                if level <= 0:
                    # 기다린다 — 아무것도 내지 않고 차는 원래 명령대로 간다.
                    self.sl_wait = True
                    self.get_logger().info(
                        f"🚦⏸ 빨간불 확정 — 대기 중 [{why}] "
                        f"(정지선 {self._sl_px_txt()} · 신호등 "
                        f"{self._near_metric(self.last_dec):.0f}/"
                        f"{self.tl_solo_stop_min_height:.0f}px, "
                        f"확정 후 {now - self.red_conf_t:.1f}s)",
                        throttle_duration_sec=1.0)
                else:
                    self.sl_wait = False
                    self._set_stop_level(level, why)
            elif not self.stopping:
                # 확정이 끊겼다 — 이번 접근의 기억을 지운다(물고 있는 동안은 지우지 않는다).
                self._reset_stop_line_wait()

        # ③ 발행 — 요구는 매 틱, 브레이크는 _apply_brake 가 변화분·재확인만.
        self.pub_req.publish(Int32(data=int(self.stop_level)))
        self._publish_sl(now)

        if self.stopping:
            self._apply_brake(self.stop_level)
            if self.publish_cmd_vel:
                # 펄스 0·조향 0 — 풀 때 직전 주행값이 되살아나지 않게, 정지 중 바퀴는 일직선.
                out = Twist()
                out.linear.x  = 0.0
                out.angular.z = 0.0
                self.pub_cmd.publish(out)
        else:
            self._apply_brake(0)

    def _release_on_red_gone(self, now):
        """빨간불을 보는 동안만 잡는다 — 해제는 _release_ready(RED 가 사라진 뒤의 초록, 또는 더 긴 미감지).
        GREEN 확정 ★하나만★ 해제 근거로 쓰면 RED·GREEN 이 번갈아 잡힐 때 두 스트릭이 함께 살아
        틱 주기로 2↔0 을 왕복한다(2026-08-14 로스백 50회). 놓은 뒤는 원래 명령의 주인이 잇는다.
        """
        green = self._green_confirmed(now)
        self.get_logger().info(
            (f"⚪ 빨간불 {self.red_release_hold_s:.1f}초 미감지 + 초록불 확정 — 리니어 해제" if green else
             f"⚪ 빨간불 {self.red_release_hold_lost_s:.1f}초 미감지(초록도 못 봄) — 리니어 해제"))
        self._set_stop_level(0, '빨간불 사라짐')

    def _reset_stop_line_wait(self):
        """대기 상태를 이번 접근분까지 통째로 지운다(다음 교차로를 위해)."""
        self.sl_wait    = False
        self.sl_engaged = False
        self.sl_engaged_max_y = SL_NONE
        self.red_conf_t = None

    def _publish_sl(self, now):
        """정지선 진단 토픽(기록용). sl_px 는 0 밑으로 안 낸다 — −1 이 '미검출' 하나만 뜻하게."""
        fresh = self._sl_present(now)
        self.pub_sl_bev_y.publish(Float32(
            data=float(self.sl_bev_y if fresh else SL_NONE)))
        self.pub_sl_px.publish(Float32(
            data=float(max(0.0, self.sl_px) if fresh else -1.0)))
        self.pub_sl_y.publish(Float32(
            data=float(self.sl_y if fresh else -1.0)))
        self.pub_sl_wait.publish(Bool(data=bool(self.sl_wait)))

    @property
    def stopping(self):
        """이 노드가 지금 브레이크를 물고 있는가."""
        return self.stop_level > 0

    def _set_stop_level(self, level, why=''):
        """정지 단계를 정한다. 실제 발행은 tick() 과 _apply_brake() 가 한다.

        단조 증가 — 0 으로 내리는 것은 해제뿐이다(허락 상실 · 빨간불 사라짐 · 래치면 초록 확정).
        """
        level = max(0, min(2, int(level)))
        if level > 0 and level < self.stop_level:
            return                      # 내리는 요청은 무시한다(왕복 금지)
        if level == self.stop_level:
            return
        prev, self.stop_level = self.stop_level, level
        self.stop_why = why
        if level <= 0:
            self._reset_stop_line_wait()
            return
        # 근거(정지선 앞 / 정지선 놓침 / 신호등 단독)를 남긴다 — cam_testbed 계약이 이 대괄호를 잡는다.
        self.get_logger().warn(
            f"🚦🛑 빨간불 확정 — 정지 [{why}] (리니어 {prev}단 → {level}단, "
            f"정지선 {self._sl_px_txt()}, 신호등 "
            f"{self._near_metric(self.last_dec):.0f}/"
            f"{self.tl_solo_stop_min_height:.0f}px)"
            + (" + /cmd_vel_raw 0/0" if self.publish_cmd_vel else ""))

    def _apply_brake(self, level):
        """리니어 단계를 요청한다 — 바뀔 때 + 물고 있는 동안 BRAKE_KEEPALIVE_S 마다.

        (안전 규약 ③) 0단은 우리가 건 적이 있을 때만, DRIVE_DONE 이면 내지 않는다.
        DRIVE_RUN 중 driving 의 종점 접근제동과 겹쳐도 driving 이 자기 단계를 재발행해 되돌린다.
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
            # 재확인 — master 가 0.5s 마다 자기 레버값(0)을 재발행해 덮기 때문이다.
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
        #  ★주입이 켜져 있으면 계속 떠든다★ 조용히 켜져 있는 것이 제일 위험하다.
        if self.fake_box_h > 0.0:
            self.get_logger().warn(
                f"🧪 신호등 주입 모드 ON ({self.fake_box_h:.0f}px) — 실차 금지",
                throttle_duration_sec=5.0)
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
                f"🛑 신호등 정지 유지 중 [{self.stop_why}] {self.stop_level}단 | tl={self.tl_state} "
                f"raw={self.last_raw} drop={self.last_red_drop} fps={self.fps:.1f}"
                + (f" sl={self._sl_px_txt()}" if self._sl_present(now) else " sl=없음"),
                throttle_duration_sec=2.0)

    def shutdown_release(self):
        """내려갈 때 리니어를 반드시 풀고 간다 — arduino 는 마지막 /brake_level 을 캐시로
        물고 있어, 2단을 건 채 죽으면 차가 못 움직인다. 발행 뒤 잠깐 스핀해 실제로 나가게 한다
        (그래서 rclpy 컨텍스트가 살아 있어야 한다 — _install_shutdown_signals)."""
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
    """SIGINT/SIGTERM 을 KeyboardInterrupt 로 받는다. rclpy 기본 핸들러는 컨텍스트를 먼저
    내려 finally 의 해제 발행이 실패한다(리니어가 물린 채 노드만 사라졌다)."""
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
    # 두 스레드 — 추론이 길어져도 브레이크 유지·요구 발행(tick)이 굶지 않게.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    # spin 은 배경 스레드, 창은 메인 스레드(HighGUI 는 스레드 안전하지 않다).
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
