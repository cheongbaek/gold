#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ports.py — ★GPS · IMU · 카메라 장치 식별의 단일 소유자★ (Ubuntu 22.04 전용)

★ [2026-08-14] white1 에도 카메라 판정을 들여왔다 ★ 신호등 정지 노드
  (white1/traffic_light.py)가 usb_cam 을 쓰게 되어, white806 이 구 white/ports.py 에서
  되가져왔던 것(video_capture_devices · probe_camera · resolve_camera · camera_formats ·
  resolve_camera_format + CAM_* 상수)을 그대로 옮겨 왔다.
  ★고쳐서 가져오지 않았다★ — 저 코드는 2026-08-04 의 실제 사고(덮개 닫힌 내장 웹캠을
  열어 검은 화면으로 주행)를 겪고 나온 것이고, 그 실측 근거가 주석에 그대로 붙어 있다.

╔══════════════════════════════════════════════════════════════════════════════╗
║  [2026-08-05] nxde/ports.py 에서 이 파일로 옮겨왔다                            ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  하드웨어 담당을 다시 나눴다:                                                  ║
║    · white one_launch.py  → GPS · IMU · 카메라   (이 파일이 경로를 정해준다)    ║
║                             + nxde 의 arduino 노드를 함께 띄운다               ║
║    · nxde                 → 아두이노 A/B 통신 노드뿐 (★런치파일 없음★)         ║
║  그래서 아두이노 탐색 부분(ARDUINO_VIDS / looks_like_arduino /                 ║
║  arduino_candidate_ports)은 이 파일에 없다 — ★nxde/arduino.py 가 자체 소유★     ║
║  한다. 아두이노 노드는 white 를 import 하지 않고 혼자 돌 수 있어야 하기 때문이다  ║
║  (차량 구동의 최소 단위이고, 역방향 의존은 colcon 순환을 만든다).                ║
║                                                                              ║
║  ⚠️ 두 곳이 같은 대역(/dev/ttyACM* · /dev/ttyUSB*)을 나눠 쓰므로 서로를 밟지    ║
║     않게 하는 장치가 필요하다. 그 책임은 아두이노 쪽에 있다 —                   ║
║     nxde/arduino.py 가 GPS/IMU VID/PID 와 udev 링크를 스스로 보고 제외한다.     ║
║     (예전에는 구 g.launch.py 가 GPS/IMU 경로를 확정해 exclude_ports 로          ║
║      넘겨줬는데, 런치가 갈라지면서 그 전달 경로가 끊겼다. one_launch.py 는      ║
║      여전히 확정 경로를 넘겨주지만 ★없어도 동작한다★.)                          ║
╚══════════════════════════════════════════════════════════════════════════════╝

═══════════════════════════════════════════════════════════════════════════════
 ★ 장치 경로 해석 순서 (resolve_device) ★
   1) udev 심볼릭링크가 있으면 그것 (/dev/gps, /dev/imu …)  ← ★권장★
   2) 없으면 VID/PID 로 현재 연결된 포트를 스캔
   3) 그것도 없으면 심볼릭링크 경로를 '그대로' 반환한다 (존재하지 않아도)

 3) 이 중요하다. 장치가 런치 시점에 안 꽂혀 있으면 VID/PID 스캔은 실패하는데, 이때
 '없는 경로'라도 **안정적인 이름**을 돌려주면 나중에 꽂는 순간 그 경로가 생기므로
 respawn(외부 노드) 또는 자체 재연결 루프(우리 노드)가 자동으로 붙는다.
 반대로 /dev/ttyUSB0 같은 열거 순서 의존 경로를 넘기면 다른 장치를 열어버릴 수 있다.

 ★ udev 설정은 사실상 필수다 ★ 설정법은 nxde/README.md 8절 참고.
   GPS·IMU·아두이노가 전부 /dev/ttyACM*·/dev/ttyUSB* 대역을 공유하기 때문이다.

 ★ 런치 전에 `ros2 run nxde check` 로 연결을 먼저 확인할 것 ★
   장치가 꽂혀 있는지, 어느 경로에 붙었는지, GPS 가 실제로 RTK 를 물고 있는지를
   포트 점유 없이 보고하고 끝난다. 여기서 초록불을 본 뒤 런치를 띄우면
   '런치 시점에 없던 장치를 나중에 꽂아 생기는 경합'을 통째로 피할 수 있다.
═══════════════════════════════════════════════════════════════════════════════
"""

import os
import time

try:
    from serial.tools import list_ports
except Exception:      # pyserial 이 없는 환경(런치 파서 검사 등)에서도 import 는 되게
    list_ports = None


# ── udev 심볼릭링크 권장 이름 (README 8절의 규칙과 일치해야 한다) ──
SYMLINK_GPS = '/dev/gps'
SYMLINK_IMU = '/dev/imu'

# ── 카메라는 udev 링크를 쓰지 않는다 ──
#   /dev/video* 자체가 이미 'video4linux 캡처 노드'를 뜻하고 sysfs 로 VID/PID·index 를
#   바로 읽을 수 있다. 아무 카메라도 없을 때 넘길 '안정적인 경로'만 상수로 둔다.
#
#   ⚠️ 반드시 '캡처' 노드여야 한다. UVC 카메라 한 대가 노드를 2개 만드는데
#      짝수(index=0)가 Video Capture, 홀수(index=1)가 Metadata Capture 다.
#      2026-08-04 실측: video0/1 = 내장 웹캠(04f2:b7f3), video2/3 = See3CAM(2560:c137).
#      → 외부 카메라의 캡처 노드는 video2 다. video3 을 넣으면 메타데이터를 열어
#        영상이 안 나온다.
#
#   ⚠️ ★[2026-08-12] 이 상수를 '카메라 없을 때의 대기 경로'로 쓰면 안 된다★
#      기계가 바뀌면 같은 번호가 전혀 다른 것을 가리킨다 — 현 개발 노트북 실측:
#        video0/1 = 내장 RGB, ★video2/3 = 내장 적외선★, video4/5 = See3CAM
#      즉 여기서 video2 로 폴백하면 '카메라를 못 찾았으니 대기' 가 아니라
#      ★내장 적외선 카메라를 여는 것★ 이 된다. 그래서 못 찾았을 때는
#      next_free_video_path() 가 예측한 '비어 있는 다음 번호'를 쓰고, 이 상수는
#      그 예측마저 실패했을 때의 최후 값으로만 남긴다.
FALLBACK_CAM = '/dev/video2'

# ── ★카메라 선택 우선순위 (2026-08-05 지정)★ ──
#   외부 See3CAM 이 붙는 자리가 video2, 노트북 내장 웹캠이 video0 이다.
#   "video2 를 먼저 보고, 없으면 video0" 이 운용 규칙이므로 그 순서를 명시한다.
#   ★단 순서만으로 고르지 않는다★ — 각 후보를 실제로 열어 프레임을 보고(probe_camera)
#   살아있는 것 중에서 이 순서를 적용한다. 순서만 믿으면 '덮개 닫힌 내장 웹캠'이나
#   '동결된 스트림'을 그대로 열어 검은 화면으로 주행하게 된다(2026-08-04 실제 사고).
#   목록에 없는 장치(video4 …)는 이 뒤로 밀리며, 그들끼리는 '외부 우선 → 번호순'이다.
CAM_PREFERRED_ORDER = ('/dev/video2', '/dev/video0')

# ── ★차량 카메라 화이트리스트 (1순위 판정)★ [2026-08-12] ──
#   "내장을 빼는" 방식만으로는 부족하다 — 블랙리스트에 없는 새 노트북으로 옮기는 순간
#   그 기계의 내장 웹캠이 조용히 통과한다(실제로 그랬다. 아래 INTERNAL_CAM_VIDPID 의
#   2026-08-12 항목 참고). 그래서 '이것이 차량 카메라다'를 먼저 못박고, 블랙리스트와
#   이름 규칙은 보조로 쓴다.
#   ⚠️ 여기 있는 장치를 찾으면 ★프레임 검증과 무관하게 최우선★ 이다. 다만 죽어 있으면
#      (검정/동결) 그 사실은 로그에 그대로 찍힌다 — 고르는 것과 상태 보고는 별개다.
EXTERNAL_CAM_VIDPID = [
    (0x2560, 0xC137),   # e-con Systems See3CAM_CU31 (차량 카메라)
]

# ── 노트북 '내장' 웹캠 VID/PID (블랙리스트 — 보조 판정) ──
#   카메라는 반드시 외부 USB 카메라를 써야 한다. 내장 웹캠은 덮개를 닫으면 키보드
#   데크를 보게 되어 균일한(검은/포화) 프레임만 나온다 — 2026-08-04 실측: 컨트롤
#   기본값에서 평균 4.5/255(검정), See3CAM 기준 설정에서는 전 픽셀 255(포화).
#   video_device 를 '/dev/video0' 로 고정해 두면 외부 카메라가 없거나 나중에 열거될 때
#   조용히 내장 웹캠을 열어버린다 → 이 목록으로 내장을 '제외'하고 외부를 고른다.
INTERNAL_CAM_VIDPID = [
    (0x04F2, 0xB7F3),   # Chicony "HP True Vision FHD Camera" (구 노트북 내장)
    # ★[2026-08-12] 이 항목이 없어서 실제로 뚫렸다★ 현 개발 노트북에서 resolve_camera()
    #   가 내장 웹캠(/dev/video0)을 고르면서 로그에는 '외부'라고 찍었다 — 블랙리스트에
    #   이 VID/PID 가 없으니 '내장 아님 = 외부'로 판단했기 때문이다. 목록 방식의 실패
    #   모드가 정확히 이것이라, 같은 날 EXTERNAL_CAM_VIDPID(화이트리스트)를 넣었다.
    (0x3277, 0x0018),   # Sonix "ASUS FHD webcam" (현 개발 노트북 내장)
]

# ── 이름으로 거르는 노드 (보조 판정) ──
#   ★내장 카메라 한 대가 캡처 노드를 2개 만든다★ 2026-08-12 실측(ASUS 노트북):
#     video0 = 'ASUS FHD webcam: ASUS FHD webca'  (RGB)
#     video2 = 'ASUS FHD webcam: ASUS IR camera'  (적외선)
#   둘은 VID/PID 가 같아서 VID/PID 만으로는 갈리지 않는다. 적외선 노드는 조명이 없으면
#   검은 화면, 있으면 단색으로 밝게 나와 ★프레임 검증도 속일 수 있다★ — 이름으로 뺀다.
#   (구 white 의 '짝수 index 만' 규칙도 여기서는 안 통한다. 둘 다 index=0 이다.)
CAM_NAME_EXCLUDE = ('IR CAMERA',)     # 대문자 비교

# ── 이름 힌트 (환경변수 CAM_NAME_HINT 로 런치에서 준다) ──
#   장치 이름에 이 문자열이 들어 있으면 최우선으로 고른다. VID/PID 를 모르는 새 카메라를
#   급히 물릴 때 코드 수정 없이 지정하는 통로다 (예: CAM_NAME_HINT=See3CAM).
CAM_NAME_HINT_ENV = 'CAM_NAME_HINT'

# ── 영상 '쓸만함' 판정 임계값 ──
#   VID/PID 만 믿지 않고 실제로 한 번 열어 프레임을 본다. 어느 쪽이 차량 카메라인지
#   확실히 갈리기 때문이다 — 2026-08-04 실측(640x480, 워밍업 3장 버림):
#     /dev/video0 내장(덮개로 가려짐) : 평균   0.67  std  0.00  프레임간차이 0.0000
#     /dev/video2 See3CAM_CU31        : 평균 170.46  std 44.04  프레임간차이 4.7055
#   세 지표가 모두 자릿수 단위로 벌어지므로 임계값은 넉넉하게 잡아도 안전하다.
#   ★프레임간차이가 가장 강한 신호다★ — 살아있는 센서는 샷노이즈 때문에 연속 프레임이
#   절대 같을 수 없다. 정확히 0 이면 스트림 동결 또는 렌즈 차단이다.
CAM_MIN_MEAN =   8.0    # 이보다 어두우면 '검은 화면'
CAM_MAX_MEAN = 247.0    # 이보다 밝으면 '포화'(전 픽셀 255)
CAM_MIN_STD  =   2.0    # 평탄하면 내용이 없다
CAM_MIN_DIFF =   0.1    # 연속 프레임이 같으면 동결

# ── V4L2 fourcc → usb_cam 의 pixel_format 이름 ──
#   ★카메라마다 지원 포맷이 다르다★ usb_cam 은 지원하지 않는 포맷을 주면
#   std::invalid_argument 로 즉사하고 respawn 무한루프를 돈다. 그래서 '선택된 장치가
#   요청 해상도에서 실제로 내는 포맷'을 보고 정한다. 2026-08-04 실측:
#     See3CAM_CU31(2560:c137) 1920x1080 : UYVY 뿐  (MJPEG 없음)
#     내장 Chicony(04f2:b7f3) 1920x1080 : MJPG 뿐  (YUYV 는 640x480 까지)
#   하드코딩하면 카메라를 바꿀 때마다 이 함정에 다시 빠진다.
V4L2_TO_USBCAM_FORMAT = {
    'UYVY': 'uyvy',
    'YUYV': 'yuyv',
    'MJPG': 'mjpeg2rgb',
}
# 무압축 우선 — MJPEG 는 디코드 비용이 붙고 압축 손실이 있다
CAM_FORMAT_PREFERENCE = ('UYVY', 'YUYV', 'MJPG')


# ── VID/PID 후보 (구 white/launch/one_launch.py 의 실측 목록) ──
GPS_VIDPID = [
    (0x1546, 0x01A9),   # u-blox 9 계열
    (0x1546, 0x01A8),   # u-blox 8 계열 / 일부 수신기
]
IMU_VIDPID = [
    (0x10C4, 0xEA60),   # iAHRS / CP210x 계열
]


def _comports():
    if list_ports is None:
        return []
    try:
        return sorted(list_ports.comports(), key=lambda p: p.device)
    except Exception:
        return []


def find_by_vidpid(candidates, exclude=None):
    """VID/PID 후보 목록으로 현재 연결된 포트를 찾는다. 없으면 None.

    exclude : 이미 다른 장치로 확정된 경로 집합 (같은 VID/PID 장치가 여러 개일 때)"""
    exclude = exclude or set()
    for port in _comports():
        if port.device in exclude:
            continue
        for vid, pid in candidates:
            if port.vid == vid and port.pid == pid:
                return port.device
    return None


def resolve_device(symlink, candidates, exclude=None, log=None):
    """장치 경로를 정한다. 위 파일 헤더의 3단 순서를 따른다.

    반환은 항상 문자열이다(존재하지 않는 경로일 수 있다 — 헤더 3) 참고)."""
    if symlink and os.path.exists(symlink):
        if log:
            log(f"✅ udev 심볼릭링크 사용: {symlink}")
        return symlink

    found = find_by_vidpid(candidates, exclude)
    if found:
        if log:
            log(f"✅ VID/PID 스캔으로 발견: {found}"
                + (f" (권장: udev 로 {symlink} 고정)" if symlink else ""))
        return found

    if log:
        log(f"⚠️ 장치를 찾지 못했습니다 → '{symlink}' 로 계속 시도합니다. "
            f"지금 안 꽂혀 있어도 나중에 꽂으면 자동으로 붙습니다 "
            f"(udev 규칙이 없으면 그 경로가 생기지 않으니 README 8절을 먼저 볼 것). "
            f"★런치 전에 `ros2 run nxde check` 로 확인하는 편이 빠릅니다★")
    return symlink


def _sysfs_usb_vidpid(start_path):
    """sysfs 경로에서 위로 걸어올라가 USB 장치의 (vid, pid) 를 찾는다. 없으면 None."""
    path = os.path.realpath(start_path)
    while path and path != '/':
        try:
            with open(os.path.join(path, 'idVendor')) as fv, \
                 open(os.path.join(path, 'idProduct')) as fp:
                return (int(fv.read().strip(), 16), int(fp.read().strip(), 16))
        except OSError:
            path = os.path.dirname(path)
    return None


# ═══════════════════════════════════════════════════════════════════════════
#  카메라 — 구 white/ports.py 에서 그대로 이식 (2026-08-12)
#    쓰는 곳은 launch/one_launch.py 하나뿐이다(usb_cam 의 video_device·
#    pixel_format 을 정한다). traffic_light 노드는 /image_raw 만 보므로
#    이 파일을 import 하지 않는다.
# ═══════════════════════════════════════════════════════════════════════════
def video_capture_devices():
    """/dev/video* 중 '영상 캡처' 노드만 [(경로, (vid,pid)|None, 이름), ...] 로 반환.

    UVC 카메라 한 대는 보통 노드를 2개 만든다 — index=0 이 영상 캡처, index=1 은
    메타데이터다. index 로 걸러야 metadata 노드를 카메라로 착각하지 않는다
    (2026-08-04 실측: video0=Video Capture/index0, video1=Metadata Capture/index1).

    ★[2026-08-12] 세 번째 원소로 장치 이름을 함께 돌려준다★ VID/PID 만으로는
    갈리지 않는 경우가 있다 — 같은 내장 카메라가 RGB 노드와 적외선 노드를 각각
    index=0 으로 만든다(실측: 'ASUS FHD webca' / 'ASUS IR camera', 둘 다 3277:0018).
    이름이 있어야 적외선 노드를 빼고, 로그에 '무엇을 골랐는지'를 사람이 읽을 수 있다.
    """
    base = '/sys/class/video4linux'
    try:
        names = os.listdir(base)
    except OSError:
        return []

    def _num(name):
        digits = name[5:]
        return int(digits) if digits.isdigit() else 1 << 30

    out = []
    for name in sorted((n for n in names if n.startswith('video')), key=_num):
        node = os.path.join(base, name)
        try:
            with open(os.path.join(node, 'index')) as f:
                if f.read().strip() != '0':
                    continue          # 메타데이터/보조 노드
        except OSError:
            continue
        try:
            with open(os.path.join(node, 'name')) as f:
                cam_name = f.read().strip()
        except OSError:
            cam_name = ''
        out.append(('/dev/' + name,
                    _sysfs_usb_vidpid(os.path.join(node, 'device')),
                    cam_name))
    return out


def camera_formats(dev, timeout_s=5.0):
    """장치가 지원하는 {fourcc: {(w,h), ...}} 를 반환. 실패하면 빈 dict.

    v4l2-ctl 출력을 파싱한다 (one_launch.py 가 이미 v4l2-ctl 을 쓰므로 새 의존성 아님).
    """
    import re
    import subprocess
    try:
        out = subprocess.run(
            ['v4l2-ctl', '--list-formats-ext', '-d', dev],
            capture_output=True, text=True, timeout=timeout_s).stdout
    except Exception:
        return {}

    formats, current = {}, None
    for line in out.splitlines():
        m = re.match(r"\s*\[\d+\]:\s*'(\w+)'", line)
        if m:
            current = m.group(1)
            formats.setdefault(current, set())
            continue
        m = re.search(r'Size:\s*\w+\s+(\d+)x(\d+)', line)
        if m and current:
            formats[current].add((int(m.group(1)), int(m.group(2))))
    return formats


def resolve_camera_format(dev, width, height, default='uyvy', log=None):
    """dev 가 width x height 에서 실제로 내는 포맷을 usb_cam 이름으로 반환."""
    formats = camera_formats(dev)
    if not formats:
        if log:
            log(f"⚠️ {dev} 지원 포맷을 읽지 못해 '{default}' 로 진행합니다.")
        return default

    want = (width, height)
    for fourcc in CAM_FORMAT_PREFERENCE:
        if want in formats.get(fourcc, ()):
            name = V4L2_TO_USBCAM_FORMAT[fourcc]
            if log:
                log(f"✅ 픽셀 포맷: {name} ({fourcc} @ {width}x{height})")
            return name

    # 요청 해상도를 내는 포맷이 없다 — 지원 목록을 그대로 보여주고 default 로 둔다
    if log:
        detail = ', '.join(
            f"{fc}:{sorted(sz, reverse=True)[:3]}" for fc, sz in formats.items())
        log(f"⚠️ {dev} 는 {width}x{height} 를 지원하지 않습니다 → '{default}' 로 진행합니다. "
            f"지원 목록: {detail}")
    return default


def probe_camera(dev, frames=6, warmup=3, timeout_s=4.0):
    """카메라를 열어 실제 프레임을 보고 '쓸 수 있는 영상'인지 판정한다.

    반환 (verdict, detail):
      verdict = True  → 정상 영상
                False → 검정/포화/평탄/동결 (쓸 수 없다)
                None  → 판정 불가 (cv2 없음 · 열기 실패 · 이미 사용 중)
    ★None 을 False 로 취급하면 안 된다★ usb_cam 이 이미 그 장치를 열고 있으면 여기서
    열기가 실패하는데, 그건 '죽은 카메라'가 아니라 '멀쩡히 쓰이는 중'이다.
    """
    try:
        import cv2
        import numpy as np
    except Exception:
        return None, "cv2/numpy 없음 — 내용 판정 생략"

    cap = None
    try:
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not cap.isOpened():
            return None, "열기 실패(이미 사용 중일 수 있음)"
        # 판정용이므로 작게 — 1920x1080 을 받을 이유가 없다(런치 시간 절약)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        grabbed = []
        deadline = time.monotonic() + timeout_s
        while len(grabbed) < frames and time.monotonic() < deadline:
            ok, frame = cap.read()
            if ok and frame is not None:
                grabbed.append(frame)
        if len(grabbed) <= warmup:
            return None, f"프레임 부족({len(grabbed)}장)"

        # 앞 warmup 장은 노출 수렴 전이라 버린다
        grays = [f.mean(axis=2) for f in grabbed[warmup:]]
        mean = float(np.mean([g.mean() for g in grays]))
        std = float(np.mean([g.std() for g in grays]))
        diff = float(np.mean([
            np.abs(grays[i].astype(int) - grays[i - 1].astype(int)).mean()
            for i in range(1, len(grays))
        ])) if len(grays) > 1 else 0.0

        stat = f"평균={mean:.2f} std={std:.2f} 프레임간차이={diff:.4f}"
        if diff < CAM_MIN_DIFF:
            return False, f"{stat} → 동결(연속 프레임 동일 = 렌즈 차단/스트림 정지)"
        if mean < CAM_MIN_MEAN:
            return False, f"{stat} → 검은 화면"
        if mean > CAM_MAX_MEAN:
            return False, f"{stat} → 포화(과다노출)"
        if std < CAM_MIN_STD:
            return False, f"{stat} → 평탄(내용 없음)"
        return True, stat
    except Exception as e:
        return None, f"판정 중 오류: {e}"
    finally:
        if cap is not None:
            try:
                cap.release()
            except Exception:
                pass


def _is_internal_cam(vidpid):
    return vidpid in INTERNAL_CAM_VIDPID


def next_free_video_path(limit=64):
    """지금 비어 있는 첫 /dev/videoN — '이따 꽂으면 여기로 붙을 것'의 최선 추정.

    카메라를 못 찾았을 때 usb_cam 에 줄 대기 경로다. 커널은 새 V4L2 장치에
    ★비어 있는 가장 작은 번호★ 를 준다. 그래서 지금 video0~5 가 차 있으면 다음에
    꽂는 카메라는 video6 이 된다 — 그 경로로 usb_cam 을 띄워 두면 respawn 이 돌다가
    실제로 꽂는 순간 그대로 붙는다.

    ⚠️ 완벽하지 않다. 런치와 카메라 연결 사이에 ★다른★ V4L2 장치(가상캠·캡처보드)가
       먼저 잡히면 번호가 밀린다. 그 경우엔 video_device:= 로 직접 주면 된다.
    """
    for i in range(limit):
        if not os.path.exists(f'/dev/video{i}'):
            return f'/dev/video{i}'
    return FALLBACK_CAM


def classify_camera(vidpid, name=''):
    """이 캡처 노드는 무엇인가 — 'external' / 'internal' / 'unknown'.

    'internal' 은 '노트북 내장'만이 아니라 ★쓰면 안 되는 노드★ 전부를 뜻한다
    (내장 웹캠 + 적외선처럼 영상이 아닌 노드).

    ★판정 순서★
      1) 이름이 CAM_NAME_EXCLUDE 에 걸림 → 'internal'
         ★화이트리스트보다 먼저 본다★ — 적외선 노드는 어느 벤더 것이든 영상이 아니다.
         내장 카메라가 RGB/IR 두 노드를 같은 VID/PID 로 내듯이(실측), 외부 카메라도
         같은 VID/PID 로 IR 노드를 낼 수 있다. 그때 VID/PID 를 먼저 보면 적외선을
         차량 카메라로 집는다(2026-08-12 자체 테스트 ⑧에서 실제로 그랬다).
      2) EXTERNAL_CAM_VIDPID 화이트리스트 → 'external' (차량 카메라라고 못박은 것)
      3) INTERNAL_CAM_VIDPID 블랙리스트 → 'internal'
      4) 그 외 → 'unknown' (판단 근거가 없다. 외부일 수도, 처음 보는 내장일 수도)

    ⚠️ 'unknown' 을 외부로 취급하면 안 된다 — 그게 2026-08-12 에 뚫린 경로다
       (블랙리스트에 없던 새 노트북의 내장 웹캠이 '외부'로 통과했다).
       resolve_camera() 는 화이트리스트가 하나라도 잡히면 unknown 을 아예 보지 않는다.
    """
    up = (name or '').upper()
    if any(bad in up for bad in CAM_NAME_EXCLUDE):
        return 'internal'
    if vidpid in EXTERNAL_CAM_VIDPID:
        return 'external'
    if vidpid in INTERNAL_CAM_VIDPID:
        return 'internal'
    return 'unknown'


def _cam_sort_key(entry, name_hint=''):
    """카메라 후보 정렬 키 — ★'무엇인가'가 1순위, 경로 순서는 3순위★.

    (이름힌트 불일치, 화이트리스트 아님, 경로 순위, 장치번호)

    ★[2026-08-12] 경로 순서를 1순위에서 끌어내렸다★ 예전 키는 CAM_PREFERRED_ORDER
    (video2 → video0)를 맨 앞에 두었는데, 그러면 ★외부 카메라가 목록 밖 번호로
    열거되는 순간 내장에게 진다★. 실측: See3CAM 이 /dev/video4 로 붙고 내장이
    video0 이었더니, 둘 다 살아있는데도 내장(video0, rank=1)이 See3CAM(rank=2)을
    이겼다. 번호는 USB 열거 순서일 뿐이라 '무엇인 장치인가'보다 앞설 이유가 없다.
    """
    dev, vidpid, name = entry
    up = (name or '').upper()
    hint_miss = 0 if (name_hint and name_hint in up) else 1
    not_ext = 0 if classify_camera(vidpid, name) == 'external' else 1
    try:
        rank = CAM_PREFERRED_ORDER.index(dev)
    except ValueError:
        rank = len(CAM_PREFERRED_ORDER)
    num = int(dev[len('/dev/video'):]) if dev[len('/dev/video'):].isdigit() else 1 << 30
    return (hint_miss, not_ext, rank, num)


def resolve_camera(fallback=None, log=None, probe=True, name_hint=None):
    """카메라 경로를 정한다 — ★내장은 아예 후보에서 뺀다 + 실제 프레임 검증★.

      1) 캡처 노드(/dev/video*, index=0)를 모은다
      2) ★분류★ classify_camera() 로 external / internal / unknown 을 가른다.
         internal 은 여기서 ★버린다★ (되살리는 스위치는 두지 않는다 — 아래 참고)
      3) external(화이트리스트)이 하나라도 있으면 ★그것들만★ 후보로 삼는다.
         하나도 없으면 이름 힌트로 지정된 것만 쓰고, 그것도 없으면 ★아무것도 고르지
         않고 기다린다★ — 'unknown' 을 자동으로 쓰면 기계를 옮기는 순간 그 기계의
         내장 웹캠을 열게 된다(블랙리스트에 없으니 unknown 이다).
      4) 남은 후보를 실제로 열어 프레임을 본다(probe). 검정/포화/동결은 뒤로 민다.
      5) 아무 후보도 없으면 fallback 을 반환하고 ★크게 경고★한다 — 나중에 꽂으면
         usb_cam 의 respawn 이 자동으로 붙는다.

    ★[2026-08-12] 왜 '뒤로 미는' 게 아니라 '버리는' 가★
      예전 구현은 내장 여부를 정렬 키의 2순위로만 썼다. 그래서 외부 카메라가
      CAM_PREFERRED_ORDER 밖 번호(/dev/video4)로 열거되자 ★둘 다 살아있는데도 내장이
      이겼다★(실측: video0 내장 rank=1 vs video4 See3CAM rank=2 → video0 선택).
      내장 웹캠으로 신호등을 보는 것은 '조금 나쁜 선택'이 아니라 ★기능이 없는 것★ 이라,
      순위 조정이 아니라 배제가 맞다.

    ★'내장도 허용' 스위치를 두지 않는 이유★ 잠깐 켜 두고 잊으면 그대로 주행에 들어간다
      — 이 함수의 존재 이유를 런타임 플래그 하나로 되돌리는 셈이다. 내장으로 시험할
      일이 있으면 런치 인자 video_device:=/dev/videoN 으로 ★그때만★ 명시하면 된다
      (그 경로는 이 함수를 아예 거치지 않는다).

    ★순서만으로 고르지 않는 이유★ 존재하지만 죽은 노드가 실제로 있다(덮개·케이블·
    다른 프로세스 점유·적외선 노드). 순서만 믿으면 그대로 열어 검은 화면으로 주행한다.

    시리얼 장치와 달리 udev 심볼릭링크를 쓰지 않는다. `/dev/video*` 자체가 이미
    'video4linux 캡처 노드'라는 뜻이고 sysfs 로 VID/PID·index 를 바로 읽을 수 있어,
    링크를 한 겹 더 두면 오결선 위험만 늘었다 (실제로 ATTRS 부정매칭이 내장 웹캠에
    /dev/cam 을 붙이는 사고가 있었다 — 2026-08-04).
    """
    if name_hint is None:
        name_hint = os.environ.get(CAM_NAME_HINT_ENV, '')
    name_hint = (name_hint or '').strip().upper()
    # 못 찾았을 때 줄 대기 경로. ★FALLBACK_CAM 을 기본값으로 쓰지 않는다★ —
    # 그 번호가 이 기계에서는 내장 적외선 노드다(상수 주석 참고).
    if fallback is None:
        fallback = next_free_video_path()

    devices = video_capture_devices()

    def _tag(vp):
        return f"{vp[0]:04x}:{vp[1]:04x}" if vp else "VID/PID 불명"

    def _desc(entry):
        dev, vp, name = entry
        kind = {'external': '외부', 'internal': '내장', 'unknown': '불명'}[
            classify_camera(vp, name)]
        return f"{dev} ({_tag(vp)}, {kind}, '{name}')"

    def _warn_fallback(why):
        if log:
            log(f"⛔ {why}")
            log(f"   → 대기 경로 '{fallback}' 로 usb_cam 을 띄웁니다(지금은 비어 있는 "
                f"번호입니다). 카메라를 꽂으면 커널이 이 번호를 주므로 respawn 이 "
                f"그대로 붙습니다.")
            log(f"   ★꽂을 때까지 신호등 정지는 동작하지 않습니다★ — fail-open 이라 "
                f"차는 빨간불에 그냥 지나갑니다.")
        return fallback

    if not devices:
        return _warn_fallback("카메라를 하나도 찾지 못했습니다")

    # ── 2) 분류 · 내장 배제 ────────────────────────────────────────────────
    usable = []
    for entry in devices:
        if classify_camera(entry[1], entry[2]) == 'internal':
            if log:
                log(f"   🚫 제외(내장/비영상): {_desc(entry)}")
            continue
        usable.append(entry)

    if not usable:
        return _warn_fallback("쓸 수 있는 외부 카메라가 없습니다(전부 내장으로 판정)")

    # ── 3) 화이트리스트가 잡히면 그것만 본다 ────────────────────────────────
    #   ★[2026-08-12] '불명'은 자동으로 쓰지 않는다★ 블랙리스트 방식은 ★기계를 옮기는
    #   순간 무력해진다★ — 그 기계의 내장 웹캠 VID/PID 는 목록에 없으니 '불명'이 되고,
    #   불명을 후보로 받으면 결국 내장 웹캠을 연다(현 개발 노트북에서 실제로 그랬다).
    #   차량 PC 는 개발 노트북과 다른 기계라 그 목록을 미리 채울 수도 없다.
    #   → 근거 있는 장치(화이트리스트)나 사람이 직접 지정한 장치(이름 힌트·video_device)
    #     가 아니면 ★고르지 않고 기다린다★. 신호등이 안 도는 것은 fail-open 이라
    #     차가 못 가는 일은 없고, 무엇이 잘못됐는지는 아래 로그가 그대로 말해 준다.
    ext = [e for e in usable if classify_camera(e[1], e[2]) == 'external']
    hinted = [e for e in usable
              if name_hint and name_hint in (e[2] or '').upper()]
    if ext:
        pool, scope = ext, "차량 카메라(화이트리스트)"
    elif hinted:
        # 사람이 이름으로 직접 지정했다 — 화이트리스트가 없어도 그 뜻을 따른다.
        pool, scope = hinted, f"이름 힌트 '{name_hint}'"
    else:
        wl = ', '.join(f'{v:04x}:{p:04x}' for v, p in EXTERNAL_CAM_VIDPID)
        if log:
            log(f"   ⚠️ 화이트리스트({wl})에 해당하는 카메라가 없습니다. "
                f"남은 후보는 근거가 없어(불명) ★자동으로 고르지 않습니다★:")
            for e in usable:
                log(f"        · {_desc(e)}")
            log(f"   → 이 중 하나를 쓰려면 셋 중 하나를 하십시오: "
                f"① ports.py 의 EXTERNAL_CAM_VIDPID 에 VID/PID 추가(권장) "
                f"② 환경변수 {CAM_NAME_HINT_ENV}=<이름 일부> "
                f"③ 런치 인자 video_device:=<경로> (검증을 전부 건너뜁니다)")
        return _warn_fallback("차량 카메라(화이트리스트)를 찾지 못했습니다")

    def _pick(cands, why):
        entry = sorted(cands, key=lambda e: _cam_sort_key(e, name_hint))[0]
        if log:
            log(f"✅ 카메라 선택: {_desc(entry)} — {why}")
        return entry[0]

    if not probe:
        return _pick(pool, f"probe=False, {scope} 중 순위 기준")

    # ── 4) 프레임 검증 ─────────────────────────────────────────────────────
    alive, dead, unsure = [], [], []
    for entry in pool:
        verdict, detail = probe_camera(entry[0])
        if log:
            mark = {True: "✅", False: "❌", None: "❔"}[verdict]
            log(f"   {mark} {_desc(entry)}: {detail}")
        (alive if verdict is True else dead if verdict is False else unsure).append(entry)

    if alive:
        return _pick(alive, f"{scope} + 실제 영상 확인됨")
    # 판정 불가(이미 사용 중 등)는 '죽었다'가 아니다 — 죽은 것보다 우선한다
    if unsure:
        return _pick(unsure, f"{scope}, 내용 판정은 못 했지만 죽지는 않았음")
    if log:
        log("⚠️ 후보가 전부 검정/포화/동결입니다 — 렌즈 캡·케이블·노출 설정을 "
            "확인하십시오. 일단 순위 기준으로 고릅니다.")
    return _pick(dead, f"{scope}, 살아있는 카메라가 없어 순위 기준")
