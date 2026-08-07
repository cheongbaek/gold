#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ports.py — ★GPS · IMU · 카메라 장치 식별의 단일 소유자★ (Ubuntu 22.04 전용)

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
FALLBACK_CAM = '/dev/video2'

# ── ★카메라 선택 우선순위 (2026-08-05 지정)★ ──
#   외부 See3CAM 이 붙는 자리가 video2, 노트북 내장 웹캠이 video0 이다.
#   "video2 를 먼저 보고, 없으면 video0" 이 운용 규칙이므로 그 순서를 명시한다.
#   ★단 순서만으로 고르지 않는다★ — 각 후보를 실제로 열어 프레임을 보고(probe_camera)
#   살아있는 것 중에서 이 순서를 적용한다. 순서만 믿으면 '덮개 닫힌 내장 웹캠'이나
#   '동결된 스트림'을 그대로 열어 검은 화면으로 주행하게 된다(2026-08-04 실제 사고).
#   목록에 없는 장치(video4 …)는 이 뒤로 밀리며, 그들끼리는 '외부 우선 → 번호순'이다.
CAM_PREFERRED_ORDER = ('/dev/video2', '/dev/video0')

# ── 노트북 '내장' 웹캠 VID/PID ──
#   카메라는 반드시 외부 USB 카메라를 써야 한다. 내장 웹캠은 덮개를 닫으면 키보드
#   데크를 보게 되어 균일한(검은/포화) 프레임만 나온다 — 2026-08-04 실측: 컨트롤
#   기본값에서 평균 4.5/255(검정), See3CAM 기준 설정에서는 전 픽셀 255(포화).
#   video_device 를 '/dev/video0' 로 고정해 두면 외부 카메라가 없거나 나중에 열거될 때
#   조용히 내장 웹캠을 열어버린다 → 이 목록으로 내장을 '제외'하고 외부를 고른다.
INTERNAL_CAM_VIDPID = [
    (0x04F2, 0xB7F3),   # Chicony "HP True Vision FHD Camera" (이 노트북 내장)
]

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


def video_capture_devices():
    """/dev/video* 중 '영상 캡처' 노드만 [(경로, (vid,pid)|None), ...] 로 반환.

    UVC 카메라 한 대는 보통 노드를 2개 만든다 — index=0 이 영상 캡처, index=1 은
    메타데이터다. index 로 걸러야 metadata 노드를 카메라로 착각하지 않는다
    (2026-08-04 실측: video0=Video Capture/index0, video1=Metadata Capture/index1).
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
        out.append(('/dev/' + name, _sysfs_usb_vidpid(os.path.join(node, 'device'))))
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


def _cam_sort_key(entry):
    """카메라 후보 정렬 키 — ★CAM_PREFERRED_ORDER 가 1순위★.

    (우선순위 순번, 내장이면 뒤로, 장치번호) 순으로 정렬한다.
      · video2 → video0 : 지정된 운용 순서 (2026-08-05)
      · 목록에 없는 장치 : 그 뒤로 밀리고, 그들끼리는 외부 우선 → 번호순
    """
    dev, vidpid = entry
    try:
        rank = CAM_PREFERRED_ORDER.index(dev)
    except ValueError:
        rank = len(CAM_PREFERRED_ORDER)
    num = int(dev[len('/dev/video'):]) if dev[len('/dev/video'):].isdigit() else 1 << 30
    return (rank, 1 if _is_internal_cam(vidpid) else 0, num)


def resolve_camera(fallback=FALLBACK_CAM, log=None, probe=True):
    """카메라 경로를 정한다 — ★video2 → video0 순서 + 실제 프레임 검증★.

      1) 캡처 노드(/dev/video*, index=0)를 모은다 — 예: video0(내장) · video2(See3CAM)
      2) ★각 노드를 실제로 열어 프레임을 본다★ 검정/포화/동결이면 후보에서 뺀다.
         "둘 중 하나는 까맣게 나오고 나머지가 진짜 카메라"라서 이게 가장 확실하다.
      3) 살아있는 노드 중 CAM_PREFERRED_ORDER(video2 → video0) 순으로 고른다.
         목록에 없는 장치는 그 뒤로 밀리고, 그들끼리는 외부 웹캠 우선 → 번호순.
      4) 전부 죽었으면 같은 순서 규칙으로 고르고 **경고**한다
      5) 아무 카메라도 없으면 fallback 을 그대로 반환 — 나중에 꽂으면 respawn 이 붙는다

    probe=False 로 주면 프레임 판정을 생략하고 순서 규칙만 쓴다(빠르지만 덜 확실).

    ★순서만으로 고르지 않는 이유★ video2 가 '존재하지만 죽은' 경우가 실제로 있다
    (덮개·케이블·다른 프로세스 점유). 순서만 믿으면 그대로 열어 검은 화면으로 주행한다.

    시리얼 장치와 달리 udev 심볼릭링크를 쓰지 않는다. `/dev/video*` 자체가 이미
    'video4linux 캡처 노드'라는 뜻이고 sysfs 로 VID/PID·index 를 바로 읽을 수 있어,
    링크를 한 겹 더 두면 오결선 위험만 늘었다 (실제로 ATTRS 부정매칭이 내장 웹캠에
    /dev/cam 을 붙이는 사고가 있었다 — 2026-08-04).
    """
    devices = video_capture_devices()

    def _tag(vp):
        return f"{vp[0]:04x}:{vp[1]:04x}" if vp else "VID/PID 불명"

    def _pick(cands, why):
        dev, vp = sorted(cands, key=_cam_sort_key)[0]
        if log:
            log(f"✅ 카메라 선택: {dev} ({_tag(vp)}, "
                f"{'내장' if _is_internal_cam(vp) else '외부'}) — {why}")
        return dev

    if not devices:
        if log:
            log(f"⚠️ 카메라를 찾지 못했습니다 → '{fallback}' 로 계속 시도합니다 "
                f"(나중에 꽂으면 respawn 이 자동으로 붙습니다).")
        return fallback

    if probe:
        alive, dead, unknown = [], [], []
        for dev, vp in devices:
            verdict, detail = probe_camera(dev)
            if log:
                mark = {True: "✅", False: "❌", None: "❔"}[verdict]
                log(f"   {mark} {dev} ({_tag(vp)}): {detail}")
            (alive if verdict is True else dead if verdict is False else unknown
             ).append((dev, vp))

        if alive:
            return _pick(alive, "실제 영상 확인됨 (video2 우선)")
        # 판정 불가(이미 사용 중 등)는 '죽었다'가 아니다 — 죽은 것보다 우선한다
        if unknown:
            return _pick(unknown, "내용 판정은 못 했지만 죽지는 않았음")
        if log:
            log("⚠️ 모든 카메라가 검정/포화/동결로 판정됐습니다 — 렌즈 차단(덮개)·케이블·"
                "노출 설정을 확인하십시오. 일단 지정 순서(video2→video0)로 고릅니다.")
        return _pick(dead, "살아있는 카메라가 없어 지정 순서 기준")

    return _pick(devices, "probe=False, 지정 순서 기준")
