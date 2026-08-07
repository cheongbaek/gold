#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check — ★런치 전 하드웨어 연결 점검(누드테스트)★

    ros2 run nxde check                 # 전체 점검
    ros2 run nxde check --no-camera     # 카메라 건너뛰기 (영상 판정이 제일 오래 걸린다)
    ros2 run nxde check --quick         # 포트를 열지 않고 VID/PID 만 본다 (즉시 끝남)

═══════════════════════════════════════════════════════════════════════════════
 왜 필요한가
═══════════════════════════════════════════════════════════════════════════════
 런치를 띄운 뒤에 장치를 꽂으면 **경합이 생긴다.** GPS·IMU·아두이노 A/B 가 모두
 /dev/ttyACM*·/dev/ttyUSB* 대역을 나눠 쓰는데, 런치 시점에 없던 장치는 각 노드가
 자기 방식으로 뒤늦게 찾으러 다니고(재스캔·respawn) 그 과정에서 서로의 포트를
 배타 open 으로 밀어낸다. 특히 nmea 드라이버는 respawn 으로 계속 다시 뜨는데
 아두이노 탐색이 GPS 포트를 5초씩 물면 서로 싸운다(RTK 가 안 붙는 증상).

 → **꽂고, 이걸 돌려서 초록불을 보고, 그 다음에 런치를 띄운다.** 그러면 그 경합이
   구조적으로 생기지 않는다.

═══════════════════════════════════════════════════════════════════════════════
 이 파일은 ★자립형★ 이다 — 어떤 패키지도 import 하지 않는다
═══════════════════════════════════════════════════════════════════════════════
 rclpy 조차 쓰지 않는다(ROS 그래프가 없어도 돌아야 하는 진단 도구다).

 ⚠️ 그래서 VID/PID 표가 두 곳에 있다. 운용상의 소유자는 각각 따로다:
      · GPS · IMU · 카메라 해석 → white/white/ports.py
      · 아두이노 탐색           → nxde/nxde/arduino.py
    이 파일은 그 둘을 '보고'만 하는 사본이며, white 를 import 하면 nxde ↔ white
    순환의존이 생겨 colcon 빌드 순서가 깨지기 때문에 일부러 복제했다.
    ★하드웨어를 바꾸면 세 곳을 함께 고쳐야 한다★ (아래 표에도 같은 경고를 달아 둠).

═══════════════════════════════════════════════════════════════════════════════
 무엇을 보고하는가
═══════════════════════════════════════════════════════════════════════════════
   A보드      Arduino Mega  텔레메트리 "S," (인휠 PID + 주행펄스)
   B보드      Arduino Mega  텔레메트리 "P," (조향 + 제동 + 모드스위치)
   조이스틱   Arduino Mega  텔레메트리 "J," 또는 "U,"   ※ 없어도 정상(선택 장치)
   GPS        u-blox        ★NMEA GGA 의 fix quality 를 실제로 읽어 RTK 여부를 본다★
   IMU        iAHRS/CP210x  포트 열림 + 데이터 유입
   카메라     /dev/video*   ★실제 프레임을 찍어 검정/포화/동결을 판정★

 ⚠️ 이 도구는 포트를 '열어서' 확인한다 (--quick 이 아니면).
    · 아두이노는 포트를 열 때 자동리셋이 걸린다 — 확인에 5초쯤 걸리는 이유다.
    · 끝나면 모두 닫는다. 그래도 런치는 이게 완전히 끝난 뒤에 띄우는 것이 안전하다.
"""

import argparse
import os
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except Exception:
    serial = None
    list_ports = None


# ══════════════════════════════════════════════════════════════════════════════
#  장치 표  ★white/ports.py · nxde/arduino.py 와 함께 고쳐야 한다 (헤더 경고 참고)★
# ══════════════════════════════════════════════════════════════════════════════
GPS_VIDPID = [
    (0x1546, 0x01A9),   # u-blox 9 계열
    (0x1546, 0x01A8),   # u-blox 8 계열
]
IMU_VIDPID = [
    (0x10C4, 0xEA60),   # iAHRS / CP210x
]
ARDUINO_VIDS = {0x2341, 0x1A86, 0x2A03}   # 정품 / CH340 클론 / Arduino LLC

INTERNAL_CAM_VIDPID = [
    (0x04F2, 0xB7F3),   # 노트북 내장 Chicony 웹캠
]
CAM_PREFERRED_ORDER = ('/dev/video2', '/dev/video0')

# udev 심볼릭링크 (99-white.rules)
SYMLINKS = ('/dev/gps', '/dev/imu', '/dev/kasa_a', '/dev/kasa_b')

BAUD = 115200
ARDUINO_READ_S = 5.0    # 자동리셋 + 부트로더 대기를 감안한 식별 시간
GPS_READ_S     = 4.0    # GGA 는 보통 1Hz 이므로 몇 초는 봐야 한다
IMU_READ_S     = 1.5

# 영상 판정 임계값 (white/ports.py 와 동일 — 2026-08-04 실측 기준)
CAM_MIN_MEAN, CAM_MAX_MEAN = 8.0, 247.0
CAM_MIN_STD, CAM_MIN_DIFF  = 2.0, 0.1

# GGA fix quality → 사람이 읽는 이름
#   ★4/5 가 RTK 다★ nmea_navsat_driver 는 4/5/9 를 모두 NavSatStatus.STATUS_GBAS_FIX(2) 로
#   매핑하므로, ROS 쪽에서는 4(고정해)와 5(부동해)를 구분할 수 없다. 여기서는 원값을 본다.
GGA_QUALITY = {
    0: ("❌", "측위 불가 (fix 없음)"),
    1: ("🟡", "단독 GPS (오차 ~2-5m) — RTK 아님"),
    2: ("🟠", "DGPS/SBAS 보정 — RTK 아님"),
    3: ("🟠", "PPS"),
    4: ("🟢", "★RTK Fixed★ (고정해, 오차 ~2cm)"),
    5: ("🟡", "RTK Float (부동해, 오차 ~수십cm)"),
    6: ("🟠", "추측항법(INS)"),
    7: ("🟠", "수동 입력"),
    8: ("🟠", "시뮬레이션"),
    9: ("🟠", "SBAS"),
}


def _hr(ch='─', n=78):
    return ch * n


def _pad(text, width):
    """한글(전각) 폭을 감안해 오른쪽을 공백으로 채운다.

    '%-20s' 는 ★문자 수★ 로 세므로 한글이 섞이면 표가 어긋난다 — 한글 한 글자는
    터미널에서 두 칸을 먹는다. east_asian_width 로 실제 표시폭을 계산한다.
    """
    import unicodedata
    w = sum(2 if unicodedata.east_asian_width(c) in ('W', 'F') else 1 for c in text)
    return text + ' ' * max(0, width - w)


def _tag(vidpid):
    return f"{vidpid[0]:04x}:{vidpid[1]:04x}" if vidpid else "VID/PID 불명"


# ══════════════════════════════════════════════════════════════════════════════
#  시리얼
# ══════════════════════════════════════════════════════════════════════════════
def comports():
    if list_ports is None:
        return []
    try:
        return sorted(list_ports.comports(), key=lambda p: p.device)
    except Exception:
        return []


def classify(port):
    """포트를 VID/PID 로 1차 분류. ('gps'|'imu'|'arduino'|'unknown')"""
    vp = (port.vid, port.pid)
    if vp in GPS_VIDPID:
        return 'gps'
    if vp in IMU_VIDPID:
        return 'imu'
    if port.vid in ARDUINO_VIDS:
        return 'arduino'
    desc = (port.description or '').lower()
    if 'arduino' in desc or 'ch340' in desc:
        return 'arduino'
    return 'unknown'


def open_port(dev, timeout=0.2):
    try:
        return serial.Serial(dev, BAUD, timeout=timeout, exclusive=True)
    except Exception as e:
        return e


def read_lines(ser, seconds):
    """seconds 동안 읽어 완성된 줄들을 yield 한다."""
    buf = b''
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        try:
            data = ser.read(256)
        except Exception:
            return
        if not data:
            continue
        buf += data
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            yield line.decode('ascii', errors='ignore').strip()


def identify_arduino(dev):
    """아두이노 포트를 열어 텔레메트리 접두어로 역할을 식별.

    반환 (role, detail):
      role = 'A' | 'B' | 'JOY' | None
    """
    ser = open_port(dev)
    if isinstance(ser, Exception):
        return None, f"열기 실패: {ser}"
    try:
        for text in read_lines(ser, ARDUINO_READ_S):
            if text.startswith('S,'):
                return 'A', f"텔레메트리 '{text[:40]}'"
            if text.startswith('P,'):
                return 'B', f"텔레메트리 '{text[:40]}'"
            if text.startswith('J,') or text.startswith('U,'):
                return 'JOY', f"텔레메트리 '{text[:40]}'"
        return None, (f"{ARDUINO_READ_S:.0f}초 동안 'S,'/'P,'/'J,' 접두어가 안 나왔습니다 "
                      f"— 펌웨어가 안 올라갔거나 다른 아두이노일 수 있습니다")
    finally:
        try:
            ser.close()
        except Exception:
            pass


def probe_gps(dev):
    """GPS 포트를 열어 NMEA GGA 를 읽고 fix quality 를 보고한다.

    ★RTK 진단의 핵심★ 반환 (ok, detail).
    """
    ser = open_port(dev)
    if isinstance(ser, Exception):
        return False, f"열기 실패: {ser}"
    try:
        sentences = 0
        best = None            # (quality, sats, hdop)
        talkers = set()
        for text in read_lines(ser, GPS_READ_S):
            if not text.startswith('$'):
                continue
            sentences += 1
            kind = text[1:6]
            talkers.add(kind)
            if not kind.endswith('GGA'):
                continue
            f = text.split(',')
            if len(f) < 9:
                continue
            try:
                quality = int(f[6]) if f[6] else 0
                sats = int(f[7]) if f[7] else 0
                hdop = float(f[8]) if f[8] else float('nan')
            except ValueError:
                continue
            if best is None or quality > best[0]:
                best = (quality, sats, hdop)

        if sentences == 0:
            return False, ("NMEA 문장이 하나도 안 나왔습니다 — 보드레이트(115200)나 "
                           "수신기 출력 설정을 확인하세요")
        if best is None:
            return False, (f"NMEA {sentences}문장은 오는데 ★GGA 가 없습니다★ "
                           f"(관측: {','.join(sorted(talkers))}). "
                           f"GGA 가 없으면 RTK 여부를 알 수 없고, nmea_navsat_driver 도 "
                           f"RMC 만으로는 status 를 FIX/NO_FIX 로만 채웁니다 "
                           f"→ u-center 에서 GGA 출력을 켜십시오")

        quality, sats, hdop = best
        mark, label = GGA_QUALITY.get(quality, ("❓", f"알 수 없는 quality={quality}"))
        detail = f"{mark} {label} | 위성 {sats}개 | HDOP {hdop:.2f} | NMEA {sentences}문장"
        # RTK(4/5)가 아니면 '연결은 됐지만 정밀도는 아직'이다 — 연결 점검 자체는 통과시킨다
        return True, detail
    finally:
        try:
            ser.close()
        except Exception:
            pass


def probe_imu(dev):
    """IMU 포트를 열어 데이터가 흐르는지만 본다.

    ★침묵이 곧 고장은 아니다★ iAHRS 는 'so=1' 로 스트리밍을 켜준 적이 없으면 조용하다.
    그 설정은 iahrs 노드가 연결할 때 보낸다. 여기서는 아무것도 쓰지 않는다(상태를
    바꾸지 않기 위해) — 그래서 '데이터 없음'은 참고 정보로만 보고한다.
    """
    ser = open_port(dev)
    if isinstance(ser, Exception):
        return False, f"열기 실패: {ser}"
    try:
        lines = [t for t in read_lines(ser, IMU_READ_S) if t]
        if not lines:
            return True, ("포트 열림 (데이터 없음 — iAHRS 는 스트리밍을 켜주기 전에는 "
                          "조용합니다. 정상일 수 있습니다)")
        sample = lines[-1]
        fields = len(sample.split(','))
        note = " ★9필드 = 가속3+각속3+오일러3 정상★" if fields == 9 else ""
        return True, f"포트 열림, 데이터 유입 {len(lines)}줄 ({fields}필드){note}"
    finally:
        try:
            ser.close()
        except Exception:
            pass


# ══════════════════════════════════════════════════════════════════════════════
#  카메라
# ══════════════════════════════════════════════════════════════════════════════
def _sysfs_usb_vidpid(start_path):
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
    """'영상 캡처' 노드만 [(경로, vidpid|None), ...]. index=1 은 메타데이터라 제외."""
    base = '/sys/class/video4linux'
    try:
        names = os.listdir(base)
    except OSError:
        return []

    def _num(name):
        d = name[5:]
        return int(d) if d.isdigit() else 1 << 30

    out = []
    for name in sorted((n for n in names if n.startswith('video')), key=_num):
        node = os.path.join(base, name)
        try:
            with open(os.path.join(node, 'index')) as f:
                if f.read().strip() != '0':
                    continue
        except OSError:
            continue
        out.append(('/dev/' + name, _sysfs_usb_vidpid(os.path.join(node, 'device'))))
    return out


def probe_camera(dev, frames=6, warmup=3, timeout_s=4.0):
    """실제 프레임을 찍어 판정. (verdict, detail) — verdict True/False/None."""
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

        grays = [f.mean(axis=2) for f in grabbed[warmup:]]
        mean = float(np.mean([g.mean() for g in grays]))
        std = float(np.mean([g.std() for g in grays]))
        diff = float(np.mean([
            np.abs(grays[i].astype(int) - grays[i - 1].astype(int)).mean()
            for i in range(1, len(grays))
        ])) if len(grays) > 1 else 0.0

        stat = f"평균={mean:.1f} std={std:.1f} 프레임간차이={diff:.4f}"
        if diff < CAM_MIN_DIFF:
            return False, f"{stat} → 동결(렌즈 차단/스트림 정지)"
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


def camera_pixel_formats(dev):
    """v4l2-ctl 로 지원 포맷 요약 (usb_cam 즉사 원인 진단용). 실패하면 None."""
    import re
    import subprocess
    try:
        out = subprocess.run(['v4l2-ctl', '--list-formats-ext', '-d', dev],
                             capture_output=True, text=True, timeout=5.0).stdout
    except Exception:
        return None
    fourccs = re.findall(r"\[\d+\]:\s*'(\w+)'", out)
    has_1080 = '1920x1080' in out
    if not fourccs:
        return None
    return f"{'/'.join(fourccs)}{' · 1920x1080 지원' if has_1080 else ' · 1920x1080 미지원'}"


# ══════════════════════════════════════════════════════════════════════════════
#  본체
# ══════════════════════════════════════════════════════════════════════════════
def run(args):
    print()
    print(_hr('═'))
    print(" 🔌 하드웨어 연결 점검 (nxde check)")
    print(_hr('═'))

    if serial is None:
        print(" ❌ pyserial 이 없습니다 — `sudo apt install python3-serial`")
        return 2

    # ── udev 심볼릭링크 ──────────────────────────────────────────────
    print("\n[udev 심볼릭링크]  99-white.rules")
    links = {}
    for link in SYMLINKS:
        if os.path.exists(link):
            real = os.path.realpath(link)
            links[link] = real
            print(f"  ✅ {link:14s} → {real}")
        else:
            print(f"  ·  {link:14s} 없음")
    if not links:
        print("     ⚠️ 링크가 하나도 없습니다. 장치를 안 꽂았거나 udev 규칙이 없습니다.")
        print("        규칙 확인: ls -l /etc/udev/rules.d/99-white.rules")

    # ── 시리얼 포트 열거 ─────────────────────────────────────────────
    ports_all = comports()
    serial_ports = [p for p in ports_all
                    if ('ACM' in p.device) or ('USB' in p.device)]

    print(f"\n[시리얼 포트]  {len(serial_ports)}개 발견")
    if not serial_ports:
        print("  ❌ /dev/ttyACM* · /dev/ttyUSB* 가 없습니다 — USB 케이블·전원을 확인하세요.")

    found = {'A': None, 'B': None, 'JOY': None, 'gps': None, 'imu': None}
    details = {}

    for p in serial_ports:
        kind = classify(p)
        desc = (p.description or '').strip()
        print(f"\n  {p.device}  [{_tag((p.vid, p.pid))}]  {desc}")

        if args.quick:
            label = {'gps': 'GPS(u-blox)', 'imu': 'IMU(iAHRS/CP210x)',
                     'arduino': 'Arduino Mega 계열', 'unknown': '미분류'}[kind]
            print(f"     → {label}  (--quick: 포트를 열지 않음)")
            if kind in ('gps', 'imu') and found[kind] is None:
                found[kind] = p.device
            continue

        if kind == 'gps':
            ok, detail = probe_gps(p.device)
            print(f"     → GPS(u-blox): {detail}")
            if ok and found['gps'] is None:
                found['gps'] = p.device
                details['gps'] = detail
        elif kind == 'imu':
            ok, detail = probe_imu(p.device)
            print(f"     → IMU(iAHRS): {detail}")
            if ok and found['imu'] is None:
                found['imu'] = p.device
                details['imu'] = detail
        elif kind == 'arduino':
            print(f"     → Arduino Mega 계열 — 역할 식별 중 "
                  f"(포트 자동리셋 때문에 최대 {ARDUINO_READ_S:.0f}초)...")
            role, detail = identify_arduino(p.device)
            if role:
                name = {'A': 'A보드 (인휠 PID + 주행펄스)',
                        'B': 'B보드 (조향 + 제동 + 모드스위치)',
                        'JOY': '조이스틱 보드'}[role]
                print(f"     → ✅ {name}: {detail}")
                if found[role] is None:
                    found[role] = p.device
                    details[role] = detail
                else:
                    print(f"     ⚠️ {name} 가 이미 {found[role]} 로 잡혔습니다 — 중복 연결?")
            else:
                print(f"     → ❌ 역할 불명: {detail}")
        else:
            print(f"     → 미분류 장치 (아두이노/GPS/IMU 어느 표에도 없음)")

    # ── 카메라 ──────────────────────────────────────────────────────
    cam_pick, cam_alive = None, []
    if args.no_camera:
        print("\n[카메라]  --no-camera 로 건너뜀")
    else:
        devices = video_capture_devices()
        print(f"\n[카메라]  캡처 노드 {len(devices)}개 "
              f"(우선순위 {' → '.join(CAM_PREFERRED_ORDER)})")
        if not devices:
            print("  ❌ /dev/video* 캡처 노드가 없습니다 — USB 카메라를 확인하세요.")
        for dev, vp in devices:
            internal = vp in INTERNAL_CAM_VIDPID
            verdict, detail = probe_camera(dev)
            mark = {True: "✅", False: "❌", None: "❔"}[verdict]
            fmts = camera_pixel_formats(dev)
            print(f"  {mark} {dev}  [{_tag(vp)}]  {'내장' if internal else '외부'}")
            print(f"       {detail}")
            if fmts:
                print(f"       포맷: {fmts}")
            if verdict is True:
                cam_alive.append((dev, vp))

        def _rank(entry):
            d, vp = entry
            try:
                r = CAM_PREFERRED_ORDER.index(d)
            except ValueError:
                r = len(CAM_PREFERRED_ORDER)
            n = int(d[len('/dev/video'):]) if d[len('/dev/video'):].isdigit() else 1 << 30
            return (r, 1 if vp in INTERNAL_CAM_VIDPID else 0, n)

        if cam_alive:
            cam_pick = sorted(cam_alive, key=_rank)[0][0]
            print(f"  → one_launch.py 는 ★{cam_pick}★ 을 고를 것입니다.")

    # ── 요약 ────────────────────────────────────────────────────────
    print()
    print(_hr('═'))
    print(" 요약")
    print(_hr('═'))

    rows = [
        ("A보드 (주행)",   found['A'],   True),
        ("B보드 (조향/제동)", found['B'], True),
        ("GPS (u-blox)",   found['gps'], True),
        ("IMU (iAHRS)",    found['imu'], True),
        ("카메라",          cam_pick,     not args.no_camera),
        ("조이스틱",        found['JOY'], False),   # 선택 장치
    ]
    missing_required = []
    for label, dev, required in rows:
        cell = _pad(label, 22)
        if dev:
            print(f"  ✅ {cell} {dev}")
        elif args.no_camera and label == "카메라":
            # ★이 분기가 'not required' 보다 먼저 와야 한다★ --no-camera 면 rows 의
            #   required 가 False 로 들어오므로, 순서를 바꾸면 '선택 장치'로 잘못 표시된다.
            print(f"  ·  {cell} 건너뜀 (--no-camera)")
        elif not required:
            print(f"  ·  {cell} 없음 (선택 장치 — 없어도 정상)")
        else:
            print(f"  ❌ {cell} ★없음★")
            missing_required.append(label)

    if args.quick:
        print("\n  ※ --quick 이라 A/B 역할과 GPS RTK 상태는 확인하지 않았습니다.")

    print()
    if missing_required:
        print(f"  ❌ 필수 장치 {len(missing_required)}개 미확인: "
              f"{', '.join(missing_required)}")
        print("     연결·전원·USB 케이블을 확인한 뒤 다시 실행하십시오.")
        rc = 1
    else:
        print("  ✅ 필수 장치가 모두 확인되었습니다. 런치를 띄워도 됩니다:")
        print("       터미널 1 :  ros2 launch white one_launch.py   ← 하드웨어 + 자율주행")
        print("       터미널 2 :  ros2 run    white prompt          ← CLI 메뉴")
        rc = 0

    if not args.quick:
        print("\n  ⚠️ 이 점검은 포트를 열었다 닫았습니다. 아두이노는 그때 자동리셋이 걸리므로")
        print("     런치는 ★이 명령이 끝난 뒤★ 띄우십시오(지금 띄우면 됩니다).")
    print(_hr('═'))
    print()
    return rc


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog='ros2 run nxde check',
        description='런치 전 하드웨어 연결 점검 — 보고하고 종료한다.')
    ap.add_argument('--quick', action='store_true',
                    help='포트를 열지 않고 VID/PID 만 본다 (즉시 끝나지만 A/B 역할·RTK 미확인)')
    ap.add_argument('--no-camera', action='store_true',
                    help='카메라 판정을 건너뛴다 (영상 판정이 가장 오래 걸린다)')
    # ros2 run 이 붙이는 --ros-args 뒤쪽은 이 도구가 쓰지 않으므로 무시한다
    argv = [a for a in (argv if argv is not None else sys.argv[1:])]
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    args = ap.parse_args(argv)

    try:
        return run(args)
    except KeyboardInterrupt:
        print("\n중단됨.")
        return 130


if __name__ == '__main__':
    sys.exit(main())
