#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ports.py — ★GPS · IMU 장치 식별의 단일 소유자★ (Ubuntu 22.04 전용)

★ [white1] 카메라 코드를 전부 걷어냈다 ★ 이 스택은 GPS+IMU 만 쓴다.
  (구 white/ports.py 의 video_capture_devices · probe_camera · resolve_camera ·
   camera_formats · resolve_camera_format 과 CAM_* 상수는 여기 없다.)

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


