#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""kill.py ― ★돌고 있는 ROS2 를 한 번에 끝낸다★ [nxde] (Ubuntu 22.04 전용)

    ros2 run nxde kill              전부 강제종료 + 포트 초기화 (기본)
    ros2 run nxde kill --dry-run    무엇을 죽일지만 보여주고 아무것도 안 한다
    ros2 run nxde kill --quiet      한 줄 요약만
    ros2 run nxde kill --no-ports   프로세스만 죽이고 포트는 건드리지 않는다

════════════════════════════════════════════════════════════════════════════════
 ★ 왜 만들었나 — `ros2 launch` 의 종료가 질척거린다 ★
════════════════════════════════════════════════════════════════════════════════
  실제로 겪은 순서다(2026-09-04 one_launch.py 종료 로그):

    ① Ctrl-C 를 눌러도 launch 가 "user interrupted with ctrl-c (SIGINT) again,
       ignoring..." 를 스무 줄 뱉으며 안 내려간다. 두 번째 이후의 Ctrl-C 는
       ★설계상 무시된다★ — 더 눌러도 빨라지지 않는다.
    ② hud 는 SIGINT 5초 → SIGTERM 10초 → 결국 SIGKILL 로 죽었다(15초 낭비).
       tkinter mainloop 이 rclpy 의 신호 처리기와 엮여 어느 신호에도 안 나간다.
    ③ gps 는 `rcl_shutdown already called` RCLError 트레이스백을 쏟았다.
    ④ os_driver 는 컨텍스트가 죽은 뒤 전이(transition)를 시도해 errorprocessing.

  ②③④ 는 각 노드에서 따로 고쳤다(같은 커밋). ★그래도 이 도구가 필요하다★:
  respawn 이 걸린 노드, 터미널이 먼저 닫혀 고아가 된 노드, 다른 터미널에서 띄운
  `ros2 run` 은 launch 의 종료 절차가 아예 닿지 않는다. 그때 사람이 하는 일은 늘
  같았다 — `pkill -f`, `fuser -k /dev/ttyACM*`, `ros2 daemon stop` 을 순서대로
  치는 것. ★그 한 묶음을 한 번에 한다★

════════════════════════════════════════════════════════════════════════════════
 ★ 무엇을 죽이나 — 판정 근거 ★
════════════════════════════════════════════════════════════════════════════════
  `ros2 node list` 를 쓰지 않는다. 이유가 둘이다:
    · ★pid 를 주지 않는다★ 죽일 수가 없다. 노드 이름과 프로세스는 1:1 이 아니다
      (컴포넌트 컨테이너 하나에 노드가 여럿, 노드 없는 launch 부모 프로세스도 있다).
    · ★디스커버리에 의존한다★ 지금 고치려는 문제가 바로 그 층이 엉킨 상태다.
      DDS 가 반쯤 죽은 채로 남으면 목록 자체가 안 나오거나 유령이 뜬다.

  대신 ★/proc 를 직접 훑는다★ (같은 사용자 소유 프로세스만 — 커널이 남의 fd·maps 를
  감춘다. 우리가 정리할 잔재는 전부 같은 사용자이므로 실용상 충분하다):

    1) ★1순위 : /proc/<pid>/maps 에 ROS 런타임이 매핑돼 있는가★
       살아 있는 ROS2 프로세스는 언어와 무관하게 librcl 계열을 반드시 물고 있다
       (C++ 는 librclcpp, 파이썬은 _rclpy + librcl). ★이 판정이 가장 정확하다★ —
       실행파일 이름·경로·인자에 'ros' 가 한 글자도 없어도 걸린다.
    2) 2순위 : cmdline 이 ROS2 임을 말하는가 (`--ros-args`, /opt/ros/, 워크스페이스
       install 경로, ros2cli, _ros2_daemon). maps 가 아직 안 붙은 ★기동 직전★ 의
       프로세스와, rclpy 를 늦게 import 하는 launch 부모를 잡는다.

  ★죽이지 않는 것★
    · 자기 자신 (그래서 이 파일은 ★rclpy 를 import 하지 않는다★ — 아래 절)
    · ★자기 조상 프로세스 전부★ `ros2 run nxde kill` 의 부모인 `ros2` CLI 가 2)에
      걸리는데, 그것을 죽이면 우리 stdout 이 끊기고 셸이 먼저 돌아와 ★보고가 유실★
      된다. 조상은 우리가 끝나면 스스로 끝난다.
    · 다른 사용자·root 소유 프로세스 (권한이 없다 — 남은 것은 보고만 한다)

════════════════════════════════════════════════════════════════════════════════
 ★ 어떻게 죽이나 — SIGTERM 을 거치지 않는다 ★
════════════════════════════════════════════════════════════════════════════════
  ★처음부터 SIGKILL 이다★ 이 도구를 부르는 시점은 이미 '정상 종료가 실패한 뒤'다.
  거기서 SIGTERM 을 다시 보내고 기다리는 것은 위 ②(15초)를 한 번 더 반복하는 짓이다.
  유예를 두지 않으므로 ★사람이 기다릴 시간이 없다★ — 그것이 이 도구의 존재 이유다.

  ★순서는 있다 — launch·부모를 먼저 죽인다★ 유예가 아니라 순서 문제다.
  respawn 이 걸린 노드를 먼저 죽이면 그 부모(launch)가 즉시 되살린다. 부모를
  먼저 지우면 되살릴 주체가 사라진다. 한 루프 안에서 신호만 순서대로 보내므로
  전체가 밀리초 안에 끝난다(사이에 sleep 이 없다).

════════════════════════════════════════════════════════════════════════════════
 ★ 포트 초기화 — 무엇을 되돌리나 ★
════════════════════════════════════════════════════════════════════════════════
  SIGKILL 은 프로세스의 정리 코드를 실행하지 않는다. 커널이 fd 를 닫아 주므로
  ★점유 자체는 풀린다★ (배타 open·소켓 바인드 모두). 그런데 그것만으로 다음 런치가
  깨끗하게 뜨지 않는 자리가 셋 남는다:

    ① ★시리얼 커널 버퍼에 남은 바이트★ 죽는 순간 보드가 보내던 텔레메트리 조각이
       그대로 큐에 남는다. 다음 런치의 보드 식별(identify_port)이 그 조각을 먼저
       읽어 "형식 오류" 로 판정해 ★A/B 를 잘못 짚거나 식별을 놓친다★.
       → 열어서 termios TCIOFLUSH 로 송·수신 큐를 비운다.
    ② ★FastDDS 공유메모리 잔재★ /dev/shm/fastrtps_* 와 sem.fastrtps_*.
       정상 종료라면 지워지는데 SIGKILL 은 안 지운다. 쌓이면 디스커버리가 유령
       참가자를 보고, /dev/shm 이 가득 차면 새 노드가 아예 못 뜬다.
       → ROS 프로세스가 전부 사라진 것을 확인한 뒤에만 지운다.
       ※ ★이 기계의 RMW 는 rmw_cyclonedds_cpp 다★ (2026-09-04 실측) — Cyclone 은
         기본 설정에서 UDP 만 쓰므로 남기는 것이 없고, 이 단계는 늘 '잔재 없음' 이
         된다. 그래도 지우지 않는다 — RMW_IMPLEMENTATION 은 환경변수 하나로
         바뀌고(런치·기계마다 다르다), 그때 이 자리가 없으면 원인을 찾기 어려운
         디스커버리 고장으로 돌아온다. ★비용이 glob 두 번★ 이라 두는 편이 싸다.
    ③ ★ros2 daemon★ 죽은 노드의 목록을 캐시한 채로 남는다. 위에서 프로세스로 함께
       죽으므로 별도 처리가 없다 — 다음 `ros2` 명령이 새로 띄운다.

  ★UDP 포트(라이다 7502/7503, DDS 7400+)는 따로 할 일이 없다★ 소켓은 fd 라서 ①과
  달리 커널이 닫는 순간 완전히 풀린다. 확인만 해서 보고한다.

  ★[2026-09-04] 보드에 정지값을 쓰지 않는다★ A 펌웨어에는 통신 워치독이 없어서,
  주행 중에 이 도구로 죽이면 보드가 마지막 PWM 을 유지한다. 그 보강은 별건으로
  미뤘다(포트를 여는 이 자리가 그것을 넣을 곳이다 — write(b'0\\n') 한 줄이다).

════════════════════════════════════════════════════════════════════════════════
 ★ 이 파일은 rclpy 를 import 하지 않는다 ★
════════════════════════════════════════════════════════════════════════════════
  ROS 노드가 아니다(tts.py 와 같은 부류의 작업용 도구다). 이유가 셋이다:
    · ★자기 자신을 죽이지 않게 되는 것이 구조로 보장된다★ maps 에 librcl 이 안
      붙으므로 1순위 판정에 원리적으로 안 걸린다. 자기 pid 를 빼는 코드가
      한 줄 더 있긴 하지만, 그 한 줄이 틀려도 사고가 나지 않는다.
    · ★디스커버리에 참여하지 않는다★ 정리하려는 판에 참가자를 하나 더 넣지 않는다.
    · 뜨는 데 시간이 안 걸린다(rclpy.init 은 컨텍스트·DDS 초기화를 한다).
"""

import errno
import os
import signal
import sys
import termios
import time
import glob


# ── ROS2 런타임 판정 : /proc/<pid>/maps 에서 찾을 이름 (1순위) ────────────────
#   librcl.so  : C·C++·파이썬 공통의 최하층. 사실상 이것 하나로 충분하다.
#   나머지는 안전망이다 — 배포판이 이름을 바꾸거나 정적 링크한 경우를 위해 둔다.
ROS_LIB_HINTS = (
    'librcl.so',        # rcl (모든 클라이언트 라이브러리의 바닥)
    'librclcpp',        # C++ 노드
    '_rclpy',           # 파이썬 노드 (rclpy 의 pybind11 확장)
    'librmw',           # 미들웨어 추상층
    'libfastrtps',      # 기본 RMW 구현
)

# ── ROS2 판정 : cmdline 조각 (2순위) ─────────────────────────────────────────
#   ★maps 가 아직 안 붙은 프로세스를 잡는다★ 갓 exec 된 노드, rclpy 를 늦게
#   import 하는 launch 부모, 그리고 ros2cli 자신.
ROS_CMD_HINTS = (
    '--ros-args',       # 런치가 띄운 노드는 예외 없이 이것을 달고 있다
    '/opt/ros/',        # 배포판 실행파일
    'ros2cli',
    '_ros2_daemon',
    'ros2launch',
    'ros2/launch',
)

# ── launch·부모로 보이는 것 (먼저 죽인다 — respawn 을 끊기 위해) ──────────────
PARENT_HINTS = ('ros2launch', 'ros2/launch', 'ros2cli', '_ros2_daemon',
                ' launch ', 'bin/ros2')

# ── 초기화할 시리얼 포트 글롭 ────────────────────────────────────────────────
#   아두이노 A/B·GPS·IMU 가 전부 이 대역에 있다. ★어느 것이 무엇인지 가리지 않는다★
#   — 하는 일이 '큐 비우기' 뿐이라 어느 장치에도 무해하고, 가리려면 VID/PID 판정을
#   복제해야 한다(그것은 arduino.py·white1/ports.py 의 소유다).
SERIAL_GLOBS = ('/dev/ttyACM*', '/dev/ttyUSB*')

# 카메라·라이다는 fd 를 닫으면 끝이라 초기화할 것이 없다. 점유 확인만 한다.
DEVICE_GLOBS = ('/dev/video*',)

# FastDDS 공유메모리 잔재 (정상 종료면 스스로 지운다)
SHM_GLOBS = ('/dev/shm/fastrtps_*', '/dev/shm/sem.fastrtps_*')

# 죽은 것을 확인하는 데 쓰는 총 대기시간 [s]. SIGKILL 이므로 커널이 즉시 처리하고,
# 이 시간은 '좀비를 부모가 거둘 때까지' 를 보는 것이다 — 길 필요가 없다.
REAP_WAIT_S = 0.4
REAP_POLL_S = 0.02


# ═══════════════════════════════════════════════════════════════════════════
#  /proc 읽기
# ═══════════════════════════════════════════════════════════════════════════
def _read(path):
    try:
        with open(path, 'rb') as f:
            return f.read()
    except OSError:
        return b''


def _cmdline(pid):
    raw = _read(f'/proc/{pid}/cmdline')
    return raw.replace(b'\0', b' ').decode('utf-8', 'replace').strip()


def _comm(pid):
    return _read(f'/proc/{pid}/comm').decode('utf-8', 'replace').strip() or '?'


def _ppid(pid):
    """부모 pid. 읽을 수 없으면 0."""
    for line in _read(f'/proc/{pid}/status').decode('utf-8', 'replace').splitlines():
        if line.startswith('PPid:'):
            try:
                return int(line.split()[1])
            except (IndexError, ValueError):
                return 0
    return 0


def _maps_has_ros(pid):
    """maps 에 ROS 런타임이 매핑돼 있는가. ★1순위 판정★

    maps 는 프로세스마다 수백~수천 줄이라 통째로 읽으면 낭비다. 라이브러리 매핑은
    파일 경로가 붙은 줄에만 있으므로 조각 검사로 끝낸다.
    """
    try:
        with open(f'/proc/{pid}/maps', 'rb') as f:
            for line in f:
                if b'.so' not in line:
                    continue
                low = line.lower()
                for hint in ROS_LIB_HINTS:
                    if hint.encode('ascii') in low:
                        return True
    except OSError:
        return False
    return False


def _ancestors(pid):
    """자기 조상 pid 전부. ★이들은 죽이지 않는다★ (헤더 '죽이지 않는 것' 절)"""
    out = set()
    cur = _ppid(pid)
    while cur > 1 and cur not in out:
        out.add(cur)
        cur = _ppid(cur)
    return out


def _my_prefixes():
    """이 워크스페이스의 install 접두어들 — 2순위 판정에 쓴다.

    AMENT_PREFIX_PATH 에는 /opt/ros/humble 과 함께 <ws>/install/<pkg> 들이 들어 있다.
    노드 실행파일의 절대경로가 그 아래이므로, 이것으로 ★이 워크스페이스가 띄운 것★
    을 이름 없이도 알아본다.
    """
    out = []
    for key in ('AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH'):
        for part in os.environ.get(key, '').split(os.pathsep):
            part = part.strip()
            if len(part) > 1 and part not in out:
                out.append(part)
    return out


def find_ros_processes():
    """죽일 대상 목록. [(pid, comm, cmdline, 판정근거)] — launch·부모가 앞에 온다."""
    me = os.getpid()
    skip = {me, 1} | _ancestors(me)
    prefixes = _my_prefixes()

    found = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid in skip:
            continue
        cmd = _cmdline(pid)
        if not cmd:
            continue          # 커널 스레드
        why = ''
        if _maps_has_ros(pid):
            why = 'maps'
        else:
            low = cmd.lower()
            if any(h in low for h in ROS_CMD_HINTS):
                why = 'cmdline'
            elif any(p in cmd for p in prefixes):
                why = 'prefix'
        if not why:
            continue
        found.append((pid, _comm(pid), cmd, why))

    def sort_key(item):
        low = item[2].lower()
        return (0 if any(h in low for h in PARENT_HINTS) else 1, item[0])

    found.sort(key=sort_key)
    return found


def alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True           # 살아 있지만 우리 소유가 아니다


# ═══════════════════════════════════════════════════════════════════════════
#  죽이기
# ═══════════════════════════════════════════════════════════════════════════
def kill_all(targets):
    """SIGKILL 을 순서대로 ★유예 없이★ 보낸다. (죽은 수, 못 죽인 목록)"""
    denied, missing = [], 0
    for pid, comm, _cmd, _why in targets:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            missing += 1      # 부모를 먼저 지워 함께 사라졌다 — 정상이다
        except PermissionError:
            denied.append((pid, comm))

    # 좀비를 부모가 거둘 시간만 준다(유예가 아니다 — 확인이다).
    deadline = time.monotonic() + REAP_WAIT_S
    while time.monotonic() < deadline:
        if not any(alive(t[0]) for t in targets):
            break
        time.sleep(REAP_POLL_S)

    survived = [(pid, comm) for pid, comm, _c, _w in targets if alive(pid)]
    return missing, denied, survived


# ═══════════════════════════════════════════════════════════════════════════
#  포트 초기화
# ═══════════════════════════════════════════════════════════════════════════
def device_holders(path):
    """이 장치 노드를 fd 로 물고 있는 프로세스 [(pid, comm)].

    arduino.py 의 port_holders 와 같은 방식이다. ★거기서 import 하지 않는다★ —
    이 도구는 pyserial 이 없거나 arduino.py 가 import 에 실패하는 상황(정리하러
    부른 상황이 대개 그렇다)에서도 반드시 돌아야 한다.
    """
    try:
        target = os.path.realpath(path)
    except OSError:
        return []
    me = os.getpid()
    out = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == me:
            continue
        fd_dir = f'/proc/{entry}/fd'
        try:
            fds = os.listdir(fd_dir)
        except OSError:
            continue
        for fd in fds:
            try:
                if os.path.realpath(os.path.join(fd_dir, fd)) != target:
                    continue
            except OSError:
                continue
            out.append((pid, _comm(pid)))
            break
    return out


def flush_serial(path):
    """시리얼 송·수신 큐를 비운다. (성공?, 메시지)

    ★왜 여는가★ 헤더 '포트 초기화' ①절 — 죽는 순간 큐에 남은 텔레메트리 조각이
    다음 런치의 보드 식별을 망친다. O_NONBLOCK 으로 여는 것이 요점이다: 그것 없이는
    모뎀 제어선을 기다리며 open 이 블로킹될 수 있다(DCD 가 없는 장치).
    """
    fd = None
    try:
        fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        termios.tcflush(fd, termios.TCIOFLUSH)
        return True, '큐 비움'
    except OSError as e:
        if e.errno in (errno.EACCES, errno.EPERM):
            return False, '권한 없음(dialout 그룹 확인)'
        if e.errno == errno.EBUSY or e.errno == errno.EAGAIN:
            return False, '아직 점유 중'
        if e.errno == errno.ENOENT:
            return False, '없음'
        return False, f'{e.strerror}'
    finally:
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass


def clear_shm(log):
    """FastDDS 공유메모리 잔재를 지운다. 지운 개수.

    ★ROS 프로세스가 전부 사라진 뒤에만 부른다★ 살아 있는 참가자의 세그먼트를
    지우면 그쪽이 디스커버리 도중에 깨진다 — 정리가 아니라 고장이다.
    """
    n = 0
    for pattern in SHM_GLOBS:
        for path in sorted(glob.glob(pattern)):
            try:
                os.unlink(path)
                n += 1
            except OSError as e:
                log(f"    · {path} 삭제 실패: {e.strerror}")
    return n


def reset_ports(log, quiet):
    """시리얼 큐 비우기 + 남은 점유 보고 + 공유메모리 정리. 요약 문자열."""
    parts = []

    # ── 시리얼 ────────────────────────────────────────────────────────────
    flushed, stuck = [], []
    for pattern in SERIAL_GLOBS:
        for path in sorted(glob.glob(pattern)):
            holders = device_holders(path)
            if holders:
                stuck.append((path, holders))
                continue
            ok, msg = flush_serial(path)
            if ok:
                flushed.append(path)
            else:
                stuck.append((path, [(0, msg)]))
    if flushed:
        parts.append(f"시리얼 {len(flushed)}개 초기화")
        if not quiet:
            log(f"  🔌 시리얼 큐 비움 : {', '.join(flushed)}")
    for path, holders in stuck:
        who = ', '.join(f"{c}(pid {p})" if p else c for p, c in holders)
        log(f"  ⚠️  {path} — {who}")

    # ── 장치 노드 점유 확인 (닫히면 끝이라 초기화할 것이 없다) ─────────────
    busy = []
    for pattern in DEVICE_GLOBS:
        for path in sorted(glob.glob(pattern)):
            holders = device_holders(path)
            if holders:
                busy.append((path, holders))
    for path, holders in busy:
        who = ', '.join(f"{c}(pid {p})" for p, c in holders)
        log(f"  ⚠️  {path} 를 아직 물고 있습니다 — {who}")
    if not busy and not quiet:
        log("  📷 /dev/video* 점유 없음")

    # ── 공유메모리 ────────────────────────────────────────────────────────
    n = clear_shm(log)
    if n:
        parts.append(f"공유메모리 {n}개 정리")
        if not quiet:
            log(f"  🧹 FastDDS 공유메모리 잔재 {n}개 삭제")
    elif not quiet:
        log("  🧹 FastDDS 공유메모리 잔재 없음")

    return ', '.join(parts) if parts else '초기화할 것 없음'


# ═══════════════════════════════════════════════════════════════════════════
#  main
# ═══════════════════════════════════════════════════════════════════════════
USAGE = """사용법: ros2 run nxde kill [옵션]

  (없음)      돌고 있는 ROS2 프로세스를 전부 SIGKILL 하고 포트를 초기화한다
  --dry-run   무엇을 죽일지만 보여주고 ★아무것도 하지 않는다★
  --no-ports  프로세스만 죽이고 포트 초기화는 건너뛴다
  --quiet     한 줄 요약만 낸다
  --help      이 도움말
"""


def main(args=None):
    argv = list(sys.argv[1:] if args is None else args)
    if '--help' in argv or '-h' in argv:
        print(USAGE)
        return 0
    dry = '--dry-run' in argv or '-n' in argv
    quiet = '--quiet' in argv or '-q' in argv
    do_ports = '--no-ports' not in argv
    unknown = [a for a in argv if a not in
               ('--dry-run', '-n', '--quiet', '-q', '--no-ports', '--help', '-h')]
    if unknown:
        print(f"모르는 인자: {' '.join(unknown)}\n\n{USAGE}", file=sys.stderr)
        return 2

    def log(msg):
        print(msg, flush=True)

    targets = find_ros_processes()

    if not targets:
        log("✅ 돌고 있는 ROS2 프로세스가 없습니다.")
        if do_ports and not dry:
            summary = reset_ports(log, quiet)
            log(f"🔌 포트 초기화 — {summary}")
        return 0

    if not quiet:
        log(f"🔎 ROS2 프로세스 {len(targets)}개 (★launch·부모가 위★):")
        for pid, comm, cmd, why in targets:
            short = cmd if len(cmd) <= 96 else cmd[:93] + '...'
            log(f"  {pid:>7}  {comm:<16} [{why:<7}] {short}")

    if dry:
        log(f"\n🅰 --dry-run — 아무것도 하지 않았습니다 "
            f"({len(targets)}개가 대상이었습니다).")
        return 0

    missing, denied, survived = kill_all(targets)
    killed = len(targets) - len(survived) - missing

    if not quiet:
        log(f"\n💀 SIGKILL {killed}개 종료"
            + (f", {missing}개는 부모와 함께 이미 사라짐" if missing else ""))
    for pid, comm in denied:
        log(f"  ⚠️  pid {pid} ({comm}) — 권한이 없습니다(다른 사용자/root 소유)")
    for pid, comm in survived:
        log(f"  ⚠️  pid {pid} ({comm}) — ★아직 살아 있습니다★ "
            f"(root 소유라면: sudo kill -9 {pid})")

    port_summary = ''
    if do_ports:
        if survived:
            # ★살아남은 ROS 프로세스가 있으면 공유메모리를 지우지 않는다★
            #   clear_shm 주석 참고 — 그쪽 세그먼트를 지우는 것은 고장이다.
            log("  ⏭  살아남은 프로세스가 있어 포트 초기화를 건너뜁니다 "
                "(위 pid 를 먼저 정리하십시오)")
        else:
            if not quiet:
                log("")
            port_summary = reset_ports(log, quiet)

    if quiet:
        log(f"💀 {killed}개 종료" + (f" / {port_summary}" if port_summary else ""))
    elif port_summary:
        log(f"🔌 포트 초기화 — {port_summary}")

    if survived or denied:
        return 1
    if not quiet:
        log("✅ 정리 완료 — 다시 런치해도 됩니다.")
    # ★자신도 즉시 끝낸다★ 이 시점에 우리가 붙잡고 있는 자원은 없다.
    return 0


if __name__ == '__main__':
    sys.exit(main())
