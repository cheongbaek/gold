# proc_guard : 부모 프로세스가 죽으면 이 프로세스도 정리하고 스스로 종료한다 (고아 방지)
#
# ★ 이 파일은 POSIX(Ubuntu) 전용판이다 ★
#   kasa_ws 쪽 원본에는 Windows 분기(OpenProcess/WaitForSingleObject)가 함께 들어 있다.
#   이 워크스페이스는 Ubuntu 22.04 전용이라 그 분기를 걷어냈다. Windows에서 돌릴 일이
#   생기면 kasa_ws/src/nxde/nxde/proc_guard.py 를 그대로 가져오면 된다.
#
# ★ 왜 필요한가 ★
#   ROS2 launch 는 자식 노드에 SIGINT 를 보내 내리지만, 아래 두 경우에 이 노드가 살아남아
#   시리얼 포트를 물고 마지막 명령을 계속 재전송할 수 있다:
#     - 런치를 띄운 터미널이 먼저 닫히는 경우
#     - 다른 노드의 on_exit → Shutdown 경로에서 정리가 어긋나는 경우
#   그 상태가 되면 (1) 다음 런치가 /dev/ttyACM* 을 못 잡고 (2) 마지막 명령이 주행값이면
#   차가 계속 움직이는 안전 문제가 된다.
#
# ★ 대응 ★
#   시작 시점의 부모 pid 를 기억해 두고 데몬 스레드에서 os.getppid() 를 폴링한다.
#   부모가 죽으면 POSIX 는 이 프로세스를 init(1) 에 재부모화하므로 getppid() 가 바뀐다.
#   그 순간 cleanup 콜백(정지값 전송·포트 close)을 실행하고 프로세스를 끝낸다.
#
#   ※ 이미 고아가 되어 떠 있는 프로세스는 이 모듈이 손대지 못한다. 그런 잔재는
#     `pkill -f nxde.arduino` 또는 `fuser -k /dev/ttyACM*` 로 직접 정리한다.

import os
import sys
import threading
import time

_POLL_S = 1.0   # 부모 생존 확인 주기

# ★ 부모 사망 경로에서는 기본적으로 아무것도 출력하지 않는다 ★
#   launch 는 자식이 죽은 것을 확인한 뒤 그 프로세스의 출력 스트림을 정리하는데, 그 이후에
#   도착한 출력을 처리하려다 내부 예외를 던진다. 정리 자체는 launch 로그의
#   "process has died" 로 추적되므로 조용히 끝내고, 진단이 필요할 때만 환경변수로 켠다.
_VERBOSE = bool(os.environ.get('NXDE_PROC_GUARD_VERBOSE'))


def _log(message):
    try:
        print(f"[proc_guard] {message}", flush=True)
    except Exception:
        pass


def _die(ppid, cleanup):
    if _VERBOSE:
        _log(f"부모 프로세스(pid {ppid})가 종료됨 — 정리 후 스스로 종료합니다 (고아 방지)")
    if cleanup is not None:
        try:
            cleanup()
        except BaseException as e:   # 정리 실패가 종료 자체를 막지 않도록
            if _VERBOSE:
                _log(f"정리 콜백 실패(무시하고 종료): {e}")
    if _VERBOSE:
        try:
            sys.stdout.flush()
            sys.stderr.flush()
        except Exception:
            pass
    # rclpy.spin 이나 시리얼 read 등 어디에 블로킹돼 있어도 확실히 끝나야 하므로
    # os._exit 를 쓴다 (필요한 정리는 위 cleanup 에서 이미 끝났다).
    os._exit(0)


def watch_parent(cleanup=None):
    """부모 프로세스의 종료를 감시하는 데몬 스레드를 띄운다.

    cleanup : 부모가 죽었을 때 종료 직전에 한 번 호출되는 콜백(선택).
              시리얼 정지값 전송·포트 close 등 '반드시 해야 하는 정리'만 짧게 넣는다.
              ※ 이 콜백 안에서 stdout/stderr 에 쓰지 말 것 — 위 _VERBOSE 주석 참고.
    반환값  : 감시 스레드
    """
    ppid = os.getppid()

    def waiter():
        while os.getppid() == ppid:
            time.sleep(_POLL_S)
        _die(ppid, cleanup)

    thread = threading.Thread(target=waiter, name='proc_guard', daemon=True)
    thread.start()
    return thread
