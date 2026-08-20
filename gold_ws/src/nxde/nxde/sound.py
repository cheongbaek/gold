#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sound.py ― 음성 안내 [nxde]
════════════════════════════════════════════════════════════════════════════════
    ros2 run nxde sound          (보통은 white806 one_launch.py 가 함께 띄운다)

`<음원 폴더>/*.mp3` 를 사건에 맞춰 ★시스템 기본 스피커★ 로 재생한다.
음원 파일은 사람이 직접 넣는다 — 이 노드는 합성하지 않는다.

  ★[2026-08-14] 음원의 주인은 white1 이다★ 안내 문구가 전부 그 스택의 사건
  (매핑·주행 시작, 도착, 경로이탈, E-STOP)이라 음원도 `white1/sound/` 로 옮겼고,
  white1 의 one_launch.py 가 그 경로를 ★sound_dir 파라미터★ 로 넘겨 준다.
  이 노드의 기본값은 여전히 자기 패키지(<nxde>/sound)라, `ros2 run nxde sound` 로
  혼자 띄우면 그쪽을 본다 — 그때는 경로를 직접 주는 것이 맞다:
      ros2 run nxde sound --ros-args -p sound_dir:=<...>/src/white1/sound

════════════════════════════════════════════════════════════════════════════════
 무엇이 언제 나오는가
════════════════════════════════════════════════════════════════════════════════
  one_launch_1  이 노드가 뜨는 순간             = one_launch.py 를 실행한 직후
  one_launch_2  /board_status 가 A:1,B:1 이 되는 순간 (A·B 보드 모두 연결)
  prompt_1      prompt.py 가 시작할 때                 ← ★그쪽이 직접 낸다★
  mapping       매핑을 걸었는데 스위치가 자율주행이라 대기할 때  ← ★prompt 가 낸다★
  driving       주행을 걸었는데 스위치가 수동조종이라 대기할 때  ← ★prompt 가 낸다★
  estop_x       스위치는 맞았는데 E-STOP 이 물려 있어 대기할 때  ← ★prompt 가 낸다★
  prompt_2      /drive_state 가 MAP_HEADING 으로 들어갈 때 (매핑 실제 시작)
  prompt_3      /drive_state 가 MAP_* 에서 빠져나올 때   (매핑 종료)
  prompt_4      /drive_state 가 DRIVE_HEADING 으로 들어갈 때 (주행 실제 시작)
  prompt_5      /drive_event 에 '🎯 도착' 이 뜰 때       (목적지 도착으로 종료)
  driving_1     /drive_event 에 '경로이탈' 이 뜰 때      (이탈로 종료)
  estop         /estop 이 True 로 올라간 순간 ★한 번★   [2026-08-21 : 반복 → 1회]
  estop_re      /estop 이 False 로 떨어진 직후 — ★단, prompt 가 대기 중이면 내지 않는다★

  ★prompt_1·mapping·driving·estop_x 는 prompt 쪽에서 낸다★ 넷 다 '사람이 화면에서
  무엇을 눌렀고 무엇을 기다리는 중인가'라서 토픽에 나타나지 않기 때문이다(대기
  상태는 prompt 의 로컬 UI 상태다). 나머지는 전부 토픽으로 관측되므로 이 노드
  하나가 맡는다 — 그래야 화면(prompt)을 쓰든 안 쓰든 같은 안내가 나온다.

════════════════════════════════════════════════════════════════════════════════
 ★★ [2026-08-21] E-STOP 안내가 바뀌었다 ★★
════════════════════════════════════════════════════════════════════════════════
  ① ★estop 은 1회 재생이다★ (종전 : 해제될 때까지 반복)
     E-STOP 이 '주행 취소'에서 ★일시정지★ 로 바뀌면서(white1/driving.py) 체결
     상태가 몇 초씩 이어지는 것이 정상 절차가 됐다. 그 동안 경고음이 계속 울리면
     ㉠ 사람이 차 옆에서 주고받는 말을 덮고 ㉡ 뒤이어 나와야 할 estop_x('해제
     하세요') 가 반복재생에 막혀 아예 나오지 못한다(Player.play 는 loop 중에
     들어온 것을 버린다). 발동 사실은 한 번 말하면 충분하다.

  ② ★대기 중 해제에는 estop_re 를 내지 않는다★
     prompt 가 매핑/주행 시작을 기다리는 중이라면, 해제되는 그 순간이 곧 시작
     이라 prompt_2/prompt_4(시작 안내)가 바로 나간다. 거기에 해제음을 겹치면
     무엇이 시작됐는지 들리지 않는다 — ★해제 사실보다 시작 사실이 중요하다★.
     판정 근거는 prompt 가 발행하는 ★/prompt_wait★ (String, '' = 대기 아님) 이고,
     PROMPT_WAIT_STALE_S 이상 끊기면 '대기 아님'으로 되돌린다 — prompt 를 안
     띄우거나 그 노드가 죽었으면 겹칠 시작 안내도 없으므로 종전대로 내는 것이 맞다.

════════════════════════════════════════════════════════════════════════════════
 설계에서 지킨 것
════════════════════════════════════════════════════════════════════════════════
  · ★제어에 끼어들지 않는다★ 구독만 하고 아무것도 발행하지 않는다. 노드가 죽어도
    주행은 그대로다(one_launch 에서 respawn 도 걸지 않는다 — 되살아날 때마다
    one_launch_1 이 다시 나오는 편이 더 헷갈린다).
  · ★ROS 콜백에서 재생을 기다리지 않는다★ 재생은 subprocess 로 띄우고 즉시 돌아온다.
    콜백이 막히면 그 실행기 스레드에 묶인 다른 구독까지 함께 늦는다.
  · ★[2026-08-21] E-stop 중에도 다른 안내가 나간다★ 종전에는 반복재생이 다른
    안내를 전부 막았는데, E-STOP 이 일시정지가 되면서 그 동안에도 사람에게 할 말
    (estop_x = '해제하세요')이 생겼다. 대신 겹침은 ★한 번에 하나만 재생★ 이라는
    Player 의 성질로 막는다 — 늦게 온 안내가 앞의 것을 끊는다.
  · ★없는 파일은 경고 한 번만★ 파일 이름이 틀려도 노드는 계속 돈다.
  · 재생기는 있는 것을 골라 쓴다(ffplay → mpg123 → mpv → cvlc). 출력 장치를 지정
    하지 않으므로 ★시스템 기본 스피커★ 로 나간다.
"""

import os
import shutil
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String


# ── 음원 이름 (확장자 없이. sound/ 안의 파일명과 같아야 한다) ──────────────────
SND_LAUNCH       = 'one_launch_1'
SND_BOARDS       = 'one_launch_2'
SND_PROMPT       = 'prompt_1'
SND_WAIT_MAP     = 'mapping'
SND_WAIT_DRIVE   = 'driving'
SND_MAP_BEGIN    = 'prompt_2'
SND_MAP_END      = 'prompt_3'
SND_DRIVE_BEGIN  = 'prompt_4'
SND_ARRIVED      = 'prompt_5'
SND_DEVIATED     = 'driving_1'
SND_ESTOP        = 'estop'
SND_ESTOP_CLEAR  = 'estop_re'
# ★[2026-08-21] '시작하려는데 E-STOP 이 물려 있다 — 해제하라'★ 재생은 prompt 가
#   한다(그쪽 로컬 대기 상태라서). 이름을 여기 두는 이유는 음원 목록의 주인이
#   이 파일이기 때문이다 — 이름이 갈라지면 파일명 오타를 한쪽에서만 고치게 된다.
SND_ESTOP_HOLD   = 'estop_x'

# 이벤트 문구에서 찾을 조각 (driving.py 가 /drive_event 로 내보내는 문장)
EVENT_ARRIVED  = '🎯 도착'
EVENT_DEVIATED = '경로이탈'

# 반복재생 사이의 간격 [s] — 0 이면 파일 경계가 붙어 한 덩어리로 들린다
#   ※ [2026-08-21] 지금 loop() 를 쓰는 곳은 없다(E-STOP 이 1회 재생이 되면서
#     마지막 사용처가 사라졌다). Player 는 prompt 도 함께 쓰는 범용 클래스라
#     기능은 남겨 둔다.
LOOP_GAP_S = 0.35

# ★/prompt_wait 신선도 [s]★ 이 시간 넘게 안 오면 '대기 아님'으로 본다.
#   prompt 는 자기 루프(0.4s)마다 값을 다시 보낸다 — 그보다 넉넉히 잡되, 화면이
#   죽었을 때 억제가 오래 남지 않을 만큼 짧게. cb_estop 주석 참고.
PROMPT_WAIT_STALE_S = 2.0

# 있는 것을 위에서부터 고른다. 전부 '창 없이 한 번 재생하고 끝'.
PLAYERS = (
    ('ffplay', ['-nodisp', '-autoexit', '-loglevel', 'quiet']),
    ('mpg123', ['-q']),
    ('mpv',    ['--no-video', '--really-quiet']),
    ('cvlc',   ['--intf', 'dummy', '--play-and-exit']),
)


def sound_dir(explicit: str = "") -> str:
    """음원 폴더. white806/paths.py 와 같은 규칙이다.

    1) 명시 경로  2) 환경변수 NXDE_SOUND_DIR  3) ★소스 트리★ <...>/src/nxde/sound
    colcon 을 --symlink-install 로 빌드하면 이 파일이 소스를 가리키는 심볼릭 링크라
    realpath 로 소스 위치를 되찾을 수 있다. 못 찾으면 ~/nxde_sound.
    """
    if explicit:
        return os.path.abspath(os.path.expanduser(explicit))
    env = os.environ.get('NXDE_SOUND_DIR', '').strip()
    if env:
        return os.path.abspath(os.path.expanduser(env))
    here = os.path.dirname(os.path.realpath(__file__))   # .../src/nxde/nxde
    root = os.path.dirname(here)                         # .../src/nxde
    if os.path.isfile(os.path.join(root, 'package.xml')):
        return os.path.join(root, 'sound')
    return os.path.expanduser('~/nxde_sound')


def _find_player():
    for name, args in PLAYERS:
        path = shutil.which(name)
        if path:
            return [path] + args
    return None


class Player:
    """mp3 재생기. ★어디서든 쓸 수 있게 ROS 에 의존하지 않는다★ (prompt 도 쓴다)

    play()  한 번 재생. 재생 중이던 것은 끊는다(늦게 온 사건이 더 중요하다).
    loop()  멈추라고 할 때까지 반복. 반복 중에는 play() 를 받지 않는다.
    stop()  전부 정지.
    """

    def __init__(self, directory: str = "", log=None, enabled: bool = True):
        self.dir = directory or sound_dir()
        self.log = log or (lambda msg: None)
        self.enabled = enabled
        self._cmd = _find_player()
        self._proc = None
        self._loop = None            # 반복재생 중인 음원 이름
        self._lock = threading.RLock()
        self._warned = set()
        if self.enabled and self._cmd is None:
            self.log("재생기를 찾지 못했다(ffplay/mpg123/mpv/cvlc) — 음성 안내 없이 돈다")

    # ── 내부 ──────────────────────────────────────────────────────────────────
    def _path(self, name):
        p = os.path.join(self.dir, f"{name}.mp3")
        if not os.path.isfile(p):
            if name not in self._warned:
                self._warned.add(name)
                self.log(f"음원 없음: {p}")
            return None
        return p

    def _kill(self):
        if self._proc is not None and self._proc.poll() is None:
            try:
                self._proc.kill()
            except Exception:
                pass
        self._proc = None

    def _spawn(self, path):
        try:
            self._proc = subprocess.Popen(
                self._cmd + [path],
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL)
            return True
        except Exception as e:                    # 재생 실패로 노드를 죽이지 않는다
            self.log(f"재생 실패({os.path.basename(path)}): {e}")
            self._proc = None
            return False

    # ── 바깥에서 부르는 것 ─────────────────────────────────────────────────────
    def play(self, name):
        if not self.enabled or self._cmd is None:
            return
        with self._lock:
            if self._loop is not None:
                return                            # ★E-stop 반복 중에는 아무것도 끼지 않는다★
            path = self._path(name)
            if path is None:
                return
            self._kill()
            self._spawn(path)

    def loop(self, name):
        if not self.enabled or self._cmd is None:
            return
        with self._lock:
            if self._loop == name:
                return                            # 이미 같은 것을 돌리고 있다
            path = self._path(name)
            if path is None:
                return
            self._kill()
            self._loop = name
            threading.Thread(target=self._loop_worker, args=(name, path),
                             daemon=True).start()

    def _loop_worker(self, name, path):
        while True:
            with self._lock:
                if self._loop != name:
                    return
                if not self._spawn(path):
                    self._loop = None
                    return
                proc = self._proc
            try:
                proc.wait()
            except Exception:
                return
            # 간격을 두되, 그 사이에 stop() 이 오면 곧바로 빠진다
            for _ in range(max(1, int(LOOP_GAP_S * 20))):
                with self._lock:
                    if self._loop != name:
                        return
                time.sleep(0.05)

    def stop_loop(self):
        with self._lock:
            if self._loop is None:
                return False
            self._loop = None
            self._kill()
            return True

    def stop(self):
        with self._lock:
            self._loop = None
            self._kill()


class SoundNode(Node):

    def __init__(self):
        super().__init__('sound_node')
        self.declare_parameter('enable', True)
        self.declare_parameter('sound_dir', '')
        self.declare_parameter('startup_sound', True)

        enabled = bool(self.get_parameter('enable').value)
        self.player = Player(self.get_parameter('sound_dir').value or '',
                             log=self.get_logger().warning,
                             enabled=enabled)

        # ★첫 수신은 엣지로 치지 않는다★ 노드가 늦게 떠도 그때의 상태를 '방금 바뀐 것'
        #   으로 오해하지 않게 한다(E-stop 이 이미 걸려 있는 경우만 예외 — 그건 지금
        #   울려야 한다).
        self.state = None
        self.estop = None
        self.boards = None
        # ★[2026-08-21] prompt 가 무엇을 기다리는 중인가★ '' = 대기 아님.
        #   해석하지 않고 '비었나 아닌가'만 본다 — 어느 게이트인지는 prompt 의
        #   사정이고, 이 노드가 알아야 하는 것은 '지금 시작 안내가 뒤따르는가'뿐이다.
        self.prompt_wait = ''
        self.prompt_wait_t = 0.0     # ★마지막으로 '대기 중'을 본 시각★ (cb_wait 참고)

        self.create_subscription(String, '/drive_state',  self.cb_state,  10)
        self.create_subscription(String, '/drive_event',  self.cb_event,  10)
        self.create_subscription(Bool,   '/estop',        self.cb_estop,  10)
        self.create_subscription(String, '/board_status', self.cb_boards, 10)
        self.create_subscription(String, '/prompt_wait',  self.cb_wait,   10)

        self.get_logger().info(
            f"🔈 음성 안내 {'준비' if enabled else '꺼짐(enable:=false)'} — {self.player.dir}")
        if enabled and bool(self.get_parameter('startup_sound').value):
            self.player.play(SND_LAUNCH)

    # ── 상태기계 ──────────────────────────────────────────────────────────────
    def cb_state(self, msg: String):
        new = str(msg.data)
        old, self.state = self.state, new
        if old is None or old == new:
            return
        was_map = old.startswith('MAP_')
        if new == 'MAP_HEADING' and not was_map:
            self.player.play(SND_MAP_BEGIN)
        elif was_map and not new.startswith('MAP_'):
            self.player.play(SND_MAP_END)
        elif new == 'DRIVE_HEADING' and not old.startswith('DRIVE_'):
            self.player.play(SND_DRIVE_BEGIN)

    def cb_event(self, msg: String):
        """도착·이탈은 상태만으로는 구별되지 않는다 — 둘 다 DRIVE_DONE 으로 간다.
        그래서 이유가 적혀 있는 이벤트 문구를 본다."""
        text = str(msg.data)
        if EVENT_ARRIVED in text:
            self.player.play(SND_ARRIVED)
        elif EVENT_DEVIATED in text:
            self.player.play(SND_DEVIATED)

    def cb_wait(self, msg: String):
        """prompt 의 대기 상태. ★값은 안 본다 — 비었는지만 본다★

        ★비어 있지 않을 때만 시각을 갱신한다★ 여기가 요점이다. prompt 는 대기가
        끝나는 그 틱에 START 를 보내면서 곧바로 '' 를 발행하는데, 우리 쪽 /estop
        하강 콜백은 그보다 먼저 올 수도 나중에 올 수도 있다(전자가 대부분이다 —
        토픽 콜백은 ms, prompt 의 화면 루프는 0.4s). 마지막 '대기 중' 시각을 들고
        있으면 어느 쪽이 이기든 같은 결론이 나온다.
        """
        self.prompt_wait = str(msg.data).strip()
        if self.prompt_wait:
            self.prompt_wait_t = time.time()

    def prompt_waiting(self):
        """prompt 가 지금(또는 방금까지) 매핑/주행 시작을 기다리고 있었는가.

        ★'방금까지'를 포함하는 것이 의도다★ 위 cb_wait 주석의 경합 때문이고,
        대가로 ★대기를 취소한 직후 PROMPT_WAIT_STALE_S 안에 해제하면 해제음이
        빠진다★. 둘 중에서는 이쪽이 낫다 — 놓치는 쪽은 소리 하나가 없는 것이고,
        반대쪽은 시작 안내와 겹쳐 ★둘 다 안 들리는 것★ 이다.

        ★끊기면 '아니다'로 돌아온다★ prompt 가 죽었거나 애초에 안 떠 있으면 이
        토픽이 오지 않는데, 그때는 해제 뒤에 이어질 시작 안내도 없으므로 해제음을
        내는 쪽이 맞다(억제가 영영 남지 않는다).
        """
        return (time.time() - self.prompt_wait_t) <= PROMPT_WAIT_STALE_S

    def cb_estop(self, msg: Bool):
        """[2026-08-21] 발동 = ★1회★ / 해제 = 대기 중이 아닐 때만. 파일 헤더 참고."""
        new = bool(msg.data)
        old, self.estop = self.estop, new
        if new and not old:
            self.player.play(SND_ESTOP)           # ★한 번만★ (종전 : 해제까지 반복)
        elif old and not new:
            if self.prompt_waiting():
                # 해제가 곧 시작이다 — prompt_2/prompt_4 가 바로 뒤따르므로 비워 둔다.
                return
            self.player.play(SND_ESTOP_CLEAR)

    def cb_boards(self, msg: String):
        """"A:1,B:1,ESTOP:0,MODE:1" — 둘 다 1 이 되는 순간이 '연결 완료'다."""
        fields = dict(p.split(':', 1) for p in str(msg.data).split(',') if ':' in p)
        both = (fields.get('A') == '1' and fields.get('B') == '1')
        old, self.boards = self.boards, both
        if both and not old:
            self.player.play(SND_BOARDS)

    def destroy_node(self):
        self.player.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SoundNode()
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
