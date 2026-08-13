#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sound.py ― 음성 안내 [nxde]
════════════════════════════════════════════════════════════════════════════════
    ros2 run nxde sound          (보통은 white806 one_launch.py 가 함께 띄운다)

`<nxde 패키지>/sound/*.mp3` 를 사건에 맞춰 ★시스템 기본 스피커★ 로 재생한다.
음원 파일은 사람이 직접 넣는다 — 이 노드는 합성하지 않는다.

════════════════════════════════════════════════════════════════════════════════
 무엇이 언제 나오는가
════════════════════════════════════════════════════════════════════════════════
  one_launch_1  이 노드가 뜨는 순간             = one_launch.py 를 실행한 직후
  one_launch_2  /board_status 가 A:1,B:1 이 되는 순간 (A·B 보드 모두 연결)
  prompt_1      prompt.py 가 시작할 때                 ← ★그쪽이 직접 낸다★
  mapping       매핑을 걸었는데 스위치가 자율주행이라 대기할 때  ← ★prompt 가 낸다★
  driving       주행을 걸었는데 스위치가 수동조종이라 대기할 때  ← ★prompt 가 낸다★
  prompt_2      /drive_state 가 MAP_HEADING 으로 들어갈 때 (매핑 실제 시작)
  prompt_3      /drive_state 가 MAP_* 에서 빠져나올 때   (매핑 종료)
  prompt_4      /drive_state 가 DRIVE_HEADING 으로 들어갈 때 (주행 실제 시작)
  prompt_5      /drive_event 에 '🎯 도착' 이 뜰 때       (목적지 도착으로 종료)
  driving_1     /drive_event 에 '경로이탈' 이 뜰 때      (이탈로 종료)
  estop         /estop=True 인 ★동안 반복★
  estop_re      /estop 이 False 로 떨어진 직후

  ★prompt_1·mapping·driving 만 prompt 쪽에서 낸다★ 셋 다 '사람이 화면에서 무엇을
  눌렀나'라서 토픽에 나타나지 않기 때문이다(대기 상태는 prompt 의 로컬 UI 상태다).
  나머지는 전부 토픽으로 관측되므로 이 노드 하나가 맡는다 — 그래야 화면(prompt)을
  쓰든 안 쓰든 같은 안내가 나온다.

════════════════════════════════════════════════════════════════════════════════
 설계에서 지킨 것
════════════════════════════════════════════════════════════════════════════════
  · ★제어에 끼어들지 않는다★ 구독만 하고 아무것도 발행하지 않는다. 노드가 죽어도
    주행은 그대로다(one_launch 에서 respawn 도 걸지 않는다 — 되살아날 때마다
    one_launch_1 이 다시 나오는 편이 더 헷갈린다).
  · ★ROS 콜백에서 재생을 기다리지 않는다★ 재생은 subprocess 로 띄우고 즉시 돌아온다.
    콜백이 막히면 그 실행기 스레드에 묶인 다른 구독까지 함께 늦는다.
  · ★E-stop 중에는 다른 안내를 내지 않는다★ 반복재생이 걸린 동안 들어온 일반
    안내는 버린다. 비상 상황에서 안내가 겹쳐 들리는 것이 제일 나쁘다.
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

# 이벤트 문구에서 찾을 조각 (driving.py 가 /drive_event 로 내보내는 문장)
EVENT_ARRIVED  = '🎯 도착'
EVENT_DEVIATED = '경로이탈'

# 반복재생 사이의 간격 [s] — 0 이면 파일 경계가 붙어 한 덩어리로 들린다
LOOP_GAP_S = 0.35

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

        self.create_subscription(String, '/drive_state',  self.cb_state,  10)
        self.create_subscription(String, '/drive_event',  self.cb_event,  10)
        self.create_subscription(Bool,   '/estop',        self.cb_estop,  10)
        self.create_subscription(String, '/board_status', self.cb_boards, 10)

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

    def cb_estop(self, msg: Bool):
        new = bool(msg.data)
        old, self.estop = self.estop, new
        if new and not old:
            self.player.loop(SND_ESTOP)           # ★해제될 때까지 반복★
        elif old and not new:
            self.player.stop_loop()
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
