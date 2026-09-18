#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""soundutil.py ― 음원 재생 공통부 [nxde] ★rclpy 를 import 하지 않는다★

`sound_dir()` 와 `Player` 를 여기 한 곳에 둔다. 원래 `nxde/sound.py` 안에
있었는데, kill.py(★의도적으로 rclpy 를 import 하지 않는 도구★ — 그 파일 헤더
참고)에서도 종료음(kill.wav) 하나를 재생하려니 ROS 노드 쪽 코드와 분리할
곳이 필요해 이쪽으로 뽑았다.

  · `sound.py` 는 ROS 구독으로 사건이 오면 이 Player 를 부른다.
  · `kill.py` 는 ROS 와 무관하게, 정리가 끝난 그 자리에서 바로 부른다.
  · `braketest.py` 처럼 이미 rclpy 를 쓰는 노드도 그냥 이걸 가져다 쓰면 된다
    (rclpy 를 이미 물고 있으니 추가로 잃을 것이 없다).

★[2026-09-14] 확장자를 mp3 하나로 고정하지 않는다★ 안내 음성은 전부 mp3 로
만들어 왔지만(edge-tts 출력), E-STOP 경고음처럼 사람이 녹음/합성해 온 wav 를
그대로 쓰고 싶은 경우가 생겼다(siren.wav/siren_rev.wav). 재생기(ffplay 등)는
확장자가 아니라 내용으로 포맷을 알아내므로 굳이 mp3 로 바꿀 필요가 없다 —
`_path()` 가 같은 이름으로 mp3 를 먼저 찾고, 없으면 wav 를 본다.
"""

import os
import shutil
import subprocess
import threading
import time

PACKAGE_NAME = 'nxde'

# ══════════════════════════════════════════════════════════════════════════
#  ★★ 음량 — 안내 음성·경고음의 크기는 여기 두 줄이 정한다 ★★  [2026-09-18]
# ══════════════════════════════════════════════════════════════════════════
#  ★이 파일이 단일 소유자다★ — sound.py(주행 안내) · kill.py(종료음) ·
#  prompt.py · braketest.py · tts.py 가 전부 아래 PLAYERS 를 쓴다.
#  종전에는 tts.py 만 증폭을 갖고 있어서, ★같은 스피커로 나가는 주행 안내가
#  TTS 보다 3 배 작았다★.
#
#  ★증폭 + 리미터가 한 쌍이다★ 단순 증폭만 하면 큰 음절에서 파형이 잘려
#  찌그러진다 — 말이 커지는 게 아니라 알아듣기 어려워진다. lookahead 리미터가
#  천장을 눌러 주므로 배수를 올려도 깨지지 않는다.
#
#  ⚠️ ★3.0 이 실질 상한이다 — 더 올려도 커지지 않는다★ (실측, 리미터 적용 후 평균):
#      one_launch_1.mp3  원본 −18.9 → x2 −13.3 / ★x3 −11.8★ / x4 −11.1 / x5 −10.7
#      prompt_4.mp3      원본 −17.9 → x2 −12.0 / ★x3 −10.4★ / x4 −10.0 / x5  −9.7
#      estop.wav         원본  −2.2 → x2  −1.8 / ★x3  −1.8★ / x4  −1.8 / x5  −1.8
#    x3 에서 이미 리미터가 포화해, x4·x5 는 ★+0.5 dB 를 얻고 다이내믹만 잃는다★.
#    여기서 더 키우려면 재생 단계가 아니라 ★음원 파일 자체★ 를 다시 만들어야 한다.
#    (loudnorm 단일 패스도 재 보았으나 짧은 안내음에서는 오히려 −32 dB 로 줄었다.)
PLAY_GAIN = 3.0
PLAY_LIMIT = 0.97                 # 리미터 천장(0~1). 1.0 에 가까울수록 크고 위험하다


def gain_args(name):
    """재생기별 증폭 인자. ★PLAY_GAIN 한 곳만 고치면 전부 따라온다★

    네 재생기가 각자 다른 단위를 쓴다 — 여기서 한 번 환산해 두지 않으면 재생기가
    바뀔 때마다 음량이 조용히 달라진다(있는 것을 위에서부터 고르는 구조라, 어느
    것이 걸릴지는 기계마다 다르다).
    """
    g = max(1.0, float(PLAY_GAIN))
    if g <= 1.0:
        return []
    if name == 'ffplay':
        #  ★증폭 + 리미터★ 클리핑 없이 키우는 유일한 조합이다(상수 주석 참고).
        return ['-af', 'volume=%.2f,alimiter=limit=%.2f' % (g, PLAY_LIMIT)]
    if name == 'mpg123':
        #  -f 는 ★출력 스케일★ 이고 32768 이 1배다(그래서 배수 × 32768).
        return ['-f', str(int(32768 * g))]
    if name == 'mpv':
        #  --volume 은 % 이고 기본 상한이 130 이라 --volume-max 를 함께 올려야 한다.
        return ['--volume-max=%d' % int(g * 100), '--volume=%d' % int(g * 100)]
    if name == 'cvlc':
        return ['--gain', '%.2f' % g]
    return []


# 있는 것을 위에서부터 고른다. 전부 '창 없이 한 번 재생하고 끝'.
PLAYERS = tuple(
    (name, args + gain_args(name)) for name, args in (
        ('ffplay', ['-nodisp', '-autoexit', '-loglevel', 'quiet']),
        ('mpg123', ['-q']),
        ('mpv',    ['--no-video', '--really-quiet']),
        ('cvlc',   ['--intf', 'dummy', '--play-and-exit']),
    )
)

# 이 순서로 찾는다 — mp3 가 압도적으로 많으므로(안내 음성 전부) 먼저 본다.
SOUND_EXTS = ('.mp3', '.wav')

# 반복재생 사이의 간격 [s] — 0 이면 파일 경계가 붙어 한 덩어리로 들린다
LOOP_GAP_S = 0.35


def sound_dir(explicit: str = "") -> str:
    """음원 폴더. white1/paths.py 와 같은 규칙이다.

    1) 명시 경로  2) 환경변수 NXDE_SOUND_DIR  3) ★소스 트리★ <...>/src/nxde/sound
    (★있을 때만★ — [2026-08-14] 이후 이 폴더는 보통 없다)  4) ★white1 의 음원 폴더★
    5) 못 찾으면 ~/nxde_sound.

    ★import 를 감싼다★ white1 은 이 패키지의 의존이 아니다(package.xml 에 없다).
    nxde 만 빌드한 기계에서도 이 함수가 예외 없이 돌아야 한다 — 그때는 5) 로 간다.
    """
    if explicit:
        return os.path.abspath(os.path.expanduser(explicit))
    env = os.environ.get('NXDE_SOUND_DIR', '').strip()
    if env:
        return os.path.abspath(os.path.expanduser(env))
    here = os.path.dirname(os.path.realpath(__file__))   # .../src/nxde/nxde
    root = os.path.dirname(here)                         # .../src/nxde
    own = os.path.join(root, 'sound')
    if os.path.isfile(os.path.join(root, 'package.xml')) and os.path.isdir(own):
        return own
    try:
        from white1 import paths as white1_paths
        borrowed = white1_paths.sound_dir()
    except Exception:
        borrowed = ''
    if borrowed and os.path.isdir(borrowed):
        return borrowed
    return os.path.expanduser('~/nxde_sound')


def _find_player():
    for name, args in PLAYERS:
        path = shutil.which(name)
        if path:
            return [path] + args
    return None


def _find_file(directory, name):
    """<directory>/<name>.{mp3,wav} 중 있는 것. 없으면 None."""
    for ext in SOUND_EXTS:
        p = os.path.join(directory, name + ext)
        if os.path.isfile(p):
            return p
    return None


class Player:
    """음원 재생기. ★어디서든 쓸 수 있게 ROS 에 의존하지 않는다★ (prompt·kill 도 쓴다)

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
        elif self.enabled:
            #  ★음량이 기대와 다를 때 제일 먼저 볼 줄이다★ 재생기마다 증폭 단위가
            #  달라서(gain_args), 어느 것이 걸렸는지가 곧 음량의 절반이다.
            self.log("재생기 %s · 재생증폭 x%.1f (음원 %s)"
                     % (os.path.basename(self._cmd[0]), PLAY_GAIN, self.dir))

    # ── 내부 ──────────────────────────────────────────────────────────────────
    def _path(self, name):
        p = _find_file(self.dir, name)
        if p is None:
            if name not in self._warned:
                self._warned.add(name)
                self.log(f"음원 없음: {os.path.join(self.dir, name)}.{{"
                         f"{'|'.join(e.lstrip('.') for e in SOUND_EXTS)}}}")
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
                return                            # ★반복재생 중에는 아무것도 끼지 않는다★
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

    def audit(self, names):
        """★음원 폴더를 뜨는 즉시 점검한다★ (폴더가 없나, 빠진 이름 목록) 을 돌려준다."""
        if not os.path.isdir(self.dir):
            return True, list(names)
        missing = [n for n in names if _find_file(self.dir, n) is None]
        return False, missing

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
