#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
panel.py ― ★터치 조작반★ [nxde / 2026-10-16]
════════════════════════════════════════════════════════════════════════════════
    ros2 run nxde panel          (또는 python3 panel.py — 표준 라이브러리만 쓴다)

  키보드 없이 마우스(터치)만으로 스택을 띄우고 내린다. 터미널 창은 [Check] 만 띄운다.

    [Check] [Kill] [Exit]
    [white1 one_launch]       <상태> [x]   ros2 launch white1 one_launch.py (HUD·인지 창·RViz 없이)
    [subcourse] [maincourse]  <상태> [x]   ros2 run white1 prompt --route <그 CSV>  (= 2) 주행 → 경로 선택)
    [braketest] [nolidar]     <상태> [x]   ros2 launch white1 braketest.launch.py nolidar:=false/true

  <상태> = 쉬는 중 / 실행 중 / 종료 중.  [x] = 그 줄에 ★Ctrl+C★ (프로세스 그룹에 SIGINT —
  터미널에서 누른 것과 같다). STOP_TERM_S 안에 안 내려가면 SIGTERM, STOP_KILL_S 면 SIGKILL.

  ★[Kill]·[Exit]·창의 X 는 먼저 이 패널이 띄운 것에 Ctrl+C 를 준다★ (GRACE_BEFORE_KILL_S 까지
  기다림) — nxde kill 은 처음부터 SIGKILL 이라 arduino 의 종료 정리(B보드에 'x,0' = 조향 힘빼기 ·
  ★리니어 0단★)를 건너뛴다. braketest 가 2단을 유지하는 중에 그대로 죽이면 리니어가 물린 채로
  남는다. 그 뒤에 `ros2 run nxde kill` 을 돌려 나머지(다른 터미널에서 띄운 것까지)를 정리한다.
  [Exit] 는 거기에 더해 이 창을 닫는다.

  ★한 번에 하나★ one_launch 와 braketest 는 같은 보드(arduino)를 쓴다 — 한쪽이 쉬는 중이
  아니면 다른 쪽 버튼을 잠근다. 각 줄의 출력은 ~/.ros/log/panel/<줄>-<시각>.log 에 남는다.

  ROS 환경 : 명령마다 /opt/ros/<distro>/setup.bash 와 이 파일이 속한 워크스페이스의
  install/setup.bash 를 source 하고 exec 한다(직접 실행해도 돌게). ROS_DOMAIN_ID · RMW 는
  ★이 패널을 띄운 환경을 그대로 물려준다★ — 아래 줄에 지금 값을 보여 준다(바탕화면에서
  띄우면 ~/.bashrc 의 값이 없을 수 있다).
"""

import glob
import os
import shlex
import shutil
import signal
import subprocess
import threading
import time
import tkinter as tk
from tkinter import font as tkfont

# ══════════════════════════════════════════════════════════════════════════════
#  명령 — ★여기만 고치면 된다★
# ══════════════════════════════════════════════════════════════════════════════
#  one_launch 는 HUD·신호등 인지 창·라이다 RViz 를 끈다(사용자 지시 '추가 창을 띄우지 않는다').
CMD_ONE_LAUNCH = ['ros2', 'launch', 'white1', 'one_launch.py',
                  'use_hud:=false', 'tl_show_window:=false', 'use_lidar_rviz:=false']
CMD_CHECK = ['ros2', 'run', 'nxde', 'check']
CMD_KILL = ['ros2', 'run', 'nxde', 'kill']


def cmd_prompt(route):
    """prompt 를 메뉴 없이 그 경로로 — '2) 주행 → 경로 선택' 바로가기 (white1 prompt --route)."""
    return ['ros2', 'run', 'white1', 'prompt', '--route', route]


def cmd_braketest(nolidar):
    return ['ros2', 'launch', 'white1', 'braketest.launch.py',
            f"nolidar:={'true' if nolidar else 'false'}"]


STOP_TERM_S = 20.0          # [x] 뒤 이만큼 안 끝나면 그룹에 SIGTERM
STOP_KILL_S = 30.0          # 그래도 안 끝나면 SIGKILL
GRACE_BEFORE_KILL_S = 6.0   # [Kill]·[Exit] : 이 패널이 띄운 것에 Ctrl+C 를 주고 기다리는 상한
CHECK_CLOSE_S = 5           # [Check] 창 : 결과가 나오면 이만큼 뒤에 닫힌다
KILL_TIMEOUT_S = 60.0
POLL_MS = 300
LOG_DIR = os.path.expanduser('~/.ros/log/panel')

ST_IDLE, ST_RUN, ST_STOP = '쉬는 중', '실행 중', '종료 중'
ST_COLOR = {ST_IDLE: ('#3a3f47', '#c9d1d9'), ST_RUN: ('#1f6f3a', '#ffffff'),
            ST_STOP: ('#8a5a00', '#ffffff')}

#  ★snap 앱(VSCode 등)의 터미널에서 띄우면 그 앱의 GTK·로케일 변수가 물려 들어와 GTK 프로그램
#  (gnome-terminal·zenity)이 snap 라이브러리를 물고 죽는다★ (2026-10-16 실측 — braketest.py 와 같다)
_SNAP_LEAK = ('GTK_PATH', 'GTK_EXE_PREFIX', 'GTK_IM_MODULE_FILE', 'GDK_PIXBUF_MODULE_FILE',
              'GDK_PIXBUF_MODULEDIR', 'GIO_MODULE_DIR', 'GSETTINGS_SCHEMA_DIR', 'LOCPATH')


def clean_env():
    env = {k: v for k, v in os.environ.items() if k not in _SNAP_LEAK}
    for k in ('XDG_DATA_DIRS', 'XDG_CONFIG_DIRS'):
        orig = os.environ.get(f'{k}_VSCODE_SNAP_ORIG')
        if orig is not None:
            env[k] = orig
    return env


# ══════════════════════════════════════════════════════════════════════════════
#  ROS 환경 — 명령마다 source 하고 exec 한다
# ══════════════════════════════════════════════════════════════════════════════
def _ros_setup():
    distro = os.environ.get('ROS_DISTRO', '')
    cands = ([f'/opt/ros/{distro}/setup.bash'] if distro else []) + \
        sorted(glob.glob('/opt/ros/*/setup.bash'))
    return next((c for c in cands if os.path.isfile(c)), None)


def _ws_setup():
    """이 파일이 속한 워크스페이스의 install/setup.bash (설치본·소스 어느 쪽에서 돌아도)."""
    d = os.path.dirname(os.path.realpath(__file__))
    while d and d != os.path.dirname(d):
        p = os.path.join(d, 'install', 'setup.bash')
        if os.path.isfile(p):
            return p
        d = os.path.dirname(d)
    return None


def bash_line(argv, tail=''):
    """argv 를 ROS 환경을 잡은 bash 한 줄로. tail 이 없으면 exec 한다(시그널이 곧장 닿게)."""
    parts = [f'source {shlex.quote(s)} >/dev/null 2>&1' for s in (_ros_setup(), _ws_setup()) if s]
    run = ' '.join(shlex.quote(a) for a in argv)
    parts.append(run + tail if tail else 'exec ' + run)
    return '; '.join(parts)


# ══════════════════════════════════════════════════════════════════════════════
#  한 줄 = 프로세스 하나 (자기 프로세스 그룹)
# ══════════════════════════════════════════════════════════════════════════════
class Slot:
    def __init__(self, tag):
        self.tag = tag
        self.proc = None
        self.pgid = None            # ★상태는 그룹으로 본다★ — 맨 위 프로세스만 끝나도 자식이 남을 수 있다
        self.log = None
        self.state = ST_IDLE
        self.stop_t = 0.0
        self.sent = set()

    def group_alive(self):
        if self.pgid is None:
            return False
        try:
            os.killpg(self.pgid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True

    def alive(self):
        """맨 위 프로세스든 그 그룹의 누구든 하나라도 살아 있으면 True."""
        if self.proc is None:
            return False
        return self.proc.poll() is None or self.group_alive()

    def start(self, argv):
        if self.alive():
            return
        os.makedirs(LOG_DIR, exist_ok=True)
        path = os.path.join(LOG_DIR, f"{self.tag}-{time.strftime('%Y%m%d_%H%M%S')}.log")
        self.log = open(path, 'w', buffering=1, encoding='utf-8')
        self.log.write(f"# {' '.join(argv)}\n")
        #  ★start_new_session★ 자기 프로세스 그룹 — [x] 의 SIGINT 가 launch 와 그 자식 전부에
        #  닿는다(터미널의 Ctrl+C 가 전경 그룹 전체에 가는 것과 같다).
        self.proc = subprocess.Popen(
            ['bash', '-c', bash_line(argv)], stdin=subprocess.DEVNULL,
            stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True,
            env=clean_env())
        self.pgid = self.proc.pid
        self.state = ST_RUN
        self.sent = set()

    def _signal(self, sig):
        try:
            os.killpg(self.pgid, sig)
        except (ProcessLookupError, PermissionError):
            pass

    def stop(self):
        """Ctrl+C 와 같다 — 그룹에 SIGINT."""
        if not self.alive() or self.state == ST_STOP:
            return
        self._signal(signal.SIGINT)
        self.state = ST_STOP
        self.stop_t = time.time()

    def poll(self):
        if self.proc is None:
            self.state = ST_IDLE
            return
        if not self.alive():
            self.proc = None
            self.pgid = None
            self.state = ST_IDLE
            if self.log:
                self.log.close()
                self.log = None
            return
        if self.state == ST_STOP:
            held = time.time() - self.stop_t
            if held >= STOP_KILL_S and 'kill' not in self.sent:
                self._signal(signal.SIGKILL)
                self.sent.add('kill')
            elif held >= STOP_TERM_S and 'term' not in self.sent:
                self._signal(signal.SIGTERM)
                self.sent.add('term')


# ══════════════════════════════════════════════════════════════════════════════
#  화면
# ══════════════════════════════════════════════════════════════════════════════
class Panel:
    def __init__(self, root):
        self.root = root
        self.busy = ''              # '' / 'kill' / 'exit' — 작업 중이면 버튼을 잠근다
        self.note = '준비'
        self.quit_now = False
        self.slots = {k: Slot(k) for k in ('one_launch', 'prompt', 'braketest')}

        root.title('nxde panel')
        root.configure(bg='#0d1117')
        root.geometry('900x560')
        root.minsize(640, 400)
        root.protocol('WM_DELETE_WINDOW', self.on_exit)

        fams = set(tkfont.families(root))
        fam = next((f for f in ('Noto Sans CJK KR', 'NanumGothic', 'Noto Sans KR',
                                'DejaVu Sans') if f in fams), 'TkDefaultFont')
        big = (fam, 20, 'bold')
        small = (fam, 11)

        def btn(text, cmd, bg='#30363d', fg='#f0f6fc'):
            return tk.Button(root, text=text, command=cmd, font=big, bg=bg, fg=fg,
                             activebackground='#58a6ff', activeforeground='#0d1117',
                             disabledforeground='#6e7681', relief='flat', bd=0,
                             highlightthickness=0, padx=8, pady=14, cursor='hand2')

        def status():
            return tk.Label(root, text=ST_IDLE, font=big, width=7,
                            bg=ST_COLOR[ST_IDLE][0], fg=ST_COLOR[ST_IDLE][1])

        pad = dict(padx=6, pady=6, sticky='nsew')
        self.b_check = btn('Check', self.on_check)
        self.b_kill = btn('Kill', self.on_kill, bg='#6e2b2b')
        self.b_exit = btn('Exit', self.on_exit, bg='#6e2b2b')
        self.b_check.grid(row=0, column=0, **pad)
        self.b_kill.grid(row=0, column=1, **pad)
        self.b_exit.grid(row=0, column=2, columnspan=2, **pad)

        self.b_one = btn('white1 one_launch', lambda: self.start('one_launch', CMD_ONE_LAUNCH))
        self.b_one.grid(row=1, column=0, columnspan=2, **pad)
        self.s_one = status()
        self.s_one.grid(row=1, column=2, **pad)
        self.x_one = btn('x', lambda: self.slots['one_launch'].stop(), bg='#8b1e1e')
        self.x_one.grid(row=1, column=3, **pad)

        self.b_sub = btn('subcourse', lambda: self.start('prompt', cmd_prompt('subcourse.csv')))
        self.b_main = btn('maincourse', lambda: self.start('prompt', cmd_prompt('maincourse.csv')))
        self.b_sub.grid(row=2, column=0, **pad)
        self.b_main.grid(row=2, column=1, **pad)
        self.s_prompt = status()
        self.s_prompt.grid(row=2, column=2, **pad)
        self.x_prompt = btn('x', lambda: self.slots['prompt'].stop(), bg='#8b1e1e')
        self.x_prompt.grid(row=2, column=3, **pad)

        self.b_bt = btn('braketest', lambda: self.start('braketest', cmd_braketest(False)))
        self.b_nl = btn('nolidar', lambda: self.start('braketest', cmd_braketest(True)))
        self.b_bt.grid(row=3, column=0, **pad)
        self.b_nl.grid(row=3, column=1, **pad)
        self.s_bt = status()
        self.s_bt.grid(row=3, column=2, **pad)
        self.x_bt = btn('x', lambda: self.slots['braketest'].stop(), bg='#8b1e1e')
        self.x_bt.grid(row=3, column=3, **pad)

        env = (f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '(없음=0)')} · "
               f"{os.environ.get('RMW_IMPLEMENTATION', 'RMW 기본')}")
        self.foot = tk.Label(root, text='', font=small, bg='#0d1117', fg='#8b949e', anchor='w')
        self.foot.grid(row=4, column=0, columnspan=3, padx=8, pady=(0, 6), sticky='we')
        tk.Label(root, text=env, font=small, bg='#0d1117', fg='#8b949e', anchor='e').grid(
            row=4, column=3, padx=8, pady=(0, 6), sticky='e')

        for c, w in enumerate((3, 3, 2, 1)):
            root.grid_columnconfigure(c, weight=w, uniform='col' if c < 2 else '')
        for r in range(4):
            root.grid_rowconfigure(r, weight=1)

        self.tick()

    # ── 동작 ──────────────────────────────────────────────────────────────────
    def start(self, tag, argv):
        if self.busy:
            return
        self.slots[tag].start(argv)
        self.note = f"{tag} 시작 — 로그 {self.slots[tag].log.name}"

    def on_check(self):
        """ros2 run nxde check 를 터미널 창에서 — 결과가 나오면 CHECK_CLOSE_S 뒤 닫힌다."""
        line = bash_line(CMD_CHECK, tail=(f'; echo; echo "── {CHECK_CLOSE_S}초 뒤 창이 닫힙니다 ──"; '
                                          f'sleep {CHECK_CLOSE_S}'))
        if shutil.which('gnome-terminal'):
            argv = ['gnome-terminal', '--title=nxde check', '--', 'bash', '-c', line]
        elif shutil.which('x-terminal-emulator'):
            argv = ['x-terminal-emulator', '-e', 'bash', '-c', line]
        else:
            self.note = '⚠️ 터미널 프로그램이 없다 (gnome-terminal / x-terminal-emulator)'
            return
        try:
            subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL, env=clean_env(), start_new_session=True)
            self.note = 'Check 창을 띄웠다'
        except OSError as e:
            self.note = f'⚠️ Check 창을 못 띄웠다 — {e}'

    def on_kill(self):
        self._kill_then('kill')

    def on_exit(self):
        self._kill_then('exit')

    def _kill_then(self, what):
        if self.busy:
            if what == 'exit':
                self.busy = 'exit'          # Kill 도중 Exit — 끝나면 닫는다
            return
        self.busy = what
        threading.Thread(target=self._kill_worker, daemon=True).start()

    def _kill_worker(self):
        #  ① 이 패널이 띄운 것부터 Ctrl+C — arduino 종료 정리(리니어 0단)가 돌 시간을 준다
        live = [s for s in self.slots.values() if s.alive()]
        if live:
            self.note = f"Ctrl+C → {', '.join(s.tag for s in live)} (최대 {GRACE_BEFORE_KILL_S:.0f}s)"
            for s in live:
                s.stop()
            t_end = time.time() + GRACE_BEFORE_KILL_S
            while time.time() < t_end and any(s.alive() for s in live):
                time.sleep(0.2)
        #  ② ros2 run nxde kill — 나머지 전부(다른 터미널에서 띄운 것까지)
        self.note = 'kill 실행 중…'
        try:
            os.makedirs(LOG_DIR, exist_ok=True)
            r = subprocess.run(['bash', '-c', bash_line(CMD_KILL)], stdin=subprocess.DEVNULL,
                               capture_output=True, text=True, timeout=KILL_TIMEOUT_S,
                               env=clean_env())
            out = (r.stdout or '') + (r.stderr or '')
            with open(os.path.join(LOG_DIR, f"kill-{time.strftime('%Y%m%d_%H%M%S')}.log"),
                      'w', encoding='utf-8') as f:
                f.write(out)
            lines = [ln.strip() for ln in out.splitlines() if ln.strip()]
            self.note = 'Kill — ' + (lines[-1] if lines else f'끝 (exit {r.returncode})')
        except subprocess.TimeoutExpired:
            self.note = f'⚠️ kill 이 {KILL_TIMEOUT_S:.0f}s 안에 안 끝났다'
        except OSError as e:
            self.note = f'⚠️ kill 실행 실패 — {e}'
        if self.busy == 'exit':
            self.quit_now = True
        self.busy = ''

    # ── 화면 갱신 ──────────────────────────────────────────────────────────────
    def tick(self):
        if self.quit_now:
            self.root.destroy()
            return
        for s in self.slots.values():
            s.poll()
        st = {k: s.state for k, s in self.slots.items()}
        for lbl, k in ((self.s_one, 'one_launch'), (self.s_prompt, 'prompt'),
                       (self.s_bt, 'braketest')):
            bg, fg = ST_COLOR[st[k]]
            lbl.configure(text=st[k], bg=bg, fg=fg)

        busy = bool(self.busy)

        def en(w, on):
            w.configure(state='normal' if on else 'disabled')

        #  ★one_launch 와 braketest 는 같은 보드를 쓴다★ — 한쪽이 쉬는 중이 아니면 다른 쪽을 잠근다
        en(self.b_one, not busy and st['one_launch'] == ST_IDLE and st['braketest'] == ST_IDLE)
        en(self.b_sub, not busy and st['prompt'] == ST_IDLE)
        en(self.b_main, not busy and st['prompt'] == ST_IDLE)
        for b in (self.b_bt, self.b_nl):
            en(b, not busy and st['braketest'] == ST_IDLE and st['one_launch'] == ST_IDLE)
        for x, k in ((self.x_one, 'one_launch'), (self.x_prompt, 'prompt'), (self.x_bt, 'braketest')):
            en(x, not busy and st[k] == ST_RUN)
        en(self.b_kill, not busy)
        en(self.b_check, not busy)
        self.foot.configure(text=self.note)
        self.root.after(POLL_MS, self.tick)


def main(args=None):
    root = tk.Tk()
    panel = Panel(root)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        #  ★띄운 터미널에서 Ctrl+C★ — 자식들은 자기 세션이라 그 신호가 안 닿는다. 대신 전한다
        #  (kill 은 돌리지 않는다 — 그것은 [Kill]·[Exit] 의 일이다).
        for s in panel.slots.values():
            s.stop()


if __name__ == '__main__':
    main()
