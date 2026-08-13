#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
prompt_g.py ― GUI 메인화면 [white1]
════════════════════════════════════════════════════════════════════════════════
 prompt.py 와 ★기능이 같다★ — 조작 수단만 CLI 에서 tkinter 창으로 바뀐 것이다.
════════════════════════════════════════════════════════════════════════════════
    ros2 run white1 prompt_g          (one_launch.py 를 띄운 뒤 별 터미널에서)

  둘 중 하나만 띄운다. 같이 띄워도 깨지지는 않지만(둘 다 /drive_cmd 를 쏘는
  발행자일 뿐이다) 두 화면이 서로 다른 대기 상태를 들고 있게 되어 헷갈린다.

  ┌ 화면 ─────────────────────────────────────────────────────────────────────┐
  │   {/speed}                                              [ 매 핑 ]         │
  │   {/encoder}                                            [ 주 행 ]         │
  │                                                                           │
  │   {조향각 명령}                                                            │
  │   {주행펄스 명령}                                        [ 종 료 ]         │
  └───────────────────────────────────────────────────────────────────────────┘

  · {/speed}    speed.py 의 IMU 적분 속도 [km/h] 를 그대로.
  · {/encoder}  A보드 펄스 합 × 3.18 [km/h]. ★환산 상수 하나짜리 근사★ 이고
                driving 의 MAX_PULSE_LIMIT(4펄스 ≈ 12.7km/h) 와 같은 척도다.
  · {조향각 명령}·{주행펄스 명령}  /cmd_vel_raw 로 나가는 최종 지령
                (angular.z = 조향 −좌/+우, linear.x = 펄스). ★실측이 아니라
                지령★ 이다 — 실제 조향각은 /steer_angle_measured 다.
  · 값이 2초 넘게 안 들어오면 '—' 로 바꾼다. 죽은 노드의 마지막 숫자를 현재값인
    양 띄워 두는 것이 제일 위험하다(IDLE 에서 /cmd_vel_raw 가 멎는 것은 정상).

  ┌ 버튼 ─────────────────────────────────────────────────────────────────────┐
  │  [매핑]  스위치가 수동조종이면 즉시 MAP_START, 자율주행이면 내릴 때까지     │
  │          대기했다가 전환되는 순간 보낸다. 매핑 중에는 [매핑 중단] 이 된다.  │
  │  [주행]  파일 선택 창(gps_data)에서 경로 CSV 를 고르면 그 이름을 보내고,    │
  │          스위치가 자율주행이면 즉시 DRIVE_START, 수동조종이면 올릴 때까지   │
  │          대기한다. 주행 중에는 [주행 중단] 이 된다.                        │
  │  [종료]  ★이 창(노드)만 닫는다★ one_launch.py 로 띄운 노드들은 그대로      │
  │          살아 있다. 진행 중인 동작이 있으면 정지 명령을 보낼지 먼저 묻는다. │
  └───────────────────────────────────────────────────────────────────────────┘

  대기·시작 판정은 prompt.py 와 같은 규칙이다 — 이 화면이 /vehicle_mode 를 스스로
  폴링해서 목표 위치가 되는 순간 START 를 보낸다. 안전 게이트(정말 그 위치인가)는
  driving.py 의 cb_drive_cmd 가 명령을 받을 때 다시 본다.

  ★스레드★ rclpy 는 별 스레드에서 돌고 tkinter 는 메인 스레드가 잡는다. ROS
  콜백은 값을 대입만 하고 위젯을 만지지 않는다 — 화면 갱신은 메인 스레드의
  after() 틱 한 곳에서만 한다(Tk 는 스레드 안전하지 않다).
"""

import os
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, Int32, String

from white1 import paths

# ★음성 안내★ prompt.py 와 같은 셋만 이 화면이 낸다(시작 인사 + 대기 안내 2종).
#   나머지는 nxde 의 sound 노드가 토픽을 보고 낸다 — prompt.py 헤더의 같은 주석 참고.
try:
    from nxde.sound import Player, SND_PROMPT, SND_WAIT_MAP, SND_WAIT_DRIVE
except Exception:                       # noqa: BLE001
    Player = None
    SND_PROMPT = SND_WAIT_MAP = SND_WAIT_DRIVE = ''

try:
    import tkinter as tk
    from tkinter import filedialog, messagebox
    from tkinter import font as tkfont
except ImportError as exc:      # ROS 데스크톱 설치에는 보통 들어 있다
    raise SystemExit(
        "tkinter 가 없다 — `sudo apt install python3-tk` 후 다시 실행할 것") from exc


PULSE_TO_KMH = 3.18     # A보드 펄스 1 ≈ 3.18 km/h (driving.py 의 환산과 동일)
UI_PERIOD_MS = 100      # 화면 갱신 주기
STALE_S      = 2.0      # 이보다 오래된 값은 '—'

BG      = '#1c1f24'
BG_BOX  = '#111318'
FG      = '#e8eaed'
FG_DIM  = '#8b929c'
FG_VAL  = '#7fd4a2'
FG_WARN = '#ffb454'


class PromptGuiNode(Node):
    """구독·발행만 한다. 판단은 전부 App 쪽(메인 스레드)에서."""

    def __init__(self):
        super().__init__('prompt_gui_node')
        self.declare_parameter('data_dir', '')
        self.data_dir = paths.data_dir(self.get_parameter('data_dir').value or '')

        self.pub_cmd = self.create_publisher(String, '/drive_cmd', 10)

        self.state = 'IDLE'
        self.auto_mode = None
        self.estop = False
        self.selected = ''
        self.last_event = ''

        # (값, 수신시각) — 시각이 있어야 '멎은 값'을 구별한다
        self.speed_kmh = (None, 0.0)
        self.enc_pulse = (None, 0.0)
        self.cmd_steer = (None, 0.0)
        self.cmd_pulse = (None, 0.0)

        self.create_subscription(String,  '/drive_state',  self.cb_state, 10)
        self.create_subscription(String,  '/drive_event',  self.cb_event, 10)
        self.create_subscription(Bool,    '/vehicle_mode', self.cb_mode, 10)
        self.create_subscription(Bool,    '/estop',        self.cb_estop, 10)
        self.create_subscription(Float32, '/speed',        self.cb_speed, 10)
        self.create_subscription(Int32,   '/encoder',      self.cb_encoder, 10)
        self.create_subscription(Twist,   '/cmd_vel_raw',  self.cb_cmd, 10)

        self.sound = Player(log=self.get_logger().warning) if Player else None
        self.play(SND_PROMPT)          # 창이 뜨자마자

    def play(self, name):
        """음성 안내. 재생기·음원이 없어도 조용히 지나간다."""
        if self.sound and name:
            self.sound.play(name)

    # ── 콜백 ★위젯을 만지지 않는다★ ──────────────────────────────────────────
    def cb_state(self, m):
        self.state = str(m.data)

    def cb_event(self, m):
        self.last_event = str(m.data)

    def cb_mode(self, m):
        self.auto_mode = bool(m.data)

    def cb_estop(self, m):
        self.estop = bool(m.data)

    def cb_speed(self, m):
        self.speed_kmh = (float(m.data), time.monotonic())

    def cb_encoder(self, m):
        self.enc_pulse = (int(m.data), time.monotonic())

    def cb_cmd(self, m):
        now = time.monotonic()
        self.cmd_pulse = (float(m.linear.x), now)
        self.cmd_steer = (float(m.angular.z), now)

    def send(self, text: str):
        self.pub_cmd.publish(String(data=text))


class App:

    def __init__(self, node: PromptGuiNode):
        self.node = node
        self.pending = None            # None | 'MAP' | 'DRIVE' — 스위치 전환 대기

        self.root = tk.Tk()
        self.root.title('white1 — 매핑 / 주행')
        self.root.configure(bg=BG)
        self.root.minsize(640, 420)
        self.root.protocol('WM_DELETE_WINDOW', self.on_quit)

        base = tkfont.nametofont('TkDefaultFont')
        self.f_cap  = base.copy(); self.f_cap.configure(size=11)
        self.f_val  = base.copy(); self.f_val.configure(size=26, weight='bold')
        self.f_btn  = base.copy(); self.f_btn.configure(size=16, weight='bold')
        self.f_bar  = base.copy(); self.f_bar.configure(size=11)

        self._build()
        self.tick()

    # ── 화면 구성 ──────────────────────────────────────────────────────────────
    def _build(self):
        r = self.root
        r.columnconfigure(0, weight=1)
        r.columnconfigure(1, weight=0)

        self.v_status = tk.StringVar(value='상태 —')
        tk.Label(r, textvariable=self.v_status, font=self.f_bar, bg=BG, fg=FG,
                 anchor='w', padx=14, pady=8).grid(
            row=0, column=0, columnspan=2, sticky='ew')

        self.v_speed = self._box(r, 1, '/speed  (IMU 환산)')
        self.v_enc   = self._box(r, 2, '/encoder  (펄스 × 3.18)')
        tk.Frame(r, height=14, bg=BG).grid(row=3, column=0)
        self.v_steer = self._box(r, 4, '조향각 명령  (−좌 / +우)')
        self.v_pulse = self._box(r, 5, '주행펄스 명령')

        self.b_map = tk.Button(r, text='매 핑', font=self.f_btn, width=9,
                               command=self.on_map)
        self.b_map.grid(row=1, column=1, padx=(10, 16), pady=6, sticky='ew')

        self.b_drive = tk.Button(r, text='주 행', font=self.f_btn, width=9,
                                 command=self.on_drive)
        self.b_drive.grid(row=2, column=1, padx=(10, 16), pady=6, sticky='ew')

        self.b_quit = tk.Button(r, text='종 료', font=self.f_btn, width=9,
                                command=self.on_quit)
        self.b_quit.grid(row=5, column=1, padx=(10, 16), pady=6, sticky='ew')

        self.v_msg = tk.StringVar(value='')
        tk.Label(r, textvariable=self.v_msg, font=self.f_bar, bg=BG, fg=FG_DIM,
                 anchor='w', padx=14, pady=8, wraplength=640, justify='left').grid(
            row=6, column=0, columnspan=2, sticky='ew')
        r.rowconfigure(6, weight=1)

    def _box(self, parent, row, caption):
        """{네모박스} 하나 — 위에 작은 이름, 아래에 큰 값."""
        frame = tk.Frame(parent, bg=BG_BOX, bd=2, relief='groove')
        frame.grid(row=row, column=0, sticky='ew', padx=(16, 6), pady=6)
        tk.Label(frame, text=caption, font=self.f_cap, bg=BG_BOX, fg=FG_DIM,
                 anchor='w').pack(fill='x', padx=12, pady=(6, 0))
        var = tk.StringVar(value='—')
        tk.Label(frame, textvariable=var, font=self.f_val, bg=BG_BOX, fg=FG_VAL,
                 anchor='w').pack(fill='x', padx=12, pady=(0, 8))
        return var

    def say(self, text):
        self.v_msg.set(text)

    # ── 버튼 ───────────────────────────────────────────────────────────────────
    def on_map(self):
        st = self.node.state
        if st.startswith('MAP_'):                  # 진행 중 → 중단
            self.node.send('STOP')
            self.say('🛑 매핑 중단 요청 — 여기까지가 저장된다')
            return
        if self.pending == 'MAP':                  # 대기 중 → 취소
            self.pending = None
            self.say('매핑 대기를 취소했다')
            return
        if not self._startable(for_drive=False):
            return
        if self.node.auto_mode is False:
            self.node.send('MAP_START')
            self.say('🗺️ 매핑 시작 — 페달로 곧게 굴려 헤딩을 잡을 것')
        else:
            self.pending = 'MAP'
            self.node.play(SND_WAIT_MAP)               # "스위치를 수동조종으로"
            self.say('⏳ 스위치를 수동조종으로 내리면 그 순간 매핑을 시작한다')

    def on_drive(self):
        st = self.node.state
        if st.startswith('DRIVE_'):                # 진행 중 → 중단
            self.node.send('STOP')
            self.say('🛑 주행 중단 요청 — 리니어 2단')
            return
        if self.pending == 'DRIVE':
            self.pending = None
            self.say('주행 대기를 취소했다')
            return
        if not self._startable(for_drive=True):
            return

        data_dir = self.node.data_dir
        os.makedirs(data_dir, exist_ok=True)
        path = filedialog.askopenfilename(
            parent=self.root, title='주행할 경로 CSV 선택',
            initialdir=data_dir,
            filetypes=[('경로 CSV', '*.csv'), ('모든 파일', '*.*')])
        if not path:
            self.say('경로를 고르지 않았다')
            return
        # ★driving 은 data_dir + 파일명 으로 연다★ 다른 폴더의 파일을 골라도
        #   이름만 넘어가 '경로 파일 없음' 이 된다 — 여기서 미리 걸러 준다.
        if os.path.realpath(os.path.dirname(path)) != os.path.realpath(data_dir):
            messagebox.showwarning(
                '경로 폴더', f'경로 CSV 는 아래 폴더 안에 있어야 한다:\n{data_dir}',
                parent=self.root)
            return

        name = os.path.basename(path)
        self.node.selected = name
        self.node.send(name)                       # 경로 선택
        if self.node.auto_mode is True:
            self.node.send('DRIVE_START')
            self.say(f'▶ 주행 시작 — {name}')
        else:
            self.pending = 'DRIVE'
            self.node.play(SND_WAIT_DRIVE)             # "스위치를 자율주행으로"
            self.say(f'⏳ {name} 선택됨 — 스위치를 자율주행으로 올리면 출발한다')

    def on_quit(self):
        st = self.node.state
        if st != 'IDLE':
            ok = messagebox.askyesno(
                '종료',
                f'{st} 진행 중이다.\n\n정지(STOP) 명령을 보내고 이 창만 닫는다.\n'
                'one_launch.py 로 띄운 노드들은 그대로 살아 있다.\n\n계속할까?',
                parent=self.root)
            if not ok:
                return
            self.node.send('STOP')
        self.root.destroy()

    def _startable(self, for_drive):
        """시작 버튼을 눌러도 되는 상황인가 — 아니면 이유를 띄우고 False.

        ★[2026-08-14] E-STOP 은 주행에만 건다★ 매핑은 E-STOP 중에도 시작할 수 있다
        (driving.cb_estop 주석 — 차가 하드웨어로 멈춰 있어 점이 쌓이지 않는다).
        """
        if self.node.auto_mode is None:
            self.say('⚠️ 주행모드(D5)를 아직 못 받았다 — nxde arduino / B보드 확인')
            return False
        if for_drive and self.node.estop:
            self.say('🚨 E-STOP 체결 중 — 자율주행은 시작할 수 없다'
                     ' (D12 를 되돌려야 한다. 매핑은 가능)')
            return False
        if self.node.state != 'IDLE':
            self.say(f'지금은 시작할 수 없다 — 현재 상태 {self.node.state}')
            return False
        return True

    # ── 주기 갱신 ──────────────────────────────────────────────────────────────
    def tick(self):
        if not rclpy.ok():
            self.root.destroy()
            return

        n = self.node
        # ★상태가 IDLE 을 벗어나면 대기 흐름은 의미가 없다★ (prompt.py 와 같은 규칙)
        if n.state != 'IDLE' and self.pending:
            self.pending = None

        # ★E-STOP 이 걸리면 주행 대기를 접는다 [2026-08-14]★ 스위치를 올려도 driving
        #   이 거절하므로 대기 상태로 붙잡아 두면 사람만 헷갈린다(매핑 대기는 그대로).
        if n.estop and self.pending == 'DRIVE':
            self.pending = None
            self.say('🚨 E-STOP 체결 — 주행 대기를 취소했다 (매핑은 가능)')

        # 목표 스위치 위치가 됐으면 그 순간 START 를 보낸다
        if self.pending == 'MAP' and n.auto_mode is False:
            n.send('MAP_START')
            self.pending = None
            self.say('🗺️ 수동조종 전환 감지 — 매핑 시작')
        elif self.pending == 'DRIVE' and n.auto_mode is True:
            n.send('DRIVE_START')
            self.pending = None
            self.say(f'▶ 자율주행 전환 감지 — 주행 시작 [{n.selected}]')

        now = time.monotonic()
        self.v_speed.set(self._fmt(n.speed_kmh, now, '{:.1f} km/h'))
        enc, t_enc = n.enc_pulse
        self.v_enc.set(self._fmt(
            (None if enc is None else enc * PULSE_TO_KMH, t_enc), now, '{:.1f} km/h'))
        self.v_steer.set(self._fmt(n.cmd_steer, now, '{:+.1f} °'))
        self.v_pulse.set(self._fmt(n.cmd_pulse, now, '{:.0f} 펄스'))

        self.v_status.set(self._status_line())
        self._refresh_buttons()
        self.root.after(UI_PERIOD_MS, self.tick)

    @staticmethod
    def _fmt(pair, now, form):
        val, stamp = pair
        if val is None or (now - stamp) > STALE_S:
            return '—'
        return form.format(val)

    def _status_line(self):
        n = self.node
        if n.auto_mode is None:
            mode = '모드 미수신'
        else:
            mode = '자율주행' if n.auto_mode else '수동조종'
        wait = {'MAP': '  |  매핑 대기 중', 'DRIVE': '  |  주행 대기 중'}.get(
            self.pending, '')
        est = '  |  🚨 E-STOP' if n.estop else ''
        route = f'  |  경로: {n.selected}' if n.selected else ''
        line = f'상태 {n.state}  |  {mode}{route}{wait}{est}'
        if n.last_event:
            line += f'\n{n.last_event}'
        return line

    def _refresh_buttons(self):
        st = self.node.state
        # ★'ESTOP' 상태는 없어졌다 [2026-08-14]★ E-STOP 중에는 ★주행 버튼만★ 막고
        #   매핑은 열어 둔다(driving.cb_estop). 체결 사실은 상태줄이 계속 띄운다.
        if st.startswith('MAP_'):
            self._set(self.b_map, '매핑 중단', True)
            self._set(self.b_drive, '주 행', False)
        elif st.startswith('DRIVE_'):
            self._set(self.b_map, '매 핑', False)
            self._set(self.b_drive, '주행 중단', True)
        else:                                       # IDLE
            self._set(self.b_map,
                      '대기 취소' if self.pending == 'MAP' else '매 핑',
                      self.pending in (None, 'MAP'))
            self._set(self.b_drive,
                      '대기 취소' if self.pending == 'DRIVE' else '주 행',
                      self.pending in (None, 'DRIVE') and not self.node.estop)

    @staticmethod
    def _set(button, text, enabled):
        if button['text'] != text:
            button.config(text=text)
        want = 'normal' if enabled else 'disabled'
        if str(button['state']) != want:
            button.config(state=want)

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = PromptGuiNode()

    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    try:
        App(node).run()
    except KeyboardInterrupt:
        pass
    except tk.TclError as e:
        # 화면 없는 터미널(ssh 등)에서 흔한 실패 — CLI 판을 안내한다
        node.get_logger().error(
            f"창을 열 수 없다({e}) — DISPLAY 가 없으면 `ros2 run white1 prompt` 를 쓸 것")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
