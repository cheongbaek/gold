#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""joystick — ★조이스틱 + LCD 단독 점검 도구★ [nxde]

    ros2 run nxde joystick

═══════════════════════════════════════════════════════════════════════════════
 이 노드의 목적은 하나다 — ★조이스틱과 LCD 가 제대로 붙었는가★
═══════════════════════════════════════════════════════════════════════════════
  · 스틱·버튼이 화면에 그대로 보이는가 (L/R 패드 · K1/K2 · SWA/SWB)
  · 영점이 잡히는가, 보드 리셋 버튼으로 다시 잡히는가
  · 그 위치가 ★펄스 0~15 / 조향 −40~+40 / 제동 0~2★ 로 맞게 환산되는가
  · ★그 값이 조이스틱 LCD 에 그대로 찍히는가★ (`D,...` 줄을 이 노드가 직접 쓴다)

  ★환산 규칙을 여기서 다시 적지 않는다★ `nxde/joyread.py` 가 단일 소유자이고,
  실제 주행(`nxde/arduino.py` 의 조이스틱 분기)이 쓰는 것과 ★같은 함수★ 를 쓴다.
  그래야 "점검 도구에서는 맞는데 실차에서 다르다" 가 생기지 않는다.

═══════════════════════════════════════════════════════════════════════════════
 ⚠️ ★단독 실행 전용이다★
═══════════════════════════════════════════════════════════════════════════════
  실제 주행에서 조이스틱을 잡는 것은 ★`nxde/arduino` 노드★ 다(2026-09-16).
  이 노드는 그 자리를 대신하지 않고, 같은 포트를 두고 다투지 않도록 ★따로 띄우는
  것을 전제★ 로 한다. 동시 실행을 방어하지 않는다 — 먼저 연 쪽이 포트를 가져가고
  나머지는 '못 찾음' 이 된다(배타 open).

      점검할 때      ros2 run nxde joystick          ← 이 노드 하나만
      주행할 때      ros2 launch white1 one_launch.py ← arduino 가 잡고 직접 몬다

  ★★ 이 노드는 차를 움직이는 명령을 하나도 내보내지 않는다 ★★
  `/cmd_vel_raw`(주행펄스·★조향★) · `/control_state` · `/brake_level` 어느 것도
  발행하지 않는다. 켜는 스위치도 두지 않았다 — 조이스틱과 LCD 배선을 보는 도구에
  차가 굴러갈 경로가 있으면, 그 경로는 언젠가 사람이 의도하지 않은 순간에 열린다.
  ★조이스틱으로 실제로 모는 것은 `nxde/arduino` 하나다★ (2026-09-16).

  그래서 이 노드가 시리얼로 쓰는 줄은 ★LCD 표시값(`D,...`) 한 종류뿐★ 이고,
  A/B 보드에는 한 글자도 보내지 않는다.

═══════════════════════════════════════════════════════════════════════════════
 조작
═══════════════════════════════════════════════════════════════════════════════
    L스틱 위      주행 펄스 0~pulse_max
    L스틱 아래    브레이크 0 / 1 / 2 단 (중앙~맨아래 3등분)
    R스틱 좌우    조향 −40~+40  (★− 좌 / + 우★)
    SWA 짧게      ON / OFF 토글 → ★LCD 우측의 ON·OFF 가 이걸 따라간다★
                  (실차에서는 이것이 '조종 시작/일시정지' 다 — 여기서는 표시뿐이다)
    SWB           보드 안의 스톱워치(짧게=시작/일시정지, 길게=00:00:00). 화면엔
                  눌림만 보이고 ROS 는 아무것도 하지 않는다 — 스톱워치의 주인은 보드다.
    보드 리셋      스틱 영점 재보정 ("J,RESET" 한 줄이 그 신호다)

  ★LCD 로 보내는 값은 SWA 와 무관하게 언제나 '지금 스틱이 만드는 값' 이다★
  OFF 일 때 0 으로 지우면 SWA 를 켜야만 숫자를 확인할 수 있어 점검이 번거로워진다.
  SWA 가 정하는 것은 ★LCD 의 ON/OFF 표시★ 하나뿐이다.

  ★GPS 속도 칸은 언제나 00 이다★ (사용자 지시) 이 노드는 `/gps_fused` 를 보지
  않는다. 속도를 '모른다'는 뜻으로 −1 을 보내고, 보드가 그것을 00 으로 찍는다.
"""

import threading
import time
import tkinter as tk

import rclpy
import rclpy.executors
from rclpy.node import Node
import serial

from nxde import joyread
from nxde.proc_guard import watch_parent
# 포트 후보 목록의 소유자는 arduino.py 다 — GPS/IMU VID/PID 와 udev 링크
# (/dev/gps · /dev/imu)가 그쪽에서 이미 빠진다. 표를 두 벌로 만들지 않는다.
from nxde.arduino import candidate_ports


BAUD = 115200
VERIFY_WINDOW_S = 3.5      # 후보 포트 하나당 "J," 를 기다려볼 최대 시간
RECONNECT_S = 3.0          # 못 찾았을 때 재탐색 간격

UPDATE_MS = 50             # 화면·전송 틱 (20Hz — 실차 제어주기와 같은 결)
LCD_KEEPALIVE_S = 0.5      # 값이 안 바뀌어도 이 주기로 다시 보낸다
                           #   ★보드는 1.5s 끊기면 화면을 OFF·00 으로 되돌린다★
                           #   (joyled.ino LINK_STALE_MS) — 그보다 넉넉히 짧게.
LCD_KMH_NONE = -1          # ★속도를 모른다★ → 보드가 00 으로 찍는다

# ── 화면 ────────────────────────────────────────────────────────────────────
PAD      = 240     # 스틱 패드 한 변
DOT_R    = 9       # 스틱 위치 점
CENTER_R = 12      # 스틱 버튼(누름) 표시 원
SW_R     = 16      # 스위치 원

BG          = '#1e1e1e'
PAD_BG      = '#2b2b2b'
LINE        = '#4a4a4a'
DOT_COLOR   = '#4fc3f7'
PRESS_COLOR = '#ef5350'
IDLE_COLOR  = '#3a3a3a'
OK_COLOR    = '#66bb6a'
WARN_COLOR  = '#ffb454'
TEXT        = '#dddddd'
DIM         = '#777777'


# ══════════════════════════════════════════════════════════════════════════════
#  포트 탐색
# ══════════════════════════════════════════════════════════════════════════════
def find_joystick_port(log, port_hint='', exclude=None):
    """"J," 를 보내는 포트를 찾아 연다 → (device, serial). 못 찾으면 (None, None).

    ★조이스틱은 A/B 보드와 같은 메가라 VID/PID 로는 구분되지 않는다★ 실제로 열어
    "J," 줄이 오는지 확인한 뒤에만 채택하고, 아니면 곧바로 닫는다(포트를 열고 있는
    동안 그 보드는 DTR 리셋으로 멎어 있다).
    """
    if port_hint:
        devices = [port_hint]
    else:
        devices = list(candidate_ports(exclude=exclude))
        if not devices:
            log("열어 볼 시리얼 포트가 하나도 없습니다 — USB 연결을 확인하세요")
    for device in devices:
        ser = None
        try:
            ser = serial.Serial(device, BAUD, timeout=1, exclusive=True)
            deadline = time.monotonic() + VERIFY_WINDOW_S
            while time.monotonic() < deadline:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if joyread.is_joy_line(line):
                    ser.timeout = 0.2
                    log(f"[조이스틱 연결] {device}")
                    return device, ser
        except (serial.SerialException, OSError) as e:
            log(f"[건너뜀] {device} — {e}")
        if ser is not None:
            try:
                ser.close()
            except (serial.SerialException, OSError):
                pass
    return None, None


# ══════════════════════════════════════════════════════════════════════════════
#  ROS 노드 — ★기본은 아무것도 발행하지 않는다★
# ══════════════════════════════════════════════════════════════════════════════
class JoyTestNode(Node):

    def __init__(self):
        super().__init__('joystick')

        self.port_hint = str(self.declare_parameter('port', '').value or '')
        self.pulse_max = max(1, min(joyread.PULSE_MAX,
                                    int(self.declare_parameter('pulse_max',
                                                               joyread.PULSE_MAX).value)))
        self.deadzone = int(self.declare_parameter('deadzone_raw',
                                                   joyread.DEADZONE_RAW).value)
        self.exclude_ports = [
            str(p) for p in self.declare_parameter('exclude_ports', ['']).value if str(p)]

        self.joy = joyread.JoystickState(deadzone=self.deadzone,
                                         pulse_max=self.pulse_max,
                                         log=self.get_logger().info)
        self.joy.reset()

        # ── 시리얼 ──
        self.lock = threading.Lock()     # ser 를 읽는 스레드와 쓰는 틱이 다르다
        self.ser = None
        self.port = None
        self.rx_lines = 0
        self.last_raw = ''
        self._running = True

        # ── 조작 상태 ──
        self.swa_on = False              # SWA 토글 (LCD 의 ON/OFF 표시)
        self.cmd = (0, 0, 0)             # 지금 스틱이 만드는 (펄스, 조향, 제동)
        self._last_lcd = None
        self._last_lcd_t = 0.0
        self.lcd_line = ''               # 마지막으로 보낸 D 줄 (화면 표시)

        # ── ROS ──
        #   ★발행하는 토픽이 하나도 없다★ 구독도 없다. 이 노드가 바깥으로 내보내는
        #   것은 조이스틱 보드로 나가는 LCD 표시줄(D,...) 하나뿐이다(파일 헤더).
        self.get_logger().info(
            "🔍 조이스틱·LCD 점검 — ★차를 움직이는 명령은 하나도 내보내지 않습니다★ "
            "(/cmd_vel_raw·/control_state·/brake_level 전부 발행하지 않는다). "
            "조이스틱으로 모는 것은 arduino 노드입니다")

        self._reader = threading.Thread(target=self._reader_loop,
                                        name='joy_serial', daemon=True)
        self._reader.start()

    # ── 시리얼 리더 ───────────────────────────────────────────────────
    def _reader_loop(self):
        buf = b''
        while self._running and rclpy.ok():
            if self.ser is None:
                device, ser = find_joystick_port(
                    self.get_logger().info, self.port_hint, self.exclude_ports)
                if ser is None:
                    self.get_logger().warn(
                        f"조이스틱 보드를 찾지 못했습니다 — {RECONNECT_S:.0f}초 후 재시도 "
                        f"(연결·전원·펌웨어 확인. ★arduino 노드가 이미 잡고 있으면 "
                        f"여기서는 못 엽니다★ — 이 도구는 단독 실행 전용입니다)",
                        throttle_duration_sec=15.0)
                    for _ in range(int(RECONNECT_S * 10)):
                        if not self._running:
                            return
                        time.sleep(0.1)
                    continue
                with self.lock:
                    self.ser, self.port = ser, device
                buf = b''
                self.joy.reset('조이스틱 연결')

            ser = self.ser          # ★지역 참조★ 다른 곳에서 None 으로 떨어뜨린다
            if ser is None:
                continue
            try:
                data = ser.read(4096)
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f"조이스틱 단절: {e} → 재연결 시도")
                self._drop()
                continue
            if not data:
                continue
            buf += data
            if b'\n' not in buf:
                continue
            lines = buf.split(b'\n')
            buf = lines[-1]
            for raw in lines[:-1]:
                text = raw.decode('utf-8', errors='ignore').strip()
                if not text:
                    continue
                self.rx_lines += 1
                self.last_raw = text
                self.joy.feed_line(text)

    def _drop(self):
        with self.lock:
            ser, self.ser = self.ser, None
            self.port = None
        if ser is not None:
            try:
                ser.close()
            except (serial.SerialException, OSError):
                pass
        self.joy.reset('조이스틱 단절')
        self._last_lcd = None

    # ── 틱 (GUI 메인스레드에서 부른다) ────────────────────────────────
    def tick(self):
        """스냅샷 → 환산 → LCD 전송 (+ 켜져 있으면 토픽 발행). 스냅샷을 돌려준다."""
        snap = self.joy.snapshot()          # ★한 틱에 한 번★ (SWA 카운터를 비운다)

        # SWA 릴리즈 = ON/OFF 토글. ★영점 전에는 받지 않는다★ — 실차(arduino)의
        #   규칙과 같게 두어, 여기서 익힌 조작이 그대로 통하게 한다.
        for _ in range(snap['swa_releases']):
            if not snap['calibrated']:
                self.get_logger().warn("SWA 무시 — 영점이 아직 안 끝났습니다")
                break
            self.swa_on = not self.swa_on

        self.cmd = self.joy.command(snap)
        self._tx_lcd()
        return snap

    def _tx_lcd(self):
        """"D,<on>,<pulse>,<steer>,<kmh>" — ★GPS 속도는 언제나 −1(모름)★

        값이 바뀌었거나 LCD_KEEPALIVE_S 가 지났을 때만 쓴다. 매 틱 쓰면 시리얼이
        쉴 새 없이 차는데, 보드가 그 줄을 화면 갱신(10Hz)보다 빨리 소화할 이유가 없다.
        """
        pulse, steer, _brake = self.cmd
        line = f'D,{1 if self.swa_on else 0},{pulse},{steer},{LCD_KMH_NONE}'
        now = time.monotonic()
        if line == self._last_lcd and (now - self._last_lcd_t) < LCD_KEEPALIVE_S:
            return
        with self.lock:
            ser = self.ser
            if ser is None:
                return
            try:
                ser.write((line + '\n').encode('ascii'))
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f"LCD 전송 실패: {e}")
                threading.Thread(target=self._drop, daemon=True).start()
                return
        self._last_lcd = line
        self._last_lcd_t = now
        self.lcd_line = line

    # ── 종료 ─────────────────────────────────────────────────────────
    def stop_and_close(self):
        """종료 정리. ★정지값을 낼 곳이 없다★ — 애초에 아무것도 발행하지 않으므로
        이 노드가 죽어서 차가 계속 가는 경로 자체가 없다. 포트만 놓으면 끝이다."""
        if not self._running:
            return
        self._running = False
        with self.lock:
            ser, self.ser = self.ser, None
        if ser is not None:
            try:
                ser.close()
            except (serial.SerialException, OSError):
                pass


# ══════════════════════════════════════════════════════════════════════════════
#  화면
# ══════════════════════════════════════════════════════════════════════════════
def axis_to_screen(n):
    """−1..1 → 패드 좌표. n=1(raw 1023 쪽) → 0, n=0 → 가운데, n=−1 → PAD."""
    return (1 - n) / 2 * PAD


class JoyGui:

    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.running = True

        root.title("nxde joystick — 조이스틱 · LCD 점검")
        root.configure(bg=BG)

        self.status = tk.Label(root, text="조이스틱을 찾는 중...", bg=BG, fg=TEXT,
                               font=("Consolas", 12), wraplength=780, justify='center')
        self.status.pack(pady=(14, 8))

        main = tk.Frame(root, bg=BG)
        main.pack(padx=16)
        self.pad_l = self._make_pad(main, "L  (위=엑셀 / 아래=브레이크)", 0, 'h')
        self._make_switches(main)
        self.pad_r = self._make_pad(main, "R  (좌우=조향)", 2, 'v')

        self._make_values(root)

        root.bind('<KeyPress-q>', lambda e: self._on_close())
        root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._tick()

    # ── 위젯 ─────────────────────────────────────────────────────────
    def _make_pad(self, parent, label, column, highlight):
        frame = tk.Frame(parent, bg=BG)
        frame.grid(row=0, column=column, padx=12)
        tk.Label(frame, text=label, bg=BG, fg=TEXT,
                 font=("Consolas", 11, "bold")).pack()
        canvas = tk.Canvas(frame, width=PAD, height=PAD, bg=PAD_BG,
                           highlightthickness=1, highlightbackground=LINE)
        canvas.pack()
        #  ★쓰는 축만 하늘색으로 강조한다★ L 은 세로(Y1), R 은 가로(X2) 하나씩만
        #  차량 지령이 되고 나머지 축은 아무 일도 하지 않는다 — 화면이 그 사실을
        #  그대로 보여 주면 "왜 이 축은 반응이 없지" 가 생기지 않는다.
        if highlight == 'h':
            canvas.create_line(PAD / 2, 0, PAD / 2, PAD, fill=LINE)
            hi = canvas.create_line(0, PAD / 2, PAD, PAD / 2, fill=DOT_COLOR)
        else:
            hi = canvas.create_line(PAD / 2, 0, PAD / 2, PAD, fill=DOT_COLOR)
            canvas.create_line(0, PAD / 2, PAD, PAD / 2, fill=LINE)
        k = canvas.create_oval(PAD / 2 - CENTER_R, PAD / 2 - CENTER_R,
                               PAD / 2 + CENTER_R, PAD / 2 + CENTER_R,
                               fill="", outline=LINE)
        dot = canvas.create_oval(0, 0, 0, 0, fill=DOT_COLOR, outline="")
        value = tk.Label(frame, text="X:---- Y:----", bg=BG, fg=TEXT,
                         font=("Consolas", 10))
        value.pack()
        return {'canvas': canvas, 'dot': dot, 'k': k, 'hi': hi,
                'label': value, 'highlight': highlight}

    def _make_switches(self, parent):
        frame = tk.Frame(parent, bg=BG)
        frame.grid(row=0, column=1, padx=10)
        self.sw_ids = {}
        for name, note in (("SWA", "ON/OFF"), ("SWB", "스톱워치")):
            box = tk.Frame(frame, bg=BG)
            box.pack(pady=10)
            c = tk.Canvas(box, width=SW_R * 2 + 6, height=SW_R * 2 + 6,
                          bg=BG, highlightthickness=0)
            c.pack()
            oid = c.create_oval(3, 3, SW_R * 2 + 3, SW_R * 2 + 3,
                                fill=IDLE_COLOR, outline=LINE)
            tk.Label(box, text=name, bg=BG, fg=TEXT,
                     font=("Consolas", 10, "bold")).pack()
            tk.Label(box, text=note, bg=BG, fg=DIM,
                     font=("Consolas", 8)).pack()
            self.sw_ids[name] = (c, oid)
        self.onoff = tk.Label(frame, text="OFF", bg=IDLE_COLOR, fg=TEXT, width=7,
                              font=("Consolas", 14, "bold"), pady=4)
        self.onoff.pack(pady=(14, 0))
        tk.Label(frame, text="LCD 우측", bg=BG, fg=DIM,
                 font=("Consolas", 8)).pack()

    def _make_values(self, root):
        box = tk.Frame(root, bg=BG)
        box.pack(pady=(12, 4))
        self.val_vars = {}
        for col, (key, title, unit) in enumerate((('pulse', "펄스", "0~15"),
                                                  ('steer', "조향", "−좌 / +우"),
                                                  ('brake', "제동", "0~2단"))):
            cell = tk.Frame(box, bg=BG)
            cell.grid(row=0, column=col, padx=26)
            tk.Label(cell, text=title, bg=BG, fg=TEXT,
                     font=("Consolas", 12, "bold")).pack()
            var = tk.StringVar(value="0")
            tk.Label(cell, textvariable=var, bg=BG, fg=OK_COLOR,
                     font=("Consolas", 30, "bold")).pack()
            tk.Label(cell, text=unit, bg=BG, fg=DIM, font=("Consolas", 8)).pack()
            self.val_vars[key] = var

        self.lcd_label = tk.Label(root, text="LCD 전송: (아직 없음)", bg=BG,
                                  fg=DOT_COLOR, font=("Consolas", 12, "bold"))
        self.lcd_label.pack(pady=(6, 2))
        tk.Label(root,
                 text="LCD 2행 = REF:<펄스> <조향> <속도> km   ·   "
                      "★속도 칸은 이 도구에서 언제나 00 이다★ (GPS 를 보지 않는다)",
                 bg=BG, fg=DIM, font=("Consolas", 9)).pack()
        self.rx_label = tk.Label(root, text="", bg=BG, fg=DIM,
                                 font=("Consolas", 9))
        self.rx_label.pack(pady=(4, 12))

    # ── 갱신 ─────────────────────────────────────────────────────────
    def _update_pad(self, pad, x_raw, y_raw, k, cx, cy):
        sx = axis_to_screen(joyread.normalize(x_raw, cx))
        sy = axis_to_screen(joyread.normalize(y_raw, cy))
        c = pad['canvas']
        c.coords(pad['dot'], sx - DOT_R, sy - DOT_R, sx + DOT_R, sy + DOT_R)
        if pad['highlight'] == 'h':
            c.coords(pad['hi'], 0, sy, PAD, sy)
        else:
            c.coords(pad['hi'], sx, 0, sx, PAD)
        c.itemconfig(pad['k'], fill=PRESS_COLOR if k == 0 else "")
        pad['label'].config(text=f"X:{x_raw:4d} Y:{y_raw:4d}")

    def _tick(self):
        if not self.running:
            return
        node = self.node
        snap = node.tick()
        data = snap['data']
        cx1, cy1, cx2, cy2 = snap['center']

        if data is not None:
            x1, y1, k1, x2, y2, k2, swa, swb = data
            self._update_pad(self.pad_l, x1, y1, k1, cx1, cy1)
            self._update_pad(self.pad_r, x2, y2, k2, cx2, cy2)
            for name, value in (("SWA", swa), ("SWB", swb)):
                c, oid = self.sw_ids[name]
                c.itemconfig(oid, fill=PRESS_COLOR if value == 0 else IDLE_COLOR)

        # ── 상태 한 줄 ──
        if node.ser is None:
            self.status.config(text="🔌 조이스틱을 찾는 중... (연결·전원·펌웨어 확인)",
                               fg=WARN_COLOR)
        elif not snap['calibrated']:
            self.status.config(
                text=f"⏳ 영점 잡는 중 {snap['calib_n']}/{joyread.CALIBRATION_SAMPLES} "
                     f"— ★스틱을 건드리지 마십시오★  ({node.port})", fg=WARN_COLOR)
        elif not snap['fresh']:
            self.status.config(text="⚠️ 입력이 끊겼습니다 (보드는 열려 있는데 줄이 안 옵니다)",
                               fg=WARN_COLOR)
        else:
            self.status.config(
                text=f"✅ {node.port}  ·  스틱을 움직이면 아래 값과 LCD 가 함께 바뀝니다"
                     f"   |  보드 리셋 = 영점 다시 잡기  ·  q = 종료"
                     f"\n★점검 전용 — 차를 움직이는 명령은 내보내지 않습니다★",
                fg=OK_COLOR)

        pulse, steer, brake = node.cmd
        self.val_vars['pulse'].set(f"{pulse:02d}")
        self.val_vars['steer'].set(f"{steer:+03d}")
        self.val_vars['brake'].set(str(brake))
        self.onoff.config(text="ON" if node.swa_on else "OFF",
                          bg=OK_COLOR if node.swa_on else IDLE_COLOR)
        self.lcd_label.config(text=f"LCD 전송: {node.lcd_line or '(아직 없음)'}")
        self.rx_label.config(
            text=f"수신 {node.rx_lines}줄   마지막 줄: {node.last_raw or '—'}")

        self.root.after(UPDATE_MS, self._tick)

    def stop(self):
        self.running = False

    def _on_close(self):
        self.stop()
        self.node.stop_and_close()
        self.root.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = JoyTestNode()

    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    root = tk.Tk()
    gui = JoyGui(root, node)
    # 런치로 띄운 경우 고아로 남아 포트를 물고 있지 않도록 부모를 감시한다
    watch_parent(cleanup=node.stop_and_close)
    try:
        root.mainloop()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        gui.stop()
        node.stop_and_close()
        time.sleep(0.1)
        if rclpy.ok():
            rclpy.shutdown()
        spin.join(timeout=1.0)
        try:
            node.destroy_node()
        except Exception:
            pass
        print("joystick 점검 도구 종료", flush=True)


if __name__ == '__main__':
    main()
