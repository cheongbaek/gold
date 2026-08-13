#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
prompt.py ― CLI 메인화면 [white1]
════════════════════════════════════════════════════════════════════════════════
 [2026-08-11] 1)매핑 / 2)주행 번호 메뉴로 재편
════════════════════════════════════════════════════════════════════════════════
  ★시작은 이제 이 화면뿐이다★ [2026-08-11] driving.py 의 on_mode_edge 는 더 이상
  아무것도 시작시키지 않는다(취소·정리만 한다) — 매핑·주행을 시작하는 유일한
  경로는 이 화면이 /drive_cmd 로 보내는 'MAP_START'/'DRIVE_START' 뿐이다. 1(매핑)
  ·2(주행)를 고르면:

    · 스위치가 반대쪽이면      → 이 화면이 기다린다(자체 폴링 — driving.py 의
                                엣지 감지에 의존하지 않는다). 사람이 스위치를
                                넘기는 순간 /vehicle_mode 가 바뀌는 것을 이 화면이
                                감지하고, 그 즉시 START 명령을 보낸다.
    · 스위치가 이미 목표쪽이면 → 곧바로 START 명령을 보낸다.

  두 경우 다 이 화면이 명령을 보낸다는 점은 같다 — 차이는 '언제' 보내느냐뿐이다.
  안전 게이트(스위치 위치가 실제로 맞는가)는 driving.py 의 cb_drive_cmd 가 명령을
  받을 때마다 다시 확인한다.

  ┌ 조작 요령 ────────────────────────────────────────────────────────────────┐
  │  1) 매핑 : 스위치가 수동조종이면 바로 시작, 자율주행이면 내릴 때까지 대기.   │
  │            시작되면 그 순간부터(헤딩이 잡히기 전부터) CSV 에 기록된다 —      │
  │            기록되는 좌표가 이 화면에 그대로 표시된다.                       │
  │            헤딩은 ★사람이 페달+핸들을 일자로★ 잡는다(driving 은 관찰만).    │
  │            아무 키나 누르면 중단(저장) + 메뉴로. 스위치를 올려도 종료.      │
  │                                                                            │
  │  2) 주행 : 경로 파일을 고른 뒤, 스위치가 자율주행이면 바로 시작, 수동조종   │
  │            이면 올릴 때까지 대기.                                          │
  │            도착하면 리니어 2단 → 완전정지 확인 2초 뒤 자동으로 메뉴 복귀.   │
  │            아무 키나 누르면 중단(리니어 2단 후 같은 자동복귀). 스위치를     │
  │            내려도 즉시 중단.                                               │
  └────────────────────────────────────────────────────────────────────────────┘
"""

import os
import select
import sys
import threading
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String

from white1 import paths

# ★음성 안내 [2026-08-12]★ 이 화면이 직접 내는 것은 세 개뿐이다 — 시작 인사와
#   '스위치를 돌려 달라'는 대기 안내 둘. 셋 다 토픽에 나타나지 않는 이 화면의 로컬
#   상태라서다. 나머지(매핑 시작·종료, 주행 시작·도착, E-stop …)는 전부 nxde 의
#   sound 노드가 토픽을 보고 낸다 — 그래야 이 화면을 안 띄워도 안내가 나온다.
#   ★nxde 가 없어도 이 화면은 그대로 돈다★ (exec_depend 이지만 방어한다)
try:
    from nxde.sound import Player, SND_PROMPT, SND_WAIT_MAP, SND_WAIT_DRIVE
except Exception:                       # noqa: BLE001 — 음성이 없다고 CLI 를 막지 않는다
    Player = None
    SND_PROMPT = SND_WAIT_MAP = SND_WAIT_DRIVE = ''


BANNER = "═" * 74

# ── 이 화면의 로컬 UI 모드 ── driving_node 의 self.state 와는 별개다. IDLE 인
#   동안에도 '경로 고르는 중'·'스위치 전환 대기 중' 처럼 이 화면만의 하위 흐름이
#   있어야 하기 때문이다.
UI_MENU        = 'MENU'
UI_PICK_ROUTE  = 'PICK_ROUTE'
UI_WAIT_MAP    = 'WAIT_MAP'
UI_WAIT_DRIVE  = 'WAIT_DRIVE'


class PromptNode(Node):

    def __init__(self):
        super().__init__('prompt_node')
        self.declare_parameter('data_dir', '')
        self.data_dir = paths.data_dir(self.get_parameter('data_dir').value or '')

        self.pub_cmd = self.create_publisher(String, '/drive_cmd', 10)

        self.sound = Player(log=self.get_logger().warning) if Player else None
        self.play(SND_PROMPT)          # "메인화면입니다" — 뜨자마자

        self.state = 'IDLE'
        self.auto_mode = None
        self.estop = False
        self.selected = ''
        self.events = []          # 최근 이벤트 몇 줄
        self.last_point = None    # 매핑 중 마지막으로 기록된 좌표 한 줄

        self.create_subscription(String, '/drive_state', self.cb_state, 10)
        self.create_subscription(String, '/drive_event', self.cb_event, 10)
        self.create_subscription(Bool, '/vehicle_mode', self.cb_mode, 10)
        self.create_subscription(Bool, '/estop', self.cb_estop, 10)
        # ★mapping.py 가 CSV 에 실제로 쓴 행만 여기로 나온다★ 매핑 진행을
        #   눈으로 확인하기 위한 용도일 뿐, 이 화면은 이 값을 판단에 쓰지 않는다.
        self.create_subscription(String, '/mapping_point', self.cb_point, 10)

    def cb_state(self, m):
        new = str(m.data)
        # 새 매핑 세션이 시작되는 순간(비-매핑 → MAP_HEADING) 이전 세션의 마지막
        # 좌표가 화면에 남아 있지 않게 지운다.
        if new == 'MAP_HEADING' and not self.state.startswith('MAP_'):
            self.last_point = None
        self.state = new

    def cb_event(self, m):
        self.events.append(str(m.data))
        del self.events[:-6]

    def cb_mode(self, m):
        self.auto_mode = bool(m.data)

    def cb_estop(self, m):
        self.estop = bool(m.data)

    def cb_point(self, m):
        self.last_point = str(m.data)

    def play(self, name):
        """음성 안내. 재생기·음원이 없어도 조용히 지나간다."""
        if self.sound and name:
            self.sound.play(name)

    # ── 화면 ───────────────────────────────────────────────────────────────────
    def routes(self):
        try:
            names = [f for f in os.listdir(self.data_dir)
                     if f.startswith('route_') and f.endswith('.csv')]
        except OSError:
            return []
        return sorted(names, reverse=True)

    def mode_str(self):
        if self.auto_mode is None:
            return "❓ 모드 미수신 (nxde arduino / B보드 확인)"
        return "🤖 자율주행" if self.auto_mode else "🕹️ 수동조종"

    def header(self):
        est = "🚨 E-STOP 발동 중" if self.estop else "정상"
        return (f"\n{BANNER}\n"
                f" white1   상태: {self.state}   모드: {self.mode_str()}   {est}\n"
                f" 선택된 경로: {self.selected or '(없음)'}\n"
                f"{BANNER}")

    def menu_screen(self, routes):
        latest = routes[0] if routes else '(없음)'
        lines = [self.header(),
                 f" 저장된 경로: {len(routes)}개   (최신: {latest})",
                 ""]
        # ★E-STOP 중에는 2)주행이 막힌다 — 1)매핑은 된다 [2026-08-14]★
        #   실제 게이트는 driving 의 cb_drive_cmd 이고, 여기서는 미리 알려 준다.
        if self.estop:
            lines.append(" 🚨 E-STOP 체결 중 — 2) 주행 시작은 막혀 있다"
                         " (1) 매핑은 가능. 해제하면 바로 풀린다)")
        lines += [
            " 1) 매핑 시작   2) 주행 시작   |  r = 새로고침  |  s = 정지(리니어 2단)  |  q = 종료",
            " ▶ 매핑: 스위치 수동조종이면 즉시 / 자율주행이면 내릴 때까지 대기",
            " ▶ 주행: 경로 선택 후 스위치 자율주행이면 즉시 / 수동조종이면 올릴 때까지 대기",
            ""]
        return "\n".join(lines)

    def pick_route_screen(self, routes):
        lines = [self.header(), " 주행할 경로 (최신순)"]
        if not routes:
            lines.append("   (없음)")
        for i, name in enumerate(routes[:12], 1):
            mark = "★" if name == self.selected else " "
            lines.append(f"  {mark}{i:2d}) {name}")
        lines += ["", " 번호를 입력하세요 — 취소: q 또는 그냥 [Enter]", ""]
        return "\n".join(lines)

    def wait_screen(self, need_auto, label):
        if self.auto_mode is None:
            cur = "미수신"
        else:
            cur = "자율주행" if self.auto_mode else "수동조종"
        need = "자율주행" if need_auto else "수동조종"
        lines = [self.header(),
                 f" ⏳ {label} 대기 중 — 스위치를 ★{need}★ 로 전환하세요 (현재: {cur})",
                 " 아무 키나 누르면 취소하고 메뉴로 돌아갑니다.",
                 ""]
        return "\n".join(lines)

    def busy_screen(self):
        lines = [self.header()]
        if self.state in ('MAP_HEADING', 'DRIVE_HEADING'):
            lines.append(" 🧭 헤딩 초기화 중 — 조향 0°로 곧게 굴러간다. 확정되면 자동 진행.")
        elif self.state == 'MAP_RUN':
            lines.append(" 🗺️ 매핑 중 — 페달로 운전하세요.")
        elif self.state == 'DRIVE_RUN':
            lines.append(" 🚗 자율주행 중.")
        elif self.state == 'DRIVE_DONE':
            # 도착·경로이탈·STOP 이 모두 이 상태로 온다. E-STOP(D12) 은 별개 상태다.
            lines.append(" 🎯 주행 종료(도착·경로이탈·정지명령) — 리니어 2단. "
                         "완전정지 2초 뒤 자동으로 메뉴 복귀.")
        # ★'ESTOP' 상태는 이제 없다 [2026-08-14]★ E-STOP 은 상태가 아니라 플래그이고
        #   자율주행만 취소하고 메뉴로 돌아온다(driving.cb_estop). 체결 사실은
        #   header() 가 계속 띄운다.
        if self.state.startswith('MAP_') and self.last_point:
            lines.append(f" 📍 최근 기록: {self.last_point}")
        lines.append(" (아무 키나 누르면 중단하고 메뉴로 돌아갑니다)")
        lines.append("")
        lines += [f"   · {e}" for e in self.events[-5:]]
        lines.append("")
        return "\n".join(lines)


def main(args=None):
    rclpy.init(args=args)
    node = PromptNode()

    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    ui = UI_MENU
    pending = None          # None | 'MAP' | 'DRIVE' — 목표 스위치 위치가 될 때까지 대기 중
    last_screen = None

    try:
        while rclpy.ok():
            idle = node.state == 'IDLE'

            # ★상태가 IDLE 을 벗어나면 이 화면의 대기/선택 흐름은 의미가 없어진다★
            #   (방금 보낸 MAP_START/DRIVE_START 가 먹혔거나, E-stop 등 다른 경로로
            #   상태가 바뀐 경우 모두 포함) — 다음에 IDLE 로 돌아오면 깨끗한
            #   메뉴에서 다시 시작한다.
            if not idle:
                ui = UI_MENU
                pending = None

            # ★E-STOP 이 걸리면 주행 대기를 접는다 [2026-08-14]★ 스위치를 올려도
            #   driving 이 거절하므로, 대기 화면에 붙잡아 두면 사람만 헷갈린다.
            #   매핑 대기는 그대로 둔다 — E-STOP 중에도 매핑은 시작할 수 있다.
            if node.estop and (pending == 'DRIVE' or ui == UI_WAIT_DRIVE):
                pending = None
                ui = UI_MENU
                last_screen = None
                print("🚨 E-STOP 체결 — 주행 대기를 취소했습니다 (매핑은 가능)")

            # ★목표 스위치 위치가 이미 됐으면 대기를 끝내고 명령을 보낸다★
            if pending == 'MAP' and node.auto_mode is False:
                node.pub_cmd.publish(String(data='MAP_START'))
                pending = None
                ui = UI_MENU
            elif pending == 'DRIVE' and node.auto_mode is True:
                node.pub_cmd.publish(String(data='DRIVE_START'))
                pending = None
                ui = UI_MENU

            routes = node.routes()
            if ui == UI_MENU:
                screen = node.menu_screen(routes) if idle else node.busy_screen()
            elif ui == UI_PICK_ROUTE:
                screen = node.pick_route_screen(routes)
            elif ui == UI_WAIT_MAP:
                screen = node.wait_screen(need_auto=False, label="매핑")
            else:  # UI_WAIT_DRIVE
                screen = node.wait_screen(need_auto=True, label="주행")

            if screen != last_screen:
                print(screen, flush=True)
                last_screen = screen
                if ui == UI_MENU and idle:
                    print("> ", end="", flush=True)

            ready, _, _ = select.select([sys.stdin], [], [], 0.4)
            if not ready:
                continue
            line = sys.stdin.readline().strip()

            # ── 대기 화면(스위치 전환 대기) : 아무 키나 취소로 처리 ──
            if ui in (UI_WAIT_MAP, UI_WAIT_DRIVE):
                pending = None
                ui = UI_MENU
                last_screen = None
                continue

            # ── 메뉴 화면인데 driving 이 바쁜 상태(매핑/주행 진행 중) : 아무 키나 중단 ──
            if ui == UI_MENU and not idle:
                node.pub_cmd.publish(String(data='STOP'))
                last_screen = None
                continue

            # ── 경로 선택 화면 ──
            if ui == UI_PICK_ROUTE:
                if line.lower() == 'q' or not line:
                    ui = UI_MENU
                    last_screen = None
                    continue
                if line.isdigit():
                    i = int(line)
                    if 1 <= i <= min(len(routes), 12):
                        node.selected = routes[i - 1]
                        node.pub_cmd.publish(String(data=node.selected))
                        if node.auto_mode is None:
                            print("⚠️ 주행모드를 아직 알 수 없습니다 (nxde arduino 연결 확인)")
                            ui = UI_MENU
                        elif node.auto_mode:
                            node.pub_cmd.publish(String(data='DRIVE_START'))
                            ui = UI_MENU
                        else:
                            pending = 'DRIVE'
                            ui = UI_WAIT_DRIVE
                            node.play(SND_WAIT_DRIVE)   # "스위치를 자율주행으로"
                        last_screen = None
                        continue
                last_screen = None
                continue

            # ── 메인 메뉴 (IDLE) ──
            if not line:
                print("> ", end="", flush=True)
                continue
            if line.lower() == 'q':
                break
            if line.lower() == 'r':
                last_screen = None
                continue
            if line.lower() == 's':
                node.pub_cmd.publish(String(data='STOP'))
                print("🛑 정지 명령 전송")
                last_screen = None
                continue
            if line == '1':
                if node.auto_mode is False:
                    node.pub_cmd.publish(String(data='MAP_START'))
                    ui = UI_MENU
                elif node.auto_mode is True:
                    pending = 'MAP'
                    ui = UI_WAIT_MAP
                    node.play(SND_WAIT_MAP)             # "스위치를 수동조종으로"
                else:
                    print("⚠️ 주행모드를 아직 알 수 없습니다 (nxde arduino 연결 확인)")
                last_screen = None
                continue
            if line == '2':
                if node.estop:
                    print("🚨 E-STOP 체결 중에는 주행을 시작할 수 없습니다 — "
                          "해제 후 다시 (매핑은 가능합니다)")
                    last_screen = None
                    continue
                if not routes:
                    print("⚠️ 저장된 경로가 없습니다 — 먼저 매핑하세요")
                    last_screen = None
                    continue
                ui = UI_PICK_ROUTE
                last_screen = None
                continue
            print("입력을 이해하지 못했습니다.")
            last_screen = None
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
