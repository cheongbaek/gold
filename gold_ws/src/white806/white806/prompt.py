#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
prompt.py ― CLI 메인화면 [white806]
════════════════════════════════════════════════════════════════════════════════
★이 화면은 주행을 시작시키지 않는다★ 시작·종료 트리거는 전부 ★B보드 D5 모드
스위치★ 다(driving.py 상태기계). 여기서 하는 일은 두 가지뿐이다.

  · 달릴 경로를 고른다 (→ /drive_cmd 로 파일명 발행 = '선택'일 뿐 출발이 아니다)
  · 지금 무슨 상태인지 보여준다 (/drive_state · /vehicle_mode · /estop)

  ┌ 조작 요령 ────────────────────────────────────────────────────────────────┐
  │  매핑 : 스위치를 ★자율 → 수동★ 으로 내린다                                 │
  │         이미 수동이면 자율로 한 번 올렸다 다시 내린다 (엣지가 필요하다)      │
  │         끝낼 때는 자율로 올린다 → 경로 저장 + 메인화면                      │
  │                                                                            │
  │  주행 : 경로를 고른 뒤 스위치를 ★수동 → 자율★ 로 올린다                    │
  │         이미 자율이면 수동으로 한 번 내렸다 다시 올린다                     │
  │         도착하면 리니어 2단으로 선다 → 수동으로 내리면 기록 저장 + 메인화면  │
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

from white806 import paths


BANNER = "═" * 74


class PromptNode(Node):

    def __init__(self):
        super().__init__('prompt_node')
        self.declare_parameter('data_dir', '')
        self.data_dir = paths.data_dir(self.get_parameter('data_dir').value or '')

        self.pub_cmd = self.create_publisher(String, '/drive_cmd', 10)

        self.state = 'IDLE'
        self.auto_mode = None
        self.estop = False
        self.selected = ''
        self.events = []          # 최근 이벤트 몇 줄

        self.create_subscription(String, '/drive_state', self.cb_state, 10)
        self.create_subscription(String, '/drive_event', self.cb_event, 10)
        self.create_subscription(Bool, '/vehicle_mode', self.cb_mode, 10)
        self.create_subscription(Bool, '/estop', self.cb_estop, 10)

    def cb_state(self, m):
        self.state = str(m.data)

    def cb_event(self, m):
        self.events.append(str(m.data))
        del self.events[:-6]

    def cb_mode(self, m):
        self.auto_mode = bool(m.data)

    def cb_estop(self, m):
        self.estop = bool(m.data)

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
                f" white806   상태: {self.state}   모드: {self.mode_str()}   {est}\n"
                f" 선택된 경로: {self.selected or '(없음)'}\n"
                f"{BANNER}")

    def main_menu(self, routes):
        lines = [self.header(), " 경로 목록 (최신순)"]
        if not routes:
            lines.append("   (없음 — 스위치를 자율→수동으로 내려 매핑부터)")
        for i, name in enumerate(routes[:12], 1):
            mark = "★" if name == self.selected else " "
            lines.append(f"  {mark}{i:2d}) {name}")
        lines += [
            "",
            " 번호 = 경로 선택 |  r = 목록 새로고침 |  s = 정지(리니어 2단) |  q = 종료",
            " ▶ 주행: 경로 고르고 스위치 ↑(수동→자율)   ▶ 매핑: 스위치 ↓(자율→수동)",
            "",
        ]
        return "\n".join(lines)

    def busy_screen(self):
        lines = [self.header()]
        if self.state in ('MAP_HEADING', 'DRIVE_HEADING'):
            lines.append(" 🧭 헤딩 초기화 중 — 조향 0°로 곧게 굴러간다. 확정되면 자동 정지.")
        elif self.state == 'MAP_RUN':
            lines.append(" 🗺️ 매핑 중 — 페달로 운전하세요. 끝내려면 스위치 ↑(수동→자율).")
        elif self.state == 'DRIVE_RUN':
            lines.append(" 🚗 자율주행 중 — 중단하려면 스위치 ↓(자율→수동).")
        elif self.state == 'DRIVE_DONE':
            lines.append(" 🎯 도착 — 리니어 2단 체결됨. 스위치 ↓(자율→수동)로 해제+저장.")
        elif self.state == 'ESTOP':
            lines.append(" 🚨 E-STOP — 해제하면 처음부터 다시 시작합니다.")
        lines.append("")
        lines += [f"   · {e}" for e in self.events[-5:]]
        lines.append("")
        return "\n".join(lines)


def main(args=None):
    rclpy.init(args=args)
    node = PromptNode()

    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    last_screen = None
    try:
        while rclpy.ok():
            routes = node.routes()
            idle = node.state == 'IDLE'
            screen = node.main_menu(routes) if idle else node.busy_screen()

            # 상태가 바뀌었을 때만 다시 그린다(터미널이 깜빡이지 않게)
            if screen != last_screen:
                print(screen, flush=True)
                last_screen = screen
                if idle:
                    print("> ", end="", flush=True)

            # 입력을 0.4초씩 기다린다 — 블로킹 input() 을 쓰면 스위치로 상태가
            # 바뀌어도 화면이 갱신되지 않는다.
            ready, _, _ = select.select([sys.stdin], [], [], 0.4)
            if not ready:
                continue
            line = sys.stdin.readline().strip()
            if not line:
                if idle:
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
            if line.isdigit() and idle:
                i = int(line)
                if 1 <= i <= min(len(routes), 12):
                    node.selected = routes[i - 1]
                    node.pub_cmd.publish(String(data=node.selected))
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
