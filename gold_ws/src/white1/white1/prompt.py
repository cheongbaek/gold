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

════════════════════════════════════════════════════════════════════════════════
 ★★ [2026-08-21] 시작 대기가 ★2단 게이트★ 가 되었다 — ①스위치 → ②E-STOP ★★
════════════════════════════════════════════════════════════════════════════════
  1)매핑·2)주행 을 고르면 아래 두 조건을 ★이 순서로★ 확인하고, 하나라도 어긋나면
  그 자리에서 기다린다. 둘 다 맞는 순간 START 명령이 나간다.

      ① 스위치   매핑=수동조종 / 주행=자율주행     안 맞으면 → mapping.mp3 / driving.mp3
      ② E-STOP   해제되어 있을 것                  물려 있으면 → estop_x.mp3

  ★순서가 곧 안내 우선순위다★ 스위치가 틀린 채로 E-STOP 까지 물려 있으면 먼저
  스위치 안내만 나가고, 사람이 스위치를 넘기면 그때 E-STOP 안내가 나간다. 둘을
  동시에 말하면 무엇부터 해야 하는지가 흐려진다. 대기 중에 스위치가 다시 어긋나면
  게이트도 ①로 되돌아가고 안내도 다시 나간다.

  ★E-STOP 은 이제 매핑도 막는다★ 종전에는 매핑만 E-STOP 중에도 시작할 수 있었다
  (차가 어차피 안 구르니까). 두 절차의 대기 규칙을 같게 두는 편이 실차에서 덜
  헷갈려서 통일했다. 진짜 게이트는 driving.py 의 cb_drive_cmd 다 — 이 화면은
  '그 게이트에 걸릴 명령을 애초에 보내지 않는' 역할이다.

  ★대기 중에 해제되면 해제음(estop_re)은 나오지 않는다★ 그 순간 곧바로 시작
  안내(prompt_2/prompt_4)가 나가기 때문이다 — 두 안내가 겹치면 무엇이 시작됐는지
  들리지 않는다. 이 억제는 이 화면이 발행하는 ★/prompt_wait★ 를 sound.py 가 보고
  한다(그쪽 cb_estop 참고). 이 화면을 안 띄우면 토픽이 없으므로 종전대로 해제음이
  나온다 — 그때는 겹칠 시작 안내가 없으니 그게 맞다.

  ┌ 조작 요령 ────────────────────────────────────────────────────────────────┐
  │  1) 매핑 : 스위치가 수동조종이면 바로 시작, 자율주행이면 내릴 때까지 대기.   │
  │            E-STOP 이 물려 있으면 해제할 때까지 한 번 더 대기.               │
  │            시작되면 그 순간부터(헤딩이 잡히기 전부터) CSV 에 기록된다 —      │
  │            기록되는 좌표가 이 화면에 그대로 표시된다.                       │
  │            헤딩은 ★사람이 페달+핸들을 일자로★ 잡는다(driving 은 관찰만).    │
  │            아무 키나 누르면 중단(저장) + 메뉴로. 스위치를 올려도 종료.      │
  │                                                                            │
  │  2) 주행 : 경로 파일을 고른 뒤, 스위치가 자율주행이면 바로 시작, 수동조종   │
  │            이면 올릴 때까지 대기. E-STOP 이 물려 있으면 해제까지 대기.       │
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
import rclpy.executors
from rclpy.node import Node

from std_msgs.msg import Bool, String

from white1 import paths

# ★음성 안내 [2026-08-12 → 2026-08-21]★ 이 화면이 직접 내는 것은 ★네 개★ 다 —
#   시작 인사와 대기 안내 셋(스위치 둘 + E-STOP 하나). 전부 토픽에 나타나지 않는
#   이 화면의 로컬 상태(무엇을 누르고 무엇을 기다리는 중인가)라서다. 나머지(매핑
#   시작·종료, 주행 시작·도착, E-STOP 발동·해제)는 전부 nxde 의 sound 노드가
#   토픽을 보고 낸다 — 그래야 이 화면을 안 띄워도 같은 안내가 나온다.
#   ★nxde 가 없어도 이 화면은 그대로 돈다★ (exec_depend 이지만 방어한다)
try:
    from nxde.sound import (Player, SND_PROMPT, SND_WAIT_MAP, SND_WAIT_DRIVE,
                            SND_ESTOP_HOLD)
except Exception:                       # noqa: BLE001 — 음성이 없다고 CLI 를 막지 않는다
    Player = None
    SND_PROMPT = SND_WAIT_MAP = SND_WAIT_DRIVE = SND_ESTOP_HOLD = ''


BANNER = "═" * 74

# ── 이 화면의 로컬 UI 모드 ── driving_node 의 self.state 와는 별개다. IDLE 인
#   동안에도 '경로 고르는 중'·'스위치 전환 대기 중' 처럼 이 화면만의 하위 흐름이
#   있어야 하기 때문이다.
UI_MENU        = 'MENU'
UI_PICK_ROUTE  = 'PICK_ROUTE'
# ★[2026-08-21] 대기 화면은 하나로 합쳤다★ 종전에는 WAIT_MAP·WAIT_DRIVE 둘이었는데,
#   '무엇을 기다리는가'가 이제 두 축(무엇을 시작하려는가 × 어느 게이트에 걸렸나)이라
#   화면 상수로 표현하면 네 개가 된다. pending 과 gate 두 값으로 그리는 편이 낫다.
UI_WAIT        = 'WAIT'

# 시작 게이트 — ★이 순서로★ 확인한다(파일 헤더 참고).
GATE_SWITCH = 'SWITCH'    # 스위치가 목표 위치가 아니다
GATE_ESTOP  = 'ESTOP'     # E-STOP 이 물려 있다


class PromptNode(Node):

    def __init__(self):
        super().__init__('prompt_node')
        self.declare_parameter('data_dir', '')
        self.data_dir = paths.data_dir(self.get_parameter('data_dir').value or '')
        # ★[2026-08-14] 음원 폴더를 직접 넘긴다★ 음원이 nxde/sound → white1/sound 로
        #   옮겨 왔는데, Player 를 빈 인자로 만들면 nxde 자기 패키지 폴더(이제 없다)를
        #   본다. 이 화면이 내는 세 안내(시작 인사·스위치 대기 둘)가 조용히 사라진다.
        self.declare_parameter('sound_dir', '')
        sound_path = paths.sound_dir(self.get_parameter('sound_dir').value or '')

        self.pub_cmd = self.create_publisher(String, '/drive_cmd', 10)
        # ★[2026-08-21] 이 화면이 지금 무엇을 기다리는가★ '' = 대기 아님.
        #   sound.py 가 이것을 보고 ★대기 중 E-STOP 해제음을 억제★ 한다 —
        #   그 순간 곧바로 시작 안내가 나가므로 겹치면 안 되기 때문이다.
        #   ★매 틱 발행한다★ 값이 안 바뀌어도 보낸다 — 받는 쪽이 신선도로
        #   '이 화면이 살아 있는가'를 판정할 수 있어야 한다(화면이 죽으면 억제도
        #   함께 풀려야 맞다. sound.py PROMPT_WAIT_STALE_S).
        self.pub_wait = self.create_publisher(String, '/prompt_wait', 10)
        self._gate_said = ''        # 같은 안내를 두 번 내지 않기 위한 마지막 게이트

        self.sound = Player(sound_path, log=self.get_logger().warning) if Player else None
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

    # ── 시작 게이트 ────────────────────────────────────────────────────────────
    def gate_for(self, pending):
        """지금 무엇에 막혀 있는가. None = 막힌 것 없음(= 바로 시작해도 된다).

        ★순서가 규약이다★ 스위치를 먼저 보고, 그 다음에 E-STOP 을 본다. 안내
        우선순위(mapping/driving → estop_x)가 이 순서에서 그대로 따라 나온다.
        """
        if not pending:
            return None
        want_auto = (pending == 'DRIVE')
        # 미수신(None)도 '아직 맞지 않았다'로 본다 — 모르는 채로 출발시키지 않는다.
        if self.auto_mode is not want_auto:
            return GATE_SWITCH
        if self.estop:
            return GATE_ESTOP
        return None

    def announce_gate(self, pending, gate):
        """게이트가 ★바뀔 때만★ 한 번 안내한다.

        같은 게이트에 계속 걸려 있는 동안 반복하지 않는 것은 종전 스위치 안내와
        같은 규칙이다. 게이트가 ①→② 로 넘어가거나, 대기 중에 스위치가 다시
        어긋나 ②→① 로 되돌아가면 그때 새로 나간다.
        """
        key = f"{pending}:{gate}" if (pending and gate) else ''
        if key == self._gate_said:
            return
        self._gate_said = key
        if gate == GATE_SWITCH:
            self.play(SND_WAIT_MAP if pending == 'MAP' else SND_WAIT_DRIVE)
        elif gate == GATE_ESTOP:
            self.play(SND_ESTOP_HOLD)

    def publish_wait(self, pending, gate):
        """대기 상태를 sound.py 에 알린다. 'MAP_ESTOP' 처럼 두 축을 붙여 보낸다."""
        self.pub_wait.publish(String(
            data=f"{pending}_{gate}" if (pending and gate) else ''))

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
        # ★[2026-08-21] E-STOP 은 매핑·주행 ★둘 다★ 막는다 — 다만 '거절'이 아니라
        #   '대기'다. 눌러 두면 해제되는 순간 알아서 출발한다.
        if self.estop:
            lines.append(" 🚨 E-STOP 체결 중 — 지금 눌러 두면 해제되는 순간 시작한다"
                         " (거절되지 않는다)")
        lines += [
            " 1) 매핑 시작   2) 주행 시작   |  r = 새로고침  |  s = 정지(리니어 2단)  |  q = 종료",
            " ▶ 매핑: ①스위치 수동조종 → ②E-STOP 해제  (둘 다 되면 즉시 시작)",
            " ▶ 주행: 경로 선택 후 ①스위치 자율주행 → ②E-STOP 해제",
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

    def wait_screen(self, pending, gate):
        """2단 게이트 대기 화면. ★지금 걸린 한 가지만 크게 말한다★

        둘 다 어긋나 있어도 ①만 띄우고, 그것이 해결되면 화면이 ②로 바뀐다 —
        음성 우선순위와 화면이 같은 것을 말해야 사람이 헷갈리지 않는다.
        남은 단계는 아래 줄에 작게 보여 준다(무엇이 더 남았는지는 알아야 한다).
        """
        label = "매핑" if pending == 'MAP' else "주행"
        need = "수동조종" if pending == 'MAP' else "자율주행"
        if self.auto_mode is None:
            cur = "미수신"
        else:
            cur = "자율주행" if self.auto_mode else "수동조종"

        if gate == GATE_SWITCH:
            head = f" ⏳ {label} 대기 ①/② — 스위치를 ★{need}★ 로 전환하세요 (현재: {cur})"
            rest = ("   다음 단계: ②E-STOP 해제 (지금 체결 중)" if self.estop
                    else "   다음 단계: ②E-STOP 확인 — 지금은 해제 상태다")
        else:
            head = f" 🚨 {label} 대기 ②/② — ★E-STOP 을 해제하세요★ (스위치: {cur} ✔)"
            rest = ("   해제하는 즉시 매핑이 시작됩니다 — 페달로 몰 준비를 하고 해제할 것"
                    if pending == 'MAP' else
                    "   해제하는 즉시 ★차가 출발합니다★ — 차 주변을 먼저 확인할 것")
        lines = [self.header(), head, rest,
                 " 아무 키나 누르면 취소하고 메뉴로 돌아갑니다.",
                 ""]
        return "\n".join(lines)

    def busy_screen(self):
        lines = [self.header()]
        # ★[2026-08-21] E-STOP 은 주행을 취소하지 않고 붙잡는다★ 상태는 DRIVE_* 그대로라
        #   '달리는 중' 문구를 그냥 두면 화면이 사실과 반대가 된다 — 여기서 가로챈다.
        if self.estop and self.state.startswith('DRIVE_'):
            lines += [" ⏸️ E-STOP 일시정지 중 — 상태·경로·헤딩을 그대로 들고 서 있습니다.",
                      "    해제하면 그 자리에서 이어서 재개합니다"
                      " — ★차 주변을 먼저 확인하고 해제할 것★",
                      " (아무 키나 누르면 중단하고 메뉴로 돌아갑니다)",
                      ""]
            lines += [f"   · {e}" for e in self.events[-5:]]
            lines.append("")
            return "\n".join(lines)
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
        # ★'ESTOP' 상태는 없다★ E-STOP 은 상태가 아니라 플래그다. [2026-08-21] 그리고
        #   이제 주행을 취소하지도 않는다 — 일시정지다(driving.cb_estop). 체결 사실은
        #   header() 가, 그 의미는 바로 위 ⏸️ 줄이 띄운다.
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
    pending = None          # None | 'MAP' | 'DRIVE' — 두 게이트를 다 통과할 때까지 대기 중
    last_screen = None

    try:
        while rclpy.ok():
            idle = node.state == 'IDLE'

            # ★상태가 IDLE 을 벗어나면 이 화면의 대기/선택 흐름은 의미가 없어진다★
            #   (방금 보낸 MAP_START/DRIVE_START 가 먹혔거나, 다른 경로로 상태가
            #   바뀐 경우 모두 포함) — 다음에 IDLE 로 돌아오면 깨끗한 메뉴에서
            #   다시 시작한다.
            if not idle:
                ui = UI_MENU
                pending = None

            # ★★ [2026-08-21] 2단 게이트 : ①스위치 → ②E-STOP ★★
            #   ★E-STOP 으로 대기를 취소하지 않는다★ 종전에는 주행 대기 중에
            #   E-STOP 이 걸리면 대기를 접고 메뉴로 돌려보냈는데(그때는 driving 이
            #   거절했으니 맞는 처리였다), 이제 E-STOP 은 '거절'이 아니라 '한 단계
            #   더 기다림'이다 — 눌러 둔 사람의 의도를 버리지 않는다.
            gate = node.gate_for(pending)
            if pending and gate is None:
                # 두 게이트를 다 통과했다 — 그 즉시 시작 명령을 보낸다.
                node.pub_cmd.publish(String(
                    data='MAP_START' if pending == 'MAP' else 'DRIVE_START'))
                pending = None
                ui = UI_MENU
                last_screen = None
            # 안내음성(변화 시 한 번)과 대기상태 발행(매 틱)은 게이트 하나로 통일한다.
            node.announce_gate(pending, gate)
            node.publish_wait(pending, gate)

            routes = node.routes()
            if ui == UI_WAIT and pending:
                screen = node.wait_screen(pending, gate)
            elif ui == UI_PICK_ROUTE:
                screen = node.pick_route_screen(routes)
            else:   # UI_MENU (대기가 끝났는데 화면만 남은 경우도 여기로 떨어진다)
                ui = UI_MENU
                screen = node.menu_screen(routes) if idle else node.busy_screen()

            if screen != last_screen:
                print(screen, flush=True)
                last_screen = screen
                if ui == UI_MENU and idle:
                    print("> ", end="", flush=True)

            ready, _, _ = select.select([sys.stdin], [], [], 0.4)
            if not ready:
                continue
            line = sys.stdin.readline().strip()

            # ── 대기 화면(스위치·E-STOP 대기) : 아무 키나 취소로 처리 ──
            if ui == UI_WAIT:
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
                        else:
                            # ★게이트 판정·안내·시작을 전부 루프 앞머리에 맡긴다★
                            #   여기서 직접 START 를 보내던 분기를 없앴다 — 시작
                            #   조건이 둘이 되면서, 판정이 두 곳에 있으면 한쪽만
                            #   고치는 실수가 난다.
                            pending = 'DRIVE'
                            ui = UI_WAIT
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
                if node.auto_mode is None:
                    print("⚠️ 주행모드를 아직 알 수 없습니다 (nxde arduino 연결 확인)")
                else:
                    pending = 'MAP'          # 게이트 판정은 루프 앞머리 한 곳에서
                    ui = UI_WAIT
                last_screen = None
                continue
            if line == '2':
                # ★[2026-08-21] E-STOP 이어도 여기서 막지 않는다★ 경로를 고르고
                #   대기 화면으로 들어가면, 해제되는 순간 알아서 출발한다.
                if not routes:
                    print("⚠️ 저장된 경로가 없습니다 — 먼저 매핑하세요")
                    last_screen = None
                    continue
                ui = UI_PICK_ROUTE
                last_screen = None
                continue
            print("입력을 이해하지 못했습니다.")
            last_screen = None
    # ★[2026-09-04] ExternalShutdownException 도 받는다★ launch 가 내려갈 때
    #   rclpy 의 신호 처리기가 컨텍스트를 먼저 닫으면 spin 은 KeyboardInterrupt 가
    #   아니라 이것을 던진다. 안 받으면 노드마다 트레이스백을 십수 줄 쏟아, 정작
    #   봐야 할 종료 로그를 밀어낸다(구독/발행 노드가 종료에 실패할 일은 없으므로
    #   그 트레이스백의 정보량은 0 이다). 원인 로그: gps 의 `rcl_shutdown already
    #   called` RCLError — 그것은 아래 rclpy.ok() 가드가 막는다.
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
