#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
signal_sim.py ― 신호 구간(T)의 ★폐루프 모의★  [2026-09-30 → 2026-10-01 신호 접근]
════════════════════════════════════════════════════════════════════════════════
driving(DrivingNode) 와 traffic_timer(TrafficTimerNode) 를 ★한 프로세스·가짜 시계★ 로
돌리고, 가짜 카메라가 ★진짜 신호 위상★ 으로 신호등을 RED/GREEN 으로 보여 준다.
차량 모델·센서·액추에이터는 follow_sim.Sim 을 그대로 쓴다(그 파일 헤더).

  · 경로 : gps_data 의 사본 — T·S 는 그대로, ★L 만 지운다★ (L 은 mppi 가 있어야 해서).
           사본은 follow_sim 과 ★다른 임시 폴더★ 에 둔다(그쪽은 terrain 을 전부 지운 같은
           이름의 사본을 쓴다)
  · 신호 : 진짜 0초 = 출발 + offset.
      maincourse — T1·T2 0~32 · T5 0~57 녹색(카메라에 보인다) · T3·T4 는 타이머 몫이라 카메라는
                   UNKNOWN 만 낸다
      subcourse  — 시각표가 없다. 구간 k 마다 ★가짜★ 녹색 [0, 40) 초를 (30·k) 초씩 밀어 쓴다
    카메라는 [구간 시작 − 35 m, 구간 끝 + 2 m] 에서만 그 구간의 신호등을 본다(9/13 기록),
    15 Hz, 15% 는 UNKNOWN.

확인하는 것 (offset 마다 한 줄 + 실패하면 이유) — CLAUDE.md 4.10 '신호 접근':
  ① 진짜 적색(T1 은 '초기화 전' 도)에 정지선을 넘지 않았다 — 늦게 받은 정지 신호로 '통과'
     한 경우(⚠️ 정지 신호가 늦었다)만 경고로 남긴다
  ② 섰다면 정지선 앞 0~1.6 m 에 섰다 (끝단 1 m 에서 2단 · 정지거리 ≈0.5~0.9 m)
  ③ 정지 신호 동안 명령펄스 0 (코스트·1단 구간)
  ④ 감속 중 녹색이면 다시 가속했다 (감속 해제 이벤트 뒤 펄스가 돌아온다)
  ⑤ 진행 신호로 들어와 끝까지 진행이면 감속하지 않았다 (구간 안 최저속도 ≥ 진입속도 − 여유)
  ⑥ 0초 오차 · 카메라 제동 허락은 항상 꺼짐 · 끝까지 갔는가
  ⑦ [2026-10-07] 0초는 T1(처음부터 녹색이면 T2)에서 — T2 정지선을 0초 전에 넘으면 실패.
     T1 은 0초 없이 넘어도 되지만(처음부터 녹색) 그때는 진짜 녹색이어야 한다(①)

실행 (ROS 환경만 source 하면 된다 — 빌드 불필요):
    source /opt/ros/humble/setup.bash
    python3 sim/signal_sim.py                          # maincourse, offset 0~95 초 5 초 간격
    python3 sim/signal_sim.py --route subcourse.csv
    python3 sim/signal_sim.py --offsets 3,48 --pulse 4 -v
    python3 sim/signal_sim.py --offsets 0 --estop T3:1:3 -v   # T3 대기 중 E-STOP
    python3 sim/signal_sim.py --offsets 0 --estop '@T1 녹색으로 통과:2:62' -v
                                     # T1 녹색 통과 뒤 E-STOP 으로 늦춰 T2 감속 중 전환을 본다
"""
import argparse
import csv
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import follow_sim as fs                              # noqa: E402  (도메인 77 · 가짜 시계)
import numpy as np                                   # noqa: E402
import rclpy                                         # noqa: E402
import rclpy.node                                    # noqa: E402
from std_msgs.msg import Bool, String                # noqa: E402

import white1.traffic_timer as tt                    # noqa: E402

drv = fs.drv
CLOCK = fs.CLOCK
tt.time = CLOCK                                      # 타이머도 같은 가짜 시계

ROUTE_DIR = os.path.join(tempfile.gettempdir(), 'white1_signal_sim_routes')
VISIBLE_PRE_M = 35.0       # 신호등이 보이기 시작하는 거리 (9/13 기록)
CAM_HZ = 15.0
CAM_DROP = 0.15            # UNKNOWN 비율 (등기구가 ROI 를 벗어나는 프레임)
ZERO_ERR_MAX_S = 0.25      # 타이머 0초가 진짜 녹색 시작보다 늦어도 되는 한도
STOP_OK_M = (0.0, 1.6)     # 정지선 앞 이 거리 안에 서야 한다 [m] (안테나 기준)
SUB_GREEN = (0.0, 40.0)    # subcourse 가짜 녹색 창 [s]
SUB_SHIFT_S = 30.0         # subcourse 구간마다 창을 이만큼 민다


def prepare_route(name):
    """L 만 '0' 으로 지운 사본 — T·S 는 그대로 둔다."""
    os.makedirs(ROUTE_DIR, exist_ok=True)
    src = os.path.join(fs.GPS_DIR, name)
    dst = os.path.join(ROUTE_DIR, name)
    rows = list(csv.DictReader(open(src, encoding='utf-8-sig')))
    fields = list(rows[0].keys())
    tmp = f'{dst}.{os.getpid()}.tmp'
    with open(tmp, 'w', encoding='utf-8', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in rows:
            if r.get('terrain', '').strip().upper() == 'L':
                r['terrain'] = '0'
            w.writerow(r)
    os.replace(tmp, dst)
    lat = np.array([float(r['latitude']) for r in rows])
    lon = np.array([float(r['longitude']) for r in rows])
    return lat, lon


fs.ROUTE_TMP = ROUTE_DIR                             # Sim 이 data_dir 로 넘긴다
fs.prepare_route = prepare_route


class _Fwd:
    """발행을 가로채 콜백으로 바로 넘긴다 (ROS 통신 없이 노드끼리 잇는다)."""

    def __init__(self, *sinks):
        self.sinks = sinks
        self.last = None

    def publish(self, msg):
        self.last = msg
        for s in self.sinks:
            s(msg)


class SignalSim(fs.Sim):
    def __init__(self, route, offset, drive_pulse=7, seed=1, t_max=900.0, estop=None):
        super().__init__(route, drive_pulse=drive_pulse, plant={'seed': seed}, t_max=t_max)
        n = self.node
        self.main = tt.is_maincourse(route)
        self.T0 = CLOCK.t + offset           # ★진짜★ 0초 (신호등 쪽 시계)
        self.cam_rng = np.random.default_rng(seed + 100)
        timer = tt.TrafficTimerNode()
        self.timer = timer
        timer.pub = _Fwd(n.cb_traffic_timer)
        n.pub_dstate = _Fwd(timer.cb_drive_state)
        n.pub_tl_zone = _Fwd(timer.cb_zone)
        n.pub_tl_permit = _Fwd()
        timer.set_route(route)
        self.segs = list(n._tl_segs)         # [(라벨, i0, i1)]
        self.s = n.wp_s
        self.rows = []
        #  E-STOP 주입 (라벨, 지연, 길이) — 그 라벨의 '신호 대기' 이벤트 뒤 지연 초에 누르고
        #  길이 초 뒤 뗀다. 대기를 지운 뒤에도 신호를 다시 판단하는지 보는 용도다.
        self.estop = estop
        self._estop_on = self._estop_off = None

    # ── 진짜 신호 ─────────────────────────────────────────────────────────
    def true_phase(self, t=None):
        return ((CLOCK.t if t is None else t) - self.T0) % tt.CYCLE_S

    def true_green(self, k, t=None):
        """구간 k 의 신호가 진짜로 녹색인가. 시각표가 없는 라벨은 None."""
        lab = self.segs[k][0]
        ph = self.true_phase(t)
        if self.main:
            if lab in ('T1', 'T2'):
                return tt.in_window('T1', ph)
            if lab in ('T3', 'T4', 'T5'):
                return tt.in_window(lab, ph)
            return None
        ph = (ph + SUB_SHIFT_S * k) % tt.CYCLE_S
        return SUB_GREEN[0] <= ph < SUB_GREEN[1]

    def camera(self):
        """보이는 구간의 신호등을 진짜 위상으로 RED/GREEN, 그 밖은 UNKNOWN."""
        here = self.s[min(self.node.wp_idx, len(self.s) - 1)]
        st = 'UNKNOWN'
        for k, (lab, i0, i1) in enumerate(self.segs):
            if not (self.s[i0] - VISIBLE_PRE_M <= here <= self.s[i1] + 2.0):
                continue
            if self.main and lab in ('T3', 'T4'):
                break                          # 카메라가 못 읽는 곳 (그래서 타이머다)
            if self.cam_rng.random() >= CAM_DROP:
                st = 'GREEN' if self.true_green(k) else 'RED'
            break
        self.timer.cb_tl_state(String(data=st))
        self.node.cb_tl_state(String(data=st))

    def estop_step(self):
        if not self.estop:
            return
        lab, delay, dur = self.estop
        if self._estop_on is None:
            #  '@문구' 면 그 이벤트 뒤에 누른다 [2026-10-07] — 예 '@T1 녹색으로 통과' 로
            #  T1 을 녹색에 지난 뒤 멈춰 세워, T2 에 적색 끝 무렵 닿게 한다.
            hit = self.ev(lab[1:] if lab.startswith('@') else f'🚦 {lab} 신호 대기')
            if hit:
                self._estop_on = hit[0][0] + delay
                self._estop_off = self._estop_on + dur
        elif self._estop_on <= CLOCK.t < self._estop_off and not self.node.estop:
            self.node.cb_estop(Bool(data=True))
            self.v = 0.0                         # 하드웨어(B보드 2단)가 세운다 — 모의는 즉시
        elif CLOCK.t >= self._estop_off and self.node.estop:
            self.node.cb_estop(Bool(data=False))

    def run(self):
        n = self.node
        dt = 0.01
        k = steps = 0
        cam_next = CLOCK.t
        while CLOCK.t - self.t0 < self.t_max:
            if CLOCK.t >= cam_next:
                self.camera()
                cam_next += 1.0 / CAM_HZ
            self.estop_step()
            if steps % 5 == 0:                   # 20 Hz — driving·timer 주기
                self.timer.tick()
                self.feed_sensors(k)
                n.loop()
                self.read_outputs()
                permit = n.pub_tl_permit.last
                self.rows.append((CLOCK.t, n.arc_here(), self.v, self.brake_level,
                                  self.cmd_pulse, n._sa_phase, n._sp_state,
                                  bool(permit.data) if permit else False))
                k += 1
                if n.state != drv.S_DRIVE_RUN:
                    break
            self.step_plant(dt)
            CLOCK.t += dt
            steps += 1
        self.done_state = n.state
        rclpy.node.Node.destroy_node(self.timer)
        rclpy.node.Node.destroy_node(n)
        rclpy.shutdown()
        return self

    # ── 판정 ─────────────────────────────────────────────────────────────
    def ev(self, needle):
        """needle 이 든 이벤트 [(★절대★ 시각, 문구)]. self.events 는 출발 기준 상대 시각이다."""
        return [(self.t0 + t, e) for t, e in self.events if needle in e]

    def judge(self):
        """(요약 한 줄, 실패 목록, 경고 목록)"""
        fails, warns, parts = [], [], []
        n = self.node
        t0 = self.timer.t0
        if self.main:
            if t0 is None:
                fails.append('T1·T2: 타이머가 초기화되지 않았다')
            else:
                k = round((t0 - self.T0) / tt.CYCLE_S)
                err = t0 - (self.T0 + k * tt.CYCLE_S)
                parts.append(f"0초={self.timer.init_by} 오차 {err * 1000:+4.0f}ms")
                #  ★늦게만 잡힌다★ 첫 녹색 '프레임' 시각이라 카메라 주기만큼, 그 프레임이
                #  빠지면(UNKNOWN) 몇 프레임 더 늦는다.
                if not (0.0 <= err <= ZERO_ERR_MAX_S):
                    fails.append(f'T1: 0초 오차 {err:.3f}s')
        for k, (lab, i0, i1) in enumerate(self.segs):
            kind = n.signal_kind(lab)
            s0, s_line = self.s[i0], self.s[i1]
            inz = [r for r in self.rows if s0 <= r[1] <= s_line]
            if not inz:
                fails.append(f'{lab}: 구간에 못 들어갔다')
                continue
            if kind is None:                   # 판단하지 않는 신호 — 아무것도 안 해야 한다
                if self.ev(f'🚦 {lab} ') or any(r[5] or r[6] for r in inz):
                    fails.append(f'{lab}: 무시해야 하는데 판단했다')
                continue
            txt = f"{lab} 진입{self.true_phase(inz[0][0]):5.1f}s"
            late = self.ev(f'⚠️ {lab} 정지 신호가 늦었다')
            # ① 정지선을 넘은 순간의 진짜 신호
            #    ★정지 지점(끝 1 m)을 녹색에 지났으면 되돌릴 수 없다★ — 그 뒤 정지선을 넘는
            #    0.1~0.3 s 사이에 창이 닫히는 것은 '여유 없음' 결정의 결과라 경고로만 남긴다.
            cross = next((r for r in self.rows if r[1] >= s_line), None)
            at_pt = next((r for r in self.rows
                          if r[1] >= s_line - drv.SIG_STOP_BEFORE_M), None)
            if cross is not None:
                g = self.true_green(k, cross[0])
                ph_x = self.true_phase(cross[0])
                if lab in tt.INIT_SIGNALS[1:] and self.main:
                    #  ⑦ T1 은 처음부터 녹색이면 0초 없이 지나간다(진짜 녹색인지는 아래 ①)
                    first_init = self.ev('🚦 신호 타이머 초기화')
                    if not first_init or first_init[0][0] > cross[0]:
                        fails.append(f'{lab}: 타이머 초기화 전에 정지선을 넘었다')
                if g is False:
                    pt_green = at_pt is not None and self.true_green(k, at_pt[0])
                    why = (' — 늦은 정지 신호로 통과' if late else
                           f' — 정지 지점은 녹색({self.true_phase(at_pt[0]):.1f}s)에 지났다'
                           if pt_green else '')
                    (warns if (late or pt_green) else fails).append(
                        f'{lab}: 진짜 적색({ph_x:.1f}s)에 정지선을 넘었다' + why)
            # ② 선 자리 — 서브페이즈(2단) 안에서 처음 멈춘 곳
            stops = [r for r in inz if r[2] < 0.05 and r[6] != 0]
            if stops:
                gap = s_line - stops[0][1]
                txt += f" 정지(선 {gap:.2f}m 앞)"
                if not (STOP_OK_M[0] <= gap <= STOP_OK_M[1]):
                    (fails if gap < 0 else warns).append(
                        f'{lab}: 정지선 {gap:+.2f}m 앞에 섰다 (기준 0~{STOP_OK_M[1]}m)')
            # ③ 정지 신호 동안 펄스 0
            bad = [r for r in inz if r[5] in (drv.SA_COAST, drv.SA_BRAKE1) and r[4] != 0]
            if bad:
                fails.append(f'{lab}: 코스트·1단 중에 펄스 {bad[0][4]} 가 나갔다')
            if any(r[5] == drv.SA_BRAKE1 for r in inz):
                txt += " 1단"
            # ④ 감속 중 녹색 → 다시 가속
            rel = [e for e in self.ev(f'🟢 {lab} 진행 신호') if '감속을 풀고' in e[1]]
            if rel:
                t_rel = rel[0][0]
                later = [r for r in self.rows if t_rel < r[0] <= t_rel + 2.0]
                if not any(r[4] > 0 for r in later):
                    fails.append(f'{lab}: 감속을 풀었는데 2 s 안에 펄스가 안 돌아왔다')
                txt += " 감속해제"
            # ⑤ 진행 신호로 들어와 계속 진행이면 감속 없음
            if not self.ev(f'🚦 {lab} 정지 신호') and not stops:
                vmin = min(r[2] for r in inz)
                if inz[0][2] > 1.0 and vmin < inz[0][2] - 0.6:
                    warns.append(f'{lab}: 진행 신호인데 {inz[0][2]:.1f} → {vmin:.1f} m/s 로 줄었다'
                                 ' (곡률 감속일 수 있다)')
                txt += " 통과"
            if cross is not None:
                txt += f"→선{self.true_phase(cross[0]):5.1f}s"
            parts.append(txt)
        if any(r[7] for r in self.rows):
            fails.append('카메라 제동 허락(/tl_permit)이 켜졌다 — 항상 False 여야 한다')
        arrived = bool(self.ev('🎯 도착'))
        if not arrived:
            last = self.events[-1][1] if self.events else '(이벤트 없음)'
            fails.append(f'끝까지 못 갔다 — 상태 {self.done_state}: {last}')
        parts.append('도착' if arrived else self.done_state)
        return ' | '.join(parts), fails, warns


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--route', default=tt.MAINCOURSE_FILE)
    ap.add_argument('--offsets', default=','.join(str(o) for o in range(0, 100, 5)),
                    help='진짜 0초 = 출발 + offset [s] (쉼표로 여럿)')
    ap.add_argument('--pulse', type=int, default=7)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('-v', '--verbose', action='store_true', help='이벤트를 다 찍는다')
    ap.add_argument('--estop', default='',
                    help="LABEL:지연:길이 — 그 신호 대기 뒤 E-STOP 을 누른다 (예 T3:1:3). "
                         "LABEL 이 '@문구' 면 그 문구의 이벤트 뒤")
    a = ap.parse_args()
    n_fail = n_warn = 0
    for off in [float(x) for x in a.offsets.split(',') if x.strip()]:
        es = None
        if a.estop:
            lab, d, du = a.estop.rsplit(':', 2)
            es = (lab, float(d), float(du))
        sim = SignalSim(a.route, off, drive_pulse=a.pulse, seed=a.seed, estop=es).run()
        line, fails, warns = sim.judge()
        mark = 'OK ' if not fails else 'NG '
        print(f"{mark} offset {off:5.1f}s  {line}", flush=True)
        for f in fails:
            print(f"      ✗ {f}")
        for w in warns:
            print(f"      ⚠ {w}")
        n_fail += bool(fails)
        n_warn += bool(warns)
        if a.verbose or fails:
            for t, e in sim.events:
                if '일시정지 중 — 해제하면' in e:
                    continue              # 모의에서는 스로틀이 없어 20 Hz 로 쏟아진다
                if any(c in e for c in ('🚦', '🟢', '🔻', '⏸️', '▶', '✅', '🛑', '⛔', '⚠️',
                                        '🎯 도착', '🚨', 'E-STOP')):
                    print(f"      {t:7.2f}s  {e}")
    print(f"\n{'전부 통과' if not n_fail else f'{n_fail}건 실패'}"
          + (f" · 경고 {n_warn}건" if n_warn else ""))
    return 1 if n_fail else 0


if __name__ == '__main__':
    sys.exit(main())
