#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
signal_sim.py ― 본선 코스 신호 타이머의 ★폐루프 모의★  [2026-09-30]
════════════════════════════════════════════════════════════════════════════════
driving(DrivingNode) 와 traffic_timer(TrafficTimerNode) 를 ★한 프로세스·가짜 시계★ 로
돌리고, 가짜 카메라가 ★진짜 신호 위상★ 으로 T1 신호등을 RED/GREEN 으로 보여 준다.
차량 모델·센서·액추에이터는 follow_sim.Sim 을 그대로 쓴다(그 파일 헤더).

  · 경로 : gps_data/maincourse.csv 의 사본 — T1~T5·S 는 그대로, ★L 만 지운다★
           (L 은 mppi 가 있어야 해서 — follow_sim 과 같은 이유). 사본은 follow_sim 과
           ★다른 임시 폴더★ 에 둔다(그쪽은 terrain 을 전부 지운 같은 이름의 사본을 쓴다)
  · 신호 : 진짜 0초 = 출발 + offset. T1 이 [시작 − 35 m, 끝 + 2 m] 안일 때만 보인다
           (9/13 기록: T1 신호등이 처음 잡힌 것이 시작 35 m 앞). 15 Hz, 15% 는 UNKNOWN.
  · T5 는 카메라 몫이라 여기서는 허락(/tl_permit)이 T5 에서만 서는지만 본다.

확인하는 것 (offset 마다 한 줄 + 실패하면 이유):
  ① T1 — 전환 전에 들어서면 진입 즉시 서고, 적색→녹색 첫 프레임이 0초, 그 뒤 출발.
         접근 중(40 m 안)에 전환을 봤고 들어설 때 T1 녹색이면 서지 않는다
  ② 0초 오차 — 타이머 0초 − 진짜 녹색 시작 (카메라 한 프레임 1/15 s 안이어야 한다)
  ③ T2 — 아무것도 안 한다
  ④ T3·T4 — 들어서는 순간의 ★진짜 위상★ 이 창 안이면 통과, 밖이면 서고, 출발은 창 안
  ⑤ 정지 위치 — 진입 즉시 2단으로 선 자리가 구간 끝(≈정지선)보다 앞인가
  ⑥ 끝까지 갔는가 (도착)

실행 (ROS 환경만 source 하면 된다 — 빌드 불필요):
    source /opt/ros/humble/setup.bash
    python3 sim/signal_sim.py                     # offset 0~95 초, 5 초 간격
    python3 sim/signal_sim.py --offsets 3,48 --pulse 7 -v
"""
import argparse
import csv
import math
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

ROUTE = tt.MAINCOURSE_FILE
ROUTE_DIR = os.path.join(tempfile.gettempdir(), 'white1_signal_sim_routes')
VISIBLE_PRE_M = 35.0       # T1 신호등이 보이기 시작하는 거리 (9/13 기록)
CAM_HZ = 15.0
CAM_DROP = 0.15            # UNKNOWN 비율 (등기구가 ROI 를 벗어나는 프레임)
ZERO_ERR_MAX_S = 0.25      # 타이머 0초가 진짜 녹색 시작보다 늦어도 되는 한도


def prepare_route(name):
    """L 만 '0' 으로 지운 사본 — T1~T5·S 는 그대로 둔다."""
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
    """발행을 가로채 콜백으로 바로 넘긴다 (ROS 통신 없이 두 노드를 잇는다)."""

    def __init__(self, *sinks):
        self.sinks = sinks
        self.last = None

    def publish(self, msg):
        self.last = msg
        for s in self.sinks:
            s(msg)


class SignalSim(fs.Sim):
    def __init__(self, offset, drive_pulse=7, seed=1, t_max=600.0, estop=None):
        super().__init__(ROUTE, drive_pulse=drive_pulse, plant={'seed': seed}, t_max=t_max)
        n = self.node
        #  E-STOP 주입 (라벨, 지연, 길이) — 그 라벨의 '신호 대기' 이벤트 뒤 지연 초에 누르고
        #  길이 초 뒤 뗀다. 대기를 지운 뒤에도 신호를 다시 판단하는지 보는 용도다.
        self.estop = estop
        self._estop_on = self._estop_off = None
        self.T0 = CLOCK.t + offset           # ★진짜★ 0초 (신호등 쪽 시계)
        self.cam_rng = np.random.default_rng(seed + 100)
        timer = tt.TrafficTimerNode()
        self.timer = timer
        timer.pub = _Fwd(n.cb_traffic_timer)
        n.pub_dstate = _Fwd(timer.cb_drive_state)
        n.pub_tl_zone = _Fwd(timer.cb_zone)
        n.pub_tl_permit = _Fwd()
        timer.set_route(ROUTE)
        self.segs = {lab: (i0, i1) for lab, i0, i1 in n._tl_segs}
        self.s = n.wp_s
        self.rows = []

    def true_phase(self, t=None):
        return ((CLOCK.t if t is None else t) - self.T0) % tt.CYCLE_S

    def camera(self):
        """T1 이 보이는 동안만 진짜 위상으로 RED/GREEN, 그 밖은 UNKNOWN."""
        n = self.node
        i0, i1 = self.segs['T1']
        here = self.s[min(n.wp_idx, len(self.s) - 1)]
        st = 'UNKNOWN'
        if self.s[i0] - VISIBLE_PRE_M <= here <= self.s[i1] + 2.0 \
                and self.cam_rng.random() >= CAM_DROP:
            st = 'GREEN' if tt.in_window('T1', self.true_phase()) else 'RED'
        self.timer.cb_tl_state(String(data=st))

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
                #  ★허락·구간은 loop() 맨 앞(publish_state_topics)에서 ★옮기기 전★ 포인터로
                #  나간다★ — 같은 포인터로 대조해야 경계 한 틱이 어긋나 보이지 않는다.
                wp_pub = n.wp_idx
                n.loop()
                self.read_outputs()
                permit = n.pub_tl_permit.last
                zone = n.pub_tl_zone.last
                self.rows.append((CLOCK.t, n.wp_idx, self.v, self.brake_level,
                                  zone.data if zone else '',
                                  bool(permit.data) if permit else False, wp_pub))
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

    def estop_step(self):
        if not self.estop:
            return
        lab, delay, dur = self.estop
        if self._estop_on is None:
            hit = self.ev(f'🚦 {lab} 신호 대기')
            if hit:
                self._estop_on = hit[0][0] + delay
                self._estop_off = self._estop_on + dur
        elif self._estop_on <= CLOCK.t < self._estop_off and not self.node.estop:
            self.node.cb_estop(Bool(data=True))
            self.v = 0.0                         # 하드웨어(B보드 2단)가 세운다 — 모의는 즉시
        elif CLOCK.t >= self._estop_off and self.node.estop:
            self.node.cb_estop(Bool(data=False))

    # ── 판정 ─────────────────────────────────────────────────────────────
    def first_tick_in(self, lab):
        i0, i1 = self.segs[lab]
        for r in self.rows:
            if i0 <= r[1] <= i1:
                return r
        return None

    def first_tick_past(self, lab):
        _, i1 = self.segs[lab]
        for r in self.rows:
            if r[1] > i1:
                return r
        return None

    def ev(self, needle):
        """needle 이 든 이벤트 [(★절대★ 시각, 문구)]. self.events 는 출발 기준 상대 시각이다."""
        return [(self.t0 + t, e) for t, e in self.events if needle in e]

    def judge(self):
        """(요약 한 줄, 실패 목록, 경고 목록)

        ★판단 사건마다★ 진짜 위상을 대조한다 — '통과'·'출발(🟢)' 은 창 안이어야 하고,
        '신호 대기' 는 창 밖이어야 한다(경계는 0초 오차만큼 봐준다). E-STOP 으로 대기가
        지워져 같은 신호를 두 번 판단해도 그대로 맞는 잣대다.
        """
        fails, warns, parts = [], [], []
        # ② 0초 오차
        t0 = self.timer.t0
        if t0 is None:
            fails.append('T1: 타이머가 초기화되지 않았다')
        else:
            k = round((t0 - self.T0) / tt.CYCLE_S)
            err = t0 - (self.T0 + k * tt.CYCLE_S)
            parts.append(f"0초오차 {err * 1000:+4.0f}ms")
            #  ★늦게만 잡힌다★ 첫 녹색 '프레임' 시각이라 카메라 주기만큼, 그 프레임이
            #  빠지면(UNKNOWN) 몇 프레임 더 늦는다. 0.25 s 면 3~4 프레임 누락까지 받는다.
            if not (0.0 <= err <= ZERO_ERR_MAX_S):
                fails.append(f'T1: 0초 오차 {err:.3f}s')
        # ①④ T1·T3·T4 — 사건마다 진짜 위상 대조
        for lab in tt.TIMER_SIGNALS:
            ent = self.first_tick_in(lab)
            if ent is None:
                fails.append(f'{lab}: 구간에 못 들어갔다')
                continue
            txt = f"{lab} 진입 {self.true_phase(ent[0]):5.1f}s"
            go_ev = self.ev(f'🚦 {lab} 통과') + self.ev(f'🟢 {lab} 녹색')
            stop_ev = self.ev(f'🚦 {lab} 신호 대기')
            if not go_ev:
                fails.append(f'{lab}: 통과도 출발도 없다')
            for t, e in sorted(go_ev):
                ph = self.true_phase(t)
                txt += f" {'통과' if '통과' in e else '출발'} {ph:4.1f}s"
                if not tt.in_window(lab, ph):
                    fails.append(f'{lab}: 진짜 적색({ph:.1f}s)에 {"통과" if "통과" in e else "출발"}')
            #  ★T1 은 녹색이어도 선다★ (전환을 보기 전이면 — 사용자 지시) 그래서 T1 의
            #  '신호 대기' 는 위상과 무관하게 맞다. 틀린 경우는 '적색에 통과·출발' 뿐이다.
            for t, e in (stop_ev if lab != tt.INIT_SIGNAL else []):
                ph = self.true_phase(t)
                a, b = tt.SIGNAL_WINDOWS[lab]
                edge = min(abs(ph - a), abs(ph - b), abs(ph - a - tt.CYCLE_S))
                if tt.in_window(lab, ph) and edge > ZERO_ERR_MAX_S:
                    fails.append(f'{lab}: 진짜 녹색({ph:.1f}s)인데 섰다')
            if stop_ev:
                txt += f" (정지 {len(stop_ev)}회"
                # ⑤ 정지 위치 — 처음 선 자리의 호길이 vs 구간 끝(≈정지선)
                i0, i1 = self.segs[lab]
                for r in self.rows:
                    if r[0] > stop_ev[0][0] and r[2] < 0.05:
                        left = self.s[i1] - self.s[min(r[1], len(self.s) - 1)]
                        txt += f", 끝 {left:4.1f}m 앞"
                        if left < 0:
                            fails.append(f'{lab}: 구간 끝을 {-left:.1f}m 넘어 섰다')
                        break
                txt += ")"
            past = self.first_tick_past(lab)
            if past is not None and not tt.in_window(lab, self.true_phase(past[0])):
                ph_x = self.true_phase(past[0])
                txt += f" [끝 통과 {ph_x:4.1f}s ★창 밖★]"
                warns.append(f'{lab}: 구간 끝(≈정지선)을 창이 닫힌 뒤({ph_x:.1f}s)에 지났다')
            parts.append(txt)
        # ③ T2 — 아무것도 안 한다
        if self.ev('🚦 T2'):
            fails.append('T2: 무시해야 하는데 판단했다')
        # T5 — 카메라 허락은 T5 에서만
        i0, i1 = self.segs['T5']
        bad = [r for r in self.rows if r[5] and not (i0 <= r[6] <= i1)]
        if bad:
            fails.append(f'T5: 구간 밖에서 카메라 허락 {len(bad)}틱')
        if not any(r[5] for r in self.rows):
            fails.append('T5: 카메라 허락이 한 번도 안 섰다')
        # ⑥ 도착
        arrived = bool(self.ev('🎯 도착'))
        if not arrived:
            last = self.events[-1][1] if self.events else '(이벤트 없음)'
            fails.append(f'끝까지 못 갔다 — 상태 {self.done_state}: {last}')
        parts.append('도착' if arrived else self.done_state)
        return ' | '.join(parts), fails, warns


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--offsets', default=','.join(str(o) for o in range(0, 100, 5)),
                    help='진짜 0초 = 출발 + offset [s] (쉼표로 여럿)')
    ap.add_argument('--pulse', type=int, default=7)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('-v', '--verbose', action='store_true', help='이벤트를 다 찍는다')
    ap.add_argument('--estop', default='',
                    help="LABEL:지연:길이 — 그 신호 대기 뒤 E-STOP 을 누른다 (예 T3:1:3)")
    a = ap.parse_args()
    n_fail = n_warn = 0
    for off in [float(x) for x in a.offsets.split(',') if x.strip()]:
        es = None
        if a.estop:
            lab, d, du = a.estop.split(':')
            es = (lab, float(d), float(du))
        sim = SignalSim(off, drive_pulse=a.pulse, seed=a.seed, estop=es).run()
        line, fails, warns = sim.judge()
        mark = 'OK ' if not fails else 'NG '
        print(f"{mark} offset {off:5.1f}s  {line}", flush=True)
        for f in fails:
            print(f"      ✗ {f}")
        for w in warns:
            print(f"      ⚠ {w}")
        n_warn += bool(warns)
        if a.verbose or fails:
            for t, e in sim.events:
                if '일시정지 중 — 해제하면' in e:
                    continue              # 모의에서는 스로틀이 없어 20 Hz 로 쏟아진다
                if any(c in e for c in ('🚦', '🟢', '⏸️', '▶', '✅', '🛑', '⛔', '🎯 도착',
                                        '🚨', 'E-STOP')):
                    print(f"      {t:7.2f}s  {e}")
        n_fail += bool(fails)
    print(f"\n{'전부 통과' if not n_fail else f'{n_fail}건 실패'}"
          + (f" · 경고 {n_warn}건" if n_warn else ""))
    return 1 if n_fail else 0


if __name__ == '__main__':
    sys.exit(main())
