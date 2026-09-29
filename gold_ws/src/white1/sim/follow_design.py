#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
follow_design.py ― TODO-1(흔들림 감소 + 10펄스 증속) 설계 후보를 모의로 비교한다  [2026-09-29]
════════════════════════════════════════════════════════════════════════════════
★여기 있는 것은 driving.py 를 고치지 않고 '고쳤다면' 을 흉내 낸 패치다★
  노드의 메서드를 감싸기만 한다. 실제로 넣을 때의 코드 설계는 CHANGELOG.md 의
  TO DO LIST — TODO-1 절이 정본이고, 이 파일은 그 수치를 다시 내는 도구다.

  A  조향 백래시 선보상     backlash_comp(0.8, 0.3)   최종 pot 에 '움직이는 방향 × 0.8°'
  B  순수추종 기하 = 뒷차축  rear_axle_pp(1.25)        안테나 좌표를 뒤로 옮겨 겨눈다
  C  요레이트 댐핑          yaw_damp(0.15)           도로휠 += 0.15·(r − v·κ경로)
  D  코너 속도 = 횡가속 상한 corner_ay_cap(2.0)       v = √(2.0·R), 원거리 스캔 40 m
  E  LFD 상한 11.3 → 14 m   (ModuleConst)            10펄스가 실제로 나오게

사용 (ROS 환경만 source — 빌드 불필요. 코어를 다 쓴다):
    source /opt/ros/humble/setup.bash
    cd gold_ws/src/white1/sim
    python3 follow_design.py compare --cands 현행,A,A+C,A+B+C,A+B+C+D+E --pulses 7,10
    python3 follow_design.py robust  --cands 현행,A+B+C+D+E --pulses 7,10
    python3 follow_design.py logs                       # 9/13 주행 4개를 같은 잣대로
    python3 follow_design.py log <record.csv> <route.csv>   # 새 실차 로그 (라이다 구간 제외)
"""
import argparse
import concurrent.futures as cf
import math
import multiprocessing as mp
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


# ══════════════════════════════════════════════════════════════════════════════
#  설계 후보 — 노드 메서드 감싸기
# ══════════════════════════════════════════════════════════════════════════════
def backlash_comp(b_c=0.8, h=0.3):
    """A ★조향 백래시 선보상★ 최종 pot(트림 포함)에 '움직이는 방향 × b_c' 를 더한다.
    방향은 지령이 극값에서 h 이상 되돌아올 때만 뒤집는다(잡음 반전 방지)."""
    def m(n):
        orig = n._with_trim
        st = {'dir': 0, 'ext': None}

        def with_trim(pot_signed):
            u = orig(pot_signed)
            if st['ext'] is None:
                st['ext'] = u
            if st['dir'] >= 0 and u < st['ext'] - h:
                st['dir'], st['ext'] = -1, u
            elif st['dir'] <= 0 and u > st['ext'] + h:
                st['dir'], st['ext'] = 1, u
            elif st['dir'] > 0:
                st['ext'] = max(st['ext'], u)
            elif st['dir'] < 0:
                st['ext'] = min(st['ext'], u)
            return max(-40.0, min(40.0, u + b_c * st['dir']))
        n._with_trim = with_trim
    return m


def rear_axle_pp(ant_x=1.25):
    """B ★순수추종 기하만 뒷차축 기준★ 안테나 좌표를 ant_x 만큼 뒤로 옮겨 겨눈다.
    CTE·진행 포인터·종점 판정은 안테나 그대로 — 경로가 안테나 궤적이므로."""
    def m(n):
        orig = n.pure_pursuit_steer

        def pps(lfd):
            x, y = n.x, n.y
            h = math.radians(n.heading)
            n.x, n.y = x - ant_x * math.cos(h), y - ant_x * math.sin(h)
            try:
                return orig(lfd)
            finally:
                n.x, n.y = x, y
        n.pure_pursuit_steer = pps
    return m


def yaw_damp(k=0.15, lpf=0.3):
    """C ★요레이트 댐핑★ 도로휠(B보드 부호 +우) += k·LPF(r − v·κ경로) [deg / (deg/s)].
    r = 투영·바이어스 보정된 자이로, κ경로 = 진행 포인터 ±3 m 의 부호 있는 곡률."""
    def m(n):
        orig = n.pure_pursuit_steer
        st = {'e': 0.0}

        def kappa_at():
            w, s, i = n.waypoints, n.wp_s, n.wp_idx
            j0 = i
            while j0 > 0 and s[i] - s[j0] < 3.0:
                j0 -= 1
            j2 = i
            while j2 < len(w) - 1 and s[j2] - s[i] < 3.0:
                j2 += 1
            if j2 - j0 < 2 or s[j2] - s[j0] < 1.0:
                return 0.0
            h0 = math.atan2(w[i][1] - w[j0][1], w[i][0] - w[j0][0])
            h1 = math.atan2(w[j2][1] - w[i][1], w[j2][0] - w[i][0])
            dh = (h1 - h0 + math.pi) % (2 * math.pi) - math.pi
            return dh / ((s[j2] - s[j0]) / 2.0)

        def pps(lfd):
            road = orig(lfd)
            v = n.gps_ms() or 0.0
            err = math.degrees(n.gyro_z - v * kappa_at())
            st['e'] += lpf * (err - st['e'])
            return max(-n.road_max, min(n.road_max, road + k * st['e']))
        n.pure_pursuit_steer = pps
    return m


def corner_ay_cap(a_lat=2.0, floor_pulse=3):
    """D ★코너 속도 = 횡가속 상한★ v = √(a_lat·R). corner_speed 결과 위에 min 으로만 얹는다.
    코너 하한 펄스는 drive_pulse 와 떼어 floor_pulse(3)에 둔다 (10펄스에서 int(10/2)=5 가 되지 않게)."""
    import white1.driving as drv

    def m(n):
        orig = n.corner_speed
        n.corner_min_pulse = min(n.corner_min_pulse, floor_pulse)
        n.min_speed_ms = n.corner_min_pulse * drv.MS_PER_PULSE
        L = n.wheelbase

        def vcap(demand_deg):
            k = math.tan(math.radians(max(0.0, demand_deg))) / L
            return math.sqrt(a_lat / k) if k > 1e-5 else float('inf')

        def cs(near_win, near_peak, far_win, far_dist, far_peak, far_peak_dist, lfd_win_only, lfd_speed):
            pulse, gate_dist, gvc = orig(near_win, near_peak, far_win, far_dist, far_peak,
                                         far_peak_dist, lfd_win_only, lfd_speed)
            bl = drv.PEAK_SPEED_BLEND
            v_t = vcap(near_win + bl * max(0.0, near_peak - near_win))
            far_for_gate = far_win + bl * max(0.0, far_peak - far_win)
            gd = far_dist
            if far_peak_dist != float('inf') and far_peak > far_win + 1.0:
                gd = min(gd, far_peak_dist)
            if gd != float('inf') and far_for_gate > near_win + 1.0:
                vc = vcap(far_for_gate)
                v_t = min(v_t, math.sqrt(vc * vc + 2.0 * drv.BRAKE_GATE_DECEL
                                         * max(0.0, gd - drv.BRAKE_GATE_MARGIN)))
                gvc = vc if gvc is None else min(gvc, vc)
            if math.isfinite(v_t):
                pulse = min(pulse, max(floor_pulse, int(v_t / drv.MS_PER_PULSE + 0.5)))
            return pulse, gate_dist, gvc
        n.corner_speed = cs
    return m


class ModuleConst:
    """driving 모듈 상수를 잠깐 바꾼다 (LFD_MAX_M 등)."""
    def __init__(self, **kw):
        self.kw, self.old = kw, {}

    def __enter__(self):
        import white1.driving as drv
        for k, v in self.kw.items():
            self.old[k] = getattr(drv, k)
            setattr(drv, k, v)

    def __exit__(self, *a):
        import white1.driving as drv
        for k, v in self.old.items():
            setattr(drv, k, v)


def candidate(name):
    """→ (mods, params, consts)"""
    A = backlash_comp(0.8, 0.3)
    B = rear_axle_pp(1.25)
    C = yaw_damp(0.15)
    D = corner_ay_cap(2.0)
    far = {'CURVE_PREVIEW_FAR_MAX': 40.0}
    table = {
        '현행': ([], {}, {}),
        'A': ([A], {}, {}),
        'C': ([C], {}, {}),
        'A+C': ([A, C], {}, {}),
        'A+B': ([A, B], {}, {}),
        'A+B+C': ([A, B, C], {}, {}),
        'A+B+C+D': ([A, B, C, D], {}, far),
        'A+B+C+D+E': ([A, B, C, D], {}, dict(far, LFD_MAX_M=14.0)),
        # 기각된 대안 (CHANGELOG TODO-1 '검토했으나 기각')
        '전달비1.55': ([_model_gain(1.55)], {}, {}),
        'LFD실측속도': ([_lfd_by_speed()], {}, {}),
    }
    return table[name]


def _model_gain(g):
    import white1.driving as drv

    def m(n):
        n.plant_gain, n.understeer = g, 0.0
        n.road_max = drv.STEER_MAX_DEG / g
    return m


def _lfd_by_speed(alpha=0.1, floor_pulse=2.0):
    import white1.driving as drv

    def m(n):
        orig = n.lookahead_m
        st = {'v': None}

        def la(d2goal, near_win=0.0, near_peak=0.0):
            v = n.gps_ms()
            if v is not None:
                st['v'] = v if st['v'] is None else st['v'] + alpha * (v - st['v'])
            dp = n.drive_pulse
            if st['v'] is not None:
                n.drive_pulse = max(floor_pulse, min(dp, st['v'] / drv.MS_PER_PULSE))
            try:
                return orig(d2goal, near_win, near_peak)
            finally:
                n.drive_pulse = dp
        n.lookahead_m = la
    return m


# 강건성 — 차량 모델 오차 13종 (DEFAULT_PLANT 덮어쓰기)
VARIANTS = [
    ('기준(식별모델)', {}),
    ('백래시 0.6', {'b': 0.6}),
    ('백래시 1.4', {'b': 1.4}),
    ('물리서보 pwm0 80', {'servo': 'phys', 'pwm0': 80}),
    ('물리서보 pwm0 95', {'servo': 'phys', 'pwm0': 95}),
    ('전달비 G 1.4', {'G': 1.4}),
    ('전달비 G 1.8', {'G': 1.8}),
    ('조향지연 T 0.3', {'T': 0.30}),
    ('안테나 실제 0.9 m', {'ant_x': 0.9}),
    ('GPS 지연 0.2 s', {'gps_lat': 0.2}),
    ('고속 언더스티어', {'us_k': 0.003}),
    ('트림 −3.5', {'trim_true': -3.5}),
    ('A보드 +10%', {'k_over': 1.10}),
]


# ══════════════════════════════════════════════════════════════════════════════
#  실행
# ══════════════════════════════════════════════════════════════════════════════
def _work(job):
    cand, pulse, route, seed, plant = job
    import follow_sim as fs
    mods, params, consts = candidate(cand)
    pl = dict(plant)
    pl['seed'] = seed
    with ModuleConst(**consts):
        s = fs.Sim(fs.ROUTES[route], drive_pulse=pulse, params=params, plant=pl, mods=mods).run()
    o = fs.stats(s.tr, s.wps)
    o['stops'] = [e[1] for e in s.events if '⛔' in e[1]]
    o['done'] = float(s.tr['wp'][-1]) / max(1, len(s.wps) - 1) >= 0.97 and not o['stops']
    return job, o


def _pool(jobs, workers):
    out = {}
    ctx = mp.get_context('spawn')
    with cf.ProcessPoolExecutor(max_workers=workers, mp_context=ctx) as ex:
        for job, o in ex.map(_work, jobs):
            out.setdefault(job, o)
    return out


def _mean(L, k):
    x = [o.get(k, float('nan')) for o in L]
    return float(np.nanmean(x)) if np.any(np.isfinite(x)) else float('nan')


def cmd_compare(a):
    cands = a.cands.split(',')
    pulses = [int(x) for x in a.pulses.split(',')]
    routes = a.routes.split(',')
    seeds = range(1, a.seeds + 1)
    jobs = [(c, p, r, s, ()) for c in cands for p in pulses for r in routes for s in seeds]
    res = _pool(jobs, a.workers)
    print("흔들림 = RMS(r − v·κ경로) [°/s] · 코너 = κ>0.04 구간 · 안쪽(+) = 코너 안쪽으로 파고듦")
    for p in pulses:
        for r in routes:
            for c in cands:
                L = [res[(c, p, r, s, ())] for s in seeds]
                bad = sum(0 if o['done'] else 1 for o in L)
                print(f"{p:2d}p {r} {c:12s} 흔들림 {_mean(L, 'wob'):4.2f}°/s | |CTE| {_mean(L, 'cte_abs'):.2f} "
                      f"최대 {max(o['cte_max'] for o in L):.2f} | 코너 최대 {max(o.get('c_max', 0) for o in L):.2f} "
                      f"안쪽 {_mean(L, 'c_in'):+.2f} | 직선 std {_mean(L, 's_std'):.2f} | 횡가속 p99 {_mean(L, 'ay_p99'):.2f} "
                      f"| 서보 {_mean(L, 'servo'):.1f} | {_mean(L, 'v'):4.1f} km/h{'  ⛔ ' + str(bad) if bad else ''}")
            print()


def cmd_robust(a):
    cands = a.cands.split(',')
    pulses = [int(x) for x in a.pulses.split(',')]
    routes = a.routes.split(',')
    seeds = range(1, a.seeds + 1)
    # 차량 모델 덮어쓰기는 (키, 값) 튜플로 싣는다 — 작업 키로 쓰려면 해시가 돼야 한다
    jobs = [(c, p, r, s, tuple(sorted(pl.items()))) for _, pl in VARIANTS for c in cands
            for p in pulses for r in routes for s in seeds]
    res = _pool(jobs, a.workers)
    for p in pulses:
        print(f"\n== {p}펄스 : 흔들림[°/s] |CTE| 코너최대 최대|CTE| km/h 미완주 ==")
        for vn, pl in VARIANTS:
            key = tuple(sorted(pl.items()))
            cells = []
            for c in cands:
                L = [res[(c, p, r, s, key)] for r in routes for s in seeds]
                bad = sum(0 if o['done'] else 1 for o in L)
                cells.append(f"{c}: {_mean(L, 'wob'):5.2f} {_mean(L, 'cte_abs'):.2f} "
                             f"{max(o.get('c_max', 0) for o in L):.2f} {max(o['cte_max'] for o in L):.2f} "
                             f"{_mean(L, 'v'):4.1f}{' ⛔' + str(bad) if bad else ''}")
            print(f"{vn:16s} | " + " | ".join(cells), flush=True)


def _print_log(name, tr, wps):
    import follow_sim as fs
    o = fs.stats(tr, wps)
    b = fs.ident_servo(tr)
    print(f"{name}: 흔들림 {o['wob']:.2f}°/s | |CTE| {o['cte_abs']:.2f} 최대 {o['cte_max']:.2f} | "
          f"코너 최대 {o.get('c_max', float('nan')):.2f} 안쪽 {o.get('c_in', float('nan')):+.2f} | "
          f"직선 std {o.get('s_std', float('nan')):.2f} 주기 {o.get('s_per', float('nan')):.1f}s | "
          f"횡가속 p99 {o['ay_p99']:.2f} | {o['v']:.1f} km/h || 조향 식별 : 백래시 {b[1]:.2f}° "
          f"τ {b[2]:.2f}s T {b[3]:.2f}s 전달비 {b[4]:.2f} 트림 {b[5]:+.2f}")


def cmd_logs(a):
    import follow_sim as fs
    for key, (rec, route) in fs.LOGS.items():
        tr, wps = fs.log_trace(rec, route)
        _print_log(key, tr, wps)


def cmd_log(a):
    import follow_sim as fs
    tr, wps = fs.log_trace(a.record, a.route)
    _print_log(os.path.basename(a.record), tr, wps)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    for name in ('compare', 'robust'):
        sp = sub.add_parser(name)
        sp.add_argument('--cands', default='현행,A+B+C+D+E')
        sp.add_argument('--pulses', default='7,10')
        sp.add_argument('--routes', default='A,B,C')
        sp.add_argument('--seeds', type=int, default=3)
        sp.add_argument('--workers', type=int, default=max(1, (os.cpu_count() or 2) - 2))
    sub.add_parser('logs')
    sp = sub.add_parser('log')
    sp.add_argument('record')
    sp.add_argument('route')
    a = ap.parse_args()
    {'compare': cmd_compare, 'robust': cmd_robust, 'logs': cmd_logs, 'log': cmd_log}[a.cmd](a)


if __name__ == '__main__':
    main()
