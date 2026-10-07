#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
follow_design.py ― TODO-1(흔들림 감소 + 10펄스 증속) 설계를 모의로 비교한다  [2026-09-29 → 09-30]
════════════════════════════════════════════════════════════════════════════════
★[2026-09-30] 이제 패치가 아니라 driving.py 의 ★실제 파라미터★ 를 넘긴다★
  A~E 가 driving.py 에 들어갔으므로 모의가 실차와 같은 코드를 돈다(파라미터 기본값은
  전부 꺼짐 = 현행 — 궤적이 수정 전과 비트 단위로 같다). 근거·설계는 CHANGELOG.md 의
  2026-09-30 항목(원래 TO DO LIST — TODO-1)이 정본이다.

  A  조향 백래시 선보상     steer_backlash_deg 0.8      최종 pot 에 '움직이는 방향 × 0.8°'
  B  순수추종 기하 = 뒷차축  pp_rear_axle true (gps_ant_x_m 1.25)
  C  요레이트 댐핑          yaw_damp_k 0.15             도로휠 += 0.15·(r − v·κ경로)
  D  코너 속도 = 횡가속 상한 corner_ay_max 2.0 + curve_preview_far_m 40
  E  LFD 상한 11.3 → 14 m   lfd_max_m 14.0              10펄스가 실제로 나오게

  기각한 대안 둘(전달비 1.55 · LFD 실측속도)은 코드에 없으므로 패치로 남긴다.

사용 (ROS 환경만 source — 빌드 불필요. 소스트리의 driving.py 를 그대로 쓴다):
    source /opt/ros/humble/setup.bash
    cd gold_ws/src/white2/sim
    python3 follow_design.py compare --cands 현행,A,A+C,A+B+C,A+B+C+D+E --pulses 7,10
    python3 follow_design.py compare --cands A+C --extra '{"steer_backlash_deg": 0.6}'   # 손잡이 하나만
    python3 follow_design.py robust  --cands 현행,A+B+C+D+E --pulses 7,10
    python3 follow_design.py logs                       # 9/13 주행 4개를 같은 잣대로
    python3 follow_design.py log <record.csv> <route.csv>   # 새 실차 로그 (라이다 구간 제외)
"""
import argparse
import concurrent.futures as cf
import json
import math
import multiprocessing as mp
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


# ══════════════════════════════════════════════════════════════════════════════
#  설계 후보 — driving.py 의 실제 파라미터 [2026-09-30]
# ══════════════════════════════════════════════════════════════════════════════
A = {'steer_backlash_deg': 0.8}
B = {'pp_rear_axle': True, 'gps_ant_x_m': 1.25}
C = {'yaw_damp_k': 0.15}
D = {'corner_ay_max': 2.0, 'curve_preview_far_m': 40.0}
E = {'lfd_max_m': 14.0}


def candidate(name):
    """→ (mods, params). mods 는 기각한 대안(코드에 없는 것)만 쓴다."""
    table = {
        '현행': ([], {}),
        'A': ([], dict(A)),
        'C': ([], dict(C)),
        'A+C': ([], {**A, **C}),
        'A+B': ([], {**A, **B}),
        'A+B+C': ([], {**A, **B, **C}),
        'A+B+C+D': ([], {**A, **B, **C, **D}),
        'A+B+C+D+E': ([], {**A, **B, **C, **D, **E}),
        # 기각된 대안 (CHANGELOG 2026-09-30 '검토했으나 기각')
        '전달비1.55': ([_model_gain(1.55)], {}),
        'LFD실측속도': ([_lfd_by_speed()], {}),
    }
    return table[name]


def _model_gain(g):
    import white2.driving as drv

    def m(n):
        n.plant_gain, n.understeer = g, 0.0
        n.road_max = drv.STEER_MAX_DEG / g
    return m


def _lfd_by_speed(alpha=0.1, floor_pulse=2.0):
    import white2.driving as drv

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
    cand, pulse, route, seed, plant, extra = job
    import follow_sim as fs
    mods, params = candidate(cand)
    params = dict(params)
    params.update(dict(extra))              # --extra : 모든 후보 위에 같은 값을 덮는다
    pl = dict(plant)
    pl['seed'] = seed
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


def _extra(a):
    """--extra JSON → 작업 키로 쓸 수 있는 (키, 값) 튜플."""
    if not getattr(a, 'extra', ''):
        return ()
    return tuple(sorted(json.loads(a.extra).items()))


def _mean(L, k):
    x = [o.get(k, float('nan')) for o in L]
    return float(np.nanmean(x)) if np.any(np.isfinite(x)) else float('nan')


def cmd_compare(a):
    cands = a.cands.split(',')
    pulses = [int(x) for x in a.pulses.split(',')]
    routes = a.routes.split(',')
    seeds = range(1, a.seeds + 1)
    ex = _extra(a)
    jobs = [(c, p, r, s, (), ex) for c in cands for p in pulses for r in routes for s in seeds]
    res = _pool(jobs, a.workers)
    print("흔들림 = RMS(r − v·κ경로) [°/s] · 코너 = κ>0.04 구간 · 안쪽(+) = 코너 안쪽으로 파고듦")
    for p in pulses:
        for r in routes:
            for c in cands:
                L = [res[(c, p, r, s, (), ex)] for s in seeds]
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
    ex = _extra(a)
    jobs = [(c, p, r, s, tuple(sorted(pl.items())), ex) for _, pl in VARIANTS for c in cands
            for p in pulses for r in routes for s in seeds]
    res = _pool(jobs, a.workers)
    for p in pulses:
        print(f"\n== {p}펄스 : 흔들림[°/s] |CTE| 코너최대 최대|CTE| km/h 미완주 ==")
        for vn, pl in VARIANTS:
            key = tuple(sorted(pl.items()))
            cells = []
            for c in cands:
                L = [res[(c, p, r, s, key, ex)] for r in routes for s in seeds]
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
        sp.add_argument('--extra', default='',
                        help='모든 후보 위에 덮을 driving 파라미터 JSON (예: {"steer_backlash_deg": 0.6})')
    sub.add_parser('logs')
    sp = sub.add_parser('log')
    sp.add_argument('record')
    sp.add_argument('route')
    a = ap.parse_args()
    {'compare': cmd_compare, 'robust': cmd_robust, 'logs': cmd_logs, 'log': cmd_log}[a.cmd](a)


if __name__ == '__main__':
    main()
