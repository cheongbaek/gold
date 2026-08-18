#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tl_tune.py ― 신호등 근접도 임계(tl_red_stop_min_height) 실측 튜닝 [gold 루트, ROS 불필요]
════════════════════════════════════════════════════════════════════════════════
    python3 tl_tune.py                 ← 파일 선택창이 뜬다
    python3 tl_tune.py 주행기록.csv     ← 창 없이 바로

  record.py 가 남긴 ★접근 주행 1회★ CSV 를 읽어, 임계를 정하는 데 필요한 두 숫자를
  뽑는다. 렌즈 화각을 몰라도 되고 신호등 좌표를 측량할 필요도 없다.

      ① k [px·m]  — 렌즈 상수.  ★박스높이 = k / 거리★
                    → 임계 T[px] 를 걸면 ★k/T [m] 앞에서 물린다★ 가 바로 나온다
      ② d_lost [m] — 등기구가 화면 위로 벗어나 ★안 보이기 시작하는 거리★
                    → 정지 지점이 이보다 가까우면 해제 유예 0.5초 뒤 차가 굴러간다.
                      ★임계의 상한을 정하는 값이다★

════════════════════════════════════════════════════════════════════════════════
 어떻게 신호등까지의 거리를 아는가 (측량 없이)
════════════════════════════════════════════════════════════════════════════════
  모르는 채로 푼다. 박스높이 h 와 거리 d 는 h = k/d 이므로

        1/h = (1/k)·x + (c/k)          x = 그 지점에서 ★가장 가까웠던 지점★ 까지의
                                           GPS 주행거리(측정 가능)
                                       c = 가장 가까웠던 지점에서 신호등까지의 남은
                                           거리(모름)

  1/h 를 x 로 회귀하면 기울기에서 k, 절편에서 c 가 같이 나온다. R² 가 0.9 미만이면
  등기구가 아닌 것(미등·간판)을 섞어 본 것이니 그 기록은 버린다.
"""

import csv
import math
import os
import sys

EARTH_R = 6378137.0

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOTS = (os.path.join(HERE, 'gold_ws', 'src', 'white1'),
             os.path.join(HERE, 'gold_ws', 'src', 'white806'))

# 제동 상수 — BRAKING.md 실측. 보수적으로 약한 쪽(2.2)을 쓴다.
A_BRAKE2 = 2.2       # 리니어 2단 감속도 [m/s²]
T_REACT  = 0.6       # 확정 0.4s + 카메라·추론·시리얼 파이프라인 0.2s
D_MARGIN = 3.0       # 정지 지점이 d_lost 보다 이만큼은 멀어야 한다


def pick_file():
    try:
        import tkinter as tk
        from tkinter import filedialog
    except Exception:
        print('tkinter 가 없다. 경로를 인자로 줄 것: python3 tl_tune.py <csv>')
        sys.exit(2)
    root = tk.Tk()
    root.withdraw()
    start = HERE
    for p in PKG_ROOTS:
        if os.path.isdir(os.path.join(p, 'ros2bag')):
            start = os.path.join(p, 'ros2bag')
            break
    path = filedialog.askopenfilename(
        title='신호등 접근 주행 기록 CSV 를 고르세요 (ros2bag/*.csv)',
        initialdir=start, filetypes=[('CSV', '*.csv'), ('all', '*.*')])
    root.destroy()
    return path


def read_rows(path):
    with open(path, newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        print('빈 파일이다.')
        sys.exit(1)
    for c in ('tl_near_metric', 'fix_lat', 'fix_lon'):
        if c not in rows[0]:
            print(f"'{c}' 열이 없다 — 신호등 열이 추가되기 전(2026-08-14 이전) 기록이다.")
            sys.exit(1)
    return rows


def fnum(row, key):
    v = (row.get(key) or '').strip()
    if v == '':
        return None
    try:
        return float(v)
    except ValueError:
        return None


def build_track(rows):
    """유효 fix 만 골라 (t, 누적주행거리, near, state, brake) 로 만든다."""
    out, lat0, prev, dist = [], None, None, 0.0
    for r in rows:
        lat, lon = fnum(r, 'fix_lat'), fnum(r, 'fix_lon')
        if lat is None or lon is None or (lat == 0.0 and lon == 0.0):
            continue
        if lat0 is None:
            lat0 = lat
        x = EARTH_R * math.radians(lon) * math.cos(math.radians(lat0))
        y = EARTH_R * math.radians(lat)
        if prev is not None:
            dist += math.hypot(x - prev[0], y - prev[1])
        prev = (x, y)
        out.append({
            't': fnum(r, 't_rel') or 0.0,
            's': dist,                                   # 누적 주행거리 [m]
            'near': fnum(r, 'tl_near_metric') or 0.0,
            'state': (r.get('tl_state') or '').strip(),
            'brake': fnum(r, 'brake_level'),
        })
    return out


def regress(xs, ys):
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    if sxx == 0.0:
        return None
    a = sxy / sxx
    b = my - a * mx
    ss_res = sum((y - (a * x + b)) ** 2 for x, y in zip(xs, ys))
    ss_tot = sum((y - my) ** 2 for y in ys)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    return a, b, r2


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else pick_file()
    if not path:
        print('취소했다.')
        return
    trk = build_track(read_rows(path))
    seen = [p for p in trk if p['near'] > 0.0]
    print(f"\n파일 : {os.path.basename(path)}")
    print(f"유효 fix {len(trk)}행 · 빨간불이 보인 행 {len(seen)}행")
    if len(seen) < 8:
        print('빨간불 관측이 8행 미만이다 — 접근 주행을 다시 기록할 것.')
        return

    # ── 기준점 = ★가장 크게 보인 순간★ (= 가장 가까웠던 순간) ─────────────────
    ref = max(seen, key=lambda p: p['near'])
    fit_pts = [p for p in seen if p['s'] <= ref['s']]
    xs = [ref['s'] - p['s'] for p in fit_pts]           # 기준점까지 남은 거리 [m]
    ys = [1.0 / p['near'] for p in fit_pts]
    span = max(xs) - min(xs)
    if span < 3.0:
        print(f'접근 구간이 {span:.1f} m 뿐이다 — 최소 10 m 는 굴러가며 봐야 한다.')
        return
    res = regress(xs, ys)
    if res is None:
        return
    a, b, r2 = res
    if a <= 0:
        print('회귀 기울기가 음수다 — 멀어지며 커졌다는 뜻이라 등기구가 아니다.')
        return
    k = 1.0 / a
    c = b * k                                            # 기준점 → 신호등 남은 거리
    print(f"\n── ① 렌즈 상수 ─────────────────────────────────────────────")
    print(f"   k = {k:.0f} px·m      (박스높이 = k / 거리)   R² = {r2:.3f}"
          + ('' if r2 >= 0.9 else '   ⚠️ 0.9 미만 — 다른 물체가 섞였다. 버릴 것'))
    print(f"   접근 구간 {span:.1f} m · 박스높이 {min(p['near'] for p in fit_pts):.0f}"
          f" → {ref['near']:.0f} px")
    print(f"   가장 가까웠던 지점에서 신호등까지 남은 거리 c ≈ {c:.1f} m")

    # ── 소실거리 : 기준점 이후로 다시 0 이 된 지점 ─────────────────────────────
    after = [p for p in trk if p['s'] > ref['s']]
    lost = next((p for p in after if p['near'] == 0.0), None)
    if lost is not None and (lost['s'] - ref['s']) < 30.0:
        d_lost = c - (lost['s'] - ref['s'])
        print(f"\n── ② 소실거리 ─────────────────────────────────────────────")
        print(f"   d_lost ≈ {d_lost:.1f} m 에서 등기구가 화면 위로 벗어났다"
              f" (t={lost['t']:.1f}s)")
    else:
        d_lost = c
        print(f"\n── ② 소실거리 ─────────────────────────────────────────────")
        print(f"   기록 안에서 소실이 없었다. 가장 가까이 본 {c:.1f} m 를 하한으로 쓴다"
              f" — ★실제 소실거리는 이보다 가깝다★")

    # ── 속도 : GPS 변위로. speed_kmh 는 쓰지 않는다(BRAKING.md 근거) ───────────
    spd = []
    for p, q in zip(fit_pts, fit_pts[1:]):
        dt = q['t'] - p['t']
        if 0.02 < dt < 1.0:
            spd.append((q['s'] - p['s']) / dt)
    spd.sort()
    v = spd[len(spd) // 2] if spd else 3.54
    print(f"\n   접근 속도(중앙값) v = {v:.2f} m/s ({v * 3.6:.1f} km/h)"
          + ('' if spd else '  ※ 산출 불가 — 4펄스로 가정'))

    # ── 임계별로 무슨 일이 벌어지는가 ─────────────────────────────────────────
    d_brake = v * v / (2.0 * A_BRAKE2) + v * T_REACT
    d_stop_min = d_lost + D_MARGIN
    d_eng_min = d_stop_min + d_brake
    t_max = k / d_eng_min
    print(f"\n── ③ 임계별 결과 ──────────────────────────────────────────")
    print(f"   제동+반응거리 = {d_brake:.1f} m  (2단 {A_BRAKE2} m/s² + 반응 {T_REACT}s)")
    print(f"   {'임계[px]':>8} {'물리는 거리':>12} {'서는 지점':>10}   판정")
    for t in (15, 20, 25, 30, 40, 50, 60, 80):
        d_eng = k / t
        d_stp = d_eng - d_brake
        if d_stp < d_lost:
            verdict = '✗ 서 있는 동안 신호등이 안 보인다 → 굴러간다'
        elif d_stp < d_stop_min:
            verdict = '△ 여유 부족'
        else:
            verdict = '○'
        print(f"   {t:>8} {d_eng:>11.1f} m {d_stp:>9.1f} m   {verdict}")
    print(f"\n   ★임계 상한 = {t_max:.0f} px★ (이보다 크면 정지 지점이 소실거리 안으로 들어온다)")
    print(f"   권장 : {max(10, int(t_max * 0.8)):d} ~ {int(t_max):d} px"
          f"   →  ros2 launch white1 master.launch.py"
          f" tl_red_stop_min_height:={max(10, int(t_max * 0.8)):d}")
    print(f"   ⚠️ 상한만 계산한 것이다. 오탐(미등·간판)이 잡히는 최대 박스높이를 화면"
          f" HUD 에서 확인해 그보다는 높게 잡을 것.")

    # ── 그림 ─────────────────────────────────────────────────────────────────
    try:
        import matplotlib.pyplot as plt
    except Exception:
        return
    d_obs = [x + c for x in xs]
    h_obs = [p['near'] for p in fit_pts]
    fig, ax = plt.subplots(figsize=(9, 5.5))
    ax.plot(d_obs, h_obs, 'o', ms=4, color='#c0392b', label='observed box height')
    dd = [c + i * span / 200.0 for i in range(201)]
    ax.plot(dd, [k / d for d in dd], '-', color='#2c3e50', lw=1.2,
            label=f'fit  h = {k:.0f} / d   (R2={r2:.3f})')
    ax.axvline(d_lost, color='#e67e22', ls='--', lw=1.5,
               label=f'lost at {d_lost:.1f} m')
    ax.axhline(t_max, color='#27ae60', ls=':', lw=1.5,
               label=f'threshold upper bound {t_max:.0f} px')
    ax.axhline(25, color='#7f8c8d', ls=':', lw=1.2, label='current 25 px')
    ax.set_xlabel('distance to traffic light [m]')
    ax.set_ylabel('red box height [px]')
    ax.set_title(os.path.basename(path))
    ax.invert_xaxis()
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9)
    fig.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
