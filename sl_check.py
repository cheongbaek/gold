#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sl_check.py ― 정지선 앞 정지 시험 자동 판정 [gold 루트, ROS 불필요]
════════════════════════════════════════════════════════════════════════════════
    python3 sl_check.py                 ← 파일 선택창이 뜬다
    python3 sl_check.py 주행기록.csv     ← 창 없이 바로
    python3 sl_check.py 기록.csv --trig 0.72   ← 트리거 값을 알려주면 4-3 을 정확히 본다

  record.py 가 남긴 ★접근 주행 1회★ CSV 를 읽어, STOPLINE_TEST.md 단계 4 의 판정
  기준(4-1 ~ 4-8)을 그대로 계산한다. 사람이 CSV 를 눈으로 훑으며 판정하지 않게 하는
  것이 목적이다 — 그 판정은 '언제 참았고 언제 밟았나'라 눈으로는 놓치기 쉽다.

  tl_tune.py 와 짝이다:
      tl_tune.py  → 신호등 근접도 임계(tl_red_stop_min_height) 를 정한다
      sl_check.py → 정지선 트리거(sl_trigger_y_frac) 로 달린 결과를 판정한다

  ★판정하지 않는 것★ 실제 정지 위치(정지선 앞 몇 m 인가)는 자로 재야 한다 — CSV 에
  정지선의 절대 위치가 없기 때문이다. 대신 ★대기 중 이동거리★ 를 GPS 로 뽑아 준다.
"""

import csv
import math
import os
import sys

EARTH_R = 6378137.0

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOTS = (os.path.join(HERE, 'gold_ws', 'src', 'white1'),
             os.path.join(HERE, 'gold_ws', 'src', 'white806'))

# 판정 기준 — STOPLINE_TEST.md 단계 4 의 표와 같은 값이다.
WAIT_MIN_S     = 0.3     # 4-1 대기 구간이 이보다 짧으면 '정지선을 못 봤다'
TRIG_TOL       = 0.03    # 4-3 체결 시 sl_y 가 트리거에서 이만큼 안에 들면 정상
WAIT_MAX_S     = 5.0     # 4-8 traffic_light 의 sl_wait_max_s 기본값
RED_TO_BRAKE_S = (0.3, 0.8)   # 5-1 정지선이 없을 때의 정상 반응 시간대


def pick_file():
    try:
        import tkinter as tk
        from tkinter import filedialog
    except Exception:
        print('tkinter 가 없다. 경로를 인자로 줄 것: python3 sl_check.py <csv>')
        sys.exit(2)
    root = tk.Tk()
    root.withdraw()
    start = HERE
    for p in PKG_ROOTS:
        if os.path.isdir(os.path.join(p, 'ros2bag')):
            start = os.path.join(p, 'ros2bag')
            break
    return filedialog.askopenfilename(
        title='정지선 시험 주행 CSV 선택', initialdir=start,
        filetypes=[('CSV', '*.csv'), ('all', '*.*')])


def fnum(row, key):
    v = (row.get(key) or '').strip()
    if v == '':
        return None
    try:
        return float(v)
    except ValueError:
        return None


def fbool(row, key):
    v = (row.get(key) or '').strip().lower()
    if v == '':
        return None
    return v in ('true', '1', '1.0', 'yes')


def read_rows(path):
    with open(path, newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        print('빈 파일이다.')
        sys.exit(1)
    for c in ('sl_y', 'sl_wait'):
        if c not in rows[0]:
            print(f"'{c}' 열이 없다 — 정지선 열이 추가되기 전(2026-08-14 이전) 기록이다.\n"
                  f"   colcon build 를 다시 하고 새로 기록할 것.")
            sys.exit(1)
    return rows


def build(rows):
    """(t, 누적주행거리, tl_state, near, sl_y, sl_wait, brake) 로 정리한다."""
    out, lat0, prev, dist = [], None, None, 0.0
    for r in rows:
        lat, lon = fnum(r, 'fix_lat'), fnum(r, 'fix_lon')
        if lat is not None and lon is not None and not (lat == 0.0 and lon == 0.0):
            if lat0 is None:
                lat0 = lat
            x = EARTH_R * math.radians(lon) * math.cos(math.radians(lat0))
            y = EARTH_R * math.radians(lat)
            if prev is not None:
                dist += math.hypot(x - prev[0], y - prev[1])
            prev = (x, y)
        out.append({
            't':     fnum(r, 't_rel') or 0.0,
            's':     dist,
            'state': (r.get('tl_state') or '').strip(),
            'near':  fnum(r, 'tl_near_metric'),
            'sl':    fnum(r, 'sl_y'),
            'wait':  fbool(r, 'sl_wait'),
            'brake': fnum(r, 'brake_level'),
        })
    return out


def spans(trk, pred):
    """조건이 참인 연속 구간들을 [(i0, i1)] 로 돌려준다."""
    out, start = [], None
    for i, p in enumerate(trk):
        if pred(p):
            if start is None:
                start = i
        elif start is not None:
            out.append((start, i - 1))
            start = None
    if start is not None:
        out.append((start, len(trk) - 1))
    return out


def verdict(ok, msg_ok, msg_ng):
    return (f"○ {msg_ok}" if ok else f"✗ {msg_ng}")


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    trig = None
    for i, a in enumerate(sys.argv):
        if a == '--trig' and i + 1 < len(sys.argv):
            trig = float(sys.argv[i + 1])

    path = args[0] if args else pick_file()
    if not path:
        print('취소했다.')
        return
    trk = build(read_rows(path))
    print(f"\n파일 : {os.path.basename(path)}   ({len(trk)}행, "
          f"{trk[-1]['t'] - trk[0]['t']:.1f}초, 주행 {trk[-1]['s']:.1f} m)")

    # ★기록의 성격을 먼저 정한다★ 정지선이 한 번도 안 잡힌 기록은 '실패'가 아니라
    #   ★회귀 시험(단계 5-1)★ 일 수 있다 — 정지선이 없는 신호등에서 종전대로 서는지
    #   보는 기록이 그것이다. 성격을 안 가르면 그 기록마다 ✗ 가 쏟아진다.
    saw_sl = any(p['sl'] is not None and p['sl'] >= 0.0 for p in trk)
    mode = ('★정지선 경로★ (단계 4 기준으로 본다)' if saw_sl else
            '★종전 경로★ — 정지선이 한 번도 안 잡혔다. 회귀 시험(단계 5-1) 기준으로 본다')
    print(f"   판정 모드 : {mode}")

    # ── 브레이크 체결 시점 (0/빈값 → 2) ──────────────────────────────────────
    engages, releases, prev_b = [], [], None
    for i, p in enumerate(trk):
        b = p['brake']
        if b is None:
            continue
        if prev_b is not None:
            if prev_b == 0 and b > 0:
                engages.append(i)
            elif prev_b > 0 and b == 0:
                releases.append(i)
        prev_b = b

    # ── 대기 구간 ────────────────────────────────────────────────────────────
    wsp = [(a, b) for (a, b) in spans(trk, lambda p: p['wait'] is True)
           if trk[b]['t'] - trk[a]['t'] > 0.0]
    wait_max = max((trk[b]['t'] - trk[a]['t'] for a, b in wsp), default=0.0)

    print("\n" + "═" * 74)
    print(" 단계 4 판정 — STOPLINE_TEST.md 의 표와 같은 번호다")
    print("═" * 74)

    # 4-1 대기 구간이 존재하는가
    print(f" 4-1 대기 구간      : {len(wsp)}개, 최장 {wait_max:.2f}초")
    if not saw_sl:
        print("     — 해당 없음(종전 경로 기록). 정지선이 없으면 참지 않는 것이 정상이다")
    else:
        print("     " + verdict(
            wait_max >= WAIT_MIN_S,
            f"{WAIT_MIN_S}초 이상 참았다 — 정지선을 보고 기다렸다",
            f"{WAIT_MIN_S}초 미만 — 정지선은 잡혔는데 참지 않았다. 트리거가 낮아 "
            f"보자마자 참이 됐거나(트리거 ↑), 확정 전에 지나쳤다(sl_hold_s 확인)"))

    # 4-2 대기 중에 브레이크가 섞였는가
    bad = 0
    for a, b in wsp:
        bad += sum(1 for p in trk[a:b + 1] if (p['brake'] or 0) > 0)
    print(f"\n 4-2 대기 중 brake>0 : {bad}행")
    print("     " + verdict(bad == 0, "대기 구간이 깨끗하다",
                            "대기 중에 브레이크가 물렸다 — 다른 발행자(driving?)를 볼 것"))

    # 4-3 체결 시각의 sl_y
    print(f"\n 4-3 브레이크 체결   : {len(engages)}회")
    for i in engages:
        p = trk[i]
        sl = p['sl']
        # 체결 직전에 대기하고 있었는가 = 정지선 경로로 밟은 것인가
        was_wait = any(a <= i <= b + 15 for a, b in wsp)
        tag = ''
        if sl is None or sl < 0:
            tag = '정지선 없음/놓침'
        elif trig is not None:
            tag = ('정지선 앞' if sl >= trig - TRIG_TOL
                   else f'★트리거({trig:.2f})에 못 미친 채 물렸다★')
        else:
            tag = '정지선 앞(트리거 미지정 — --trig 로 주면 정밀 판정)'
        print(f"     t={p['t']:7.2f}s  sl_y={'--' if sl is None or sl < 0 else f'{sl:.3f}'}"
              f"  tl={p['state']:<8} near={p['near'] if p['near'] is not None else '--'}"
              f"  {'[대기 후]' if was_wait else '[대기 없음]'}  {tag}")
    if not engages:
        print("     ✗ 브레이크가 한 번도 안 물렸다 — 이 기록으로는 판정할 수 없다")

    # 4-6 채터링 — 한 번의 접근에서 두 번 이상 물렸으면 리니어가 왕복한 것이다
    print(f"\n 4-6 브레이크 왕복   : 체결 {len(engages)}회 / 해제 {len(releases)}회")
    print("     " + verdict(len(engages) <= 1,
                            "왕복 없음",
                            f"★{len(engages)}번 물렸다 — 리니어 왕복이다(8항 회귀 확인)★"))

    # 4-7 정지 유지 중 near_metric
    zero_near = 0
    if engages:
        i0 = engages[0]
        held = [p for p in trk[i0:] if (p['brake'] or 0) > 0]
        zero_near = sum(1 for p in held if p['near'] is not None and p['near'] <= 0.0)
        print(f"\n 4-7 정지 유지 중 near=0 : {zero_near}행 / {len(held)}행")
        print("     " + verdict(
            zero_near == 0, "등기구가 계속 보였다",
            "정지 중 신호등이 화면에서 사라졌다 → ★카메라를 5~8° 위로 틸트★"
            " (해제 유예 뒤 차가 굴러간다)"))

    # 4-8 대기 상한
    print(f"\n 4-8 대기 상한(5초)  : 최장 {wait_max:.2f}초")
    print("     " + verdict(
        wait_max < WAIT_MAX_S - 0.2, "상한에 닿지 않았다",
        "★상한에 닿았다★ — 트리거가 도달 불가능하거나 정지선 오검출이다"))

    # ── 대기 구간의 물리적 크기 ──────────────────────────────────────────────
    if wsp:
        a, b = max(wsp, key=lambda ab: trk[ab[1]]['t'] - trk[ab[0]]['t'])
        d = trk[b]['s'] - trk[a]['s']
        dt = trk[b]['t'] - trk[a]['t']
        sl0 = next((p['sl'] for p in trk[a:b + 1] if p['sl'] is not None and p['sl'] >= 0), None)
        sl1 = next((p['sl'] for p in reversed(trk[a:b + 1])
                    if p['sl'] is not None and p['sl'] >= 0), None)
        print("\n" + "─" * 74)
        print(f" 최장 대기 구간 : t={trk[a]['t']:.2f} → {trk[b]['t']:.2f}s ({dt:.2f}초)")
        print(f"   그동안 이동거리 = ★{d:.2f} m★   평균속도 {d / dt if dt > 0 else 0:.2f} m/s")
        print(f"   정지선 sl_y : {sl0 if sl0 is None else f'{sl0:.3f}'}"
              f" → {sl1 if sl1 is None else f'{sl1:.3f}'}  (가까워질수록 커진다)")
        print(f"   ※ 이 거리만큼 ★정지선 앞으로 더 갔다★ — 종전 코드였다면 여기서 섰다")

    # ── 정지선 인지 품질 ─────────────────────────────────────────────────────
    seen = [p for p in trk if p['sl'] is not None and p['sl'] >= 0.0]
    red = [p for p in trk if p['state'] in ('RED', 'RED_FAR')]
    print("\n" + "─" * 74)
    print(f" 정지선 인지 : 검출 {len(seen)}행 / 빨간불 구간 {len(red)}행"
          + (f"  ({100.0 * len(seen) / len(red):.0f}%)" if red else ""))
    if seen:
        print(f"   sl_y 범위 {min(p['sl'] for p in seen):.3f} ~ "
              f"{max(p['sl'] for p in seen):.3f}")
        gaps = []
        prev_t = None
        for p in seen:
            if prev_t is not None and p['t'] - prev_t > 0.2:
                gaps.append(p['t'] - prev_t)
            prev_t = p['t']
        print(f"   0.2초 이상 끊긴 횟수 = {len(gaps)}"
              + (f" (최장 {max(gaps):.2f}초)" if gaps else "")
              + ("   ※ 0.5초를 넘으면 '놓침'으로 판정되어 그 자리에서 선다"
                 if gaps and max(gaps) > 0.4 else ""))
    else:
        print("   — 정지선 검출 0행. ★정지선을 볼 수 있는 곳에서 잰 기록이라면★ "
              "단계 1 로 돌아갈 것\n"
              "     (정지선이 없는 곳에서 잰 회귀 기록이라면 이것이 정상이다)")

    # ── 회귀(5-1) : 정지선 없이 물린 경우의 반응 시간 ────────────────────────
    if engages:
        i = engages[0]
        rs = None
        for j in range(i, -1, -1):
            if trk[j]['state'] != 'RED':
                rs = j + 1
                break
        if rs is not None and rs <= i:
            dt = trk[i]['t'] - trk[rs]['t']
            print("\n" + "─" * 74)
            print(f" 참고 : RED 첫 관측 → 브레이크 체결 = {dt:.2f}초")
            if (trk[i]['sl'] or -1) < 0:
                print("     " + verdict(
                    RED_TO_BRAKE_S[0] <= dt <= RED_TO_BRAKE_S[1],
                    f"정지선 없는 경로의 정상 범위 {RED_TO_BRAKE_S} 안이다 (5-1 합격)",
                    f"★{RED_TO_BRAKE_S} 범위 밖★ — 정지선이 없는데 반응이 달라졌다면"
                    " 관문이 새는 것이다(오검출 확인)"))
            else:
                print("     (정지선 경로로 물렸으므로 이 시간은 5-1 기준과 비교하지 않는다)")

    print("\n" + "═" * 74)
    print(" 자로 재야 하는 것 : 실제 정지 위치(범퍼 → 정지선) = ______ m  [4-5]")
    print(" 눈으로 볼 것     : 노드 로그의 정지 사유 [정지선 앞/놓침/없음]  [4-4]")
    print("═" * 74)


if __name__ == '__main__':
    main()
