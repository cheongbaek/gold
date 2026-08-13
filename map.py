#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""map.py ― 매핑 궤적 vs 주행 궤적 육안 비교 [gold 루트, ROS 불필요]
════════════════════════════════════════════════════════════════════════════════
    python3 map.py                     ← 파일 선택창이 두 번 뜬다
    python3 map.py 매핑.csv 주행.csv    ← 창 없이 바로 (반복 확인용)

  ① 첫 번째 창 : ★매핑 CSV★  gps_data/route_*.csv        → ★파란 선★
       mapping.py 가 남긴 것. latitude·longitude 열을 읽는다.
  ② 두 번째 창 : ★주행 기록 CSV★ ros2bag/route_*-*.csv   → ★빨간 선★
       record.py 가 남긴 것. 로스백 표의 fix_lat·fix_lon(=/fix 원값) 열을 읽는다.
    두 번째를 취소하면 매핑만 그린다.

════════════════════════════════════════════════════════════════════════════════
 왜 위경도를 그대로 그리지 않는가
════════════════════════════════════════════════════════════════════════════════
  위도 1° 와 경도 1° 는 길이가 다르다(이 위도에서 경도 쪽이 약 0.8배). 위경도를 그냥
  x·y 로 찍으면 궤적이 동서로 늘어나 ★코너의 모양이 실제와 달라 보인다★ — 육안 비교가
  목적이므로 그건 곤란하다. 그래서 mapping.py·driving.py 가 쓰는 것과 ★같은 국소평면
  근사★ 로 미터로 바꾼다:
        x = R·Δlon·cos(lat0)   [동쪽+]      y = R·Δlat   [북쪽+]
  원점은 ★매핑 첫 점★ 이다 — 두 파일을 같은 원점으로 놓아야 겹쳐 볼 수 있다.
  축은 aspect='equal' 이라 화면상 1m 가 가로세로 같은 길이다.
"""

import csv
import math
import os
import sys

import matplotlib.font_manager as fm
import matplotlib.pyplot as plt

EARTH_R = 6378137.0

# ── 열 이름 후보 (앞에 있는 것부터 찾는다. 소문자로 비교) ─────────────────────
#   매핑 CSV = latitude/longitude, 주행 기록 CSV = fix_lat/fix_lon.
#   한 파일이 둘 다 갖고 있으면 앞에 적힌 쪽을 쓴다.
LAT_KEYS = ('latitude', 'fix_lat', 'lat', 'gps_lat', 'ego_lat')
LON_KEYS = ('longitude', 'fix_lon', 'lon', 'lng', 'gps_lon', 'ego_lon')

# ── 파일 선택창의 시작 폴더 후보 (있는 것 중 CSV 가 들어 있는 첫 폴더) ────────
HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOTS = (os.path.join(HERE, 'gold_ws', 'src', 'white1'),
             os.path.join(HERE, 'gold_ws', 'src', 'white806'),
             os.path.join(HERE, 'gold_ws', 'src', 'white'))

KO_FONTS = ('NanumGothic', 'NanumBarunGothic', 'Noto Sans CJK KR',
            'Malgun Gothic', 'AppleGothic')

TEXT_KO = {'map': '매핑 궤적', 'rec': '주행 궤적', 'x': '동쪽 [m]', 'y': '북쪽 [m]',
           'start': '시작', 'title': '매핑(파랑) vs 주행(빨강)',
           'pick_map': '① 매핑 CSV 를 고르세요 (gps_data/route_*.csv)',
           'pick_rec': '② 주행 기록 CSV 를 고르세요 (ros2bag/*.csv)'}
TEXT_EN = {'map': 'mapped', 'rec': 'driven', 'x': 'East [m]', 'y': 'North [m]',
           'start': 'start', 'title': 'mapped (blue) vs driven (red)',
           'pick_map': '(1) pick mapping CSV', 'pick_rec': '(2) pick record CSV'}


def setup_font():
    """한글 폰트가 있으면 쓰고, 없으면 영어 라벨로 내려간다(네모칸 방지)."""
    have = {f.name for f in fm.fontManager.ttflist}
    for name in KO_FONTS:
        if name in have:
            plt.rcParams['font.family'] = name
            plt.rcParams['axes.unicode_minus'] = False   # 한글폰트는 −(U+2212) 가 없다
            return TEXT_KO
    print("⚠️ 한글 폰트를 못 찾았다 — 영어 라벨로 그린다 "
          "(sudo apt install fonts-nanum 이면 해결된다)")
    return TEXT_EN


def start_dir(subdir):
    """선택창을 열 폴더. CSV 가 실제로 들어 있는 첫 후보를 고른다."""
    for root in PKG_ROOTS:
        d = os.path.join(root, subdir)
        try:
            if any(f.endswith('.csv') for f in os.listdir(d)):
                return d
        except OSError:
            continue
    for root in PKG_ROOTS:                       # CSV 는 없어도 폴더는 있을 수 있다
        d = os.path.join(root, subdir)
        if os.path.isdir(d):
            return d
    return HERE


def pick_file(title, initialdir):
    """파일 선택창. 취소하면 '' 를 돌려준다."""
    import tkinter as tk
    from tkinter import filedialog
    root = tk.Tk()
    root.withdraw()                              # 빈 본창은 띄우지 않는다
    root.attributes('-topmost', True)            # 선택창이 뒤로 숨지 않게
    path = filedialog.askopenfilename(
        title=title, initialdir=initialdir,
        filetypes=[('CSV', '*.csv'), ('모든 파일', '*.*')])
    root.destroy()
    return path or ''


def read_track(path):
    """CSV → (lats, lons). 열 이름은 위 후보에서 찾고, 못 찾으면 예외를 낸다.

    ★값이 비었거나 NaN 인 행은 건너뛴다★ record.py 의 표는 20Hz 스냅샷이라 GPS 가
    아직 한 번도 안 온 앞부분이 비어 있고, 그 뒤로는 5Hz 값이 그대로 유지(hold)되어
    같은 점이 여러 줄 반복된다 — 반복은 같은 자리에 찍히므로 그림에 영향이 없다.
    """
    with open(path, newline='', encoding='utf-8-sig', errors='replace') as fp:
        reader = csv.DictReader(fp)
        if not reader.fieldnames:
            raise ValueError('빈 파일이다')
        head = {(name or '').strip().lower(): name for name in reader.fieldnames}
        lat_key = next((head[k] for k in LAT_KEYS if k in head), None)
        lon_key = next((head[k] for k in LON_KEYS if k in head), None)
        if lat_key is None or lon_key is None:
            raise ValueError(f'위경도 열을 못 찾았다 — 열: {", ".join(reader.fieldnames[:12])}…')

        lats, lons = [], []
        for row in reader:
            try:
                lat = float(row[lat_key])
                lon = float(row[lon_key])
            except (TypeError, ValueError):
                continue                         # 빈칸·문자열
            if math.isnan(lat) or math.isnan(lon):
                continue
            if abs(lat) < 1e-6 and abs(lon) < 1e-6:
                continue                         # 0,0 = 미수신
            if abs(lat) > 90.0 or abs(lon) > 180.0:
                continue
            lats.append(lat)
            lons.append(lon)
    if not lats:
        raise ValueError('쓸 수 있는 위경도 행이 하나도 없다')
    return lats, lons


def to_local(lats, lons, lat0, lon0):
    """위경도 → 원점(lat0, lon0) 기준 국소평면 [m]. mapping.py._delta 와 같은 식."""
    c = math.cos(math.radians(lat0))
    xs = [EARTH_R * math.radians(lon - lon0) * c for lon in lons]
    ys = [EARTH_R * math.radians(lat - lat0) for lat in lats]
    return xs, ys


def path_length(xs, ys):
    return sum(math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
               for i in range(1, len(xs)))


def draw(tracks, text):
    """tracks = [(라벨, xs, ys, 색)] 을 한 그림에 겹쳐 그린다."""
    fig, ax = plt.subplots(figsize=(9, 8))
    for i, (label, xs, ys, color) in enumerate(tracks):
        ax.plot(xs, ys, '-', color=color, lw=1.6, alpha=0.85,
                label=f"{label}  ({len(xs)}점, {path_length(xs, ys):.1f} m)")
        ax.plot(xs[0], ys[0], 'o', color=color, ms=9, mfc='white', mew=2)
        ax.plot(xs[-1], ys[-1], 's', color=color, ms=8)
        # 두 궤적의 출발점은 거의 같은 자리라 글자가 겹친다 — 위아래로 갈라 놓는다
        ax.annotate(f"{label} {text['start']}", (xs[0], ys[0]),
                    textcoords='offset points', xytext=(10, 10 - 22 * i),
                    fontsize=9, color=color)

    ax.set_aspect('equal', adjustable='box')      # ★1m 가 가로세로 같은 길이★
    ax.grid(True, ls=':', alpha=0.5)
    ax.set_xlabel(text['x'])
    ax.set_ylabel(text['y'])
    ax.set_title(text['title'])
    ax.legend(loc='best', fontsize=9)
    fig.tight_layout()
    plt.show()


def main(argv):
    text = setup_font()

    # 인자로 주면 창을 띄우지 않는다(같은 파일을 반복해 볼 때 편하다)
    if len(argv) > 1:
        map_path = argv[1]
        rec_path = argv[2] if len(argv) > 2 else ''
    else:
        map_path = pick_file(text['pick_map'], start_dir('gps_data'))
        if not map_path:
            print('취소했다 — 매핑 CSV 가 있어야 그릴 수 있다')
            return 1
        rec_path = pick_file(text['pick_rec'], start_dir('ros2bag'))
        if not rec_path:
            print('두 번째 선택을 취소했다 — 매핑 궤적만 그린다')

    tracks = []
    origin = None
    for path, label, color in ((map_path, text['map'], 'tab:blue'),
                               (rec_path, text['rec'], 'tab:red')):
        if not path:
            continue
        try:
            lats, lons = read_track(path)
        except (OSError, ValueError) as e:
            print(f"❌ {os.path.basename(path)} : {e}")
            continue
        if origin is None:                        # ★두 궤적은 같은 원점을 쓴다★
            origin = (lats[0], lons[0])
        xs, ys = to_local(lats, lons, *origin)
        tracks.append((label, xs, ys, color))
        print(f"{label:>4s} : {os.path.basename(path)}  "
              f"{len(xs)}점, {path_length(xs, ys):.1f} m")

    if not tracks:
        print('그릴 것이 없다')
        return 1
    draw(tracks, text)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
