#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
routepick.py ― 경로 CSV 를 GUI 창으로 고르고 ★마지막 점★ 을 읽는다 [white1 / 2026-10-16]
════════════════════════════════════════════════════════════════════════════════
  braketest.launch.py 가 노드를 띄우기 ★전에★ pick_csv() 로 창을 띄우고, 고른 파일의
  절대경로를 braketest 의 route 파라미터로 넘긴다. braketest.py 는 last_point() 로
  그 파일의 마지막 점을 목표 좌표로 쓴다 — ★CSV 를 읽는 규칙은 이 파일 하나가 소유한다★
  (런치에서 검사한 것과 노드가 읽는 것이 어긋나지 않게).

  · 파일 이름은 보지 않는다 — ★확장자 .csv 인 파일이면 어느 폴더의 무엇이든★ 고를 수 있다.
  · 열 이름은 latitude/longitude (매핑 CSV) 또는 lat/lon 을 받는다. BOM 이 붙어 있어도 된다.
  · 창은 zenity(GTK) 를 먼저 쓰고, 없거나 실행에 실패하면 tkinter 로 띄운다.
  · rclpy 를 import 하지 않는다 — 런치 프로세스에서도 부른다.
"""

import csv
import os
import shutil
import subprocess

LAT_KEYS = ('latitude', 'lat')
LON_KEYS = ('longitude', 'lon')


def _pick(row, keys):
    for k in keys:
        v = row.get(k)
        if v not in (None, ''):
            return float(v)
    raise KeyError(keys[0])


def last_point(path):
    """CSV 의 ★마지막 유효 좌표★ 와 유효 행 수 → ((lat, lon), n). 못 읽으면 예외.

    좌표로 못 읽는 행(빈 칸·글자)은 건너뛴다. 유효 행이 하나도 없으면 ValueError.
    """
    last, n = None, 0
    with open(path, 'r', encoding='utf-8-sig', newline='') as f:
        reader = csv.DictReader(f)
        cols = [c.strip().lower() for c in (reader.fieldnames or [])]
        reader.fieldnames = cols
        if not (any(k in cols for k in LAT_KEYS) and any(k in cols for k in LON_KEYS)):
            raise ValueError(f"위도·경도 열이 없다 (열: {', '.join(cols) or '없음'}) — "
                             f"latitude/longitude 또는 lat/lon 이 필요하다")
        for row in reader:
            try:
                lat, lon = _pick(row, LAT_KEYS), _pick(row, LON_KEYS)
            except (KeyError, ValueError, TypeError):
                continue
            if -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0 and (lat, lon) != (0.0, 0.0):
                last, n = (lat, lon), n + 1
    if last is None:
        raise ValueError("좌표로 읽을 수 있는 행이 없다")
    return last, n


#  ★snap 앱(VSCode 등)의 터미널에서 띄우면 그 앱의 GTK·로케일 변수가 물려 들어와
#  zenity 가 snap 라이브러리를 물고 죽는다★ (2026-10-16 실측: exit 127, libpthread
#  symbol lookup error). zenity 에 넘기는 환경에서 그 변수를 빼고 원래 값으로 되돌린다.
_SNAP_LEAK = ('GTK_PATH', 'GTK_EXE_PREFIX', 'GTK_IM_MODULE_FILE', 'GDK_PIXBUF_MODULE_FILE',
              'GDK_PIXBUF_MODULEDIR', 'GIO_MODULE_DIR', 'GSETTINGS_SCHEMA_DIR', 'LOCPATH')


def _clean_env():
    env = {k: v for k, v in os.environ.items() if k not in _SNAP_LEAK}
    for k in ('XDG_DATA_DIRS', 'XDG_CONFIG_DIRS'):
        orig = os.environ.get(f'{k}_VSCODE_SNAP_ORIG')
        if orig is not None:
            env[k] = orig
    return env


def _zenity(args):
    """zenity 를 돌린다 → (returncode, stdout). 0 = 골랐다, 1·5 = 취소·시간초과.
    ★그 밖의 코드는 '창을 못 띄웠다'★ 라 OSError 를 던진다 — 취소로 읽으면 안 된다."""
    r = subprocess.run(['zenity'] + args, capture_output=True, text=True, env=_clean_env())
    if r.returncode not in (0, 1, 5):
        raise OSError(f"zenity 실패 (exit {r.returncode}): {r.stderr.strip()[:200]}")
    return r.returncode, r.stdout.strip()


def _ask_zenity(start_dir, title):
    code, out = _zenity(['--file-selection', f'--title={title}',
                         f'--filename={os.path.join(start_dir, "")}',
                         '--file-filter=CSV | *.csv *.CSV', '--file-filter=모든 파일 | *'])
    return out if code == 0 and out else None


def _say_zenity(text):
    _zenity(['--error', '--no-wrap', '--title=경로 CSV', f'--text={text}'])


def _ask_tk(start_dir, title):
    import tkinter
    from tkinter import filedialog
    root = tkinter.Tk()
    root.withdraw()
    try:
        p = filedialog.askopenfilename(
            parent=root, title=title, initialdir=start_dir,
            filetypes=[('CSV', '*.csv *.CSV'), ('모든 파일', '*')])
    finally:
        root.destroy()
    return p or None


def _say_tk(text):
    import tkinter
    from tkinter import messagebox
    root = tkinter.Tk()
    root.withdraw()
    try:
        messagebox.showerror('경로 CSV', text, parent=root)
    finally:
        root.destroy()


def has_display():
    return bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))


def pick_csv(start_dir, title='경로 CSV 선택', log=print):
    """창을 띄워 CSV 하나를 고르게 한다 → (절대경로, (lat, lon), 행 수) / 취소면 None.

    고른 파일이 좌표를 못 내면 ★이유를 알려 주고 창을 다시 띄운다★ — 고칠 기회를 준다.
    화면이 없으면(SSH) RuntimeError — 부르는 쪽이 처리한다.
    """
    if not has_display():
        raise RuntimeError("화면(DISPLAY)이 없다")
    use_zenity = shutil.which('zenity') is not None
    ask, say = (_ask_zenity, _say_zenity) if use_zenity else (_ask_tk, _say_tk)
    start = start_dir if os.path.isdir(start_dir) else os.path.expanduser('~')
    while True:
        try:
            path = ask(start, title)
        except OSError as e:
            if ask is _ask_tk:
                raise
            log(f"    [경로] ⚠️ {e} — tkinter 창으로 바꾼다")
            ask, say = _ask_tk, _say_tk
            continue
        if not path:
            return None
        path = os.path.abspath(path)
        try:
            ll, n = last_point(path)
            return path, ll, n
        except Exception as e:          # noqa: BLE001 — 이유를 사람에게 보여 준다
            msg = f"{os.path.basename(path)} 를 쓸 수 없다:\n{e}\n\n다른 파일을 고를 것."
            log(f"    [경로] ⚠️ {msg.replace(chr(10), ' ')}")
            say(msg)
            start = os.path.dirname(path)
