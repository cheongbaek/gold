#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
videomerge.py ― 녹화 조각 → 최종본 한 파일 [nxde] [2026-10-16]
════════════════════════════════════════════════════════════════════════════════
    ros2 run nxde video_merge                 기본 폴더들의 parts/ 에 남은 세션을 합친다
    ros2 run nxde video_merge <폴더> …         그 폴더들만
    ros2 run nxde video_merge --legacy <폴더>  ★옛 방식(조각이 맨 위에 섞인 것)★ 도 합친다
                                              — 원래 조각은 휴지통으로 (gio trash)
    ros2 run nxde video_merge --dry-run …     무엇을 할지만 보여 준다
    python3 -m nxde.videomerge <parts/세션> [--keep]   녹화 노드가 부르는 꼴(한 세션)
    python3 -m nxde.videomerge --orphans <폴더>        녹화 노드가 뜰 때 부르는 꼴

★왜 있나★ nxde/video 는 SIGKILL 에 대비해 120 s 마다 조각을 닫는다(video.py '세그먼트 분할').
그 조각이 최종 폴더에 그대로 쌓이면 'tl-…_033201.mp4' 가 전체인지 첫 조각인지 이름으로 못
가른다. 그래서 [2026-10-16] 부터 폴더를 이렇게 쓴다:

    video/
      tl-20261016_033201.mp4          ← ★최종본만★ (한 주행 = 한 파일)
      parts/tl-20261016_040512/       ← 녹화 중이거나 아직 못 합친 세션의 조각
          tl-20261016_040512-01.mp4 …

★누가 부르나★ nxde/video 가 파일을 닫은 직후 이 모듈을 ★별도 프로세스★ 로 띄운다
(`python3 -m nxde.videomerge <세션 폴더>`, 자기 세션). ROS 를 import 하지 않으므로
`nxde kill` 의 판정(/proc maps·--ros-args·install 경로)에 걸리지 않고, 런치의 Ctrl+C 가
프로세스 그룹에 가도 닿지 않는다. 녹화 노드가 뜰 때도 한 번 불러 SIGKILL·전원 차단으로
못 합친 세션(orphan)을 마저 합친다.

★합치는 법★ ffmpeg concat 으로 ★다시 인코딩하지 않고★ 잇는다(-c copy — 화질 손실 0,
85 MB·11조각에 0.12 s). 순서:
  ① 조각마다 ffprobe — 열리지 않는 것(SIGKILL 로 끊긴 마지막 조각 = moov 없음)은 뺀다
  ② 이어 붙여 세션 폴더 안의 임시 파일로
  ③ ★프레임 수가 조각 합과 같고 길이가 맞을 때만★ 최종본 자리로 옮기고 조각을 지운다
     — 하나라도 어긋나면 조각을 그대로 두고 경고한다(잃는 것이 없다)
  조각이 하나뿐이면 합치지 않고 옮기기만 한다. 남은 디스크가 (조각 합 + MIN_FREE_MB)
  보다 적으면 합치지 않는다(잠시 두 배가 필요하다).

★잠금★ 녹화 중인 세션 폴더에는 `.recording`(녹화 노드 pid)이 있다 — orphan 정리는 그
pid 가 살아 있으면 건드리지 않는다(같은 폴더에 녹화 노드가 둘이어도 안전). 합치는 동안은
`.merging` 을 둔다(동시에 두 번 합치지 않게).

로그 : ~/.ros/log/video_merge.log (별도 프로세스라 화면에 안 보인다 — 녹화 노드 로그에는
'합본을 맡겼다' 까지만 찍힌다).
"""

import math
import os
import re
import shutil
import subprocess
import sys
from datetime import datetime

PARTS_DIR = 'parts'                  # 조각 폴더 이름 — video.py 가 import 한다(단일 소유자)
LOCK_RECORDING = '.recording'        # 녹화 노드가 세운다 (내용 = pid)
LOCK_MERGING = '.merging'            # 이 모듈이 합치는 동안 (내용 = pid)
MIN_FREE_MB = 500.0                  # video.py 의 MIN_FREE_MB 와 같은 값
VIDEO_EXT = ('.mp4', '.avi')
LOG_PATH = os.path.join(os.path.expanduser('~'), '.ros', 'log', 'video_merge.log')

#  옛 방식 파일 이름 : <prefix>-<YYYYmmdd_HHMMSS>[-NN].<ext>
LEGACY_RE = re.compile(r'^(?P<session>.+-\d{8}_\d{6})(?:-(?P<n>\d{2}))?(?P<ext>\.(?:mp4|avi))$')
#  새 방식 조각 이름 : <session>-NN.<ext>
PART_RE = re.compile(r'-(?P<n>\d{2,})(?P<ext>\.(?:mp4|avi))$')


# ══════════════════════════════════════════════════════════════════════════════
#  작은 도구
# ══════════════════════════════════════════════════════════════════════════════
def _log_to(path):
    def log(msg):
        line = f"[{datetime.now().strftime('%m-%d %H:%M:%S')}] {msg}"
        print(line, flush=True)
        if path:
            try:
                os.makedirs(os.path.dirname(path), exist_ok=True)
                with open(path, 'a', encoding='utf-8') as f:
                    f.write(line + '\n')
            except OSError:
                pass
    return log


def _pid_alive(lock_path):
    """잠금 파일의 pid 가 살아 있는가. 파일이 없거나 못 읽으면 False."""
    try:
        with open(lock_path) as f:
            pid = int(f.read().strip() or 0)
    except (OSError, ValueError):
        return False
    if pid <= 0:
        return False
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def probe(path):
    """→ (열림, 프레임 수, 길이 s, time_base 분모, fps). 열리지 않으면 (False, 0, 0.0, 0, 0.0)."""
    def run(args):
        r = subprocess.run(['ffprobe', '-v', 'error', '-select_streams', 'v:0'] + args + [path],
                           capture_output=True, text=True)
        return r.returncode, r.stdout.strip()
    rc, out = run(['-show_entries', 'stream=nb_frames,time_base,r_frame_rate:format=duration',
                   '-of', 'default=nw=1'])
    if rc != 0 or not out:
        return False, 0, 0.0, 0, 0.0
    vals = dict(line.split('=', 1) for line in out.splitlines() if '=' in line)
    try:
        dur = float(vals.get('duration', 0) or 0)
    except ValueError:
        dur = 0.0
    try:
        frames = int(vals.get('nb_frames', 0))
    except ValueError:                       # avi 등 컨테이너에 프레임 수가 없다 — 세어 본다
        rc, out = run(['-count_packets', '-show_entries', 'stream=nb_read_packets', '-of', 'csv=p=0'])
        frames = int(out) if rc == 0 and out.isdigit() else 0
    try:
        tb = int(vals.get('time_base', '1/0').split('/')[1])
    except (ValueError, IndexError):
        tb = 0
    try:
        num, den = vals.get('r_frame_rate', '0/1').split('/')
        fps = float(num) / float(den) if float(den) else 0.0
    except (ValueError, ZeroDivisionError):
        fps = 0.0
    return frames > 0, frames, dur, tb, fps


def part_end(path):
    """조각의 마지막 프레임이 ★실제로 끝나는 시각★ [s] = max(pts + duration) — 컨테이너 길이(반올림)나
    r_frame_rate(근삿값, 예 25.759 → 644/25)로 셈하면 경계에서 몇 ms 겹친다. 못 읽으면 0."""
    r = subprocess.run(['ffprobe', '-v', 'error', '-select_streams', 'v:0', '-show_entries',
                        'packet=pts_time,duration_time', '-of', 'csv=p=0', path],
                       capture_output=True, text=True)
    end = 0.0
    for line in r.stdout.splitlines():
        a = line.split(',')
        try:
            end = max(end, float(a[0]) + float(a[1]))
        except (ValueError, IndexError):
            continue
    return end


def _part_key(path):
    m = PART_RE.search(os.path.basename(path))
    return int(m.group('n')) if m else 0


def _unique(path):
    """이미 있으면 -merged, -merged2 … 를 붙인다(덮어쓰지 않는다)."""
    if not os.path.exists(path):
        return path
    root, ext = os.path.splitext(path)
    i = 1
    while True:
        cand = f"{root}-merged{'' if i == 1 else i}{ext}"
        if not os.path.exists(cand):
            return cand
        i += 1


# ══════════════════════════════════════════════════════════════════════════════
#  세션 하나 합치기
# ══════════════════════════════════════════════════════════════════════════════
def merge_session(session_dir, out_dir=None, keep_parts=False, dry_run=False, log=print):
    """parts/<세션>/ 의 조각 → <out_dir>/<세션>.<ext>. 최종본 경로를 돌려준다(못 했으면 None).

    keep_parts=True 면 검증 뒤에도 조각을 지우지 않는다(옛 방식 정리에서 휴지통으로 보낼 때)."""
    session_dir = os.path.abspath(session_dir)
    session = os.path.basename(session_dir.rstrip(os.sep))
    if out_dir is None:                      # .../video/parts/<세션> → .../video
        out_dir = os.path.dirname(os.path.dirname(session_dir))
    parts = sorted((os.path.join(session_dir, n) for n in os.listdir(session_dir)
                    if n.lower().endswith(VIDEO_EXT) and not n.startswith('.')), key=_part_key)
    if not parts:
        log(f"· {session}: 조각이 없다 — 빈 폴더를 지운다")
        if not dry_run:
            shutil.rmtree(session_dir, ignore_errors=True)
        return None

    ext = os.path.splitext(parts[0])[1]
    good, bad = [], []
    for p in parts:
        ok, fr, du, tb, fps = probe(p)
        (good if ok else bad).append((p, fr, du, tb, fps))
    for p, *_ in bad:
        log(f"  ⚠️ {os.path.basename(p)} 는 열리지 않는다(moov 없음 — 강제 종료로 끊긴 조각) → 뺀다")
    if not good:
        log(f"❌ {session}: 열리는 조각이 하나도 없다 — 조각을 그대로 둔다 ({session_dir})")
        return None

    final = _unique(os.path.join(out_dir, session + ext))
    frames = sum(g[1] for g in good)
    dur = sum(g[2] for g in good)
    size = sum(os.path.getsize(g[0]) for g in good)
    if dry_run:
        log(f"(dry-run) {session}: 조각 {len(good)}개{f' (깨진 {len(bad)}개 제외)' if bad else ''} "
            f"→ {final}  {frames}프레임 / {dur:.1f}s / {size / 1e6:.1f}MB")
        return final
    free_mb = shutil.disk_usage(out_dir).free / 1e6
    if len(good) > 1 and free_mb < size / 1e6 + MIN_FREE_MB:
        log(f"❌ {session}: 남은 디스크 {free_mb:.0f}MB 로는 합칠 수 없다(조각 {size / 1e6:.0f}MB "
            f"+ 여유 {MIN_FREE_MB:.0f}MB) — 조각을 그대로 둔다")
        return None

    lock = os.path.join(session_dir, LOCK_MERGING)
    try:
        fd = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
    except FileExistsError:
        if _pid_alive(lock):
            log(f"· {session}: 다른 프로세스가 합치는 중 — 건너뛴다")
            return None
        os.remove(lock)                      # 죽은 합본의 잔재
        fd = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
    os.write(fd, str(os.getpid()).encode())
    os.close(fd)

    try:
        if len(good) == 1:                   # 합칠 것이 없다 — 옮기기만
            os.replace(good[0][0], final)
            ok, fr, du, *_ = probe(final)
        else:
            # ★시간 단위(time_base)를 맞춘다★ concat 은 첫 파일의 단위를 따르므로 조각마다 다르면
            # 길이가 틀어진다(실측 : 첫 조각 1/11992 · 나머지 1/29979 → 합본이 2.5배로 늘었다 —
            # 2026-10-16 이전 녹화 노드가 첫 조각만 반올림 안 한 fps 로 열었다). 다시 인코딩하지 않고
            # 단위만 바꿔 다시 담는다(-c copy -video_track_timescale).
            srcs = [g[0] for g in good]
            # 기준 = ★조각들 단위의 최소공배수★ — 모든 조각의 틱이 정수배로 바뀌어 반올림이 없다.
            # 한 단위로 '맞추기' 만 하면(예 1/10304 → 1/25759, 비율 2.4999…) 프레임마다 반올림한
            # 간격이 누적돼 DTS 가 겹친다(실측 — 원본은 400틱 등간격인데 다시 담은 조각에 경고 6~9줄).
            # mp4 시간 단위는 32비트라 넘으면(드묾) 가장 많은 조각의 단위로 물러선다.
            tbs = sorted({g[3] for g in good if g[3] > 0})
            target = math.lcm(*tbs) if tbs else 0
            if target > 2 ** 31 - 1:
                cnt = {}
                for g in good:
                    cnt[g[3]] = cnt.get(g[3], 0) + 1
                target = max(cnt, key=lambda k: (cnt[k], k))
            if ext == '.mp4' and target and any(g[3] != target for g in good):
                srcs = []
                for i, g in enumerate(good):
                    if g[3] == target:
                        srcs.append(g[0])
                        continue
                    norm = os.path.join(session_dir, f'.norm-{i:02d}.mp4')
                    r = subprocess.run(['ffmpeg', '-v', 'error', '-y', '-i', g[0], '-c', 'copy',
                                        '-video_track_timescale', str(target), '-f', 'mp4', norm],
                                       capture_output=True, text=True)
                    if r.returncode != 0:
                        log(f"❌ {session}: 시간 단위 맞추기 실패 — 조각을 그대로 둔다\n   {r.stderr.strip()[:300]}")
                        return None
                    srcs.append(norm)
                log(f"  · {session}: 조각마다 시간 단위가 달라 1/{target} 로 맞춰 잇는다")
            # ★조각 길이를 '마지막 프레임이 끝나는 시각' 으로 못 박는다★ concat 은 다음 조각의 시작을
            # 앞 조각의 컨테이너 길이로 정하는데, 그 값이 반올림돼 마지막 프레임 끝보다 조금 짧으면
            # 경계에서 두 프레임이 같은 시각(DTS)에 놓인다(실측 '…non monotonically increasing dts').
            lst = os.path.join(session_dir, '.concat.txt')
            with open(lst, 'w', encoding='utf-8') as f:
                for p in srcs:
                    f.write("file '" + p.replace("'", "'\\''") + "'\n")
                    end = part_end(p)
                    if end > 0:
                        f.write(f"duration {end:.6f}\n")
            tmp = os.path.join(session_dir, '.merging' + ext)   # '.merging.mp4' — 잠금 파일 '.merging' 과 다른 이름
            fmt = 'mp4' if ext == '.mp4' else 'avi'
            cmd = ['ffmpeg', '-v', 'error', '-y', '-f', 'concat', '-safe', '0', '-i', lst,
                   '-c', 'copy'] + (['-movflags', '+faststart'] if fmt == 'mp4' else []) + ['-f', fmt, tmp]
            r = subprocess.run(cmd, capture_output=True, text=True)
            if r.returncode != 0:
                log(f"❌ {session}: ffmpeg 실패 — 조각을 그대로 둔다\n   {r.stderr.strip()[:300]}")
                return None
            ok, fr, du, *_ = probe(tmp)
            # ★검증★ 프레임 수는 정확히 같아야 하고, 길이는 조각마다 한 프레임 남짓까지만 다를 수 있다
            if not ok or fr != frames or abs(du - dur) > 0.1 + 0.05 * len(good):
                log(f"❌ {session}: 합본 검증 실패 (합본 {fr}프레임/{du:.2f}s ≠ 조각 합 "
                    f"{frames}프레임/{dur:.2f}s) — 조각을 그대로 둔다")
                os.remove(tmp)
                return None
            os.replace(tmp, final)
            os.remove(lst)
            for p in srcs:
                if os.path.basename(p).startswith('.norm-'):
                    os.remove(p)
        log(f"✅ {session}: 조각 {len(good)}개{f' (깨진 {len(bad)}개 제외)' if bad else ''} → "
            f"{final}  {fr}프레임 / {du:.1f}s / {os.path.getsize(final) / 1e6:.1f}MB")
    finally:
        # 임시 파일은 성공·실패 어느 쪽이든 남기지 않는다(조각과 잠금만 남는다)
        for n in os.listdir(session_dir) if os.path.isdir(session_dir) else []:
            if n.startswith('.norm-') or n == '.concat.txt' or n.startswith('.merging.'):
                try:
                    os.remove(os.path.join(session_dir, n))
                except OSError:
                    pass
        try:
            os.remove(lock)
        except OSError:
            pass

    if not keep_parts:
        shutil.rmtree(session_dir, ignore_errors=True)
        parent = os.path.dirname(session_dir)
        try:
            os.rmdir(parent)                 # parts/ 가 비었으면 지운다(다른 세션이 있으면 실패 = 그대로)
        except OSError:
            pass
    return final


def merge_orphans(base_dir, dry_run=False, log=print):
    """<base_dir>/parts/ 의 세션 중 ★녹화 중이 아닌 것★ 을 전부 합친다. 합친 개수."""
    pdir = os.path.join(base_dir, PARTS_DIR)
    if not os.path.isdir(pdir):
        return 0
    n = 0
    for name in sorted(os.listdir(pdir)):
        sdir = os.path.join(pdir, name)
        if not os.path.isdir(sdir):
            continue
        if _pid_alive(os.path.join(sdir, LOCK_RECORDING)):
            log(f"· {name}: 녹화 중 — 건너뛴다")
            continue
        try:
            os.remove(os.path.join(sdir, LOCK_RECORDING))   # 죽은 녹화의 잠금
        except OSError:
            pass
        if merge_session(sdir, dry_run=dry_run, log=log):
            n += 1
    return n


# ══════════════════════════════════════════════════════════════════════════════
#  옛 방식(조각이 맨 위에 섞인 폴더) 정리 — 한 번만 쓰는 이주 도구
# ══════════════════════════════════════════════════════════════════════════════
def _trash(path, log):
    r = subprocess.run(['gio', 'trash', path], capture_output=True, text=True)
    if r.returncode != 0:
        log(f"  ⚠️ 휴지통으로 못 보냈다({path}) — 남겨 둔다: {r.stderr.strip()[-200:]}")
        return False
    return True


def migrate_legacy(base_dir, dry_run=False, log=print):
    """맨 위에 '<세션>.mp4 + <세션>-02.mp4 …' 로 섞인 세션을 최종본 하나로 합치고 원래 조각은
    휴지통으로 보낸다. ★한 파일짜리 세션은 건드리지 않는다★(이미 최종본이다). 합친 개수."""
    groups = {}
    for name in sorted(os.listdir(base_dir)):
        m = LEGACY_RE.match(name)
        if m and os.path.isfile(os.path.join(base_dir, name)):
            groups.setdefault(m.group('session') + m.group('ext'), []).append(
                (int(m.group('n') or 1), name))
    n = 0
    for key, items in groups.items():
        if len(items) < 2:
            continue
        session, ext = os.path.splitext(key)
        items.sort()
        if dry_run:
            log(f"(dry-run) {session}: 옛 조각 {len(items)}개 → {os.path.join(base_dir, session + ext)}")
            n += 1
            continue
        # ① 원래 조각을 parts/<세션>/<세션>-NN 으로 옮긴다(맨 위의 첫 조각 이름이 최종본 자리를 비키게)
        sdir = os.path.join(base_dir, PARTS_DIR, session)
        os.makedirs(sdir, exist_ok=True)
        for i, name in items:
            os.replace(os.path.join(base_dir, name), os.path.join(sdir, f"{session}-{i:02d}{ext}"))
        # ② 합친다 — 조각은 남긴다(③ 에서 휴지통으로)
        final = merge_session(sdir, out_dir=base_dir, keep_parts=True, log=log)
        if not final:
            # 실패 — 원래 자리로 되돌린다(잃는 것도 바뀌는 것도 없게)
            for i, name in items:
                src = os.path.join(sdir, f"{session}-{i:02d}{ext}")
                if os.path.exists(src):
                    os.replace(src, os.path.join(base_dir, name))
            shutil.rmtree(sdir, ignore_errors=True)
            log(f"  ↩️ {session}: 원래 자리로 되돌렸다")
            continue
        # ③ 원래 조각은 휴지통으로(되살릴 수 있게)
        if _trash(sdir, log):
            n += 1
    try:
        os.rmdir(os.path.join(base_dir, PARTS_DIR))
    except OSError:
        pass
    return n


# ══════════════════════════════════════════════════════════════════════════════
#  기본 폴더 · 명령행
# ══════════════════════════════════════════════════════════════════════════════
def default_dirs():
    """이 워크스페이스의 녹화 폴더들 — <ws>/src/*/video + ~/nxde_video (있는 것만)."""
    out = []
    for part in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
        if part.rstrip(os.sep).endswith(os.path.join('install', 'nxde')):
            ws = os.path.dirname(os.path.dirname(part.rstrip(os.sep)))
            src = os.path.join(ws, 'src')
            if os.path.isdir(src):
                for name in sorted(os.listdir(src)):
                    d = os.path.join(src, name, 'video')
                    if os.path.isdir(d):
                        out.append(d)
    home = os.path.join(os.path.expanduser('~'), 'nxde_video')
    if os.path.isdir(home):
        out.append(home)
    return out


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    dry = '--dry-run' in argv
    legacy = '--legacy' in argv
    orphans = '--orphans' in argv
    args = [a for a in argv if not a.startswith('--')]
    log = _log_to(None if dry else LOG_PATH)

    # 녹화 노드가 부르는 꼴 : 세션 폴더 하나 (parts/<세션>)
    if len(args) == 1 and not orphans and not legacy and \
            os.path.basename(os.path.dirname(os.path.abspath(args[0]))) == PARTS_DIR:
        return 0 if merge_session(args[0], keep_parts='--keep' in argv, log=log) else 1

    dirs = args or default_dirs()
    if not dirs:
        print("녹화 폴더를 못 찾았다 — ros2 run nxde video_merge <폴더> 로 준다")
        return 1
    total = 0
    for d in dirs:
        d = os.path.abspath(os.path.expanduser(d))
        if not os.path.isdir(d):
            log(f"· {d} 없음 — 건너뛴다")
            continue
        log(f"📂 {d}")
        total += merge_orphans(d, dry_run=dry, log=log)
        if legacy:
            total += migrate_legacy(d, dry_run=dry, log=log)
    log(f"끝 — 합친 세션 {total}개")
    return 0


if __name__ == '__main__':
    sys.exit(main())
