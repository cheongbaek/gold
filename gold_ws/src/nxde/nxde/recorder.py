#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
recorder.py ― ★프레임 → mp4 조각 → 최종본★ 녹화 엔진 [nxde, 2026-10-18]
════════════════════════════════════════════════════════════════════════════════
nxde/video.py(토픽을 받아 적는 노드)와 white1·white2 traffic_light(디버그 화면을 그린 그
자리에서 적는다)가 ★같이 쓰는 녹화의 단일 소유자★ 다. ROS 를 import 하지 않는다 —
로그는 생성자에 넘긴 함수로 낸다.

    rec = SegmentRecorder(out_dir, prefix='tl', log=say, src='/tl/debug_image')
    rec.feed(frame_bgr)        # 프레임이 올 때마다 (스레드 안전)
    rec.tick()                 # 1초쯤마다 — 진행 로그 · 디스크 감시
    rec.close()                # 끝 — 몇 번 불려도 안전. 마지막 조각을 닫고 합본을 맡긴다

★왜 노드에서 떼어 냈나★ traffic_light 가 그린 3 MB 캔버스를 /tl/debug_image 로 내보내
nxde video 가 받아 적던 구조에서, 그 ★발행(rclpy publish + cv2_to_imgmsg)이 traffic_light
안에서 GIL 을 쥔 시간의 41%★ 였다(py-spy --gil, 2026-10-18). cv2.VideoWriter.write 는 C++ 안에서
GIL 을 놓으므로 같은 프로세스에서 직접 적으면 인지 스레드를 거의 방해하지 않는다
(Python 작업 스레드 속도 : 발행 −10% · 직접 인코딩 −0.8%).

아래 동작은 전부 video.py 에서 그대로 옮겼다(근거 주석도 함께) :
  · fps 실측 — 처음 FPS_PROBE_N 프레임 간격의 중앙값. fps 를 주면 실측을 건너뛴다
  · 시각 기준 칸 — 영상 길이 = 벽시계 (늦으면 직전 프레임 반복, 빠르면 버림, 공백은 MAX_FILL_S 까지)
  · 조각 분할 — SEGMENT_S 마다 닫는다(SIGKILL 을 맞아도 직전 조각은 산다)
  · parts/<세션>/<세션>-NN 에 쓰고, 끝나면 nxde/videomerge 가 <세션>.mp4 한 파일로 합친다
  · 남은 디스크 MIN_FREE_MB 밑이면 스스로 끝낸다 · close() 는 멱등

⚠️ feed() 에 넘긴 배열을 반복용으로 ★참조만★ 들고 있는다(복사하지 않는다). 다음 feed() 전에
   그 배열을 덮어쓰면 반복 칸에 바뀐 그림이 들어간다 — traffic_light 의 CanvasPool(두 장 번갈아)은
   '직전 프레임' 을 다음 그림이 덮기 전에 쓰므로 안전하다.
"""

import os
import shutil
import subprocess
import sys
import threading
import time
from datetime import datetime

import cv2

#  ★[2026-10-16] 조각 → 최종본★ 폴더 이름·잠금의 단일 소유자는 videomerge 다(ROS 를 import 하지 않는 모듈).
from nxde import videomerge


# ══════════════════════════════════════════════════════════════════════════════
#  상수 — 옮겨 온 근거는 각 줄과 video.py 머리말
# ══════════════════════════════════════════════════════════════════════════════
#  코덱 → 컨테이너. 컨테이너를 코덱과 따로 고르면 재생이 안 되는 조합이 나오므로 묶는다.
#    mp4v : 어디서나 열리고 용량이 작다. ★닫아야 재생 가능★ (기본)
#    MJPG : 프레임마다 독립 JPEG → 잘려도 앞부분을 살린다. 용량이 몇 배.
#    XVID : MJPG 보다 작고 avi. 잘림 내성은 mp4v 와 MJPG 사이.
CODEC_EXT = {'mp4v': '.mp4', 'avc1': '.mp4', 'MJPG': '.avi', 'XVID': '.avi'}
DEFAULT_CODEC = 'mp4v'

FPS_PROBE_N     = 30     # 실측에 쓸 프레임 수(30fps 면 약 1초)
FPS_PROBE_MAX_S = 4.0    # 그 안에 못 모으면 모은 것으로 확정한다(저프레임 카메라 대비)
FPS_MIN, FPS_MAX = 1.0, 120.0   # 실측이 이 범위를 벗어나면 못 믿는다 → 폴백
FPS_FALLBACK    = 30.0   # camera_launch 의 framerate 와 같은 값

# ══════════════════════════════════════════════════════════════════════════════
#  ★★ [2026-09-13] 세그먼트 분할 — SIGKILL 을 맞아도 직전 조각은 살아남는다 ★★
# ══════════════════════════════════════════════════════════════════════════════
#  ★mp4 는 인덱스(moov atom)를 파일 ★끝★ 에 쓴다★ — release() 가 안 불리면 파일
#  전체가 재생 불가가 된다("moov atom not found"). 신호 핸들러·atexit·finally 삼중
#  방어가 있지만 ★SIGKILL 은 그 셋을 전부 건너뛴다★. 2026-09-13 실측 : video/ 의 mp4
#  4개 중 ★3개가 깨져 있었다★(236M · 181M · 55M → moov 없음 / 8.5M → 정상).
#  ★그래서 파일을 잘게 나눈다.★ 닫는 순간 그 조각은 ★완결된 mp4★ 이므로, 그 뒤에
#  SIGKILL 을 맞아도 잃는 것은 마지막 조각(최대 SEGMENT_S 초)뿐이다.
#  ⚠️ 0 으로 두면 분할하지 않는다 — 그때는 SIGKILL 에 전부 잃는다.
SEGMENT_S = 120.0        # [s] 이 주기마다 파일을 닫고 새로 연다. 0 = 분할 안 함

# ══════════════════════════════════════════════════════════════════════════════
#  ★★ [2026-10-16] 시각 기준으로 넣는다 — 영상 길이 = 벽시계 ★★
# ══════════════════════════════════════════════════════════════════════════════
#  fps 는 처음 FPS_PROBE_N 프레임으로 한 번 재고 파일에 박힌다. 그런데 실제 도착률은
#  그 뒤에 바뀐다 — 디버그 화면은 기동 직후 29.2 fps 로 쟀는데 평균 20.8 fps 로 들어와
#  ★48.5초 녹화가 34.5초짜리 영상★ 이 됐다(1.4배 빨리 재생 — record CSV 와 시각이 안 맞는다).
#  그래서 출력의 k 번째 프레임을 '조각 시작 + k/fps' 시점으로 정하고, 프레임이 오면
#  그 시점의 칸에 넣는다: 늦게 오면 ★직전 프레임을 반복★ 해 빈 칸을 채우고, 빨리 오면
#  이미 찬 칸이므로 버린다. 반복 프레임은 mpeg4 에서 거의 0 바이트다.
#  공백이 MAX_FILL_S 를 넘으면(카메라 두절) 그만큼만 채우고 나머지는 경고로 남긴다 —
#  한 번에 수천 장을 인코딩하느라 멈추지 않게.
MAX_FILL_S = 5.0         # [s] 한 공백에서 반복으로 채우는 최대 길이

LOG_PERIOD_S = 10.0      # 진행 상황 로그 주기
MIN_FREE_MB = 500.0      # 남은 디스크가 이 밑이면 스스로 종료
FREE_CHECK_PERIOD_S = 5.0


def _default_log(text, warn=False):
    print(('[WARN] ' if warn else '[INFO] ') + text, flush=True)


class SegmentRecorder:
    """프레임을 시각 기준 칸에 넣어 mp4 조각으로 적고, 끝나면 한 파일로 합친다."""

    def __init__(self, out_dir, prefix='cam', log=None, src='',
                 fps=0.0, scale=1.0, codec=DEFAULT_CODEC,
                 segment_s=SEGMENT_S, merge=True, keep_parts=False):
        """out_dir 는 이미 풀린 절대경로. log(text, warn=False) 로 말한다.
        src 는 로그·경고에 쓰는 프레임 출처 이름('/tl/debug_image' 같은 것)."""
        self._log = log or _default_log
        self.src = src or '프레임'
        self.fps_p = float(fps)
        self.scale = float(scale)
        self.codec = str(codec)
        if self.codec not in CODEC_EXT:
            self._log(f"codec '{self.codec}' 은 모르는 값 — '{DEFAULT_CODEC}' 로 진행한다 "
                      f"(아는 것: {', '.join(CODEC_EXT)})", True)
            self.codec = DEFAULT_CODEC
        if not (0.05 <= self.scale <= 1.0):
            self._log(f"scale {self.scale} → 1.0 으로 클램프(0.05~1.0)", True)
            self.scale = 1.0

        os.makedirs(out_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')   # record.py 와 같은 규칙
        #  ★[2026-10-16] 맨 위에는 최종본만★ 조각은 parts/<세션>/<세션>-NN 에 쓰고, 닫은 뒤
        #  videomerge 가 <out_dir>/<세션>.mp4 한 파일로 합친다(그 모듈 머리말).
        self.ext = CODEC_EXT[self.codec]
        self.out_dir = out_dir
        self.session = f"{prefix or 'cam'}-{stamp}"
        self.parts_dir = os.path.join(out_dir, videomerge.PARTS_DIR, self.session)
        self.final_path = os.path.join(out_dir, self.session + self.ext)
        self.merge = bool(merge)
        self.keep_parts = bool(keep_parts)
        self.path = os.path.join(self.parts_dir, f"{self.session}-01{self.ext}")

        # ── 상태 ──
        self.writer = None
        self.size   = None            # (w, h) 실제로 쓰는 크기
        self.n_written = 0            # 파일에 쓴 프레임(반복 포함) — 영상 길이 = 이것 ÷ fps
        self.n_dropped = 0            # 크기 불일치·인코딩 실패로 못 쓴 프레임
        self.n_dup = 0                # 빈 칸을 채우려고 반복해 쓴 프레임 [2026-10-16]
        self.n_skip = 0               # fps 보다 빨리 와서 버린 프레임 [2026-10-16]
        self.lost_s = 0.0             # MAX_FILL_S 를 넘어 못 채운 공백 [s]
        self._last_frame = None       # 반복에 쓸 직전 프레임
        self.t_first = None           # 첫 ★기록된★ 프레임 시각
        self.t_last_frame = 0.0       # 마지막 수신 시각(두절 경고용 — 부르는 쪽이 본다)
        self.fps_used = None
        self._probe_t = []            # 실측용 도착 시각
        self._closed = False
        self.stop_reason = None       # 'disk' 면 디스크 부족으로 스스로 끝냈다
        # ★RLock 이어야 한다 (plain Lock 이면 교착)★ 신호 핸들러가 close() 를 ★feed 와 같은
        #   스레드에서★ 부를 수 있다 — 파이썬 신호 핸들러는 바이트코드 사이에서 실행되므로,
        #   feed 가 이 락을 쥔 순간 신호가 들어오면 close() 가 같은 스레드에서 같은 락을 다시
        #   잡으려 한다. Lock 이면 거기서 영구 정지하고 ★파일이 닫히지 않는다★.
        #   (close() 뒤 feed 로 돌아와도 _write 가 writer None 을 보고 멈춘다 [2026-10-16].)
        self._lock = threading.RLock()
        self._t_log = time.time()
        self._t_free = time.time()
        self._t_warn = {}             # 프레임마다 날 수 있는 경고의 마지막 시각(간격 제한)
        #  ── 세그먼트 상태 [2026-09-13] ──
        self.segment_s = max(0.0, float(segment_s))
        self._seg_i = 1               # 지금 쓰고 있는 조각 번호(1 부터)
        self._seg_t0 = 0.0            # 이 조각을 연 시각
        self._seg_frames = 0          # 이 조각에 쓴 프레임 수
        self._seg_done = 0            # 완결된 조각 수(로그용)

    # ══════════════════════════════════════════════════════════════════════════
    #  시작 안내 · 남은 세션 합치기
    # ══════════════════════════════════════════════════════════════════════════
    def describe(self, why=''):
        """시작할 때 한 번 — 어디에 무엇을 적는지."""
        self._log(f"🎥 녹화 준비 — {self.src} → 최종본 {self.final_path}")
        self._log(
            f"   저장 위치 : {self.out_dir}" + (f"  ({why})" if why else "")
            + f" · 녹화 중 조각은 {videomerge.PARTS_DIR}/{self.session}/"
            + (" — 끝나면 한 파일로 합친다" if self.merge else " — ★합치지 않는다(merge:=false)★"))
        #  ★[2026-10-16] 지난번에 못 합친 세션(SIGKILL·전원 차단)을 마저 합친다★ — 별도 프로세스.
        #  녹화 중인 세션(다른 녹화)은 .recording 의 pid 로 가려 건드리지 않는다.
        if self.merge and os.path.isdir(os.path.join(self.out_dir, videomerge.PARTS_DIR)):
            if self._spawn_merge('--orphans', self.out_dir):
                self._log(f"   ♻️ {videomerge.PARTS_DIR}/ 에 남은 세션을 합치도록 맡겼다 "
                          f"(로그 {videomerge.LOG_PATH})")
        self._log(
            f"   코덱 {self.codec} / scale {self.scale:g} / fps "
            f"{'실측(첫 %d프레임은 파일에 안 들어간다)' % FPS_PROBE_N if self.fps_p <= 0 else f'{self.fps_p:g} 고정'}"
            f" | ★Ctrl-C 로 끝내면 그 시점까지 저장된다★")

    @property
    def closed(self):
        return self._closed

    @property
    def recording(self):
        return self.writer is not None

    # ══════════════════════════════════════════════════════════════════════════
    #  종료 — ★몇 번 불려도 안전해야 한다★ (신호·atexit·finally 가 겹칠 수 있다)
    # ══════════════════════════════════════════════════════════════════════════
    def close(self):
        """VideoWriter 를 닫고 합본을 맡긴다. 돌려주는 값 = 닫은 파일이 있었는가."""
        with self._lock:
            if self._closed:
                return False
            self._closed = True
            w = self.writer
            self.writer = None
        if w is None:
            self._log(f"⚠️ 프레임을 한 장도 못 받아 파일을 만들지 않았다 — {self.src} 확인", True)
            return False
        try:
            w.release()
        except Exception as e:
            self._log(f"❌ 파일 닫기 실패: {e} — 파일이 재생되지 않을 수 있다", True)
            return False
        vid_s = self.video_seconds()
        wall_s = (time.time() - self.t_first) if self.t_first else 0.0
        mb = self.size_mb()
        #  ★[2026-10-16] 세션 전체로 보고한다★ 조각은 parts/<세션>/ 에 있고 곧 한 파일로 합쳐진다.
        self._log(
            f"✅ 녹화 끝 — 조각 {self._seg_i}개 · {self.n_written}프레임 / ★영상 {vid_s:.1f}초★ / "
            f"fps {self.fps_used:g} (마지막 조각 {mb:.1f}MB)"
            + (f" / 기록 실패 {self.n_dropped}" if self.n_dropped else ""))
        #  ★[2026-10-16] 시각 맞춤 결과★ 반복·버림은 정상 동작이다(수신률이 fps 와 다를 뿐).
        #  수신률 = 실제로 받은 프레임 ÷ 벽시계 — 반복분을 빼야 '몇 fps 로 들어왔나' 가 된다.
        got = self.n_written - self.n_dup
        self._log(
            f"   시각 맞춤 : 반복 {self.n_dup} / 버림 {self.n_skip} 프레임 "
            f"(실수신 평균 {got / wall_s if wall_s > 0.5 else 0.0:.1f}fps)")
        # ★영상이 벽시계보다 짧으면 MAX_FILL_S 를 넘는 공백이 있었다★ (카메라 두절 등)
        if self.lost_s > 0.5 or (wall_s > 0.5 and abs(wall_s - vid_s) > max(1.0, 0.05 * wall_s)):
            self._log(
                f"   ⚠️ 벽시계 {wall_s:.1f}초인데 영상은 {vid_s:.1f}초 — "
                f"{self.lost_s:.1f}초는 {MAX_FILL_S:g}초를 넘는 공백이라 채우지 않았다. "
                f"카메라·인지 두절을 의심할 것", True)
        #  ★[2026-10-16] 잠금을 풀고 합본을 맡긴다★ 별도 프로세스라 부르는 쪽의 종료가 늦어지지
        #  않는다(nxde kill 의 2 s 유예 안). 합본이 죽어도 조각은 남고 다음 기동 때 다시 합친다.
        try:
            os.remove(os.path.join(self.parts_dir, videomerge.LOCK_RECORDING))
        except OSError:
            pass
        if self.merge and self._spawn_merge(self.parts_dir, *(['--keep'] if self.keep_parts else [])):
            self._log(f"   📦 최종본 → {self.final_path}  (합본은 백그라운드 — 로그 {videomerge.LOG_PATH})")
        else:
            self._log(f"   조각 → {self.parts_dir}  (합치려면 ros2 run nxde video_merge)")
        return True

    def _warn_every(self, key, period, text):
        """같은 경고를 period 초에 한 번만 — 프레임마다 나는 경고가 로그를 덮지 않게."""
        now = time.time()
        if now - self._t_warn.get(key, 0.0) >= period:
            self._t_warn[key] = now
            self._log(text, True)

    def _spawn_merge(self, *args):
        """videomerge 를 ★별도 프로세스★ 로 띄운다(자기 세션 — 런치의 Ctrl+C·nxde kill 이 닿지 않는다).
        ★신호 핸들러 안에서도 불린다★ — 기다리지 않고 바로 돌아온다."""
        try:
            os.makedirs(os.path.dirname(videomerge.LOG_PATH), exist_ok=True)
            with open(videomerge.LOG_PATH, 'a') as err:     # import 실패 같은 것도 로그에 남게
                subprocess.Popen([sys.executable, '-m', 'nxde.videomerge', *args],
                                 stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=err,
                                 start_new_session=True, close_fds=True)
            return True
        except Exception as e:
            self._log(f"⚠️ 합본을 띄우지 못했다({e})", True)
            return False

    # ══════════════════════════════════════════════════════════════════════════
    #  기록
    # ══════════════════════════════════════════════════════════════════════════
    def feed(self, frame, now=None):
        """BGR 프레임 한 장. 스레드 안전. 돌려주는 값 = 이 프레임을 파일에 넣었는가."""
        if now is None:
            now = time.time()
        self.t_last_frame = now
        if self._closed:
            return False
        if self.scale != 1.0:
            frame = cv2.resize(
                frame, (max(1, int(frame.shape[1] * self.scale)),
                        max(1, int(frame.shape[0] * self.scale))),
                interpolation=cv2.INTER_AREA)

        with self._lock:
            if self._closed:
                return False                # 이미 닫혔다(종료 중) — 더 쓰지 않는다
            if self.writer is None:
                if not self._open_writer(frame, now):
                    return False            # 실측 중이거나 열기 실패
            # ★크기가 바뀌면 writer 를 다시 열 수 없다★ 맞춰서 넣는다.
            #   (조용히 깨진 파일이 되는 것보다 리사이즈해서 이어 적고 경고를 남기는 편이 낫다)
            h, w = frame.shape[:2]
            if (w, h) != self.size:
                self._warn_every('size', 10.0,
                                 f"프레임 크기 변경 {w}x{h} → {self.size[0]}x{self.size[1]} 로 맞춘다")
                frame = cv2.resize(frame, self.size, interpolation=cv2.INTER_AREA)
            # ★[2026-10-16] 시각 기준 칸★ 상수절 MAX_FILL_S 참고.
            k = int((now - self._seg_t0) * self.fps_used)   # 이 프레임이 들어갈 칸
            if k < self._seg_frames:
                self.n_skip += 1            # 이 칸은 이미 찼다 — fps 보다 빨리 왔다
                return False
            gap = k - self._seg_frames      # 직전 프레임 뒤로 비어 있는 칸 수
            if gap > 0 and self._last_frame is not None:
                fill = min(gap, int(MAX_FILL_S * self.fps_used))
                for _ in range(fill):
                    if not self._write(self._last_frame):
                        return False
                    self.n_dup += 1
                if gap > fill:
                    # 못 채운 만큼 기준을 당긴다 — 이후 프레임이 끝없이 '늦은' 것으로 보이지 않게
                    lost = (gap - fill) / self.fps_used
                    self.lost_s += lost
                    self._seg_t0 += lost
                    self._log(
                        f"⚠️ {gap / self.fps_used:.1f}초 공백 — {MAX_FILL_S:g}초만 채우고 "
                        f"{lost:.1f}초는 건너뛴다(그만큼 영상이 벽시계보다 짧아진다)", True)
            if not self._write(frame):
                return False
            self._last_frame = frame
            if self.t_first is None:
                self.t_first = now
            #  ★[2026-09-13] 세그먼트 롤오버★ 프레임을 쓴 직후가 조각을 끊기에 가장 안전한
            #  지점이다(writer 가 반쯤 쓴 프레임을 들고 있지 않다). _lock 을 이미 쥐고 있다.
            if self.segment_s > 0.0 and (now - self._seg_t0) >= self.segment_s:
                self._roll_segment()
            return True

    def _write(self, frame) -> bool:
        """한 칸 쓴다. ★_lock 을 잡은 채로 불린다.★ 못 썼으면 False.

        ★self.writer 를 매번 다시 읽는다★ — 신호 핸들러가 feed 도중(같은 스레드, 바이트코드
        사이)에 close() 로 writer 를 떼어 갈 수 있다."""
        w = self.writer
        if w is None or self._closed:
            return False
        try:
            w.write(frame)
        except Exception as e:
            self.n_dropped += 1
            self._warn_every('write', 5.0, f"프레임 기록 실패: {e}")
            return False
        self.n_written += 1
        self._seg_frames += 1
        return True

    def _open_writer(self, frame, now) -> bool:
        """→ 열었으면 True. 실측이 아직 안 끝났으면 False(그 프레임은 버린다).
        ★_lock 을 잡은 채로 불린다.★"""
        if self.fps_p > 0:
            fps = self.fps_p
        else:
            # ── fps 실측 ── 도착 간격의 ★중앙값★ 을 쓴다. 평균은 기동 직후 한 번의
            #    긴 간격(드라이버 워밍업)에 끌려가고, 중앙값은 그것에 반응하지 않는다.
            self._probe_t.append(now)
            n = len(self._probe_t)
            span = now - self._probe_t[0]
            if n < FPS_PROBE_N and span < FPS_PROBE_MAX_S:
                return False                # 아직 모으는 중
            if n < 2 or span <= 0.0:
                fps = FPS_FALLBACK
            else:
                d = sorted(self._probe_t[i + 1] - self._probe_t[i]
                           for i in range(n - 1))
                med = d[len(d) // 2]
                fps = (1.0 / med) if med > 1e-6 else FPS_FALLBACK
            if not (FPS_MIN <= fps <= FPS_MAX):
                self._log(f"실측 fps {fps:.2f} 가 허용범위({FPS_MIN}~{FPS_MAX}) 밖 — "
                          f"{FPS_FALLBACK:g} 로 진행한다", True)
                fps = FPS_FALLBACK
            self._log(f"📐 fps 실측 {fps:.2f} ({n}프레임 / {span:.2f}초) — 이 값으로 파일을 연다")

        h, w = frame.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*self.codec)
        #  ★[2026-10-16] 세션 폴더와 녹화 잠금★ 첫 프레임을 쓸 때 만든다(프레임이 없으면 폴더도 없다).
        #  잠금(pid)이 살아 있는 동안 videomerge 의 orphan 정리가 이 세션을 건드리지 않는다.
        try:
            os.makedirs(self.parts_dir, exist_ok=True)
            with open(os.path.join(self.parts_dir, videomerge.LOCK_RECORDING), 'w') as f:
                f.write(str(os.getpid()))
        except OSError as e:
            self._log(f"❌ 조각 폴더를 만들 수 없다: {self.parts_dir} ({e})", True)
            self._closed = True
            return False
        #  ★[2026-10-16] 첫 조각부터 반올림한 같은 값으로 연다★ 조각마다 OpenCV 가 다른 시간 단위를
        #  고르면 합칠 때 길이가 틀어진다(videomerge 가 맞춰 주지만 애초에 같게 쓴다).
        fps = round(float(fps), 3)
        writer = cv2.VideoWriter(self.path, fourcc, fps, (w, h))
        if not writer.isOpened():
            # ★여기서 죽지 않는다★ 코덱이 없는 환경일 수 있다. 사유를 정확히 알려 준다.
            self._log(
                f"❌ 파일을 열 수 없다: {self.path}\n"
                f"   코덱 '{self.codec}' 을 OpenCV 가 못 쓰는 환경일 수 있다 — "
                f"MJPG (avi) 로 다시 시도해 볼 것.\n"
                f"   쓰기 권한·경로도 확인: {os.path.dirname(self.path)}", True)
            self._closed = True             # 더 시도하지 않는다(로그 폭주 방지)
            return False
        self.writer = writer
        self.size = (w, h)
        self.fps_used = fps
        self._seg_t0 = now
        self._seg_frames = 0
        seg = (f"  (조각당 {self.segment_s:.0f}초씩 나눠 적는다 — "
               f"중간에 죽어도 직전 조각까지는 남는다)"
               if self.segment_s > 0 else "  (분할 없음 — 끝까지 한 파일)")
        self._log(f"🔴 녹화 시작 {w}x{h} @ {self.fps_used:g}fps → "
                  f"{os.path.basename(self.path)}{seg}")
        return True

    # ══════════════════════════════════════════════════════════════════════════
    #  세그먼트 분할 [2026-09-13]
    # ══════════════════════════════════════════════════════════════════════════
    def _segment_path(self, i):
        """i 번째 조각의 경로 = parts/<세션>/<세션>-NN [2026-10-16]. 최종본 이름(<세션>.mp4)은
        종전 첫 조각 이름과 같아 record CSV 와 시각을 맞추는 규칙(tl-<시각>.mp4)이 그대로다."""
        return os.path.join(self.parts_dir, f"{self.session}-{i:02d}{self.ext}")

    def _roll_segment(self):
        """지금 조각을 ★완결★ 하고 다음 조각을 연다. ★_lock 을 쥔 채로 불린다.★

        ★close() 를 재사용하지 않는다★ close() 는 `_closed` 를 세워 '녹화 끝' 을
        뜻하고 멱등이라, 롤오버에 쓰면 다음 조각을 열 수 없게 된다.
        """
        w = self.writer
        if w is None:
            return
        self.writer = None              # 먼저 떼어 둔다(죽은 writer 를 안 보게)
        done_path = self.path
        frames = self._seg_frames
        try:
            w.release()
        except Exception as e:
            self._log(f"❌ 조각 닫기 실패: {e}", True)
            return
        self._seg_done += 1
        try:
            mb = os.path.getsize(done_path) / 1e6
        except OSError:
            mb = 0.0
        vid_s = (frames / self.fps_used) if self.fps_used else 0.0
        self._log(f"💾 조각 {self._seg_i:02d} 저장 — {os.path.basename(done_path)}  "
                  f"{frames}프레임 / {vid_s:.1f}초 / {mb:.1f}MB")
        # ── 다음 조각을 연다 ── fps·크기는 ★재측정하지 않는다★
        self._seg_i += 1
        self.path = self._segment_path(self._seg_i)
        fourcc = cv2.VideoWriter_fourcc(*self.codec)
        writer = cv2.VideoWriter(self.path, fourcc, float(self.fps_used), self.size)
        if not writer.isOpened():
            self._log(f"❌ 다음 조각을 열 수 없다: {self.path} — 녹화를 여기서 끝낸다", True)
            self._closed = True
            return
        self.writer = writer
        #  ★[2026-10-16] 시간축을 잇는다★ 다음 칸은 직전 조각의 마지막 칸 바로 뒤 시각이다.
        self._seg_t0 += frames / self.fps_used
        self._seg_frames = 0

    # ══════════════════════════════════════════════════════════════════════════
    #  주기 점검 — 진행 상황 · 디스크
    # ══════════════════════════════════════════════════════════════════════════
    def tick(self, now=None):
        """1초쯤마다 부른다. 디스크가 모자라 스스로 닫았으면 True."""
        if now is None:
            now = time.time()
        # ── 진행 상황 ──
        if self.writer is not None and now - self._t_log >= LOG_PERIOD_S:
            self._t_log = now
            vid_s = self.video_seconds()
            mb = self.size_mb()
            # ★MB/분은 '영상 1분당' 이다★ 벽시계로 나누면 프레임이 안 오는 구간에서
            #   값이 0 으로 수렴해 '용량이 안 늘어난다'는 잘못된 인상을 준다.
            rate = (mb / vid_s * 60.0) if vid_s > 0.5 else 0.0
            self._log(
                f"🎥 {self.n_written}프레임 / 영상 {vid_s:.0f}초 / {mb:.1f}MB "
                f"(영상 1분당 {rate:.0f}MB) / 반복 {self.n_dup} · 버림 {self.n_skip}"
                + (f" / 기록 실패 {self.n_dropped}" if self.n_dropped else ""))

        # ── 디스크 ── ★가득 채우면 주행 기록까지 못 쓴다 → 녹화를 포기한다★
        if now - self._t_free >= FREE_CHECK_PERIOD_S and not self._closed:
            self._t_free = now
            try:
                free_mb = shutil.disk_usage(self.out_dir).free / 1e6
            except OSError:
                return False
            if free_mb < MIN_FREE_MB:
                self._log(
                    f"🛑 남은 디스크 {free_mb:.0f}MB < {MIN_FREE_MB:.0f}MB — "
                    f"녹화를 여기서 끝낸다(파일은 정상 종료). 디스크를 채우면 주행 "
                    f"기록 CSV·로그까지 못 쓰게 되므로 녹화를 잃는 쪽을 택한다", True)
                self.stop_reason = 'disk'
                self.close()
                return True
        return False

    def size_mb(self):
        try:
            return os.path.getsize(self.path) / 1e6
        except OSError:
            return 0.0

    def video_seconds(self):
        """★파일의 실제 재생 길이★ = 기록한 프레임 수 ÷ 파일에 박힌 fps.
        벽시계 경과를 쓰면 안 된다 — 프레임이 안 오는 동안에도 시간은 흐른다."""
        if not self.fps_used:
            return 0.0
        return self.n_written / self.fps_used
