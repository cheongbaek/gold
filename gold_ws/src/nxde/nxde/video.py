#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
video.py ― 인지 카메라 화면 녹화 [nxde]
════════════════════════════════════════════════════════════════════════════════
    ros2 run nxde video

차선·신호등 인지가 보는 그 화면(`/image_raw`)을 노드가 도는 동안 파일로 적는다.
★Ctrl-C 로 끝내면 그 시점까지의 영상이 재생 가능한 상태로 닫힌다.★

    저장 위치 : <nxde 패키지 루트(setup.py 가 있는 곳)>/video/
    파일 이름 : cam-<날짜>_<시각>.mp4      예) cam-20260818_174233.mp4

────────────────────────────────────────────────────────────────────────────────
 ★무엇을 녹화하는가 — '기본 처리만 된 화면' 이 정확히 무엇인지★
────────────────────────────────────────────────────────────────────────────────
★인지 노드가 받는 프레임을 그대로 적는다.★ 구체적으로:

  · 토픽      : `/image_raw` (usb_cam 이 V4L2 에서 받아 내보내는 것)
  · 변환      : `imgmsg_to_cv2(desired_encoding='bgr8')`
                ★white1/traffic_light.py 의 cb_image 와 같은 한 줄이다★ — 그래서
                여기 적히는 그림이 인지가 실제로 본 그림과 같다.
  · 더하지 않는 것 : ★YOLO 박스·ROI 선·정지선 폴리곤·FPS/STATE HUD 를 안 그린다★
                traffic_light 의 'Traffic Light' 창은 그 오버레이가 얹힌 ★디버그
                화면★ 이고, 여기서 원하는 것은 판정의 ★입력★ 이다. 오버레이가 필요하면
                그 창을 화면 녹화하는 것이 맞고, 이 노드가 할 일이 아니다.

★어안(fisheye) 왜곡보정에 대하여 — ★여기 적히는 것은 보정 전 원본이다★
  ★[2026-08-19 정정]★ 파이프라인에 왜곡보정이 들어왔다. 다만 ★토픽이 아니라 노드
  안에서★ 한다: white1/camera_model.py 가 계수를 들고 있고, 인지 노드(traffic_light,
  앞으로 붙을 차선 인지)가 프레임을 받은 직후 한 번 편다.
  `/image_raw` 는 여전히 ★usb_cam 이 준 원본★ 이다(camera_launch 가 `camera_info_url`
  을 주지 않는 이유가 그것으로 바뀌었다) — 그래서 ★이 노드는 고칠 것이 없고, 녹화된
  mp4 에 어안이 남아 있는 것이 정상이다★ (지시사항 — 원본 촬영에는 보정이 필요 없다).

  ⚠️ 그 대신 ★이제 '녹화된 그림'과 '인지가 판정한 그림'이 한 겹 다르다★ 인지는 편
    그림을 보고, 여기 적히는 것은 펴기 전이다. 영상으로 오검출을 따질 때 이 차이를
    감안한다(원본에서 화면 가장자리로 갈수록 휘어 보이는 것은 왜곡이지 오검출이
    아니다). 인지가 본 그대로가 필요하면 traffic_light 의 디버그 창을 화면 녹화한다.

────────────────────────────────────────────────────────────────────────────────
 ★왜 /dev/video2 를 직접 열지 않고 토픽을 받는가★
────────────────────────────────────────────────────────────────────────────────
두 가지 이유가 있고, 둘 다 결정적이다.

  ① ★V4L2 장치는 두 번 열 수 없다.★ usb_cam 이 스트리밍 중인 장치를 이 노드가
     또 열면 둘 중 하나가 실패한다 — 그런데 실패하는 쪽이 usb_cam 이면 ★신호등
     인지가 죽는다★. 녹화 도구가 주행 기능을 끄는 일은 있어서는 안 된다.
  ② ★토픽을 받으면 '인지가 실제로 받은 프레임'이 적힌다.★ 장치를 따로 열면 노출·
     화이트밸런스가 다른 별개 스트림이 되어, 영상과 판정이 서로 다른 그림이 된다.

그래서 카메라 장치 선택(ports.py 의 video2 → video0 순서)·해상도·노출은 전부
usb_cam 쪽 설정이고 이 노드는 관여하지 않는다. ★usb_cam 이 안 떠 있으면 이 노드는
프레임을 못 받고 그렇다고 경고한다★(무엇을 확인해야 하는지 함께 찍는다).

────────────────────────────────────────────────────────────────────────────────
 ★재생 속도 — fps 를 실측하는 이유★
────────────────────────────────────────────────────────────────────────────────
VideoWriter 의 fps 는 ★파일을 열 때 한 번 정해지고 바뀌지 않는다★. 이 값이 실제
도착률과 다르면 영상이 빠르거나 느리게 재생되고, 그러면 ★"몇 초에 무슨 일이 있었나"
를 영상으로 따질 수 없게 된다★ — 로그(CSV)와 시각을 맞추는 것이 이 녹화의 주 용도인데
그것이 깨진다.

camera_launch 는 usb_cam 에 `framerate: 30.0` 을 주지만 ★실제 도착률은 그보다 낮을 수
있다★(USB 대역·노출시간·CPU). 그래서 기본값은 ★실측★ 이다:
  · `fps:=0`(기본) : 처음 FPS_PROBE_N 프레임의 도착 간격으로 fps 를 재고 그 값으로 연다.
    ⚠️ ★그 구간(약 1초)은 파일에 들어가지 않는다★ — 아직 파일을 못 열었기 때문이다.
       프레임을 쌓아 두었다가 나중에 쓰는 방법도 있지만, 1080p BGR 한 장이 6.2MB 라
       30장이면 190MB 다. 앞 1초와 190MB 를 맞바꾸지 않는다.
  · `fps:=30` 처럼 명시하면 실측을 건너뛰고 ★첫 프레임부터★ 적는다.
    시작 순간이 중요한 시험(출발 직후 등)에서는 이쪽을 쓴다.

────────────────────────────────────────────────────────────────────────────────
 ★디스크 — 이 노드가 시스템을 망가뜨리지 않게★
────────────────────────────────────────────────────────────────────────────────
1080p 는 크다. mp4v 기준 대략 ★분당 20~60MB★ 이고, 코스를 여러 번 돌면 GB 단위가 된다.
  · 진행 상황을 LOG_PERIOD_S 마다 찍는다(프레임 수·경과·파일 크기·실측 fps).
  · ★남은 디스크가 MIN_FREE_MB 밑으로 내려가면 스스로 녹화를 끝낸다★(파일은 정상
    종료로 닫는다). 디스크를 가득 채우면 주행 기록 CSV·로그까지 못 쓰게 되므로,
    녹화를 잃는 쪽이 언제나 낫다.
  · 용량을 줄이려면 `scale:=0.5`(가로세로 절반 = 용량 약 1/4) 를 쓴다. 판정 입력이
    아니라 '사람이 보고 되짚는 용도'라면 절반으로도 충분하다.

⚠️ ★인코딩은 CPU 를 쓴다.★ traffic_light 의 YOLO 는 GPU(cuda:0)라 직접 다투지는
   않지만, CPU 가 모자라면 `/image_raw` 구독이 밀려 인지 쪽 프레임률이 떨어질 수 있다.
   실차에서 인지 FPS 가 눈에 띄게 떨어지면 `scale` 을 낮추거나 녹화를 껐다 켠다.

⚠️ ★강제 종료(kill -9 · 전원 차단)는 파일을 살릴 수 없다.★ mp4 는 닫을 때 색인을
   쓰므로 그 전에 죽으면 재생이 안 된다. Ctrl-C · SIGTERM · 런치 종료는 모두 정상
   처리한다(아래 _install_signal_handlers). 전원까지 걱정되는 시험이라면
   `codec:=MJPG` 로 두면 .avi 로 적히고, 잘려도 앞부분은 대개 살린다(용량은 몇 배).
"""

import atexit
import os
import shutil
import signal
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge


# ══════════════════════════════════════════════════════════════════════════════
#  상수
# ══════════════════════════════════════════════════════════════════════════════
DEFAULT_TOPIC = '/image_raw'
#   white1/camera_launch.py 가 usb_cam 을 이 이름으로 띄운다. traffic_light 의
#   image_topic 기본값도 같다 — ★같은 토픽을 봐야 '인지가 본 화면'이 된다★.

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

LOG_PERIOD_S = 10.0      # 진행 상황 로그 주기
NO_FRAME_WARN_S = 5.0    # 이 시간 동안 프레임이 없으면 경고(usb_cam 확인 안내)
MIN_FREE_MB = 500.0      # 남은 디스크가 이 밑이면 스스로 종료
FREE_CHECK_PERIOD_S = 5.0


def _package_root():
    """setup.py 가 있는 nxde 패키지 루트. 못 찾으면 None.

    ★symlink-install(egg-link) 이면 __file__ 이 소스트리를 가리킨다★ 그래서 여기서
    찾은 루트가 곧 사용자가 말한 'setup.py 가 있는 디렉터리' 다(실측: nxde 는
    egg-link 방식이고 nxde.__file__ 이 src/nxde/nxde/__init__.py 로 풀린다).
    ★symlink 가 아닌 빌드면 install/ 아래가 나온다★ — 거기 쌓으면 재빌드에 날아가므로
    그때는 루트로 인정하지 않고 홈으로 폴백한다(아래 resolve_output_dir).
    white1/paths.py 가 gps_data·ros2bag 위치를 정하는 방식과 같은 태도다.
    """
    here = os.path.dirname(os.path.realpath(__file__))   # .../src/nxde/nxde
    root = os.path.dirname(here)                         # .../src/nxde
    if os.path.isfile(os.path.join(root, 'setup.py')):
        return root
    return None


def resolve_output_dir(explicit: str):
    """→ (경로, 사유문구). 우선순위: 파라미터 → 패키지 루트/video → ~/nxde_video"""
    if explicit:
        return os.path.abspath(os.path.expanduser(explicit)), '파라미터 지정'
    root = _package_root()
    if root and 'install' not in root.split(os.sep):
        return os.path.join(root, 'video'), 'nxde 패키지 루트(setup.py 옆)'
    # 여기로 오면 symlink-install 이 아니다 — install/ 은 재빌드에 지워진다.
    return (os.path.join(os.path.expanduser('~'), 'nxde_video'),
            '★설치본에서 실행 중(재빌드에 지워짐) → 홈으로 폴백★')


# ══════════════════════════════════════════════════════════════════════════════
#  노드
# ══════════════════════════════════════════════════════════════════════════════
class VideoNode(Node):

    def __init__(self):
        super().__init__('video_node')

        self.declare_parameter('image_topic', DEFAULT_TOPIC)
        self.declare_parameter('output_dir', '')
        #   0 = 실측(권장). 명시하면 실측을 건너뛰고 첫 프레임부터 적는다.
        self.declare_parameter('fps', 0.0)
        #   1.0 = 원본 그대로. 0.5 면 가로세로 절반(용량 약 1/4).
        self.declare_parameter('scale', 1.0)
        self.declare_parameter('codec', DEFAULT_CODEC)
        #   파일 이름 앞머리. 여러 대를 동시에 녹화하거나 시험을 구분할 때 바꾼다.
        self.declare_parameter('prefix', 'cam')

        self.topic  = str(self.get_parameter('image_topic').value)
        self.fps_p  = float(self.get_parameter('fps').value)
        self.scale  = float(self.get_parameter('scale').value)
        self.codec  = str(self.get_parameter('codec').value)
        prefix      = str(self.get_parameter('prefix').value) or 'cam'
        if self.codec not in CODEC_EXT:
            self.get_logger().warn(
                f"codec '{self.codec}' 은 모르는 값 — '{DEFAULT_CODEC}' 로 진행한다 "
                f"(아는 것: {', '.join(CODEC_EXT)})")
            self.codec = DEFAULT_CODEC
        if not (0.05 <= self.scale <= 1.0):
            self.get_logger().warn(f"scale {self.scale} → 1.0 으로 클램프(0.05~1.0)")
            self.scale = 1.0

        out_dir, why = resolve_output_dir(str(self.get_parameter('output_dir').value))
        os.makedirs(out_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')   # record.py 와 같은 규칙
        self.path = os.path.join(out_dir, f"{prefix}-{stamp}{CODEC_EXT[self.codec]}")

        # ── 상태 ──
        self.bridge = CvBridge()
        self.writer = None
        self.size   = None            # (w, h) 실제로 쓰는 크기
        self.n_written = 0
        self.n_dropped = 0            # 크기 불일치·인코딩 실패로 못 쓴 프레임
        self.t_first = None           # 첫 ★기록된★ 프레임 시각
        self.t_last_frame = 0.0       # 마지막 수신 시각(두절 경고용)
        self.fps_used = None
        self._probe_t = []            # 실측용 도착 시각
        self._closed = False
        # ★RLock 이어야 한다 (plain Lock 이면 교착)★ SIGTERM 핸들러가 close() 를
        #   ★콜백과 같은 스레드에서★ 부른다 — 파이썬 신호 핸들러는 바이트코드 사이에서
        #   실행되므로, 콜백이 이 락을 쥔 순간 신호가 들어오면 close() 가 같은 스레드에서
        #   같은 락을 다시 잡으려 한다. Lock 이면 거기서 영구 정지하고 ★파일이 닫히지
        #   않는다★ — 정확히 막으려던 실패로 되돌아간다. RLock 은 같은 스레드의 재진입을
        #   허용해 그 경로가 성립하지 않는다.
        #   (close() 뒤 핸들러가 SystemExit 을 올리므로 콜백이 죽은 writer 를 다시 쓰는
        #    일도 없다 — 예외가 콜백을 그 자리에서 중단시킨다.)
        self._lock = threading.RLock()
        self._t_log = time.time()
        self._t_free = time.time()
        self._warned_no_frame = False
        self._stop_reason = None

        # ── 구독 ──
        #   ★QoS 를 usb_cam 에 맞춘다★ BEST_EFFORT / KEEP_LAST / depth=1.
        #   RELIABLE 로 잡으면 ★발행자와 규약이 안 맞아 아예 연결되지 않는다★(프레임이
        #   한 장도 안 온다). white1/traffic_light.py 의 qos_img 와 같은 값이다.
        #   depth=1 인 것도 의도다 — 인코딩이 밀리면 ★최신 프레임만 남기고 버린다★.
        #   큐를 키우면 메모리가 늘고(1080p 한 장 6.2MB) 영상 시각이 실제보다 뒤로 밀린다.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, self.topic, self.cb_image, qos)
        self.create_timer(1.0, self._on_tick)

        self._install_signal_handlers()

        self.get_logger().info(
            f"🎥 nxde video 준비 — {self.topic} → {self.path}")
        self.get_logger().info(
            f"   저장 위치 : {out_dir}  ({why})")
        self.get_logger().info(
            f"   코덱 {self.codec} / scale {self.scale:g} / fps "
            f"{'실측(첫 %d프레임은 파일에 안 들어간다)' % FPS_PROBE_N if self.fps_p <= 0 else f'{self.fps_p:g} 고정'}"
            f" | ★Ctrl-C 로 끝내면 그 시점까지 저장된다★")

    # ══════════════════════════════════════════════════════════════════════════
    #  종료 처리 — ★이게 이 노드의 핵심 요구사항이다★
    # ══════════════════════════════════════════════════════════════════════════
    def _install_signal_handlers(self):
        """SIGTERM 에도 파일을 닫는다.

        Ctrl-C(SIGINT)는 rclpy 가 KeyboardInterrupt 로 올려 주므로 main 의 finally 가
        받는다. 그런데 ★런치가 노드를 내릴 때는 SIGTERM 을 보낸다★ — 그건 기본 동작이
        즉시 종료라서, 그대로 두면 ★파일이 닫히지 않아 재생이 안 된다★.
        atexit 까지 겹쳐 두는 이유는 sys.exit 경로로 빠질 때를 대비한 것이다
        (close() 는 몇 번 불려도 안전하다).
        """
        def _term(signum, _frame):
            self._say(f"신호 {signum} 수신 — 파일을 닫는다")
            self.close()
            raise SystemExit(0)
        try:
            signal.signal(signal.SIGTERM, _term)
        except (ValueError, OSError):
            pass            # 메인 스레드가 아니면 등록할 수 없다 — atexit 이 받는다
        atexit.register(self.close)

    def _say(self, text, warn=False):
        """종료 경로에서도 메시지가 보이게 한다.

        ★close() 는 rclpy 컨텍스트가 이미 내려간 뒤에 불릴 수 있다★(SIGINT →
        KeyboardInterrupt → finally). 그때 get_logger() 로 찍으면 내용은 콘솔에 나오지만
        'Failed to publish log message to rosout: publisher's context is invalid' 가
        따라붙어 ★저장이 실패한 것처럼 보인다★. 컨텍스트가 죽었으면 print 로 낸다.
        """
        if rclpy.ok():
            (self.get_logger().warn if warn else self.get_logger().info)(text)
        else:
            print(('[WARN] ' if warn else '[INFO] ') + text, flush=True)

    def close(self):
        """VideoWriter 를 닫는다. ★몇 번 불려도 안전해야 한다★(신호·atexit·finally
        가 겹칠 수 있다)."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
            w = self.writer
            self.writer = None
        if w is None:
            self._say(f"⚠️ 프레임을 한 장도 못 받아 파일을 만들지 않았다 — "
                      f"{self.topic} 확인. usb_cam 이 떠 있나? "
                      f"(ros2 topic hz {self.topic})", warn=True)
            return
        try:
            w.release()
        except Exception as e:
            self._say(f"❌ 파일 닫기 실패: {e} — 파일이 재생되지 않을 수 있다", warn=True)
            return
        vid_s = self._video_seconds()
        wall_s = (time.time() - self.t_first) if self.t_first else 0.0
        mb = self._size_mb()
        self._say(
            f"✅ 저장 완료 — {os.path.basename(self.path)}  "
            f"{self.n_written}프레임 / ★영상 {vid_s:.1f}초★ / {mb:.1f}MB / "
            f"fps {self.fps_used:g}"
            + (f" / 버린 프레임 {self.n_dropped}" if self.n_dropped else ""))
        # ★둘이 다르면 그만큼 프레임이 안 온 것이다★ 그것 자체가 진단이므로 알려 준다
        #   (usb_cam 이 중간에 죽었거나, 인코딩이 밀려 BEST_EFFORT 로 버려졌거나).
        if wall_s > 0.5 and abs(wall_s - vid_s) > max(1.0, 0.1 * wall_s):
            self._say(
                f"   ⚠️ 벽시계 {wall_s:.1f}초인데 영상은 {vid_s:.1f}초 — "
                f"{wall_s - vid_s:.1f}초분의 프레임이 오지 않았다("
                f"평균 {self.n_written / wall_s:.1f}fps 수신). usb_cam 두절이나 "
                f"인코딩 지연을 의심할 것", warn=True)
        self._say(f"   {self.path}")

    # ══════════════════════════════════════════════════════════════════════════
    #  수신 · 기록
    # ══════════════════════════════════════════════════════════════════════════
    def cb_image(self, msg: Image):
        now = time.time()
        self.t_last_frame = now
        self._warned_no_frame = False

        # ★traffic_light 와 같은 한 줄★ — 그래서 같은 그림이 적힌다
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.n_dropped += 1
            self.get_logger().error(
                f"cv_bridge 변환 실패({msg.encoding}): {e}", throttle_duration_sec=5.0)
            return

        if self.scale != 1.0:
            frame = cv2.resize(
                frame, (max(1, int(frame.shape[1] * self.scale)),
                        max(1, int(frame.shape[0] * self.scale))),
                interpolation=cv2.INTER_AREA)

        with self._lock:
            if self._closed:
                return                      # 이미 닫혔다(종료 중) — 더 쓰지 않는다
            if self.writer is None:
                if not self._open_writer(frame, now):
                    return                  # 실측 중이거나 열기 실패
            # ★크기가 바뀌면 writer 를 다시 열 수 없다★ 맞춰서 넣는다.
            #   (usb_cam 이 도중에 해상도를 바꿀 일은 없지만, 그때 조용히 깨진 파일이
            #    되는 것보다 리사이즈해서 이어 적고 경고를 남기는 편이 낫다)
            h, w = frame.shape[:2]
            if (w, h) != self.size:
                self.get_logger().warn(
                    f"프레임 크기 변경 {w}x{h} → {self.size[0]}x{self.size[1]} 로 맞춘다",
                    throttle_duration_sec=10.0)
                frame = cv2.resize(frame, self.size, interpolation=cv2.INTER_AREA)
            try:
                self.writer.write(frame)
            except Exception as e:
                self.n_dropped += 1
                self.get_logger().error(f"프레임 기록 실패: {e}",
                                        throttle_duration_sec=5.0)
                return
            self.n_written += 1
            if self.t_first is None:
                self.t_first = now

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
                self.get_logger().warn(
                    f"실측 fps {fps:.2f} 가 허용범위({FPS_MIN}~{FPS_MAX}) 밖 — "
                    f"{FPS_FALLBACK:g} 로 진행한다")
                fps = FPS_FALLBACK
            self.get_logger().info(
                f"📐 fps 실측 {fps:.2f} ({n}프레임 / {span:.2f}초) — 이 값으로 파일을 연다")

        h, w = frame.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*self.codec)
        writer = cv2.VideoWriter(self.path, fourcc, float(fps), (w, h))
        if not writer.isOpened():
            # ★여기서 죽지 않는다★ 코덱이 없는 환경일 수 있다. 사유를 정확히 알려 준다.
            self.get_logger().error(
                f"❌ 파일을 열 수 없다: {self.path}\n"
                f"   코덱 '{self.codec}' 을 OpenCV 가 못 쓰는 환경일 수 있다 — "
                f"`codec:=MJPG` (avi) 로 다시 시도해 볼 것.\n"
                f"   쓰기 권한·경로도 확인: {os.path.dirname(self.path)}")
            self._closed = True             # 더 시도하지 않는다(로그 폭주 방지)
            return False
        self.writer = writer
        self.size = (w, h)
        self.fps_used = round(float(fps), 3)
        self.get_logger().info(
            f"🔴 녹화 시작 {w}x{h} @ {self.fps_used:g}fps → "
            f"{os.path.basename(self.path)}")
        return True

    # ══════════════════════════════════════════════════════════════════════════
    #  주기 점검 — 진행 상황 · 프레임 두절 · 디스크
    # ══════════════════════════════════════════════════════════════════════════
    def _on_tick(self):
        now = time.time()

        # ── 프레임이 안 온다 ──
        if (self.t_last_frame == 0.0 or now - self.t_last_frame > NO_FRAME_WARN_S):
            if not self._warned_no_frame:
                self._warned_no_frame = True
                self.get_logger().warn(
                    f"⚠️ {self.topic} 프레임이 {NO_FRAME_WARN_S:.0f}초 이상 없다 — "
                    f"usb_cam 이 떠 있는지, 카메라를 꽂았는지 확인 "
                    f"(ros2 topic hz {self.topic})")

        # ── 진행 상황 ──
        if self.writer is not None and now - self._t_log >= LOG_PERIOD_S:
            self._t_log = now
            vid_s = self._video_seconds()
            mb = self._size_mb()
            # ★MB/분은 '영상 1분당' 이다★ 벽시계로 나누면 프레임이 안 오는 구간에서
            #   값이 0 으로 수렴해 '용량이 안 늘어난다'는 잘못된 인상을 준다.
            rate = (mb / vid_s * 60.0) if vid_s > 0.5 else 0.0
            self.get_logger().info(
                f"🎥 {self.n_written}프레임 / 영상 {vid_s:.0f}초 / {mb:.1f}MB "
                f"(영상 1분당 {rate:.0f}MB)"
                + (f" / 버린 프레임 {self.n_dropped}" if self.n_dropped else ""))

        # ── 디스크 ── ★가득 채우면 주행 기록까지 못 쓴다 → 녹화를 포기한다★
        if now - self._t_free >= FREE_CHECK_PERIOD_S:
            self._t_free = now
            try:
                free_mb = shutil.disk_usage(os.path.dirname(self.path)).free / 1e6
            except OSError:
                return
            if free_mb < MIN_FREE_MB and not self._closed:
                self.get_logger().error(
                    f"🛑 남은 디스크 {free_mb:.0f}MB < {MIN_FREE_MB:.0f}MB — "
                    f"녹화를 여기서 끝낸다(파일은 정상 종료). 디스크를 채우면 주행 "
                    f"기록 CSV·로그까지 못 쓰게 되므로 녹화를 잃는 쪽을 택한다")
                self._stop_reason = 'disk'
                self.close()

    def _size_mb(self):
        try:
            return os.path.getsize(self.path) / 1e6
        except OSError:
            return 0.0

    def _video_seconds(self):
        """★파일의 실제 재생 길이★ = 기록한 프레임 수 ÷ 파일에 박힌 fps.

        벽시계 경과를 쓰면 안 된다 — 프레임이 안 오는 동안에도 시간은 흐르므로
        '9.6초짜리 영상'을 '117초'로 보고하게 된다(실측으로 잡은 오류다).
        재생기에서 보이는 길이와 같은 값이어야 로그·CSV 와 시각을 맞출 수 있다.
        """
        if not self.fps_used:
            return 0.0
        return self.n_written / self.fps_used

    def destroy_node(self):
        self.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        # ★close() 를 destroy_node 앞에 명시적으로 부른다★ 로거가 아직 살아 있어야
        #   '저장 완료' 줄이 화면에 보인다(destroy 뒤에는 안 보인다).
        node.close()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
