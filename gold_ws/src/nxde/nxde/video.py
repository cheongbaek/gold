#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
video.py ― 인지 카메라 화면 녹화 [nxde]
════════════════════════════════════════════════════════════════════════════════
    ros2 run nxde video

차선·신호등 인지가 보는 그 화면(`/image_raw`)을 노드가 도는 동안 파일로 적는다.
★Ctrl-C 로 끝내면 그 시점까지의 영상이 재생 가능한 상태로 닫힌다.★

★[2026-10-18] 녹화 엔진은 nxde/recorder.py 로 옮겼다★ — 이 노드는 토픽을 받아 그 엔진에 넣는다.
  인지 디버그 화면은 이제 traffic_light 가 ★그 자리에서 같은 엔진으로★ 적는다(/tl/debug_image 를
  거치지 않는다 — 3 MB 발행이 인지 스레드와 GIL 을 다퉜다). 이 노드는 원본(/image_raw) 같은
  다른 토픽을 적을 때 쓴다(camera_launch 의 tl_video_topic).

    저장 위치 : output_dir 파라미터 → 없으면 <nxde 패키지 루트(setup.py 가 있는 곳)>/video/
                ★단, 이 워크스페이스는 복사설치(--symlink-install 금지)라 ★언제나★
                ~/nxde_video/ 로 떨어진다★ (resolve_output_dir). 인지 디버그 녹화는
                camera_launch 가 output_dir 을 <src>/white{1,2}/video 로 넘긴다.
    파일 이름 : cam-<날짜>_<시각>.mp4      예) cam-20260818_174233.mp4   ← ★최종본(한 녹화 = 한 파일)★
                [2026-10-16] 녹화 중에는 조각을 parts/<세션>/<세션>-01.mp4 … 에 쓰고, 끝나면
                nxde/videomerge 가 위 이름 한 파일로 합친다(검증 뒤 조각 삭제). 그래서 폴더 맨 위에는
                최종본만 있다. 강제 종료로 못 합친 세션은 다음 기동 때(또는 ros2 run nxde
                video_merge) 합친다 — recorder.py '세그먼트 분할', videomerge.py 머리말.

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
  · ★[2026-10-16] 그 뒤 도착률이 바뀌어도 영상 길이는 벽시계와 같다★ — 프레임을 시각
    기준 칸에 넣는다(늦으면 직전 프레임 반복, 빠르면 버림). recorder.py 상수절 MAX_FILL_S 참고.
    종전에는 처음 잰 fps 와 실제 도착률이 다르면 영상이 그만큼 빠르거나 느리게 재생됐다.

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
import signal
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image

from cv_bridge import CvBridge

#  ★[2026-10-18] 녹화 엔진은 nxde/recorder.py 가 소유한다★ fps 실측·시각 칸·조각 분할·합본·
#  디스크 감시가 전부 거기 있다 — traffic_light 도 같은 엔진으로 디버그 화면을 직접 적는다.
#  이 노드는 '토픽을 받아 그 엔진에 넣는' 일만 한다.
from nxde.recorder import DEFAULT_CODEC, SEGMENT_S, SegmentRecorder


# ══════════════════════════════════════════════════════════════════════════════
#  상수
# ══════════════════════════════════════════════════════════════════════════════
DEFAULT_TOPIC = '/image_raw'
#   white1/camera_launch.py 가 usb_cam 을 이 이름으로 띄운다. traffic_light 의
#   image_topic 기본값도 같다 — ★같은 토픽을 봐야 '인지가 본 화면'이 된다★.

NO_FRAME_WARN_S = 5.0    # 이 시간 동안 프레임이 없으면 경고(usb_cam 확인 안내)


def _package_root():
    """setup.py 가 있는 nxde 패키지 루트. 못 찾으면 None.

    ★symlink-install(egg-link) 이면 __file__ 이 소스트리를 가리킨다★ 그래서 여기서
    찾은 루트가 곧 사용자가 말한 'setup.py 가 있는 디렉터리' 다.
    ★symlink 가 아닌 빌드면 install/ 아래가 나온다★ — 거기 쌓으면 재빌드에 날아가므로
    그때는 루트로 인정하지 않고 홈으로 폴백한다(아래 resolve_output_dir).
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
        #  ★[2026-09-13] 세그먼트 분할★ recorder.py 상수절 참고. 0 이면 한 파일에 쓴다.
        self.declare_parameter('segment_s', SEGMENT_S)
        #  ★[2026-10-16] 끝나면 조각을 최종본 한 파일로 합친다★ (videomerge 머리말)
        self.declare_parameter('merge', True)
        #   True 면 합본 검증 뒤에도 parts/<세션>/ 의 조각을 지우지 않는다.
        self.declare_parameter('keep_parts', False)

        g = lambda k: self.get_parameter(k).value
        self.topic = str(g('image_topic'))
        out_dir, why = resolve_output_dir(str(g('output_dir')))
        self.rec = SegmentRecorder(
            out_dir, prefix=str(g('prefix')) or 'cam', log=self._say, src=self.topic,
            fps=float(g('fps')), scale=float(g('scale')), codec=str(g('codec')),
            segment_s=float(g('segment_s')), merge=bool(g('merge')),
            keep_parts=bool(g('keep_parts')))
        self.final_path = self.rec.final_path

        self.bridge = CvBridge()
        self.exit_requested = False   # 신호를 받았다 — main 이 이것을 보고 나간다 [2026-10-16]
        self._warned_no_frame = False

        # ── 구독 ──
        #   ★QoS 를 usb_cam 에 맞춘다★ BEST_EFFORT / KEEP_LAST / depth=1.
        #   RELIABLE 로 잡으면 ★발행자와 규약이 안 맞아 아예 연결되지 않는다★(프레임이
        #   한 장도 안 온다). white1/traffic_light.py 의 qos_img 와 같은 값이다.
        #   depth=1 인 것도 의도다 — 인코딩이 밀리면 ★최신 프레임만 남기고 버린다★.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, self.topic, self.cb_image, qos)
        self.create_timer(1.0, self._on_tick)

        self._install_signal_handlers()
        self.rec.describe(why)

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
        #  ★[2026-10-16] 여기서 예외를 던지지 않는다★ 종전에는 close() 뒤 SystemExit 을
        #  올렸는데, 신호가 rclpy 의 메시지 꺼내기(take_message — C++ 가 파이썬 객체를 만드는
        #  중) 안에서 처리되면 그 예외가 pybind11 을 지나며 RuntimeError('Unable to convert
        #  call argument') 로 바뀌어 ★파일은 멀쩡히 닫혔는데 트레이스백 + exit 1★ 로 끝났다
        #  (`nxde kill` 실측). → 파일을 닫고 표시만 세운다. main 의 spin_once 루프가 0.1 s
        #  안에 그것을 보고 정상으로 나간다. 콜백 도중에 닫혔어도 recorder 가 writer None 을
        #  보고 멈춘다.
        def _term(signum, _frame):
            self._say(f"신호 {signum} 수신 — 파일을 닫는다")
            self.close()
            self.exit_requested = True
        #  ★[2026-09-13] SIGINT 도 여기서 직접 잡는다★ rclpy 가 스핀을 풀고 → 예외가 올라가고
        #  → finally 가 도는 동안 런치가 유예를 넘겼다고 보고 SIGKILL 을 보내면 파일이 그대로
        #  깨진다(2026-09-13 실측: 큰 파일만 깨졌다). → ★신호를 받은 그 자리에서 먼저 닫는다.★
        for sig in (signal.SIGTERM, signal.SIGINT):
            try:
                signal.signal(sig, _term)
            except (ValueError, OSError):
                pass        # 메인 스레드가 아니면 등록할 수 없다 — atexit 이 받는다
        atexit.register(self.close)

    def _say(self, text, warn=False):
        """종료 경로에서도 메시지가 보이게 한다 — recorder 의 로그도 여기로 온다.

        ★close() 는 rclpy 컨텍스트가 이미 내려간 뒤에 불릴 수 있다★(SIGINT →
        KeyboardInterrupt → finally). 그때 get_logger() 로 찍으면 'Failed to publish log
        message to rosout' 가 따라붙어 ★저장이 실패한 것처럼 보인다★. 컨텍스트가 죽었으면 print 로 낸다.

        ★★ info 와 warn 을 ★한 줄에서★ 부르면 안 된다 [2026-09-10] ★★
        rclpy 의 rcutils_logger 는 로그 설정을 ★호출한 줄★ 단위로 캐시하고, 같은 줄에서
        심각도가 바뀌면 ValueError: Logger severity cannot be changed between calls. 를
        ★던진다★ — 신호 핸들러 안에서 터지면 종료가 트레이스백 + exit 1 로 끝난다(실측).
        → ★두 줄로 나눠 호출 위치를 가른다.★ 줄을 합치지 말 것.
        """
        if rclpy.ok():
            log = self.get_logger()
            if warn:
                log.warn(text)      # ★아래 info 와 반드시 다른 줄이어야 한다★
            else:
                log.info(text)
        else:
            print(('[WARN] ' if warn else '[INFO] ') + text, flush=True)

    def close(self):
        """녹화를 닫는다. ★몇 번 불려도 안전★(신호·atexit·finally 가 겹칠 수 있다)."""
        if self.rec.closed:
            return
        if not self.rec.recording:
            self._say(f"   (ros2 topic hz {self.topic} — usb_cam·발행 노드가 떠 있나?)")
        self.rec.close()

    # ══════════════════════════════════════════════════════════════════════════
    #  수신
    # ══════════════════════════════════════════════════════════════════════════
    def cb_image(self, msg: Image):
        now = time.time()
        self._warned_no_frame = False
        # ★traffic_light 와 같은 한 줄★ — 그래서 같은 그림이 적힌다
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.rec.t_last_frame = now
            self.rec.n_dropped += 1
            self.get_logger().error(
                f"cv_bridge 변환 실패({msg.encoding}): {e}", throttle_duration_sec=5.0)
            return
        self.rec.feed(frame, now)

    # ══════════════════════════════════════════════════════════════════════════
    #  주기 점검 — 프레임 두절 (진행 상황·디스크는 recorder.tick)
    # ══════════════════════════════════════════════════════════════════════════
    def _on_tick(self):
        now = time.time()
        t_last = self.rec.t_last_frame
        if (t_last == 0.0 or now - t_last > NO_FRAME_WARN_S):
            if not self._warned_no_frame:
                self._warned_no_frame = True
                self.get_logger().warn(
                    f"⚠️ {self.topic} 프레임이 {NO_FRAME_WARN_S:.0f}초 이상 없다 — "
                    f"usb_cam 이 떠 있는지, 카메라를 꽂았는지 확인 "
                    f"(ros2 topic hz {self.topic})")
        self.rec.tick(now)

    def destroy_node(self):
        self.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoNode()
    try:
        #  ★[2026-10-16] spin 대신 spin_once 루프★ 신호 핸들러는 파일을 닫고 exit_requested
        #  만 세운다(예외를 던지면 pybind11 안에서 RuntimeError 로 바뀌어 exit 1 이 됐다).
        #  여기서 0.1 s 안에 그 표시를 보고 나간다 — nxde kill 의 SIGTERM 유예(2 s)보다 짧다.
        while rclpy.ok() and not node.exit_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
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
