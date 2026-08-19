#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""camera_model.py — ★카메라 기하(어안 왜곡보정·BEV)의 단일 소유자★ [white1]

    from white1 import camera_model
    camera_model.declare_params(self)          # 파라미터 선언 (이름·기본값의 정본)
    self.cam = camera_model.CameraModel.from_node(self)
    frame = self.cam.undistort(frame)          # ★인지 노드가 부르는 한 줄★

camera_launch.py 가 런치 조각의 단일 소유자인 것과 같은 태도다. 왜곡보정 계수와 BEV
사다리꼴을 노드마다 적어 두면 반드시 갈라진다 — 구 white 가 그랬다(perception.py 안에
fx/fy/cx/cy 가 하드코딩되어 있고, calibration/*.yaml 은 아무도 안 읽는 기록용이었다).
여기 한 곳에 두면 ★신호등·정지선·앞으로 붙일 차선 인지가 같은 그림을 본다★.

────────────────────────────────────────────────────────────────────────────────
 ★왜 /image_raw 를 보정하지 않고 노드 안에서 보정하는가 [2026-08-19 결정]★
────────────────────────────────────────────────────────────────────────────────
usb_cam 에 camera_info_url 을 주고 image_proc 로 /image_rect 를 만드는 길도 있다.
그러지 않은 이유는 둘이다:
  ① ★원본 녹화가 원본이어야 한다★ nxde/video.py 는 /image_raw 를 그대로 적는다.
     파이프라인이 /image_raw 를 보정된 그림으로 바꿔 버리면 '카메라가 실제로 준 그림'
     을 다시는 볼 수 없다(지시사항 — 원본 촬영에는 보정이 필요 없다).
  ② 1080p 30fps 를 프로세스 사이로 한 벌 더 흘리면 약 186MB/s 다. 보정은 어차피
     인지 노드 안에서 remap 한 번(3~5ms)이면 끝난다.
→ 그래서 ★/image_raw 는 원본, 보정은 인지 노드가 이 모듈로 직접★ 한다.

────────────────────────────────────────────────────────────────────────────────
 ★왜 BEV(IPM)가 필요한가 — 픽셀 행으로는 두 단계를 나눌 수 없다★
────────────────────────────────────────────────────────────────────────────────
원본 화면에서 '정지선이 몇 번째 행에 있나'는 거리에 ★비례하지 않는다★(원근). 종전처럼
정지/비정지 두 갈래만 가를 때는 문턱 하나면 됐지만, ★1단 예비제동 → 2단 확정 정지★ 처럼
문턱이 둘이 되면 두 문턱 사이의 간격이 거리로 얼마인지 알 수 없게 된다. BEV 로 펴면
★한 픽셀이 어디서나 같은 거리★ 라서 문턱을 여러 개 두어도 뜻이 유지된다.

그리고 BEV 를 걸려면 왜곡보정이 먼저다 — 어안이 남은 그림에 원근변환을 걸면 직선이
휜 채로 펴져서 숫자가 거짓말을 한다(종전 traffic_light.py 헤더가 '픽셀 행을 쓰는 이유'
로 적어 둔 바로 그것이고, 이제 그 전제가 해소됐다).

────────────────────────────────────────────────────────────────────────────────
 ★fail-open — 이 모듈은 노드를 죽이지 않는다★
────────────────────────────────────────────────────────────────────────────────
캘리브 파일이 없거나(패키지를 안 깔았다·경로 오타) 맵 생성이 실패하면 ★경고 한 줄을
남기고 보정을 끈 채 원본을 그대로 흘린다★. 그러면 신호등 판정은 종전 그대로 돌고,
BEV 거리만 '신뢰 못함'이 된다. 카메라 기하 때문에 차가 못 서는 일은 없어야 한다.
"""

import os

import cv2
import numpy as np

#  캘리브 파일 기본 위치 — setup.py 가 share/white1/calibration/ 로 깐다.
CALIB_DIR_NAME  = 'calibration'
CALIB_FILE_NAME = 'usb_cam_calibration.yaml'

#  ★BEV 사다리꼴 기본값 [2026-08-19]★  좌상, 우상, 우하, 좌하 (보정된 원본 화면 좌표)
#
#  구 white(perception.py ipm_src_pts)는 [620,650, 1300,650, 1920,1080, 0,1080] 이었다.
#  그 값에서 ★상단만 y=650 → 560 으로 올렸다★ — 1단 예비제동은 2단보다 ★먼 곳★ 을 보고
#  걸어야 하는데, 상단이 650 이면 BEV 가 담는 깊이가 2단 문턱을 조금 넘는 정도라서
#  예비제동을 걸 자리가 남지 않는다.
#  x 값은 ★같은 노면 경계선 위에서★ 뽑았다(사다리꼴 두 변을 그대로 연장):
#     반폭(y) = 340 + 620·(y−650)/430   →  y=560 에서 210 → x = 960∓210 = 750 / 1170
#  ⚠️ ★그래도 이것은 구 차량 마운트에서 유도한 값이다★ 이 차의 카메라 높이·틸트로
#     다시 잡아야 한다(STOPLINE_TEST.md 단계 2). 화면에 사다리꼴을 그려 두었으니
#     노면의 직사각형(정지선 폭 × 몇 m)에 맞춰 눈으로 맞추면 된다.
BEV_SRC_PTS_DEFAULT = [750.0, 560.0, 1170.0, 560.0, 1920.0, 1080.0, 0.0, 1080.0]

BEV_W_DEFAULT = 640
BEV_H_DEFAULT = 480

#  이 값보다 먼 것은 '아주 멀다' 로 뭉갠다. 소실선 근처의 점은 원근변환이 발산해
#  −10⁶ 같은 숫자가 나오는데, 그것을 그대로 로그·HUD 에 찍으면 읽을 수가 없다.
#  ★판정에는 영향이 없다★ 어느 문턱보다도 크기 때문이다.
BEV_FAR_CLAMP_PX = 9999.0


def declare_params(node):
    """카메라 기하 파라미터를 선언한다 — ★이름과 기본값의 정본이 여기다★

    카메라를 쓰는 노드는 전부 이 함수를 부른다. 그래야 신호등·차선이 ★같은 이름의
    같은 파라미터★ 를 갖게 되어, camera_launch.camera_params() 한 벌을 그대로
    두 노드에 먹일 수 있다.
    """
    d = node.declare_parameter
    # ── 어안 왜곡보정 ──────────────────────────────────────────────────────
    #   ★기본 켬★ [2026-08-19 지시] 카메라 노드는 기본으로 보정된 그림을 본다.
    d('cam_undistort', True)
    #   getOptimalNewCameraMatrix 의 alpha. 0 = 검은 테두리가 남지 않게 잘라낸다
    #   (화각이 조금 좁아진다). 1 = 원본 화소를 다 남기고 테두리에 검은 영역을 둔다.
    #   구 white 도 0.0 이었다 — 인지 입력에 검은 테두리를 넣지 않으려는 것이다.
    d('cam_undistort_alpha', 0.0)
    #   빈 문자열이면 share/white1/calibration/usb_cam_calibration.yaml 을 찾는다.
    d('cam_calib_file', '')

    # ── BEV (IPM) ─────────────────────────────────────────────────────────
    d('bev_src_pts', BEV_SRC_PTS_DEFAULT)
    d('bev_w', BEV_W_DEFAULT)
    d('bev_h', BEV_H_DEFAULT)
    #   ★앞범퍼가 BEV 의 몇 번째 행인가 = 거리 0 의 기준★ 기본값은 BEV 최하단이다.
    #   차체가 화면 하단을 가려서 범퍼가 사다리꼴 밑변보다 더 아래(=차 쪽)에 있으면
    #   bev_h 보다 큰 값을 준다. 실측 절차는 STOPLINE_TEST.md 단계 2.
    d('bev_bumper_y_px', float(BEV_H_DEFAULT))
    #   BEV 한 픽셀이 몇 m 인가. ★제어에는 쓰지 않는다★ — 0 이 아니면 HUD·로그에
    #   참고 미터를 함께 찍어 준다(픽셀 문턱을 미터로 감 잡을 때만 쓴다).
    d('bev_px_to_m', 0.0)


class CameraModel:
    """왜곡보정 맵과 BEV 호모그래피를 들고 있는 값 객체.

    ★상태를 갖지 않는다★ (프레임을 기억하지 않는다). 노드가 프레임마다 부른다.
    """

    def __init__(self, log, undistort, alpha, calib_path,
                 src_pts, bev_w, bev_h, bumper_y, px_to_m):
        self._log       = log
        self.enabled    = bool(undistort)     # 왜곡보정을 실제로 하는가(로드 실패 시 False)
        self.alpha      = float(alpha)
        self.calib_path = calib_path
        self.bev_w      = int(bev_w)
        self.bev_h      = int(bev_h)
        self.bumper_y   = float(bumper_y)
        self.px_to_m    = float(px_to_m)

        self.K = None          # 캘리브 원본 내부행렬 (calib_size 기준)
        self.D = None
        self.calib_size = None  # 캘리브를 뜬 해상도 (w, h)
        self._maps = None       # (map1, map2, roi, size) — 첫 프레임에서 만든다
        self._map_size = None

        if self.enabled:
            self.enabled = self._load_calib(calib_path)

        # ── BEV 호모그래피 ────────────────────────────────────────────────
        #   ★보정 여부와 무관하게 만든다★ 보정이 꺼져도 숫자는 나온다 — 다만 그 숫자를
        #   믿으면 안 되고, 그 경고는 describe() 가 한 번 찍는다.
        self.src_pts = np.asarray(src_pts, dtype=np.float64).reshape(4, 2)
        self.dst_pts = np.float64([
            [0, 0], [self.bev_w, 0], [self.bev_w, self.bev_h], [0, self.bev_h],
        ])
        self.M_bev = cv2.getPerspectiveTransform(
            self.src_pts.astype(np.float32), self.dst_pts.astype(np.float32))
        self.M_bev_inv = cv2.getPerspectiveTransform(
            self.dst_pts.astype(np.float32), self.src_pts.astype(np.float32))
        #  ★동차좌표 w 의 부호 기준 [2026-08-19]★ 호모그래피는 ★전체 스케일까지만★
        #  정해지므로 w 의 절대적인 부호에는 뜻이 없다(−M 도 같은 변환이다). 그래서
        #  '노면 위의 점'인 사다리꼴 중심으로 한 번 재서 그 부호를 기준으로 삼는다.
        #  같은 부호면 노면 쪽, 반대 부호면 ★소실선 너머★ 다(poly_to_bev 참고).
        c = self.src_pts.mean(axis=0)
        w_ref = float(self.M_bev[2, 0] * c[0] + self.M_bev[2, 1] * c[1] + self.M_bev[2, 2])
        self._w_sign = 1.0 if w_ref >= 0.0 else -1.0

    # ══════════════════════════════════════════════════════════════════════════
    #  생성
    # ══════════════════════════════════════════════════════════════════════════
    @classmethod
    def from_node(cls, node):
        """declare_params() 로 선언된 파라미터를 읽어 만든다."""
        g = lambda k: node.get_parameter(k).value
        path = str(g('cam_calib_file')).strip() or default_calib_path()
        return cls(
            log=node.get_logger(),
            undistort=bool(g('cam_undistort')),
            alpha=float(g('cam_undistort_alpha')),
            calib_path=path,
            src_pts=list(g('bev_src_pts')),
            bev_w=int(g('bev_w')),
            bev_h=int(g('bev_h')),
            bumper_y=float(g('bev_bumper_y_px')),
            px_to_m=float(g('bev_px_to_m')),
        )

    def _load_calib(self, path):
        """캘리브 yaml 을 읽는다. ★실패해도 예외를 밖으로 내지 않는다★ (fail-open)."""
        try:
            import yaml
            with open(path, 'r') as f:
                y = yaml.safe_load(f)
            cm = y['camera_matrix']['data']
            dc = y['distortion_coefficients']['data']
            self.K = np.asarray(cm, dtype=np.float64).reshape(3, 3)
            self.D = np.asarray(dc, dtype=np.float64).reshape(1, -1)
            self.calib_size = (int(y.get('image_width', 1920)),
                               int(y.get('image_height', 1080)))
            return True
        except Exception as e:
            self._log.error(
                f"카메라 캘리브를 못 읽었다 — ★왜곡보정 없이 돈다★ ({path}: {e})\n"
                "   colcon build 를 다시 하거나 cam_calib_file 로 경로를 주면 된다. "
                "보정 없이도 신호등 판정은 종전대로 돌지만 BEV 거리는 믿을 수 없다.")
            return False

    def _build_maps(self, w, h):
        """프레임 크기에 맞춰 remap 맵을 만든다 — ★첫 프레임에서 한 번★.

        캘리브를 뜬 해상도와 실제 프레임 크기가 다를 수 있다(camera_launch 의
        CAM_WIDTH/HEIGHT 를 낮추는 경우). 그때는 내부행렬을 비율로 줄여 쓴다 —
        핀홀 모델에서 해상도만 바꾸면 fx·fy·cx·cy 가 같은 비율로 스케일된다.
        """
        cw, ch = self.calib_size
        K = self.K.copy()
        if (cw, ch) != (w, h):
            sx, sy = float(w) / float(cw), float(h) / float(ch)
            K[0, 0] *= sx; K[0, 2] *= sx
            K[1, 1] *= sy; K[1, 2] *= sy
            self._log.warn(
                f"캘리브 해상도({cw}x{ch})와 프레임({w}x{h})이 다르다 — "
                f"내부행렬을 비율로 스케일해 쓴다(sx={sx:.3f} sy={sy:.3f})")
        newK, roi = cv2.getOptimalNewCameraMatrix(K, self.D, (w, h), self.alpha, (w, h))
        m1, m2 = cv2.initUndistortRectifyMap(K, self.D, None, newK, (w, h), cv2.CV_16SC2)
        self._maps = (m1, m2, roi)
        self._map_size = (w, h)

    # ══════════════════════════════════════════════════════════════════════════
    #  보정
    # ══════════════════════════════════════════════════════════════════════════
    def undistort(self, frame):
        """어안 왜곡을 편다. ★프레임 크기는 그대로 유지된다★

        구 white(perception.py cb)와 같은 절차다 — remap → 유효 ROI 로 크롭 →
        원래 크기로 리사이즈. 크기를 유지하는 이유는 ★tl_roi·박스 높이 임계 같은
        픽셀 단위 설정이 계속 통하게★ 하려는 것이다(뜻이 조금 달라지긴 한다 —
        크롭·확대만큼 물체가 커지므로 근접도 임계는 재실측 대상이다. todo 8항).

        보정이 꺼져 있거나 로드에 실패했으면 ★원본을 그대로 돌려준다★.
        """
        if not self.enabled or frame is None:
            return frame
        h, w = frame.shape[:2]
        try:
            if self._maps is None or self._map_size != (w, h):
                self._build_maps(w, h)
            m1, m2, roi = self._maps
            out = cv2.remap(frame, m1, m2, interpolation=cv2.INTER_LINEAR,
                            borderMode=cv2.BORDER_CONSTANT)
            rx, ry, rw, rh = roi
            if rw > 0 and rh > 0 and (rw, rh) != (w, h):
                out = cv2.resize(out[ry:ry + rh, rx:rx + rw], (w, h),
                                 interpolation=cv2.INTER_LINEAR)
            return out
        except Exception as e:
            # 한 프레임 실패로 노드가 죽으면 안 된다 — 보정을 끄고 원본으로 계속 간다.
            self._log.error(f"왜곡보정 실패 — 보정을 끄고 계속한다: {e}")
            self.enabled = False
            return frame

    # ══════════════════════════════════════════════════════════════════════════
    #  BEV
    # ══════════════════════════════════════════════════════════════════════════
    def to_bev(self, frame):
        """이미지를 통째로 BEV 로 편다. ★HUD 전용★

        거리 계산에는 필요 없다(폴리곤 점만 옮기면 된다 — poly_to_bev). 이미지 워프는
        1ms 안팎이지만 매 프레임 낼 이유가 없으므로 창을 띄울 때만 부른다.
        """
        return cv2.warpPerspective(frame, self.M_bev, (self.bev_w, self.bev_h))

    def poly_to_bev(self, pts):
        """폴리곤(보정된 원본 좌표) → BEV 좌표. ★소실선 너머의 점은 버린다★

        돌려주는 것 : (bev_pts[N,2], ok[N]) — ok=False 인 점은 좌표가 무의미하다.

        cv2.perspectiveTransform 을 안 쓰고 직접 곱하는 이유가 이 ok 다. 동차좌표의
        w 가 0 이하가 되는 점(=가상 카메라 뒤, 실질적으로 소실선 위쪽)은 나눗셈이
        발산해 −10⁶ 같은 좌표가 나오는데, cv2 는 그것을 그냥 돌려준다. 그 값을 최하단
        판정에 섞으면 ★없는 정지선이 코앞에 있는 것처럼★ 보일 수 있다.
        """
        p = np.asarray(pts, dtype=np.float64).reshape(-1, 2)
        hom = np.hstack([p, np.ones((len(p), 1))]) @ self.M_bev.T
        w = hom[:, 2]
        ok = (w * self._w_sign) > 1e-9
        out = np.zeros((len(p), 2), dtype=np.float64)
        if np.any(ok):
            out[ok, 0] = hom[ok, 0] / w[ok]
            out[ok, 1] = hom[ok, 1] / w[ok]
        return out, ok

    def bumper_dist_px(self, y_bev):
        """BEV 의 y 행에서 ★앞범퍼까지의 픽셀 거리★. 음수 = 이미 지나쳤다."""
        return float(self.bumper_y) - float(y_bev)

    def nearest_dist_px(self, pts):
        """폴리곤에서 ★차에 가장 가까운 점★ 까지의 픽셀 거리를 돌려준다.

        돌려주는 것 : (dist_px, bev_pts) — 쓸 수 있는 점이 하나도 없으면 (None, None).
        가장 가까운 점 = BEV 에서 y 가 가장 큰 점(BEV 는 아래로 갈수록 차 쪽이다).
        """
        bev, ok = self.poly_to_bev(pts)
        if not np.any(ok):
            return None, None
        y_near = float(np.max(bev[ok, 1]))
        d = self.bumper_dist_px(y_near)
        return float(np.clip(d, -BEV_FAR_CLAMP_PX, BEV_FAR_CLAMP_PX)), bev

    def m_txt(self, px):
        """픽셀 거리에 참고 미터를 붙인 문자열. bev_px_to_m 이 0 이면 빈 문자열."""
        if self.px_to_m <= 0.0:
            return ''
        return f"({px * self.px_to_m:.2f}m)"

    # ══════════════════════════════════════════════════════════════════════════
    #  진단
    # ══════════════════════════════════════════════════════════════════════════
    def describe(self):
        """기동 로그 한 줄. ★지금 무슨 그림을 보고 있는지★ 를 사람이 확인하는 곳이다."""
        if self.enabled:
            head = (f"📐 카메라 보정 ON alpha={self.alpha:g} "
                    f"({os.path.basename(self.calib_path)})")
        else:
            head = "📐 카메라 보정 ★OFF★ — 원본 그대로 본다"
        sp = ' '.join(f"({x:.0f},{y:.0f})" for x, y in self.src_pts)
        bev = (f" | BEV {self.bev_w}x{self.bev_h} 범퍼행={self.bumper_y:.0f}px "
               f"사다리꼴 {sp}")
        if self.px_to_m > 0.0:
            bev += f" ({self.px_to_m:.4f} m/px)"
        if not self.enabled:
            bev += "\n   ⚠️ 보정이 꺼진 채로는 ★BEV 거리를 믿으면 안 된다★ — 어안이 남은 "\
                   "그림에 원근변환을 걸면 직선이 휜 채로 펴진다"
        return head + bev


def default_calib_path():
    """share/white1/calibration/usb_cam_calibration.yaml 의 절대경로.

    ament 인덱스를 못 찾으면(소스 트리에서 맨손으로 돌리는 경우) 이 파일 기준 상대
    경로로 폴백한다 — 둘 다 실패하면 어차피 _load_calib 이 fail-open 으로 받는다.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        share = os.path.join(get_package_share_directory('white1'),
                             CALIB_DIR_NAME, CALIB_FILE_NAME)
        if os.path.exists(share):
            return share
    except Exception:
        pass
    #  아직 colcon build 를 안 했거나 소스 트리에서 맨손으로 돌리는 경우 —
    #  ★있는 파일을 놔두고 fail-open 으로 떨어지지 않게★ 소스 쪽도 본다.
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(os.path.join(here, '..', CALIB_DIR_NAME, CALIB_FILE_NAME))
