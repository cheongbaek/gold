#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import time
import math
import warnings
import os
import threading
import datetime
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String, Float32, Bool
from cv_bridge import CvBridge
from ultralytics import YOLO

HSV_FALLBACK_CONF = 0.55
MIN_FIT_PIX       = 50     # polyfit에 필요한 최소 inlier 픽셀 수

# NumPy 2.x: np.RankWarning → np.exceptions.RankWarning
try:
    _RankWarning = np.exceptions.RankWarning
except AttributeError:
    _RankWarning = np.RankWarning


# ================================================================
# [추가] 시각화 창 녹화용 스레드 기반 webm 레코더
#   - 창별로 별도의 .webm(VP8) 파일 생성
#   - write()는 '최신 프레임' 슬롯에 저장만 하고 즉시 리턴(실시간 루프 보호)
#   - 워커 스레드가 정확히 record_fps 주기로 최신 프레임을 샘플링해 인코딩
#     → 소스가 빠르면 중간 프레임 드롭, 느리면 직전 프레임 복제
#     → 초당 정확히 record_fps장 기록 → 재생속도=실시간(배속/슬로우 없음)
#
# [세션] 주행 1회 = 세션 폴더 1개. start_session()/stop_session() 으로 몇 번이든
#   재개할 수 있다(예전 close() 는 편도라 한 번 닫으면 워커가 전부 죽었다).
#   폴더 생성도 start_session() 시점으로 옮겼다 — 예전엔 __init__ 에서 makedirs 해서
#   주행을 안 해도 빈 세션 폴더가 계속 쌓였다.
# ================================================================
class _ThreadedWebmRecorder:
    def __init__(self, out_dir, fps=30.0, fourcc="VP80",
                 max_width=0, logger=None):
        self.enabled   = True
        self.fps       = float(fps)
        self.fourcc    = cv2.VideoWriter_fourcc(*fourcc)
        self.logger    = logger
        self.out_dir   = out_dir
        self.max_width = int(max_width)                    # 0=원본, >0이면 이 폭으로 축소
        self._min_dt   = (1.0 / self.fps) if self.fps > 0 else (1.0 / 30.0)
        self.session_dir = None   # start_session() 에서 결정
        self._active   = False
        self._latest   = {}   # name -> 최신 프레임(np.ndarray)
        self._locks    = {}   # name -> Lock
        self._threads  = {}   # name -> Thread
        self._failed   = set()
        self._stop     = threading.Event()   # 현재 세션의 정지신호(세션마다 새로 만든다)
        self._stop.set()                     # 세션 없음 = 정지 상태

    def is_active(self):
        return self.enabled and self._active

    def start_session(self, tag=""):
        """녹화 세션 시작. 이미 녹화 중이면 무시(멱등).
        멱등성은 선택이 아니다 — /control_state=True 는 prompt._drive_flow 와
        driving.load_waypoints 양쪽에서 발행되어 매 주행 두 번 들어온다."""
        if not self.enabled or self._active:
            return
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        session_dir = os.path.join(self.out_dir, f"{stamp}_{tag}" if tag else stamp)
        try:
            os.makedirs(session_dir, exist_ok=True)
        except Exception as e:
            self._err(f"[REC] 출력 폴더 생성 실패: {e} → 이번 세션 녹화 없음")
            return
        self.session_dir = session_dir
        self._latest.clear()
        self._locks.clear()
        self._threads.clear()
        self._failed.clear()
        # 세션마다 새 Event. 워커는 자기 세션의 Event 를 인자로 들고 가므로
        # 여기서 교체해도 이전 세션 워커의 종료신호를 잃지 않는다.
        self._stop = threading.Event()
        self._active = True
        self._log(f"[REC] ▶ 녹화 세션 시작 → {session_dir}")

    def stop_session(self):
        """세션 종료 — 워커를 멈추고 파일을 닫는다. 이후 start_session() 으로 재개 가능."""
        if not self._active:
            return
        self._active = False
        self._stop.set()
        for t in self._threads.values():
            t.join(timeout=5.0)
        self._threads.clear()
        self._locks.clear()
        self._latest.clear()
        self._log(f"[REC] ■ 녹화 세션 저장 완료 → {self.session_dir}")

    def _log(self, msg):
        if self.logger is not None:
            self.logger.info(msg)
        else:
            print(msg)

    def _err(self, msg):
        if self.logger is not None:
            self.logger.error(msg)
        else:
            print(msg)

    def write(self, name, img):
        if (not self.enabled) or (not self._active) or img is None or (name in self._failed):
            return
        # 다운스케일(특히 1080p Lane 인코딩 부하 감소). 없으면 복사(버퍼 공유 방지)
        if self.max_width and img.shape[1] > self.max_width:
            s = self.max_width / float(img.shape[1])
            img = cv2.resize(img, (self.max_width, int(round(img.shape[0] * s))))
        else:
            img = img.copy()
        lock = self._locks.get(name)
        if lock is None:
            if not self._start_stream(name, img.shape[1], img.shape[0]):
                return
            lock = self._locks[name]
        with lock:
            self._latest[name] = img   # 최신 프레임 갱신(덮어쓰기). 인코딩은 워커가 담당

    def _start_stream(self, name, w, h):
        path = os.path.join(self.session_dir, f"{name}.webm")
        vw = cv2.VideoWriter(path, self.fourcc, self.fps, (w, h))
        if not vw.isOpened():
            self._err(f"[REC] '{name}' writer 열기 실패 → 이 창 녹화 비활성")
            self._failed.add(name)
            return False
        self._locks[name]  = threading.Lock()
        self._latest[name] = None
        # 자기 세션의 stop Event 를 인자로 넘긴다. self._stop 을 직접 읽게 하면
        # 다음 세션이 Event 를 교체했을 때 이전 세션 워커가 새 Event(미설정)를
        # 보고 영영 안 멈춘다.
        t = threading.Thread(target=self._worker,
                             args=(name, vw, (w, h), self._stop), daemon=True)
        self._threads[name] = t
        t.start()
        self._log(f"[REC] 녹화 시작 → {path} ({w}x{h}, {self.fps:.1f}fps)")
        return True

    def _worker(self, name, vw, size, stop_ev):
        w, h = size
        lock = self._locks[name]
        next_t = None                      # 첫 프레임 도착 후 시계 시작
        while not stop_ev.is_set():
            with lock:
                img = self._latest.get(name)
            if img is None:                # 아직 첫 프레임 없음
                stop_ev.wait(timeout=0.01)
                continue
            if next_t is None:
                next_t = time.monotonic()
            now = time.monotonic()
            if now < next_t:               # 다음 프레임 시각까지 대기
                stop_ev.wait(timeout=min(next_t - now, 0.1))
                continue
            if (img.shape[1], img.shape[0]) != (w, h):
                img = cv2.resize(img, (w, h))
            try:
                vw.write(img)              # 고정 주기로 최신 프레임 1장 기록
            except Exception as e:
                self._err(f"[REC] '{name}' write 오류: {e}")
            next_t += self._min_dt
            now2 = time.monotonic()
            if now2 - next_t > self._min_dt:   # 인코더가 크게 밀리면 재동기(버스트 방지)
                next_t = now2 + self._min_dt
        vw.release()

    def close(self):
        """노드 종료 — 진행 중 세션을 닫는다(이후 재개 없음)."""
        if not self.enabled:
            return
        self.stop_session()
        self.enabled = False
        self._log("[REC] 모든 녹화 파일 저장 완료")


class LaneLine(Node):
    def __init__(self):
        super().__init__("roi_laneline")

        # =============================
        # 파라미터 선언
        # =============================
        self.declare_parameter("image_topic",       "/image_raw")
        self.declare_parameter("show_window",       True)
        self.declare_parameter("lane_weights_roi",
            '/home/mad2/runs2/runs/segment/lane_line_new2/weights/best.engine')
        self.declare_parameter("device",            "cuda")
        self.declare_parameter("lane_conf",         0.3)
        self.declare_parameter("lane_imgsz",        640)
        self.declare_parameter("lane_interval",     1)

        self.declare_parameter("lane_roi_xmin",     0)
        self.declare_parameter("lane_roi_ymin",     0)
        self.declare_parameter("lane_roi_xmax",     1920)
        self.declare_parameter("lane_roi_ymax",     1080)

        # IPM (BEV) 파라미터
        self.declare_parameter("ipm_src_pts",
            [620, 650, 1300, 650, 1920, 1080, 0, 1080])
        self.declare_parameter("bev_w",             640)
        self.declare_parameter("bev_h",             480)

        # [v7-D] 반폭 초기값 자동 파생용 (judgment의 lane_width_m과 같은 값을 launch에서 넘길 것)
        #   measured_half_width 초기값 = (lane_width_m/2) / pixel_to_meter_bev
        #   auto_half_width=False면 기존 하드코딩(bev_w×0.28) 사용.
        self.declare_parameter("lane_width_m",      3.0)
        self.declare_parameter("auto_half_width",   True)
        self.declare_parameter("pixel_to_meter_bev", 0.006)

        # BEV 크롭
        self.declare_parameter("bev_crop_y_min",    0)
        self.declare_parameter("bev_crop_y_max",    480)

        # 시각화 전용 (publish와 무관)
        self.declare_parameter("lookahead_ratio",   0.45)
        self.declare_parameter("bottom_ratio",      0.92)

        # Sliding Window 파라미터
        self.declare_parameter("sw_num_windows",        10)
        self.declare_parameter("sw_margin",             60)
        self.declare_parameter("sw_minpix",             30)
        self.declare_parameter("sw_hist_bottom_ratio",  0.3)
        self.declare_parameter("polyfit_degree",        2)
        self.declare_parameter("sw_min_hist_peak",      500)

        # v13: Temporal EMA / Sanity Check / Soft Confidence
        self.declare_parameter("fit_ema_alpha",         0.5)   # 새 fit 가중치 (0=고정, 1=스무딩 없음)
        self.declare_parameter("fit_hold_max_frames",   15)    # 연속 LOST 시 fit reset까지 프레임 수
        self.declare_parameter("sanity_enable",         True)
        self.declare_parameter("sanity_width_min_ratio", 0.15) # BEV 폭 대비 최소 차선폭 (하한만 검사)
        self.declare_parameter("conf_pix_saturate",     100)  # 이 픽셀 수에서 conf=1.0

        # [우선순위2] 개별 fit 타당성 게이트 (_fit_plausible)
        #   max_fit_curvature : 2차항 |a| 상한 → 화면을 가로지르는 ghost 곡선(점선 조각/
        #     횡단보도 경계 오염)을 기각. BEV 픽셀 기준이라 IPM 캘리브에 따라 튜닝 필요
        #     (작을수록 엄격). 현재 src_pts가 v15보다 넓어 0.002는 다소 관대할 수 있음.
        self.declare_parameter("max_fit_curvature",     0.002)

        # [우선순위1] 평행 차선 제약 — 점선(dash) 쪽 곡률 튐 방지
        #   신뢰도 높은 실선 쪽 곡률(a,b)을 점선 쪽에 복사하고 offset(c)만 dash 픽셀로 재추정.
        #   ① enable        : on/off
        #   ② yspan_ratio   : 약한 쪽 y-span이 (크롭높이×이 값) 미만이면 점선으로 간주
        #   ③ min_pts       : offset 재추정에 필요한 최소 dash 픽셀 수
        self.declare_parameter("parallel_constraint_enable", True)
        self.declare_parameter("parallel_yspan_ratio",       0.7)
        self.declare_parameter("parallel_min_pts",           20)

        # 시각화 옵션
        self.declare_parameter("draw_roi",          True)
        self.declare_parameter("draw_fps",          True)
        self.declare_parameter("draw_lane_outline", True)

        # GPU 최적화
        self.declare_parameter("model_half_precision", False)

        # BEV 캘리브레이션 모드
        self.declare_parameter("bev_calibration_mode", False) 
        self.declare_parameter("bev_calib_real_distance_m", 3.0)

        # Sliding Window 디버그 창
        self.declare_parameter("show_bev_lane_only", True)

        # =============================
        # Traffic Light 파라미터
        # =============================
        self.declare_parameter("tl_weights_roi",
            '/home/mad2/runs2/runs/detect/combined_light/weights/best.engine')
        self.declare_parameter("tl_conf",              0.35)
        self.declare_parameter("tl_imgsz",             640)
        # ★[2026-08-05 복원] 2 → 1★ kasa 이식 과정에서 2 로 올라가 있었다(신호등 판정을
        #   두 프레임에 한 번만 = 반응 절반). perception 이 정상 구간에서도 6.5Hz 뿐이라
        #   2 로 두면 실효 3.2Hz 가 되고, camera_judgment 의 tl_hold_s(0.4s) 를 채우려면
        #   ★연속 RED 를 볼 기회 자체가 부족해진다★. CPU 가 모자라면 다시 올릴 것.
        self.declare_parameter("tl_interval",          1)
        self.declare_parameter("tl_roi_xmin",          0)
        self.declare_parameter("tl_roi_ymin",          0)
        self.declare_parameter("tl_roi_xmax",          1920)
        self.declare_parameter("tl_roi_ymax",          540)
        self.declare_parameter("tl_min_area",          20)
        self.declare_parameter("tl_max_area",          30000)
        self.declare_parameter("tl_min_aspect",        0.2)
        self.declare_parameter("tl_max_aspect",        4.0)
        self.declare_parameter("tl_red_stop_min_height", 25)
        self.declare_parameter("use_hsv_refine",       True)
        self.declare_parameter("hsv_min_color_pixels", 15)
        self.declare_parameter("hsv_crop_center_ratio",0.5)
        self.declare_parameter("hsv_red_h1_low",       0)
        self.declare_parameter("hsv_red_h1_high",      10)
        self.declare_parameter("hsv_red_h2_low",       170)
        self.declare_parameter("hsv_red_h2_high",      180)
        self.declare_parameter("hsv_green_h_low",      45)
        self.declare_parameter("hsv_green_h_high",     90)
        # ★[2026-08-05 복원] 실측 기반 하향값으로 되돌렸다 (80/70 → 55/60)★
        #   tool/tl_hsv_probe.py 로 실제 신호등 크롭 3장을 재본 결과, 기존값(S≥80, V≥70)은
        #   3장 중 2장이 통과하지 못했다(관찰 최저 S=62, V=66). 그보다 낮게 잡아야 RED 가
        #   검출된다. kasa 이식에서 이 값이 옛 80/70 으로 돌아가 있었고, 그 상태로는
        #   RED 프레임이 드물어져 camera_judgment 의 tl_hold_s 를 채우지 못한다
        #   (tl_gap_grace_s 소실과 겹쳐 '신호등에서 안 서는' 원인이 되었다).
        #   ⚠️ 낮추면 붉은 간판·후미등 오검출이 늘 수 있다 — tl_min_area / tl_min_aspect /
        #      tl_red_stop_min_height 와 YOLO conf(tl_conf) 가 그 1차 방어선이다.
        self.declare_parameter("hsv_sat_low",          55)
        self.declare_parameter("hsv_val_low",          60)
        self.declare_parameter("tl_draw_roi",          True)
        self.declare_parameter("tl_draw_fps",          True)

        # 어안 왜곡 보정
        self.declare_parameter("enable_undistort",  True)
        self.declare_parameter("undistort_alpha",   0.0)

        # =============================
        # 파라미터 로드
        # =============================
        self.image_topic   = str(self.get_parameter("image_topic").value)
        self.show_window   = bool(self.get_parameter("show_window").value)
        self.device        = str(self.get_parameter("device").value)
        self.lane_conf     = float(self.get_parameter("lane_conf").value)
        self.lane_imgsz    = int(self.get_parameter("lane_imgsz").value)
        self.lane_interval = max(1, int(self.get_parameter("lane_interval").value))

        self.lane_roi = (
            int(self.get_parameter("lane_roi_xmin").value),
            int(self.get_parameter("lane_roi_ymin").value),
            int(self.get_parameter("lane_roi_xmax").value),
            int(self.get_parameter("lane_roi_ymax").value),
        )

        src_flat     = self.get_parameter("ipm_src_pts").value
        self.src_pts = np.float32(src_flat).reshape(4, 2)
        self.bev_w   = int(self.get_parameter("bev_w").value)
        self.bev_h   = int(self.get_parameter("bev_h").value)

        # [v7-D] 반폭 초기값 자동 파생용 파라미터 로드
        self.lane_width_m         = float(self.get_parameter("lane_width_m").value)
        self.auto_half_width      = bool(self.get_parameter("auto_half_width").value)
        self.pixel_to_meter_bev   = float(self.get_parameter("pixel_to_meter_bev").value)

        self.bev_crop_y_min = int(self.get_parameter("bev_crop_y_min").value)
        self.bev_crop_y_max = int(self.get_parameter("bev_crop_y_max").value)
        self.bev_crop_y_min = max(0, min(self.bev_crop_y_min, self.bev_h - 1))
        self.bev_crop_y_max = max(self.bev_crop_y_min + 1, min(self.bev_crop_y_max, self.bev_h))

        self.dst_pts = np.float32([
            [0, 0], [self.bev_w, 0],
            [self.bev_w, self.bev_h], [0, self.bev_h],
        ])
        self.M_ipm     = cv2.getPerspectiveTransform(self.src_pts, self.dst_pts)
        self.M_ipm_inv = cv2.getPerspectiveTransform(self.dst_pts, self.src_pts)

        # 시각화 전용
        self.lookahead_ratio = float(self.get_parameter("lookahead_ratio").value)
        self.bottom_ratio    = float(self.get_parameter("bottom_ratio").value)

        # Sliding Window
        self.sw_num_windows       = int(self.get_parameter("sw_num_windows").value)
        self.sw_margin            = int(self.get_parameter("sw_margin").value)
        self.sw_minpix            = int(self.get_parameter("sw_minpix").value)
        self.sw_hist_bottom_ratio = float(self.get_parameter("sw_hist_bottom_ratio").value)
        self.polyfit_degree       = int(self.get_parameter("polyfit_degree").value)
        self.sw_min_hist_peak     = int(self.get_parameter("sw_min_hist_peak").value)

        # v13 신규
        self.fit_ema_alpha          = float(self.get_parameter("fit_ema_alpha").value)
        self.fit_hold_max_frames    = int(self.get_parameter("fit_hold_max_frames").value)
        self.sanity_enable          = bool(self.get_parameter("sanity_enable").value)
        self.sanity_width_min_ratio = float(self.get_parameter("sanity_width_min_ratio").value)
        self.conf_pix_saturate      = float(self.get_parameter("conf_pix_saturate").value)

        # [우선순위2] 개별 fit 타당성 게이트
        self.max_fit_curvature = float(self.get_parameter("max_fit_curvature").value)

        # [우선순위1] 평행 차선 제약
        self.parallel_constraint_enable = bool(self.get_parameter("parallel_constraint_enable").value)
        self.parallel_yspan_ratio       = float(self.get_parameter("parallel_yspan_ratio").value)
        self.parallel_min_pts           = int(self.get_parameter("parallel_min_pts").value)

        self.draw_roi          = bool(self.get_parameter("draw_roi").value)
        self.draw_fps          = bool(self.get_parameter("draw_fps").value)
        self.draw_lane_outline = bool(self.get_parameter("draw_lane_outline").value)
        self.model_half_precision = bool(self.get_parameter("model_half_precision").value)

        self.bev_calibration_mode = bool(self.get_parameter("bev_calibration_mode").value)
        self.bev_calib_real_distance_m = float(self.get_parameter("bev_calib_real_distance_m").value)
        self._calib_clicks = []

        self.show_bev_lane_only = bool(self.get_parameter("show_bev_lane_only").value)

        # ── Traffic Light 파라미터 로드 ──
        self.tl_conf     = float(self.get_parameter("tl_conf").value)
        self.tl_imgsz    = int(self.get_parameter("tl_imgsz").value)
        self.tl_interval = max(1, int(self.get_parameter("tl_interval").value))
        self.tl_roi = (
            int(self.get_parameter("tl_roi_xmin").value),
            int(self.get_parameter("tl_roi_ymin").value),
            int(self.get_parameter("tl_roi_xmax").value),
            int(self.get_parameter("tl_roi_ymax").value),
        )
        self.tl_min_area   = int(self.get_parameter("tl_min_area").value)
        self.tl_max_area   = int(self.get_parameter("tl_max_area").value)
        self.tl_min_aspect = float(self.get_parameter("tl_min_aspect").value)
        self.tl_max_aspect = float(self.get_parameter("tl_max_aspect").value)
        self.tl_red_stop_min_height = int(self.get_parameter("tl_red_stop_min_height").value)
        self.use_hsv_refine        = bool(self.get_parameter("use_hsv_refine").value)
        self.hsv_min_color_pixels  = int(self.get_parameter("hsv_min_color_pixels").value)
        self.hsv_crop_center_ratio = float(self.get_parameter("hsv_crop_center_ratio").value)
        self.hsv_red_h1_low  = int(self.get_parameter("hsv_red_h1_low").value)
        self.hsv_red_h1_high = int(self.get_parameter("hsv_red_h1_high").value)
        self.hsv_red_h2_low  = int(self.get_parameter("hsv_red_h2_low").value)
        self.hsv_red_h2_high = int(self.get_parameter("hsv_red_h2_high").value)
        self.hsv_green_h_low  = int(self.get_parameter("hsv_green_h_low").value)
        self.hsv_green_h_high = int(self.get_parameter("hsv_green_h_high").value)
        self.hsv_sat_low = int(self.get_parameter("hsv_sat_low").value)
        self.hsv_val_low = int(self.get_parameter("hsv_val_low").value)
        self.tl_draw_roi = bool(self.get_parameter("tl_draw_roi").value)
        self.tl_draw_fps = bool(self.get_parameter("tl_draw_fps").value)

        # ── 어안 왜곡 보정 초기화 ──
        self.enable_undistort = bool(self.get_parameter("enable_undistort").value)
        self.undistort_alpha  = float(self.get_parameter("undistort_alpha").value)

        if self.enable_undistort:
            fx, fy = 956.30137, 962.44368
            cx, cy = 979.72871, 531.00886
            k1, k2 = -0.276622, 0.050981
            p1, p2 = 0.000303, -0.001998
            k3     = 0.0
            self._undist_K = np.array([
                [fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0],
            ], dtype=np.float64)
            self._undist_D = np.array([k1, k2, p1, p2, k3], dtype=np.float64)
            img_w, img_h = 1920, 1080
            self._undist_new_K, self._undist_roi = \
                cv2.getOptimalNewCameraMatrix(
                    self._undist_K, self._undist_D,
                    (img_w, img_h), self.undistort_alpha, (img_w, img_h))
            self._undist_map1, self._undist_map2 = \
                cv2.initUndistortRectifyMap(
                    self._undist_K, self._undist_D, None,
                    self._undist_new_K, (img_w, img_h), cv2.CV_16SC2)
            rx, ry, rw, rh = self._undist_roi
            self.get_logger().info(
                f"어안 왜곡 보정 활성화 | alpha={self.undistort_alpha}, "
                f"ROI=({rx},{ry},{rw},{rh})")

        if self.show_window and "cuda" in self.device.lower():
            self.get_logger().warn(
                "show_window=True + GPU 모드: cv2.imshow()가 성능을 저하시킬 수 있습니다.")

        # =============================
        # 모델 로드
        # =============================
        self.model_lane = YOLO(
            str(self.get_parameter("lane_weights_roi").value), task="segment")
        if self.model_half_precision:
            self.get_logger().info("FP16 half precision 모드 활성화")

        self.model_tl  = YOLO(str(self.get_parameter("tl_weights_roi").value))
        self.TL_ALLOW  = {0, 1}
        self.TL_LABEL  = {0: "GREEN", 1: "RED"}

        # =============================
        # 런타임 상태
        # =============================
        self.frame_count        = 0
        self.last_lane_segments = []
        self._fps               = 0.0
        self._last_fps_t        = time.monotonic()

        # TL states
        self.tl_frame_count  = 0
        self.last_tl_boxes   = []
        self.tl_fps          = 0.0
        self.tl_last_fps_t   = time.monotonic()

        # Sliding Window / polyfit 결과 캐시
        self.last_left_fit  = None
        self.last_right_fit = None
        self.last_raw_left_fit  = None   # EMA 전 raw fit (search_around_poly 탐색 기준)
        self.last_raw_right_fit = None
        self.last_confidence = 0.0
        self.last_sw_debug  = None
        self._fit_lost_count = 0   # 연속 LOST 프레임 카운터

        self.last_stop_line_dist = -1.0 # 정지선 거리 캐시 추가
        self.last_crosswalk_dist = -1.0 # 횡단보도 거리 캐시 추가

        # 시각화 전용 centerline 캐시
        # [v7-D] 반폭 초기값을 lane_width_m에서 자동 계산 (judgment와 동일 기준).
        #   auto_half_width=False면 기존 하드코딩(bev_w×0.28) 사용.
        if self.auto_half_width and self.pixel_to_meter_bev > 0:
            self.measured_half_width = (self.lane_width_m * 0.5) / self.pixel_to_meter_bev
            self.get_logger().info(
                f"[v7-D] auto_half_width ON: lane_width={self.lane_width_m:.2f}m "
                f"→ 반폭 초기값 {self.measured_half_width:.0f}px "
                f"(ratio={self.measured_half_width/self.bev_w:.3f})")
        else:
            self.measured_half_width = self.bev_w * 0.28
            self.get_logger().info(
                f"[v7-D] auto_half_width OFF: 반폭 초기값 {self.measured_half_width:.0f}px (bev_w×0.28)")
        # [v7-C] 반폭 폴백 초기값 기록 + 실측 대비 괴리 경고용 상태
        self._hw_fallback_init   = self.measured_half_width   # 초기 폴백값(비교 기준)
        self._hw_last_warn_t     = 0.0                        # 경고 rate-limit 타임스탬프
        self.last_viz = {
            "confidence": 0.0,
            "xb": 0, "yb": 0, "xl": 0, "yl": 0,
        }
        self._viz_target_stamp  = 0.0    # judgment viz_target 마지막 수신 시각
        self._viz_from_judgment = False   # judgment 연결 여부

        # judgment 디버그 상태 수신 (표시 전용)
        self._judgment_state = "N/A"

        # =============================
        # ROS 인터페이스
        # =============================
        qos_img = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        self.bridge    = CvBridge()
        self.pub_debug = self.create_publisher(Image, "/lane/debug", qos_img)
        self.pub_lane_state = self.create_publisher(
            Float32MultiArray, "/lane/state", qos)
        self.pub_tl_state = self.create_publisher(
            String, "/tl/state", qos)
        
        # 정지선 거리 퍼블리셔 추가
        self.pub_stop_line = self.create_publisher(
            Float32, "/stop_line_dist", qos)

        # 횡단보도 거리 퍼블리셔 추가
        self.pub_crosswalk = self.create_publisher(
            Float32, "/crosswalk_dist", qos)

        self.create_subscription(Image, self.image_topic, self.cb, qos_img)

        # judgment가 publish하는 실제 제어 목표점 수신 (시각화 동기화용)
        self.create_subscription(
            Float32MultiArray, "/lane/viz_target", self._viz_target_cb, qos)

        # judgment 상태 수신 (시각화 전용 — publish_debug_info=True일 때만 동작)
        self.create_subscription(
            String, "/judgment_state", self._judgment_state_cb, qos)

        if self.show_window:
            cv2.namedWindow("Lane",          cv2.WINDOW_NORMAL)
            cv2.namedWindow("BEV",           cv2.WINDOW_NORMAL)
            cv2.namedWindow("Traffic Light", cv2.WINDOW_NORMAL)
            if self.show_bev_lane_only:
                cv2.namedWindow("Sliding Window", cv2.WINDOW_NORMAL)
            if self.bev_calibration_mode:
                cv2.setMouseCallback("BEV", self._bev_mouse_cb)
                self.get_logger().info(
                    f"[BEV 캘리브 모드 ON] 기준 거리: {self.bev_calib_real_distance_m:.3f}m")

        # ============================================================
        # [추가] 시각화 창 녹화 (창별 webm, 백그라운드 스레드)
        #   record_video : 녹화 on/off
        #   record_fps   : 저장 fps(카메라 발행 속도에 맞출 것)
        #   record_dir   : 저장 폴더
        # ============================================================
        #   record_only_when_driving : True=실제 주행(/control_state=True) 중에만 녹화
        self.declare_parameter("record_video", True)
        self.declare_parameter("record_fps",   25.0)
        self.declare_parameter("record_dir",   "/home/mad2/records")
        self.declare_parameter("record_max_width", 640)   # 0=원본, 권장 640
        self.declare_parameter("record_only_when_driving", True)
        self._rec = None
        self.record_only_when_driving = bool(
            self.get_parameter("record_only_when_driving").value)
        if bool(self.get_parameter("record_video").value):
            self._rec = _ThreadedWebmRecorder(
                out_dir=str(self.get_parameter("record_dir").value),
                fps=float(self.get_parameter("record_fps").value),
                max_width=int(self.get_parameter("record_max_width").value),
                logger=self.get_logger())
            if self.record_only_when_driving:
                # /control_state = 모터 권한 = "실제 주행 중". motor.py 가 이 값으로
                # 바퀴를 풀고 잠근다. True ← prompt._drive_flow / driving.load_waypoints,
                # False ← prompt 긴급정지·종료 / driving.instant_stop.
                self.create_subscription(
                    Bool, "/control_state", self._control_state_cb, 10)
                self.get_logger().info(
                    "[REC] 주행 연동 녹화 — /control_state=True 구간만 기록")
            else:
                # 예전 동작(노드 살아있는 내내 녹화)
                self._rec.start_session()

        self.get_logger().info(
            f"LaneLine v14+ (hybrid_search_around_poly+fit_plausible+parallel_constraint+viz_sync) started | "
            f"undistort={self.enable_undistort}, "
            f"lane_interval={self.lane_interval}, "
            f"sw={self.sw_num_windows}win/{self.sw_margin}margin/{self.sw_minpix}minpix, "
            f"polyfit_deg={self.polyfit_degree}, "
            f"ema_alpha={self.fit_ema_alpha}, sanity={self.sanity_enable}, "
            f"bev_crop_y=[{self.bev_crop_y_min},{self.bev_crop_y_max}], "
            f"pub=[aL,bL,cL,aR,bR,cR,conf,hw] (8 floats)")

    # ============================================================
    # Traffic Light 유틸리티
    # ============================================================
    def _clamp_roi(self, w, h, xmin, ymin, xmax, ymax):
        xmin = max(0, min(int(xmin), w - 1))
        xmax = max(0, min(int(xmax), w))
        ymin = max(0, min(int(ymin), h - 1))
        ymax = max(0, min(int(ymax), h))
        if xmax <= xmin or ymax <= ymin:
            return 0, 0, w, h
        return xmin, ymin, xmax, ymax

    def _central_crop(self, img, ratio=0.8):
        if img is None or img.size == 0:
            return img
        h, w = img.shape[:2]
        ratio = max(0.2, min(1.0, ratio))
        nw, nh = int(w * ratio), int(h * ratio)
        x1, y1 = max(0, (w - nw) // 2), max(0, (h - nh) // 2)
        return img[y1:y1 + nh, x1:x1 + nw]

    def _get_signal_state_hsv(self, crop_bgr):
        if crop_bgr is None or crop_bgr.size == 0:
            return "UNKNOWN", 0, 0
        crop = self._central_crop(crop_bgr, self.hsv_crop_center_ratio)
        hsv  = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)

        lower_red1  = np.array([self.hsv_red_h1_low,  self.hsv_sat_low, self.hsv_val_low], dtype=np.uint8)
        upper_red1  = np.array([self.hsv_red_h1_high, 255, 255], dtype=np.uint8)
        lower_red2  = np.array([self.hsv_red_h2_low,  self.hsv_sat_low, self.hsv_val_low], dtype=np.uint8)
        upper_red2  = np.array([self.hsv_red_h2_high, 255, 255], dtype=np.uint8)
        lower_green = np.array([self.hsv_green_h_low, self.hsv_sat_low, self.hsv_val_low], dtype=np.uint8)
        upper_green = np.array([self.hsv_green_h_high, 255, 255], dtype=np.uint8)

        mask_r = cv2.bitwise_or(
            cv2.inRange(hsv, lower_red1, upper_red1),
            cv2.inRange(hsv, lower_red2, upper_red2))
        mask_g = cv2.inRange(hsv, lower_green, upper_green)

        kernel = np.ones((3, 3), np.uint8)
        mask_r = cv2.morphologyEx(mask_r, cv2.MORPH_OPEN, kernel)
        mask_g = cv2.morphologyEx(mask_g, cv2.MORPH_OPEN, kernel)

        r_pixels = cv2.countNonZero(mask_r)
        g_pixels = cv2.countNonZero(mask_g)

        if r_pixels > g_pixels and r_pixels >= self.hsv_min_color_pixels:
            return "RED", r_pixels, g_pixels
        if g_pixels > r_pixels and g_pixels >= self.hsv_min_color_pixels:
            return "GREEN", r_pixels, g_pixels
        return "UNKNOWN", r_pixels, g_pixels

    def _all_tl_boxes(self, result, tl_roi_img):
        out = []
        if result is None or result.boxes is None or len(result.boxes) == 0:
            return out
        for b in result.boxes:
            conf   = float(b.conf[0].item()) if hasattr(b, "conf") else 0.0
            cls_id = int(b.cls[0].item())    if hasattr(b, "cls")  else -1
            if cls_id not in self.TL_ALLOW or conf < self.tl_conf:
                continue
            x1, y1, x2, y2 = [int(v) for v in b.xyxy[0].tolist()]
            bw     = max(1, x2 - x1)
            bh     = max(1, y2 - y1)
            area   = bw * bh
            aspect = bw / float(bh)
            if area   < self.tl_min_area   or area   > self.tl_max_area:   continue
            if aspect < self.tl_min_aspect or aspect > self.tl_max_aspect: continue

            final_label = self.TL_LABEL.get(cls_id, str(cls_id))
            hsv_red = hsv_green = 0
            if self.use_hsv_refine and conf < HSV_FALLBACK_CONF:
                crop = tl_roi_img[max(0, y1):max(0, y2), max(0, x1):max(0, x2)]
                hsv_label, hsv_red, hsv_green = self._get_signal_state_hsv(crop)
                if hsv_label in ("RED", "GREEN"):
                    final_label = hsv_label

            out.append({
                "label": final_label, "conf": conf,
                "box": (x1, y1, x2, y2),
                "box_h": bh,
                "hsv_red": hsv_red, "hsv_green": hsv_green,
            })
        return out

    def _resolve_tl_state(self, boxes):
        if not boxes:
            return "UNKNOWN"

        red_boxes = [b for b in boxes if b["label"] == "RED"]
        if red_boxes:
            if any(b.get("box_h", 0) >= self.tl_red_stop_min_height for b in red_boxes):
                return "RED"
            return "RED_FAR"

        labels = [b["label"] for b in boxes]
        if "GREEN" in labels: return "GREEN"
        return "UNKNOWN"

    # ============================================================
    # BEV 캘리브레이션 마우스 콜백
    # ============================================================
    def _bev_mouse_cb(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        self._calib_clicks.append((x, y))
        n = len(self._calib_clicks)
        self.get_logger().info(f"[BEV 캘리브] 점 {n}: ({x}, {y})")
        if n >= 2:
            p1, p2 = self._calib_clicks[-2], self._calib_clicks[-1]
            dx, dy = p2[0] - p1[0], p2[1] - p1[1]
            pixel_dist = math.sqrt(dx * dx + dy * dy)
            real_m = self.bev_calib_real_distance_m
            if pixel_dist > 0:
                self.get_logger().info(
                    f"═══ BEV 캘리브 결과 ═══\n"
                    f"  {p1} → {p2} | {pixel_dist:.1f}px = {real_m:.3f}m\n"
                    f"  ▶ pixel_to_meter_bev = {real_m/pixel_dist:.6f}")
            self._calib_clicks.clear()

    # ============================================================
    # 좌표 변환 유틸리티 (시각화용)
    # ============================================================
    def _bev_pt_to_orig(self, pt_bev):
        pt_3d = np.array([[[pt_bev[0], pt_bev[1]]]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(pt_3d, self.M_ipm_inv)
        return (int(transformed[0][0][0]), int(transformed[0][0][1]))

    # ============================================================
    # YOLO 마스크 → BEV 바이너리 이미지 생성
    # ============================================================
    def _build_bev_binary(self, frame, segments_orig):
        h, w = frame.shape[:2]
        mask = np.zeros((h, w), dtype=np.uint8)
        
        # 1. 차선(1)만 흰색으로 칠하기
        for (pts, cls_id, conf_det) in segments_orig:
            if int(cls_id) == 1:
                pts_int = np.int32(pts).reshape((-1, 1, 2))
                cv2.fillPoly(mask, [pts_int], 255)

        # 2. 횡단보도(0)를 검은색으로 덮기 (차선과 겹치는 부분 지우기)
        for (pts, cls_id, conf_det) in segments_orig:
            if int(cls_id) == 0:
                pts_int = np.int32(pts).reshape((-1, 1, 2))
                cv2.fillPoly(mask, [pts_int], 0)

        bev_mask = cv2.warpPerspective(
            mask, self.M_ipm, (self.bev_w, self.bev_h),
            flags=cv2.INTER_NEAREST)

        if self.bev_crop_y_min > 0:
            bev_mask[:self.bev_crop_y_min, :] = 0
        if self.bev_crop_y_max < self.bev_h:
            bev_mask[self.bev_crop_y_max:, :] = 0
        return bev_mask

    # ============================================================
    # Sliding Window 차선 검출
    # ============================================================
    def _sliding_window_detect(self, bev_binary, need_debug=False):
        num_win = self.sw_num_windows
        margin  = self.sw_margin
        minpix  = self.sw_minpix
        degree  = self.polyfit_degree
        h, w    = bev_binary.shape

        # 히스토그램
        hist_start = int(h * (1.0 - self.sw_hist_bottom_ratio))
        histogram = np.sum(bev_binary[hist_start:, :], axis=0)
        midpoint = w // 2
        leftx_base  = int(np.argmax(histogram[:midpoint]))
        rightx_base = int(np.argmax(histogram[midpoint:])) + midpoint

        left_peak  = histogram[leftx_base]  if leftx_base < midpoint  else 0
        right_peak = histogram[rightx_base] if rightx_base >= midpoint else 0
        left_valid  = left_peak  > self.sw_min_hist_peak
        right_valid = right_peak > self.sw_min_hist_peak

        nz   = bev_binary.nonzero()
        nz_y, nz_x = nz[0], nz[1]

        window_height  = h // num_win
        leftx_current  = leftx_base
        rightx_current = rightx_base
        left_lane_inds  = []
        right_lane_inds = []

        debug_img = np.zeros((h, w, 3), dtype=np.uint8) if need_debug else None

        for win_idx in range(num_win):
            win_y_low  = h - (win_idx + 1) * window_height
            win_y_high = h - win_idx * window_height

            win_xl_low  = leftx_current  - margin
            win_xl_high = leftx_current  + margin
            win_xr_low  = rightx_current - margin
            win_xr_high = rightx_current + margin

            if need_debug:
                if left_valid:
                    cv2.rectangle(debug_img,
                        (win_xl_low, win_y_low), (win_xl_high, win_y_high),
                        (0, 255, 0), 2)
                if right_valid:
                    cv2.rectangle(debug_img,
                        (win_xr_low, win_y_low), (win_xr_high, win_y_high),
                        (0, 255, 0), 2)

            good_left = (
                (nz_y >= win_y_low) & (nz_y < win_y_high) &
                (nz_x >= win_xl_low) & (nz_x < win_xl_high)
            ).nonzero()[0]
            good_right = (
                (nz_y >= win_y_low) & (nz_y < win_y_high) &
                (nz_x >= win_xr_low) & (nz_x < win_xr_high)
            ).nonzero()[0]

            if left_valid:
                left_lane_inds.append(good_left)
            if right_valid:
                right_lane_inds.append(good_right)

            if left_valid and len(good_left) > minpix:
                leftx_current = int(np.mean(nz_x[good_left]))
            if right_valid and len(good_right) > minpix:
                rightx_current = int(np.mean(nz_x[good_right]))

        # Polyfit (RankWarning을 실제로 catch)
        left_fit  = right_fit  = None
        left_npix = right_npix = 0
        left_pts  = right_pts  = None   # (ys, xs) — 평행 제약 offset 재추정용

        if left_valid and left_lane_inds:
            cat = np.concatenate(left_lane_inds)
            if len(cat) >= MIN_FIT_PIX:
                with warnings.catch_warnings():
                    warnings.simplefilter("error", _RankWarning)
                    try:
                        left_fit  = np.polyfit(nz_y[cat], nz_x[cat], degree)
                        left_npix = len(cat)
                        left_pts  = (nz_y[cat], nz_x[cat])
                    except (_RankWarning, np.linalg.LinAlgError, ValueError):
                        left_fit = None
                if need_debug:
                    debug_img[nz_y[cat], nz_x[cat]] = [255, 0, 0]

        if right_valid and right_lane_inds:
            cat = np.concatenate(right_lane_inds)
            if len(cat) >= MIN_FIT_PIX:
                with warnings.catch_warnings():
                    warnings.simplefilter("error", _RankWarning)
                    try:
                        right_fit  = np.polyfit(nz_y[cat], nz_x[cat], degree)
                        right_npix = len(cat)
                        right_pts  = (nz_y[cat], nz_x[cat])
                    except (_RankWarning, np.linalg.LinAlgError, ValueError):
                        right_fit = None
                if need_debug:
                    debug_img[nz_y[cat], nz_x[cat]] = [0, 0, 255]

        # 디버그: polyfit 곡선 그리기
        if need_debug:
            plot_y = np.linspace(0, h - 1, h)
            if left_fit is not None:
                fx = np.clip(np.polyval(left_fit, plot_y), 0, w - 1).astype(int)
                cv2.polylines(debug_img,
                    [np.column_stack((fx, plot_y.astype(int)))],
                    False, (255, 255, 0), 1)
            if right_fit is not None:
                fx = np.clip(np.polyval(right_fit, plot_y), 0, w - 1).astype(int)
                cv2.polylines(debug_img,
                    [np.column_stack((fx, plot_y.astype(int)))],
                    False, (0, 255, 255), 1)

        return left_fit, right_fit, left_npix, right_npix, debug_img, left_pts, right_pts

    # ============================================================
    # Search Around Poly (이전 fit 주변 탐색)
    # ============================================================
    def _search_around_poly(self, bev_binary, prev_left_fit, prev_right_fit,
                            need_debug=False):
        """
        이전 프레임의 polyfit 곡선 ± margin 영역에서만 픽셀을 탐색.
        점선/횡단보도 구간에서 히스토그램 오염 없이 안정적 추적 가능.
        """
        margin = self.sw_margin
        degree = self.polyfit_degree
        h, w   = bev_binary.shape

        nz = bev_binary.nonzero()
        nz_y, nz_x = nz[0], nz[1]

        new_left_fit  = None
        new_right_fit = None
        l_npix = r_npix = 0
        l_pts  = r_pts  = None   # (ys, xs) — 평행 제약 offset 재추정용

        debug_img = np.zeros((h, w, 3), dtype=np.uint8) if need_debug else None

        if prev_left_fit is not None and len(nz_y) > 0:
            left_center = np.polyval(prev_left_fit, nz_y)
            left_inds = (
                (nz_x > left_center - margin) &
                (nz_x < left_center + margin)
            ).nonzero()[0]

            if len(left_inds) >= MIN_FIT_PIX:
                with warnings.catch_warnings():
                    warnings.simplefilter("error", _RankWarning)
                    try:
                        new_left_fit = np.polyfit(
                            nz_y[left_inds], nz_x[left_inds], degree)
                        l_npix = len(left_inds)
                        l_pts  = (nz_y[left_inds], nz_x[left_inds])
                    except (_RankWarning, np.linalg.LinAlgError, ValueError):
                        new_left_fit = None
                if need_debug:
                    debug_img[nz_y[left_inds], nz_x[left_inds]] = [255, 0, 0]

        if prev_right_fit is not None and len(nz_y) > 0:
            right_center = np.polyval(prev_right_fit, nz_y)
            right_inds = (
                (nz_x > right_center - margin) &
                (nz_x < right_center + margin)
            ).nonzero()[0]

            if len(right_inds) >= MIN_FIT_PIX:
                with warnings.catch_warnings():
                    warnings.simplefilter("error", _RankWarning)
                    try:
                        new_right_fit = np.polyfit(
                            nz_y[right_inds], nz_x[right_inds], degree)
                        r_npix = len(right_inds)
                        r_pts  = (nz_y[right_inds], nz_x[right_inds])
                    except (_RankWarning, np.linalg.LinAlgError, ValueError):
                        new_right_fit = None
                if need_debug:
                    debug_img[nz_y[right_inds], nz_x[right_inds]] = [0, 0, 255]

        # 디버그: polyfit 곡선 + search band 시각화 (벡터화)
        if need_debug:
            plot_y = np.linspace(0, h - 1, h)
            ys_int = plot_y.astype(int)

            if prev_left_fit is not None:
                band_x = np.clip(np.polyval(prev_left_fit, plot_y), 0, w - 1).astype(int)
                left_edge  = np.clip(band_x - margin, 0, w - 1)
                right_edge = np.clip(band_x + margin, 0, w - 1)
                cv2.polylines(debug_img,
                    [np.column_stack((left_edge, ys_int))],
                    False, (0, 100, 0), 1)
                cv2.polylines(debug_img,
                    [np.column_stack((right_edge, ys_int))],
                    False, (0, 100, 0), 1)

            if prev_right_fit is not None:
                band_x = np.clip(np.polyval(prev_right_fit, plot_y), 0, w - 1).astype(int)
                left_edge  = np.clip(band_x - margin, 0, w - 1)
                right_edge = np.clip(band_x + margin, 0, w - 1)
                cv2.polylines(debug_img,
                    [np.column_stack((left_edge, ys_int))],
                    False, (0, 100, 0), 1)
                cv2.polylines(debug_img,
                    [np.column_stack((right_edge, ys_int))],
                    False, (0, 100, 0), 1)

            if new_left_fit is not None:
                fx = np.clip(np.polyval(new_left_fit, plot_y), 0, w - 1).astype(int)
                cv2.polylines(debug_img,
                    [np.column_stack((fx, ys_int))],
                    False, (255, 255, 0), 1)
            if new_right_fit is not None:
                fx = np.clip(np.polyval(new_right_fit, plot_y), 0, w - 1).astype(int)
                cv2.polylines(debug_img,
                    [np.column_stack((fx, ys_int))],
                    False, (0, 255, 255), 1)

        return new_left_fit, new_right_fit, l_npix, r_npix, debug_img, l_pts, r_pts

    # ============================================================
    # v13: Sanity Check / EMA / Soft Confidence
    # ============================================================
    def _sanity_check_pair(self, lf, rf):
        """
        명백히 말이 안 되는 fit만 거른다 (미니멀).
        - 좌우 역전 (rxs < lxs)
        - 차선폭이 극단적으로 좁음 (같은 차선 중복 검출 / 가드레일 오인)
        - 기울기 발산 (V자 / ⋀자 — 한쪽이 잘못 잡힌 시그널)

        평행성(폭 std) / 폭 상한 검사는 빼버림 — IPM 캘리브가 맡을 일.
        실패 시 호출자에서 둘 다 LOST 처리 → 다음 프레임에서 fresh re-detection.
        """
        if lf is None or rf is None:
            return True, None

        h, w = self.bev_h, self.bev_w
        y0 = max(self.bev_crop_y_min, int(h * 0.5))
        y1 = min(self.bev_crop_y_max - 1, h - 1)
        if y1 <= y0:
            return True, None

        ys = np.linspace(y0, y1, 5)
        lxs = np.polyval(lf, ys)
        rxs = np.polyval(rf, ys)
        widths = rxs - lxs

        # 1) 좌우 역전 (노이즈 마진 -10px 허용)
        if np.any(widths < -10):
            return False, "both"

        # 2) 폭 극단적 하한만 검사 (상한은 안 봄)
        mean_w = float(np.mean(widths))
        if mean_w < w * self.sanity_width_min_ratio:
            return False, "both"

        # 3) 기울기 발산 검사 (양쪽 모두 의미 있는 기울기를 가지고 부호가 반대)
        dy = float(y1 - y0)
        l_slope = (lxs[-1] - lxs[0]) / dy
        r_slope = (rxs[-1] - rxs[0]) / dy
        if abs(l_slope) > 0.3 and abs(r_slope) > 0.3 and l_slope * r_slope < 0:
            return False, "both"

        return True, None

    @staticmethod
    def _pts_yspan(pts):
        """pts=(ys, xs) 의 y 방향 span(px). 없으면 0."""
        if pts is None or len(pts[0]) == 0:
            return 0.0
        ys = pts[0]
        return float(ys.max() - ys.min())

    def _fit_plausible(self, fit, prev_fit):
        """
        [우선순위2] 개별 fit 타당성 게이트 (pair sanity보다 먼저).
        한쪽 fit이 화면을 가로지르는 ghost 곡선(점선 조각/횡단보도 경계 오염)이어도
        반대쪽이 없거나 기울기 조건을 빗겨가면 pair sanity를 통과하는 경우를 막는다.
        - 크롭 구간에서 화면 밖으로 크게 벗어나는 fit 기각
        - 2차항 |a| 상한(max_fit_curvature) 초과 기각
        - 직전 raw fit 대비 하단 x 급점프가 sw_margin×2 초과 시 기각 (스파이크 차단)
        """
        if fit is None:
            return False
        y0 = max(0, self.bev_crop_y_min)
        y1 = min(self.bev_h - 1, self.bev_crop_y_max - 1)
        ys = np.linspace(y0, y1, 5)
        xs = np.polyval(fit, ys)
        if np.any(xs < -0.3 * self.bev_w) or np.any(xs > 1.3 * self.bev_w):
            return False
        if abs(float(fit[0])) > self.max_fit_curvature:
            return False
        if prev_fit is not None:
            yb = self.bev_h * self.bottom_ratio
            jump = abs(float(np.polyval(fit, yb)) - float(np.polyval(prev_fit, yb)))
            if jump > self.sw_margin * 2.0:
                return False
        return True

    def _apply_parallel_constraint(self, lf, rf, l_pts, r_pts, l_npix, r_npix):
        """
        [우선순위1] 점선 곡률 튐 방지 → 평행 차선 제약.
        좌우 차선은 물리적으로 평행하다. 신뢰도(픽셀수×y-span)가 높은 쪽을 실선(신뢰)으로,
        낮은 쪽을 점선으로 보고 실선의 곡률(a,b)을 점선 쪽에 이식, offset(c)만 dash 픽셀로
        재추정한다. → 점선 쪽 곡선이 실선과 나란히 움직여 인코드 튐/조향 흔들림 차단.
        - 양쪽 fit이 모두 있을 때만 (한쪽만이면 measured_half_width 폴백이 담당).
        - 약한 쪽 y-span이 (크롭높이×yspan_ratio) 이상이면 둘 다 실선 → 개입 안 함.
        - npix는 손대지 않는다 (confidence는 실제 검출 픽셀 기준 유지).
        """
        if not self.parallel_constraint_enable:
            return lf, rf
        if lf is None or rf is None or self.polyfit_degree < 2:
            return lf, rf

        crop_h      = max(1, self.bev_crop_y_max - self.bev_crop_y_min)
        span_thresh = self.parallel_yspan_ratio * crop_h

        l_span = self._pts_yspan(l_pts)
        r_span = self._pts_yspan(r_pts)

        # 연속성 지표(픽셀수×y-span)가 큰 쪽=실선(신뢰), 작은 쪽=점선.
        l_score = l_npix * max(1.0, l_span)
        r_score = r_npix * max(1.0, r_span)

        if l_score >= r_score:
            strong_fit, weak_span, weak_pts, weak_side = lf, r_span, r_pts, "R"
        else:
            strong_fit, weak_span, weak_pts, weak_side = rf, l_span, l_pts, "L"

        # 약한 쪽도 충분히 길면(둘 다 실선급) 개입하지 않음
        if weak_span >= span_thresh:
            return lf, rf

        a, b = float(strong_fit[0]), float(strong_fit[1])

        # offset c: dash 픽셀이 충분하면 그걸로(중앙값=outlier 견고), 아니면 기존 fit 하단 앵커.
        if weak_pts is not None and len(weak_pts[0]) >= self.parallel_min_pts:
            wy = weak_pts[0].astype(np.float64)
            wx = weak_pts[1].astype(np.float64)
            c = float(np.median(wx - (a * wy * wy + b * wy)))
        else:
            weak_fit = rf if weak_side == "R" else lf
            yb = self.bev_h * self.bottom_ratio
            c = float(np.polyval(weak_fit, yb) - (a * yb * yb + b * yb))

        new_weak = np.array([a, b, c], dtype=np.float64)
        if weak_side == "R":
            return lf, new_weak
        return new_weak, rf

    def _ema_fit(self, new_fit, prev_fit):
        """새 fit을 이전 fit과 EMA 블렌딩. new_fit이 None이면 prev 유지."""
        a = self.fit_ema_alpha
        if new_fit is None:
            return prev_fit
        if prev_fit is None or a >= 1.0:
            return np.array(new_fit, dtype=np.float64)
        return a * np.array(new_fit, dtype=np.float64) + \
               (1.0 - a) * np.array(prev_fit, dtype=np.float64)

    def _soft_confidence(self, lf, rf, l_npix, r_npix):
        """
        픽셀 수 기반 연속 confidence.
        - BOTH : 0.5 + 0.5 * avg(pconf)  → 0.5 ~ 1.0
        - ONLY : 0.25 * max(pconf)       → 0.0 ~ 0.25
        - LOST : 0.0
        judgment에서 0.5 이상=BOTH, 0~0.25=ONLY, 0=LOST 로 구분 가능.
        """
        def _pc(n):
            return min(1.0, n / self.conf_pix_saturate) if self.conf_pix_saturate > 0 else 1.0

        lp = _pc(l_npix) if lf is not None else 0.0
        rp = _pc(r_npix) if rf is not None else 0.0

        if lf is not None and rf is not None:
            return 0.5 + 0.5 * (lp + rp) / 2.0
        if lf is not None or rf is not None:
            return 0.25 * max(lp, rp)
        return 0.0

    # ============================================================
    # judgment 제어 목표점 수신 (시각화 동기화)
    # ============================================================
    def _viz_target_cb(self, msg: Float32MultiArray):
        """judgment가 실제 사용하는 centerline 목표점 + 제어값을 수신하여 시각화에 반영."""
        if len(msg.data) >= 5:
            self.last_viz = {
                "xb": float(msg.data[0]),
                "yb": float(msg.data[1]),
                "xl": float(msg.data[2]),
                "yl": float(msg.data[3]),
                "confidence": float(msg.data[4]),
            }
            # 확장 필드 (하위호환 — judgment v8+에서 7개 전송)
            if len(msg.data) >= 7:
                self.last_viz["steer"] = float(msg.data[5])
                self.last_viz["speed"] = float(msg.data[6])
            self._viz_target_stamp  = time.monotonic()
            self._viz_from_judgment = True

    def _judgment_state_cb(self, msg: String):
        """judgment 노드의 FSM 상태를 수신 (시각화 표시 전용)."""
        self._judgment_state = msg.data

    def _control_state_cb(self, msg: Bool):
        """[녹화] 모터 권한 = 주행 여부. 주행 시작에 세션을 열고 정지에 닫는다.
        start/stop 둘 다 멱등이라 중복 발행(prompt+driving)은 그대로 흘려보낸다."""
        if self._rec is None:
            return
        if msg.data:
            self._rec.start_session()
        else:
            self._rec.stop_session()

    # ============================================================
    # 시각화 전용 centerline 계산 (fallback — judgment 미연결 시)
    # ============================================================
    def _compute_centerline_for_viz(self, left_fit, right_fit):
        """
        시각화 + judgment 적응형 반폭 계산용.
        judgment가 최근 200ms 이내 값을 보내고 있으면 centerline 계산은 건너뛰고
        measured_half_width 갱신만 수행한다 (publish용).
        """
        # measured_half_width는 항상 갱신 (ONLY 모드 + publish용)
        self._update_measured_half_width(left_fit, right_fit)

        # judgment가 최근 200ms 이내 값을 보내고 있으면 fallback 계산 건너뜀
        if self._viz_from_judgment:
            if (time.monotonic() - self._viz_target_stamp) < 0.2:
                return
            else:
                self._viz_from_judgment = False  # judgment 응답 끊김 → fallback 전환

        y_bottom    = int(self.bev_h * self.bottom_ratio)
        y_lookahead = int(self.bev_h * self.lookahead_ratio)

        left_exists  = left_fit is not None
        right_exists = right_fit is not None

        xb = xl = None

        if left_exists and right_exists:
            xb = 0.5 * (np.polyval(left_fit, y_bottom) +
                         np.polyval(right_fit, y_bottom))
            xl = 0.5 * (np.polyval(left_fit, y_lookahead) +
                         np.polyval(right_fit, y_lookahead))

        elif left_exists:
            hw = self.measured_half_width
            xb = np.polyval(left_fit, y_bottom)    + hw
            xl = np.polyval(left_fit, y_lookahead) + hw

        elif right_exists:
            hw = self.measured_half_width
            xb = np.polyval(right_fit, y_bottom)    - hw
            xl = np.polyval(right_fit, y_lookahead) - hw

        if xb is not None and xl is not None:
            self.last_viz = {
                "confidence": self.last_confidence,
                "xb": float(xb), "yb": y_bottom,
                "xl": float(xl), "yl": y_lookahead,
            }
        else:
            self.last_viz["confidence"] = 0.0

    def _update_measured_half_width(self, left_fit, right_fit):
        """BOTH 모드에서 측정된 반폭을 EMA로 갱신한다."""
        if left_fit is not None and right_fit is not None:
            y_bottom = int(self.bev_h * self.bottom_ratio)
            # 수평 간격 (같은 y에서의 x 차이)
            dx = (np.polyval(right_fit, y_bottom) -
                  np.polyval(left_fit,  y_bottom))
            # 두 차선 평균 기울기 m = dx/dy 로 법선(수직) 방향 보정.
            # 곡선/기울어진 차선에서 수평 간격은 실제 차선폭을 sqrt(1+m^2)배 과대평가하므로
            # cos(theta) = 1/sqrt(1+m^2) 를 곱해 실제 수직 반폭으로 환산한다.
            m = 0.5 * (np.polyval(np.polyder(left_fit),  y_bottom) +
                       np.polyval(np.polyder(right_fit), y_bottom))
            cos_t = 1.0 / np.sqrt(1.0 + m * m)
            measured = 0.5 * dx * cos_t

            # 범위 필터 + 급변 완화
            # [v7-C 옵션2] 기존: 이전 값 대비 20% 이상 점프면 통째로 버림
            #   → 폴백 초기값이 실제와 20% 이상 어긋나면 영영 갱신 못 하는 정체 발생.
            #   변경: 버리지 않고 ±20%로 클램프해 EMA에 조금씩 반영 → 서서히 실측으로 수렴.
            #   (알고리즘 골자인 EMA·법선보정·범위필터는 유지, "버림"만 "클램프"로 교체)
            if 30 < measured < self.bev_w * 0.5:
                if self.measured_half_width > 0:
                    lo = self.measured_half_width * 0.8
                    hi = self.measured_half_width * 1.2
                    measured_clamped = min(hi, max(lo, measured))
                    self.measured_half_width = (
                        0.9 * self.measured_half_width + 0.1 * measured_clamped)

                    # [v7-C] 초기 폴백값과 실측이 20% 이상 벌어진 채 유지되면 경고(3초 간격)
                    if self._hw_fallback_init > 0:
                        gap = abs(self.measured_half_width - self._hw_fallback_init) \
                              / self._hw_fallback_init
                        if gap > 0.2:
                            now_w = time.monotonic()
                            if now_w - self._hw_last_warn_t > 3.0:
                                self.get_logger().warn(
                                    f"[v7-C] 반폭 실측 {self.measured_half_width:.0f}px 이 "
                                    f"초기 폴백 {self._hw_fallback_init:.0f}px 대비 {gap*100:.0f}% 괴리. "
                                    f"single_lane_half_width_ratio 를 "
                                    f"{self.measured_half_width/self.bev_w:.3f} 부근으로 조정 권장")
                                self._hw_last_warn_t = now_w
                else:
                    self.measured_half_width = measured

    # ============================================================
    # Traffic Light 처리
    # ============================================================
    def _process_tl(self, frame):
        self.tl_frame_count += 1
        run_tl = (self.tl_frame_count % self.tl_interval == 0)

        h, w = frame.shape[:2]
        txmin, tymin, txmax, tymax = self._clamp_roi(w, h, *self.tl_roi)

        tl_roi_img = frame[tymin:tymax, txmin:txmax]

        tl_boxes = []
        if tl_roi_img.size != 0:
            if run_tl:
                try:
                    res_tl = self.model_tl.predict(
                        source=tl_roi_img.copy(),
                        conf=self.tl_conf,
                        imgsz=self.tl_imgsz,
                        device=self.device,
                        verbose=False,
                    )[0]
                    tl_boxes = self._all_tl_boxes(res_tl, tl_roi_img)
                    self.last_tl_boxes = tl_boxes
                except Exception as e:
                    self.get_logger().error(f"YOLO TL error: {e}")
                    self.last_tl_boxes = []
            else:
                tl_boxes = self.last_tl_boxes

        # 퍼블리시는 항상 수행
        resolved_state = self._resolve_tl_state(tl_boxes)
        state_msg      = String()
        state_msg.data = resolved_state
        self.pub_tl_state.publish(state_msg)

        # FPS 측정 (추론 프레임에서만)
        if run_tl:
            now = time.monotonic()
            dt  = max(1e-6, now - self.tl_last_fps_t)
            inst_fps = 1.0 / dt
            self.tl_fps = 0.9 * self.tl_fps + 0.1 * inst_fps if self.tl_fps > 0 else inst_fps
            self.tl_last_fps_t = now

        # 시각화는 창을 켰거나 녹화 중일 때만 (그 외에는 frame 복사 절약)
        if self.show_window or (self._rec is not None and self._rec.is_active()):
            debug_frame = frame.copy()

            if self.tl_draw_roi:
                cv2.rectangle(debug_frame, (txmin, tymin), (txmax, tymax), (0, 255, 255), 2)

            for item in tl_boxes:
                label = item["label"]
                conf  = item["conf"]
                x1, y1, x2, y2 = item["box"]
                color = (0, 0, 255) if label == "RED" else (0, 255, 0) if label == "GREEN" else (150, 150, 150)
                cv2.rectangle(debug_frame,
                    (x1 + txmin, y1 + tymin), (x2 + txmin, y2 + tymin), color, 2)
                cv2.putText(debug_frame, f"{label} {conf:.2f}",
                    (x1 + txmin, max(0, y1 + tymin - 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            if self.tl_draw_fps:
                cv2.putText(debug_frame, f"FPS: {self.tl_fps:.1f}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            # 종합 판정 상태 표시
            res_color = ((0,0,255) if "RED" in resolved_state
                         else (0,255,0) if resolved_state == "GREEN"
                         else (150,150,150))
            cv2.putText(debug_frame, f"STATE: {resolved_state}", (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, res_color, 2)

            if self.show_window:
                cv2.imshow("Traffic Light", debug_frame)
            if self._rec: self._rec.write("TrafficLight", debug_frame)

    # ============================================================
    # 메인 콜백
    # ============================================================
    def cb(self, msg: Image) -> None:
        self.frame_count += 1
        run_lane = (self.frame_count % self.lane_interval == 0)

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        # 어안 왜곡 보정
        if self.enable_undistort:
            frame = cv2.remap(
                frame, self._undist_map1, self._undist_map2,
                interpolation=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT)
            rx, ry, rw, rh = self._undist_roi
            if rw > 0 and rh > 0:
                frame = frame[ry:ry+rh, rx:rx+rw]
                frame = cv2.resize(frame, (1920, 1080),
                                   interpolation=cv2.INTER_LINEAR)

        raw_frame = frame.copy()

        lxmin, lymin, lxmax, lymax = self.lane_roi
        lane_roi_img = frame[lymin:lymax, lxmin:lxmax]
        # 녹화 중이면 창이 없어도 시각화를 그려야 한다 — 아래 3.시각화 블록이
        # bev_img/오버레이를 만들고 그 결과를 _rec.write() 가 받는다. 여기에 녹화를
        # 넣지 않으면 헤드리스 녹화가 조용히 0바이트가 된다.
        rec_on = self._rec is not None and self._rec.is_active()
        need_debug = (self.show_window
                      or self.pub_debug.get_subscription_count() > 0
                      or rec_on)

        # ============================================================
        # 1. 인지
        # ============================================================
        if run_lane:
            segments_orig = []
            if lane_roi_img.size != 0:
                res_lane = self.model_lane.predict(
                    source=lane_roi_img, conf=self.lane_conf,
                    imgsz=self.lane_imgsz, device=self.device,
                    half=self.model_half_precision, verbose=False)[0]

                if res_lane.masks is not None:
                    for i in range(len(res_lane.masks.xy)):
                        pts = np.asarray(res_lane.masks.xy[i], dtype=np.float32)
                        if len(pts) < 3:
                            continue
                        pts[:, 0] += lxmin
                        pts[:, 1] += lymin
                        conf_det = float(res_lane.boxes[i].conf[0].item())
                        cls_id   = int(res_lane.boxes[i].cls[0].item())
                        segments_orig.append((pts, cls_id, conf_det))

            self.last_lane_segments = segments_orig
            
            # ── 정지선(Stop Line) 거리 계산 ──
            current_stop_line_dist = -1.0
            for (pts, cls_id, conf_det) in segments_orig:
                if int(cls_id) == 2:
                    pts_float = np.float32(pts).reshape(-1, 1, 2)
                    bev_pts = cv2.perspectiveTransform(pts_float, self.M_ipm)
                    max_y = np.max(bev_pts[:, 0, 1])
                    stop_line_y_bev = min(int(max_y), self.bev_h)
                    ratio = 1.0 - (stop_line_y_bev / float(self.bev_h))
                    ratio = max(0.0, ratio) # 음수 방지
                    current_stop_line_dist = self.bev_calib_real_distance_m * ratio
                    break
            self.last_stop_line_dist = current_stop_line_dist

            # ── 횡단보도(Crosswalk) 거리 계산 (정지선과 동일 방식, cls 0) ──
            current_crosswalk_dist = -1.0
            for (pts, cls_id, conf_det) in segments_orig:
                if int(cls_id) == 0:
                    pts_float = np.float32(pts).reshape(-1, 1, 2)
                    bev_pts = cv2.perspectiveTransform(pts_float, self.M_ipm)
                    max_y = np.max(bev_pts[:, 0, 1])
                    crosswalk_y_bev = min(int(max_y), self.bev_h)
                    ratio = 1.0 - (crosswalk_y_bev / float(self.bev_h))
                    ratio = max(0.0, ratio) # 음수 방지
                    current_crosswalk_dist = self.bev_calib_real_distance_m * ratio
                    break
            self.last_crosswalk_dist = current_crosswalk_dist

            bev_binary = self._build_bev_binary(frame, segments_orig)

            # ── Hybrid: search_around_poly 우선, 실패 시 개별 차선 sliding window fallback ──
            use_search_around = (
                self.last_raw_left_fit is not None or
                self.last_raw_right_fit is not None
            )

            if use_search_around:
                # raw fit으로 탐색 (EMA 지연 없이 현재 위치에 가장 가까움)
                sa_left, sa_right, sa_l_npix, sa_r_npix, sa_debug, sa_l_pts, sa_r_pts = \
                    self._search_around_poly(
                        bev_binary, self.last_raw_left_fit, self.last_raw_right_fit,
                        need_debug=need_debug)

                # 한쪽이라도 못 찾으면 sliding window로 보충
                if sa_left is None or sa_right is None:
                    sw_left, sw_right, sw_l_npix, sw_r_npix, sw_debug_img, sw_l_pts, sw_r_pts = \
                        self._sliding_window_detect(bev_binary, need_debug=need_debug)

                    left_fit  = sa_left  if sa_left  is not None else sw_left
                    l_npix    = sa_l_npix if sa_left  is not None else sw_l_npix
                    l_pts     = sa_l_pts  if sa_left  is not None else sw_l_pts

                    right_fit = sa_right if sa_right is not None else sw_right
                    r_npix    = sa_r_npix if sa_right is not None else sw_r_npix
                    r_pts     = sa_r_pts  if sa_right is not None else sw_r_pts

                    sw_debug  = sa_debug
                else:
                    left_fit, right_fit = sa_left, sa_right
                    l_npix, r_npix     = sa_l_npix, sa_r_npix
                    l_pts, r_pts       = sa_l_pts, sa_r_pts
                    sw_debug           = sa_debug
            else:
                # 최초 프레임 또는 완전 LOST → sliding window로 초기화
                left_fit, right_fit, l_npix, r_npix, sw_debug, l_pts, r_pts = \
                    self._sliding_window_detect(bev_binary, need_debug=need_debug)

            self.last_sw_debug = sw_debug

            # ── [우선순위2] 개별 fit 타당성 게이트 (pair sanity보다 먼저) ──
            if left_fit is not None and not self._fit_plausible(left_fit, self.last_raw_left_fit):
                left_fit = None
                l_npix = 0
                l_pts = None
            if right_fit is not None and not self._fit_plausible(right_fit, self.last_raw_right_fit):
                right_fit = None
                r_npix = 0
                r_pts = None

            # ── Sanity Check (미니멀) ──
            # fail 시 둘 다 LOST 처리 → 다음 프레임 sliding window로 fresh re-detection
            if self.sanity_enable:
                ok, _ = self._sanity_check_pair(left_fit, right_fit)
                if not ok:
                    left_fit = None
                    right_fit = None
                    l_npix = 0
                    r_npix = 0
                    l_pts = None
                    r_pts = None
                    self.last_left_fit = None
                    self.last_right_fit = None
                    self.last_raw_left_fit = None
                    self.last_raw_right_fit = None

            # ── [우선순위1] 평행 차선 제약 (점선 곡률 튐 방지) ──
            #   신뢰도 높은 실선 쪽 곡률(a,b)을 점선 쪽에 이식, offset(c)만 재추정.
            #   raw fit 저장 전에 적용 → 다음 프레임 search 시드도 나란하게 유지.
            left_fit, right_fit = self._apply_parallel_constraint(
                left_fit, right_fit, l_pts, r_pts, l_npix, r_npix)

            # ── raw fit 저장 (출처 무관 — SA/SW 둘 다 다음 프레임 추적의 시드로 사용) ──
            if left_fit is not None:
                self.last_raw_left_fit = left_fit
            if right_fit is not None:
                self.last_raw_right_fit = right_fit

            # ── EMA 스무딩 (이전 fit과 블렌딩) ──
            left_fit  = self._ema_fit(left_fit,  self.last_left_fit)
            right_fit = self._ema_fit(right_fit, self.last_right_fit)

            # ── stale fit 영구 hold 방지 ──
            both_none = (l_npix == 0 and r_npix == 0)
            if both_none:
                self._fit_lost_count += 1
                if self._fit_lost_count >= self.fit_hold_max_frames:
                    left_fit  = None
                    right_fit = None
                    self.last_raw_left_fit  = None
                    self.last_raw_right_fit = None
            else:
                self._fit_lost_count = 0

            self.last_left_fit  = left_fit
            self.last_right_fit = right_fit

            # ── Soft Confidence ──
            self.last_confidence = self._soft_confidence(
                left_fit, right_fit, l_npix, r_npix)

            # 시각화용 centerline 계산 (fallback — judgment viz_target이 오면 덮어씀)
            self._compute_centerline_for_viz(left_fit, right_fit)

        # ============================================================
        # 2. Publish — polyfit 계수 + confidence
        # ============================================================
        lf = self.last_left_fit  if self.last_left_fit  is not None else [0.0, 0.0, 0.0]
        rf = self.last_right_fit if self.last_right_fit is not None else [0.0, 0.0, 0.0]

        state_msg = Float32MultiArray()
        state_msg.data = [
            float(lf[0]), float(lf[1]), float(lf[2]),   # aL, bL, cL
            float(rf[0]), float(rf[1]), float(rf[2]),   # aR, bR, cR
            float(self.last_confidence),                  # confidence
            float(self.measured_half_width),              # half_width (BEV px)
        ]
        self.pub_lane_state.publish(state_msg)
        
        # 정지선 거리 퍼블리시
        dist_msg = Float32()
        dist_msg.data = float(self.last_stop_line_dist)
        self.pub_stop_line.publish(dist_msg)

        # 횡단보도 거리 퍼블리시
        cw_msg = Float32()
        cw_msg.data = float(self.last_crosswalk_dist)
        self.pub_crosswalk.publish(cw_msg)

        # ============================================================
        # 3. 시각화
        # ============================================================
        if need_debug:
            bev_img = cv2.warpPerspective(
                frame, self.M_ipm, (self.bev_w, self.bev_h))

            # BEV 크롭 영역 표시 (인플레이스 — 전체 복사 불필요)
            if self.bev_crop_y_min > 0:
                roi = bev_img[0:self.bev_crop_y_min, :]
                roi[:] = (roi.astype(np.uint16) + 80) // 2
            if self.bev_crop_y_max < self.bev_h:
                roi = bev_img[self.bev_crop_y_max:, :]
                roi[:] = (roi.astype(np.uint16) + 80) // 2

            # BEV에 polyfit 곡선
            plot_y = np.linspace(0, self.bev_h - 1, self.bev_h)
            if self.last_left_fit is not None:
                lx = np.clip(np.polyval(self.last_left_fit, plot_y),
                             0, self.bev_w - 1).astype(int)
                cv2.polylines(bev_img,
                    [np.column_stack((lx, plot_y.astype(int)))],
                    False, (255, 0, 0), 2)
            if self.last_right_fit is not None:
                rx = np.clip(np.polyval(self.last_right_fit, plot_y),
                             0, self.bev_w - 1).astype(int)
                cv2.polylines(bev_img,
                    [np.column_stack((rx, plot_y.astype(int)))],
                    False, (0, 0, 255), 2)

            # Lane ROI 사각형
            if self.draw_roi:
                cv2.rectangle(frame, (lxmin, lymin), (lxmax, lymax), (255, 255, 0), 2)

            # 원본에 차선 윤곽
            if self.draw_lane_outline:
                for (pts, cls_id, conf_det) in self.last_lane_segments:
                    cls_id = int(cls_id)
                    if cls_id == 1:
                        color = (255, 0, 0)
                    elif cls_id == 0:
                        color = (0, 255, 0)
                    elif cls_id == 2:
                        color = (0, 0, 255)
                    else: color = (255, 255, 0)
                    cv2.polylines(frame, np.int32([pts]), True, color, 2)


            # ── centerline 시각화 ──
            conf = self.last_viz["confidence"]
            if conf > 0.0:
                xb = self.last_viz["xb"]
                yb = self.last_viz["yb"]
                xl = self.last_viz["xl"]
                yl = self.last_viz["yl"]

                if conf >= 0.5:
                    c_bot, c_look, c_line = (0,255,255), (0,255,0), (0,140,255)
                elif conf > 0.0:
                    c_bot, c_look, c_line = (0,180,255), (0,180,200), (0,120,200)
                else:
                    c_bot = c_look = c_line = (180,180,180)

                # BEV에 표시
                cv2.circle(bev_img, (int(xb), int(yb)), 8, c_bot, -1)
                cv2.circle(bev_img, (int(xl), int(yl)), 8, c_look, -1)
                cv2.line(bev_img, (int(xb), int(yb)),
                         (int(xl), int(yl)), c_line, 3)

                mode_text = "BOTH" if conf >= 0.5 else "ONLY" if conf > 0.0 else "LOST"
                cv2.putText(bev_img, mode_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0,255,0) if conf >= 0.5 else
                    (0,180,255) if conf > 0.0 else (100,100,255), 2)
                cv2.putText(bev_img, f"conf:{conf:.2f}", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200,200,200), 2)

                # 원본에 표시
                orig_b = self._bev_pt_to_orig((xb, yb))
                orig_l = self._bev_pt_to_orig((xl, yl))
                cv2.circle(frame, orig_b,  10, c_bot,  -1)
                cv2.circle(frame, orig_l,  10, c_look, -1)
                cv2.line(frame, orig_b, orig_l, c_line, 4)

            # ── 핵심 수치 오버레이 (BEV 좌하단) ──
            info_y = 78
            # judgment 상태
            cv2.putText(bev_img, f"J:{self._judgment_state}", (10, info_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 100), 1)
            info_y += 18
            # 반폭 (ONLY 모드 디버깅 핵심)
            cv2.putText(bev_img, f"hw:{self.measured_half_width:.0f}px",
                (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            info_y += 18
            # 정지선 거리
            if self.last_stop_line_dist >= 0:
                cv2.putText(bev_img,
                    f"stop:{self.last_stop_line_dist:.2f}m",
                    (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 150, 255), 1)
                info_y += 18
            # steer / speed (judgment에서 수신)
            if "steer" in self.last_viz:
                cv2.putText(bev_img,
                    f"steer:{self.last_viz['steer']:.3f} spd:{self.last_viz['speed']:.1f}",
                    (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 100), 1)

            # IPM src 영역
            cv2.polylines(frame, [np.int32(self.src_pts)], True, (0, 0, 255), 2)

            # FPS (추론이 실제 실행된 프레임에서만 측정)
            if run_lane:
                now_t = time.monotonic()
                dt = max(1e-6, now_t - self._last_fps_t)
                self._fps = (0.9 * self._fps + 0.1 / dt if self._fps > 0 else 1.0 / dt)
                self._last_fps_t = now_t
            if self.draw_fps:
                cv2.putText(frame, f"FPS: {self._fps:.1f}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            # 디버그 퍼블리시
            frame_resized = cv2.resize(frame, (self.bev_w, self.bev_h))
            combined = np.hstack([frame_resized, bev_img])
            self.pub_debug.publish(
                self.bridge.cv2_to_imgmsg(combined, encoding="bgr8"))

            # 창 표시와 녹화는 독립이다 — show_window=False 로 헤드리스 주행해도
            # 녹화는 계속된다(예전엔 write 가 show_window 안에 있어 같이 꺼졌다).
            # [성능] Lane 창은 원본(1920x1080)이 아니라 frame_resized(bev_w x bev_h,
            #   위에서 디버그 퍼블리시용으로 이미 계산됨)로 표시 — CUDA+imshow 동기화
            #   비용이 해상도에 비례해 커서, 축소만으로 창 응답성이 크게 좋아진다.
            #   녹화(_rec.write)는 원본 그대로 — 사후 분석 화질엔 영향 없음.
            if self.show_window:
                cv2.imshow("Lane", frame_resized)
            if self._rec: self._rec.write("Lane", frame)
            # BEV 는 캘리브 오버레이를 얹기 전에 기록한다(write 가 내부에서 복사본을
            # 뜨므로 이후 그리기는 녹화본에 영향 없음). 클릭 안내는 대화형 창 전용.
            if self._rec: self._rec.write("BEV", bev_img)
            if self.show_window:
                if self.bev_calibration_mode:
                    cv2.putText(bev_img,
                        f"CALIB: click 2 pts (real={self.bev_calib_real_distance_m:.2f}m)",
                        (10, self.bev_h - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                    for pt in self._calib_clicks:
                        cv2.circle(bev_img, pt, 6, (0, 0, 255), -1)
                cv2.imshow("BEV", bev_img)
            if self.show_bev_lane_only and self.last_sw_debug is not None:
                if self.show_window:
                    cv2.imshow("Sliding Window", self.last_sw_debug)
                if self._rec: self._rec.write("SlidingWindow", self.last_sw_debug)

        # ============================================================
        # 4. Traffic Light 처리
        # ============================================================
        self._process_tl(raw_frame)

        if self.show_window:
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = LaneLine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if getattr(node, "_rec", None):
            node._rec.close()
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()