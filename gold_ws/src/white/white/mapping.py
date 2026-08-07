#!/usr/bin/env python3
"""
mapping.py ― 경로 수집(매핑) 노드 [자동 리모델링 포함]

기능
1. /ego_state를 5Hz로 CSV 저장
2. direction 컬럼 유지 (+1 전진 / -1 후진) — ★kasa 는 후진이 없어 항상 +1★
3. 매핑 종료 시 원본 route_YYYYMMDD_HHMMSS.csv 저장
4. 곧바로 route_YYYYMMDD_HHMMSS_remodeled.csv 자동 생성
5. [카메라 융합] /lane_metrics 를 lane_cte/lane_conf/lane_flags 컬럼으로 동시 기록

═══════════════════════════════════════════════════════════════════════════════
 ★★ [2026-08-04] 수집 방식 변경 : 무선 컨트롤러 → 수동조종 모드 실계측 ★★
═══════════════════════════════════════════════════════════════════════════════
 이전(white 차량) : 무선 컨트롤러로 차를 몰고, 그 조종값이 실려 있는 /cmd_vel_raw 의
   linear.x **부호만** 방향 판정에 썼다. 조향 컬럼(steer)은 /ego_state[6] 에서 읽었는데
   ★그 자리는 gps_imu 가 항상 0 을 넣는 미사용 필드였다★ — 즉 steer 는 늘 0.00 이었다.

 현재(kasa 차량) : ★무선 컨트롤러를 쓰지 않는다★ 사람이 차에 타서 **수동조종 모드**
   (B보드 D5 스위치 개방)로 페달을 밟고 핸들을 직접 돌린다. 그때의 실계측 3종을 기록한다:

     ① throttle_pulse  /drive_pulse_cmd       페달 raw → 환산된 주행 목표펄스 (0~15)
                                              환산 규칙(throttle_raw_min/max·manual_pulse_max)이
                                              nxde/arduino.py 에만 있으므로 그쪽이 발행한다
     ② wheel_pulse     /encoder               실제로 돈 주행 펄스 (좌+우 합, 20ms 창)
        wheel_speed                           위 값을 m/s 로 환산 (kasa_units, 1카운트=0.442)
     ③ steer_measured  /steer_angle_measured  DC 조향모터 가변저항 실측 각도 [deg]
                                              (★− 좌 / + 우★ — B보드 값 그대로)

   부수 기록 : throttle_raw(A0 원값) / auto_mode(D5) / estop
     · auto_mode 는 ★수집 중 모드가 바뀌지 않았는지 사후 검증하는 근거★ 다.
       자율주행 모드로 넘어간 구간은 사람의 조작이 아니므로 학습·분석에서 걸러야 한다.
     · estop=1 인 행은 차가 강제 정지된 구간이라 마찬가지로 제외 대상이다.

   ★ 모드 강제는 prompt 가 한다 ★ 자율주행 모드에서 수집을 시작할 수 없다(메뉴에서 차단).
     이 노드는 그래도 방어적으로 시작 시점의 모드를 검사해 경고를 남긴다.

   ★ steer 컬럼은 유지한다 ★ route_remodeler 와 구버전 분석툴이 열 위치로 읽으므로
     지우지 않고, 값만 ③(실측 조향각)으로 채운다. 늘 0 이던 자리가 실값이 된 셈이다.

주의
- route_remodeler.py를 같은 white 패키지 폴더에 같이 넣어야 자동 리모델링 import가 됩니다.
- 자동 리모델링 실패 시에도 원본 CSV는 그대로 남습니다.

[카메라 융합 Phase0] lane_* 컬럼의 의미
  목표: "GPS가 끊겨도 매핑 때와 같은 선을 유지한다".
  원리: 주행 중 카메라가 읽는 cte 와, 매핑 때 같은 지점에서 읽었던 cte(=lane_cte)의 차이가
        곧 '매핑 선 대비 횡오차'다. 매핑과 주행이 같은 카메라를 쓰므로 카메라 계통편향
        (cam_yaw_offset·px2m 캘리브 오차)이 뺄셈에서 상쇄된다 → 차선 중심의 절대좌표를
        몰라도 되고, 캘리브를 먼저 잡을 필요도 없다.
  주의: 편향 상쇄는 매핑과 주행의 '검출 모드가 같을 때'만 성립한다. 한쪽이 단독차선
        (F_SINGLE=반폭 추정)이면 편향이 달라 비교가 깨지므로 lane_flags 를 같이 남긴다.
        conf_eff>=0.5 는 perception 규약상 '양쪽 차선 검출'을 함의한다(_soft_confidence).
"""

import os
import csv
import time
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float32MultiArray, Bool, Int32

# 패키지 설치 후 실행 / 로컬 단독 실행 둘 다 대응
try:
    from white.route_remodeler import remodel_route
except Exception:
    try:
        from route_remodeler import remodel_route
    except Exception:
        remodel_route = None

# [kasa] /encoder 카운트 → m/s 환산 (상수의 단일 소유자)
from white import kasa_units as ku


class MappingNode(Node):
    # [카메라 융합] /lane_metrics 신선도 한계 [s]. gps_imu.CAM_HEAD_MAX_AGE 와 동일 기준.
    #   이보다 오래된 계측은 conf=0 으로 기록해 하류(driving)에서 자동 배제되게 한다.
    LANE_MAX_AGE = 0.40

    def __init__(self):
        super().__init__("mapping_node")
        # ★ driving.py / prompt.py / route_remodeler.py / tool의 분석툴과 반드시
        #   같은 폴더를 가리킬 것. 예전 워크스페이스 경로(~/white_ws0709b)가 남아
        #   있었는데, 아래 makedirs가 이미 없어진 그 폴더를 조용히 다시 만들어서
        #   매핑한 route가 유령 폴더로 들어가고 분석툴에선 안 보였다(2026-07-15).
        self.data_dir = os.path.expanduser("~/white_ws/white_ws/gps_data")
        os.makedirs(self.data_dir, exist_ok=True)

        self.is_mapping = False
        self.csv_file = None
        self.csv_writer = None
        self.current_csv_path = None
        self._flush_counter = 0          # 🔧 BUG8: 주기적 flush 카운터
        self._remodel_thread = None      # 🔧 BUG9: 비동기 리모델링 스레드

        # 자동 리모델링 기본값
        self.auto_remodel_enabled = True
        self.remodel_spacing = 0.25      # waypoint 간격 [m]
        self.remodel_epsilon = 0.08      # 직선 smoothing 최대 이동 [m]
        self.remodel_smooth_iter = 1     # 직선 smoothing 반복 횟수

        # ego_state
        self.lat = None
        self.lon = None
        self.heading = 0.0
        self.speed = 0.0
        # (steer 는 더 이상 ego_state[6] 에서 읽지 않는다 — steer_measured 로 대체됨)
        self.pitch = 0.0
        self.terrain = 0.0

        # 주행 방향. kasa 는 후진이 없어 실질적으로 항상 +1 이다(컬럼은 호환을 위해 유지).
        self.direction = 1

        # ── ★수동조종 실계측 3종 (수집 라벨)★ ──
        self.throttle_pulse = 0    # ① /drive_pulse_cmd  페달 환산 주행 목표펄스 0~15
        self.wheel_pulse    = 0    # ② /encoder          실제 주행 펄스 (좌+우 합)
        # ③ /steer_angle_measured  실측 조향각 [deg]
        #   ★부호는 − 좌 / + 우 (ROS·kasa 규약)★ — 2026-08-04 규약 개정 반영.
        #   [2026-08-05 정정] 여기 주석이 '+좌'로 남아 있었다(1/5카 시절 규약).
        #   값은 B보드 값을 그대로 통과시키므로 동작은 원래 맞았고 주석만 틀렸다.
        #   kasa_units.py 3절 참고 — 부호가 뒤집히는 지점은 driving.publish_cmd 한 곳뿐이다.
        self.steer_measured = 0.0
        self.throttle_raw   = 0    # 부수 : A0 페달 원값 0~1023
        self.auto_mode      = None # 부수 : B보드 D5 (True 자율 / False 수동 / None 미수신)
        self.estop          = False
        # 수집 중 자율주행 모드가 섞였는지 집계 (종료 로그 + 사후 필터 근거)
        self._auto_rows = 0
        self._estop_rows = 0

        # [카메라 융합] /lane_metrics 최신값 (camera_judgment 브리지 출력)
        self.lane_cte   = 0.0    # [0] cte_rear_m (우측+)
        self.lane_theta = 0.0    # [2] theta_lane_deg (지도접선−헤딩, CCW+)
        self.lane_conf  = 0.0    # [4] conf_eff (치명 게이트 실패 시 0)
        self.lane_curv  = 0.0    # [3] curvature_1pm (차선 곡률)
        self.lane_flags = 0.0    # [6] flags
        self.lane_time  = 0.0
        self._lane_seen = False  # 이번 매핑에서 차선을 한 번이라도 봤나(종료 로그용)
        self._lane_rows = 0      # 유효 차선(conf>=0.5) 기록 행 수
        self._total_rows = 0     # 전체 기록 행 수(차선 커버리지 산출용)

        self.create_subscription(Float64MultiArray, "/ego_state", self.cb_ego, 10)
        self.create_subscription(Bool, "/mapping_cmd", self.cb_mapping_cmd, 10)
        self.create_subscription(Float32MultiArray, "/lane_metrics", self.cb_lane_metrics, 10)

        # ── ★수동조종 실계측 3종 + 부수 상태★ (전부 nxde arduino 노드가 발행) ──
        #   ※ /cmd_vel_raw 구독은 제거했다 — 무선 컨트롤러를 쓰지 않으므로 '조종값'이
        #     그 토픽에 실리지 않고, 방향 판정도 후진이 없어 필요가 없어졌다.
        self.create_subscription(Int32, "/drive_pulse_cmd",      self.cb_drive_pulse,  10)
        self.create_subscription(Int32, "/encoder",              self.cb_encoder,      10)
        self.create_subscription(Int32, "/steer_angle_measured", self.cb_steer_meas,   10)
        self.create_subscription(Int32, "/throttle_pedal",       self.cb_throttle_raw, 10)
        self.create_subscription(Bool,  "/vehicle_mode",         self.cb_vehicle_mode, 10)
        self.create_subscription(Bool,  "/estop",                self.cb_estop,        10)

        self.create_timer(0.2, self.record_loop)  # 5Hz 기록
        self.get_logger().info(
            f"🗺️ Mapping node 시작 | auto_remodel={self.auto_remodel_enabled} | "
            f"spacing={self.remodel_spacing}m epsilon={self.remodel_epsilon}m smooth={self.remodel_smooth_iter} | "
            f"차선기록: /lane_metrics (max_age={self.LANE_MAX_AGE}s) | "
            f"수집라벨: /drive_pulse_cmd(페달환산) /encoder(실주행펄스) /steer_angle_measured(실측각)"
        )

    # ── ★수동조종 실계측 3종★ ────────────────────────────────────────────
    def cb_drive_pulse(self, msg: Int32):
        """① 페달 환산 주행 목표펄스. 수동조종에서는 사람이 밟은 페달의 환산값이고,
        자율주행에서는 driving 이 계획한 값이다(그래서 auto_mode 를 함께 기록한다)."""
        self.throttle_pulse = int(msg.data)

    def cb_encoder(self, msg: Int32):
        """② 실제로 돈 주행 펄스 (좌+우 합, 20ms 창, 부호 없음).
        후진이 없으므로 direction 은 항상 +1 로 둔다(컬럼은 호환을 위해 유지)."""
        self.wheel_pulse = int(msg.data)
        self.direction = 1

    def cb_steer_meas(self, msg: Int32):
        """③ DC 조향모터 가변저항 실측 각도 [deg]. ★부호는 − 좌 / + 우★ (ROS 규약).
        B보드 값 그대로 오므로 여기서 뒤집지 않는다."""
        self.steer_measured = float(msg.data)

    def cb_throttle_raw(self, msg: Int32):
        self.throttle_raw = int(msg.data)

    def cb_vehicle_mode(self, msg: Bool):
        self.auto_mode = bool(msg.data)

    def cb_estop(self, msg: Bool):
        self.estop = bool(msg.data)

    def cb_ego(self, msg: Float64MultiArray):
        d = msg.data
        if len(d) >= 7:
            self.lat = d[0]
            self.lon = d[1]
            self.heading = d[4]
            self.speed = d[5]
            # ★ d[6] 은 gps_imu 가 항상 0 을 넣는 미사용 필드다 ★ (ARCHITECTURE §핵심 토픽 계약
            #   의 /ego_state 규약 참고) 그래서 예전 steer 컬럼은 늘 0.00 이었다. 이제
            #   steer 컬럼은 /steer_angle_measured(실측 조향각)로 채운다 — record_loop 참고.
            #   d[6] 은 읽지 않는다.
            # kasa 는 후진이 없어 direction 은 항상 +1 (cb_encoder 에서 세팅)
        if len(d) >= 9:
            self.pitch = d[7]
            self.terrain = d[8]

    def cb_lane_metrics(self, msg: Float32MultiArray):
        """camera_judgment /lane_metrics[10] 수신. 기록은 record_loop(5Hz)에서."""
        d = msg.data
        if len(d) < 7:
            return
        self.lane_cte   = float(d[0])   # cte_rear_m (우측+)
        self.lane_theta = float(d[2])   # theta_lane_deg (지도접선−헤딩, CCW+)
        self.lane_curv  = float(d[3])   # curvature_1pm
        self.lane_conf  = float(d[4])   # conf_eff
        self.lane_flags = float(d[6])   # flags
        self.lane_time  = time.time()
        self._lane_seen = True

    def _lane_snapshot(self):
        """기록용 차선 5값. 신선하지 않으면 conf=0 으로 무효화해서 내보낸다.
        (하류 driving 은 conf 게이트만 보면 되도록 여기서 신선도를 흡수한다.)
        반환: (cte, conf, flags, theta, curv). 무효 시 theta/curv 도 0 —
        conf=0 행의 각도/곡률은 오검출이라 리모델 보간에 섞이면 안 된다."""
        if self.lane_time <= 0.0 or (time.time() - self.lane_time) > self.LANE_MAX_AGE:
            return 0.0, 0.0, 0.0, 0.0, 0.0
        return (self.lane_cte, self.lane_conf, self.lane_flags,
                self.lane_theta, self.lane_curv)

    def cb_mapping_cmd(self, msg: Bool):
        if msg.data and not self.is_mapping:
            self.start_mapping()
        elif not msg.data and self.is_mapping:
            self.stop_mapping()

    def start_mapping(self):
        fname = os.path.join(
            self.data_dir,
            f"route_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
        )
        self.current_csv_path = fname
        self.csv_file = open(fname, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            # ── 기존 8컬럼 (열 위치 불변 — route_remodeler·구버전 분석툴 호환) ──
            #   steer 는 이제 ③ 실측 조향각으로 채운다(예전엔 항상 0.00 이었다).
            "latitude", "longitude", "heading", "speed", "steer",
            "direction", "pitch", "terrain",
            # [카메라 융합] 기존 8컬럼 뒤에만 덧붙인다 → 구버전 리더는 무시, 신버전은 .get()으로 읽음
            # lane_theta/lane_curv 는 lane_flags 뒤에 추가(더 뒤에 붙일수록 구버전 파서 무영향)
            "lane_cte", "lane_conf", "lane_flags", "lane_theta", "lane_curv",
            # ── ★[2026-08-04] 수동조종 실계측 (맨 뒤에만 추가 → 구버전 파서 무영향)★ ──
            "throttle_pulse",   # ① 페달 환산 목표펄스 0~15 (/drive_pulse_cmd)
            "wheel_pulse",      # ② 실 주행 펄스 좌+우 합 (/encoder)
            "wheel_speed",      # ② 를 m/s 로 환산 (1카운트 = 0.442 m/s)
            # ③ 실측 조향각 [deg, ★− 좌 / + 우★] (/steer_angle_measured) = steer 와 동일값
            #    ⚠️ 1/5카가 기록한 구맵의 steer 컬럼은 반대 부호(+좌)다 — 구맵을
            #       tool/map_check.py 로 분석할 때 좌우가 뒤집혀 보인다(주행에는 영향 없음:
            #       driving.py 는 CSV 의 steer 컬럼을 읽지 않는다).
            "steer_measured",
            "throttle_raw",     # 부수 : A0 페달 원값 0~1023
            "auto_mode",        # 부수 : 1 자율 / 0 수동조종 — ★수집 유효구간 판별용★
            "estop",            # 부수 : 1 이면 강제정지 구간 → 분석에서 제외
        ])
        self.is_mapping = True
        self._lane_rows = 0
        self._total_rows = 0
        self._auto_rows = 0
        self._estop_rows = 0
        self.get_logger().info(f"🗺️ 매핑 시작: {fname}")

        # ★ 수집은 수동조종 모드에서 해야 한다 ★ 모드 강제는 prompt 가 하지만, 여기서도
        #   방어적으로 검사해 근거를 로그에 남긴다(prompt 없이 /mapping_cmd 를 직접 쏜 경우).
        if self.auto_mode is None:
            self.get_logger().warn(
                "⚠️ /vehicle_mode 미수신 — nxde arduino 노드나 B보드가 아직 연결되지 않은 듯합니다. "
                "수동조종 계측 3종(throttle_pulse/wheel_pulse/steer_measured)이 0으로 기록됩니다. "
                "arduino 노드 기동과 /board_status 를 확인하세요 "
                "(`ros2 run nxde check` 로 연결부터 점검)."
            )
        elif self.auto_mode:
            self.get_logger().warn(
                "⚠️ 지금은 ★자율주행 모드★ 입니다 — 수집은 수동조종 모드에서 해야 합니다. "
                "차량의 주행모드 스위치(B보드 D5)를 수동조종으로 전환하세요. "
                "(이대로 기록하면 auto_mode=1 행이 남고, 그 구간은 사람 조작이 아니라 "
                "학습·분석에서 걸러야 합니다)"
            )

        if not self._lane_seen:
            self.get_logger().warn(
                "⚠️ /lane_metrics 미수신 상태로 매핑 시작 — lane_* 컬럼이 전부 0으로 기록됩니다. "
                "카메라 융합(use_camera:=true)을 쓸 계획이면 perception/camera_judgment 기동을 확인하세요."
            )

    def stop_mapping(self):
        self.is_mapping = False
        saved_path = self.current_csv_path

        if self.csv_file:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None

        self.get_logger().info(f"✅ 매핑 종료 및 원본 저장 완료: {saved_path}")

        # [카메라 융합] 차선 커버리지 = 이 맵으로 GPS단절 횡보정을 쓸 수 있는 구간 비율.
        #   낮으면 대부분 구간에서 보정이 걸리지 않는다(헤딩 보정만 남음).
        if self._total_rows > 0:
            pct = 100.0 * self._lane_rows / self._total_rows
            msg = (f"📷 차선 커버리지: {self._lane_rows}/{self._total_rows} 행 "
                   f"({pct:.1f}%) — 양쪽차선 conf≥0.5 기준")
            if pct < 20.0:
                self.get_logger().warn(msg + " | 커버리지가 낮아 GPS단절 횡보정 효과가 제한적입니다.")
            else:
                self.get_logger().info(msg)

            # ★ 수집 유효성 요약 ★ 수동조종으로만 몰았다면 auto/estop 이 0 이어야 정상이다.
            if self._auto_rows or self._estop_rows:
                self.get_logger().warn(
                    f"⚠️ 수집 유효성: 자율주행 모드 {self._auto_rows}행 / "
                    f"E-stop {self._estop_rows}행 (전체 {self._total_rows}행)이 섞였습니다. "
                    f"그 구간은 사람 조작이 아니므로 auto_mode·estop 컬럼으로 걸러 쓰세요."
                )
            else:
                self.get_logger().info(
                    f"✅ 수집 유효성: 전 구간 수동조종·정상 ({self._total_rows}행)"
                )

        if self.auto_remodel_enabled and saved_path:
            self.run_auto_remodel(saved_path)

    def run_auto_remodel(self, input_csv: str):
        if remodel_route is None:
            self.get_logger().error(
                "❌ route_remodeler.py를 import하지 못했습니다. "
                "white/route_remodeler.py 파일이 같은 패키지에 있는지 확인하세요."
            )
            return

        # 🔧 BUG9: 리모델링을 백그라운드 스레드에서 실행 → ROS2 콜백 블로킹 방지
        def _worker():
            try:
                self.get_logger().info("🔧 매핑 경로 자동 리모델링 시작 (백그라운드)...")
                out_csv = remodel_route(
                    input_csv=input_csv,
                    output_csv=None,
                    spacing=self.remodel_spacing,
                    epsilon=self.remodel_epsilon,
                    smooth_iter=self.remodel_smooth_iter,
                )
                self.get_logger().info(f"✅ 리모델링 경로 저장 완료: {out_csv}")
                self.get_logger().info("🚗 자율주행에서는 *_remodeled.csv 파일을 선택하면 됩니다.")
            except Exception as e:
                self.get_logger().error(f"❌ 자동 리모델링 실패: {e}")
                self.get_logger().warn("원본 매핑 CSV는 정상 저장되어 있으므로 원본으로 주행하거나 수동 리모델링하세요.")

        self._remodel_thread = threading.Thread(target=_worker, daemon=True)
        self._remodel_thread.start()

    def record_loop(self):
        if self.is_mapping and self.lat is not None and self.csv_writer is not None:
            lane_cte, lane_conf, lane_flags, lane_theta, lane_curv = self._lane_snapshot()
            self._total_rows += 1
            if lane_conf >= 0.5:
                self._lane_rows += 1
            # ★ 수집 유효구간 판별 근거 ★ 자율 모드 행과 e-stop 행은 사람 조작이 아니다.
            is_auto = bool(self.auto_mode)   # None(미수신)은 0 으로 기록한다
            if is_auto:
                self._auto_rows += 1
            if self.estop:
                self._estop_rows += 1
            self.csv_writer.writerow([
                f"{self.lat:.8f}",
                f"{self.lon:.8f}",
                f"{self.heading:.2f}",
                f"{self.speed:.4f}",
                # ★ steer 컬럼 = ③ 실측 조향각 ★ (예전엔 ego_state[6]=항상 0 이었다)
                f"{self.steer_measured:.2f}",
                str(self.direction),
                f"{self.pitch:.2f}",
                f"{self.terrain:.1f}",
                f"{lane_cte:.4f}",
                f"{lane_conf:.3f}",
                f"{int(lane_flags)}",
                f"{lane_theta:.2f}",
                f"{lane_curv:.6f}",
                # ── 수동조종 실계측 ──
                str(self.throttle_pulse),
                str(self.wheel_pulse),
                f"{ku.encoder_count_to_ms(self.wheel_pulse):.4f}",
                f"{self.steer_measured:.2f}",
                str(self.throttle_raw),
                "1" if is_auto else "0",
                "1" if self.estop else "0",
            ])
            # 🔧 BUG8: 매 10행(2초)마다 flush → 크래시 시 데이터 유실 방지
            self._flush_counter += 1
            if self._flush_counter >= 10:
                self.csv_file.flush()
                self._flush_counter = 0


def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.is_mapping:
            node.stop_mapping()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()