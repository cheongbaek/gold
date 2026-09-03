#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
camera_judgment.py ― 카메라 판단 노드 [white_ws 융합 v1.0]
─────────────────────────────────────────────────────────────────
역할(2가지, test2_judgment 개조 + lane_camera_bridge 흡수):

 1) 차선 계측 브리지: perception 의 /lane/state(BEV 픽셀공간 다항식)를
    미터·뒷차축 기준 /lane_metrics[10] 로 변환 → gps_imu 가 헤딩보정에 사용.
    (부호규약은 white_ws_backup/lane_camera_bridge.py 에서 실주행 bag 3건으로
     검증된 값을 그대로 이식: 최종 발행 cte 만 부호反전 → GPS CTE 규약=우측+)

 2) 신호등 게이트: driving 이 내는 /cmd_vel_drive(경로추종 조향+속도)를 받아,
    적색+정지선/횡단보도 조건이면 speed 를 정지/서행으로 덮어 /cmd_vel_raw 발행.
    · 조향(angular.z)은 항상 driving 값을 통과 → 최종 조향은 GPS 경로추종이 담당.
    · fail-open: 신선하고 확실한 정지조건이 없으면 driving 명령을 그대로 통과.
    · ★★ [2026-08-05] 완전정지(TL_STOP)에서는 ★리니어모터 2단★ 을 함께 물린다 ★★
      /brake_level = 2 (풀브레이킹) → nxde/arduino.py → B보드가 엔코더 실측 시간표로 밟는다.
      속도명령만 0 으로 덮는 것으로는 kasa 가 경사에서 밀린다(인휠 코스트/회생뿐).
      감속 중(TL_BRAKE)·서행(TL_SLOW)에는 걸지 않는다 — 굴러가는 중에 리니어를 밟으면
      인휠 PID 와 싸운다. 정지조건이 풀리면 0단(놓음)으로 되돌린다. _apply_brake() 참고.
    · ⚠️ [kasa 이식] 원래 여기에 "driving 이 /cmd_vel_drive 를 끊으면 이 노드도 발행 중단
      → 아두이노 워치독 정지(fail-safe)" 라고 적혀 있었으나 **kasa 에서는 성립하지 않는다.**
      A보드 펌웨어에 무입력 타임아웃이 없고(0713에서 제거) nxde/arduino.py 가 마지막
      명령을 1초 주기로 계속 재전송하므로, 발행을 멈추면 ★마지막 주행값이 유지된다★.
      그래서 driving 쪽이 '계산 불가' 상태에서도 0 을 명시적으로 발행하도록 바꿨다
      (driving.control_loop 의 bail-out 분기 참고). 정지 수단은 위 리니어 2단 외에
      /control_state=False · E-stop 스위치 · B보드 D5 수동조종 전환이 있다.

이 노드는 자체 조향을 계산하지 않는다(과거 test2_judgment 의 Pure Pursuit 는 제거).

발행:
  /lane_metrics  Float32MultiArray[10]
     [0]cte_rear_m [1]cte_near_m [2]theta_lane_deg [3]curvature_1pm
     [4]conf_eff(게이트 반영,치명 실패 시 0) [5]lane_width_m
     [6]flags [7]d_near_m [8]conf_raw [9]seq
  /cmd_vel_raw   geometry_msgs/Twist   (게이트 통과된 최종 제어 → nxde/arduino)
  /judgment_state String                (게이트 FSM 상태, 디버그/시각화)
  /brake_level   std_msgs/Int32         ★신호등 완전정지 시 2단(리니어), 해제 시 0단★

구독:
  /lane/state      Float32MultiArray[8]  ← perception  [aL,bL,cL,aR,bR,cR,conf,hw]
  /cmd_vel_drive   Twist                 ← driving      (경로추종 조향+속도)
  /tl/state        String                ← perception   RED/RED_FAR/GREEN/UNKNOWN
  /stop_line_dist  Float32               ← perception   [m], 미검출 -1
  /crosswalk_dist  Float32               ← perception   [m], 미검출 -1

flags 비트: 1=차선없음 2=폭이상 4=CTE점프 8=conf미달 16=단독차선(반폭 추정)
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String, Float32, Int32

# [white2] 조향 클램프(±STEER_MAX_DEG)에 쓴다 — 1/5카는 /cmd_vel_raw 도 m/s 그대로라
#   kasa 처럼 발행 직전 펄스 환산이 필요 없다.
from white2 import car_units as ku

F_NO_LANE   = 1
F_WIDTH_BAD = 2
F_JUMP      = 4
F_CONF_LOW  = 8
F_SINGLE    = 16

TL_CODE = {"UNKNOWN": -1.0, "GREEN": 0.0, "RED_FAR": 1.0, "RED": 2.0}


class CameraJudgment(Node):
    def __init__(self):
        super().__init__("camera_judgment")

        # ── 공유 기하 파라미터 (perception 과 반드시 일치) ────────────────
        self.declare_parameter("pixel_to_meter_bev",      0.006)   # [m/px] BEV 캘리브
        self.declare_parameter("bev_w",                   640)
        self.declare_parameter("bev_h",                   480)
        self.declare_parameter("bottom_ratio",            0.92)    # 근점(측정) 행
        self.declare_parameter("lookahead_ratio",         0.45)    # θ 산출용 전방 행
        self.declare_parameter("lane_width_m",            3.0)
        self.declare_parameter("bev_bottom_ahead_rear_m", 0.55)    # 뒷차축→BEV 최하단 [m]
        self.declare_parameter("cam_yaw_offset_deg",      0.0)     # 카메라 요 캘리브
        # ── 브리지 자가검증 게이트 ──────────────────────────────────────
        self.declare_parameter("conf_min",                0.15)
        self.declare_parameter("width_min_m",             1.8)
        self.declare_parameter("width_max_m",             4.5)
        self.declare_parameter("jump_max_m",              0.35)
        self.declare_parameter("stale_warn_s",            2.0)
        # ── 신호등 게이트 ──────────────────────────────────────────────
        self.declare_parameter("tl_enable",               True)
        self.declare_parameter("tl_decel",                1.0)     # 정지선 접근 감속도 [m/s²]
        self.declare_parameter("tl_stop_margin",          0.5)     # 이 앞에서 완전정지 [m]
        self.declare_parameter("tl_far_cap",              1.2)     # RED_FAR 순항 상한 [m/s]
        self.declare_parameter("tl_nodist_cap",           0.8)     # RED+정지선미검출 서행 상한
        self.declare_parameter("tl_hold_s",               0.4)     # RED 확정 유지시간(깜빡임 필터)
        # [2026-07-31 → 2026-08-05 복원] cb_tl 이 RED 아닌 프레임 단 1개에도 red_since 를
        #   완전 리셋해서 실주행에선 tl_hold_s(0.4s) 를 사실상 못 채웠다. 실측 rosbag
        #   (tl_state.csv) 패턴: RED,RED,UNKNOWN,UNKNOWN,RED,RED,RED,RED,UNKNOWN
        #   (프레임간 ~0.04~0.15s) — 빨간불을 6프레임이나 정확히 봤는데도 중간 UNKNOWN
        #   2회 때문에 스트릭이 두 번 끊겨 0.4s 연속을 못 채웠다. tl_gap_grace_s 이내의
        #   끊김은 마지막 RED 목격 시각 기준으로 봐주고 스트릭(red_since)을 유지한다
        #   → 진짜로 신호가 바뀌거나 오래 끊긴 경우만 리셋.
        #   ★kasa 이식 과정에서 이 로직이 통째로 빠져 있었다(2026-08-05 되살림)★
        self.declare_parameter("tl_gap_grace_s",          0.3)     # RED 스트릭 유지 허용 끊김[s]
        self.declare_parameter("tl_stale_s",              0.6)     # 정지선 신선도
        # [2026-07-29] /tl/state 신선도 한계. 구 하드코딩 1.0s 는 perception 처리율
        #   (정상 6.5Hz + ~3.5s 멈춤)에 비해 빡빡해 RED 를 놓치고 통과했다.
        self.declare_parameter("tl_state_max_age",        3.0)     # [s] 이보다 오래된 tl_state 는 무시
        # [2026-07-29] RED 확정인데 정지선을 못 잡은 경우의 동작.
        #   True  = 정지(요구사항: 빨간불이면 선다). False = 구 동작(tl_nodist_cap 서행).
        #   실측에서 stop_line_dist 가 100% -1(미검출)이라, False 면 영원히 정지하지 않는다.
        self.declare_parameter("tl_nodist_stop",          True)
        # ★★ [2026-08-05] 신호등 정지는 ★리니어모터 2단 체결★ 로 한다 ★★
        #   그전에는 /cmd_vel_raw 의 속도만 0 으로 덮었다. kasa 는 인휠모터 회생/코스트만으로는
        #   경사에서 밀리고, 정지 자체를 '기계적으로' 잡아줄 수단이 리니어 브레이크다.
        #   → RED 확정으로 완전정지 판정이 나면 /brake_level 로 2단(풀브레이킹)을 요청한다.
        #     정지 조건이 풀리면 0단(놓음)으로 되돌린다.
        #   ※ B보드 펌웨어(kasa_0804_B.ino)가 단계 0/1/2 를 엔코더 실측 시간표로 정확히 밟는다:
        #       0 = 기본위치(놓음) / 1 = 행정의 1/3 / 2 = 풀브레이킹
        #   ⚠️ E-stop 은 이것과 별개다 — B보드가 스스로 2단을 물고 해제 시 0단으로 돌아온다.
        #   ⚠️ 이 브레이크는 '수동조종 전환'과 아무 관계가 없다. 모드 전환 시 2단을 물던
        #      옛 래치(arduino.py manual_brake_level)는 2026-08-04 에 제거됐고 되살리지 않는다.
        self.declare_parameter("tl_brake_level",          2)       # 신호등 정지 시 걸 단계(0~2)
        self.declare_parameter("tl_brake_release_level",  0)       # 정지 해제 시 되돌릴 단계
        # ── 횡단보도 게이트(정지 아님, 서행) ────────────────────────────
        self.declare_parameter("cw_enable",               True)
        self.declare_parameter("cw_enter_dist",           0.6)     # 이 이내면 서행 진입 [m]
        self.declare_parameter("cw_slow_cap",             1.0)     # 횡단보도 서행 상한 [m/s]
        self.declare_parameter("cw_hold_s",               1.1)     # 통과 후 서행 유지시간

        g = lambda k: self.get_parameter(k).value
        self.px2m       = float(g("pixel_to_meter_bev"))
        self.bev_w      = int(g("bev_w"))
        self.bev_h      = int(g("bev_h"))
        self.y_near     = float(self.bev_h) * float(g("bottom_ratio"))
        self.y_look     = float(self.bev_h) * float(g("lookahead_ratio"))
        self.lane_w_m   = float(g("lane_width_m"))
        self.d_bev0     = float(g("bev_bottom_ahead_rear_m"))
        self.yaw_off    = float(g("cam_yaw_offset_deg"))
        self.conf_min   = float(g("conf_min"))
        self.w_min      = float(g("width_min_m"))
        self.w_max      = float(g("width_max_m"))
        self.jump_max   = float(g("jump_max_m"))
        self.stale_warn = float(g("stale_warn_s"))
        self.tl_enable  = bool(g("tl_enable"))
        self.tl_decel   = float(g("tl_decel"))
        self.tl_stop_margin = float(g("tl_stop_margin"))
        self.tl_far_cap = float(g("tl_far_cap"))
        self.tl_nodist_cap = float(g("tl_nodist_cap"))
        self.tl_hold_s  = float(g("tl_hold_s"))
        self.tl_gap_grace_s = float(g("tl_gap_grace_s"))
        self.tl_stale_s = float(g("tl_stale_s"))
        self.tl_state_max_age = float(g("tl_state_max_age"))
        self.tl_nodist_stop   = bool(g("tl_nodist_stop"))
        self.tl_brake_level         = max(0, min(2, int(g("tl_brake_level"))))
        self.tl_brake_release_level = max(0, min(2, int(g("tl_brake_release_level"))))
        self.cw_enable  = bool(g("cw_enable"))
        self.cw_enter_dist = float(g("cw_enter_dist"))
        self.cw_slow_cap = float(g("cw_slow_cap"))
        self.cw_hold_s  = float(g("cw_hold_s"))

        # 근점의 뒷차축 전방 거리 [m] / 단독차선 반폭 기본치
        self.d_near = self.d_bev0 + (self.bev_h - self.y_near) * self.px2m
        self.half_w_px_default = (self.lane_w_m * 0.5) / self.px2m

        # ── 상태 ──────────────────────────────────────────────────────
        self.prev_cte_near = None
        self.prev_t        = 0.0
        self.last_lane_rx  = 0.0
        self.seq           = 0

        self.tl_state   = "UNKNOWN"
        self.tl_time    = 0.0
        self.red_since  = None
        self.red_last_seen = None   # 마지막으로 RED/RED_FAR 를 실제로 본 시각(끊김 유예용)
        # 지금 걸어 둔 브레이크 단계. 변할 때만 /brake_level 을 발행한다
        #   (매 프레임 재발행하면 B보드가 같은 단계를 다시 밟으려 하지는 않지만 — 펌웨어가
        #    brake_level == brake_cmd_level 이면 무시한다 — 로그와 토픽이 시끄러워진다)
        self.brake_now  = None
        self.stop_dist  = -1.0
        self.stop_time  = 0.0
        self.cw_dist    = -1.0
        self.cw_time    = 0.0

        self._last_state = ""

        # ── ROS 인터페이스 ─────────────────────────────────────────────
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self.pub_metrics = self.create_publisher(Float32MultiArray, "/lane_metrics", 10)
        self.pub_cmd     = self.create_publisher(Twist,             "/cmd_vel_raw",  qos)
        self.pub_state   = self.create_publisher(String,            "/judgment_state", qos)
        # ★신호등 정지 = 리니어모터 2단 체결★ (nxde/arduino.py → B보드)
        self.pub_brake   = self.create_publisher(Int32,             "/brake_level",  qos)

        self.create_subscription(Float32MultiArray, "/lane/state",     self.cb_lane,  10)
        self.create_subscription(Twist,             "/cmd_vel_drive",  self.cb_cmd,   qos)
        self.create_subscription(String,            "/tl/state",       self.cb_tl,    qos)
        self.create_subscription(Float32,           "/stop_line_dist", self.cb_stop,  qos)
        self.create_subscription(Float32,           "/crosswalk_dist", self.cb_cross, qos)

        self.create_timer(1.0, self._status_tick)

        self.get_logger().info(
            f"🎥 camera_judgment | px2m={self.px2m} d_near={self.d_near:.2f}m "
            f"yaw_off={self.yaw_off:+.1f}° | TL={'on' if self.tl_enable else 'off'} "
            f"decel={self.tl_decel} margin={self.tl_stop_margin}m hold={self.tl_hold_s}s "
            f"gap_grace={self.tl_gap_grace_s}s brake={self.tl_brake_level}단 | "
            f"CW={'on' if self.cw_enable else 'off'} cap={self.cw_slow_cap}m/s | "
            f"게이트: 폭{self.w_min}~{self.w_max}m 점프≤{self.jump_max}m conf≥{self.conf_min}")

    # ══════════════════════════════════════════════════════════════════
    # 1) 차선 계측 브리지  (/lane/state → /lane_metrics)
    # ══════════════════════════════════════════════════════════════════
    @staticmethod
    def _poly_x(fit, y):
        a, b, c = fit
        return a * y * y + b * y + c

    @staticmethod
    def _poly_dx(fit, y):
        a, b, _ = fit
        return 2.0 * a * y + b

    def _offset_normal_x(self, fit, y, signed_off):
        """차선점에서 법선방향으로 signed_off 이동한 x (커브 단독차선 과대 방지)."""
        m = self._poly_dx(fit, y)
        return self._poly_x(fit, y) + signed_off / math.sqrt(1.0 + m * m)

    def cb_lane(self, msg: Float32MultiArray):
        d = list(msg.data)
        if len(d) < 8:
            return
        now = time.time()
        self.last_lane_rx = now
        self.seq += 1

        aL, bL, cL, aR, bR, cR = d[0], d[1], d[2], d[3], d[4], d[5]
        conf_raw = float(d[6])
        half_w_px_meas = float(d[7])

        left_ok  = abs(aL) + abs(bL) + abs(cL) > 1e-9
        right_ok = abs(aR) + abs(bR) + abs(cR) > 1e-9
        flags = 0
        width_m = -1.0

        if not left_ok and not right_ok:
            flags |= F_NO_LANE
            self._publish_metrics(0.0, 0.0, 0.0, 0.0, 0.0, width_m, flags, conf_raw)
            return

        hw_px = half_w_px_meas if half_w_px_meas > 1.0 else self.half_w_px_default

        if left_ok and right_ok:
            xL_n = self._poly_x((aL, bL, cL), self.y_near)
            xR_n = self._poly_x((aR, bR, cR), self.y_near)
            xL_l = self._poly_x((aL, bL, cL), self.y_look)
            xR_l = self._poly_x((aR, bR, cR), self.y_look)
            xc_n = 0.5 * (xL_n + xR_n)
            xc_l = 0.5 * (xL_l + xR_l)
            width_m = (xR_n - xL_n) * self.px2m
            curv = (abs(aL) + abs(aR)) * 0.5 * 2.0 / self.px2m   # κ≈2a/px2m [1/m]
            if not (self.w_min <= width_m <= self.w_max):
                flags |= F_WIDTH_BAD
        else:
            flags |= F_SINGLE
            fit = (aL, bL, cL) if left_ok else (aR, bR, cR)
            sgn = +1.0 if left_ok else -1.0     # 좌차선→중앙 +x(우), 우차선→−x
            xc_n = self._offset_normal_x(fit, self.y_near, sgn * hw_px)
            xc_l = self._offset_normal_x(fit, self.y_look, sgn * hw_px)
            curv = abs(fit[0]) * 2.0 / self.px2m

        # 내부 화면좌표계(우측+) — 근점→뒷차축 투영은 이 내부값으로 self-consistent 유지
        cte_near = (xc_n - self.bev_w * 0.5) * self.px2m
        dx_px = xc_l - xc_n
        dy_px = max(1.0, self.y_near - self.y_look)
        theta = -math.atan2(dx_px, dy_px) + math.radians(self.yaw_off)   # [rad, CCW+]
        cte_rear = cte_near + self.d_near * math.sin(theta)

        # 점프 게이트
        if (self.prev_cte_near is not None
                and (now - self.prev_t) < 0.30
                and abs(cte_near - self.prev_cte_near) > self.jump_max):
            flags |= F_JUMP
        self.prev_cte_near = cte_near
        self.prev_t = now

        if conf_raw < self.conf_min:
            flags |= F_CONF_LOW

        # 치명 게이트(폭·점프·conf) 실패 → conf_eff=0 → gps_imu 가 자동으로 카메라 미반영
        fatal = flags & (F_WIDTH_BAD | F_JUMP | F_CONF_LOW)
        conf_eff = 0.0 if fatal else conf_raw

        # [검증된 부호修正] 최종 발행값만 부호反전 → GPS CTE 규약(우측+)에 정합
        self._publish_metrics(-cte_rear, -cte_near, math.degrees(theta), curv,
                              conf_eff, width_m, flags, conf_raw)

    def _publish_metrics(self, cte_rear, cte_near, theta_deg, curv,
                         conf_eff, width_m, flags, conf_raw):
        m = Float32MultiArray()
        m.data = [float(cte_rear), float(cte_near), float(theta_deg), float(curv),
                  float(conf_eff), float(width_m), float(flags),
                  float(self.d_near), float(conf_raw), float(self.seq)]
        self.pub_metrics.publish(m)

    # ══════════════════════════════════════════════════════════════════
    # 2) 신호등/횡단보도 게이트  (/cmd_vel_drive → /cmd_vel_raw)
    # ══════════════════════════════════════════════════════════════════
    def cb_tl(self, msg: String):
        s = msg.data.strip().upper()
        now = time.time()
        if s in ("RED", "RED_FAR"):
            if self.red_since is None:
                self.red_since = now
            self.red_last_seen = now
        elif (self.red_last_seen is None
              or (now - self.red_last_seen) > self.tl_gap_grace_s):
            # 마지막 RED 목격 후 tl_gap_grace_s 를 넘겨서야 리셋 → 짧은 UNKNOWN
            # 끊김(깜빡임/오검출)은 스트릭(red_since)을 죽이지 않는다.
            self.red_since = None
            self.red_last_seen = None
        self.tl_state = s if s in TL_CODE else "UNKNOWN"
        self.tl_time  = now

    def cb_stop(self, msg: Float32):
        self.stop_dist = float(msg.data)
        self.stop_time = time.time()

    def cb_cross(self, msg: Float32):
        self.cw_dist = float(msg.data)
        now = time.time()
        if 0.0 <= self.cw_dist <= self.cw_enter_dist:
            self.cw_time = now

    def _red_confirmed(self):
        """RED/RED_FAR 가 tl_hold_s 이상 지속 + 토픽 신선 → 확정(깜빡임 필터).

        [2026-07-29 수정] 신선도 한계가 1.0초 하드코딩이라 실차에서 빨간불을 그냥 통과했다.
          실측(rosbag 17_06_28): perception 이 ~3.5초씩 6회 멈춰 /tl/state 가 끊겼고,
          출발 시점(t=2.97)의 마지막 RED 는 t=1.09 → age 1.88s > 1.0 → STALE 로 버려짐
          → _red_confirmed()=False → 게이트 PASS → 차량 그대로 출발(judgment_state 전부 PASS).
          perception 이 정상 구간에서도 6.5Hz 뿐이라 1.0초는 너무 빡빡하다.
          → tl_state_max_age 파라미터로 승격(기본 3.0s). 값을 넘으면 여전히 fail-open 이라
            perception 이 완전히 죽어도 차가 영영 못 가는 일은 없다.
        """
        if self.red_since is None:
            return False
        now = time.time()
        if (now - self.tl_time) > self.tl_state_max_age:
            return False
        return (now - self.red_since) >= self.tl_hold_s

    def _crosswalk_active(self):
        if not self.cw_enable or self.cw_time <= 0.0:
            return False
        return (time.time() - self.cw_time) <= self.cw_hold_s

    def cb_cmd(self, msg: Twist):
        """driving 의 경로추종 명령을 받아 신호등/횡단보도 정지·서행을 덮어 발행.

        [white2] 입력 /cmd_vel_drive · 출력 /cmd_vel_raw 모두 linear.x = ★m/s★ 다 —
          motor.py 가 m/s 를 직접 받으므로 kasa 처럼 발행 직전 펄스 환산을 할 필요가
          없다(정지·서행 판정 v=√(2·a·d) 도 그대로 m/s 로 성립).
        조향 부호는 건드리지 않는다 — driving 이 이미 1/5카 규약(+ 좌 / − 우)으로
        보내므로 그대로 통과시킨다(클램프만 한다)."""
        out = Twist()
        out.angular.z = ku.clamp_steer_deg(msg.angular.z)  # 조향은 항상 driving(GPS 경로추종)
        v_in = float(msg.linear.x)
        sign = 1.0 if v_in >= 0.0 else -1.0
        mag  = abs(v_in)
        state = "PASS"
        now = time.time()

        # ── 신호등 ──
        if self.tl_enable and self._red_confirmed():
            sd_fresh = (now - self.stop_time) <= self.tl_stale_s
            sd = self.stop_dist if (sd_fresh and self.stop_dist >= 0.0) else -1.0
            if sd >= 0.0:
                if sd <= self.tl_stop_margin:
                    mag = 0.0
                    state = "TL_STOP"
                else:
                    v_brk = math.sqrt(max(0.0, 2.0 * self.tl_decel *
                                          (sd - self.tl_stop_margin)))
                    if self.tl_state == "RED_FAR" and sd > 6.0:
                        v_brk = min(max(v_brk, 0.3), max(self.tl_far_cap, 0.3))
                    if v_brk < mag:
                        mag = v_brk
                        state = "TL_BRAKE"
                        if mag <= 0.05:
                            mag = 0.0
                            state = "TL_STOP"
            elif self.tl_nodist_stop and self.tl_state == "RED":
                # [2026-07-29] RED 확정 + 정지선 미검출 → 정지.
                #   구 동작은 tl_nodist_cap(0.8m/s) 서행뿐이라, 정지선을 못 잡으면
                #   (실측 stop_line_dist 100% -1) 빨간불에서 영원히 안 섰다.
                #   RED_FAR(멀리 보이는 적색)은 아직 서행만 — 교차로 한참 전 급정지 방지.
                mag = 0.0
                state = "TL_STOP"
            else:
                cap = self.tl_far_cap if self.tl_state == "RED_FAR" else self.tl_nodist_cap
                if cap < mag:
                    mag = cap
                    state = "TL_SLOW"

        # ── 횡단보도(정지 아님, 서행) ──
        if self._crosswalk_active() and self.cw_slow_cap < mag:
            mag = self.cw_slow_cap
            state = "CW_SLOW" if state == "PASS" else state

        # [white2] m/s 그대로 발행 — motor.py 가 직접 받는다.
        speed_ms = sign * mag if mag > 0.0 else 0.0
        out.linear.x = speed_ms
        self.pub_cmd.publish(out)

        # ★★ 신호등 완전정지 = 리니어모터 2단 체결 ★★
        #   'TL_STOP'(빨간불 확정 + 정지 판정)에서만 물린다. TL_BRAKE(감속 중)·TL_SLOW(서행)
        #   에서는 걸지 않는다 — 굴러가는 중에 리니어를 밟으면 인휠 PID 와 싸우고, 감속은
        #   속도명령 자체를 줄이는 것으로 충분하다. 완전정지에서만 기계적으로 잡아준다.
        #   ※ 값이 바뀔 때만 발행한다(B보드는 같은 단계면 무시하지만 토픽을 조용히 유지).
        self._apply_brake(self.tl_brake_level if state == "TL_STOP"
                          else self.tl_brake_release_level)

        if state != self._last_state:
            self.pub_state.publish(String(data=state))
            self._last_state = state

    def _apply_brake(self, level):
        """리니어 브레이크 단계를 요청한다 (변할 때만 발행).

        ★이 노드가 /brake_level 을 쓰는 유일한 목적은 '신호등 완전정지'다★
        수동조종 전환이나 모드 변경으로는 절대 걸지 않는다 — 그 래치는 2026-08-04 에
        arduino.py 에서 제거됐고(스위치를 내리는 순간 리니어가 튀어나왔다) 되살리지 않는다.
        """
        level = max(0, min(2, int(level)))
        if level == self.brake_now:
            return
        self.brake_now = level
        self.pub_brake.publish(Int32(data=level))
        self.get_logger().info(
            f"🛑 리니어 브레이크 {level}단 "
            f"({'체결 — 신호등 정지' if level > 0 else '해제'})")

    # ══════════════════════════════════════════════════════════════════
    def _status_tick(self):
        now = time.time()
        if self.last_lane_rx == 0.0:
            self.get_logger().warn("⏳ /lane/state 수신 대기 (perception 미기동?)",
                                   throttle_duration_sec=5.0)
        elif now - self.last_lane_rx > self.stale_warn:
            self.get_logger().warn(
                f"⛔ /lane/state {now - self.last_lane_rx:.1f}s 두절 — 카메라/perception 확인!",
                throttle_duration_sec=5.0)

        # /judgment_state 주기 재발행 — 상태 '변화' 시에만 발행하면(cb_cmd 참조)
        #   게이트가 PASS 로 안정된 뒤 뜬 구독자(sensor_monitor)는 영원히 아무것도
        #   못 받아 고장처럼 보인다. 1Hz 로 현재 상태를 다시 실어준다.
        #   ⚠ pub_state 의 qos 객체는 pub_cmd(/cmd_vel_raw)와 공유하므로 여기에
        #     TRANSIENT_LOCAL 을 붙이면 늦게 뜬 motor 에 낡은 속도명령이 배달된다.
        #     그래서 QoS 대신 재발행으로 푼다. (엣지 발행은 그대로 유지 — 1Hz 보다
        #     빠른 상태변화도 depth=10 큐로 전달된다.)
        if self._last_state:
            self.pub_state.publish(String(data=self._last_state))


def main(args=None):
    rclpy.init(args=args)
    node = CameraJudgment()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 정지 명령 1회(안전)
        try:
            node.pub_cmd.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
