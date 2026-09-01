#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mapping.py ― 경로 수집 [white1]
════════════════════════════════════════════════════════════════════════════════
사람이 수동조종으로 몰고 다닌 궤적을 CSV 로 남긴다. driving 이 나중에 이 파일의
latitude·longitude 를 읽어 그대로 따라간다.

★구 white 와 다른 점 : 순수 GPS 만 쓴다★
  구 mapping 은 gps_imu 노드가 만든 /ego_state(GPS+IMU 융합 보정값)를 받아 적었다.
  그 보정 자체가 새 하드웨어에서 문제를 일으키고 있어 갈라선 것이므로, 여기서는
  ★/fix 원값만★ 쓴다. 헤딩·속도도 연속된 두 fix 의 변위에서 직접 낸다.

  카메라 열(lane_cte · lane_conf · lane_flags · lane_theta · lane_curv)은 없앴다.
  driving 의 경로 로더는 latitude/longitude 만 읽으므로 호환에 문제가 없다.

수집 시작·종료는 이 노드가 정하지 않는다 — driving 의 상태기계가 /mapping_cmd 로
지시한다(스위치 자율→수동 = 시작, 수동→자율 = 종료). 헤딩 초기화를 위해 앞으로
굴러가는 구간도 driving 이 맡는다.

기록 규칙 : ★일정 거리마다 한 점★ (시간 간격이 아니라). 신호 대기처럼 멈춰 있는
동안 같은 자리가 수백 줄 쌓이면 경로가 아니라 점 뭉치가 되기 때문이다.
"""

import csv
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Int32, String

from white1 import paths


SPACING_M      = 0.25    # 이 거리마다 한 점 기록 (구 white 의 remodel_spacing 과 동일)
MIN_MOVE_M     = 0.05    # 헤딩·속도를 낼 때 이보다 작은 변위는 노이즈로 본다
ENC_SUM_TO_PULSE = 0.5   # /encoder 는 좌+우 합 → 바퀴 하나 기준(= 양 바퀴 평균)
#  ★[2026-08-14] wheel_speed 열이 실제의 절반이었다★ 종전 상수는 MS_PER_ENC_COUNT
#  =0.442 였는데 그것은 ★합 1카운트★ 당 값이다. 그런데 곱하는 대상(enc_pulse)은 이미
#  ENC_SUM_TO_PULSE 로 절반이 된 '바퀴 하나 기준' 값이라, 절반을 두 번 먹였다.
#  바퀴 하나 기준 1펄스 = 0.884 m/s 다(driving.py 의 MS_PER_PULSE 와 같은 값):
#      둘레 1.697147m / 96펄스(3상 XOR 6에지 × 16극쌍) / 계측창 0.020s
#  ※ 이 열은 기록용이고 driving 의 경로 로더는 latitude/longitude 만 읽는다 —
#    지난 매핑 CSV 의 wheel_speed 를 다시 볼 때는 ★2를 곱해서★ 읽을 것.
MS_PER_PULSE = 0.884     # 바퀴 하나 기준 1펄스 당 m/s
EARTH_R = 6378137.0


class MappingNode(Node):

    def __init__(self):
        super().__init__('mapping_node')

        self.declare_parameter('data_dir', '')
        self.declare_parameter('spacing_m', SPACING_M)
        self.data_dir = paths.data_dir(self.get_parameter('data_dir').value or '')
        self.spacing = float(self.get_parameter('spacing_m').value)
        os.makedirs(self.data_dir, exist_ok=True)

        # 수집 상태
        self.active = False
        self.fp = None
        self.writer = None
        self.path = ''
        self.rows = 0

        # 위치
        self.lat = self.lon = None
        self.prev_lat = self.prev_lon = None    # 헤딩·속도용 직전 fix
        self.prev_t = 0.0
        self.last_lat = self.last_lon = None    # 마지막으로 '기록한' 점
        self.heading = 0.0
        self.speed = 0.0

        # 실계측 부수 정보
        self.drive_pulse = 0
        self.enc_pulse = 0.0
        self.steer_meas = 0
        self.throttle_raw = 0
        self.auto_mode = False
        self.estop = False

        # ★기록되는 순간의 좌표를 그대로 내보낸다★ prompt 가 매핑 진행을 눈으로
        #   확인할 수 있게 한다(2026-08-11) — CSV 에 실제로 쓰인 행만 나간다.
        self.pub_point = self.create_publisher(String, '/mapping_point', 10)

        self.create_subscription(NavSatFix, '/fix',          self.cb_fix,      10)
        self.create_subscription(Bool,      '/mapping_cmd',  self.cb_cmd,      10)
        self.create_subscription(Int32, '/drive_pulse_cmd',      self.cb_dpulse, 10)
        self.create_subscription(Int32, '/encoder',              self.cb_enc,    10)
        self.create_subscription(Int32, '/steer_angle_measured', self.cb_steer,  10)
        self.create_subscription(Int32, '/throttle_pedal',       self.cb_thr,    10)
        self.create_subscription(Bool,  '/vehicle_mode',         self.cb_mode,   10)
        self.create_subscription(Bool,  '/estop',                self.cb_estop,  10)

        self.get_logger().info(f"🗺️ mapping 대기 — 저장 폴더 {self.data_dir}")

    # ── 부수 정보 ──────────────────────────────────────────────────────────────
    def cb_dpulse(self, m): self.drive_pulse = int(m.data)
    def cb_enc(self, m):    self.enc_pulse = float(m.data) * ENC_SUM_TO_PULSE
    def cb_steer(self, m):  self.steer_meas = int(m.data)
    def cb_thr(self, m):    self.throttle_raw = int(m.data)
    def cb_mode(self, m):   self.auto_mode = bool(m.data)
    def cb_estop(self, m):  self.estop = bool(m.data)

    # ── 시작 / 종료 ────────────────────────────────────────────────────────────
    def cb_cmd(self, msg: Bool):
        if bool(msg.data) and not self.active:
            self.start()
        elif not bool(msg.data) and self.active:
            self.stop()

    def start(self):
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.path = os.path.join(self.data_dir, f"route_{stamp}.csv")
        self.fp = open(self.path, 'w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.fp)
        self.writer.writerow([
            # ── 앞 8열은 구 white 와 열 위치가 같다(분석툴 호환) ──
            "latitude", "longitude", "heading", "speed", "steer",
            "direction", "pitch", "terrain",
            # ── 수동조종 실계측 ──
            "throttle_pulse",   # /drive_pulse_cmd (페달 환산 목표펄스)
            "wheel_pulse",      # /encoder 좌+우 합 → 바퀴 기준
            "wheel_speed",      # 위를 m/s 로
            "steer_measured",   # /steer_angle_measured (− 좌 / + 우)
            "throttle_raw",     # A0 페달 원값 0~1023
            "auto_mode",        # 1 자율 / 0 수동 — 수집 유효구간 판별용
            "estop",            # 1 이면 E-STOP(D12 하드웨어) 구간 → 분석에서 제외
        ])
        self.rows = 0
        self.last_lat = self.last_lon = None
        self.active = True
        self.get_logger().info(f"🗺️ 매핑 시작 → {self.path}")

    def stop(self):
        self.active = False
        try:
            self.fp.close()
        except Exception:
            pass
        self.fp = self.writer = None
        if self.rows < 2:
            self.get_logger().warning(
                f"⚠️ 수집된 점이 {self.rows}개뿐이다 — 경로로 쓸 수 없다: {self.path}")
        else:
            self.get_logger().info(f"💾 매핑 종료 — {self.rows}점 저장: {self.path}")

    # ── 수집 ───────────────────────────────────────────────────────────────────
    def cb_fix(self, msg: NavSatFix):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        self.lat, self.lon = msg.latitude, msg.longitude
        now = time.time()

        # 직전 fix 대비 변위로 헤딩·속도를 직접 낸다(융합 없음)
        if self.prev_lat is not None:
            dx, dy = self._delta(self.prev_lat, self.prev_lon, self.lat, self.lon)
            d = math.hypot(dx, dy)
            if d >= MIN_MOVE_M:
                self.heading = math.degrees(math.atan2(dy, dx))
                dt = now - self.prev_t
                if dt > 0:
                    self.speed = d / dt
                self.prev_lat, self.prev_lon, self.prev_t = self.lat, self.lon, now
        else:
            self.prev_lat, self.prev_lon, self.prev_t = self.lat, self.lon, now

        if not self.active:
            return

        # 일정 거리마다 한 점
        if self.last_lat is not None:
            dx, dy = self._delta(self.last_lat, self.last_lon, self.lat, self.lon)
            if math.hypot(dx, dy) < self.spacing:
                return
        self._write_row()
        self.last_lat, self.last_lon = self.lat, self.lon

    def _delta(self, lat0, lon0, lat1, lon1):
        x = EARTH_R * math.radians(lon1 - lon0) * math.cos(math.radians(lat0))
        y = EARTH_R * math.radians(lat1 - lat0)
        return x, y

    def _write_row(self):
        self.writer.writerow([
            f"{self.lat:.8f}", f"{self.lon:.8f}",
            f"{self.heading:.2f}", f"{self.speed:.3f}",
            f"{self.steer_meas:d}",
            1,                      # direction — 후진 수집은 없다
            "0.00", "0",            # pitch / terrain — IMU 지형판정을 쓰지 않는다
            self.drive_pulse,
            f"{self.enc_pulse:.1f}",
            f"{self.enc_pulse * MS_PER_PULSE:.3f}",
            self.steer_meas,
            self.throttle_raw,
            1 if self.auto_mode else 0,
            1 if self.estop else 0,
        ])
        self.rows += 1
        self.pub_point.publish(String(
            data=f"#{self.rows:04d}  lat={self.lat:.7f}  lon={self.lon:.7f}"))
        if self.rows % 20 == 0:
            try:
                self.fp.flush()
            except Exception:
                pass

    def destroy_node(self):
        if self.active:
            self.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
