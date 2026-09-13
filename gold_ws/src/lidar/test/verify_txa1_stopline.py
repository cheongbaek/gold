#!/usr/bin/env python3
"""Replay ~/rosbag/txa1 against cone_lidar_node and score stop-line AEB.

txa1 (2026-07-15, 1/5카, ~19.5 s, 10 Hz):
  t≲6 s   양옆 세로줄만 보임 → stop_signal 이 켜지면 실패 (옛 ROI 오탐)
  t≳7.5 s ㄷ 자 종단 가로줄이 전방 → stop_signal 이 꺼져 있으면 실패
"""
from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32

BAG = os.path.expanduser("~/rosbag/txa1")
PARAMS = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "config",
    "cone_lidar_txa1.yaml",
)

SIDE_ONLY_END_S = 6.0
GATE_START_S = 7.5
MAX_SIDE_TRUE_FRAC = 0.05
MIN_GATE_TRUE_FRAC = 0.70
FIRST_TRUE_WINDOW = (5.5, 8.5)


class Probe(Node):
    def __init__(self):
        super().__init__("txa1_stopline_probe")
        self.samples = []  # (t_rel, stop, dist, n_cloud)
        self.n_cloud = 0
        self.t0_ns = None
        self.last_t = 0.0
        self.last_stop = False
        self.last_dist = float("inf")
        self.create_subscription(
            PointCloud2, "/ouster/points", self._on_cloud, qos_profile_sensor_data
        )
        self.create_subscription(Bool, "/cone_lidar_node/stop_signal", self._on_stop, 10)
        self.create_subscription(
            Float32, "/cone_lidar_node/obstacle_distance", self._on_dist, 10
        )

    def _on_cloud(self, msg: PointCloud2):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if self.t0_ns is None:
            self.t0_ns = stamp
        self.last_t = (stamp - self.t0_ns) * 1e-9
        self.n_cloud += 1

    def _on_dist(self, msg: Float32):
        self.last_dist = float(msg.data)

    def _on_stop(self, msg: Bool):
        self.last_stop = bool(msg.data)
        self.samples.append((self.last_t, self.last_stop, self.last_dist, self.n_cloud))


def frac(samples, pred):
    if not samples:
        return 0.0, 0
    n = sum(1 for s in samples if pred(s))
    return n / len(samples), n


def score(samples, n_cloud):
    lines = []
    ok = True
    n = len(samples)
    lines.append(f"clouds={n_cloud}  stop_msgs={n}")
    if n < 80:
        return False, lines + [f"FAIL: stop_signal 표본이 너무 적다 ({n} < 80)"]

    t_max = samples[-1][0]
    n_true = sum(1 for s in samples if s[1])
    first_true = next((s[0] for s in samples if s[1]), None)
    lines.append(f"bag_span={t_max:.2f}s  stop_true={n_true}/{n} ({100*n_true/n:.1f}%)")
    lines.append(f"first_true={first_true}")

    side = [s for s in samples if s[0] <= SIDE_ONLY_END_S]
    gate = [s for s in samples if s[0] >= GATE_START_S]
    side_f, side_n = frac(side, lambda s: s[1])
    gate_f, gate_n = frac(gate, lambda s: s[1])
    lines.append(
        f"side-only t<= {SIDE_ONLY_END_S:.1f}s : true {side_n}/{len(side)} "
        f"({100*side_f:.1f}%)  허용 <={100*MAX_SIDE_TRUE_FRAC:.0f}%"
    )
    lines.append(
        f"gate     t>= {GATE_START_S:.1f}s : true {gate_n}/{len(gate)} "
        f"({100*gate_f:.1f}%)  요구 >={100*MIN_GATE_TRUE_FRAC:.0f}%"
    )

    if not side:
        ok = False
        lines.append("FAIL: 세로줄 구간 표본 없음")
    elif side_f > MAX_SIDE_TRUE_FRAC:
        ok = False
        lines.append(
            "FAIL: 세로줄만 있는 구간에서 정지했다 "
            f"({100*side_f:.1f}% > {100*MAX_SIDE_TRUE_FRAC:.0f}%)"
        )
    else:
        lines.append("PASS: 세로줄 구간은 정지하지 않음")

    if not gate:
        ok = False
        lines.append("FAIL: 가로줄 구간 표본 없음")
    elif gate_f < MIN_GATE_TRUE_FRAC:
        ok = False
        lines.append(
            "FAIL: 종단 가로줄 구간에서 정지가 부족 "
            f"({100*gate_f:.1f}% < {100*MIN_GATE_TRUE_FRAC:.0f}%)"
        )
    else:
        lines.append("PASS: 종단 가로줄을 잡고 정지함")

    if first_true is None:
        ok = False
        lines.append("FAIL: 한 번도 가로줄을 못 봤다")
    elif not (FIRST_TRUE_WINDOW[0] <= first_true <= FIRST_TRUE_WINDOW[1]):
        ok = False
        lines.append(
            f"FAIL: 첫 정지 t={first_true:.2f}s 가 "
            f"{FIRST_TRUE_WINDOW} 밖 (너무 이르면 세로줄 오탐, 너무 늦으면 놓침)"
        )
    else:
        lines.append(f"PASS: 첫 정지 t={first_true:.2f}s 가 가로줄 출현 구간")

    gate_d = [s[2] for s in gate if s[1] and s[2] < 100]
    if gate_d:
        lines.append(
            f"gate distance while stopping: min={min(gate_d):.2f} "
            f"med={sorted(gate_d)[len(gate_d)//2]:.2f} max={max(gate_d):.2f} m"
        )

    # 1초 격자 요약
    lines.append("t[s]  stop%  n")
    t = 0.0
    while t < t_max:
        bucket = [s for s in samples if t <= s[0] < t + 1.0]
        if bucket:
            f, _ = frac(bucket, lambda s: s[1])
            lines.append(f" {t:4.0f}   {100*f:5.1f}  {len(bucket)}")
        t += 1.0
    return ok, lines


def main() -> int:
    if not os.path.isdir(BAG):
        print(f"FAIL: bag not found: {BAG}", file=sys.stderr)
        return 2
    if not os.path.isfile(PARAMS):
        print(f"FAIL: params not found: {PARAMS}", file=sys.stderr)
        return 2

    env = os.environ.copy()
    env.setdefault("ROS_DOMAIN_ID", "87")
    procs = []

    def spawn(cmd, name):
        log = open(f"/tmp/{name}.log", "w")
        p = subprocess.Popen(
            cmd, env=env, stdout=log, stderr=subprocess.STDOUT, preexec_fn=os.setsid
        )
        procs.append((p, log, name))
        return p

    try:
        node_p = spawn(
            [
                "ros2",
                "run",
                "lidar",
                "cone_lidar_node",
                "--ros-args",
                "--params-file",
                PARAMS,
                "-p",
                "use_sim_time:=true",
            ],
            "cone_lidar_txa1",
        )
        time.sleep(1.5)
        if node_p.poll() is not None:
            print("FAIL: cone_lidar_node 가 바로 종료됐다. /tmp/cone_lidar_txa1.log")
            print(open("/tmp/cone_lidar_txa1.log").read()[-2000:])
            return 3

        rclpy.init()
        probe = Probe()
        play = spawn(
            [
                "ros2",
                "bag",
                "play",
                BAG,
                "--clock",
                "--rate",
                "2.0",
                "--disable-keyboard-controls",
            ],
            "txa1_play",
        )

        deadline = time.time() + 40.0
        while time.time() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.05)
            if play.poll() is not None and probe.n_cloud >= 50:
                # 재생이 끝났어도 마지막 stop 이 올 시간을 준다
                end = time.time() + 1.0
                while time.time() < end:
                    rclpy.spin_once(probe, timeout_sec=0.05)
                break
        ok, report = score(probe.samples, probe.n_cloud)
        print("\n".join(report))
        print("RESULT:", "PASS" if ok else "FAIL")
        probe.destroy_node()
        rclpy.shutdown()
        return 0 if ok else 1
    finally:
        for p, log, name in procs:
            if p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGINT)
                    p.wait(timeout=3)
                except Exception:
                    try:
                        os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                    except Exception:
                        pass
            log.close()


if __name__ == "__main__":
    sys.exit(main())
