#!/usr/bin/env python3
"""mppi_local_planner_node 폐루프 ROS 시험 — 가짜 차량 + 가짜 driving [2026-09-28]

★실차 스택과 섞이지 않게 반드시 다른 도메인에서 돌린다★ (실차는 ROS_DOMAIN_ID=7)
    export ROS_DOMAIN_ID=77
    install/mppi_local_planner/lib/mppi_local_planner/mppi_local_planner_node --ros-args \
        --params-file src/mppi_local_planner/config/params.yaml -p debug.dump_enable:=false &
    NPID=$!; python3 src/mppi_local_planner/test/ros_smoke.py 50 [일시정지_시작_X]; kill -INT $NPID
★pkill -f 로 정리하지 말 것★ — 같은 이름의 실차 노드까지 죽는다.

Closed-loop ROS smoke test for mppi_local_planner_node (fake car + fake driving).

  0-2 s   standstill (gyro bias calibration)
  then    'driving' moves the car along the approach at 1.3 m/s, /lstatus='0'
  s >= 0  /lstatus='L' → the node drives; this script integrates its /cmd_vel_raw
"""
import math
import struct
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import Bool, Float64MultiArray, Int32, String

CONES = [(8.5, -0.71), (8.5, -0.35), (8.5, 0.01), (16.7, 0.45), (16.7, 1.01), (16.7, 1.57)]
S_L_START, S_L_END = 0.0, 22.0
PAUSE_AT = float(sys.argv[2]) if len(sys.argv) > 2 else None
PAUSE_S = 2.0
ANT_X = 1.25       # GPS 안테나(앞차축 위) ↔ 라이다 원점(뒷차축 위) [m] = 축거
L = 1.25


class Fake(Node):
    def __init__(self):
        super().__init__('fake_car')
        self.pub_pts = self.create_publisher(PointCloud2, '/ouster/points', qos_profile_sensor_data)
        self.pub_imu = self.create_publisher(Imu, '/ouster/imu', qos_profile_sensor_data)
        self.pub_enc = self.create_publisher(Int32, '/encoder', 10)
        self.pub_ref = self.create_publisher(Float64MultiArray, '/lidar_ref', 10)
        self.pub_ls = self.create_publisher(String, '/lstatus', 10)
        self.pub_mode = self.create_publisher(Bool, '/vehicle_mode', 10)
        self.pub_estop = self.create_publisher(Bool, '/estop', 10)
        self.create_subscription(Twist, '/cmd_vel_raw', self.cb_cmd, 10)
        self.create_subscription(Int32, '/brake_level', self.cb_brake, 10)
        self.create_subscription(Float64MultiArray, '/lidar_diag', self.cb_diag, 10)
        self.create_subscription(Float64MultiArray, '/lidar_cones', self.cb_cones, 10)
        self.X, self.Y, self.psi = -9.0, 0.53, math.radians(-8.0)
        self.v = 0.0
        self.steer = 0.0
        self.cmd_pulse, self.cmd_pot, self.cmd_n = 0, 0.0, 0
        self.cmd_while_silent = 0
        self.brake = 0
        self.diag = None
        self.n_cones_msgs = 0
        self.t0 = time.time()
        self.log = []
        self.min_clear = 99.0
        self.max_d = 0.0
        self.create_timer(0.05, self.tick)
        self.create_timer(0.01, self.tick_imu)
        self.lstatus = '0'
        self.pause_t0 = None

    def cb_cmd(self, m):
        self.cmd_pulse, self.cmd_pot = int(m.linear.x), m.angular.z
        self.cmd_n += 1
        if self.lstatus != 'L':
            self.cmd_while_silent += 1

    def cb_brake(self, m):
        self.brake = m.data

    def cb_diag(self, m):
        self.diag = list(m.data)

    def cb_cones(self, m):
        self.n_cones_msgs += 1

    def pot_to_road(self, pot_board, v):
        target = -pot_board          # + = left
        a = abs(target)
        if a < 1e-6:
            return 0.0
        lo, hi = 0.0, 40.0           # invert pot = 1.26 d + 5.17 v^2 tan(d)/L
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            pot = 1.26 * mid + 5.17 * v * v * math.tan(math.radians(mid)) / L
            lo, hi = (mid, hi) if pot < a else (lo, mid)
        return math.copysign(math.radians(0.5 * (lo + hi)), target)

    def tick_imu(self):
        m = Imu()
        m.orientation_covariance[0] = -1.0
        m.header.stamp = self.get_clock().now().to_msg()
        m.angular_velocity.z = self.v * math.tan(self.steer) / L
        self.pub_imu.publish(m)

    def tick(self):
        t = time.time() - self.t0
        dt = 0.05
        self.pub_mode.publish(Bool(data=True))
        self.pub_estop.publish(Bool(data=False))
        in_l = S_L_START <= self.X < S_L_END and t > 3.0
        paused = False
        if PAUSE_AT is not None and in_l and self.X >= PAUSE_AT and self.pause_t0 is None:
            self.pause_t0 = t
        if self.pause_t0 is not None and t - self.pause_t0 < PAUSE_S:
            paused = True
        self.lstatus = 'L' if (in_l and not paused) else '0'
        if paused:                       # E-STOP: driving revokes and the car stops
            self.v = max(0.0, self.v - 2.2 * dt)
            in_l = False
        if paused:
            pass
        elif t > 3.0 and not in_l:        # 'driving' controls the car
            self.v = 1.3
            if self.X < S_L_START:
                self.steer = 0.0
            else:                        # after L: simple return to line
                self.steer = max(-0.3, min(0.3, -0.5 * self.Y - 1.0 * self.psi))
        elif in_l:
            self.v = 0.0 if self.brake >= 2 else (0.7 * min(self.cmd_pulse, 2) if self.cmd_pulse > 0 else max(0.0, self.v - 0.41 * dt))
            want = self.pot_to_road(self.cmd_pot, self.v)
            d = want - self.steer
            self.steer += math.copysign(min(abs(d), math.radians(55) * dt), d)
        else:
            self.v = 0.0
        self.X += self.v * math.cos(self.psi) * dt
        self.Y += self.v * math.sin(self.psi) * dt
        self.psi += self.v * math.tan(self.steer) / L * dt
        # driving-side topics
        self.pub_ls.publish(String(data=self.lstatus))
        zone_left = (S_L_START - self.X) if self.X < S_L_START else float('nan')
        valid = 1.0 if t > 3.0 else 0.0
        #  실차처럼 ★안테나 자리★ 의 CTE 를 낸다 (안테나가 라이다보다 ANT_X 앞 — 노드가 되돌린다)
        cte_ant = self.Y + ANT_X * math.sin(self.psi)
        self.pub_ref.publish(Float64MultiArray(data=[cte_ant, math.degrees(self.psi), valid, zone_left]))
        self.pub_enc.publish(Int32(data=int(round(2 * self.v / 0.884))))
        self.publish_cloud()
        # metrics (body rectangle vs cone circles r=0.12)
        for (cs, cd) in CONES:
            dx, dy = cs - self.X, cd - self.Y
            bx = dx * math.cos(self.psi) + dy * math.sin(self.psi)
            by = -dx * math.sin(self.psi) + dy * math.cos(self.psi)
            ex = max(-0.3 - bx, 0.0, bx - 1.25)
            ey = max(-0.54 - by, 0.0, by - 0.54)
            c = (math.hypot(ex, ey) if (ex > 0 or ey > 0) else -min(bx + 0.3, 1.25 - bx, by + 0.54, 0.54 - by)) - 0.12
            self.min_clear = min(self.min_clear, c)
        if in_l:
            self.max_d = max(self.max_d, abs(self.Y))
        if int(t * 20) % 10 == 0:
            self.log.append((t, self.X, self.Y, math.degrees(self.psi), self.lstatus, self.cmd_pulse,
                             self.cmd_pot, self.brake, None if self.diag is None else self.diag[9],
                             None if self.diag is None else self.diag[10]))

    def publish_cloud(self):
        pts = []
        c, s = math.cos(self.psi), math.sin(self.psi)
        for (cs, cd) in CONES:
            for a in range(16):
                th = a * 2 * math.pi / 16
                ws, wd = cs + 0.12 * math.cos(th), cd + 0.12 * math.sin(th)
                dx, dy = ws - self.X, wd - self.Y
                ex, ey = dx * c + dy * s, -dx * s + dy * c
                if math.hypot(ex, ey) > 18.0:
                    continue
                pts.append((-ex, -ey, -0.5))
        m = PointCloud2()
        m.header.frame_id = 'os_lidar'
        m.header.stamp = self.get_clock().now().to_msg()
        m.height, m.width = 1, len(pts)
        m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                    for i, n in enumerate('xyz')]
        m.point_step, m.row_step = 12, 12 * len(pts)
        m.is_dense = True
        m.data = b''.join(struct.pack('fff', *p) for p in pts)
        self.pub_pts.publish(m)


def main():
    rclpy.init()
    n = Fake()
    end = time.time() + float(sys.argv[1] if len(sys.argv) > 1 else 40)
    while rclpy.ok() and time.time() < end and n.X < 34.0:
        rclpy.spin_once(n, timeout_sec=0.01)
    print(" t     X      Y     psi  ls pulse  pot  brake tgt_y n_blk")
    for r in n.log[::2]:
        print(f"{r[0]:5.1f} {r[1]:6.2f} {r[2]:+5.2f} {r[3]:+6.1f}  {r[4]}  {r[5]:3d} {r[6]:+6.1f}  {r[7]}  "
              f"{'' if r[8] is None else f'{r[8]:+.2f}'} {'' if r[9] is None else int(r[9])}")
    print(f"RESULT min_clear={n.min_clear:+.2f} m  max|d| in L={n.max_d:.2f}  cmd_msgs={n.cmd_n}  "
          f"cmd_while_not_L={n.cmd_while_silent}  cones_msgs={n.n_cones_msgs}  final X={n.X:.1f} Y={n.Y:+.2f}")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
