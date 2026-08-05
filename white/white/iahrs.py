#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""iahrs — iAHRS IMU 시리얼 드라이버 → /imu/data (Ubuntu 22.04 전용)
────────────────────────────────────────────────────────────────────────────────
★ [2026-08-05] IMU 는 white 소관으로 되돌아왔다 ★
  하드웨어 담당을 다시 나눴다 — one_launch.py 가 GPS·IMU·카메라를, nxde 의
  g.launch.py 가 아두이노 A/B 만 담당한다. 이 파일은 one_launch.py 가 띄운다:
      ros2 launch white one_launch.py      (iahrs · gps · usb_cam 포함)
  2026-08-04 에 nxde 로 잠시 옮겨갔던 판을 그대로 되가져온 것이므로 ★기능 손실이 없다★
  (nxde/nxde/iahrs.py 는 삭제됐다. 참조 모듈만 nxde.ports → white.ports 로 바뀐다.)

★ 1/5카 원본 대비 변경점은 '포트 재탐색' 하나뿐이다 ★
  원본에도 2초 주기 재연결 루프가 이미 있었지만, **런치가 한 번 잡아준 고정 경로로만**
  재시도했다. 그래서 켤 때 IMU 가 안 꽂혀 있었으면(경로가 /dev/tty_NOT_FOUND) 나중에
  꽂아도 영원히 못 잡았고, USB 재연결로 경로 번호가 바뀌어도(ttyUSB0 → ttyUSB1) 못 잡았다.
  → 재연결마다 white/ports.py 의 VID/PID 스캔을 한 번 더 돌려 경로를 다시 정한다.
    (`rescan` 파라미터로 끌 수 있다. udev 심볼릭링크를 쓰면 사실 필요 없다)

  IMU 데이터 파싱·공분산·TF 브로드캐스트 로직은 원본과 완전히 동일하다.
"""
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped

import tf2_ros
import serial
import time
import math

from white import ports

# 🌟 외부 라이브러리(transforms3d) 의존성 제거! 자체 수학 연산 적용
def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return w, x, y, z

class IahrsDriver(Node):
    CONF_SYNC_LIN_ACC = 0x0004
    CONF_SYNC_ANG_VEL = 0x0008
    CONF_SYNC_EULER = 0x0040

    def __init__(self):
        super().__init__("iahrs_driver_node")

        # 🔧 BUG1 수정: get_parameter_or()는 rclpy에 없음 → declare_parameter + .value 사용
        self.declare_parameter("tf_prefix", "")
        self.declare_parameter("send_tf", True)
        # 🔧 BUG5 수정: 포트 하드코딩 제거 → 런치파일에서 주입 가능
        # ★ 기본값을 udev 심볼릭링크로 둔다 ★ /dev/ttyUSB0 은 열거 순서에 따라 GPS 나
        #   아두이노가 될 수 있다. udev 설정법은 nxde/README.md 8절 참고.
        #   런치 전 `ros2 run nxde check` 로 실제 경로를 먼저 확인할 수 있다.
        self.declare_parameter("port", ports.SYMLINK_IMU)
        self.declare_parameter("baud", 115200)
        # ★ 재연결마다 VID/PID 로 경로를 다시 찾는다 ★ 켤 때 안 꽂혀 있었거나 USB 재연결로
        #   경로 번호가 바뀌어도 자동으로 붙는다. udev 심볼릭링크를 쓰면 없어도 무해하다.
        self.declare_parameter("rescan", True)
        # GPS 로 이미 확정된 경로 — 재탐색에서 제외한다(그 포트를 열면 배타 open 충돌로
        #   해당 드라이버가 자기 포트를 못 잡는다). one_launch.py 가 넘겨준다.
        #   ※ 아두이노 경로는 넘기지 않아도 된다 — IMU_VIDPID(10c4:ea60) 와 아두이노
        #     VID(2341/1a86/2a03) 가 겹치지 않아 VID/PID 스캔이 애초에 아두이노를 안 고른다.
        self.declare_parameter("exclude_ports", [""])
        # [v6.7 연동] IMU 출력주기[ms]. 기본 50ms=20Hz(기존과 동일).
        #   driving.py의 지연보상 예측 정밀도를 높이려면 20ms(50Hz) 권장:
        #   런치/CLI: --ros-args -p sync_period_ms:=20
        self.declare_parameter("sync_period_ms", 50)

        self._tf_prefix   = self.get_parameter("tf_prefix").value
        self._is_send_tf  = self.get_parameter("send_tf").value
        self.sync_period_ms = int(self.get_parameter("sync_period_ms").value)

        self._br = tf2_ros.TransformBroadcaster(self)

        self.port_name = self.get_parameter("port").value
        self.baud_rate = self.get_parameter("baud").value
        self._rescan = bool(self.get_parameter("rescan").value)
        self._exclude = [str(p) for p in self.get_parameter("exclude_ports").value if str(p)]

        self._msg = Imu()

        # 공분산 설정
        self._msg.linear_acceleration_covariance[0] = 0.0064
        self._msg.linear_acceleration_covariance[4] = 0.0063
        self._msg.linear_acceleration_covariance[8] = 0.0064

        self._msg.angular_velocity_covariance[0] = 0.032 * (math.pi / 180.0)
        self._msg.angular_velocity_covariance[4] = 0.028 * (math.pi / 180.0)
        self._msg.angular_velocity_covariance[8] = 0.006 * (math.pi / 180.0)

        self._msg.orientation_covariance[0] = 0.013 * (math.pi / 180.0)
        self._msg.orientation_covariance[4] = 0.011 * (math.pi / 180.0)
        self._msg.orientation_covariance[8] = 0.006 * (math.pi / 180.0)

        self._imu_pub_handler = self.create_publisher(Imu, "imu/data", 10)

        # 🔧 [크래시 방지] 포트를 못 열어도 노드가 죽지 않게 try로 감싸고,
        #   실패 시 2초마다 재연결을 시도한다. (IMU가 늦게 인식되거나 잠깐 빠져도 복구)
        self._ser = None
        self._reconnect_timer = None
        if not self._try_open_port():
            self._reconnect_timer = self.create_timer(2.0, self._reconnect_cb)

        self.timer = self.create_timer(0.01, self._serial_timer)

    def _resolve_port(self):
        """열 포트를 정한다. rescan=True 면 매 시도마다 VID/PID 로 다시 찾는다.

        ★ 이게 원본과의 유일한 기능 차이다 ★ 원본은 런치가 준 경로만 계속 두드렸다.
        현재 경로가 실제로 존재하면 그대로 쓰고(정상 재연결), 없으면 VID/PID 로 찾는다."""
        if not self._rescan:
            return self.port_name
        resolved = ports.resolve_device(
            self.port_name if self.port_name.startswith('/dev/') else ports.SYMLINK_IMU,
            ports.IMU_VIDPID, exclude=self._exclude)
        if resolved != self.port_name:
            self.get_logger().info(f"iAHRS 포트 재탐색: {self.port_name} → {resolved}")
            self.port_name = resolved
        return self.port_name

    def _try_open_port(self):
        try:
            self._resolve_port()
            # 🌟 타임아웃(0.05초)을 설정하여 노드 무한 프리징 방지!
            ser = serial.Serial(self.port_name, self.baud_rate, timeout=0.05)
            ser.flushInput()
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            self._ser = ser
            self._reset_sensor()
            self.get_logger().info(f"✅ iAHRS 포트 연결: {self.port_name} @ {self.baud_rate}")
            return True
        except Exception as e:
            self._ser = None
            # ★ 로그를 15초로 스로틀한다 ★ 2초마다 무한 재시도하므로 그대로 두면
            #   IMU 를 안 꽂은 동안 터미널이 이 줄로 도배된다(다른 노드 로그가 묻힌다).
            self.get_logger().error(
                f"⚠️ iAHRS 포트 열기 실패({self.port_name}): {e} → 2초마다 재시도 중",
                throttle_duration_sec=15.0)
            return False

    def _reconnect_cb(self):
        if self._ser is not None and self._ser.is_open:
            # 이미 연결됨 → 재연결 타이머 정리
            self._reconnect_timer.cancel()
            self._reconnect_timer = None
            return
        if self._try_open_port():
            self._reconnect_timer.cancel()
            self._reconnect_timer = None

    def _set_sync_port(self):
        self._write_port_timeout("so=1")

    def _set_sync_period(self, period):
        self._write_port_timeout("sp={0}".format(period))

    def _set_sync_data(self, sync_data_type):
        sync_conf = "{:X}".format(sync_data_type)
        self._write_port_timeout("sd=0x" + sync_conf)

    def _is_port_available(self):
        # 🌟 에러를 뿜던 isOpen() 대신 최신 규격인 is_open 사용
        # 🔧 포트 미연결(None) 시에도 안전하게 False 반환
        return self._ser is not None and self._ser.is_open

    def _serial_timer(self):
        if self._is_port_available():
            try:
                line = self._ser.readline()
                if not line:
                    return # 타임아웃 시 안전하게 리턴
                
                # errors='ignore'를 추가하여 깨진 바이트가 들어와도 죽지 않게 보호
                sync_data = line.split(b"\r\n")[0].decode("utf-8", errors='ignore').strip()
                
                if sync_data and len(sync_data.split("=")) == 1:
                    sync_data_splitted = sync_data.split(",")

                    if len(sync_data_splitted) == 9:
                        data = list(map(float, sync_data_splitted))

                        self._msg.linear_acceleration.x = data[0] * 9.80665
                        self._msg.linear_acceleration.y = data[1] * 9.80665
                        self._msg.linear_acceleration.z = data[2] * 9.80665

                        self._msg.angular_velocity.x = data[3] * (math.pi / 180.0)
                        self._msg.angular_velocity.y = data[4] * (math.pi / 180.0)
                        self._msg.angular_velocity.z = data[5] * (math.pi / 180.0)

                        # 자체 함수로 안전하게 쿼터니언 변환
                        w, x, y, z = euler_to_quaternion(
                            data[6] * (math.pi / 180.0), 
                            data[7] * (math.pi / 180.0), 
                            data[8] * (math.pi / 180.0)
                        )
                        
                        self._msg.orientation.w = w
                        self._msg.orientation.x = x
                        self._msg.orientation.y = y
                        self._msg.orientation.z = z

                        self._msg.header.stamp = self.get_clock().now().to_msg()
                        self._msg.header.frame_id = (self._tf_prefix + "/imu_link").lstrip("/")

                        self._imu_pub_handler.publish(self._msg)
                        if self._is_send_tf:
                            self._send_tf()
                            
            except (serial.SerialException, OSError) as e:
                # 🔧 [복구] 포트 자체가 죽은 경우: 포트를 닫고 재연결 타이머를 다시 띄운다.
                self.get_logger().error(f"⚠️ iAHRS 포트 단절: {e} → 재연결 시도")
                try:
                    if self._ser is not None:
                        self._ser.close()
                except Exception:
                    pass
                self._ser = None
                if self._reconnect_timer is None:
                    self._reconnect_timer = self.create_timer(2.0, self._reconnect_cb)
            except Exception as e:
                # 데이터 파싱/디코드 노이즈는 기존처럼 무시 (포트는 정상)
                self.get_logger().warn(f"데이터 파싱 무시됨 (정상적인 노이즈): {e}")

    def _write_port(self, buffer):
        if self._is_port_available():
            self._ser.write((buffer + "\n").encode())

    def _write_port_timeout(self, buffer, timeout=0.5):
        self._write_port(buffer)
        started_time = time.time()
        while True:
            if self._is_port_available():
                line = self._ser.readline()
                if line:
                    data = line.split(b"\r\n")[0].decode("utf-8", errors='ignore').strip()
                    if data and data == buffer:
                        break
            else:
                raise Exception("USB_NOT_CONNECTED")
            
            if time.time() - started_time >= timeout:
                break
        return True

    def _send_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = (self._tf_prefix + "/base_link").lstrip("/")
        t.child_frame_id = (self._tf_prefix + "/imu_link").lstrip("/")

        t.transform.rotation.x = self._msg.orientation.x
        t.transform.rotation.y = self._msg.orientation.y
        t.transform.rotation.z = self._msg.orientation.z
        t.transform.rotation.w = self._msg.orientation.w

        self._br.sendTransform(t)

    def _reset_sensor(self):
        self.get_logger().info("Initializing iAHRS in Relative 6-DOF Mode for RViz2...")
        self._set_sync_port()
        self._set_sync_period(self.sync_period_ms)   # [v6.7 연동] 기본 50ms=20Hz, 권장 20ms=50Hz
        self._set_sync_data(
            self.CONF_SYNC_LIN_ACC | self.CONF_SYNC_ANG_VEL | self.CONF_SYNC_EULER
        )

def main(args=None):
    rclpy.init(args=args)
    iahrs_driver_node = IahrsDriver()
    try:
        rclpy.spin(iahrs_driver_node)
    except KeyboardInterrupt:
        pass
    finally:
        iahrs_driver_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()