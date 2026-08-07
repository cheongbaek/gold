#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
motor_node.py
─────────────────────────────────────────────────────────────
ROS2 ↔ Arduino Mega 모터 브리지
- /cmd_vel_raw (geometry_msgs/Twist) → "C,vel_tick,steer_deg\n"
- /control_state (std_msgs/Bool)     → "S,0/1\n"
- Arduino "E,d_val[,dt_us]\n"        → /encoder (std_msgs/Int32)

수정사항 [v5m/s 안정화판 - 매핑 중 9초 리셋 방지 + 5m/s 명령 허용]:
  1) RX watchdog가 아두이노 포트를 강제 리셋하지 않도록 변경
  2) watchdog는 경고와 상태 재전송만 수행
  3) 실제 시리얼 객체가 없을 때만 soft 재연결 시도
  4) DTR 리셋으로 차량이 갑자기 멈추는 현상 방지
─────────────────────────────────────────────────────────────
"""

import time
import threading
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Twist


# ───────────── 상수 ─────────────
SERIAL_PORT          = '/dev/ttyACM0'
SERIAL_BAUD          = 115200
SERIAL_TIMEOUT       = 0.1

WHEEL_CIRC           = 0.8482           # m (바퀴 둘레)
TICKS_PER_REV        = 300              # 엔코더 1회전 틱 수
ENC_DT               = 0.01             # 아두이노 PID 주기 (10 ms)

MAX_SPEED_MS         = 5.0              # m/s 상한 [v5m/s 확장]
MAX_STEER_DEG        = 21.0             # ° 상한

CMD_TX_PERIOD        = 0.05             # 50 ms (20 Hz)
WATCHDOG_TIMEOUT     = 3.0              # RX 무응답 경고 기준 (s) - 리셋 기준 아님
BOOT_GRACE_PERIOD    = 8.0              # 연결 직후 watchdog 비활성화 시간 (s)
WATCHDOG_WARN_PERIOD = 5.0              # 같은 RX 무응답 경고 최소 간격 (s)
HARD_RECONNECT_TIMEOUT = 60.0           # 장시간 RX 무응답 시에만 재연결 시도 (s)
RESET_ON_RX_TIMEOUT  = False            # 중요: True로 하면 예전처럼 주기적 리셋 발생 가능

ARDUINO_PID_PERIOD_US     = 10000       # 기준 PID 주기 (µs)
ARDUINO_PID_TOLERANCE_US  = 1000        # 허용 편차 (±µs)


class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')

        # 🔧 [v6] 포트/보드레이트를 ROS 파라미터로 — 런치에서 주입 가능.
        #   여러 USB 장치(iAHRS/GPS/Arduino) 혼재 시 udev by-id 심볼릭링크 권장:
        #   예) port:=/dev/arduino_mega  (99-white.rules로 고정)
        self.declare_parameter('port', SERIAL_PORT)
        self.declare_parameter('baud', SERIAL_BAUD)
        self.serial_port = self.get_parameter('port').value
        self.serial_baud = int(self.get_parameter('baud').value)

        # ─── 상태 변수 ───
        self.cmd_vel_ms   = 0.0
        self.cmd_steer    = 0.0
        self.state_enable = False

        self.ser          = None
        self.ser_lock     = threading.Lock()
        self.rx_thread    = None
        self.rx_running   = False

        self.last_rx_time   = time.time()
        self.connect_time   = 0.0          # 마지막 연결 성공 시각
        self.last_warn_time = 0.0          # PID 주기 경고 throttle
        self.last_watchdog_warn_time = 0.0 # RX watchdog 경고 throttle
        self.last_tx_ok_time = 0.0         # 마지막 TX 성공 시각
        self.reconnect_in_progress = False # 중복 재연결 방지

        # ─── 퍼블리셔 ───
        self.pub_encoder = self.create_publisher(Int32, '/encoder', 10)
        # [속도 상한 검증] 아두이노 속도PID 의 실제 PWM 출력. 'P,pwm' 프레임으로 수신.
        #   목적: "속도가 안 나는 게 PWM 포화 때문인가"를 로스백으로 직접 확인한다.
        #   (E 프레임은 건드리지 않음 — 기존 파싱/dt 진단 규약 그대로 유지)
        self.pub_pwm = self.create_publisher(Int32, '/motor_pwm', 10)
        # [FIX-5, 2026-07-30] A15 포텐셔미터 실측 조향각. 'A,angle' 프레임으로 수신.
        #   목적: driving.py 의 clamped_steer(명령, 같은 deg 단위)와 직접 대조해
        #   명령→실현 지연·비율을 추정이 아니라 측정으로 구한다(LFD 상향 재시도 때
        #   지연 추정이 로스백마다 0.70~1.25s로 갈려 설계가 실측과 안 맞았던 문제).
        self.pub_steer_angle = self.create_publisher(Int32, '/steer_angle_measured', 10)
        # [FIX-6, 2026-07-30] 조향 PID 의 실제 PWM 출력. 'D,pwm' 프레임으로 수신.
        #   목적: /steer_angle_measured 가 정상속도 주행 중에도 몇 초씩 안 움직이는
        #   구간을 발견했는데(로스백 18_36_42, 82%), 이게 PID 가 PWM 을 안 내는 건지
        #   PWM 은 나가는데 기구적으로 안 움직이는 건지 이걸로 구분한다.
        self.pub_steer_pwm = self.create_publisher(Int32, '/steer_pwm', 10)

        # ─── 서브스크라이버 ───
        self.create_subscription(Twist, '/cmd_vel_raw',
                                 self.cb_cmd_vel, 10)
        self.create_subscription(Bool, '/control_state',
                                 self.cb_state, 10)

        # ─── 초기 연결 ───
        self._connect_serial(self.serial_port, reset_board=True)

        # ─── 타이머 ───
        self.create_timer(CMD_TX_PERIOD, self.tx_loop)
        self.create_timer(1.0, self.watchdog_check)

        # ─── 시작 배너 ───
        max_tick = int(round(MAX_SPEED_MS * ENC_DT *
                             TICKS_PER_REV / WHEEL_CIRC))
        max_real = max_tick * WHEEL_CIRC / TICKS_PER_REV / ENC_DT
        self.get_logger().info(
            '╔══════════════════════════════════════════════════╗')
        self.get_logger().info(
            '║  Motor Node 시작 [IDE-style reset]                ║')
        self.get_logger().info(
            '║  /cmd_vel_raw=m/s → Arduino tick/10ms             ║')
        self.get_logger().info(
            f'║  MAX_SPEED={MAX_SPEED_MS:.2f}m/s → ±{max_tick}틱 '
            f'(≈±{max_real:.2f}m/s)')
        self.get_logger().info(
            f'║  MAX_STEER=±{MAX_STEER_DEG:.1f}°')
        self.get_logger().info(
            '╚══════════════════════════════════════════════════╝')

    # ═══════════════════════════════════════════════════════════
    #  시리얼 연결 / 재연결 - Arduino IDE 시리얼 모니터 방식 재현
    # ═══════════════════════════════════════════════════════════
    def _connect_serial(self, port, reset_board=False):
        """
        시리얼 포트를 연다.

        reset_board=True:
          - 런치 직후 최초 연결용. DTR/RTS 펄스로 아두이노를 깨끗하게 리셋한다.

        reset_board=False:
          - 주행/매핑 중 복구용. DTR/RTS를 리셋 펄스로 흔들지 않는다.
          - Arduino Mega는 포트 open/close 또는 DTR 변화만으로도 리셋될 수 있으므로
            watchdog에서 이 경로를 최대한 사용하지 않는 것이 핵심이다.
        """
        # 이전 연결 정리
        self._close_serial()

        try:
            ser = serial.Serial()
            ser.port     = port
            ser.baudrate = self.serial_baud
            ser.timeout  = SERIAL_TIMEOUT
            # [2026-07-30] 이전 launch 가 완전히 안 죽고 남아 있으면(background 스레드가
            #   blocking read 중이면 SIGINT 만으로는 안 죽는다) pyserial 은 기본적으로
            #   같은 포트를 여러 프로세스가 동시에 여는 것을 막지 않는다. 그 상태로 두면
            #   두 프로세스가 같은 UART 바이트를 나눠 받아 서로 다른 값을 파싱하고,
            #   TX 도 두 프로세스가 동시에 쓸 수 있어 아두이노 쪽 명령 프레임이 깨질 수 있다.
            #   (오늘 로스백에서 /motor_pwm 이 /encoder 의 2.83배로 찍힌 원인 — 아두이노
            #   코드는 E/P 를 항상 1:1로 보내므로 수신측 중복이 확실했다.)
            #   exclusive=True 로 열면 이미 열려 있는 포트는 open() 이 SerialException 을
            #   던져 아래 except 절에서 바로 로그로 드러난다(조용히 중복되는 대신 실패).
            ser.exclusive = True

            if reset_board:
                # 최초 실행 때만 IDE 시리얼 모니터 방식 리셋을 수행한다.
                ser.dtr = True
                ser.rts = True
                ser.open()

                ser.dtr = False
                ser.rts = False
                time.sleep(0.05)

                ser.dtr = True
                ser.rts = True

                self.get_logger().info('⏳ Arduino 부팅 대기 중... (3.0s)')
                time.sleep(3.0)
            else:
                # 주행 중 복구용 soft-open: 의도적인 LOW 펄스를 주지 않는다.
                ser.dtr = True
                ser.rts = True
                ser.open()
                time.sleep(0.2)

            try:
                ser.reset_input_buffer()
                ser.reset_output_buffer()
            except Exception:
                pass

            self.ser = ser
            self.connect_time   = time.time()
            self.last_rx_time   = time.time()

            # RX 스레드 시작
            self.rx_running = True
            self.rx_thread = threading.Thread(
                target=self.read_serial, daemon=True)
            self.rx_thread.start()

            self.get_logger().info(
                f'✅ Arduino 연결: {port} @ {self.serial_baud}bps '
                f'({"hard-reset" if reset_board else "soft-open"})')
            return True

        except serial.SerialException as e:
            self.get_logger().error(
                f'❌ Arduino 연결 실패: {e}')
            self.ser = None
            return False

    def _close_serial(self):
        """기존 시리얼 자원 정리."""
        self.rx_running = False
        if self.rx_thread is not None:
            try:
                self.rx_thread.join(timeout=0.5)
            except Exception:
                pass
            self.rx_thread = None

        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    # ═══════════════════════════════════════════════════════════
    #  RX 스레드 - 아두이노 → ROS
    # ═══════════════════════════════════════════════════════════
    def read_serial(self):
        """
        아두이노로부터 "E,d_val\n" 또는 "E,d_val,dt_us\n" 수신.
        d_val : 부호 있는 틱/10ms
        dt_us : (선택) 실제 PID 주기 µs (DIAG_DT=1일 때만)
        """
        buf = b''
        while self.rx_running and self.ser is not None:
            try:
                data = self.ser.read(64)
                if not data:
                    continue
                buf += data

                # 줄 단위 처리
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    line = line.strip().decode('ascii', errors='ignore')
                    if not line:
                        continue

                    parts = line.split(',')
                    if len(parts) < 2:
                        continue

                    # [속도 상한 검증] 'P,pwm' — 아두이노 속도PID 의 실제 PWM 출력.
                    #   E 프레임과 독립된 별도 프레임이라 기존 규약에 영향 없다.
                    #   펌웨어가 아직 P 를 안 보내면 이 분기는 그냥 안 타므로 하위호환.
                    if parts[0] == 'P':
                        try:
                            pwm_msg = Int32()
                            pwm_msg.data = int(float(parts[1]))
                            self.pub_pwm.publish(pwm_msg)
                        except ValueError:
                            pass
                        continue

                    # [FIX-5] 'A,angle' — A15 포텐셔미터 실측 조향각(deg).
                    #   P 와 같은 독립 프레임 방식. 펌웨어가 아직 A 를 안 보내면
                    #   이 분기는 안 타므로 하위호환.
                    if parts[0] == 'A':
                        try:
                            angle_msg = Int32()
                            angle_msg.data = int(float(parts[1]))
                            self.pub_steer_angle.publish(angle_msg)
                        except ValueError:
                            pass
                        continue

                    # [FIX-6] 'D,pwm' — 조향PID 의 실제 PWM 출력. A 와 짝지어 보면
                    #   기구 고착(D 큼·각도 안 변함) vs 전원/PID 미출력(D≈0) 을 가른다.
                    if parts[0] == 'D':
                        try:
                            spwm_msg = Int32()
                            spwm_msg.data = int(float(parts[1]))
                            self.pub_steer_pwm.publish(spwm_msg)
                        except ValueError:
                            pass
                        continue

                    if parts[0] != 'E':
                        continue

                    # d_val 파싱
                    try:
                        d_val = int(float(parts[1]))
                    except ValueError:
                        continue

                    # /encoder 퍼블리시
                    msg = Int32()
                    msg.data = d_val
                    self.pub_encoder.publish(msg)
                    self.last_rx_time = time.time()

                    # dt_us 모니터링 (선택)
                    if len(parts) >= 3:
                        try:
                            dt_us = int(float(parts[2]))
                            self._check_arduino_dt(dt_us)
                        except ValueError:
                            pass

            except (serial.SerialException, OSError) as e:
                # 🔧 [복구 수정] 포트 자체가 죽은 하드 에러(USB 분리 등)일 때는
                #   self.ser를 None으로 떨궈서 watchdog의 'ser is None → soft 재연결'
                #   경로가 동작하게 한다. (단순 RX 침묵은 여기로 안 옴 → 기존 무리셋 정책 유지)
                self.get_logger().warn(f'시리얼 RX 오류(포트 단절 추정): {e} → 재연결 대기')
                self.ser = None
                break
            except Exception as e:
                # 종료 경합(rx_thread 가 join 타임아웃 중 blocking read 에서 막 깨어나
                # publish 하는 순간 메인스레드가 이미 rclpy.shutdown() 한 경우) 시엔
                # get_logger().warn() 도 rosout publish 라 똑같이 실패해 줄이 배로
                # 찍힌다. context 가 이미 죽었으면 로깅도 생략한다.
                if rclpy.ok():
                    self.get_logger().warn(f'RX 처리 예외: {e}')
                continue

    def _check_arduino_dt(self, dt_us):
        """아두이노 PID 주기가 기준에서 벗어나면 경고 (1Hz throttle)."""
        diff = abs(dt_us - ARDUINO_PID_PERIOD_US)
        if diff > ARDUINO_PID_TOLERANCE_US:
            now = time.time()
            if now - self.last_warn_time >= 1.0:
                self.last_warn_time = now
                self.get_logger().warn(
                    f'⚠️ Arduino PID 주기 이상: {dt_us}µs '
                    f'(기준 {ARDUINO_PID_PERIOD_US}µs '
                    f'±{ARDUINO_PID_TOLERANCE_US}µs, 편차 {diff}µs)')

    # ═══════════════════════════════════════════════════════════
    #  ROS 콜백
    # ═══════════════════════════════════════════════════════════
    def cb_cmd_vel(self, msg: Twist):
        # linear.x = m/s, angular.z = deg (driving.py 송신 규약)
        v  = max(-MAX_SPEED_MS, min(MAX_SPEED_MS, msg.linear.x))
        st = max(-MAX_STEER_DEG, min(MAX_STEER_DEG, msg.angular.z))
        self.cmd_vel_ms = v
        self.cmd_steer  = st

    def cb_state(self, msg: Bool):
        new_state = bool(msg.data)
        if new_state != self.state_enable:
            self.state_enable = new_state
            self._tx_state()

    # ═══════════════════════════════════════════════════════════
    #  TX 루프 - ROS → 아두이노
    # ═══════════════════════════════════════════════════════════
    def tx_loop(self):
        if self.ser is None:
            return

        # m/s → tick/10ms 변환
        # tick = v * DT * TICKS_PER_REV / WHEEL_CIRC
        v = self.cmd_vel_ms
        tick_f = v * ENC_DT * TICKS_PER_REV / WHEEL_CIRC
        if v > 0 and tick_f < 1.0:
            tick = 1
        elif v < 0 and tick_f > -1.0:
            tick = -1
        else:
            tick = int(round(tick_f))

        steer = int(round(self.cmd_steer))
        steer = max(-int(MAX_STEER_DEG), min(int(MAX_STEER_DEG), steer))

        line = f'C,{tick},{steer}\n'
        try:
            with self.ser_lock:
                self.ser.write(line.encode('ascii'))
                self.last_tx_ok_time = time.time()
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f'시리얼 TX 오류: {e}')

    def _tx_state(self):
        if self.ser is None:
            return
        line = f'S,{1 if self.state_enable else 0}\n'
        try:
            with self.ser_lock:
                self.ser.write(line.encode('ascii'))
            self.last_tx_ok_time = time.time()
            self.get_logger().info(
                f'📡 아두이노 State: S,{1 if self.state_enable else 0}  '
                f'cmd={self.cmd_vel_ms:.2f}m/s')
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f'State TX 오류: {e}')

    # ═══════════════════════════════════════════════════════════
    #  Watchdog - 통신 단절 감지 & 자동 재연결
    # ═══════════════════════════════════════════════════════════
    def watchdog_check(self):
        """
        RX watchdog.

        기존 문제:
          - Arduino가 E,d_val을 보내지 않거나 Python이 E 라인을 못 받으면
            last_rx_time이 갱신되지 않는다.
          - 기존 코드는 silence > 3초가 되면 _connect_serial()을 호출했다.
          - BOOT_GRACE_PERIOD 8초 뒤 첫 watchdog에서 silence 약 8.8초가 되어
            DTR 리셋 → Arduino 재부팅 → 차량 순간 정지가 반복됐다.

        수정:
          - RX가 조용하다는 이유만으로 주행 중 포트를 리셋하지 않는다.
          - 3초 초과는 경고 + 현재 state 재전송만 한다.
          - 실제 포트 객체가 없거나, 매우 장시간 RX가 없고 RESET_ON_RX_TIMEOUT=True일 때만 재연결한다.
        """
        now = time.time()

        if now - self.connect_time < BOOT_GRACE_PERIOD:
            return

        if self.ser is None:
            if self.reconnect_in_progress:
                return
            self.get_logger().warn('🔄 시리얼 객체 없음 → soft 재연결 시도')
            self.reconnect_in_progress = True
            try:
                self._connect_serial(self.serial_port, reset_board=False)
            finally:
                self.reconnect_in_progress = False
            return

        silence = now - self.last_rx_time
        if silence <= WATCHDOG_TIMEOUT:
            return

        if now - self.last_watchdog_warn_time >= WATCHDOG_WARN_PERIOD:
            self.last_watchdog_warn_time = now
            self.get_logger().warn(
                f'⚠️ Arduino RX 무응답 {silence:.1f}s: 포트 리셋 안 함 '
                f'(매핑/컨트롤러 주행 중 DTR 리셋 방지)')
            self.get_logger().warn(
                '   확인 필요: Arduino가 Serial.println("E,...")를 계속 보내는지, '
                'USB 케이블/전원/공통 GND/Serial Monitor 중복 연결 여부')

            try:
                self._tx_state()
            except Exception:
                pass

        if RESET_ON_RX_TIMEOUT and silence > HARD_RECONNECT_TIMEOUT:
            if self.reconnect_in_progress:
                return
            self.get_logger().error(
                f'⚠️ Arduino RX 장시간 무응답 {silence:.1f}s → soft 재연결 시도')
            self.reconnect_in_progress = True
            try:
                self._connect_serial(self.serial_port, reset_board=False)
            finally:
                self.reconnect_in_progress = False

    # ═══════════════════════════════════════════════════════════
    #  종료 처리
    # ═══════════════════════════════════════════════════════════
    def destroy_node(self):
        try:
            # 안전 정지 명령 전송
            if self.ser is not None:
                with self.ser_lock:
                    self.ser.write(b'C,0,0\n')
                    self.ser.write(b'S,0\n')
                    time.sleep(0.05)
        except Exception:
            pass
        self._close_serial()
        super().destroy_node()


# ═══════════════════════════════════════════════════════════════
#  main
# ═══════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
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