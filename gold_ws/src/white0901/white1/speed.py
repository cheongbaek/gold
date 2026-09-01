#!/usr/bin/env python3
"""speed.py — IMU 적분 속도계. /imu 만 보고 /speed [km/h] 를 낸다.

════════════════════════════════════════════════════════════════════════════════
 왜 만들었나 — 엔코더도 GPS 도 저속에서 못 쓴다
════════════════════════════════════════════════════════════════════════════════
  · 엔코더(/encoder) : A보드 기동 블랭킹 구간의 ★허수 홀 카운트★ 때문에 저속에서
    거짓말을 한다. rec_20260811_214852 t=32 에서 차는 GPS 상 완전히 서 있는데
    엔코더는 2.5~5펄스를 뱉었다(kasa_0804_A.ino 헤더 [0730-2]).
  · GPS(/fix) : 5Hz 다. 위치 차분으로 속도를 만들면 ★최소 0.2초 지연★ 이고,
    저속에서는 RTK 노이즈(2cm)가 그대로 속도 노이즈가 된다(0.02/0.2 = 0.1 m/s).
  · IMU(/imu)  : 20Hz 이고 가속도는 ★속도의 미분★ 이라 반응이 즉각적이다.
    대신 적분 드리프트가 있으므로 아래 ZUPT 로 잡는다.

════════════════════════════════════════════════════════════════════════════════
 ★★ 축 하나를 그냥 적분하면 안 된다 — 실측으로 확인한 것 ★★
════════════════════════════════════════════════════════════════════════════════
  rec_20260811_214852 의 정지구간 가속도 평균은 ★(−5.21, −1.05, +8.25) m/s²★ 다.
  크기는 정확히 9.81 — 즉 ★가속도계는 중력을 포함하고, 센서가 기울어 장착돼 있다★
  (x축이 수평에서 32° 기울어 중력이 x 로 −5.21 m/s² 샌다).

  그래서 '정지 시 평균을 빼고 그 축을 적분'하면 차의 자세가 조금만 변해도 중력이
  다시 새어 들어온다. 실제로 그렇게 해 보면 26초 적분 결과가

      x축 −1.86 m/s   y축 +1.83 m/s   z축 −1.10 m/s      (GPS 실측 +3.5 m/s)

  로 부호조차 안 맞는다. ★반드시 자세로 중력을 지워야 한다.★

  ┌ 쿼터니언으로 월드좌표 변환 후 ─────────────────────────────────────────────┐
  │  정지구간 월드 가속도 평균 = (0.068, 0.027, 9.810)                         │
  │    → 중력이 월드 Z 에 정확히 정렬된다. iahrs 의 자세가 그만큼 쓸 만하다.   │
  │  같은 26초 적분 결과 = 2.59 m/s (GPS 실측 3.5) → 드리프트 ≈ 0.035 m/s²     │
  └────────────────────────────────────────────────────────────────────────────┘
════════════════════════════════════════════════════════════════════════════════
 ★★ 반드시 알고 쓸 것 — 절대속도는 못 믿는다 (2026-08-12 실측) ★★
════════════════════════════════════════════════════════════════════════════════
  rec_20260811_214852 을 이 알고리즘에 그대로 흘려 GPS 와 대조했다.

    시각      이 노드    GPS 실측
    t=40      0.75      1.18  km/h
    t=50      1.68      4.16
    t=56      3.11      8.71
    t=62      8.13     17.37        ← ★절반 수준으로 과소평가★

  원인은 적분 자체가 아니라 ★AHRS 자세가 지속 가속에 오염되는 것★ 이다. 6축 AHRS 는
  중력 방향으로 기울기를 잡는데, 차가 계속 가속하면 그 가속을 기울기로 오인한다.
  실제로 피치가 정지 32.49° → 주행 30.10° 로 ★2.4° 흔들렸고★, 이는 중력 누출
  0.41 m/s² 로 이 주행의 평균 가속도 0.185 m/s² 보다 크다. 즉 자세가 우리가 재려는
  신호를 스스로 지운다.

  ┌ 자세를 자이로로만 전파해 봤다(스트랩다운, 정지 중에만 AHRS 신뢰) ───────────┐
  │   t=56  14.66 / 8.71      t=62  26.83 / 17.37   → 이번엔 ★1.6배 과대평가★    │
  │   자이로 드리프트가 반대 방향으로 중력을 새게 한다. 더 낫지 않다.           │
  └────────────────────────────────────────────────────────────────────────────┘

  ★결론★ 이 하드웨어에서 순수 IMU 적분의 절대속도는 26초 창에서 2배 수준으로 틀린다.
  대신 ★'서 있는가 움직이는가'는 아주 정확하다★ (아래 ZUPT 표 — 15배 분리). 그리고
  그것이 엔코더가 못 하는 일이다(허수 카운트로 정지 중에 2.5~5펄스를 뱉는다).
  그러니 이 토픽은 이렇게 쓰는 것이 맞다:

    ○ 정지/기동 판정, 짧은 구간의 상대 변화
    △ 저속(≲10 km/h) 절대속도 — 경향은 맞지만 크기는 낮게 나온다
    ✗ 고속 절대속도 — 쓰지 말 것

  driving.py 의 저속 펄스 보정이 이 값을 쓰되 ★보정량을 ±2펄스로 묶고 REF 3 이하
  에서만 동작시키는★ 이유가 이것이다. 과소평가는 '더 밟는' 쪽으로 틀리므로 상한이
  반드시 필요하다.

════════════════════════════════════════════════════════════════════════════════
 ★★ ZUPT — 드리프트를 잡는 유일한 수단, 그리고 그 한계 ★★
════════════════════════════════════════════════════════════════════════════════
  적분값은 반드시 흐른다. 유일한 해법은 '지금 서 있다'를 알아채고 0 으로 되돌리는
  것(Zero-velocity UPdaTe)이다. 문제는 ★순수 IMU 로는 '정지'와 '등속'을 원리적으로
  구별할 수 없다★ 는 것이다 — 둘 다 가속도 0, 각속도 0 이다.

  이 차에서는 ★노면 진동★ 이 그 구별을 해 준다. 같은 로그에서 |a| 의 0.5초 이동
  표준편차가

      완전정지        σ = 0.064 m/s²   |gyro_z| = 0.004 rad/s
      0.05~0.3 m/s    σ = 0.279        0.022      ← 기어가는 수준인데 이미 4배
      0.4~0.8 m/s     σ = 0.937        0.111
      2.7~4.8 m/s     σ = 1.142        0.082

  로 ★15배 벌어진다★. 그래서 임계를 0.15 로 두면 '기어가는 중'을 정지로 오판하지
  않으면서 정지를 확실히 잡는다. 아스팔트가 아주 매끄럽거나 차를 들어 올린 채로
  돌리면 이 가정이 깨진다 — 그때는 /speed 가 0 으로 굳는다(안전한 방향이다).

════════════════════════════════════════════════════════════════════════════════
 발행
════════════════════════════════════════════════════════════════════════════════
  /speed  (std_msgs/Float32)  ★km/h★ — 항상 0 이상(크기). IMU 가 올 때마다(20Hz).
    ※ IMU 가 끊기면 이 토픽도 함께 멈춘다. ★받는 쪽이 신선도를 확인해야 한다★
      (driving.py 의 measured_pulse() 가 그렇게 한다). 여기서 0 을 대신 쏘지 않는
      이유는, 끊긴 것과 '진짜 0' 을 받는 쪽이 구별할 수 있어야 하기 때문이다.

  1펄스 ≈ 3.182 km/h 이므로 펄스로 환산해 쓰려면 그 값으로 나눈다
  (driving.py 의 KMH_PER_PULSE — 환산상수의 소유자는 driving.py 다).
"""

import math
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32


# ══════════════════════════════════════════════════════════════════════════════
#  튜닝 상수 — 근거는 전부 파일 헤더의 실측표에 있다
# ══════════════════════════════════════════════════════════════════════════════
GRAVITY = 9.80665

#  ── ZUPT(정지 판정) ──
ZUPT_WINDOW_S   = 0.5    # [s] 이 창의 |a| 표준편차를 본다 (20Hz → 10샘플)
ZUPT_ACC_STD    = 0.15   # [m/s²] 이 밑이면 '흔들리지 않는다' (정지 0.064 / 기어감 0.279)
ZUPT_GYRO_RPS   = 0.05   # [rad/s] 이 밑이면 '돌지 않는다' (≈2.9 deg/s)
ZUPT_HOLD_S     = 0.3    # [s] 두 조건이 이만큼 계속 성립해야 정지로 확정한다

#  ── 바이어스(=중력+가속도계 옵셋) 추정 ──
#    정지로 확정된 동안에만 갱신한다. 움직이는 중에 갱신하면 가속을 바이어스로
#    빨아들여 ★속도가 0 으로 수렴하는★ 최악의 고장이 난다.
BIAS_ALPHA      = 0.02   # 정지 중 월드가속을 바이어스로 끌어당기는 비율(1틱당)
BIAS_MIN_SAMPLES = 20    # 첫 바이어스를 확정하기까지 필요한 정지 표본 수(1초)

#  ── 드리프트 누출 ──
#    ZUPT 가 오래 안 걸리는 장거리 등속 구간에서 적분오차가 무한히 쌓이는 것을 막는다.
#    ★시상수를 길게 잡는다★ — 짧으면 등속 주행 중 속도가 슬금슬금 0 으로 빨려간다.
LEAK_TAU_S      = 30.0   # [s]

#  ── 입력 위생 ──
DT_MAX_S        = 0.25   # 이보다 긴 공백은 적분하지 않는다(IMU 끊김 뒤 첫 샘플)
QUAT_MIN_NORM   = 0.5    # 쿼터니언이 0 벡터로 오는 프레임은 버린다

MS_TO_KMH       = 3.6


class SpeedNode(Node):
    def __init__(self):
        super().__init__('speed')

        self.zupt_window_s = float(
            self.declare_parameter('zupt_window_s', ZUPT_WINDOW_S).value)
        self.zupt_acc_std = float(
            self.declare_parameter('zupt_acc_std', ZUPT_ACC_STD).value)
        self.zupt_gyro_rps = float(
            self.declare_parameter('zupt_gyro_rps', ZUPT_GYRO_RPS).value)
        self.leak_tau_s = float(
            self.declare_parameter('leak_tau_s', LEAK_TAU_S).value)

        # ── 적분 상태 ──
        self.vx = 0.0                # [m/s] 월드 동
        self.vy = 0.0                # [m/s] 월드 북
        self.bias = None             # [m/s²] 월드좌표 정지 시 가속도(중력+옵셋)
        self._bias_n = 0
        self._last_t = None

        # ── 정지 판정 ──
        self._mag_buf = []           # (t, |a|) — ZUPT_WINDOW_S 만큼만 들고 있는다
        self._still_since = None
        self.stationary = True       # 시작은 '서 있다'로 본다(부팅 시 실제로 그렇다)

        self.pub = self.create_publisher(Float32, '/speed', 10)
        self.create_subscription(Imu, '/imu', self.cb_imu, 20)

        self.get_logger().info(
            "speed 준비 — /imu 를 적분해 /speed[km/h] 를 낸다. "
            f"ZUPT σ<{self.zupt_acc_std} m/s², |gyro|<{self.zupt_gyro_rps} rad/s")

    # ══════════════════════════════════════════════════════════════════════════
    def cb_imu(self, msg: Imu):
        now = time.time()
        dt = 0.0 if self._last_t is None else now - self._last_t
        self._last_t = now

        a_body = (msg.linear_acceleration.x,
                  msg.linear_acceleration.y,
                  msg.linear_acceleration.z)
        q = (msg.orientation.x, msg.orientation.y,
             msg.orientation.z, msg.orientation.w)
        gyro = (msg.angular_velocity.x, msg.angular_velocity.y,
                msg.angular_velocity.z)

        a_world = self._to_world(q, a_body)
        if a_world is None:                      # 자세를 못 믿는 프레임은 통째로 버린다
            return

        self._update_stationary(now, a_body, gyro)

        if self.stationary:
            # ★정지 확정 — 속도를 0 으로 되돌리고 바이어스를 갱신한다★
            #   이 두 가지가 이 노드의 정확도를 사실상 전부 결정한다.
            self.vx = self.vy = 0.0
            if self.bias is None:
                self.bias = list(a_world)
                self._bias_n = 1
            else:
                for i in range(3):
                    self.bias[i] += BIAS_ALPHA * (a_world[i] - self.bias[i])
                self._bias_n = min(self._bias_n + 1, BIAS_MIN_SAMPLES)
        elif self.bias is not None and self._bias_n >= BIAS_MIN_SAMPLES \
                and 0.0 < dt <= DT_MAX_S:
            # ★수평 성분만 적분한다★ 연직은 이 차에서 쓸 데가 없고, 자세오차가
            #   그대로 들어오는 축이라 넣으면 잡음만 늘어난다.
            self.vx += (a_world[0] - self.bias[0]) * dt
            self.vy += (a_world[1] - self.bias[1]) * dt
            if self.leak_tau_s > 0.0:
                leak = math.exp(-dt / self.leak_tau_s)
                self.vx *= leak
                self.vy *= leak

        speed_kmh = math.hypot(self.vx, self.vy) * MS_TO_KMH
        self.pub.publish(Float32(data=float(speed_kmh)))

    # ══════════════════════════════════════════════════════════════════════════
    @staticmethod
    def _to_world(q, v):
        """센서좌표 가속도 → 월드좌표. ★이것이 중력을 지우는 유일한 수단이다★
        (파일 헤더의 실측표 참고 — 이 변환 없이는 부호조차 맞지 않는다)."""
        x, y, z, w = q
        n = math.sqrt(x * x + y * y + z * z + w * w)
        if n < QUAT_MIN_NORM:
            return None
        x, y, z, w = x / n, y / n, z / n, w / n
        return (
            (1 - 2 * (y * y + z * z)) * v[0] + 2 * (x * y - z * w) * v[1]
            + 2 * (x * z + y * w) * v[2],
            2 * (x * y + z * w) * v[0] + (1 - 2 * (x * x + z * z)) * v[1]
            + 2 * (y * z - x * w) * v[2],
            2 * (x * z - y * w) * v[0] + 2 * (y * z + x * w) * v[1]
            + (1 - 2 * (x * x + y * y)) * v[2],
        )

    def _update_stationary(self, now, a_body, gyro):
        """|a| 의 이동 표준편차 + 각속도로 정지를 판정한다.

        ★센서좌표 |a| 를 쓴다★ 크기는 회전에 불변이므로 자세오차가 섞이지 않는다 —
        판정에 자세를 개입시킬 이유가 없다.
        """
        mag = math.sqrt(a_body[0] ** 2 + a_body[1] ** 2 + a_body[2] ** 2)
        self._mag_buf.append((now, mag))
        cut = now - self.zupt_window_s
        while self._mag_buf and self._mag_buf[0][0] < cut:
            self._mag_buf.pop(0)

        if len(self._mag_buf) < 3:
            return                                # 판단할 표본이 없으면 상태 유지
        vals = [m for _, m in self._mag_buf]
        mean = sum(vals) / len(vals)
        std = math.sqrt(sum((v - mean) ** 2 for v in vals) / len(vals))
        gyro_mag = math.sqrt(gyro[0] ** 2 + gyro[1] ** 2 + gyro[2] ** 2)

        quiet = (std < self.zupt_acc_std) and (gyro_mag < self.zupt_gyro_rps)
        if not quiet:
            self._still_since = None
            self.stationary = False
            return
        # ★조용하다고 곧바로 정지로 보지 않는다★ 순간적인 등속 구간을 정지로 읽으면
        #   달리는 중에 속도가 0 으로 리셋된다. ZUPT_HOLD_S 만큼 이어져야 확정한다.
        if self._still_since is None:
            self._still_since = now
        elif now - self._still_since >= ZUPT_HOLD_S:
            self.stationary = True


def main(args=None):
    rclpy.init(args=args)
    node = SpeedNode()
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
