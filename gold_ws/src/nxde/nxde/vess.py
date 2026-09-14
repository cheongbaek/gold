#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""vess.py ― 가상 배기음(VESS) [nxde]
════════════════════════════════════════════════════════════════════════════════
따로 `ros2 run` 하는 노드가 아니다. `arduino.py` 와 나란히 두고, 그 파일이
rclpy.init() 을 부른 뒤 이 모듈을 ★import 하는 것만으로★ 켜진다:

    rclpy.init(args=args)
    ...
    import nxde.vess          # noqa: F401  ← 이 한 줄이 전부다

import 되는 순간 이 모듈이 ★자기 자신의 rclpy 노드★ 를 만들어 백그라운드
스레드에서 돌린다 — arduino 노드의 실행기에 얹혀살지 않는다. arduino.py 가
따로 호출해 줄 것이 없고, arduino 쪽 코드는 이 파일의 존재를 몰라도 된다
(그래서 "import vess" 한 줄로 충분하다는 요구가 성립한다).

★무엇을 내나★ — A보드가 발행하는 ★실제 바퀴 속도★ (`/encoder`, 좌+우 펄스의
합)를 구독해 그 크기만큼 배기음을 낸다. 가속페달(`/throttle_pedal`)이나 지령
펄스(`/drive_pulse_cmd`)가 아니라 ★엔코더★ 를 보는 이유는, 이 소리가 사람에게
'차가 실제로 얼마나 빠른가' 를 전달해야 하는 안전 신호이기 때문이다 — 페달을
밟았는데 바퀴가 아직 안 도는 구간(A보드 기동 블랭킹)에서 소리부터 커지면
그것이 더 위험한 오정보다.

`/encoder` 는 ★좌+우 합★ 이다(CLAUDE.md 2절) — 바퀴 하나 기준으로 되돌리려면
0.5 를 곱한다(`ENC_SUM_TO_PULSE`, driving.py 의 같은 상수와 이름을 맞췄다).
자율주행은 15 펄스를 넘길 일이 없고, 수동조종이 20 을 넘겨도 ★그 위로는 음이
더 올라가지 않는다★ — exhaust.py 의 `MAX_PULSE=20` 클램프를 그대로 물려받는다.

★E-STOP 중에는 재생되지 않는다★ (요구사항 그대로). 펄스를 0 으로 낮추는
것만으로는 부족하다 — 그러면 idle(공회전) 음이 계속 난다. `/estop` 이 True 인
동안은 렌더 블록 자체를 무음으로 덮어써 ★정말 아무 소리도 내지 않는다★
(`_MutableExhaust.muted`). 다른 안내 음원(nxde/sound.py 가 mp3/wav 를 재생기
프로세스로 트는 것)과는 완전히 다른 오디오 스트림이라 서로 겹쳐 들린다 —
요구사항의 "다른 음원과 중복 재생된다"가 코드를 더하지 않아도 이미 성립한다.

★실패해도 주행에 영향이 없다★ — 필요 라이브러리(sounddevice/soundfile) 가
없거나 오디오 장치가 없는 기계에서도 arduino 노드는 정상으로 뜬다. 이 파일의
모든 초기화가 예외를 삼키고 경고만 남긴다(sound.py 의 "재생 실패로 노드를
죽이지 않는다" 와 같은 태도).
"""

import os
import sys
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Int32

from nxde import exhaust as _exhaust
from nxde.soundutil import sound_dir

# /encoder(좌+우 펄스의 합) → 바퀴 하나 기준 펄스. driving.py 의 ENC_SUM_TO_PULSE
# 와 같은 값·같은 이름이다(CLAUDE.md 2절 "/encoder 는 좌+우 펄스의 합이다").
ENC_SUM_TO_PULSE = 0.5

# 이 위로는 더 올라가지 않는다 — exhaust.py 의 펄스 상한과 같다(20).
MAX_PULSE = _exhaust.MAX_PULSE


def _log(msg):
    print(f"[vess] {msg}", file=sys.stderr, flush=True)


class _MutableExhaust(_exhaust.VirtualExhaust):
    """E-STOP 동안 ★정말 무음★ 으로 만들기 위한 얇은 확장.

    펄스를 0 으로 낮추기만 하면 idle(공회전) 톤이 계속 울린다 — 요구사항은
    "재생되지 않는다"이므로 렌더 결과 자체를 지운다.
    """

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.muted = False

    def render_block(self, n=_exhaust.BLOCK):
        block = super().render_block(n)
        if self.muted:
            return block * 0.0
        return block


class VessNode(Node):

    def __init__(self):
        super().__init__('vess')
        self.engine = None
        try:
            layer_dir = os.path.join(sound_dir(), 'layers')
            engine = _MutableExhaust(mode='sample', layer_dir=layer_dir)
            engine.start()
            self.engine = engine
        except Exception as e:
            _log(f"가상 배기음을 켜지 못했습니다({e}) — 소리 없이 돕니다 "
                 f"(sounddevice/soundfile 설치, 오디오 장치를 확인하십시오)")

        self._estop = False
        self.create_subscription(Int32, '/encoder', self.cb_encoder, 10)
        self.create_subscription(Bool, '/estop', self.cb_estop, 10)

    def cb_encoder(self, msg: Int32):
        if self.engine is None:
            return
        pulse = min(abs(float(msg.data)) * ENC_SUM_TO_PULSE, MAX_PULSE)
        self.engine.set_pulse(pulse)

    def cb_estop(self, msg: Bool):
        self._estop = bool(msg.data)
        if self.engine is not None:
            self.engine.muted = self._estop

    def destroy_node(self):
        if self.engine is not None:
            try:
                self.engine.stop()
            except Exception:
                pass
        super().destroy_node()


def _run():
    try:
        node = VessNode()
    except Exception as e:
        _log(f"초기화 실패 — 가상 배기음 없이 돕니다: {e}")
        return
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass


_thread = threading.Thread(target=_run, daemon=True, name='vess')
_thread.start()
