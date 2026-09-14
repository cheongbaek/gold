"""가상 배기음 - 포르쉐 타이칸 스타일 전기 구동음 (펄스 0~20 리니어).

설계
----
펄스(0~20) -> 스무딩 -> 기본주파수 f0 (리니어 상승)
  -> 주행 레이어를 "같은 f0" 로 변조하고, 음량만 펄스에 따라 크로스페이드

  idle : 정지 공회전음.  음정 고정(F_STANDSTILL). 펄스 1 까지 온전, 3 에서 소멸
  high : 주행음.         펄스 0->2 에서 차올라 공회전음을 덮고, 이후 20 까지 단독.
                         피치가 그대로 속도감이 된다

예전에는 고속에서 바람 노이즈(air) 베드를 깔았으나 걷어냈다. 음원 자체는
2kHz 위로 전력이 0.5% 도 안 되게 깨끗한데, 합성 화이트노이즈를 얹으니 펄스
20 에서 RMS 를 +6.9dB 올리고 음역 중심을 712Hz -> 7538Hz 로 끌어올려 소리가
통째로 쉬익거렸다. 소스에 들어있던 게 아니라 우리가 만들어 넣던 것이라
필터링이 아니라 생성을 안 하는 것으로 해결했다.

공회전음만 음정을 고정한 이유: 정지 상태에서 음이 같이 올라가면 "엑셀 밟는
내연기관" 처럼 들린다. 현대/기아 VESS 는 정지 중 일정한 톤을 유지하다가 차가
움직이면 주행음이 커지며 덮는 방식이다. 여기서도 low 가 커지면서 가린다.

처음에는 저속음(low)을 따로 두어 3 레이어로 갔으나, 주행음 하나로 전 구간을
덮는 편이 낫다고 판단해 걷어냈다. 레이어가 둘이면 음원이 다른 두 소리가 섞이는
구간 자체가 없어져서, 기본주파수가 다른 음원끼리 부딪혀 생기는 맥놀이(beating)
걱정이 사라진다. 대신 주행음 하나가 펄스 0~20 을 다 감당하느라 배속 폭이
0.38~1.56 배로 넓어진다. 아래쪽은 공회전음이 덮는 구간이라 감당할 만하다.
공회전음은 주행음 배음열과 겹치지 않도록 비정수 차수로 구성해 두었다.
(layers/low.wav 와 LOW_PARTIALS 는 되돌릴 때를 위해 지우지 않고 남겨 두었다)

모드
----
sample : layers/{idle,high}.wav 루프를 읽어 쓴다 (기본)
synth  : 내장 가산합성. 에셋이 없을 때의 대체 경로이자 기준 음원 생성용.
         layers/ 에 wav 가 하나라도 없으면 자동으로 이쪽으로 떨어진다

실행
----
    python exhaust.py                    # 키보드 조종: W/S, 0~9, Q
    python exhaust.py --sweep            # 0 -> 20 -> 0 자동 스윕 시청
    python exhaust.py --mode synth       # 내장 합성음으로 비교해 듣기
    python exhaust.py --render out       # 펄스별 wav 21개 뽑기 (임베디드용)

다른 코드에서 쓸 때
-------------------
    eng = VirtualExhaust()
    eng.start()
    eng.set_pulse(7)      # 아무 스레드에서나 호출 가능
    ...
    eng.stop()

★[2026-09-14] nxde.vess 가 이 엔진을 실차 /encoder 에 물린다★ (nxde/vess.py).
이 파일 자체는 여전히 독립 실행 가능한 튜닝 도구다 — 위 실행 예처럼 키보드로
직접 들어 보거나(`python3 exhaust.py`), `--render` 로 임베디드용 wav 를 뽑을 때
쓴다. LAYER_DIR 은 이제 이 파일 옆이 아니라 white1/sound/layers 를 가리킨다
(아래 정의 참고) — 그 폴더가 음원의 단일 소유자다.
"""

import argparse
import os
import sys
import time

import numpy as np
from scipy.io import wavfile
from scipy.signal import lfilter

# ---------------------------------------------------------------- 튜닝 상수
SR = 44100
BLOCK = 256           # 약 5.8ms - 조작 반응이 즉각적으로 느껴지는 크기
MAX_PULSE = 20

F_IDLE = 46.0         # 펄스 0  일 때 기본주파수 (Hz)
F_TOP = 225.0         # 펄스 20 일 때 기본주파수 (Hz)  -> 약 2.3 옥타브 상승

# 공회전음만은 펄스를 따라가지 않고 이 주파수에 고정된다. 현대/기아 VESS 처럼
# 정지 중엔 음정이 일정하게 유지되다가, 펄스가 오르면 low 레이어가 커지면서
# 자연스럽게 덮인다. (음정이 같이 올라가면 "엑셀 밟는 내연기관" 처럼 들린다)
F_STANDSTILL = 46.0
SMOOTH_TAU = 0.28     # 펄스 계단을 녹이는 시정수(초). 이게 없으면 21단 계단음
MASTER_GAIN = 0.55

# 2단계 경계 (펄스 단위)
P_IDLE_HOLD = 1.0     # 여기까지는 공회전음을 100% 유지
P_IDLE_OUT = 3.0      # 공회전음이 완전히 사라지는 펄스
P_DRIVE_IN = 2.0      # 주행음이 최대에 도달하는 펄스 (공회전음을 덮는 주체)

# (배음차수, 진폭) - 차수가 정수가 아니면 금속성/이질적인 울림이 난다
# 공회전은 "지속되는 부팅음" 성격 - 장3화음(1 : 1.25 : 1.5)을 길게 끄는 소리.
# 차수를 2 배부터 시작해 근음을 92Hz 에 두었다. low 의 기본주파수(46~64Hz)와
# 한 옥타브 안에 들어와 음역이 이어진다. (예전엔 428Hz 라 혼자 붕 떠 있었다)
# f0=46Hz 기준 -> 92 / 115 / 138 / 184 / 230 / 276 / 368 / 460 Hz
IDLE_PARTIALS = ((2, .30), (2.5, .26), (3, .28), (4, .22),
                 (5, .16), (6, .12), (8, .08), (10, .06))
# [현재 미사용] 저속 레이어를 걷어내고 주행음 하나로 전 구간을 덮는 구조로
# 바꾸면서 빠졌다. 3 레이어로 되돌릴 때를 위해 남겨 둔다. 살짝 어긋난 짝
# 파셜(1.004 등)로 느린 맥놀이를, 늘린 배음(2.002, 3.008 …)으로 물리적인
# 몸통을 만든 구성이다. 짝 진폭이 본체와 비슷하면 포락선이 104% 출렁여서
# 20% 로 눌렀다.
LOW_PARTIALS = ((1, .48), (1.004, .10),
                (2.002, .36), (2.010, .07),
                (3.008, .25), (3.020, .05),
                (4.018, .16), (4.035, .03),
                (5.03, .09), (6.05, .05))
HIGH_PARTIALS = ((1, .30), (2, .26), (3, .18), (6, .14),
                 (8, .28), (8.04, .20), (10.73, .14),
                 (12, .24), (12.06, .16), (16, .12))
# 8/8.04, 12/12.06 처럼 살짝 어긋난 쌍 = 코러스. 소리에 폭과 생기를 준다
# 10.73 처럼 정수가 아닌 차수 = 금속성. 타이칸 특유의 "우주선" 질감

# 주행음(high) 하나가 펄스 0~20 전체를 덮고, 공회전음은 정지 상태만 맡는다.
TONE_LAYERS = ("idle", "high")
PARTIAL_SETS = (IDLE_PARTIALS, HIGH_PARTIALS)

# sample 모드에서 high.wav 가 "몇 번 펄스에서 배속 1.0 으로 울릴지".
# 레이어 하나가 펄스 0~20 을 다 덮으므로 배속 폭이 4.1 배로 넓을 수밖에 없다.
# 11 은 그 폭을 위쪽에 유리하게 배분한 값이다 - 펄스 20 에서 1.56 배라 마음에
# 드는 고속음이 거의 원본 그대로 울리고, 아래쪽 0.38 배는 공회전음이 덮는
# 구간이라 손해가 적다. 8 로 내리면 위가 1.91 배까지 늘어나 얇아진다.
DEFAULT_REFS = {"high": 11}

# ★[2026-09-14] white1/sound/layers 를 가리킨다★ 음원의 단일 소유자는 white1
# (paths.sound_dir()) 이다 — nxde/sound.py 의 sound_dir() 이 이미 그쪽을 빌려 오는
# 것과 같은 이유다(자기 패키지 안에 layers/ 를 따로 두지 않는다). 그 조회가
# 실패하면(단독 실행·white1 미빌드 등) 이 파일 옆의 layers/ 로 폴백한다 — 예전
# 자리에 손으로 넣어 둔 wav 가 있어도 여전히 동작하게 하려는 것이다.
try:
    from nxde.soundutil import sound_dir as _sound_dir
    LAYER_DIR = os.path.join(_sound_dir(), "layers")
except Exception:
    LAYER_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "layers")

LPF_LOW, LPF_HIGH = 1500.0, 6000.0    # 마스터 저역통과 컷오프 (속도에 따라)
# LPF_LOW 를 더 낮추면 공회전의 고역 파셜까지 깎여 작은 스피커에서 안 들린다


def pulse_to_f0(pulse):
    return F_IDLE + (F_TOP - F_IDLE) * (np.clip(pulse, 0, MAX_PULSE) / MAX_PULSE)


def smoothstep(x, a, b):
    """a -> b 구간에서 0 -> 1 로 부드럽게 넘어간다."""
    t = np.clip((x - a) / (b - a), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def layer_gains(p):
    """펄스 p(0~20) -> 레이어별 음량. p 는 스칼라/배열 모두 가능.

    레이어가 둘뿐이라 크로스페이드가 한 번으로 끝난다. 공회전음은 음정이
    고정이므로 "작아져서 묻히는" 것이지 "음이 올라가며 변하는" 게 아니다.
    """
    return {
        "idle": 1.0 - smoothstep(p, P_IDLE_HOLD, P_IDLE_OUT),
        # 펄스 0 에서 0 -> 2 에서 최대. 이게 커지면서 공회전음을 덮는다.
        "high": smoothstep(p, 0.0, P_DRIVE_IN),
    }


def layers_ready(dirpath):
    """레이어 wav 가 모두 있는지. 하나라도 없으면 sample 모드를 못 쓴다."""
    return all(os.path.exists(os.path.join(dirpath, f"{n}.wav"))
               for n in TONE_LAYERS)


def one_pole_lp(cutoff):
    """1차 저역통과 계수. y[n] = a*x[n] + (1-a)*y[n-1]"""
    a = 1.0 - np.exp(-2.0 * np.pi * cutoff / SR)
    return np.array([a]), np.array([1.0, a - 1.0])


# ---------------------------------------------------------------- 음원 소스
class SynthSource:
    """가산합성: 파셜마다 독립 위상 누산기를 둔다.

    비정수 차수(10.73 등)는 공용 위상을 1.0 으로 랩하면 딸깍 소리가 나므로
    반드시 파셜별로 위상을 들고 있어야 한다.
    """

    def __init__(self):
        ks, amps = [], []
        self.slices = {}
        for name, parts in zip(TONE_LAYERS, PARTIAL_SETS):
            start = len(ks)
            ks.extend(p[0] for p in parts)
            amps.extend(p[1] for p in parts)
            self.slices[name] = slice(start, len(ks))
        self.k = np.array(ks, dtype=np.float64)
        self.amp = np.array(amps, dtype=np.float64)
        self.phase = np.random.default_rng(7).random(len(ks))
        self.is_idle = np.zeros(len(ks), dtype=bool)
        self.is_idle[self.slices["idle"]] = True

    def render(self, f0, f0_idle, gains):
        # idle 만 고정 주파수를 쓴다 (펄스를 따라가지 않는다)
        base = np.where(self.is_idle[:, None], f0_idle, f0[None, :])
        inc = self.k[:, None] * base / SR            # (K, n) 파셜별 위상증가(cycle)
        ph = self.phase[:, None] + np.cumsum(inc, axis=1)
        self.phase = np.mod(ph[:, -1], 1.0)

        wave = np.sin(2.0 * np.pi * ph)
        wave *= (inc < 0.45)                         # 나이퀴스트 근처 파셜은 죽인다
        wave *= self.amp[:, None]

        out = np.zeros_like(f0)
        for name in TONE_LAYERS:
            out += wave[self.slices[name]].sum(axis=0) * gains[name]
        return out


class SampleSource:
    """layers/{idle,high}.wav 를 읽어 재생한다.

    idle.wav 는 배속 1.0 으로 고정 재생한다(음정이 변하지 않는다). 그래서
    기준 펄스가 필요 없고, 만든 그대로의 음높이가 그대로 공회전음이 된다.

    high.wav 는 기준 펄스에서 배속 1.0 이 되도록 리샘플한다. 기준을 틀리게
    주면 배속이 극단으로 간다 - 실제 f0 가 65.3Hz 인 파일에 기준 2 를 주면
    펄스 20 에서 3.5 배가 되어 뭉개진다. loopify.py 가 파일마다 알맞은 값을
    찍어 주지만, 레이어 하나로 전 구간을 덮는 지금 구조에서는 위아래 배속을
    어떻게 배분할지가 더 중요하다 (DEFAULT_REFS 주석 참고).
    """

    def __init__(self, dirpath, refs):
        import soundfile as sf

        self.buf, self.pos, self.f_ref = {}, {}, {}
        for name in TONE_LAYERS:
            path = os.path.join(dirpath, f"{name}.wav")
            if not os.path.exists(path):
                sys.exit(f"[오류] {path} 가 없습니다. "
                         f"loopify.py 로 먼저 루프를 만드세요.")
            data, sr = sf.read(path, dtype="float64", always_2d=True)
            data = data.mean(axis=1)
            if sr != SR:                              # 간단 선형 리샘플
                n = int(len(data) * SR / sr)
                data = np.interp(np.linspace(0, len(data), n, endpoint=False),
                                 np.arange(len(data)), data)
            peak = np.max(np.abs(data))
            self.buf[name] = data / peak if peak > 0 else data
            self.pos[name] = 0.0
            if name == "idle":
                self.f_ref[name] = None               # 배속 1.0 고정
                print(f"  {name:5} {len(data) / SR:5.2f}s  음정 고정")
            else:
                self.f_ref[name] = pulse_to_f0(refs[name])
                print(f"  {name:5} {len(data) / SR:5.2f}s  기준펄스 {refs[name]:2d} "
                      f"(f0={self.f_ref[name]:.1f}Hz)")

    def render(self, f0, f0_idle, gains):
        out = np.zeros_like(f0)
        for name in TONE_LAYERS:
            buf = self.buf[name]
            # idle.wav 는 녹음된 그대로(배속 1.0) 재생한다 -> 음정 고정
            rate = np.ones_like(f0) if name == "idle" else f0 / self.f_ref[name]
            idx = self.pos[name] + np.cumsum(rate)
            self.pos[name] = idx[-1] % len(buf)
            idx = idx % len(buf)
            i0 = idx.astype(np.int64)
            frac = idx - i0
            i1 = (i0 + 1) % len(buf)
            out += (buf[i0] * (1.0 - frac) + buf[i1] * frac) * gains[name]
        return out


# ---------------------------------------------------------------- 엔진 본체
class VirtualExhaust:
    def __init__(self, mode="sample", layer_dir=None, refs=None):
        layer_dir = layer_dir or LAYER_DIR
        if mode == "sample" and not layers_ready(layer_dir):
            print(f"[알림] {layer_dir} 에 wav 가 없어 내장 합성음으로 돌립니다. "
                  f"(loopify.py 로 루프를 만들면 그쪽을 씁니다)")
            mode = "synth"
        if mode == "sample":
            self.src = SampleSource(layer_dir, refs or DEFAULT_REFS)
        else:
            self.src = SynthSource()

        self._target = 0.0          # 외부에서 설정하는 펄스 (0~20)
        self._p = 0.0               # 스무딩된 펄스
        self._alpha = 1.0 - np.exp(-(BLOCK / SR) / SMOOTH_TAU)
        self._lfo = 0.0
        self._zi_master = np.zeros(1)
        self._stream = None

    # ---- 제어 -----------------------------------------------------------
    def set_pulse(self, pulse):
        """모터 펄스 0~20 을 넣는다. 아무 스레드에서나 호출해도 된다."""
        self._target = float(np.clip(pulse, 0, MAX_PULSE))

    @property
    def pulse(self):
        return self._target

    # ---- 렌더 -----------------------------------------------------------
    def render_block(self, n=BLOCK):
        # 펄스는 21단 계단이므로 반드시 녹여서 써야 한다. 이게 없으면
        # 가속할 때 피치가 "뚝뚝" 끊겨 올라가 즉시 가짜 티가 난다.
        p0 = self._p
        self._p += (self._target - self._p) * self._alpha
        p = np.linspace(p0, self._p, n, endpoint=False)   # 블록 안에서도 선형 보간

        f0 = pulse_to_f0(p)
        gains = layer_gains(p)
        s = self._p / MAX_PULSE

        tone = self.src.render(f0, F_STANDSTILL, gains)

        # 공회전은 살짝 흔들려야 "살아있는" 느낌이 난다
        lfo_ph = self._lfo + np.arange(n) * (6.5 / SR)
        self._lfo = (lfo_ph[-1] + 6.5 / SR) % 1.0
        tone = tone * (1.0 + 0.10 * gains["idle"] * np.sin(2.0 * np.pi * lfo_ph))

        # 마스터 저역통과: 속도가 오를수록 열린다 (가속감의 절반은 여기서 나온다)
        b, a = one_pole_lp(LPF_LOW + (LPF_HIGH - LPF_LOW) * s)
        tone, self._zi_master = lfilter(b, a, tone, zi=self._zi_master)

        out = tone[:, None] * 0.5
        return np.tanh(out * 1.4) * MASTER_GAIN               # 부드러운 클립

    # ---- 재생 -----------------------------------------------------------
    def start(self):
        import sounddevice as sd

        def callback(outdata, frames, t, status):
            outdata[:] = self.render_block(frames)

        self._stream = sd.OutputStream(samplerate=SR, channels=2,
                                       blocksize=BLOCK, dtype="float32",
                                       callback=callback)
        self._stream.start()

    def stop(self):
        if self._stream is not None:
            self._stream.stop()
            self._stream.close()
            self._stream = None


# ---------------------------------------------------------------- 실행 모드
def _make_getch():
    """콘솔에서 한 글자를 논블로킹으로 읽는 (poll, restore) 한 쌍을 만든다.

    ★[2026-09-15] msvcrt 는 Windows 전용이다★ 이 프로젝트는 Ubuntu 22.04
    전용이라(CLAUDE.md) 원래 코드(외부에서 그대로 들여온 것)가 `python3
    exhaust.py` 단독 실행에서 곧바로 ModuleNotFoundError 로 죽었다. POSIX
    쪽은 termios 로 캐노니컬/에코를 끄고(cbreak) select 로 논블로킹 폴링한다.
    ISIG 는 그대로 두므로 Ctrl+C(SIGINT)는 여전히 KeyboardInterrupt 로 온다
    — run_keyboard 가 그것을 받아 터미널 설정을 반드시 복원한다.
    """
    if sys.platform == 'win32':
        import msvcrt

        def poll():
            if not msvcrt.kbhit():
                return ''
            ch = msvcrt.getch()
            if ch in (b"\x00", b"\xe0"):                  # 화살표 = 2바이트
                ch = {b"H": b"w", b"P": b"s"}.get(msvcrt.getch(), b"")
            return ch.decode(errors='ignore') if isinstance(ch, bytes) else ch

        return poll, (lambda: None)

    import select
    import termios
    import tty

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    # ★[2026-09-15 수정] 화살표(ESC [ A/B) 후속 바이트 대기 시간★ 10ms 는
    #   IDE 통합 터미널처럼 입력이 살짝 배치돼 오는 환경에서 자주 못 미친다.
    #   그러면 "화살표를 눌렀는데 단독 ESC 로 보여 프로그램이 통째로 꺼지는"
    #   사고가 난다 — 종료 키는 q/Q 뿐이어야 하므로, ★후속 바이트가 늦게 오거나
    #   아예 안 오면 그냥 무시한다★(아무 일도 하지 않는다). 여유도 0.01 → 0.05 로.
    ESC_SEQ_TIMEOUT_S = 0.05

    def poll():
        if not select.select([sys.stdin], [], [], 0)[0]:
            return ''
        ch = sys.stdin.read(1)
        if ch != '\x1b':                                  # ESC 로 시작하지 않으면 그대로
            return ch
        if not select.select([sys.stdin], [], [], ESC_SEQ_TIMEOUT_S)[0]:
            return ''                                     # 단독/지연된 ESC → 무시(종료 아님)
        if sys.stdin.read(1) != '[':
            return ''
        if not select.select([sys.stdin], [], [], ESC_SEQ_TIMEOUT_S)[0]:
            return ''
        return {'A': 'w', 'B': 's'}.get(sys.stdin.read(1), '')  # 화살표(ESC [ A/B)

    def restore():
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

    return poll, restore


def run_keyboard(eng):
    print("W/S(또는 위/아래) = 펄스 증감,  0~9 = 직접 지정(x2),  Q = 종료")
    poll, restore = _make_getch()
    # ★eng.start() 를 try 안으로★ _make_getch() 가 이미 터미널을 cbreak 로
    # 바꿔 놨다 — 여기서 실패하면(오디오 장치 없음 등) finally 의 restore() 가
    # 반드시 돌아야 터미널이 원상태(에코 켜짐)로 돌아온다. try 밖에서 부르면
    # 예외가 restore() 를 건너뛰어 ★터미널이 망가진 채로 스크립트가 죽는다★.
    try:
        eng.start()
        while True:
            ch = poll()
            if ch in ('q', 'Q', '\x03'):
                break
            if ch in ('w', 'W'):
                eng.set_pulse(eng.pulse + 1)
            elif ch in ('s', 'S'):
                eng.set_pulse(eng.pulse - 1)
            elif ch.isdigit():
                eng.set_pulse(int(ch) * 2)
            n = int(eng.pulse)
            stage = "공회전" if n == 0 else ("저속 " if n <= 4 else "고속 ")
            print(f"\r펄스 {n:2d}/{MAX_PULSE} [{'#' * n}{'.' * (MAX_PULSE - n)}] "
                  f"{stage}  f0={pulse_to_f0(eng.pulse):6.1f}Hz ", end="", flush=True)
            time.sleep(0.03)
    except KeyboardInterrupt:
        pass
    finally:
        eng.stop()
        restore()
        print()


def run_sweep(eng, up=8.0, hold=1.5, down=6.0):
    eng.start()
    print(f"자동 스윕: 0 -> {MAX_PULSE} ({up}s) -> 유지({hold}s) -> 0 ({down}s)")
    try:
        for dur, a, b in ((up, 0, MAX_PULSE), (hold, MAX_PULSE, MAX_PULSE),
                          (down, MAX_PULSE, 0)):
            t0 = time.perf_counter()
            while True:
                dt = time.perf_counter() - t0
                if dt >= dur:
                    break
                eng.set_pulse(a + (b - a) * (dt / dur))
                print(f"\r펄스 {eng.pulse:5.1f}  "
                      f"f0={pulse_to_f0(eng.pulse):6.1f}Hz ", end="", flush=True)
                time.sleep(0.02)
        time.sleep(0.4)
    finally:
        eng.stop()
        print()


def run_render(eng, outdir, seconds=2.0):
    """펄스별 정상상태 wav 21개. 루프 이음매가 맞도록 f0 주기에 길이를 맞춘다."""
    os.makedirs(outdir, exist_ok=True)
    for p in range(MAX_PULSE + 1):
        eng._target = eng._p = float(p)
        f0 = pulse_to_f0(p)
        n = int(round(round(seconds * f0) / f0 * SR))          # f0 정수 주기로 맞춤
        for _ in range(SR // 2 // BLOCK):                      # 필터 워밍업
            eng.render_block(BLOCK)
        buf = np.concatenate([eng.render_block(BLOCK)
                              for _ in range(n // BLOCK + 1)])[:n]
        path = os.path.join(outdir, f"pulse_{p:02d}.wav")
        wavfile.write(path, SR, (buf * 32767).astype(np.int16))
        print(f"{path}  {n / SR:.3f}s  f0={f0:6.1f}Hz  "
              f"peak={np.max(np.abs(buf)):.2f}")


def run_export_demo(path, eng, up=8.0, hold=1.5, down=6.0):
    """0 -> 20 -> 0 스윕을 wav 로 저장한다 (실시간 재생 없이 들어보기용)."""
    blocks = []
    for dur, a, b in ((up, 0, MAX_PULSE), (hold, MAX_PULSE, MAX_PULSE),
                      (down, MAX_PULSE, 0)):
        for i in range(int(dur * SR) // BLOCK):
            eng.set_pulse(a + (b - a) * (i * BLOCK / (dur * SR)))
            blocks.append(eng.render_block(BLOCK))
    buf = np.concatenate(blocks)
    wavfile.write(path, SR, (buf * 32767).astype(np.int16))
    print(f"{path}  {len(buf) / SR:.1f}s  peak={np.max(np.abs(buf)):.2f}")


def main():
    ap = argparse.ArgumentParser(description="가상 배기음 (타이칸 스타일)")
    ap.add_argument("--mode", choices=["synth", "sample"], default="sample",
                    help="기본 sample = layers/*.wav 사용. "
                         "wav 가 없으면 자동으로 synth 로 떨어진다")
    ap.add_argument("--layers", default=LAYER_DIR, help="sample 모드 wav 폴더")
    for name in DEFAULT_REFS:                 # --ref-low / --ref-high
        ap.add_argument(f"--ref-{name}", type=int, default=DEFAULT_REFS[name],
                        help=f"{name}.wav 가 만들어진 기준 펄스 "
                             f"(loopify.py 출력값). idle 은 음정 고정이라 불필요")
    ap.add_argument("--sweep", action="store_true", help="자동 스윕 시청")
    ap.add_argument("--render", metavar="DIR", help="펄스별 wav 21개 저장")
    ap.add_argument("--export-demo", metavar="WAV",
                    help="0->20->0 스윕을 wav 로 저장")
    args = ap.parse_args()

    refs = {name: getattr(args, f"ref_{name}") for name in DEFAULT_REFS}
    eng = VirtualExhaust(args.mode, args.layers, refs)
    if args.export_demo:
        return run_export_demo(args.export_demo, eng)
    if args.render:
        run_render(eng, args.render)
    elif args.sweep:
        run_sweep(eng)
    else:
        run_keyboard(eng)


if __name__ == "__main__":
    main()
