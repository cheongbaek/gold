"""외부(AI) 생성 음원을 exhaust.py 용 심리스 루프 레이어로 가공한다.

AI 로 뽑은 음원은 그냥 쓰면 세 가지가 문제다.
  1) 루프 포인트가 없다  -> 한 바퀴 돌 때마다 "툭" 하고 이음매가 들린다
  2) 기준 음높이를 모른다 -> exhaust.py 의 --ref-low/--ref-high 를 못 정한다
  3) 음높이가 엉뚱하다    -> 재생 배율이 극단으로 가 소리가 뭉개진다
자기상관으로 파형 주기를 찾아 이음매가 맞는 길이로 자르고, 꼬리를 머리에
등파워로 겹쳐 넣고, 검출한 기본주파수로 쓸 기준 펄스를 알려준다.
--tune-to-pulse 를 주면 목표 음높이에 맞춰 미리 리샘플까지 해 둔다.

사용 예 (경로는 exhaust.py 의 LAYER_DIR ― 보통 white1/sound/layers)
-------
    python loopify.py raw_idle.wav <sound_dir>/layers/idle.wav --start 1.5 --len 2.0 \
        --tune-to-pulse 1     # 저속음과 음역을 맞춘다 (idle 은 배속 고정 재생)
    python loopify.py raw_low.wav  <sound_dir>/layers/low.wav  --start 1.5 --len 2.0 \
        --tune-to-pulse 3
    python loopify.py raw_high.wav <sound_dir>/layers/high.wav --start 1.5 --len 2.0 \
        --tune-to-pulse 6     # [2026-09-15] 음역 46~400Hz 기준 (종전 11)

그 다음 (기준 펄스는 위 실행이 마지막 줄에 찍어 준다)
-------
    python exhaust.py --mode sample --ref-high 6

★[2026-09-14] /home/mad1/sound 에서 이 패키지(nxde)로 옮겨 왔다★ exhaust.py 를
nxde.exhaust 로 절대 import 한다(이 패키지의 다른 파일들과 같은 스타일).
"""

import argparse
import os

import numpy as np
import soundfile as sf

from nxde.exhaust import MAX_PULSE, SR, F_IDLE, F_TOP, P_IDLE_OUT

# 재생 배율이 이 밖으로 나가면 음색이 눈에 띄게 상한다(늘어지거나 얇아진다)
RATE_MIN, RATE_MAX = 0.5, 2.0


def f0_at(pulse):
    return F_IDLE + (F_TOP - F_IDLE) * (pulse / MAX_PULSE)


def to_pulse(f0):
    return (f0 - F_IDLE) / (F_TOP - F_IDLE) * MAX_PULSE


def safe_ref_range():
    """(하한, 상한, 기하중심) 기준 펄스. ★표로 박지 않고 상수에서 유도한다★

    ★[2026-09-15] 종전에는 {"low": (1.5, 5.2), "high": (7.5, 12.0)} 이 하드
    코딩돼 있었는데, 그 값은 F_TOP=225 시절에 뽑은 것이라 음역을 400 으로
    넓히자 그대로 거짓말이 됐다.★ pulse_to_f0 가 바뀌면 함께 바뀌어야 하는
    값이므로 유도한다(CLAUDE.md 7절 '한쪽만 고치면 안 되는 짝').

      하한 : 펄스 20       배율 ≤ RATE_MAX  →  f_ref ≥ F_TOP / RATE_MAX
      상한 : 펄스 P_IDLE_OUT 배율 ≥ RATE_MIN  →  f_ref ≤ f0_at(3) / RATE_MIN

    ★저역 제약을 펄스 1 이 아니라 P_IDLE_OUT(=3) 에서 따지는 이유★ 그 아래는
    공회전음이 아직 살아 있어 주행음을 덮는다 — 배율이 늘어져도 들리지 않는
    구간이라 제약으로 칠 것이 아니다(exhaust.py 헤더의 "아래쪽은 공회전음이
    덮는 구간이라 감당할 만하다" 가 같은 이야기다).
    ↳ 검산 : F_TOP 을 종전 225 로 되돌리면 이 식이 7.4~11.1 을 돌려준다.
      하드코딩돼 있던 옛 표가 (7.5, 12.0) 이었으니 그 값을 재현한다.
      (펄스 1 로 따지면 7.4~7.1 이 나와 옛 표와 어긋난다 — 그래서 3 이 맞다)

    음역이 넓으면 두 조건이 겹치지 않는다(하한 > 상한). 지금 46~400Hz 가
    딱 그 경계다(8.70 > 8.60). 그때는 기하중심(배율이 위아래로 똑같이
    벌어지는 지점)이 유일한 답이다.
    """
    lo = to_pulse(F_TOP / RATE_MAX)
    hi = to_pulse(f0_at(P_IDLE_OUT) / RATE_MIN)
    center = to_pulse(float(np.sqrt(f0_at(1) * F_TOP)))
    return lo, hi, center


def load_mono(path, target_sr=SR):
    data, sr = sf.read(path, dtype="float64", always_2d=True)
    data = data.mean(axis=1)
    if sr != target_sr:
        n = int(len(data) * target_sr / sr)
        data = np.interp(np.linspace(0, len(data), n, endpoint=False),
                         np.arange(len(data)), data)
        print(f"  리샘플 {sr} -> {target_sr} Hz")
    return data


def detect_f0(x, fmin=30.0, fmax=1200.0):
    """자기상관으로 기본주파수를 찾는다. 실패하면 None."""
    seg = x[:min(len(x), SR * 2)] * np.hanning(min(len(x), SR * 2))
    corr = np.correlate(seg, seg, mode="full")[len(seg) - 1:]
    lo, hi = int(SR / fmax), min(int(SR / fmin), len(corr) - 1)
    if hi <= lo:
        return None
    peak = lo + int(np.argmax(corr[lo:hi]))
    if corr[peak] <= 0.15 * corr[0]:          # 주기성이 약하면 음정이 없는 소리
        return None
    return SR / peak


def best_loop_length(x, start, approx_len, search=0.25, probe=0.05):
    """start 지점에서 approx_len 근처로, 이음매가 가장 잘 맞는 길이를 고른다.

    루프 머리(probe 초)와 그 길이만큼 뒤의 파형을 정규화 상관으로 비교해
    가장 닮은 지점을 고른다. 파형 주기의 정수배에 자연히 걸린다.
    """
    head = x[start:start + int(probe * SR)]
    head_n = np.linalg.norm(head) + 1e-12

    lo = int((approx_len - search) * SR)
    hi = int((approx_len + search) * SR)
    lo = max(lo, int(0.05 * SR))
    hi = min(hi, len(x) - start - len(head))
    if hi <= lo:
        return min(int(approx_len * SR), len(x) - start - len(head)), 0.0

    best, best_score = lo, -2.0
    for n in range(lo, hi, 8):                # 8샘플 간격이면 충분히 촘촘하다
        seg = x[start + n:start + n + len(head)]
        score = float(head @ seg) / (head_n * (np.linalg.norm(seg) + 1e-12))
        if score > best_score:
            best, best_score = n, score
    return best, best_score


def make_loop(x, start, length, xfade):
    """꼬리를 머리에 등파워로 겹쳐 이음매를 지운다."""
    xf = min(xfade, length // 2, len(x) - start - length)
    loop = x[start:start + length].copy()
    if xf > 0:
        tail = x[start + length:start + length + xf]
        t = np.linspace(0.0, 1.0, xf, endpoint=False)
        loop[:xf] = loop[:xf] * np.sqrt(t) + tail * np.sqrt(1.0 - t)
    return loop, xf


# 공회전음이 어울려야 할 음역. 겹치는 구간(펄스 0~3)에서 저속음의 스펙트럼
# 중심이 110~175Hz 이므로 그 한가운데를 목표로 잡는다. 여기서 크게 벗어나면
# 공회전음만 혼자 붕 뜬 채로 저속음에 덮인다.
IDLE_CENTROID_TARGET = 175.0


def centroid(x, floor_db=-60.0):
    """스펙트럼 중심(Hz) = 사람이 느끼는 "음역의 높낮이".

    f0 검출은 화음에서 배신한다. 480Hz 근음의 장3화음을 넣으면 자기상관이
    구성음들의 공통 주기(가상 기본음 60Hz)를 잡아, 음높이를 보정하려 해도
    거의 움직이지 않는다. 중심 주파수는 화음이든 노이즈든 흔들리지 않고,
    리샘플하면 배율만큼 그대로 따라 움직인다.

    단 두 가지를 지켜야 값이 귀와 맞는다.
      1) 진폭이 아니라 전력(제곱)으로 가중한다
      2) 피크 대비 floor_db 아래는 버린다
    MP3 의 노이즈 플로어는 한 칸씩은 작아도 2 만 칸이 모이면 무시 못 할 합이
    된다. 실제로 전력의 100% 가 800Hz 아래인 음원을 896Hz 로 보고한 적이 있다.
    """
    sp = np.abs(np.fft.rfft(x * np.hanning(len(x)))) ** 2
    fr = np.fft.rfftfreq(len(x), 1.0 / SR)
    sp = np.where(sp < sp.max() * 10.0 ** (floor_db / 10.0), 0.0, sp)
    return float((fr * sp).sum() / (sp.sum() + 1e-12))


def retune(loop, f0_from, f0_to):
    """루프 전체를 리샘플해 기본주파수를 f0_to 로 옮긴다.

    AI 생성 음원은 원하는 음높이를 정확히 내주지 않는다. 음높이가 엉뚱하면
    exhaust.py 에서 재생 배율이 극단으로 가(0.3배 같은) 소리가 뭉개지므로,
    여기서 미리 목표 음높이에 맞춰 둔다. 순환 보간이라 이음매는 유지된다.
    """
    ratio = f0_to / f0_from
    n = max(int(round(len(loop) / ratio)), 2)
    idx = (np.arange(n) * ratio) % len(loop)
    i0 = idx.astype(np.int64)
    frac = idx - i0
    i1 = (i0 + 1) % len(loop)
    return loop[i0] * (1.0 - frac) + loop[i1] * frac, ratio


def seam_score(loop):
    """루프 이음매의 매끄러움. 내부 평균 대비 이음매 점프 배율(낮을수록 좋다)."""
    d_inner = np.abs(np.diff(loop)).mean() + 1e-12
    return abs(loop[0] - loop[-1]) / d_inner


def main():
    ap = argparse.ArgumentParser(description="AI 음원 -> 심리스 루프 레이어")
    ap.add_argument("src", help="원본 음원 (wav/flac/ogg/mp3)")
    ap.add_argument("dst", help="저장할 루프 wav")
    ap.add_argument("--start", type=float, default=0.0,
                    help="떼어낼 시작 시각(초). 어택/무음은 피할 것")
    ap.add_argument("--len", dest="length", type=float, default=1.0,
                    help="목표 루프 길이(초). 실제로는 주변에서 최적점을 찾는다")
    ap.add_argument("--xfade", type=float, default=0.05, help="이음매 크로스페이드(초)")
    ap.add_argument("--search", type=float, default=0.25,
                    help="목표 길이 주변 탐색 반경(초)")
    ap.add_argument("--gain", type=float, default=0.95, help="정규화 목표 피크")
    ap.add_argument("--tune-to-pulse", type=int, default=None, metavar="N",
                    help="이 펄스의 음높이로 맞춰 리샘플한다. "
                         "high 는 5~7 권장(음역 46~400Hz 기준 — 마지막 줄이 "
                         "지금 상수로 계산한 값을 찍어 준다). idle 은 리샘플 "
                         "없이 그대로 울리므로, 음역을 맞추려면 1~2 를 준다")
    ap.add_argument("--tune-centroid-to", type=float, default=None, metavar="HZ",
                    help="스펙트럼 중심을 이 주파수로 맞춰 리샘플한다. 화음이나 "
                         "노이즈처럼 음정이 불분명한 소리에는 이쪽이 확실하다. "
                         f"idle 은 {IDLE_CENTROID_TARGET:.0f} 권장")
    args = ap.parse_args()

    print(f"[1/4] 읽는 중: {args.src}")
    x = load_mono(args.src)
    print(f"  길이 {len(x) / SR:.2f}s")

    start = int(args.start * SR)
    if start >= len(x):
        raise SystemExit(f"[오류] --start {args.start}s 가 파일 길이를 넘습니다.")

    print("[2/4] 최적 루프 길이 탐색 중...")
    n, score = best_loop_length(x, start, args.length, args.search)
    print(f"  {n / SR:.4f}s 선택 (목표 {args.length}s, 파형 일치도 {score:+.3f})")

    print("[3/4] 이음매 크로스페이드...")
    loop, xf = make_loop(x, start, n, int(args.xfade * SR))

    f0 = detect_f0(loop)
    if args.tune_to_pulse is not None and args.tune_centroid_to is not None:
        raise SystemExit("[오류] --tune-to-pulse 와 --tune-centroid-to 는 함께 쓸 수 없습니다.")

    if args.tune_centroid_to is not None:
        loop, ratio = retune(loop, centroid(loop), args.tune_centroid_to)
        print(f"  음역 보정 (중심 기준) {ratio:.2f}배 -> 중심 {centroid(loop):.0f}Hz")
        f0 = detect_f0(loop)
    elif args.tune_to_pulse is not None:
        if f0 is None:
            raise SystemExit("[오류] 음높이를 못 찾아 --tune-to-pulse 를 쓸 수 없습니다. "
                             "--tune-centroid-to 를 쓰거나, 음정이 뚜렷한 구간을 "
                             "--start 로 골라 보세요.")
        f_to = F_IDLE + (F_TOP - F_IDLE) * (args.tune_to_pulse / MAX_PULSE)
        loop, ratio = retune(loop, f0, f_to)
        print(f"  음높이 보정 {f0:.1f}Hz -> {f_to:.1f}Hz ({ratio:.2f}배)")
        f0 = f_to
    else:
        ratio = 1.0

    if not 0.67 <= ratio <= 1.5:
        # ratio < 1 = 음을 내렸다 = 원본이 목표보다 높았다 -> 더 낮게 다시 뽑아야 한다
        print(f"  주의: 보정폭이 큽니다({ratio:.2f}배). 음색이 뭉개질 수 있으니 "
              f"원본을 더 {'높은' if ratio > 1 else '낮은'} 음으로 다시 뽑는 편이 낫습니다.")

    peak = np.max(np.abs(loop))
    if peak > 0:
        loop *= args.gain / peak

    os.makedirs(os.path.dirname(os.path.abspath(args.dst)), exist_ok=True)
    sf.write(args.dst, loop, SR, subtype="PCM_16")

    print("[4/4] 검사")
    seam = seam_score(loop)
    print(f"  이음매 점프 {seam:.2f}x "
          f"({'양호' if seam < 3 else '이음매가 들릴 수 있음 - --len/--start 를 바꿔보세요'})")

    cen = centroid(loop)
    print(f"  스펙트럼 중심 {cen:.0f}Hz (사람이 느끼는 음역의 높낮이)")

    name = os.path.splitext(os.path.basename(args.dst))[0]
    if name == "idle":
        print(f"  기본주파수 {f0:.1f}Hz" if f0 else "  기본주파수: 검출 실패(화음/노이즈)")
        print("  idle 은 배속 고정 재생이라 --ref 가 없습니다. 이 음원이 곧 공회전음입니다.")
        lo, hi = IDLE_CENTROID_TARGET * 0.7, IDLE_CENTROID_TARGET * 1.6
        if not lo <= cen <= hi:
            print(f"  주의: 저속음의 음역({lo:.0f}~{hi:.0f}Hz)을 벗어나 혼자 "
                  f"{'높이' if cen > hi else '낮게'} 뜹니다.\n"
                  f"        --tune-centroid-to {IDLE_CENTROID_TARGET:.0f} 로 다시 돌리세요.")
    elif f0 is None:
        print("  기본주파수: 검출 실패 (노이즈성 소리). idle 이면 무관하고, "
              "low/high 면 --ref 를 귀로 맞춰야 합니다.")
    else:
        pulse = (f0 - F_IDLE) / (F_TOP - F_IDLE) * MAX_PULSE
        print(f"  기본주파수 {f0:.1f}Hz  ->  exhaust.py 에 --ref-{name} {round(pulse)} "
              f"(정확값 {pulse:.1f})")
        lo, hi, center = safe_ref_range()
        if lo <= hi:
            if not lo <= pulse <= hi:
                print(f"  주의: {name} 의 안전 범위는 펄스 {lo:.1f}~{hi:.1f} 입니다. "
                      f"벗어나면 재생 배율이 {RATE_MIN}~{RATE_MAX}배를 넘어 소리가 "
                      f"뭉개집니다.\n"
                      f"        --tune-to-pulse {round(center)} 로 다시 돌리세요.")
        else:
            print(f"  참고: 지금 음역({F_IDLE:.0f}~{F_TOP:.0f}Hz)은 펄스 1~20 배율 폭이 "
                  f"{F_TOP / f0_at(1):.1f}배라, ★어떤 기준을 잡아도★ "
                  f"{RATE_MIN}~{RATE_MAX}배 안에 다 들어오지 않습니다.\n"
                  f"        위아래로 고르게 나누는 지점은 펄스 {center:.1f} "
                  f"(--tune-to-pulse {round(center)}) 입니다.")

    print(f"\n완료: {args.dst}  ({len(loop) / SR:.3f}s, 크로스페이드 {xf / SR * 1000:.0f}ms)")


if __name__ == "__main__":
    main()
