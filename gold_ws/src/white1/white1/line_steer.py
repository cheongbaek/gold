#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
line_steer.py ― ★두 점을 잇는 직선을 고속으로 추종하는 규칙의 단일 소유자★ [2026-10-19]
════════════════════════════════════════════════════════════════════════════════
braketest(제동검차, 5.1)와 driving 의 terrain X 고속구간(Xa → Xb)이 ★같은 함수·같은 상수★ 를 쓴다.
아래 상수·주석은 braketest.py 상단에 있던 것을 ★글자 그대로★ 옮긴 것이다(근거 기록 포함) —
braketest 는 이 이름들을 다시 import 해서 거동이 바이트 단위로 같다.

    S = 기준선 시작점, G = 끝점, û = (G−S)/|G−S|
    s = (P−S)·û  진행거리   e = (P−S)·n̂  횡오차(+ 왼쪽)   ψ = 헤딩 − 선방위
    L_p = clamp(t_preview × v)       e_p = e + L_p·sin ψ       δ = atan(2·L·e_p / L_p²)
    + 요레이트 댐핑 (2L/L_p)·RATE_T_S·r · CTE 적분(영점 트림) → pot(언더스티어 v 클램프)
    → 횡가속도 상한(pot_limit) → 요레이트 가드 → 슬루

ROS 를 import 하지 않는다(순수 함수와 상수만). 상태는 부르는 쪽이 들고 다닌다.
"""
import math

MS_PER_PULSE  = 0.884
KMH_PER_PULSE = 3.182

WHEELBASE_M      = 1.25   # 축거
STEER_PLANT_GAIN = 1.26   # pot 지령 / 도로휠각 (링키지비)
STEER_UNDERSTEER = 5.17   # [deg/(m/s²)] 언더스티어 보정
# ══════════════════════════════════════════════════════════════════════════════
#  ★★ [2026-09-13] 언더스티어 항의 속도를 다시 묶는다 — 조향이 포화한 채 떨었다 ★★
# ══════════════════════════════════════════════════════════════════════════════
#  ★2026-09-13 로그 실측★ 조향 부호반전 ★129.9 회/분★ (같은 날 driving 7펄스가
#  22.8, 4펄스가 0.6). 요레이트 std 15.97 °/s, 진동 주기 ★1.00 s = 6.28 rad/s★.
#  횡오차는 작았다(max 0.56 m) — ★궤적은 맞는데 조향만 계속 떨었다.★
#
#  ★원인은 포화다★ 이벤트 로그가 그대로 말한다 — t=2.3~9.5 s 내내 :
#      조향 -6.0°/6.0   (조준 -0.92 + 댐핑 -0.80 도로휠)
#      조향 -5.7°/5.7   (조준 -1.14 + 댐핑 -0.22 도로휠)
#  요구는 도로휠 1~2° 인데 pot 지령이 상한에 ★붙박이★ 다. 포화한 제어기는
#  계전기(bang-bang)가 되고, 계전기는 반드시 떤다.
#
#  ★왜 포화하나 — 이 식이 고속에서 pot 을 4~5배로 부풀린다★
#      v=8.3 m/s, 도로휠 1.62° 요구 :
#        pot = 1.26×1.62 + 5.17×69×tan1.62°/1.25 = 2.04 + 8.07 = ★10.1°★
#      그런데 ★이 노드가 주행 중에 스스로 잰 전달비★(update_plant_estimate)는
#      같은 구간에서 ★g_est 2.1 → 3.2 (표본 51)★ 였다. 모델 7.1 의 ★1/2.6★ 이다.
#      즉 pot 10.1 은 실제로 도로휠 ★3.7°★ 를 만든다 — 요구의 2.3배.
#  ⚠️ 헤더 '전달비' 절의 회귀 5.89 는 ★고속 직선의 1~2° 표본★ 이라 조향 불감대
#     아티팩트다(같은 날 driving 21340표본 회귀: 1~2° 에서 G 5.5, 7~12° 에서
#     ★1.4~1.5 로 수렴★ — 물리 전달비라면 각도와 무관해야 한다). 그 값을 근거로
#     [2026-09-12] 에 클램프를 없앤 것이 이번 진동을 만들었다. ★되살린다.★
#
#  ★값의 근거★ 이 노드가 잰 g_est 중앙 ≈2.7 에 모델을 맞춘다:
#      G(v_eff) = 1.26 + 5.17·v_eff²·0.01745/1.25 = 1.26 + 0.0724·v_eff²
#      v_eff = 4.42 (5펄스) → G = ★2.67★   ← 실측 2.7 과 일치
#  4.42 m/s 이하에서는 ★식이 한 글자도 바뀌지 않는다.★
UNDERSTEER_V_CLAMP_MS = 4.42   # [m/s] 언더스티어 항의 v 를 여기서 묶는다(5펄스)
#                                (0 이하면 클램프를 끈다 = [2026-09-12] 거동)
STEER_MAX_DEG    = 40     # B보드 수용 상한 (여기까지 갈 일은 없다)

# ══════════════════════════════════════════════════════════════════════════════
#  ★★ 조향 — 프리뷰 추종 (헤더 '멀리 본다' 절이 근거 전부다) ★★
# ══════════════════════════════════════════════════════════════════════════════
#  ★이득을 정하는 값은 이것 하나다★ L_p = T_PREVIEW_S × v.
#  1.5 s → 8.84 m/s 에서 13.3 m, ω_n 0.94 rad/s, 헤딩루프 위상여유 +48°.
#  ★키우면 매끄럽지만 벌어진 채로 가고, 줄이면 정밀하지만 위상여유를 먹는다.★
T_PREVIEW_S = 1.5
LP_MIN_M    = 5.0         # 출발 직후 v≈0 에서 L_p 가 0 이 되는 것만 막는다
LP_MAX_M    = 25.0        # 그 위로는 이득이 의미 없이 0 에 가까워진다

#  ★요레이트 댐핑 = 예견시간★ 지연 보상이다. 자이로는 ★지연이 없어서★ 오차가 0 을
#  지나기 전에 미리 힘을 뺀다 — 포화가 반대편으로 넘어가는 것을 막는 장치다.
#  이득을 따로 두지 않고 ★프리뷰 이득에 비례★ 시킨다(K_r = 2L/L_p × RATE_T_S) —
#  그래야 T_PREVIEW 를 바꿔도 감쇠비가 그대로 따라온다.
RATE_T_S = 0.30           # ≈ B보드 조향 불감시간 0.250 s

#  ★전달비 하한★ pot/도로휠. 로그 회귀 실측이 저속에서 4.96 인데 모델은 1.9 라,
#  가속 구간에서 지령이 불감대 밑으로 죽는 것을 막는다(헤더 '전달비' 절).
#  ★[2026-09-13] 4.0 → 2.6★ 위 클램프를 되살리면 모델이 전 속도대역에서 2.67 로
#  평평해지는데, 하한이 4.0 이면 ★하한이 늘 이겨★ 클램프가 무의미해진다(4.0/2.7 =
#  1.5배 과조향이 그대로 남는다). 2.6 은 이 노드가 실제로 잰 g_est(2.1~3.2, 중앙
#  2.7)의 아래쪽이고, 전달비 추정의 하한(PLANT_EST_CLAMP 1.5)보다는 위다.
#  ⚠️ 종전 근거였던 '저속 회귀 4.96' 도 작은 각 표본이라 위 불감대 아티팩트다.
PLANT_GAIN_MIN = 2.6

#  ══════════════════════════════════════════════════════════════════════════════
#  ★★ 전달비를 주행 중에 ★잰다★ — 이 시험 최대의 미지수를 스스로 지운다 ★★
#  ══════════════════════════════════════════════════════════════════════════════
#  실측 조향각(/steer_angle_measured = pot)과 투영 요레이트가 둘 다 있으므로
#      δ_실제 = atan(r·L / v)          G_est = |pot실측| / |δ_실제|
#  를 주행 중에 그대로 계산할 수 있다. ★이 값은 두 가지로 쓰인다★
#    ① pot 상한을 ★조이는 쪽으로만★ 쓴다 — 차가 모델보다 예민한 것으로 드러나면
#       권한을 줄이고, 둔한 것으로 드러나면 모델값을 그대로 둔다(한쪽으로만 움직이는
#       보정이라 추정이 튀어도 위험해지지 않는다).
#    ② 결과 표에 찍는다 — ★다음 시험의 튜닝 근거가 이 한 줄에서 확정된다.★
PLANT_EST_MIN_V_MS   = 4.0    # 이 밑에서는 요레이트가 작아 비율이 지저분하다
PLANT_EST_MIN_POT    = 3.0    # 불감대(1.78°)보다 확실히 큰 지령일 때만
PLANT_EST_MIN_DPS    = 2.0    # 실제로 돌고 있을 때만
PLANT_EST_ALPHA      = 0.12   # EMA (≈8표본 시정수)
PLANT_EST_MIN_N      = 15     # 이만큼 모이기 전에는 쓰지 않는다
PLANT_EST_CLAMP      = (1.5, 15.0)
POT_LIMIT_FLOOR_DEG  = 3.0    # 추정이 아무리 작아도 이 밑으로는 권한을 뺏지 않는다

#  ★횡가속도 상한★ 이것이 사람이 정할 값이다. 직선 보정에는 넉넉하고 급선회는
#  원천적으로 불가능하다. 8.84 m/s 에서 pot ≈ 12.6° 에 해당한다.
AY_MAX_MS2 = 2.0
#  ★pot 절대 상한★ AY 유도값이 저속에서 커지는 것을 자른다(런치 steer_limit_deg).
POT_HARD_MAX_DEG = 12.0
#  ★전달비를 재기 전까지의 상한★ — ★모르면 낮게 본다★
#  전달비가 모델만큼 예민할 가능성이 남아 있는 동안에는 권한을 열지 않는다.
#  종전 시험이 실제로 달린 권한(5°)과 같은 수준이고, 출발 직후 몇 초는 e ≈ 0 이라
#  이것으로 충분하다. 차가 스스로 둔하다는 것을 증명하면(PLANT_EST_MIN_N 표본)
#  그때 AY_MAX 가 허락하는 데까지 열린다.
POT_START_MAX_DEG = 6.0
#  ★자이로가 없으면 이만큼만★ 요레이트 가드가 못 도니 권한을 줄인다 — 모르면 낮게.
POT_NOIMU_MAX_DEG = 6.0
#  ★[2026-09-13] 속도를 모를 때도 같은 상한으로 묶는다★ pot_limit 은 δ_max 를
#  atan(ay_max·L/v²) 로 얻는데, v 를 모르면 하한 2.0 m/s 로 보므로 δ_max 가 32° 까지
#  열려 ★상한이 pot_hard_max(12°) 로 최대가 된다.★ 2026-09-13 로그에서 GPS 가
#  게이트에 막힌 t=10.5 s 부터 정확히 그렇게 됐다(상한 5.7 → ★12.0★).
#  '모르면 낮게 본다' 를 여기에도 적용한다 — 속도를 모르는 동안은 권한을 주지 않는다.
POT_NOSPEED_MAX_DEG = 6.0

#  ★조향 저역통과를 쓰지 않는다★ (1.0 = 통과) 기준에서 계단이 사라졌으므로
#  거를 것이 없고, 필터는 곧 지연이다(τ = dt/α). 슬루만 안전용으로 남긴다.
STEER_LPF        = 1.0
STEER_SLEW_DEG_S = 35.0   # B보드 서보 실측 슬루 70°/s 의 절반

# ── CTE 적분 = ★조향 영점 트림★ (수렴용이 아니다) ──────────────────────────────
#  ⚠️ 8초짜리 주행에서 적분이 수렴에 기여할 수는 없다. 그렇게 만들려면(T_i ≈ 1.3s)
#  교차주파수에서 위상을 크게 먹어 오히려 흔들린다. 그래서 ★T_i ≈ 5/ω_c★ 로 느리게
#  두고, 이 항의 일은 ★휠얼라인먼트·조향 영점 같은 진짜 정상편차를 지우는 것★ 으로
#  한정한다. 한 번의 주행 안에서 효과를 기대하지 말 것.
#  ★영점이 실제로 치우쳐 있다면★ 지난 로그의 직진 구간 평균 pot 을 읽어
#  STEER_TRIM_POT_DEG 에 적는 편이 정직하고 빠르다.
CTE_KI            = 0.15   # [deg(도로휠)/(m·s)]
CTE_I_CLAMP       = 3.0    # [m·s]
CTE_I_MAX_DEG     = 0.5    # [deg] 도로휠 기여 상한 (pot 기준 ≈3.5°)
CTE_I_DEADBAND_M  = 0.05
CTE_I_DECAY_PER_S = 0.5
CTE_I_FLIP_SCALE  = 0.35
CTE_I_MIN_PULSE   = 0.3    # 실측이 이 밑이면 ★동결★

#  ★조향 영점 트림 [pot deg]★ + 면 우, − 면 좌. 직진인데 한쪽으로 계속 밀리면
#  지난 로그의 steer_measured_deg 평균을 부호 반대로 적는다. 기본은 0(보정 없음).
STEER_TRIM_POT_DEG = 0.0

#  ── 내려오는 방벽 둘 — 어느 쪽이든 ★내리는 방향★ 이라 안전하다 ──
#  ⓐ GPS 가 끝내 안 오면 엔코더로도 본다. 엔코더는 기동 허수로 ★위로★ 튀므로
#     (실측 중앙 16, 최대 34) 이쪽이 먼저 걸리면 조금 일찍 내려가는 것뿐이다.
#  ⓑ 그래도 안 걸리면 시간으로 내린다. 15펄스로 무한정 달리지 않게 하는 마지막 줄.
LAUNCH_SWITCH_ENC_PULSE = 10.0  # 엔코더(바퀴 기준) 이 펄스를 넘어도 내린다
#  ★[2026-09-13 저녁] 엔코더 폴백에 기동 블랭킹을 건다★
#  ⚠️ 이것을 빼먹어서 첫 시험에서 ★출발 1초 만에 순항으로 내려갔다.★
#     로그(111151·111403) :
#         t=1.09s  엔코더 ★19.0펄스★ → 순항 전환   그런데 GPS 는 0.59 km/h
#         t=0.99s  엔코더 ★14.0펄스★ → 순항 전환   그런데 GPS 는 0.69 km/h
#     ★차는 서 있었다.★ 이 19·14 는 CLAUDE.md 1.2절의 ★기동 초반 허수 펄스★ 다
#     ("코일에 힘은 들어갔는데 바퀴가 아직 안 도는" 구간, 실측 중앙 16 · 최대 34).
#     그래서 15펄스 구간이 1초뿐이었고 30 km/h 도달이 전혀 앞당겨지지 않았다.
#  ★고친 방법★ 엔코더 폴백은 ★출발 후 이 시간이 지나야★ 본다. A보드 기동
#  블랭킹의 타임아웃(LAUNCH_MAX_MS = 3000)과 같은 값이라, 그 구간이 끝난 뒤에만
#  엔코더를 믿는다는 뜻이 된다. GPS 본선은 이 블랭킹을 받지 않는다 —
#  GPS 는 허수 펄스와 무관하고, 애초에 그쪽이 사용자가 지시한 판정이다.
#  ★[2026-10-16] 블랭킹으로도 모자랐다 — GPS 가 살아 있으면 엔코더를 아예 안 본다★
#  ⚠️ 2026-10-16 두 주행(033740·034110)이 ★출발 3.0 초 만에★ 순항으로 내려갔다:
#         t=3.04s  엔코더 ★25.5펄스★ → 순항 전환   그런데 GPS 는 11.4 km/h (≈3.6펄스)
#         t=3.04s  엔코더 ★16.5펄스★ → 순항 전환   그런데 GPS 는 11.6 km/h
#     허수 펄스가 3 초 뒤에도 이어졌다(t=2.5s 좌 22 · 우 9, GPS 10.9 km/h). 그 뒤 9펄스로
#     7 초를 더 갔지만 라이다가 세운 순간 ★24.2 km/h★ — 30 을 한 번도 못 넘겼다.
#  ★고친 방법★ GPS 속도가 신선하면(GPS_SPEED_FRESH_S) ★GPS 하나로만★ 내린다. 엔코더 폴백은
#  ★GPS 속도를 모를 때만★ 본다(블랭킹도 그대로 받는다). GPS 가 GPS_TIMEOUT_S 넘게 끊기면
#  loop 가 시험을 접으므로, 그 폴백이 실제로 쓰일 일은 거의 없다.
LAUNCH_ENC_BLANK_S      = 3.0   # [s] 이 시간 전에는 엔코더 폴백을 보지 않는다
GPS_SPEED_FRESH_S       = 0.5   # [s] /gps_fused 가 이 안에 왔으면 GPS 속도를 믿는다
LAUNCH_MAX_S            = 12.0  # [s] 이 시간이 지나면 무조건 순항으로 내린다

#  ★자이로가 이 안에 왔으면 신선하다★ (braketest 헤딩 절에서 옮겼다)
IMU_FRESH_S          = 0.5


# ══════════════════════════════════════════════════════════════════════════════
#  순수 함수 — braketest 의 같은 이름 메서드가 이것을 부른다 (식·연산 순서 그대로)
# ══════════════════════════════════════════════════════════════════════════════
def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def line_frame(sx, sy, gx, gy):
    """S→G 기준선 → (ux, uy, 길이 d, 선방위 deg). 길이가 0 이면 None."""
    dx, dy = gx - sx, gy - sy
    d = math.hypot(dx, dy)
    if d < 1e-6:
        return None
    return dx / d, dy / d, d, math.degrees(math.atan2(dy, dx))


def line_se(x, y, sx, sy, ux, uy):
    """선 기준 (s 진행거리, e 횡오차 + = 선의 왼쪽)."""
    dx, dy = x - sx, y - sy
    s = dx * ux + dy * uy
    e = -dx * uy + dy * ux
    return s, e


def preview_len(t_preview, v_ms):
    """★L_p = T_PREVIEW × v★ 정속 프리뷰. 속도가 변해도 안정도가 같다."""
    v = v_ms if (v_ms is not None and math.isfinite(v_ms)) else 0.0
    return clamp(t_preview * abs(v), LP_MIN_M, LP_MAX_M)


def aim_deg(e, psi_deg, lp):
    """선 위 L_p 앞의 점을 겨눈 도로휠각 [deg]. ★− 좌 / + 우★  (braketest.aim_deg 설명)"""
    if not math.isfinite(e):
        return 0.0
    e_p = e + lp * math.sin(math.radians(psi_deg))
    return math.degrees(math.atan2(2.0 * WHEELBASE_M * e_p, lp * lp))


def rate_damp_deg(lp, yaw_rate_dps):
    """★요레이트 댐핑★ [도로휠 deg] — 자이로가 신선할 때만 부를 것(아니면 0)."""
    return (2.0 * WHEELBASE_M / lp) * RATE_T_S * yaw_rate_dps


def plant_pot(road_deg, v_ms):
    """도로휠각 → pot 지령의 크기 [deg] (언더스티어 항 v 클램프 · PLANT_GAIN_MIN 하한)."""
    d = abs(road_deg)
    if d < 1e-9:
        return 0.0
    v = abs(v_ms) if (v_ms is not None and math.isfinite(v_ms)) else 0.0
    if UNDERSTEER_V_CLAMP_MS > 0.0:
        v = min(v, UNDERSTEER_V_CLAMP_MS)
    pot_model = (STEER_PLANT_GAIN * d
                 + STEER_UNDERSTEER * v * v * math.tan(math.radians(d))
                 / WHEELBASE_M)
    return max(pot_model, PLANT_GAIN_MIN * d)


def pot_limit(v_ms, ay_max, pot_hard_max, g_est, g_n, imu_fresh):
    """★상한을 '각도' 가 아니라 '횡가속도' 로★ [pot deg] (braketest.pot_limit 설명).
    g_est 를 모르면 nan · g_n 0 을 넘긴다 — 그러면 POT_START_MAX_DEG 로 묶인다(모르면 낮게)."""
    known = v_ms is not None and math.isfinite(v_ms) and abs(v_ms) > 1e-6
    v = abs(v_ms) if known else 0.0
    v = max(v, 2.0)
    d_max = math.degrees(math.atan(ay_max * WHEELBASE_M / (v * v)))
    lim = min(plant_pot(d_max, v), pot_hard_max, float(STEER_MAX_DEG))
    if math.isfinite(g_est) and g_n >= PLANT_EST_MIN_N:
        lim = min(lim, max(g_est * d_max, POT_LIMIT_FLOOR_DEG))
    else:
        lim = min(lim, POT_START_MAX_DEG)
    if not imu_fresh:
        lim = min(lim, POT_NOIMU_MAX_DEG)
    if not known:
        lim = min(lim, POT_NOSPEED_MAX_DEG)
    return lim


def yaw_rate_max(v_ms, ay_max):
    """지금 속도에서 ay_max 를 내는 요레이트 [deg/s]."""
    v = abs(v_ms) if (v_ms is not None and math.isfinite(v_ms)) else 0.0
    v = max(v, 2.0)
    return min(math.degrees(ay_max / v), 40.0)


def yaw_guard(pot, yaw_rate_dps, r_max, imu_fresh):
    """★최종 방벽★ 실제로 도는 속도가 상한을 넘으면 ★그 방향★ 지령만 깎는다."""
    if not imu_fresh:
        return pot
    r = abs(yaw_rate_dps)
    if r <= r_max or r < 1e-6:
        return pot
    if pot * yaw_rate_dps < 0.0:
        return pot * (r_max / r)
    return pot


def cte_integral_step(cte_i, cte_prev, cte, saturated, moving, dt):
    """CTE 적분(= 조향 영점 트림) 한 틱 → (cte_i, cte_prev, i_term). cte 가 nan 이면 부르지 말 것."""
    if (cte * cte_prev) < 0.0 \
            and abs(cte) > CTE_I_DEADBAND_M \
            and abs(cte_prev) > CTE_I_DEADBAND_M:
        cte_i *= CTE_I_FLIP_SCALE
    cte_prev = cte
    blocked = saturated and (cte * cte_i >= 0.0)
    if moving:
        if abs(cte) < CTE_I_DEADBAND_M:
            cte_i -= cte_i * min(1.0, CTE_I_DECAY_PER_S * dt)
        elif not blocked:
            cte_i += cte * dt
        cte_i = clamp(cte_i, -CTE_I_CLAMP, CTE_I_CLAMP)
    i_term = clamp(CTE_KI * cte_i, -CTE_I_MAX_DEG, CTE_I_MAX_DEG)
    return cte_i, cte_prev, i_term


def smooth_step(out, pot, dt, lim):
    """슬루 제한 한 틱 → 새 출력. dt 는 부르는 쪽이 [1e-3, 0.2] 로 잘라 넘긴다."""
    want = out + STEER_LPF * (pot - out)
    slew = STEER_SLEW_DEG_S * dt
    out += clamp(want - out, -slew, slew)
    return clamp(out, -lim, lim)


def launch_switch_why(gps_kmh, gps_fresh, since_launch_s, enc_pulse, switch_kmh, launch_max_s):
    """출발 펄스 → 순항 펄스로 ★내릴 이유★ (없으면 None). since_launch_s 는 출발 안 했으면 None.
    braketest.drive_value 의 ⓐ GPS · ⓑ 엔코더(GPS 를 모를 때만, 블랭킹 뒤) · ⓒ 시간."""
    if gps_fresh and gps_kmh >= switch_kmh:
        return f"GPS {gps_kmh:.1f} km/h ≥ {switch_kmh:.0f}"
    if (not gps_fresh and since_launch_s is not None
            and since_launch_s >= LAUNCH_ENC_BLANK_S
            and enc_pulse >= LAUNCH_SWITCH_ENC_PULSE):
        return (f"엔코더 {enc_pulse:.1f}펄스 ≥ "
                f"{LAUNCH_SWITCH_ENC_PULSE:.0f} (★GPS 속도 없음★ — 폴백, "
                f"기동 블랭킹 {LAUNCH_ENC_BLANK_S:.0f}s 경과)")
    if since_launch_s is not None and since_launch_s >= launch_max_s:
        return f"출발 {since_launch_s:.1f}s 경과 ≥ {launch_max_s:.0f}s"
    return None
