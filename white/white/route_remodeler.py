#!/usr/bin/env python3
"""
route_remodeler.py - smart line/corner remodeling version

매핑 CSV 경로를 자율주행용으로 자동 리모델링한다.

핵심 변경점
1. 원본 GPS 지그재그를 그대로 등간격화하지 않고, RDP로 경로의 큰 골격을 먼저 추출한다.
2. 직선으로 판단되는 구간은 waypoint를 회귀선/골격선 위에 정확히 정렬한다.
3. 코너는 직선과 직선 사이를 cubic Bezier 필렛으로 둥글게 연결한다.
4. 마지막에는 driving.py가 읽는 CSV 컬럼 그대로 저장한다.

주의
- 너무 좁은 코스에서 코너를 과하게 깎지 않도록 코너 반경은 보수적으로 제한한다.
- 기존 mapping.py의 remodel_route(input_csv, output_csv, spacing, epsilon, smooth_iter) 호출 방식과 호환된다.
"""

import argparse
import csv
import glob
import math
import os
from typing import Dict, List, Tuple, Optional

EARTH_R = 6378137.0
Point = Tuple[float, float]


# ═══════════════════════════════════════════════════════════════════════
# 기본 유틸
# ═══════════════════════════════════════════════════════════════════════
def find_latest_route(data_dir: str) -> Optional[str]:
    pattern = os.path.join(os.path.expanduser(data_dir), "route_*.csv")
    candidates = [
        p for p in glob.glob(pattern)
        if os.path.isfile(p) and not os.path.basename(p).endswith("_remodeled.csv")
    ]
    if not candidates:
        return None
    candidates.sort(key=os.path.getmtime, reverse=True)
    return candidates[0]


def latlon_to_xy(lat: float, lon: float, lat0: float, lon0: float) -> Point:
    x = EARTH_R * math.radians(lon - lon0) * math.cos(math.radians(lat0))
    y = EARTH_R * math.radians(lat - lat0)
    return x, y


def xy_to_latlon(x: float, y: float, lat0: float, lon0: float) -> Tuple[float, float]:
    lat = lat0 + math.degrees(y / EARTH_R)
    lon = lon0 + math.degrees(x / (EARTH_R * math.cos(math.radians(lat0))))
    return lat, lon


def dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def add(a: Point, b: Point) -> Point:
    return (a[0] + b[0], a[1] + b[1])


def sub(a: Point, b: Point) -> Point:
    return (a[0] - b[0], a[1] - b[1])


def mul(a: Point, k: float) -> Point:
    return (a[0] * k, a[1] * k)


def dot(a: Point, b: Point) -> float:
    return a[0] * b[0] + a[1] * b[1]


def norm(a: Point) -> float:
    return math.hypot(a[0], a[1])


def unit(a: Point) -> Point:
    n = norm(a)
    if n < 1e-9:
        return (0.0, 0.0)
    return (a[0] / n, a[1] / n)


def normalize_angle(deg: float) -> float:
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg


def cumulative_distance(points: List[Point]) -> List[float]:
    if not points:
        return []
    s = [0.0]
    for i in range(1, len(points)):
        s.append(s[-1] + dist(points[i - 1], points[i]))
    return s


def heading_deg(points: List[Point], i: int) -> float:
    if len(points) <= 1:
        return 0.0
    # 최종 경로의 heading은 너무 짧은 이웃이 아니라 2~3포인트 앞뒤 평균으로 계산한다.
    j0 = max(0, i - 2)
    j1 = min(len(points) - 1, i + 2)
    if j0 == j1:
        j0 = max(0, i - 1)
        j1 = min(len(points) - 1, i + 1)
    p0, p1 = points[j0], points[j1]
    return normalize_angle(math.degrees(math.atan2(p1[1] - p0[1], p1[0] - p0[0])))


def point_line_distance(p: Point, a: Point, b: Point) -> float:
    vx, vy = b[0] - a[0], b[1] - a[1]
    den = math.hypot(vx, vy)
    if den < 1e-9:
        return dist(p, a)
    return abs(vx * (a[1] - p[1]) - (a[0] - p[0]) * vy) / den


def project_point_to_line(p: Point, a: Point, b: Point) -> Point:
    ab = sub(b, a)
    den = dot(ab, ab)
    if den < 1e-9:
        return a
    t = dot(sub(p, a), ab) / den
    return add(a, mul(ab, t))


# ═══════════════════════════════════════════════════════════════════════
# CSV 읽기 / 중복 제거
# ═══════════════════════════════════════════════════════════════════════
def read_route_csv(path: str) -> List[Dict[str, str]]:
    with open(os.path.expanduser(path), "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        fieldnames = reader.fieldnames or []
    if not rows:
        raise ValueError("CSV에 waypoint가 없습니다.")
    if "latitude" not in fieldnames or "longitude" not in fieldnames:
        raise ValueError("CSV에 latitude, longitude 컬럼이 필요합니다.")
    return rows


def remove_duplicate_points_with_rows(rows: List[Dict[str, str]], points: List[Point], min_dist_m: float = 0.05):
    if not points:
        return [], []
    out_rows = [rows[0]]
    out_pts = [points[0]]
    last = points[0]
    for r, p in zip(rows[1:], points[1:]):
        if dist(last, p) >= min_dist_m:
            out_rows.append(r)
            out_pts.append(p)
            last = p
    if dist(out_pts[-1], points[-1]) > 1e-6:
        out_rows.append(rows[-1])
        out_pts.append(points[-1])
    return out_rows, out_pts


# ═══════════════════════════════════════════════════════════════════════
# 리샘플링 / 보간
# ═══════════════════════════════════════════════════════════════════════
def resample_by_spacing(points: List[Point], spacing: float) -> List[Point]:
    if len(points) <= 1:
        return points[:]
    spacing = max(0.05, float(spacing))
    s = cumulative_distance(points)
    total = s[-1]
    if total <= 1e-9:
        return [points[0]]

    targets = [0.0]
    x = spacing
    while x < total:
        targets.append(x)
        x += spacing
    if total - targets[-1] > spacing * 0.35:
        targets.append(total)
    else:
        targets[-1] = total

    out: List[Point] = []
    seg = 0
    for ts in targets:
        while seg < len(s) - 2 and s[seg + 1] < ts:
            seg += 1
        s0, s1 = s[seg], s[seg + 1]
        p0, p1 = points[seg], points[seg + 1]
        if s1 - s0 <= 1e-9:
            out.append(p0)
        else:
            u = (ts - s0) / (s1 - s0)
            out.append((p0[0] + u * (p1[0] - p0[0]), p0[1] + u * (p1[1] - p0[1])))
    return out


def interp_numeric(rows: List[Dict[str, str]], original_s: List[float], target_s: float, key: str, default: float = 0.0) -> float:
    if not rows or not original_s:
        return default
    total = original_s[-1]
    if total <= 1e-9:
        try:
            return float(rows[0].get(key, default))
        except ValueError:
            return default
    target_s = max(0.0, min(total, target_s))
    idx = 0
    while idx < len(original_s) - 2 and original_s[idx + 1] < target_s:
        idx += 1
    try:
        v0 = float(rows[idx].get(key, default))
        v1 = float(rows[idx + 1].get(key, default))
    except (ValueError, IndexError):
        return default
    s0, s1 = original_s[idx], original_s[min(idx + 1, len(original_s) - 1)]
    if s1 - s0 <= 1e-9:
        return v0
    u = (target_s - s0) / (s1 - s0)
    return v0 + u * (v1 - v0)


def nearest_direction(rows: List[Dict[str, str]], original_s: List[float], target_s: float) -> int:
    if not rows or not original_s:
        return 1
    idx = min(range(len(original_s)), key=lambda i: abs(original_s[i] - target_s))
    try:
        return int(float(rows[idx].get("direction", 1)))
    except (ValueError, IndexError):
        return 1


# ═══════════════════════════════════════════════════════════════════════
# [카메라 융합 Phase0] lane_* 컬럼 재투영
# ═══════════════════════════════════════════════════════════════════════
LANE_FIELDS = ["lane_cte", "lane_conf", "lane_flags"]
# [카메라 융합 Phase1] 각도·곡률은 선택(optional) 컬럼 — 구버전 맵(3컬럼)엔 없다.
#   코어(LANE_FIELDS)와 분리해 감지한다 → 이걸 LANE_FIELDS 에 합치면 구버전 맵이
#   has_lane_columns=False 로 떨어져 기존 lane_cte 보존까지 깨진다.
LANE_FIELDS_OPT = ["lane_theta", "lane_curv"]
LANE_CONF_MIN = 0.5      # 양쪽 차선 검출 하한(perception _soft_confidence 규약)
LANE_PROJ_WINDOW_M = 3.0  # 원본 폴리라인 투영 탐색창(루프 경로 오투영 방지)


def has_lane_columns(rows: List[Dict[str, str]]) -> bool:
    return bool(rows) and all(k in rows[0] for k in LANE_FIELDS)


def has_lane_opt_columns(rows: List[Dict[str, str]]) -> bool:
    return bool(rows) and all(k in rows[0] for k in LANE_FIELDS_OPT)


def build_lane_opt_columns(rows: List[Dict[str, str]], s_list: List[float],
                           src_s: float, ok: bool) -> Dict[str, str]:
    """theta/curv 선택 컬럼. cte 처럼 δ(횡변위) 보정이 필요 없다(각도/곡률이라 평행이동
    불변). 코어 lane 이 무효(ok=False)면 0 — conf=0 행의 각도는 오검출이라 섞으면 안 됨.
    ⚠️ theta 는 '지도접선−헤딩' 이라, 리모델 스무딩이 접선을 바꾸면 엄밀히는
       theta -= Δheading 보정이 필요하다. 스무딩 이동이 작아(epsilon~0.08m) 1차
       근사로 호길이 보간만 한다 — 각도 재보정은 주행측 검증 후 도입(Phase1 TODO)."""
    if not ok:
        return {"lane_theta": "0.00", "lane_curv": "0.000000"}
    return {
        "lane_theta": f"{interp_numeric(rows, s_list, src_s, 'lane_theta', 0.0):.2f}",
        "lane_curv":  f"{interp_numeric(rows, s_list, src_s, 'lane_curv', 0.0):.6f}",
    }


def _fnum(v, default: float = 0.0) -> float:
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def project_to_polyline(p: Point, pts: List[Point], s_list: List[float],
                        s_hint: Optional[float] = None,
                        window_m: float = LANE_PROJ_WINDOW_M) -> Tuple[float, float]:
    """p 에서 폴리라인에 내린 수선 → (호길이 s, 부호있는 횡변위).

    횡변위 부호는 폴리라인 진행방향 기준 '우측이 +' (발행 cte 규약과 동일).
    s_hint 가 있으면 그 호길이 ±window_m 안의 세그먼트만 본다 — 루프 경로에서
    반대편 구간으로 잘못 투영되는 것을 막는다.
    """
    if len(pts) < 2:
        return 0.0, 0.0
    best_d = float("inf")
    best_s = 0.0
    best_off = 0.0
    for i in range(len(pts) - 1):
        if s_hint is not None and window_m > 0.0:
            # 세그먼트가 탐색창에서 완전히 벗어나면 건너뛴다.
            if s_list[i + 1] < s_hint - window_m or s_list[i] > s_hint + window_m:
                continue
        a, b = pts[i], pts[i + 1]
        ab = sub(b, a)
        den = dot(ab, ab)
        if den < 1e-12:
            continue
        t = max(0.0, min(1.0, dot(sub(p, a), ab) / den))
        foot = add(a, mul(ab, t))
        d = dist(p, foot)
        if d < best_d:
            u = unit(ab)
            # x=동, y=북, heading=atan2(dy,dx) → 좌측법선=(-u_y, u_x), 우측법선=(u_y, -u_x)
            right_n = (u[1], -u[0])
            best_d = d
            best_s = s_list[i] + t * norm(ab)
            best_off = dot(sub(p, foot), right_n)
    if best_d == float("inf"):
        # 탐색창이 비었으면(=hint가 경로 밖) 창 없이 재시도
        return project_to_polyline(p, pts, s_list, s_hint=None, window_m=0.0)
    return best_s, best_off


def _bracket(original_s: List[float], target_s: float) -> Tuple[int, float]:
    """target_s 를 감싸는 원본 인덱스 i 와 [i, i+1] 사이 보간계수 u 를 반환."""
    n = len(original_s)
    if n < 2:
        return 0, 0.0
    target_s = max(0.0, min(original_s[-1], target_s))
    i = 0
    while i < n - 2 and original_s[i + 1] < target_s:
        i += 1
    s0, s1 = original_s[i], original_s[i + 1]
    if s1 - s0 <= 1e-9:
        return i, 0.0
    return i, (target_s - s0) / (s1 - s0)


def build_lane_columns(rows: List[Dict[str, str]], original_s: List[float],
                       original_xy: List[Point], p: Point,
                       s_hint: Optional[float] = None) -> Tuple[Dict[str, str], float, bool]:
    """리모델 점 p 의 lane_* 컬럼을 원본에서 재투영해 만든다. → (컬럼dict, |δ|, 유효여부)

    발행 cte 규약은 '우측+'(차량이 차선중심보다 우측이면 양수). 매핑 차량이 원본점 O 에서
    lane_cte 를 읽었다면  차선중심 = O − lane_cte·(우측법선)  이므로, 리모델 점 P 에 차량이
    있을 때 카메라가 읽을 값은

        cte(P) = (P − 차선중심)·우측법선 = (P − O)·우측법선 + lane_cte = δ + lane_cte

    즉 리모델이 경로를 우측으로 δ 옮겼으면 lane_cte 도 δ 만큼 커진다. ★이 보정을 빼먹으면
    맵(waypoint)은 리모델된 골격선을 가리키는데 lane_cte 는 원본 지그재그를 가리켜, 주행 중
    횡보정이 차를 리모델 전 궤적으로 도로 끌고 간다 — 리모델링이 무력화된다.

    양끝 표본이 모두 유효(conf≥LANE_CONF_MIN)할 때만 보간한다. 한쪽이 미검출이면 그 행의
    lane_cte 는 0(더미)이라, 선형보간이 실제값과 0 사이에서 없는 값을 지어낸다.
    """
    invalid = ({"lane_cte": "0.0000", "lane_conf": "0.000", "lane_flags": "0"}, 0.0, False)
    if len(original_xy) < 2 or len(rows) < 2:
        return invalid

    s_at, delta = project_to_polyline(p, original_xy, original_s, s_hint=s_hint)
    i, u = _bracket(original_s, s_at)
    if i + 1 >= len(rows):
        return invalid

    c0 = _fnum(rows[i].get("lane_conf"), 0.0)
    c1 = _fnum(rows[i + 1].get("lane_conf"), 0.0)
    if min(c0, c1) < LANE_CONF_MIN:
        return invalid

    cte = (1.0 - u) * _fnum(rows[i].get("lane_cte")) + u * _fnum(rows[i + 1].get("lane_cte"))
    cte += delta
    flags = int(_fnum(rows[i].get("lane_flags"))) | int(_fnum(rows[i + 1].get("lane_flags")))
    return ({
        "lane_cte":   f"{cte:.4f}",
        "lane_conf":  f"{min(c0, c1):.3f}",
        "lane_flags": f"{flags}",
    }, abs(delta), True)


# ═══════════════════════════════════════════════════════════════════════
# 경로 골격 추출: RDP
# ═══════════════════════════════════════════════════════════════════════
def rdp_indices(points: List[Point], epsilon: float) -> List[int]:
    """Ramer-Douglas-Peucker. 직선 구간은 양끝점만 남기고, 코너 골격점은 보존한다."""
    n = len(points)
    if n <= 2:
        return list(range(n))

    keep = {0, n - 1}

    def _rdp(start: int, end: int):
        if end <= start + 1:
            return
        a, b = points[start], points[end]
        max_d = -1.0
        max_i = start
        for i in range(start + 1, end):
            d = point_line_distance(points[i], a, b)
            if d > max_d:
                max_d = d
                max_i = i
        if max_d > epsilon:
            keep.add(max_i)
            _rdp(start, max_i)
            _rdp(max_i, end)

    _rdp(0, n - 1)
    return sorted(keep)


def remove_near_collinear_keypoints(points: List[Point], angle_thresh_deg: float = 5.0, min_seg_len: float = 0.30) -> List[Point]:
    if len(points) <= 2:
        return points[:]
    out = [points[0]]
    for i in range(1, len(points) - 1):
        a, b, c = out[-1], points[i], points[i + 1]
        v1 = unit(sub(b, a))
        v2 = unit(sub(c, b))
        if norm(v1) < 1e-9 or norm(v2) < 1e-9:
            continue
        ang = math.degrees(math.acos(max(-1.0, min(1.0, dot(v1, v2)))))
        if ang < angle_thresh_deg and dist(a, b) < min_seg_len:
            continue
        if ang < angle_thresh_deg:
            # 거의 직선이면 중간 골격점을 제거해서 긴 직선 한 줄로 만든다.
            continue
        out.append(b)
    out.append(points[-1])
    return out


# ═══════════════════════════════════════════════════════════════════════
# 직선화 / 코너 라운딩
# ═══════════════════════════════════════════════════════════════════════
def bezier_point(p0: Point, p1: Point, p2: Point, p3: Point, t: float) -> Point:
    u = 1.0 - t
    return (
        u ** 3 * p0[0] + 3 * u * u * t * p1[0] + 3 * u * t * t * p2[0] + t ** 3 * p3[0],
        u ** 3 * p0[1] + 3 * u * u * t * p1[1] + 3 * u * t * t * p2[1] + t ** 3 * p3[1],
    )


def sample_line(a: Point, b: Point, spacing: float) -> List[Point]:
    length = dist(a, b)
    if length < 1e-9:
        return [a]
    n = max(1, int(math.ceil(length / max(0.05, spacing))))
    return [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(n)]


def sample_bezier(p0: Point, p1: Point, p2: Point, p3: Point, spacing: float) -> List[Point]:
    chord = dist(p0, p3)
    ctrl = dist(p0, p1) + dist(p1, p2) + dist(p2, p3)
    approx_len = max(chord, 0.5 * (chord + ctrl))
    n = max(3, int(math.ceil(approx_len / max(0.05, spacing))))
    return [bezier_point(p0, p1, p2, p3, i / n) for i in range(n)]


def build_smart_path(
    original_xy: List[Point],
    spacing: float = 0.25,
    epsilon: float = 0.08,
    smooth_iter: int = 1,
    straight_angle_deg: float = 7.0,
    corner_start_angle_deg: float = 9.0,
    vehicle_min_radius_m: float = 1.90,   # 차량 물리 최소 선회반경 = L/tan(21°) (L=0.73). 경로는 이보다 더 조이면 추종 불가.
    corner_radius_margin: float = 1.30,   # 위 한계에 곱하는 안전 여유 → 목표 호반경(약 2.47m).
    max_corner_radius_m: float = 6.0,     # 완만한 코너를 과도하게 부풀리지 않도록 호반경 상한.
    vehicle_width_m: float = 0.775,       # [0713] 실측 차폭. 경로(=뒷차축 중심선) 안쪽으로
                                           #   차체가 반폭만큼 파고들므로, 코너 목표반경의
                                           #   물리적 하한(=vehicle_min_radius_m + width/2)으로 사용.
) -> Tuple[List[Point], Dict[str, float]]:
    """
    1) 등간격 예비 샘플링
    2) RDP로 큰 골격 추출
    3) 골격 직선은 정확한 선분으로, 코너는 Bezier 필렛으로 연결
    """
    if len(original_xy) <= 2:
        return original_xy[:], {"skeleton_points": len(original_xy), "corners": 0, "straight_segments": max(0, len(original_xy) - 1)}

    spacing = max(0.08, spacing)

    # 예비 샘플링으로 입력 점 간격을 안정화한다.
    pre_spacing = max(0.12, min(spacing, 0.20))
    pre = resample_by_spacing(original_xy, pre_spacing)

    # RDP 허용오차: mapping.py에서 epsilon=0.08을 줘도 긴 직선이 한 줄로 잡히도록 내부 기준을 살짝 키운다.
    # 너무 크게 잡으면 코너를 깎으므로 0.12~0.35m 범위로 제한한다.
    rdp_eps = max(0.12, min(0.35, epsilon * 2.2))
    idxs = rdp_indices(pre, rdp_eps)
    skeleton = [pre[i] for i in idxs]
    skeleton = remove_near_collinear_keypoints(skeleton, angle_thresh_deg=straight_angle_deg, min_seg_len=spacing * 2.0)

    if len(skeleton) <= 2:
        final_line = resample_by_spacing([skeleton[0], skeleton[-1]], spacing)
        return final_line, {"skeleton_points": len(skeleton), "corners": 0, "straight_segments": 1}

    # corner fillet를 생성한다. 직선 구간은 골격 선분 그대로 사용된다.
    rounded: List[Point] = []
    last_anchor = skeleton[0]
    corner_count = 0
    straight_count = 0

    for i in range(1, len(skeleton) - 1):
        a, b, c = skeleton[i - 1], skeleton[i], skeleton[i + 1]
        len_in = dist(a, b)
        len_out = dist(b, c)
        if len_in < 1e-6 or len_out < 1e-6:
            continue

        u_in = unit(sub(b, a))     # a -> b
        u_out = unit(sub(c, b))    # b -> c
        turn_angle = math.degrees(math.acos(max(-1.0, min(1.0, dot(u_in, u_out)))))

        if turn_angle < corner_start_angle_deg:
            # 거의 직선이면 b를 무시하고 다음 골격점까지 직선으로 이어지게 둔다.
            continue

        # ── 코너 반경 결정 (차량 선회한계 기반, 기하학적으로 올바른 방식) ───────────
        # 여기서 변수 radius는 '코너 꼭짓점 b에서 각 다리를 따라 뒤로 물린 셋백 거리'다.
        # 셋백 t 와 실제 필렛 호반경 R 의 관계:  t = R · tan(θ/2)   (θ=편향각, 직선=0)
        #   → R = t / tan(θ/2).  같은 셋백이면 코너가 급할수록(θ↑) 호반경이 더 작아진다.
        # 따라서 "급한 코너일수록 셋백을 더 크게" 줘야 호반경이 차량 최소반경 위로 유지된다.
        # (기존 로직은 급할수록 반경을 줄여서, 차가 못 도는 0.35~0.66m 코너를 만들고 있었음.)
        # [0713] 차폭 반영: 차체는 경로(뒷차축 중심선) 안쪽으로 반폭(width/2)만큼 파고드므로,
        #   목표호반경이 '차량 최소선회반경+반폭'보다 작아지면 코너 안쪽에서 차체 여유가 없어진다.
        #   기존 margin(1.30배)이 이미 이 하한보다 크면(현재 2.47m > 2.29m) 동작 변화 없음 —
        #   margin을 나중에 줄이더라도 이 물리적 하한 아래로는 절대 안 좁아지게 하는 안전장치.
        min_radius_with_width = vehicle_min_radius_m + vehicle_width_m / 2.0
        target_radius = max(vehicle_min_radius_m * corner_radius_margin, min_radius_with_width)
        target_radius = min(target_radius, max_corner_radius_m)
        half = 0.5 * math.radians(turn_angle)
        tan_half = math.tan(min(half, math.radians(80.0)))             # 과도한 급각에서 폭주 방지
        setback_needed = target_radius * tan_half                       # 목표 호반경을 내려면 필요한 셋백
        # 세그먼트가 허용하는 셋백 상한. 셋백은 인접 코너와 다리를 나눠 쓰므로 0.45배로 제한.
        geom_cap = min(len_in, len_out) * 0.45
        radius = min(setback_needed, geom_cap)
        # 차량 최소반경(여유 없이) 자체를 못 내는 셋백으로는 깎지 않는다. 단 세그먼트가 그조차
        # 허용 못 하면(=골격이 물리적으로 너무 빡빡) 줄 수 있는 최대(geom_cap)까지만 best-effort.
        radius = max(radius, min(vehicle_min_radius_m * tan_half, geom_cap))

        if radius < spacing * 1.2:
            # 너무 짧은 코너는 필렛을 넣지 않고 골격점만 통과한다.
            line_pts = sample_line(last_anchor, b, spacing)
            if rounded and line_pts and dist(rounded[-1], line_pts[0]) < 1e-6:
                line_pts = line_pts[1:]
            rounded.extend(line_pts)
            last_anchor = b
            continue

        p_in = sub(b, mul(u_in, radius))
        p_out = add(b, mul(u_out, radius))

        # 직선부 추가
        line_pts = sample_line(last_anchor, p_in, spacing)
        if rounded and line_pts and dist(rounded[-1], line_pts[0]) < 1e-6:
            line_pts = line_pts[1:]
        rounded.extend(line_pts)
        straight_count += 1

        # Bezier 코너 추가. k가 너무 크면 overshoot, 너무 작으면 각짐.
        k = 0.55
        c1 = add(p_in, mul(u_in, radius * k))
        c2 = sub(p_out, mul(u_out, radius * k))
        curve_pts = sample_bezier(p_in, c1, c2, p_out, spacing)
        if rounded and curve_pts and dist(rounded[-1], curve_pts[0]) < 1e-6:
            curve_pts = curve_pts[1:]
        rounded.extend(curve_pts)
        last_anchor = p_out
        corner_count += 1

    # 마지막 직선부 추가
    tail_pts = sample_line(last_anchor, skeleton[-1], spacing)
    if rounded and tail_pts and dist(rounded[-1], tail_pts[0]) < 1e-6:
        tail_pts = tail_pts[1:]
    rounded.extend(tail_pts)

    if not rounded or dist(rounded[-1], skeleton[-1]) > 1e-6:
        rounded.append(skeleton[-1])

    # 최종 등간격화. 이 단계에서는 형태를 다시 평균내지 않는다.
    remodeled = resample_by_spacing(rounded, spacing)

    # 옵션 smooth_iter는 코너를 깎는 이동평균이 아니라, 최종 heading 급변만 줄이는 아주 약한 Laplacian으로 사용한다.
    # 직선은 이미 완전 직선이고, 코너는 Bezier라서 기본값 1~2면 충분하다.
    if smooth_iter > 0 and len(remodeled) > 5:
        max_shift = max(0.02, min(0.06, epsilon * 0.6))
        for _ in range(min(2, smooth_iter)):
            new_pts = remodeled[:]
            for j in range(2, len(remodeled) - 2):
                # 골격 직선성을 해치지 않도록 아주 약하게만 적용
                avg = (
                    (remodeled[j - 1][0] + remodeled[j][0] + remodeled[j + 1][0]) / 3.0,
                    (remodeled[j - 1][1] + remodeled[j][1] + remodeled[j + 1][1]) / 3.0,
                )
                nx = remodeled[j][0] * 0.82 + avg[0] * 0.18
                ny = remodeled[j][1] * 0.82 + avg[1] * 0.18
                shift = math.hypot(nx - remodeled[j][0], ny - remodeled[j][1])
                if shift > max_shift:
                    sc = max_shift / shift
                    nx = remodeled[j][0] + (nx - remodeled[j][0]) * sc
                    ny = remodeled[j][1] + (ny - remodeled[j][1]) * sc
                new_pts[j] = (nx, ny)
            remodeled = new_pts

    stats = {
        "skeleton_points": float(len(skeleton)),
        "corners": float(corner_count),
        "straight_segments": float(max(1, straight_count + 1)),
        "rdp_epsilon_m": float(rdp_eps),
        "vehicle_width_m": float(vehicle_width_m),
        "min_radius_with_width_m": float(vehicle_min_radius_m + vehicle_width_m / 2.0),
    }
    return remodeled, stats


# ═══════════════════════════════════════════════════════════════════════
# 메인 리모델링 함수: mapping.py와 호환
# ═══════════════════════════════════════════════════════════════════════
def _route_directions(rows_raw: List[Dict[str, str]]) -> List[int]:
    dirs = []
    for r in rows_raw:
        try:
            dirs.append(1 if int(float(r.get("direction", 1))) >= 0 else -1)
        except (ValueError, TypeError):
            dirs.append(1)
    return dirs


def remodel_route(
    input_csv: str,
    output_csv: Optional[str] = None,
    spacing: float = 0.25,
    epsilon: float = 0.08,
    smooth_iter: int = 1,
) -> str:
    """방향 게이트 디스패처.

    - 단일 방향(전진 전용 등) 경로: 기존 로직(_remodel_route_single)을 그대로 사용 →
      출력이 기존과 100% 동일하다(현재 잘 되는 주행 경로는 변화 없음).
    - 전진/후진이 섞인 경로: 방향이 같은 구간(run)별로 따로 리모델링한 뒤 이어붙인다.
      이렇게 하면 전진↔후진 반전점(cusp)을 U턴으로 둥글게 깎지 않고 보존하며,
      direction 라벨도 구간별로 정확히 유지된다.
    """
    rows_raw = read_route_csv(input_csv)
    dirs = _route_directions(rows_raw)
    mixed = any(d >= 0 for d in dirs) and any(d < 0 for d in dirs)

    if not mixed:
        return _remodel_route_single(input_csv, rows_raw, output_csv, spacing, epsilon, smooth_iter)
    return _remodel_route_segmented(input_csv, rows_raw, dirs, output_csv, spacing, epsilon, smooth_iter)


def _remodel_route_single(
    input_csv: str,
    rows_raw: List[Dict[str, str]],
    output_csv: Optional[str],
    spacing: float,
    epsilon: float,
    smooth_iter: int,
) -> str:
    # ── (기존 remodel_route 본문 그대로) ────────────────────────────────────
    lat0 = float(rows_raw[0]["latitude"])
    lon0 = float(rows_raw[0]["longitude"])
    xy_raw = [latlon_to_xy(float(r["latitude"]), float(r["longitude"]), lat0, lon0) for r in rows_raw]

    rows, original_xy = remove_duplicate_points_with_rows(rows_raw, xy_raw, min_dist_m=0.05)
    original_s = cumulative_distance(original_xy)
    original_len = original_s[-1] if original_s else 0.0

    remodeled_xy, stats = build_smart_path(
        original_xy,
        spacing=spacing,
        epsilon=epsilon,
        smooth_iter=smooth_iter,
    )
    remodeled_s = cumulative_distance(remodeled_xy)
    remodeled_len = remodeled_s[-1] if remodeled_s else 0.0

    if output_csv is None:
        root, _ = os.path.splitext(os.path.expanduser(input_csv))
        output_csv = root + "_remodeled.csv"

    fieldnames = ["latitude", "longitude", "heading", "speed", "steer", "direction", "pitch", "terrain"]
    # [카메라 융합] 입력에 lane_* 이 있을 때만 출력에 붙인다 → 구버전 맵의 출력은 바이트 동일.
    has_lane = has_lane_columns(rows)
    has_lane_opt = has_lane and has_lane_opt_columns(rows)
    if has_lane:
        fieldnames = fieldnames + LANE_FIELDS
        if has_lane_opt:
            fieldnames = fieldnames + LANE_FIELDS_OPT
    lane_ok_count = 0
    lane_delta_sum = 0.0
    lane_delta_max = 0.0

    out_rows = []
    for i, p in enumerate(remodeled_xy):
        progress = 0.0 if remodeled_len <= 1e-9 else remodeled_s[i] / remodeled_len
        src_s = progress * original_len
        lat, lon = xy_to_latlon(p[0], p[1], lat0, lon0)
        h = heading_deg(remodeled_xy, i)
        row_out = {
            "latitude":  f"{lat:.8f}",
            "longitude": f"{lon:.8f}",
            "heading":   f"{h:.2f}",
            "speed":     f"{interp_numeric(rows, original_s, src_s, 'speed',  0.0):.4f}",
            "steer":     f"{interp_numeric(rows, original_s, src_s, 'steer',  0.0):.2f}",
            "direction": str(nearest_direction(rows, original_s, src_s)),
            "pitch":     f"{interp_numeric(rows, original_s, src_s, 'pitch',  0.0):.2f}",
            "terrain":   f"{interp_numeric(rows, original_s, src_s, 'terrain',0.0):.1f}",
        }
        if has_lane:
            # lane_* 만 호길이 진행률이 아니라 '수선 투영'으로 원본과 대응시킨다 —
            # δ(횡변위)를 얻으려면 기하학적 최근접점이 필요하기 때문. 기존 컬럼의
            # src_s 대응은 그대로 둬서 출력 변화를 lane_* 로만 국한한다.
            lane_cols, d_abs, ok = build_lane_columns(rows, original_s, original_xy, p, s_hint=src_s)
            row_out.update(lane_cols)
            if has_lane_opt:
                row_out.update(build_lane_opt_columns(rows, original_s, src_s, ok))
            if ok:
                lane_ok_count += 1
                lane_delta_sum += d_abs
                lane_delta_max = max(lane_delta_max, d_abs)
        out_rows.append(row_out)

    os.makedirs(os.path.dirname(os.path.abspath(output_csv)), exist_ok=True)
    with open(output_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(out_rows)

    report_path = os.path.splitext(output_csv)[0] + "_report.txt"
    with open(report_path, "w", encoding="utf-8") as f:
        f.write("Smart route remodel report\n")
        f.write("==========================\n")
        f.write(f"input_csv: {os.path.abspath(os.path.expanduser(input_csv))}\n")
        f.write(f"output_csv: {os.path.abspath(output_csv)}\n")
        f.write(f"original_points: {len(rows_raw)}\n")
        f.write(f"dedup_points: {len(rows)}\n")
        f.write(f"remodeled_points: {len(remodeled_xy)}\n")
        f.write(f"original_length_m: {original_len:.3f}\n")
        f.write(f"remodeled_length_m: {remodeled_len:.3f}\n")
        f.write(f"spacing_m: {spacing:.3f}\n")
        f.write(f"input_epsilon_m: {epsilon:.3f}\n")
        f.write(f"rdp_epsilon_m: {stats.get('rdp_epsilon_m', 0.0):.3f}\n")
        f.write(f"skeleton_points: {int(stats.get('skeleton_points', 0))}\n")
        f.write(f"straight_segments: {int(stats.get('straight_segments', 0))}\n")
        f.write(f"rounded_corners: {int(stats.get('corners', 0))}\n")
        f.write(f"vehicle_width_m: {stats.get('vehicle_width_m', 0.0):.3f}\n")
        f.write(f"min_corner_radius_with_width_m: {stats.get('min_radius_with_width_m', 0.0):.3f}\n")
        f.write("straight_model: exact skeleton line\n")
        f.write("corner_model: cubic Bezier fillet\n")
        f.write("corner_cutting: limited conservative fillet\n")
        if has_lane:
            n_out = max(1, len(remodeled_xy))
            f.write(f"lane_columns: present\n")
            f.write(f"lane_valid_points: {lane_ok_count}/{n_out} "
                    f"({100.0*lane_ok_count/n_out:.1f}%)\n")
            f.write(f"lane_reproject_mean_shift_m: "
                    f"{(lane_delta_sum/lane_ok_count if lane_ok_count else 0.0):.4f}\n")
            f.write(f"lane_reproject_max_shift_m: {lane_delta_max:.4f}\n")
            f.write("lane_model: cte reprojected onto remodeled line (cte += delta, right+)\n")
        else:
            f.write("lane_columns: absent (카메라 미기록 맵 — 출력은 구버전과 동일)\n")

    print("✅ 스마트 경로 리모델링 완료")
    print(f"입력 파일: {os.path.abspath(os.path.expanduser(input_csv))}")
    print(f"출력 파일: {os.path.abspath(output_csv)}")
    print(f"리포트: {os.path.abspath(report_path)}")
    print(f"포인트 수: {len(rows_raw)} → {len(remodeled_xy)}")
    print(f"경로 길이: {original_len:.2f}m → {remodeled_len:.2f}m")
    print(f"골격점: {int(stats.get('skeleton_points', 0))}, 직선: {int(stats.get('straight_segments', 0))}, 코너: {int(stats.get('corners', 0))}")
    print(f"간격: {spacing:.2f}m, RDP eps: {stats.get('rdp_epsilon_m', 0.0):.2f}m")
    if has_lane:
        n_out = max(1, len(remodeled_xy))
        print(f"차선: 유효 {lane_ok_count}/{n_out} ({100.0*lane_ok_count/n_out:.1f}%), "
              f"재투영 이동 평균 {(lane_delta_sum/lane_ok_count if lane_ok_count else 0.0):.3f}m "
              f"/ 최대 {lane_delta_max:.3f}m")
    return output_csv


def _remodel_route_segmented(
    input_csv: str,
    rows_raw: List[Dict[str, str]],
    dirs: List[int],
    output_csv: Optional[str],
    spacing: float,
    epsilon: float,
    smooth_iter: int,
) -> str:
    """전진/후진이 섞인 경로: 방향이 같은 연속 구간(run)별로 리모델링 후 이어붙임.

    반전점(cusp)을 필렛으로 깎지 않고 보존한다. 각 출력점의 direction은 해당 run의
    방향으로 강제하며, heading은 마지막에 전체 경로 기준으로 다시 계산한다.
    """
    lat0 = float(rows_raw[0]["latitude"])
    lon0 = float(rows_raw[0]["longitude"])

    # 방향이 같은 연속 구간으로 분할
    runs = []  # (run_rows, run_dir)
    start = 0
    for i in range(1, len(rows_raw) + 1):
        if i == len(rows_raw) or dirs[i] != dirs[start]:
            runs.append((rows_raw[start:i], dirs[start]))
            start = i

    out_rows: List[Dict[str, str]] = []
    remodeled_all: List[Point] = []
    total_corners = 0
    total_straight = 0
    total_skeleton = 0
    rdp_eps_used = 0.0
    original_len_sum = 0.0
    stats: Dict[str, float] = {}   # run이 전부 <3점이면 루프에서 안 채워질 수 있어 기본값 필요
    # [카메라 융합] 입력에 lane_* 이 있을 때만 출력에 붙인다 → 구버전 맵의 출력은 바이트 동일.
    has_lane = has_lane_columns(rows_raw)
    has_lane_opt = has_lane and has_lane_opt_columns(rows_raw)
    lane_ok_count = 0
    lane_delta_sum = 0.0
    lane_delta_max = 0.0

    for run_rows, run_dir in runs:
        # 너무 짧은 run(1~2점)은 깎지 않고 원본 점을 그대로 통과시킨다.
        if len(run_rows) < 3:
            for r in run_rows:
                x, y = latlon_to_xy(float(r["latitude"]), float(r["longitude"]), lat0, lon0)
                remodeled_all.append((x, y))
                row_out = {
                    "latitude":  f"{xy_to_latlon(x, y, lat0, lon0)[0]:.8f}",
                    "longitude": f"{xy_to_latlon(x, y, lat0, lon0)[1]:.8f}",
                    "heading":   "0.00",
                    "speed":     f"{float(r.get('speed', 0.0) or 0.0):.4f}" if _is_num(r.get('speed')) else "0.0000",
                    "steer":     f"{float(r.get('steer', 0.0) or 0.0):.2f}" if _is_num(r.get('steer')) else "0.00",
                    "direction": str(run_dir),
                    "pitch":     f"{float(r.get('pitch', 0.0) or 0.0):.2f}" if _is_num(r.get('pitch')) else "0.00",
                    "terrain":   f"{float(r.get('terrain', 0.0) or 0.0):.1f}" if _is_num(r.get('terrain')) else "0.0",
                }
                if has_lane:
                    # 점을 안 옮겼으므로 δ=0 → 원본 lane_* 를 그대로 통과.
                    conf = _fnum(r.get("lane_conf"), 0.0)
                    if conf >= LANE_CONF_MIN:
                        row_out.update({
                            "lane_cte":   f"{_fnum(r.get('lane_cte')):.4f}",
                            "lane_conf":  f"{conf:.3f}",
                            "lane_flags": f"{int(_fnum(r.get('lane_flags')))}",
                        })
                        if has_lane_opt:
                            row_out.update({
                                "lane_theta": f"{_fnum(r.get('lane_theta')):.2f}",
                                "lane_curv":  f"{_fnum(r.get('lane_curv')):.6f}",
                            })
                        lane_ok_count += 1
                    else:
                        row_out.update({"lane_cte": "0.0000", "lane_conf": "0.000", "lane_flags": "0"})
                        if has_lane_opt:
                            row_out.update({"lane_theta": "0.00", "lane_curv": "0.000000"})
                out_rows.append(row_out)
            continue

        xy_raw = [latlon_to_xy(float(r["latitude"]), float(r["longitude"]), lat0, lon0) for r in run_rows]
        rows_d, orig_xy = remove_duplicate_points_with_rows(run_rows, xy_raw, min_dist_m=0.05)
        orig_s = cumulative_distance(orig_xy)
        orig_len = orig_s[-1] if orig_s else 0.0
        original_len_sum += orig_len

        rem_xy, stats = build_smart_path(orig_xy, spacing=spacing, epsilon=epsilon, smooth_iter=smooth_iter)
        rem_s = cumulative_distance(rem_xy)
        rem_len = rem_s[-1] if rem_s else 0.0

        total_corners += int(stats.get("corners", 0))
        total_straight += int(stats.get("straight_segments", 0))
        total_skeleton += int(stats.get("skeleton_points", 0))
        rdp_eps_used = stats.get("rdp_epsilon_m", rdp_eps_used)

        for j, p in enumerate(rem_xy):
            progress = 0.0 if rem_len <= 1e-9 else rem_s[j] / rem_len
            src_s = progress * orig_len
            lat, lon = xy_to_latlon(p[0], p[1], lat0, lon0)
            remodeled_all.append(p)
            row_out = {
                "latitude":  f"{lat:.8f}",
                "longitude": f"{lon:.8f}",
                "heading":   "0.00",  # 아래에서 전체 경로 기준으로 채움
                "speed":     f"{interp_numeric(rows_d, orig_s, src_s, 'speed',  0.0):.4f}",
                "steer":     f"{interp_numeric(rows_d, orig_s, src_s, 'steer',  0.0):.2f}",
                "direction": str(run_dir),  # ★ run 방향 강제(라벨 뭉개짐 방지)
                "pitch":     f"{interp_numeric(rows_d, orig_s, src_s, 'pitch',  0.0):.2f}",
                "terrain":   f"{interp_numeric(rows_d, orig_s, src_s, 'terrain',0.0):.1f}",
            }
            if has_lane:
                # 투영은 반드시 '해당 run' 의 원본 폴리라인에만 — 전/후진이 겹쳐 지나는
                # 구간에서 반대 방향 run 에 잘못 붙는 것을 막는다.
                lane_cols, d_abs, ok = build_lane_columns(rows_d, orig_s, orig_xy, p, s_hint=src_s)
                row_out.update(lane_cols)
                if has_lane_opt:
                    row_out.update(build_lane_opt_columns(rows_d, orig_s, src_s, ok))
                if ok:
                    lane_ok_count += 1
                    lane_delta_sum += d_abs
                    lane_delta_max = max(lane_delta_max, d_abs)
            out_rows.append(row_out)

    # heading은 이어붙인 전체 경로 기준으로 계산(기존과 동일한 방식)
    for i in range(len(out_rows)):
        out_rows[i]["heading"] = f"{heading_deg(remodeled_all, i):.2f}"

    remodeled_s = cumulative_distance(remodeled_all)
    remodeled_len = remodeled_s[-1] if remodeled_s else 0.0

    if output_csv is None:
        root, _ = os.path.splitext(os.path.expanduser(input_csv))
        output_csv = root + "_remodeled.csv"

    fieldnames = ["latitude", "longitude", "heading", "speed", "steer", "direction", "pitch", "terrain"]
    if has_lane:
        fieldnames = fieldnames + LANE_FIELDS
        if has_lane_opt:
            fieldnames = fieldnames + LANE_FIELDS_OPT
    os.makedirs(os.path.dirname(os.path.abspath(output_csv)), exist_ok=True)
    with open(output_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(out_rows)

    report_path = os.path.splitext(output_csv)[0] + "_report.txt"
    with open(report_path, "w", encoding="utf-8") as f:
        f.write("Smart route remodel report [segmented by direction]\n")
        f.write("===================================================\n")
        f.write(f"input_csv: {os.path.abspath(os.path.expanduser(input_csv))}\n")
        f.write(f"output_csv: {os.path.abspath(output_csv)}\n")
        f.write(f"original_points: {len(rows_raw)}\n")
        f.write(f"direction_runs: {len(runs)}\n")
        f.write(f"remodeled_points: {len(remodeled_all)}\n")
        f.write(f"original_length_m(sum): {original_len_sum:.3f}\n")
        f.write(f"remodeled_length_m: {remodeled_len:.3f}\n")
        f.write(f"spacing_m: {spacing:.3f}\n")
        f.write(f"input_epsilon_m: {epsilon:.3f}\n")
        f.write(f"rdp_epsilon_m: {rdp_eps_used:.3f}\n")
        f.write(f"skeleton_points(sum): {total_skeleton}\n")
        f.write(f"straight_segments(sum): {total_straight}\n")
        f.write(f"rounded_corners(sum): {total_corners}\n")
        f.write(f"vehicle_width_m: {stats.get('vehicle_width_m', 0.0):.3f}\n")
        f.write(f"min_corner_radius_with_width_m: {stats.get('min_radius_with_width_m', 0.0):.3f}\n")
        f.write("straight_model: exact skeleton line\n")
        f.write("corner_model: cubic Bezier fillet (per direction-run)\n")
        f.write("cusp_handling: forward<->reverse reversal preserved (not filleted)\n")
        if has_lane:
            n_out = max(1, len(remodeled_all))
            f.write("lane_columns: present\n")
            f.write(f"lane_valid_points: {lane_ok_count}/{n_out} "
                    f"({100.0*lane_ok_count/n_out:.1f}%)\n")
            f.write(f"lane_reproject_mean_shift_m: "
                    f"{(lane_delta_sum/lane_ok_count if lane_ok_count else 0.0):.4f}\n")
            f.write(f"lane_reproject_max_shift_m: {lane_delta_max:.4f}\n")
            f.write("lane_model: cte reprojected onto remodeled line (cte += delta, right+)\n")
        else:
            f.write("lane_columns: absent (카메라 미기록 맵 — 출력은 구버전과 동일)\n")

    print("✅ 스마트 경로 리모델링 완료 [방향 구간 분리]")
    print(f"입력 파일: {os.path.abspath(os.path.expanduser(input_csv))}")
    print(f"출력 파일: {os.path.abspath(output_csv)}")
    print(f"리포트: {os.path.abspath(report_path)}")
    print(f"방향 구간(run): {len(runs)}개")
    print(f"포인트 수: {len(rows_raw)} → {len(remodeled_all)}")
    print(f"경로 길이(합): {original_len_sum:.2f}m → {remodeled_len:.2f}m")
    print(f"골격점합: {total_skeleton}, 직선합: {total_straight}, 코너합: {total_corners}")
    return output_csv


def _is_num(v) -> bool:
    try:
        float(v)
        return True
    except (TypeError, ValueError):
        return False


# ═══════════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(description="GPS waypoint CSV 스마트 리모델링 도구")
    parser.add_argument("--input", "-i", help="입력 route CSV 경로")
    parser.add_argument("--output", "-o", help="출력 CSV 경로")
    parser.add_argument("--latest", action="store_true", help="~/white_ws/white_ws/gps_data 안의 최신 route_*.csv 자동 선택")
    parser.add_argument("--data-dir", default="~/white_ws/white_ws/gps_data", help="route CSV 폴더")
    parser.add_argument("--spacing", type=float, default=0.25, help="출력 waypoint 간격[m]")
    parser.add_argument("--epsilon", type=float, default=0.10, help="직선/코너 골격 추출 허용오차[m]")
    parser.add_argument("--smooth", type=int, default=1, help="최종 heading 완화 반복 횟수")
    args = parser.parse_args()

    input_csv = args.input
    if args.latest or not input_csv:
        input_csv = find_latest_route(args.data_dir)
        if input_csv is None:
            raise FileNotFoundError(f"{os.path.expanduser(args.data_dir)} 안에 route_*.csv 파일이 없습니다.")

    remodel_route(input_csv, output_csv=args.output, spacing=args.spacing, epsilon=args.epsilon, smooth_iter=args.smooth)


if __name__ == "__main__":
    main()