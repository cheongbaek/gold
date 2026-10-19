# -*- coding: utf-8 -*-
"""tl_params.py ― 신호등 노드 파라미터 중 ★런치 인자로 여는 것의 기본값★ (단일 소유자) [2026-10-18]

traffic_light.py 가 이 값으로 파라미터를 선언하고, camera_launch.py 가 같은 값으로 런치 인자를 연다 —
노드 기본값과 런치 기본값이 어긋날 수 없다. ROS·cv2 를 import 하지 않는다(런치에서 가볍게 읽히도록).

    ros2 launch white1 one_launch.py tl_b_arrow_frac:=0.25      # 덮는 법 (white1 도 같다)
    ros2 launch white1 one_launch.py tl_zone_red_min_height_t:=9 tl_near_release_ratio_t:=0.7   # T 를 종전 값으로

근거·쓰는 법은 TL_TUNING.md 2절 'B' — 값을 바꾸면 cam-test 로 #400 런을 다시 돌린다.
"""

# 구간 'B'(#400) 좌회전 화살표 판독 — traffic_light._decision_boxes
B_ARROW = {
    'tl_b_arrow_frac':  0.30,   # 셋째 칸 녹색 비율이 이 이상이면 화살표 켜짐
    'tl_b_arrow_min_h': 20.0,   # 이보다 작은 차량 박스는 화살표를 안 읽는다(모름 = 정지) [px]
    'tl_b_veh_min_ar':  1.75,   # 화살표를 읽을 박스의 최소 가로세로비 — 버스 박스(1.47~1.80)를 거른다
    'tl_b_clip_hold':   True,   # 차량 박스가 ROI·화면 가장자리에 닿으면 판독하지 않고 직전 판독을 유지
}

# 구간별 RED 근접 문턱과 정지 중 배율 — traffic_light._zone_gate [2026-10-18 제안 3, 사용자 승인 2026-10-19]
#   문턱 [px] : 박스 높이가 이 이상이면 RED(정지 대상), 밑이면 RED_FAR(관측만). 정지 중(2단을 문 동안)에는 × 배율.
#   T 12 · 0.75 — 서기 전 12px : 체결 순간 내 빨강 20~27px(13건) / 다음 교차로 빨강 4~7px → 위아래 여유 8 · 5 px
#                 서 있는 동안 9px : 다음 교차로 빨강(4~7px, 문제 3 은 6~7)은 거르고 건너편 같은 현시 등기구(8~12px)는 붙잡는다 —
#                 ×1.0(12px)이면 내 신호가 화면 위로 빠진 정지에서 건너편 11px 이 RED_FAR 가 되어 빨간불에 풀렸다(140401 288~293).
#                 ⚠️ 여유는 2px(7 ↔ 9) · −1px(8px 건너편 등기구는 못 붙잡는다) — 두 분포가 붙어 있다
#   A 9 · 0.7  — #1100 은 정지선 앞에서도 내 신호가 9~10px(건너편 등기구만 보인다) — 올리면 빨간불에 안 선다
#   B 9 · 0.7  — #400 은 판단 박스가 차량 박스 하나(_decision_boxes)라 남의 신호와 무관 — 종전 값 그대로
#   0 이하의 문턱 = 그 구간도 구간 밖 문턱(tl_red_stop_min_height 25px)과 배율(tl_near_release_ratio)을 쓴다.
ZONE_GATE = {
    'tl_zone_red_min_height_t': 12.0,
    'tl_zone_red_min_height_a': 9.0,
    'tl_zone_red_min_height_b': 9.0,
    'tl_near_release_ratio_t':  0.75,
    'tl_near_release_ratio_a':  0.7,
    'tl_near_release_ratio_b':  0.7,
}
