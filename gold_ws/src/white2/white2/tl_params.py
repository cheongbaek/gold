# -*- coding: utf-8 -*-
"""tl_params.py ― 신호등 노드 파라미터 중 ★런치 인자로 여는 것의 기본값★ (단일 소유자) [2026-10-18]

traffic_light.py 가 이 값으로 파라미터를 선언하고, camera_launch.py 가 같은 값으로 런치 인자를 연다 —
노드 기본값과 런치 기본값이 어긋날 수 없다. ROS·cv2 를 import 하지 않는다(런치에서 가볍게 읽히도록).

    ros2 launch white2 one_launch.py tl_b_arrow_frac:=0.25      # 덮는 법 (white1 도 같다)

근거·쓰는 법은 TL_TUNING.md 2절 'B' — 값을 바꾸면 cam-test 로 #400 런을 다시 돌린다.
"""

# 구간 'B'(#400) 좌회전 화살표 판독 — traffic_light._decision_boxes
B_ARROW = {
    'tl_b_arrow_frac':  0.30,   # 셋째 칸 녹색 비율이 이 이상이면 화살표 켜짐
    'tl_b_arrow_min_h': 20.0,   # 이보다 작은 차량 박스는 화살표를 안 읽는다(모름 = 정지) [px]
    'tl_b_veh_min_ar':  1.75,   # 화살표를 읽을 박스의 최소 가로세로비 — 버스 박스(1.47~1.80)를 거른다
    'tl_b_clip_hold':   True,   # 차량 박스가 ROI·화면 가장자리에 닿으면 판독하지 않고 직전 판독을 유지
}
