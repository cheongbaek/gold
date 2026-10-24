"""white3 — white1 의 10/17 주행 코드 + 현재 카메라 코드 [2026-10-24 사용자 지시] (CLAUDE.md 3.1)

★이 패키지에 있는 것은 white1 의 10/17 판(96408b9)이다★ — 그 뒤(10/19~10/24)에 white1 에 들어간
주행 변경(T5 타이머 · L 진입 0·1·2단 · Y·X 구간 · RTK Fixed 대기 · gps 정차 고정)이 없다:
    driving.py        10/17 판 + C(yaw_damp_k) 기본 0.0 (10/19 ab7206d 의 driving 변경분)
    gps.py            10/17 판
    traffic_timer.py  10/17 판 (T5 는 카메라 · /traffic_timer 10칸)
    record.py         10/17 판
    prompt.py         10/17 판 (white1 현재 판과 본문이 같다 — import 만 white3)
나머지 노드(iahrs·speed·mapping·hud·traffic_light)와 모듈(paths·ports·camera_launch·camera_model·
tl_params)은 ★white1 의 현재 것을 그대로 쓴다★ — 카메라 인지는 현재 판이다.

★폴더는 white3 의 것이다★ — 이 파일이 WHITE_STACK=white3 를 건다. white1.paths 가 그 값을 보고
src/white3/{gps_data,ros2bag,video,sound} 를 고른다(white1/paths.py '스택' 절). white3 의 어떤 모듈을
import 해도 이 파일이 먼저 돈다. 런치가 띄우는 white1 노드는 NODE_ENV 로 같은 값을 받는다.
"""

import os

STACK = 'white3'
STACK_ENV = 'WHITE_STACK'          # = white1.paths.STACK_ENV (여기서 white1 을 import 하지 않으려고 적었다)
os.environ[STACK_ENV] = STACK


def hud():
    """`ros2 run white3 hud` — white1 의 hud 를 white3 폴더로 (setup.py entry point)."""
    from white1 import hud as _hud
    return _hud.main()
