"""white2 — 투트랙의 다른 한쪽 (CLAUDE.md 3.1) [2026-10-21 white1 공용화]

★이 패키지에 남은 것은 white1 과 다른 셋뿐이다★ — 신호 판단이 갈리는 곳:
    driving.py   T·A·B 태그 · traffic_light 가 /tl_permit 구간에서 2단 (white1 은 T1~T5 타이머 · 정지선 1 m 앞)
    prompt.py    traffic_timer 없음 · 고정 코스 설명
    record.py    /traffic_timer 열 없음 · /tl_zone 출처
나머지 노드(gps·iahrs·speed·mapping·hud·traffic_light)와 모듈(paths·ports·camera_launch·camera_model·
tl_params)은 ★white1 의 것을 그대로 쓴다★ — white2 의 복사본은 지웠다(머리말·이름만 달랐다).
braketest 는 white1 에만 있다.

★폴더는 white2 의 것이다★ — 이 파일이 WHITE_STACK=white2 를 건다. white1.paths 가 그 값을 보고
src/white2/{gps_data,ros2bag,video,sound} 를 고른다(white1/paths.py '스택' 절). white2 의 어떤 모듈을
import 해도 이 파일이 먼저 돌므로, white2 의 런치·prompt·driving·record·hud 가 모두 white2 폴더를 본다.
런치가 띄우는 white1 노드는 이 프로세스의 환경을 물려받는다(one_launch 의 NODE_ENV 에도 적었다).
"""

import os

STACK = 'white2'
STACK_ENV = 'WHITE_STACK'          # = white1.paths.STACK_ENV (여기서 white1 을 import 하지 않으려고 적었다)
os.environ[STACK_ENV] = STACK


def hud():
    """`ros2 run white2 hud` — white1 의 hud 를 white2 폴더로 (setup.py entry point)."""
    from white1 import hud as _hud
    return _hud.main()
