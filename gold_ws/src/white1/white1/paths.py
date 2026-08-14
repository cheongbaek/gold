#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""paths.py — ★저장 위치의 단일 소유자★

경로(맵) CSV·주행 기록 CSV·음성 안내 음원이 어디에 있는지를 여기 한 곳에서 정한다.
mapping·driving·prompt·record 가 같은 폴더를 봐야 하는데, 각자 자기 상수를
들고 있으면 반드시 어긋난다(구 white 에서 prompt.py 주석이 "mapping.py 가
저장하는 폴더와 동일해야 한다"고 경고하고 있던 바로 그 문제다).

우선순위 (세 함수 공통):
  1) 노드 파라미터로 준 명시 경로
  2) 환경변수 (WHITE1_DATA_DIR / WHITE1_RECORD_DIR / WHITE1_SOUND_DIR)
  3) ★소스 트리★ <...>/src/white1/{gps_data,ros2bag,sound}
     — colcon 을 --symlink-install 로 빌드하면 이 파일이 소스를 가리키는
       심볼릭 링크라 realpath 로 소스 위치를 되찾을 수 있다.
  4) 못 찾으면 ~/white1/{gps_data,ros2bag,sound}
     install/ 안에는 절대 쌓지 않는다 — 재빌드하면 날아간다.
"""

import os


def _package_root():
    """설치본이 아닌 '소스 트리의 white1 패키지 루트'. 못 찾으면 None."""
    here = os.path.dirname(os.path.realpath(__file__))   # .../src/white1/white1
    root = os.path.dirname(here)                         # .../src/white1
    if os.path.isfile(os.path.join(root, 'package.xml')):
        return root
    return None


def _resolve(explicit, env_key, subdir):
    if explicit:
        return os.path.abspath(os.path.expanduser(explicit))
    env = os.environ.get(env_key, '').strip()
    if env:
        return os.path.abspath(os.path.expanduser(env))
    root = _package_root()
    if root:
        return os.path.join(root, subdir)
    return os.path.expanduser(os.path.join('~/white1', subdir))


def data_dir(explicit: str = "") -> str:
    """경로(맵) CSV 폴더 — mapping 이 쓰고 driving·prompt 가 읽는다."""
    return _resolve(explicit, 'WHITE1_DATA_DIR', 'gps_data')


def record_dir(explicit: str = "") -> str:
    """주행 기록 CSV 폴더 — record 가 쓴다."""
    return _resolve(explicit, 'WHITE1_RECORD_DIR', 'ros2bag')


def sound_dir(explicit: str = "") -> str:
    """음성 안내 음원(mp3) 폴더 — nxde 의 sound 노드가 읽는다.

    ★[2026-08-14] 음원을 nxde/sound → white1/sound 로 옮겼다★
    안내 문구는 전부 이 스택의 사건(매핑·주행 시작, 도착, 경로이탈, E-STOP)이라
    ★음원의 주인은 white1★ 이 맞다. nxde 는 '보드와 말하는 패키지'로 두고,
    재생기(sound 노드)만 빌려 쓴다 — one_launch.py 가 이 경로를 sound_dir
    파라미터로 넘겨 준다. nxde 자체 기본값(<nxde>/sound)은 그대로 두었으므로
    `ros2 run nxde sound` 를 단독으로 띄우면 여전히 그쪽을 본다(그때는 비어 있다).
    """
    return _resolve(explicit, 'WHITE1_SOUND_DIR', 'sound')
