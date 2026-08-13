#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""paths.py — ★저장 위치의 단일 소유자★

경로(맵) CSV 와 주행 기록 CSV 가 어디에 쌓이는지를 여기 한 곳에서 정한다.
mapping·driving·prompt·record 가 같은 폴더를 봐야 하는데, 각자 자기 상수를
들고 있으면 반드시 어긋난다(구 white 에서 prompt.py 주석이 "mapping.py 가
저장하는 폴더와 동일해야 한다"고 경고하고 있던 바로 그 문제다).

우선순위 (양쪽 함수 공통):
  1) 노드 파라미터로 준 명시 경로
  2) 환경변수 (WHITE1_DATA_DIR / WHITE1_RECORD_DIR)
  3) ★소스 트리★ <...>/src/white1/{gps_data,ros2bag}
     — colcon 을 --symlink-install 로 빌드하면 이 파일이 소스를 가리키는
       심볼릭 링크라 realpath 로 소스 위치를 되찾을 수 있다.
  4) 못 찾으면 ~/white1/{gps_data,ros2bag}
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
