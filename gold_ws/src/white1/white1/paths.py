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
  4) ★sound 만★ 설치본 share/white1/sound (소스트리가 아예 없는 배포본용)
  5) 못 찾으면 ~/white1/{gps_data,ros2bag,sound}
     install/ 안에는 절대 쌓지 않는다 — 재빌드하면 날아간다.
     (4 는 예외다 — 음원은 ★읽기 전용★ 이라 재빌드로 날아가도 잃을 것이 없다.)

════════════════════════════════════════════════════════════════════════════════
 ★★ [2026-09-04] 복사설치(copy-install)에서 3)이 통째로 끊겨 있었다 ★★
════════════════════════════════════════════════════════════════════════════════
  증상 : `ros2 launch white1 one_launch.py` 를 띄워도 ★음성 안내가 한 마디도
         나오지 않는다★. mp3 는 src/white1/sound 와 install/.../share/white1/sound
         양쪽에 멀쩡히 있고, 재생기(ffplay)도 깔려 있는데도 그렇다.

  원인 : _package_root() 가 `realpath(__file__)` 의 부모에서 package.xml 을 찾는
         방식 하나뿐이었다. 그것은 ★--symlink-install 로 빌드했을 때만★ 성립한다.
         `colcon build` 를 옵션 없이 돌리면 이 파일이 소스를 가리키는 링크가 아니라
             install/white1/lib/python3.10/site-packages/white1/paths.py
         에 놓인 ★복사본★ 이 되고, 그 부모에는 package.xml 이 없다. 그래서
         _package_root() 가 None → sound_dir() 이 ~/white1/sound (없는 폴더) 를
         돌려주고, sound 노드는 '음원 없음' 경고만 내며 조용히 돈다.
         one_launch.py 는 이 값을 sound_dir 파라미터로 넘기므로 런치 전체가 벙어리다.

  ★한 함수의 버그가 아니었다★ data_dir 도 같이 틀려서 매핑/주행이 ~/white1/gps_data
  (빈 폴더)를 보고 있었다 — prompt 의 경로 목록이 비어 보이던 것이 같은 원인이다.
  그 흔적이 실제로 남아 있었다: ~/white1/gps_data 가 빈 채로 만들어져 있었다.

  고침 : _package_root() 가 실패하면 ★설치 접두어에서 워크스페이스를 거슬러 올라가★
         <ws>/src/white1/package.xml 을 찾는다(_source_root_from_prefix).
         'install' 이라는 이름에 기대지 않고 조상마다 <조상>/src/white1 을 확인하므로
         분리설치·병합설치(--merge-install)·중첩 소스(src/<하위>/white1) 를 다 받는다.
         그래도 못 찾으면(소스트리 없이 install 만 배포한 기계) 음원은 4) 로 간다.

  ★이 파일을 고쳐도 복사설치본은 그대로다★ 반드시 다시 빌드해야 반영된다:
      colcon build --packages-select white1 nxde
  (--symlink-install 로 빌드해 두면 3) 이 첫 줄에서 바로 맞으므로 이 폴백이
   일을 하지 않는다 — 그것도 정상이다.)
"""

import os

PACKAGE_NAME = 'white1'


def _source_root_from_prefix(start):
    """설치본 경로에서 ★소스 트리의 white1 패키지 루트★ 를 되찾는다. 못 찾으면 None.

    start 예 : <ws>/install/white1/lib/python3.10/site-packages/white1
    조상을 하나씩 올라가며 <조상>/src/white1/package.xml 을 확인한다. <ws> 에서
    맞는다. 'install' 이라는 폴더명을 조건으로 쓰지 않는 이유는 병합설치나 다른
    접두어 이름(예: --install-base 로 바꾼 경우)에서도 같은 논리가 통해야 하기
    때문이다 — '옆에 src/white1 을 끼고 있는 조상' 이 곧 워크스페이스다.
    """
    d = start
    while True:
        parent = os.path.dirname(d)
        if parent == d:          # 루트('/')까지 올라갔다
            return None
        d = parent
        src = os.path.join(d, 'src')
        if not os.path.isdir(src):
            continue
        cand = os.path.join(src, PACKAGE_NAME)
        if os.path.isfile(os.path.join(cand, 'package.xml')):
            return cand
        # src/<하위폴더>/white1 로 한 단계 접혀 있는 경우도 받는다(vcs 로 받은 트리).
        try:
            entries = sorted(os.listdir(src))
        except OSError:
            continue
        for name in entries:
            cand = os.path.join(src, name, PACKAGE_NAME)
            if os.path.isfile(os.path.join(cand, 'package.xml')):
                return cand


def _package_root():
    """설치본이 아닌 '소스 트리의 white1 패키지 루트'. 못 찾으면 None.

    ① --symlink-install : 이 파일이 소스를 가리키는 링크라 realpath 한 번으로 끝난다.
    ② 복사설치           : ①이 끊긴다 — 워크스페이스를 거슬러 올라가 찾는다([0904] 절).
    """
    here = os.path.dirname(os.path.realpath(__file__))   # .../src/white1/white1
    root = os.path.dirname(here)                         # .../src/white1
    if os.path.isfile(os.path.join(root, 'package.xml')):
        return root
    return _source_root_from_prefix(here)


def _share_subdir(subdir):
    """설치본 share/white1/<subdir>. 없거나 ament 를 못 쓰면 None.

    ★읽기 전용 자원에만 쓴다★ (지금은 sound 뿐) — 여기에 쓰기를 하면 재빌드로 날아간다.
    ament_index 를 import 로 감싸는 이유는 이 모듈이 ROS 밖(테스트·도구)에서도
    읽히기 때문이다.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory(PACKAGE_NAME)
    except Exception:
        return None
    path = os.path.join(share, subdir)
    return path if os.path.isdir(path) else None


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
    """음성 안내 음원(mp3) 폴더 — nxde 의 sound 노드와 prompt 가 읽는다.

    ★[2026-08-14] 음원을 nxde/sound → white1/sound 로 옮겼다★
    안내 문구는 전부 이 스택의 사건(매핑·주행 시작, 도착, 경로이탈, E-STOP)이라
    ★음원의 주인은 white1★ 이 맞다. nxde 는 '보드와 말하는 패키지'로 두고,
    재생기(sound 노드)만 빌려 쓴다 — one_launch.py 가 이 경로를 sound_dir
    파라미터로 넘겨 준다.

    ★[2026-09-04] 소스트리를 못 찾으면 설치본 share 를 본다★ 다른 둘과 달리 음원은
    ★읽기 전용★ 이라 install/ 을 봐도 잃을 것이 없다. 이 한 줄이 '소스트리 없이
    install 만 복사해 간 기계' 에서 안내가 나오게 한다 — 그런 기계에서 ~/white1/sound
    를 돌려주면 사람이 mp3 를 손으로 옮겨 넣기 전까지 영원히 벙어리다.
    """
    if explicit:
        return os.path.abspath(os.path.expanduser(explicit))
    env = os.environ.get('WHITE1_SOUND_DIR', '').strip()
    if env:
        return os.path.abspath(os.path.expanduser(env))
    root = _package_root()
    if root:
        return os.path.join(root, 'sound')
    share = _share_subdir('sound')
    if share:
        return share
    return os.path.expanduser('~/white1/sound')
