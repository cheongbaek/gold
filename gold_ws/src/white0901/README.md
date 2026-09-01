# white0901 — white1 파이썬 소스 백업 (2026-09-01)

GPS 경로추종(white1) 과 라이다 주행(lidar) 을 **하나의 주행 계통으로 통합**하기
전에, 손대기 직전의 white1 파이썬 소스를 그대로 떠 둔 사본이다.

## 무엇이 들어 있나
- `white1/*.py` — 패키지 노드 소스 13 개 (driving·gps·hud·traffic_light·mapping …)
- `launch/*.py` — 런치 4 개 (one_launch·master·joy·hud)
- `setup.py`   — 당시의 entry_points (노드 이름 ↔ 모듈 대응을 되살릴 때 필요)

## ROS2 패키지가 아니다
`package.xml` 도 `resource/` 도 없다. **보관용 사본**이라 빌드 대상이 아니며,
`COLCON_IGNORE` 를 두어 `colcon build` 가 이 폴더를 건너뛰게 했다.
되살릴 때는 파일을 `white1/` 의 같은 자리로 복사하면 된다.

## 기준점
- 백업 시점 HEAD : 083bd7e (feat(white1): 신호등 디버그 창을 계기판으로 개편)
- 작업트리에 커밋 안 된 수정이 있는 상태로 떴다 — 즉 **HEAD 가 아니라
  "그때 실제로 돌던 파일"** 이다. 이 폴더의 값이 곧 정본이다.
