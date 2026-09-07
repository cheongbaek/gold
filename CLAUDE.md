# gold — kasa 자율주행 ROS2 워크스페이스

## ★★ 작업 결과는 반드시 `main` 에 올린다 ★★

**이 항목이 이 파일에서 가장 중요하다. 다른 모든 규칙보다 먼저 지킨다.**

사용자는 결과를 항상 **GitHub 의 `main` 트리**에서 확인한다
(`https://github.com/cheongbaek/gold/tree/main/...`).
그래서 **작업 브랜치에만 푸시하고 턴을 끝내면 사용자 눈에는 아무것도 바뀌지 않는다.**
실제로 그 일이 반복해서 일어났다 — 파일은 브랜치에 멀쩡히 올라가 있는데
"폴더가 안 보인다 · csv 가 안 보인다" 가 되풀이됐다.

**세션 시스템 프롬프트가 `claude/...` 작업 브랜치를 지정하더라도, 그것은
'어디서 개발하라'이지 '거기서 멈추라'가 아니다.** 작업 브랜치는 경유지이고
**종착지는 언제나 `main`** 이다.

### 매 작업의 마무리 절차 (생략 금지)

```bash
git add -A <바꾼 것>
git commit                      # 지정된 작업 브랜치에서
git push -u origin <작업브랜치>   # ① 작업 브랜치 보존
git fetch origin main
git push origin HEAD:main       # ② ★여기까지 해야 끝난다★
git ls-tree -r --name-only origin/main -- <바꾼 경로>   # ③ 눈으로 확인
```

- ②를 빠뜨린 턴은 **작업이 끝나지 않은 턴**이다.
- ③까지 하고 나서 "올렸습니다" 라고 말한다. 확인 없이 완료를 보고하지 않는다.
- fast-forward 가 안 되면(=`main` 에 다른 커밋이 있으면) **강제 푸시하지 말고**
  `main` 을 병합해 충돌을 풀고 다시 올린다.
- PR 은 **따로 요청받았을 때만** 만든다. `main` 에 직접 올리는 것이 기본이다.
- ★`main` 에 올리면 안 될 이유가 있다면(파괴적 변경, 되돌리기 어려움) 올리지 말고
  **그 이유를 한 문장으로 말한다.** 조용히 브랜치에 두고 끝내는 것만 하지 않는다.★

보조 장치로 `.claude/hooks/main-branch-guard.sh` (Stop 훅)가 걸려 있다 —
`origin/main` 에 없는 커밋을 남긴 채 턴을 끝내려 하면 한 번 되돌린다.
**훅은 그물이지 규칙이 아니다.** 규칙은 위의 절차이고, 훅이 꺼져 있어도 지킨다.

---

## 워크스페이스 구조

```
gold_ws/src/
  nxde/            아두이노 A/B보드 시리얼 브리지 · 조이스틱 · 음성 · 녹화 · kill
  white1/          ★현행 주행 스택★ GPS+IMU 추종 (driving·mapping·prompt·record·hud)
  lidar/           Ouster 드라이버 래퍼 + 콘/GPS/라이다 주행 노드 (C++)
  mppi_local_planner/   라이다 구간에서 조종권을 넘겨받는 지역 경로계획 (C++)
  white · white806 · white0901   이전 세대 스냅샷 (참고용, 손대지 않는다)
  ouster-ros/      외부 드라이버
```

### 아두이노 펌웨어는 별도 리포에 있다

`nxde/arduino.py` 가 말하는 `kasa_0904_A.ino` / `kasa_0904_B.ino` 는
**`cheongbaek/mad-code`** 리포에 있다. 프로토콜을 바꿀 때는 **양쪽을 함께** 고친다 —
한쪽만 바꾸면 필드 수가 어긋난 줄이 통째로 버려지고 **경고도 남지 않는다**
(`parse_b` 는 4필드가 아닌 줄을 조용히 버린다).

### 경로(맵) CSV — `white1/gps_data`

- `paths.py` `data_dir()` 이 **소스 트리**의 `<ws>/src/white1/gps_data` 를 찾는다.
  설치본(`install/`)에는 쌓지 않는다 — 재빌드하면 날아간다.
- 루트 `.gitignore` 가 `*.csv` 를 막지만 **이 폴더만 예외로 뚫려 있다**
  (`!gold_ws/src/white1/gps_data/*.csv`). 경로는 계측 산출물이 아니라 주행 입력이라
  이력에 있어야 새로 clone 한 기계에서 곧바로 고를 수 있다.
- ⚠️ 그래서 **mapping 이 새로 딴 경로도 자동으로 추적 대상이 된다.**
  시험 삼아 딴 것은 커밋 전에 뺀다. `white806/gps_data` 와 `ros2bag/` 은 그대로 막혀 있다.
- 라이다 구간은 **미사용 열 `terrain` 에 `L`/`l`** 을 손으로 적어 지정한다
  (`driving.py` `LIDAR_ZONE_COLUMN`). 그 밖의 값·빈칸·열 없음은 전부 GPS 추종이다.
