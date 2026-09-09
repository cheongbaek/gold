# CLAUDE.md — gold (kasa 금색차 자율주행 스택)

이 저장소에서 작업할 때 제일 먼저 읽는 문서다. **차량 하드웨어 제원**, **단위·부호
규약**, **노드/토픽 계약**, **제어 알고리즘의 근거**, 그리고 **앞으로 고칠 것**을
한곳에 모았다. 값의 출처(파일:줄)를 같이 적었으니, 여기와 코드가 어긋나면
**코드가 정본이고 이 문서를 고친다.**

> **정본 지도** — 이 문서는 요약이고, 아래가 각 주제의 소유자다.
>
> | 주제 | 정본 |
> |---|---|
> | 아두이노 펌웨어 내부·배선·핀 배정 | `../mad-code/CLAUDE.md` (별도 저장소) |
> | B보드 ↔ ROS 시리얼 계약 | `gold_ws/src/white1/BOARD_B.md` |
> | 리니어 제동력 실측 | `gold_ws/src/white1/BRAKING.md` |
> | GPS 헤딩 초기화 실측 | `gold_ws/src/white1/GPS_HEADING.md` |
> | 정지선/신호등 시험 | `gold_ws/src/white1/STOPLINE_TEST.md` |
> | 변경 이력·실차 로그 근거 | `gold_ws/src/white1/CHANGELOG.md` (2082줄) |
> | 아두이노 계층 구조·안전장치 | `gold_ws/src/nxde/README.md` |
> | 라이다 패키지 | `gold_ws/src/lidar/README.md` |
> | 단위 환산 (C++ 쪽 단일 소유자) | `gold_ws/src/lidar/include/lidar/kasa_units.hpp` |
> | 튜닝 상수 (Python 쪽 단일 소유자) | `gold_ws/src/white1/white1/driving.py` 상단 상수절 |

---

## 0. 이 저장소를 다룰 때

### 0.1 ★★ 작업 결과는 반드시 `main` 에 올린다 ★★

사용자는 결과를 항상 **GitHub `main` 트리**에서 확인한다
(`https://github.com/cheongbaek/gold/tree/main/...`). **작업 브랜치에만 푸시하고
턴을 끝내면 사용자 눈에는 아무것도 바뀌지 않는다.** 작업 브랜치는 경유지이고
**종착지는 언제나 `main`** 이다.

```bash
git fetch origin main            # ★먼저★ — 여러 기계에서 직접 푸시된다
git add -A <바꾼 것>              # ★-A 를 경로 없이 쓰지 말 것★ (0.3절 CSV)
git commit
git push origin HEAD:main
git ls-tree -r --name-only origin/main -- <바꾼 경로>   # ★눈으로 확인★
```

- fast-forward 가 안 되면 **강제 푸시하지 말고** `main` 을 병합해 충돌을 풀고 올린다.
- PR 은 **따로 요청받았을 때만**. `main` 직접 푸시가 기본이다.
- `main` 에 올리면 안 될 이유가 있다면(파괴적·되돌리기 어려움) 올리지 말고
  **그 이유를 한 문장으로 말한다.** 조용히 브랜치에 두고 끝내지 않는다.

### 0.2 여러 기계가 같은 `main` 에 직접 푸시한다

**작업 전 `git fetch` 로 원격을 확인할 것.** 같은 패키지가 다른 경로로 이미
커밋돼 있거나, 다른 기계가 `CLAUDE.md`·경로 CSV 를 올려 둔 상태일 수 있다.
(실제로 2026-09-07 작업에서 원격이 2커밋 앞서 있었다 — 9/5 경로 CSV 21개 +
이 문서의 초판.)

### 0.3 `.gitignore` — `gps_data` 만 예외로 뚫려 있다

`*.csv` `*.mp3` `*.mp4` `*.png` 은 막혀 있지만
**`!gold_ws/src/white1/gps_data/*.csv` 예외가 있다** — 경로는 계측 산출물이 아니라
**주행 입력**이라 이력에 있어야 새로 clone 한 기계에서 곧바로 고를 수 있다.

> ⚠️ **그래서 `mapping` 이 새로 딴 경로도 자동으로 추적 대상이 된다.**
> **`git add -A` 를 경로 없이 쓰지 말 것.** 시험 삼아 딴 것은 커밋 전에 뺀다.
> `white806/gps_data` 와 `ros2bag/` 은 그대로 막혀 있다.

### 0.4 빌드 — ★`--symlink-install` 금지★

```bash
cd gold_ws
colcon build                                    # ★옵션 없이★
colcon build --packages-select mppi_local_planner lidar \
    --cmake-args -DCMAKE_BUILD_TYPE=Release     # C++ 만 다시
source install/setup.bash
```

`mppi_local_planner` 와 `lidar` 가 C++(ament_cmake) 라 `--symlink-install` 을 쓸 수
없다(`lidar/README.md` 151·304행에 '금지' 로 못 박혀 있다).
`paths.py` 의 소스트리 폴백은 [2026-09-04] 에 고쳐서 복사설치로도 맞게 돌지만,
**Python 소스를 고친 뒤에는 반드시 다시 빌드해야 한다** — 복사설치본은 스스로
갱신되지 않는다. `--symlink-install` 시절의 "고치고 노드만 재시작" 습관이 남아
있으면 **고친 적 없는 코드를 계속 돌리게 된다.**

빌드 대상 8개 : `white1` `nxde` `lidar` `mppi_local_planner` `ouster_ros`
`ouster_sensor_msgs` `white` `white806`
(`white0901` 은 `COLCON_IGNORE`. `white`·`white806` 은 이전 세대 스냅샷 — 손대지 않는다.)

`ouster-ros` 는 서브모듈이다 — 새로 clone 하면:
```bash
git submodule update --init gold_ws/src/ouster-ros
```

## 1. 차량 하드웨어 (kasa "금색차")

> 실측 출처가 갈리는 값은 표에 출처를 붙였다. **미실측 값은 ⚠️ 로 표시했다 — 그 값에
> 기대는 판단은 아직 근거가 없다.**

### 1.1 차체 · 기하

| 항목 | 값 | 출처 |
|---|---|---|
| **축거(휠베이스)** | **1.25 m** (1250 mm 실측) | `driving.py:338`, `kasa_units.hpp:53` |
| **윤거(track width)** | **1.10 m** (1100 mm) — ★2026-09-09 정정★ | `white/white/kasa_units.py:103` (실측 · 단일 소유자) |
| **차폭(전폭)** | ⚠️ **미실측.** AEB 코리도 반폭 0.8 m 로 가정 중(1톤급 전폭 1.5~1.6 m 추정) | `cone_lidar_node.cpp:130-132` |
| 도로휠 최대 조향각 | **±31.7°** (= pot ±40° ÷ 링키지비 1.26) | `kasa_units.hpp:63` |
| **최소회전반경** | **2.02 m** (= 1.25 / tan 31.7°) | `kasa_units.hpp:69` |
| 포화 없는 LFD 문턱 | **4.04 m** (= 2 × 최소회전반경) | `kasa_units.hpp:70` |
| 타이어 | **175/60R13**, 구름둘레 **1.6971 m** | `mad-code/CLAUDE.md` 1절 |

> **★휠베이스는 차를 바꿀 때 반드시 갈아야 하는 유일한 기하값이다★** 순수추종
> 조향각에 정비례한다. 1/5카(헤네스 브룬 T870)는 0.73 m 였고, 그대로 두면 모든
> 조향이 60% 로 축소된다.

> ### ⚠️ 윤거 0.65 는 이 문서의 오류였다 — 정정 [2026-09-09]
>
> 초판에 "윤거 0.65 m 미실측" 이라고 적었는데 **사실이 아니다.** 실제 값은
> **1.10 m** 이고 출처는 `white/white/kasa_units.py:103` 이다
> (*"축거·윤거 (kasa_ws master.py 의 디퍼렌셜 계산에 쓰인 실측값)"*).
> **0.65 는 `lidar` 패키지 두 곳에만 남은 1/5카 잔재**다.
>
> **전수 조사 — 윤거를 실제로 쓰는 곳과 값**
>
> | 쓰는 곳 | 값 | 무엇에 쓰나 |
> |---|---|---|
> | **`white1/driving.py`** | **— 쓰지 않는다** | **자전거 모델이라 축거만 쓴다** |
> | `mppi .../vehicle_model.hpp:32,98` | **1.10** ✅ | 충돌 판정 반폭 `half_w` |
> | `mppi .../node.cpp:434,452` | **1.10** ✅ | `robot_half_width = track/2 + 0.08 = 0.63`, ego_clear 반폭 |
> | `mppi/config/params.yaml:84` | **1.10** ✅ | 위 둘의 입력 |
> | `white1/hud.py:218` | 1.10 ✅ | 화면 차체 그림 폭(표시 전용) |
> | **`lidar/drive_lidar_node.cpp:187`** | **⚠️ 0.65** | 라바콘 코리도 폭·충돌 판정 |
> | **`lidar/config/drive_lidar.yaml:31`** | **⚠️ 0.65** | 위와 같은 값 |
>
> ### 알고리즘에 유의미한 영향이 있는가 — 결론
>
> **① `white1` GPS 추종 : 영향 0.** 순수추종·LFD·CTE·코너 감속·종점 접근이 전부
> **자전거 모델**이라 윤거가 식에 등장하지 않는다. `driving.py` 전체에 `track` 이
> 한 번도 나오지 않는다. **2026-09-08 경로이탈과도 무관하다**(원인은 IMU 축, 4.7절).
>
> **② `mppi` 라바콘 회피 : 맞는 값을 쓰고 있다.** 1.10 기준으로 반폭 0.55, 코스트맵
> lethal 반경 0.63 m. 여기가 틀렸다면 라바콘 사이 통과 판정이 바로 어긋났을 것이다.
>
> **③ `lidar/drive_lidar_node` : ⚠️ 실제로 위험한 방향으로 틀려 있다.**
> 0.65 를 쓰면 반폭이 **0.325 m** 로, 실제 0.55 m 보다 **0.225 m 작다.**
> 차를 실제보다 **좁게** 보므로 지나갈 수 없는 틈을 "지나갈 수 있다"고 판단한다 —
> **틀리는 방향이 안전 반대쪽이다.**
> **다만 실주행 영향은 지금 없다** — `white1/one_launch.py` 는 `drive_lidar_node` 를
> 띄우지 않는다(그 런치 주석: *"drive_gps_node·drive_lidar_node 도 띄우지 않는다"*).
> `ros2 launch lidar drive_lidar.launch.py` 를 **단독으로** 쓸 때만 문제가 된다.
>
> **→ 할 일 : `lidar` 두 곳의 0.65 를 1.10 으로 고친다.** 이번 커밋에는 넣지 않았다 —
> 그 노드는 라바콘 코리도 잠금 로직이 그 폭에 맞춰 튜닝돼 있을 수 있어
> (`path_corridor_half_width` 등과 함께 봐야 한다) 단독 검증이 필요하다.

### 1.2 구동 (인휠)

| 항목 | 값 |
|---|---|
| 모터 | **QSWP72V5000W** (QS260 계열) 허브모터 **2개**, 양 뒷바퀴 직결(감속기 없음) |
| 극수 | **16극쌍(32극)**, 홀센서 위상각 120°, 무부하 최대 **1320 RPM** |
| 컨트롤러 | **KLS7275H ×2** (모터 1개당 1개) |
| 배터리 | **58 V** 구동. 로직 12 V 는 별도 SMPS |
| 엔코더 | 홀 3상을 **SN74HC86N(XOR)** 로 합산 → 컨트롤러당 1신호. `CHANGE` 인터럽트 |
| **분해능** | **바퀴 1회전 = 96펄스** (3상 홀 6엣지 × 16극쌍) |
| **환산** | **1펄스 / 20 ms 창 = 0.884 m/s = 3.182 km/h** |
| 물리 상한 | **42.2펄스/창** (1320 RPM). 펌웨어 `PULSE_SANITY_MAX = 40` 이 그 아래 |
| 배선 | 왼쪽 : 홀 D2 → PWM **D8** / 오른쪽 : 홀 D21 → PWM **D9** (좌우 독립 PID, 교차 없음) |
| 후진 | **없다.** A보드가 음수를 받지 않는다 |

**기동 초반 허수 펄스(실측)** — "코일에 힘은 들어갔는데 바퀴가 아직 안 도는" 구간에서
홀신호에 수백~1500 단위 허수가 12~15주기(240~300 ms) 쏟아진다. 펌웨어가
`PULSE_SANITY_MAX=40` 으로 큰 것만 거르고, 그 아래는 ROS 쪽 `cb_encoder` 의
**중앙값 3점 필터**(`driving.py:914 ENC_MEDIAN_N`)가 죽인다. 실측에서 지령 0~1펄스
구간의 엔코더가 중앙 16, 최대 34 까지 튀었다(정상 구간 4~5).

### 1.3 조향

| 항목 | 값 |
|---|---|
| 액추에이터 | DC모터 + 포텐셔미터 위치제어. 드라이버 **MD20A** (DIR **D22** / PWM **D6**) |
| 위치 센서 | **A2** 가변저항 |
| **`±40°` 의 정체** | ⚠️ **도로휠각이 아니다.** `STEER_ANGLE_MAX` 는 가변저항 하드리밋 사이 **전체 행정에 붙인 이름**일 뿐 |
| 링키지비 | **pot = 1.26 × 도로휠각** (126표본 최소자승, 잔차 RMS 4.2°) |
| 언더스티어 보정 | **+5.17 × v²·tan|δ| / L** [deg/(m/s²)] |
| 전체 식 | `pot = 1.26·|δ| + 5.17·v²·tan|δ|/L`, 그 뒤 ±40 클램프 |
| 하드리밋 raw | 좌 576 / 우 362 (0731 실측). 코드 기본 `DEF_RAW_LEFT_LIMIT=684` — **EEPROM 캘리브 값이 우선** |
| 불감대 | `STEER_TOLERANCE_EXIT = 6` raw 카운트 = pot 1.78° = **도로휠 1.41°** |
| 불감대의 뜻 | LFD 5.16 m 에서 이 각도가 못 지우는 **측방오차 0.26 m** — 그보다 작은 CTE 문턱은 무의미 |
| 동특성(실측) | 불감시간 0.250 s / 63% 0.550 s / 완료 0.750 s, **슬루 70°/s**. 요레이트 τ = **0.500 s** |
| 도달 판정 | `SETTLE_MS = 500` — **매 주기 쓰면 영원히 성립하지 않는다**(값 변화 + 1 s keepalive 로만 쓸 것) |
| 영점 캘리브 | 시리얼 `a` 한 줄. **ROS 송신 경로를 일부러 두지 않았다** (`BOARD_B.md` 3절) |

### 1.4 제동 (리니어)

**리니어모터가 브레이크 페달을 물리적으로 밟는다.** 드라이버 **MD20A** 1채널
(DIR **D8** / PWM **D9**), 위치는 **A5 가변저항 raw**(절대값, 9점 중앙값).

| 단계 | pot raw | 뜻 |
|---|---|---|
| **0** | — | 놓음. **위치를 보지 않고 REV 로 `BRAKE_HOME_MS = 1000 ms`** |
| **1** | 600 | 약한 브레이킹 (행정 ≈1/3) |
| **2** | 850 | 풀브레이킹 |

**실측 감속도** (`BRAKING.md` / 로그 `ros2bag/a1a2.csv`, 186 s, RTK Fixed 100%):

| | 감속도 | 성격 |
|---|---|---|
| 코스트(펄스 0) | 0.29 ~ 0.54 → **대표 0.41 m/s²** | 인휠 자연감속뿐. **펄스 0 은 '정지'가 아니다** |
| **1단** | 0.62 / 0.97 / 1.05 → **평균 0.88 m/s²** | 부드럽다. 코스트의 약 2배 |
| 1단(구동 차단) | **1.30 m/s²** (전 구간 에너지식) | 자율주행에서는 arduino 가 REF 를 0 으로 덮으므로 이쪽이 실효값 |
| **2단** | **2.21 / 3.82 → 하한 2.2 m/s²** | '감속'이 아니라 **'정지'** |

**행정 지연** — 1단은 물린 뒤 **첫 0.55 s 동안 감속도가 0**(행정 램프 290카운트).
2단은 행정이 길어도(540카운트) 제동력이 빨리 선다(0.30 s).

**제동등** — 12 V 램프, **D11** `digitalWrite`(타이머 안 씀). 판정은
**A5 ≥ 350** 하나뿐(`kasa_0904_B.ino:323`, 0821 초판 400 → 실차에서 350).
브레이크 단계도 주행모드도 E-stop 도 보지 않는다.

> ### ★★ 리니어에 관한 불변식 ★★
> 1. **펌웨어는 시킨 대로만 움직인다.** 스스로 체결하지도 복귀하지도 않는다.
>    "E-STOP 도 아닌데 리니어가 튀어나왔다" 면 원인은 100% ROS 가 보낸 값이다.
> 2. **모드 전환(자율↔수동)은 절대로 리니어를 체결하지 않는다.** 전환 엣지에서
>    브레이크 상태를 전부 '풀린' 쪽으로 지운다 — **거는 로직이 아니라 지우는 로직**.
> 3. **`/brake_level` 은 발행자가 여럿인 '마지막 발행자가 이기는' 토픽이다.**
>    물고 있는 동안 `BRAKE_KEEPALIVE_S = 0.25 s` 로 재발행하지 않으면 남이 푼다.
>    **단 0단은 재확인하지 않는다** — '놓음'을 계속 주장하면 남의 정지를 푼다.
> 4. **최소 물림 0.5 s.** 1단 행정이 290카운트 ≈ 0.54 s 라, 0.2 s 만 물면 제동력은
>    거의 없고 기구만 왕복한다 — 리니어에 제일 나쁜 사용법이다.

### 1.5 보드 · E-STOP

**Arduino Mega 2560 ×2**, 제어 루프 공통 주기 **20 ms**(`CONTROL_WINDOW_MS`),
텔레메트리 50 ms. 시리얼 **115200 8N1**, 줄 단위.

| | A보드 (`kasa_0904_A.ino`) | B보드 (`kasa_0904_B.ino`) |
|---|---|---|
| 담당 | 인휠 좌우 독립 PID + 주행펄스 + 쓰로틀 페달(A0) | 조향 + 제동 + 모드스위치(D5) + 제동등(D11) |
| 텔레메트리 | `S,<좌펄스>,<우펄스>,...` | `P,<실측조향각>,<A5원본>,<주행모드>` (50 ms) |
| 입력 | `<목표펄스>` 0~15 (`TARGET_MAX=15`) 또는 직접 PWM | `<조향각>,<브레이크단계>` (또는 조향 `x` = 힘빼기) |
| E-STOP | **`ESTOP_ENABLED = false`** — 판정 안 함 | **`ESTOP_ENABLED = true`** — 이 보드만 D12 를 본다 |
| 워치독 | **`RX_TIMEOUT_MS = 3000`** — 3초 무입력이면 스스로 정지 | 없음 (조향·제동은 멎어도 안전) |

**E-STOP** = **D12(NC)** 하드웨어 정지. 두 보드에 병렬로 물려 있지만 **판정은 B보드
하나**가 한다. 발동·해제 확인시간 각 **100 ms**(0904 에서 500→100). 체결되면 B보드가
`STOP` 한 줄만 내보내고 리니어 2단 체결·0단 복귀까지 스스로 한다. ROS 는 `/estop` 으로
**보고받을 뿐**이다.

> **`STOP` 줄에 필드를 붙이지 말 것** — `arduino.py` 의 판정이 **문자열 완전일치**다.

**A보드 인휠 PID** (`kp=0.4, ki=0.03, kd=0.2`, `PWM_MAX=170`, `PWM_SLEW_MAX=+4/cycle`):

- **FF 테이블** (펄스→PWM, 3점 라그랑주 2차보간):
  `1→60, 2→70, 3→80, 4→90, 5→100, 6.5→110, 8→120, 10.09→130, 13.05→140, 16.05→150, 20.45→160, 24→170`
- **★재가속 함정★** 적분 누적 조건이 `abs(err) < I_ACCUM_ERR_MAX(=4)` 다. 정지한 차에
  목표 4펄스를 주면 err 가 **정확히 4** 라 조건이 거짓 → **적분이 영원히 안 자란다.**
  PWM 이 92 (= FF 90 + 0.4×4) 에 묶인다. **목표 4펄스가 목표 2펄스(PWM 111)보다 약하다.**
  `BRAKING.md` 부록 A 의 "4펄스 12.4초 / 엔코더 0 / GPS 이동 0.03 m" 가 이것이다.
  → **정지 상태에서 재출발할 때 4펄스를 피한다.** 안전값 3펄스(err=3 → 적분 누적 O).
- **기동 블랭킹** — 정지(`useSpeed ≤ LAUNCH_ENTRY_SPEED_MAX=1`)에서 진입해 피드백을
  무시하고 `FF(target)` 까지 개루프 램프. 종료는 (램프 완료 && 정상펄스 5주기 연속)
  또는 `LAUNCH_MAX_MS = 3000` 타임아웃.
- **핸드셰이크** — 식별 직후 `-` 한 줄을 보내 `YES` 를 받아야 연결로 인정한다.
  텔레메트리는 *보드→PC* 한 방향만 증명한다.

### 1.6 센서

| 센서 | 모델 | 인터페이스 | 노드 → 토픽 |
|---|---|---|---|
| **GPS** | **SMC-2000** (RTK 수신기) | USB 시리얼 **115200**, NMEA. udev `/dev/gps` 권장.<br>VID/PID 후보 `1546:01A9`(u-blox 9) / `1546:01A8` | 외부 패키지 `nmea_navsat_driver/nmea_serial_driver` → `/fix` (**5 Hz**)<br>→ `white1/gps` → `/gps_fused` (**20 Hz**) |
| **IMU** | **iAHRS** (6축) | USB 시리얼, **CP210x** `10C4:EA60`. udev `/dev/imu` 권장 | `white1/iahrs` → `/imu` (동기주기 50 ms)<br>→ `white1/speed` → `/speed` [km/h] |
| **라이다** | **Ouster OS1-32** | **유선 LAN** — 호스트 `eno1` 이 `192.168.6.100/24`, 센서 `192.168.6.11` | `ouster_ros/os_driver` → `/ouster/points` |
| **카메라** | USB 웹캠 (외부) | `/dev/video*`. 내장/적외선 노드 오선택 방지 로직 있음 | `usb_cam` → `/image_raw` → `white1/traffic_light` |

**GPS 품질 판정 — `status.status` 로는 Fixed 와 Float 이 구별되지 않는다.**
`nmea_navsat_driver` 의 GGA quality 매핑이 q4(Fixed) · q5(Float) · q9(WAAS) 를 전부
`STATUS_GBAS_FIX(2)` 로 보낸다. 그래서 `white1/gps` 는 **σ = √covariance** 로 가른다:

| 코드 | 라벨 | 뜻 |
|---|---|---|
| 1 | `SPS` | GGA q1. 오차 수 m |
| 2 | `DGPS` | GGA q2. 오차 ~1 m |
| 3 | `RTK_FLOAT` | q4/q5/q9 인데 σ > 문턱 |
| 4 | `RTK_FIXED` | σ ≤ **`RTK_FIXED_SIGMA_M = 0.30 m`** — 오차 ~2 cm |

근거: Fixed 최악 0.10 m 와 Float 최선 2.00 m 사이가 비어 있어 양쪽 3배 이상 여유.
covariance 가 비어 있으면 **Fixed 로 올리지 않는다**(모르면 낮게 본다).

**`white1/gps` 가 하는 일 4가지** — ① 품질 판정 ② 이상치 게이트(물리·방향·품질)
③ 5 Hz fix 사이 공백을 IMU 로 메워 20 Hz 가상좌표 ④ 품질 저하 구간 융합(DEGRADED,
현재 기본 OFF). **상시 칼만필터가 아니다** — Fixed 면 fix 로 스냅한다.

⚠️ **아직 없는 것** — 듀얼 안테나(GNSS heading), 자이로 바이어스 보정. 그래서 헤딩은
출발할 때 **곧게 굴려 GPS 변위로 초기화**한다(`HeadingEstimator`).

---

## 2. 단위·부호 규약 — ★이식에서 제일 많이 틀리는 곳★

`/cmd_vel_raw` 는 `geometry_msgs/Twist` 지만 **필드의 뜻이 표준과 전부 다르다.**
1/5카 코드를 그대로 꽂으면 **에러 없이 조용히 틀린다** — 빌드도 되고 토픽도 붙고
`echo` 도 정상인데 차가 전혀 다르게 움직인다.

| 필드 | 1/5카 (구 `motor.py`) | **금색차 (`nxde/arduino.py`)** |
|---|---|---|
| `linear.x` | m/s (실수) | **주행 목표펄스 정수 0~15** |
| `angular.z` 부호 | + = 좌 | **− 좌 / + 우** |
| `angular.z` 물리 | 도로휠각 δ | **pot 지령**(링키지비·언더스티어 반영 후) |
| 제동 | 없음 | **`/brake_level` (Int32) 0/1/2 별 토픽** |

- **`/encoder` 는 좌+우 펄스의 합이다** (평균이 아니다). 소비측은 `× 0.5` 로 바퀴
  하나 기준으로 되돌린다(`ENC_SUM_TO_PULSE`). 합을 쓰는 이유는 Int32 라 평균이
  `(0,1) → 0.5 → 0` 으로 깨지기 때문. 양자화 눈금도 절반(0.442 m/s)이 된다.
- **부호가 뒤집히는 지점은 한 곳뿐이어야 한다.** 두 번 뒤집으면 조용히 좌우가 바뀐다.
  C++ 쪽은 `KasaActuator::drive()` 한 곳(`kasa_units.hpp`), Python 쪽은 `driving.py`
  가 애초에 보드 부호로 계산한다.
- **후진은 없다.** 매핑도 `direction` 열에 항상 `1` 만 쓴다.

---

## 3. 소프트웨어 구조

### 3.1 패키지

```
gold_ws/src/
  white1/     ★주력 스택★ (ament_python)
    white1/
      driving.py       ★3592줄 — 헤딩 + 상태기계 + 경로추종 + 제동정책★
      gps.py           /fix + /imu → /gps_fused (품질판정·게이트·DR)
      iahrs.py         iAHRS 드라이버 → /imu   ⚠️ from white import ports (0.1절)
      speed.py         /imu 적분 → /speed [km/h]  ★보조 속도원★
      mapping.py       /fix 원값만 보고 경로 수집 → gps_data/route_*.csv
      prompt.py        CLI 메인화면 (매핑/주행/종료) — ★시작은 여기뿐★
      record.py        주행 구간 토픽 → ros2bag/<경로이름>-<시각>.csv (87열, 20 Hz)
      hud.py           차량 상면도 HUD (구독 전용)
      traffic_light.py 신호등 인지 → 빨간불이면 리니어 2단
      camera_model.py / camera_launch.py / ports.py / paths.py
    launch/one_launch.py    통합 런치
    gps_data/  ros2bag/  sound/  calibration/
    BOARD_B.md  BRAKING.md  GPS_HEADING.md  STOPLINE_TEST.md  CHANGELOG.md

  nxde/       아두이노 계층 (런치파일 없음, 전부 ros2 run)
    arduino.py   ★차량 구동의 필수 노드★ A/B 2보드 시리얼 브리지
    master.py    마우스·키보드 GUI 조종 (하드웨어 검증용)
    joystick.py  조이스틱 조종 (자율모드 한정 + 영점→SWA)
    sound.py     음성 안내 (구독 전용). 음원의 주인은 white1/sound/
    video.py     /image_raw → mp4 녹화
    check.py     ★런치 전 하드웨어 점검★ 보고하고 종료
    kill.py      돌고 있는 ROS2 를 한 번에 끝낸다 + 포트 초기화
    tts.py       안내 음성 mp3 제작 도구

  mppi_local_planner/   ★라바콘 회피 — /lstatus 가 'L' 인 구간만 몬다★ (C++)
    src/mppi_local_planner_node.cpp   게이트·조종권·지령층 (1203줄)
    src/mppi_controller.cpp           플래너 본체 (1/5카 원본 그대로)
    src/ego_costmap.cpp
    config/params.yaml   ★최상단 cruise_pulse 하나로 순항속도를 정한다★

  white · white806 · white0901   이전 세대 스냅샷 (참고용, ★손대지 않는다★)
  ouster-ros/                    외부 드라이버 (서브모듈)

  lidar/      C++ 라이다 인지·주행 (ament_cmake)
    cone_lidar_node    가상범퍼 AEB (차를 움직이지 않는다)
    drive_lidar_node   라바콘 코리도 잠금 + 헤딩홀드 직진 ★런치 = 출발★
    drive_gps_node     GPS 직선 매핑 + 스탠리 추종 ⚠️ driving.py 와 정면으로 겹친다
    include/lidar/kasa_units.hpp   ★C++ 쪽 단위·게이트 단일 소유자★
```

> **`drive_gps_node` 와 `white1/driving` 을 동시에 띄우지 말 것** — `/cmd_vel_raw`
> 발행자가 겹친다. 실주행은 `white1`, 저쪽은 라이다 AEB 시험용이다.

### 3.2 토픽 계약

**`driving` 노드 발행**

| 토픽 | 타입 | 뜻 |
|---|---|---|
| `/cmd_vel_raw` | Twist | `linear.x` = 목표펄스, `angular.z` = pot 지령(− 좌 / + 우) |
| `/control_state` | Bool | 구동 허용 |
| `/brake_level` | Int32 | 0/1/2 — **`max(내 요청, 신호등 요청)`** 으로 합쳐서 낸다 |
| `/mapping_cmd` | Bool | 매핑 시작/종료 |
| `/drive_state` `/drive_event` | String | 상태·사건 (prompt·hud·sound 가 본다) |
| `/tl_permit` | Bool | 신호등 개입 허락 |
| **`/lstatus`** | String | **`'0'` GPS추종 / `'L'` 라이다 / `'S'` 일시정지 — ★이것이 조종권이다★** |
| `/ego_state` `/drive_diag` | Float64MultiArray | 계측·진단 |

**`driving` 노드 구독** — `/gps_fused` `/imu` `/encoder` `/speed` `/vehicle_mode`
`/estop` `/drive_cmd` `/tl_brake_req` `/lidar_active`

> **`/lstatus` 는 `terrain` 원값이 아니다.** `lstatus_now()` 가 "내가 실제로 손을
> 놓았는가" 로 만든다 — `S_DRIVE_RUN` 이 아니면 무조건 `'0'`, `_revoke_lidar`
> (E-STOP·GPS 두절)로 조종권을 거두면 경로가 그대로여도 `'0'`. 그래야 mppi 가
> 선 차를 계속 몰지 않는다. CSV 의 `''`·`'0.0'`·`'-1'`·`'l'`·`'s'` 도 세 값으로
> 정규화해서 낸다 — **구독자마다 파싱이 갈리지 않게 발행자가 접는다.**

**`arduino` 노드** (`nxde`)

```
ROS → 보드 : /cmd_vel_raw  /control_state  /brake_level  /aeb_stop
보드 → ROS : /encoder  /steer_angle_measured  /vehicle_mode  /throttle_pedal
             /brake_pot  /drive_pulse_cmd  /drive_pwm_cmd  /estop  /board_status
```

- **`/aeb_stop` (Bool)** — **수동조종 중에도 통하는 유일한 제동 경로.**
  `/brake_level` 은 자율주행에서만 통한다(수동조종 분기는 브레이크를 항상 0 으로
  보낸다 — "제동은 사람 발이 한다"). `/aeb_stop` 은 모드와 무관하게 구동을 끊고
  리니어를 문다. **상태로, 끊기지 않게(20 Hz) 낼 것** — arduino 가 신선도를 보고
  끊기면 해제한다(죽은 노드가 리니어를 영구히 물지 않게).
- **`arduino.compose()` 우선순위** — 수동조종(D5 개방)이 `/control_state`·
  `/cmd_vel_raw` 보다 **먼저** 판정된다. 그래서 `one_launch` 와 `prompt` 가 떠 있어도
  사람이 페달로 모는 경로는 막히지 않는다. **`arduino` 노드 하나만 떠 있어도 수동조종
  주행이 된다.**
- **`KEEPALIVE_S = 1.0 s`** (A·B 양쪽 재전송)와 A보드 `RX_TIMEOUT_MS = 3000` 은
  **한 쌍이다** — 어느 한쪽만 늘리면 정상 주행 중에 구동이 끊긴다.

### 3.3 상태기계 (`driving.py`)

```
S_IDLE ──MAP_START──▶ S_MAP_HEADING ──헤딩 확정──▶ S_MAP_RUN ──▶ S_IDLE
       └─DRIVE_START─▶ S_DRIVE_HEADING ──헤딩 확정──▶ S_DRIVE_RUN ──▶ S_DRIVE_DONE ──▶ S_IDLE
```

- **시작은 `prompt` 의 메뉴뿐이다.** B보드 D5 스위치는 더 이상 아무것도 시작시키지
  않고 **취소·정리만** 한다(`on_mode_edge`). 시작 트리거가 두 곳에 있으면 "지금 뭐가
  방금 시작을 시켰는지" 추적이 안 된다.
- **시작 대기는 2단 게이트** — ① 스위치 위치(매핑=수동 / 주행=자율) ② E-STOP 해제.
  순서가 곧 안내 우선순위다.
- **헤딩 초기화** — 조향 0°로 곧게 굴려 GPS 변위로 초기 헤딩을 잡는다. 거리를 미리
  정하지 않고 **추정 오차 σ 가 3° 밑으로 떨어지는 순간** 확정(RTK Fixed 면 대개 1 m
  남짓). 직선 잔차 RMS 상한 0.15 m 로 "곧게 갔는가"도 함께 본다.
- **`enter()` 의 불변식** — `S_DRIVE_DONE` 이면 **조향 0 + 펄스 0 + 리니어 2단을 같은
  틱에** 내고, **그 밖의 모든 상태에서는 무조건 `set_brake(BRAKE_NONE)`** 이다
  (`driving.py:1913-1926`). ⚠️ **이 규칙 때문에 "정지 상태를 유지하는 새 state" 를
  만들 수 없다** — 6.1절 참고.
- **E-STOP 은 '취소'가 아니라 '일시정지'** — 상태와 WP 인덱스를 그대로 들고 제자리에
  서고, 해제되면 `ESTOP_RESUME_GRACE_S = 1.5 s` 뒤 **그 자리에서 이어서 재개**한다.
  (B보드가 리니어를 0단으로 되돌리는 데 1000 ms 를 쓰므로 그 사이 구동을 걸면
  브레이크가 아직 빠지는 중인 채로 민다.)

**★'정지'는 여러 가지이고 E-STOP 은 그중 하나뿐이다★** — 로그·음성·화면에서
'사람이 회로를 끊은 것'과 '차가 스스로 판단한 것'을 구별하려고 이름을 갈라 뒀다.

| 기호 | 이름 | 조건 |
|---|---|---|
| 🚨 | **E-STOP** | D12 하드웨어. **이 말은 여기에만 쓴다** |
| ⛔ | 경로이탈 안전정지 | `|CTE| > 2.0 m` → DRIVE_DONE |
| 🎯 | 도착 정지 | 반경 0.9 m 또는 통과 판정 → DRIVE_DONE |
| 🛑 | 정지명령 | prompt 의 STOP → DRIVE_DONE |
| 🔻 | 종점 1단 접근제동 | DRIVE_RUN 안에서 (감속) |
| 🅑 | 코너 1단 선행제동 | DRIVE_RUN 안에서 (감속) |

### 3.4 데이터 — 파일 위치와 CSV 양식

**`paths.py` 가 저장 위치의 단일 소유자다.** 해석 순서:
① 명시 파라미터 → ② 환경변수 `WHITE1_*_DIR` → ③ **소스트리 `<ws>/src/white1/{gps_data,ros2bag,sound}`**
→ ④ 설치 share → ⑤ `~/white1/...`

> ### ★★ `--symlink-install` 은 이제 쓰지 않는다 ★★
>
> **`mppi_local_planner` 와 `lidar` 가 C++(ament_cmake) 패키지라, 이 워크스페이스는
> 더 이상 `--symlink-install` 로 빌드할 수 없다.** (`lidar/README.md` 151행 ·
> 304행에 이미 '금지' 로 못 박혀 있다.)
>
> ```bash
> colcon build            # ★--symlink-install 없이★
> ```
>
> 그래서 `paths.py` 는 **복사설치본에서도** 소스트리를 찾아야 한다 —
> 그 폴백은 [2026-09-04] 에 고쳤으므로 지금은 맞게 돈다
> (종전에는 `~/white1/{sound,gps_data,ros2bag}` 로 떨어져 **"음성 안내가 한 마디도
> 안 나오고 경로 CSV 목록이 비어 보이던"** 원인이었다).
>
> ⚠️ **대신 Python 소스를 고친 뒤에는 반드시 다시 빌드해야 한다.**
> 복사설치본은 스스로 갱신되지 않는다 — `--symlink-install` 시절의 "고치고 노드만
> 재시작" 습관이 남아 있으면 **고친 적 없는 코드를 계속 돌리게 된다.**

#### 매핑 경로 CSV — `gps_data/route_<YYYYMMDD>_<HHMMSS>.csv`

`mapping.py` 가 쓴다. **`/fix` 원값만** 보고, **거리 0.25 m 마다 한 점**
(`SPACING_M`, 시간 간격이 아니다 — 신호 대기에서 같은 자리가 쌓이지 않게).

```
latitude,longitude,heading,speed,steer,direction,pitch,terrain,
throttle_pulse,wheel_pulse,wheel_speed,steer_measured,throttle_raw,auto_mode,estop
```

- **앞 8열은 구 white 와 열 위치가 같다**(분석툴 호환). 뒤 7열은 수동조종 실계측.
- `direction` 은 항상 `1`(후진 수집 없음), `pitch`/`terrain` 은 항상 `"0.00"`/`"0"`.
- **`driving` 의 경로 로더는 `latitude`/`longitude` + `terrain` 만 읽는다** —
  열이 더 있거나 없어도 된다.
- ⚠️ `wheel_speed` 열은 **2026-08-14 이전 파일이 실제의 절반**이다.

**★`terrain` 열 — 사람이 손으로 적는 '구간 지정' 열★**

| 값 | 뜻 | 누가 모나 |
|---|---|---|
| **`L` / `l`** | **라이다 구간** — 라바콘 회피 | `mppi_local_planner` |
| 그 밖 (빈 칸 · `0` · 숫자 · 열 없음) | GPS 추종 | `white1/driving` |

`driving.py:963-964`, 판정은 `zone_at()`(:1511), 읽는 곳은 `select_route()`(:1678).
**`'L'/'l'` 만 라이다이고 나머지는 전부 GPS 로 떨어진다** — 그래서 예전 CSV 가 그대로
돈다.

⚠️ **`gps_data/` 에는 지금 29개가 있다**(추적 21개 = 9/5자 `route_20260905_*` +
`_remodeled` 짝, 미추적 8개). **그중 `terrain` 이 비어 있지 않은 것이 둘 있다.**

| 파일 | 행 | 총 길이 | 평균 간격 | 열 구성 | `terrain` 실제 값 |
|---|---|---|---|---|---|
| `route_20200101_111111.csv` | 3230 | 745.4 m | 0.231 m | `...,terrain,lane_cte,lane_conf,lane_flags` | `-1`×66, `0`×3164 |
| `route_20200202_222222.csv` | 2205 | 568.0 m | 0.258 m | 같음 | `-1.0`×223, `0.0`×1950, `1.0`×32 |
| `route_20260824_200204.csv` 외 5개 | 76~384 | 43~90 m | 0.40~0.43 m | 현행 15열 | 전부 `0` |

앞의 둘은 **구 white 스택이 만든 파일**이고, 그 `terrain` 열에는 `/terrain_state`
지형코드(−1/0/1)가 실려 있다. 지금 규약상 전부 GPS 로 떨어져 **무해하지만**,
`L`/`S` 를 손으로 적을 때는 **그 숫자를 지우고 적는 것**임을 알고 있어야 한다.
(열 구성도 다르다 — `lane_*` 3열이 있고 `throttle_pulse` 이하가 없다.)

#### 주행 기록 CSV — `ros2bag/<경로이름>-<YYYYMMDD>_<HHMMSS>.csv`

`record.py` 가 `one_launch` 와 함께 뜨고 **자율주행 모드 + 주행 구간에서만** 스스로
켜진다. **한 행 = 한 시점의 차량 전체 상태**(20 Hz 스냅샷, 87열). 숫자 토픽은 다음
값이 올 때까지 유지, 문자열·이벤트는 새로 온 행에만 찍는다. 앞 두 열은 항상
`t_wall`(UNIX epoch)·`t_rel`. 기록 대상은 `record.py` 상단 `RECORD_TOPICS` 표 하나.

---

## 4. 제어 알고리즘 (`driving.py` DRIVE_RUN 한 틱)

```
advance_wp_idx()                     진행 포인터 (창 안 최근접, 앞쪽 우선)
  ↓
[라이다 구간이면 → 침묵하고 return]
  ↓
경로이탈 판정 (|CTE| > 2.0 m → DRIVE_DONE)
  ↓
종점 판정 (반경 0.9 m or 통과)
  ↓
scan_curve_demand()  근거리(LFD×1.6) / 원거리(14 m) 2단 스캔
  ↓
lookahead_m()        LFD = min(속도표 v√2/ω_n , 곡률캡)
  ↓
corner_speed()       ① 곡률 비례 감속 ② 제동거리 게이팅 ③ ω_n 결속 → 목표펄스
  ↓
goal_approach()      종점 1단 제동 → 크립  ★곡률보다 뒤에서 덮는다★
  ↓
corner_brake()       코너 1단 제동 + REF 캡  ★반드시 goal_approach 뒤★
  ↓
[재수렴 중이면 pulse = min(pulse, 2)]
  ↓
pure_pursuit_steer() + apply_cte_integral() → steer_command() → send()
```

### 4.1 순수추종 + 가변 LFD

```
목표점 = wp_idx 이후 LFD 이상 떨어진 첫 '앞쪽' WP
alpha  = 그 점의 차체기준 방위 (왼쪽 +)
δ      = atan(2·L·sin α / 거리)                    ← 도로휠각
pot    = 1.26·|δ| + 5.17·v²·tan|δ|/L,  ±40 클램프  ← 부호는 − 좌 / + 우
```

**LFD = v·√2 / ω_n**, `ω_n = 0.97 rad/s`(`LFD_OMEGA_N`), 하한 2.3 m / 상한 6.4 m.
표가 아니라 설계식을 옮긴 이유는 `drive_pulse` 를 바꾸면 자동으로 따라오게 하려고.

**ω_n 0.97 의 근거** — 위상여유 `PM ≈ 65° − ω_n·τ·(180/π)`, 금색차 실측 τ=0.500 s:

| ω_n | LFD | PM |
|---|---|---|
| 0.97 | 5.16 m | **+37°** |
| 1.22 | 4.10 m | +30° |
| 2.26 | 2.21 m | **0° = 발산임계** |

**금색차의 발산임계는 1/5카의 1.2 가 아니라 ≈2.3 이다**(조향이 약 2배 빠르다).
그래서 물려받은 0.97 은 넉넉히 안전한 쪽.

### 4.2 곡률 선행제동 (속도)

- 두 창으로 스캔 — `WINDOW_M 3.0 m`(지속 곡률) + `WINDOW_PEAK_M 1.2 m`(Menger 3점
  외접원, 짧고 급한 필렛). 3 m 창은 R≈2 m 코너를 11° 로 과소평가한다.
- `demand_to_speed()` : 요구 도로휠각이 `STEER_FULL_SLOWDOWN_DEG(15°)` 이상이면
  최저속도까지 선형 감속.
- **제동거리 게이팅** : `v_brake = √(v_corner² + 2·a·(게이트거리 − 1.5))`,
  `a = BRAKE_GATE_DECEL = 2.0`. **코너를 1.5 m 앞에서 이미 목표속도로.**
- 속도 대역 : `max = drive_pulse × 0.884`, `min = max(0.9, max × 1/2.8)`.
  최종 펄스 하한은 **`CORNER_MIN_PULSE = 2`**.
- ⚠️ **`BRAKE_GATE_DECEL = 2.0` 은 2단에서만 참인 값이다**(실측 코스트 0.41).
  게이트는 코스트보다 5배 세게 줄일 수 있다고 착각한 채 목표를 낸다. 그 낙관을
  보정하는 것이 아래 코너 1단 제동의 REF 캡이다.

### 4.3 코너 1단 선행제동 (2026-08-18)

**목표펄스만 낮춰서는 못 줄인다** — 낮은 목표에 도달하는 수단이 코스트(0.41 m/s²)뿐
이라 4→2펄스에 11.4 m 가 필요한데 스캔창이 14 m 다. 1단(0.88)이면 5.33 m 로 들어온다.

- **체결 판정은 경로 기하만 본다**(남은 거리 + 목표속도). **실측속도는 해제에만.**
  → 되먹임 루프가 성립할 수 없다(과거 3번 실패한 '실측 되먹임' 정책의 교훈).
- **코너당 한 번** 물고, 풀면 `LOCKOUT 1.5 s` + '새 코너' 조건까지 재체결 금지.
- **GPS 속도를 못 읽으면 아예 안 문다** (엔코더 폴백으로는 절대 제동하지 않는다 —
  허수 카운트가 제어에 들어오는 경로를 끊었다).
- 체결 지점 = `d_need × CORNER_BRAKE_LATE_K(1.2) + BRAKE_GATE_MARGIN(1.5)`.
  1.2 는 **행정 램프 손실(83%)과 우연히 같다** — **내리지 말 것.**
- **REF 캡** — 제동을 결정한 순간부터 목표펄스를 `v_corner` 위로 못 올라가게 막는다.
  안 막으면 제동으로 줄인 속도를 REF 가 그 자리에서 되돌린다(폐루프 시뮬:
  캡 없음 2.93펄스 → 캡 있음 **2.06펄스**, 목표 2).
- **2단은 주행 중에 쓰지 않는다.** 2단은 정지 장치다.
- **절대 세우지 않는다** — A보드 재가속 함정 때문에 목표를 2펄스 이상으로 유지하고
  일찍 푼다.

### 4.4 종점 접근 (1단 → 크립 → 도착 2단)

```
NONE ─(need₁ 도달)─▶ BRAKE1 ─(1단으론 못 센다)─▶ BRAKE2
  │                     └────(정지·저속·시간초과)────┐
  └─(이미 느리다)──────────────────────────────────▶ CREEP ─▶ (도착)
```

되돌아가는 경로가 없어 리니어가 채터링하지 않는다. 거리는 **직선거리**로 재고
(`s_left` 는 포인터가 앞서 튀면 0 으로 주저앉는다), 감시 창 진입에만 둘 다 요구한다.

**크립 재출발 킥** — 정지한 차를 떼어내는 것이 이 차의 최대 난점이라(67초 정체 실측),
1펄스 크립 중 멈추면 `GOAL_KICK_PULSE(4)` 로 0.6 s 짧게 밀고, 3회 실패하면 구동을
끊는다. ⚠️ **킥 펄스 4 는 A보드 적분 동결 함정과 정확히 같은 값이다**(6.2절 참고).

### 4.5 CTE 적분 (Ki 단독)

관측된 것은 '진동'이 아니라 **한쪽 편향**이었다(평균 +0.22 m, 부호 + 76%, 반전
3.1회/분). 순수추종은 조향 불감대(도로휠 1.41° = LFD 5.16 m 에서 측방 0.26 m) 아래의
편향을 **원리적으로 못 지운다**. 적분만이 시간을 들여 그 문턱을 넘는다.
P·D 는 넣지 않는다 — 순수추종 기하와 권한이 겹쳐 두 배로 꺾인다.
`CTE_KI = 0.30 deg/(m·s)`, 기여 상한 2.5°(pot 4.4°).

### 4.6 라이다 구간 이양 (2026-09-01)

**설계의 전부는 "발행자를 시간축에서 배타로 만든다" 하나다.**

```
G 구간 : driving 이 발행  ·  /lstatus = '0' → mppi 침묵
L 구간 : driving 침묵     ·  /lstatus = 'L' → mppi 가 발행
S 지점 : driving 이 세운다 ·  /lstatus = 'S' → mppi 침묵
```

> ★[2026-09-07 구현]★ `/lidar_permit`(Bool) 을 **`/lstatus`(String)** 로 대체했다.
> 계약의 성질(**신선도가 곧 허락**)과 3단 이양 순서는 그대로 옮겼고,
> `/lidar_active` 는 방향이 반대라 남는다(6.4⑤).

- **양방향 계약** — mppi 가 `/lidar_active` 로 매 틱 생존을 신고하고, **신선하지
  않으면(1.0 s) 이양하지 않는다.** 구간 중에 끊기면 그 자리에서 정지 + 리니어 2단.
  (arduino 의 `cb_cmd_vel` 은 **신선도를 보지 않고 마지막 펄스를 래치**하므로,
  아무도 안 몰면 차가 직전 펄스로 계속 굴러간다.)
- **이양 순간 마지막으로 0 펄스를 한 번 내고** 침묵한다.
- **복귀 버퍼(`REJOIN_*`)** — mppi 가 복귀하는 곳은 L 구간 진입 시점의 IMU 헤딩으로
  그은 **자기 직선**이지 GPS 경로가 아니다. 그대로 되받으면 첫 틱에 |CTE| > 2.0 m 로
  급정지한다. 재수렴 중에는 이탈 문턱 6.0 m, 속도 상한 2펄스, 적분 금지.
- ★[2026-09-07 구현] 속도를 미리 맞춰 준다★ — L 구간 진입 **5 m 앞**부터
  종점과 같은 서행 로직(`lidar_approach`)으로 **2펄스까지 내려놓고** 이양한다.
  원래 근거였던 "중간 단계를 두면 '누가 지금 속도를 정하는가'가 흐려진다" 는,
  **양쪽이 똑같이 2펄스이면 성립하지 않는다** — 정할 것이 없어지기 때문이다.
  복귀는 종전대로 `REJOIN_PULSE_MAX(2)` → CTE 0.5 m 에서 기본속도로 자연 복귀하고,
  그 계단은 **A보드 `PWM_SLEW_MAX(+4/cycle)` 가 0.20 s 에 흡수한다**(6.4④).
- 구현 4함수 : `begin_lidar_zone`(:2172) / `hold_for_lidar`(:2203) /
  `end_lidar_zone`(:2235) / `_revoke_lidar`(:2154).

---

### 4.7 ★IMU 를 어떻게 붙여도 된다 — 중력축 투영★ [2026-09-09]

**2026-09-08 실차가 이것 때문에 경로이탈로 섰다.** 로그
`ros2bag/route_20260908_203042-20260908_203234.csv`, 반경 5.5 m 좌 171° U턴:

| | 회전량 |
|---|---|
| 실제 (GPS 변위) | **+177.2°** |
| `ego_heading` (자이로 **z축 단독** 적분) | **+136.7°** ← 23% 부족, **40.5° 손실** |
| iAHRS 쿼터니언 yaw | +173.6° ← 센서는 정답을 알고 있었다 |

원인은 **IMU 장착 기울기**다. 정지 중 가속도가 `(−8.44, 0.58, 5.06)`, `|a|=9.855` 로
**IMU z축이 수직에서 59° 누워 있었다.** 차량 yaw 가 세 축에 분산되는데
`cb_imu` 가 `angular_velocity.z` **한 축만** 적분했다.

헤딩이 뒤처지자 순수추종은 "목표가 더 왼쪽" 으로 읽어 조향을 **−40°(포화)** 로
2.7초간 냈고, 차는 선회 **안쪽으로** 파고들어 CTE 가 0.06 → **2.06 m** 로 커져
`CTE_DEVIATION_M(2.0)` 을 넘었다 — **U턴이 끝나는 지점에서.**

**고친 방법 — 축을 '맞춰 다는' 것이 아니라 '재는' 것이다**

```
ω_yaw = ω⃗ · û          û = 중력 반대 방향 단위벡터 (= 차량의 '위')
```

- 차량의 수직축은 **중력이 알려준다.** 헤딩 초기화 구간은 ①곧게 ②조향 0
  ③거의 등속이라 가속도계가 거의 중력만 본다 — **축을 재기에 가장 좋은 구간이고
  마침 헤딩이 필요한 바로 그 순간**이다. 그래서 **헤딩과 같은 순간에 축도 확정**한다.
- `|‖a‖ − 9.81| > 1.2` 인 표본은 버린다(가·감속·요철이 섞인 순간).
- **못 재면 z축 단독으로 떨어진다**(종전 거동) — 그리고 그 사실을 `event` 로 말한다.
  조용히 다르게 도는 것보다 낫다.
- **헤딩 초기화에 들어갈 때마다 다시 잰다.** IMU 를 옮겨 달았거나 브래킷이 틀어졌을
  수 있고, 다시 재는 비용이 0 이다.

**실제 사고 로그로 회귀검증했다** (같은 U턴 구간, 같은 코드):

```
축 확정: True  표본 112개  up = (-0.851, 0.065, 0.521)   z축 기울기 58.6°
  종전 z축 단독       :  +93.2°   (오차 -84.0°)
  ★중력축 투영(신규)★ : +174.0°   (오차  -3.2°)      실제 +177.2°
```

구현 : `driving.py` 의 `cb_imu` / `solve_imu_axis` / `imu_tilt_deg`, 상수절
'중력축 투영'. 진단 열 `gyro_z_dps` 는 이제 **투영된 요레이트**(= 실제로 적분한 값)다.

> **`braketest.py` 에는 이 코드를 넣지 않았다.** 그 노드는 직선 전용이라 요 적분의
> 누적오차가 문제 되는 구간이 없다. 헤더에 그 이유를 적어 두었다.

---

## 5. 실행

```bash
cd gold_ws && colcon build && source install/setup.bash      # ★--symlink-install 금지 (0.4절)★

ros2 run nxde check                    # 0) 하드웨어 연결 점검 (포트 점유 없이)
ros2 launch white1 one_launch.py       # 1) 센서 + arduino + 자율주행 + 라이다
ros2 run white1 prompt                 # 2) CLI 메뉴 (별 터미널)
ros2 run white1 hud                    # 2') 상면도 HUD (구독 전용, 선택)
ros2 run nxde kill                     # 끝낼 때 / 종료가 질척거릴 때
```

**주요 런치 인자** (기본값):

```
use_arduino:=true  use_record:=true  use_mapping:=true  use_sound:=true  use_hud:=true
use_camera:=?      use_lidar:=true   use_lidar_rviz:=false  flip_lidar_xy:=true
drive_pulse:=4     heading_pulse:=3  wheelbase_m:=1.25
lidar_pulse:=2     ← ★L 구간 순항 [펄스]. 이것 하나만 고치면 된다 (6.4②)★
lfd_omega_n:=0.97  lfd_min_m:=2.3
steer_plant_gain:=1.26  steer_understeer:=5.17  cte_ki:=0.30
goal_brake_m:=20.0  goal_brake1_ms2:=1.30  goal_brake2_backstop:=true
corner_brake_enable:=true  corner_brake_late_k:=1.20
manual_use_pwm:=true  manual_pwm_min:=16  manual_pwm_max:=255
throttle_raw_min:=220  throttle_raw_max:=950  throttle_gamma:=1.4
```

**속도를 올려 볼 때** (6.2 — 상한은 6, 기본은 4로 두었다):

```bash
ros2 launch white1 one_launch.py drive_pulse:=5      # 그다음 6
```

**계약 확인** (`/lstatus` 교체 뒤 반드시):

```bash
ros2 topic echo /lstatus                       # '0' → 'L' → '0' 로 바뀌는가
ros2 topic info /cmd_vel_raw --verbose         # ★L 구간에서 발행자가 하나인가★
ros2 topic hz /lidar_active                    # mppi 가 살아 있다고 신고하는가
```

**수동조종은 어느 상태에서도 된다** — D5 가 수동이면 `arduino` 노드 하나만 떠 있어도
사람이 페달·핸들로 몬다(조향은 `x` 힘빼기).

### 5.1 브레이크 제동거리 측정 [2026-09-09 신설]

```bash
ros2 launch white1 braketest.launch.py
ros2 launch white1 braketest.launch.py route:=route_20260909_103000.csv
ros2 launch white1 braketest.launch.py drive_pulse:=8
ros2 launch white1 braketest.launch.py drive_pwm:=140      # ★직접 PWM★
```

직선 경로를 **고정 속도**로 달리다가 **둘 중 먼저 오는 것**에서 **리니어 2단** 을
물고, 완전정지하면 결과를 찍고 **리니어를 풀고** 런치가 스스로 내려간다.

| 트리거 | 무엇 |
|---|---|
| **① 라이다 장애물** | `cone_lidar_node` 의 `/cone_lidar_node/stop_signal` 확정 판정 |
| **② 종점 도달** | 그런 일 없이 경로 끝에 닿았을 때 (반경 1.5 m 또는 통과 판정) |

> **[2026-09-09] 정지 트리거가 `terrain` `'S'` 에서 라이다로 바뀌었다.**
> `terrain` 열은 **이제 보지 않는다** — `'S'` 가 있든 없든 무시한다.
> (`driving.py` 의 `'S'` 일시정지는 그대로다 — 그쪽과 무관한 변경이다.)

**라이다 정지는 `lidar one_launch.py` 의 그 시스템을 그대로 쓴다**

```
ouster 드라이버 ─/ouster/points─▶ cone_lidar_node ─/…/stop_signal─▶ braketest
                                                                        │
                                                                 /brake_level 2단
```

`braketest.launch.py` 가 **`lidar/launch/aeb.launch.py` 를 통째로 include** 한다
(= `ouster.launch.py` + `cone_lidar_node`). 감지 설정은 `lidar/config/cone_lidar.yaml`
**한 곳이 소유**하고 braketest 는 그 Bool 판정을 그대로 받는다 — **문턱을 다시 두지
않는다.** `cone_lidar_node` 는 차를 움직이지 않고, 제동은 받는 쪽이 한다.

- **`use_lidar:=false`** → 종점 정지로만 (장애물 보호 없음)
- **`require_lidar:=true`** → 라이다 판정이 살아 있을 때까지 출발하지 않는다(기본 false)
- **신선도는 fail-open** — 판정이 1.0 s 넘게 끊기면 경고만 하고 계속 간다.
  종점 정지가 어차피 세우므로, 여기서 멈추면 "라이다가 잠깐 끊겨 시험이 안 끝났다"
  가 된다(`pedal_drive_node` 와 같은 판단).

> ⚠️ **라이다 ROI 는 전방 2.0~6.0 m 인데 10펄스 2단 정지거리가 12.9~20.4 m 다.**
> **보고 나서 서기에는 원리적으로 부족하다** — 이 시험은 "얼마나 못 서는가" 를
> 재는 것이지 "설 수 있는가" 를 보는 것이 아니다. **사람·부술 물건을 장애물로
> 쓰지 말 것.** 결과 표에 `장애물까지 남은 여유` 가 음수로 찍히면 그것이
> "들이받았을 거리" 다.

- **`driving` 을 쓰지 않는다** — 코너 감속·종점 접근제동·CTE 적분·라이다 이양이
  전부 '제동거리를 재는 일' 에 개입한다. **재려는 것 하나만 남긴 노드**가
  `white1/white1/braketest.py` 다.
- **런치 = 출발.** 준비되면 스스로 헤딩을 잡고 굴러간다
  (`auto_start:=false` → `/braketest_go` 를 기다린다).
- 속도는 **`braketest.py` 상단 두 값**이 전부다 : `DRIVE_PULSE = 10` /
  `DRIVE_PWM = 0`. 런치 인자가 이긴다.
- **완전정지하면 리니어를 푼다**(`BRAKE_RELEASE_WAIT_S = 1.5 s` 기다린 뒤 종료).
  물린 채로 런치를 내리면 arduino 가 없어져 D5 를 내렸다 올리는 것 말고는 풀
  방법이 없다.

> ### ⚠️ 안전 — 기본 10펄스는 **31.8 km/h** 다
>
> | 2단 감속도 | 순수 제동거리 | + 지연 0.30 s | **합** |
> |---|---|---|---|
> | 2.2 (실측 하한) | 17.8 m | 2.7 m | **20.4 m** |
> | 3.8 (호조건) | 10.3 m | 2.7 m | **12.9 m** |
>
> **S 지점 뒤로 최소 30 m 가 비어 있어야 한다.** 경로가 S 에서 끝나면 안 된다 —
> 이 노드는 S 를 지나 **멈출 때까지 계속 달린다.** 앞에는 헤딩 초기화(1~2 m)와
> 가속 구간이 더 붙는다. `braketest` 가 시작 전에 S 뒤 잔여거리를 재서 30 m
> 미만이면 경고한다(진행은 막지 않는다).
>

**재는 값** : **정지 사유**, 체결 시 속도 `v0`(GPS 변위속도), **체결 시 장애물거리**
(라이다 정지일 때), 제동거리 `d`, 제동시간 `t`, 평균 감속도를 **두 가지
방법**(`v0/t` 와 `v0²/2d`)으로, 그리고 **트리거 초과거리**.
두 감속도가 크게 다르면 대개 '완전정지' 판정 시각이 늦은 것이다.
정지 판정은 **GPS 와 엔코더를 둘 다** 요구한다 — 엔코더 단독은 기동 블랭킹의
허수 카운트(실측 중앙 16, 최대 34)에 끌려간다.

> ### ★`auto_direct_pwm` — 자율 구간 직접 PWM (기본 꺼짐)★
>
> A보드 `handleLine` 규약상 **단일값은 무조건 펄스 모드(0~15)** 이고, 직접 PWM 은
> **`"vL,vR"` 콤마 2값**에서만 된다. `arduino.py` 는 종전에 **수동조종 분기에서만**
> 콤마를 냈다. 그래서 `arduino` 에 **`auto_direct_pwm`(기본 `False`)** 을 새로 두고,
> `braketest.launch.py` 만 그것을 켠다.
>
> - 켜져 있어도 `linear.x` 가 0~15 면 **종전과 완전히 같다.** 16~255 일 때만
>   콤마 2값으로 나간다.
> - ⚠️ **무보호 경로다** — A보드에서 PID·슬루레이트·폭주감지·**기동 블랭킹**이
>   전부 빠진다(`applySide` 의 `DRIVE_PWM` 분기에 `launching[idx] = false` 가 있다).
> - `one_launch.py` 는 이 값을 켜지 않는다.

---

## 6. 수정 사항 — ★2026-09-07 구현 완료★

> ### 구현 결과 요약 (`colcon build` 8패키지 통과)
>
> | 항목 | 결정 | 상태 |
> |---|---|---|
> | 6.1 `terrain` `S` 일시정지 | **지시 그대로 즉시 2단** | ✅ 구현 |
> | 6.2 6펄스 상향 | **상한만 6, 런치 기본은 4로 두고 단계 확인** | ✅ 구현 |
> | 6.3 종점 5 m 감속 | **5 m 지점에서 감속 시작** | ✅ 구현 |
> | 6.4 mppi 이식 · `/lstatus` · 인계 서행 | **`/lstatus` 가 `/lidar_permit` 을 대체** | ✅ 구현 |
>
> **바뀐 파일** — `white1/white1/{driving,iahrs,record,hud}.py` ·
> `white1/launch/one_launch.py` · `mppi_local_planner/{src/mppi_local_planner_node.cpp,
> config/params.yaml,README.md}` · `{lidar,mppi_local_planner}/include/.../kasa_units.hpp` ·
> `lidar/README.md`
>
> **아래 6.1~6.4 는 설계 근거와 검산의 기록으로 남긴다.** 값을 다시 만질 때
> "왜 이 숫자인가" 가 전부 여기 있다. 각 절 머리의 ★구현★ 표시가 실제 코드 위치다.
>
> ### ⚠️ 아직 실차에서 확인하지 않았다 — 전부 책상 검증(단위 테스트 + 빌드)뿐이다
>
> 확인 순서와 볼 항목은 6.2⑦ · 6.3⑧ · 6.4⑧ 에 있다. 특히:
> - **`corner_min_pulse`(코너 하한)** 은 `drive_pulse` 의 절반으로 유도된다 —
>   4펄스에서는 2(종전과 같다), 6펄스에서 3. **6펄스 코너 3펄스는 미검증이다**
>   (사람은 매핑할 때 코너를 0.44~0.66 m/s 로 돌았다).
> - **`/lstatus` 교체 직후 발행자가 하나뿐인지** 반드시 확인 :
>   `ros2 topic info /cmd_vel_raw --verbose`
> - **S 정차 위치**는 6펄스에서 S 를 5~8 m 지나서 선다(설계대로). 정지선처럼
>   쓰려면 사람이 S 를 그만큼 앞당겨 적는다.

### 6.0 구현된 것 — 코드 위치

| 무엇 | 어디 |
|---|---|
| `MAX_PULSE_LIMIT = 6` (기본 `DRIVE_PULSE`·런치는 4) | `driving.py` 상수절 |
| `corner_min_pulse = min(dp, max(2, round(dp/2)))` | `driving.py __init__` |
| `MIN_SPEED_RATIO = 1/2` · `CURVE_PREVIEW_FAR_MAX = 20` · `LFD_MAX_M = 7.8` | `driving.py` 상수절 |
| `decel_dist(v0,v1,a,lag)` — `stop_dist` 의 일반화 | `driving.py` |
| 종점 : `GOAL_DECEL_M=5.0` · `GOAL_HOLD_PULSE=2` · `GOAL_HOLD_ENC_PULSE=2` · `GOAL_KICK_PULSE=3` | `driving.py` |
| `goal_approach` 체결=고정 5 m, `need1/need2` 를 유지펄스 기준으로 재기준화 | `driving.py goal_approach` |
| 해제 조건에 `/encoder ≤ 2펄스` 추가 (GPS·시간상한과 OR) | `driving.py _goal_to_creep` |
| S : `STOP_ZONE_CHARS` · `SP_*` 서브페이즈 · `stop_zone_hit_in` · `begin_stop_zone` · `run_stop_zone` | `driving.py` |
| S 검증 로그 (연속 S · L 안의 S) | `driving.py _warn_stop_zones` |
| `_zone_segments(zone, chars)` — L/S 겸용 | `driving.py` |
| `/lstatus` 발행 : `lstatus_now()` · `publish_lstatus()` | `driving.py` |
| 인계 서행 : `LIDAR_DECEL_M` · `lidar_zone_left_m` · `lidar_approach` · `_lz_release` | `driving.py` |
| 구간 표 사전계산 (`_lz_starts` · `_stop_idx`) | `driving.py build_waypoints` |
| 소유권 순서 : 종점 > S > L인계 > 코너 | `run_follow` + `corner_brake` 의 `ok` |
| `/lstatus` 구독 · `cruise_pulse` 단일화 | `mppi_local_planner_node.cpp` |
| `cruise_pulse: 2` 최상단 · `handover.require_lstatus` | `mppi .../params.yaml` |
| `PULSE_OPERATING_MAX 4 → 6` (두 패키지) | `*/kasa_units.hpp` |
| `lidar_speed` 인자 삭제, `lidar_pulse` 하나로 | `one_launch.py` |
| `/lstatus` 기록 열 · HUD 표시 | `record.py` · `hud.py` |
| HUD 큰 숫자를 ★GPS 속도★ 로 (`/gps_fused[8]`, 폴백 IMU→ENC) [2026-09-09] | `hud.py _draw_speed` |
| IMU 중력축 투영 (4.7절) [2026-09-09] | `driving.py cb_imu` · `solve_imu_axis` |
| 브레이크 제동거리 측정 (5.1절) [2026-09-09] | `braketest.py` · `braketest.launch.py` |
| braketest 정지 트리거를 ★라이다+종점★ 으로 (terrain 무시) [2026-09-09] | `braketest.py` · `aeb.launch.py` include |
| 자율 직접 PWM 게이트 `auto_direct_pwm` [2026-09-09] | `nxde/arduino.py` |
| `from white import ports` → `from white1` | `iahrs.py` |

**책상 검증 결과** (단위 테스트로 실제 확인한 값)

```
_zone_segments(z)               → [(2,4), (9,9)]        L 구간
_zone_segments(z, S)            → [(6,6), (11,11)]      S 단일행
decel_dist 4펄스→2펄스 a=1.30    → 5.55 m   (정지기준이면 6.75 m)
decel_dist 6펄스→2펄스 a=1.30    → 12.54 m  (정지기준이면 13.74 m)
corner_min_pulse                → dp 4→2 / 5→3 / 6→3
stop_zone_hit_in(prev=3, idx=8) → 6        ★5칸 점프해도 단일행 S 를 잡는다★
  같은 S 재발동                  → None
S 시나리오  2단 → enc0 → 3.5s → 0단 → 1.0s → ★3펄스 + 조향 -7.5 유지★ → 복귀
lidar_approach  남은 4.75m       → pulse 0 + 1단, enc 1.8펄스 도달 → pulse 2 로 해제
lstatus_now  DRIVE_DONE/IDLE     → '0'      ★선 차를 mppi 가 못 몬다★
mppi cruise_pulse:=2 → 1.77 m/s (2 pulse) / :=3 → 2.65 m/s (3 pulse)
```

---

## 6bis. 설계 근거와 검산 (기록)

> **아래 둘은 아직 구현하지 않았다.** 여기 적은 것은 **구현 설계와 검산**이다.
> ★결정이 필요했던 항목은 6절 머리의 표에 결과가 있다.★ 남은 판단은 실차 확인뿐이다.

### 6.1 `terrain` 열에 `S`/`s` 추가 — 일시정지 후 재출발

#### 요구사항 (사용자 지시 그대로)

- `terrain` 열에 `L`/`l`(라이다) 외에 **`S`/`s`(일시정지 후 출발)** 를 추가한다.
- **`L` 은 여러 행에 걸쳐 연속으로, `S` 는 행 하나에만** 적는다.
- **그 행에 도달하거나 지나치는 즉시 리니어 2단 체결로 정지.**
- **A보드 양 펄스가 모두 0 이 된 후 3.5 초 뒤 리니어 0단, 그다음 재출발.**
- 전체 CSV 에서 `S` 는 **1~2 군데**.

#### ① 열 파싱 — 고칠 것이 거의 없다

`select_route()`(`driving.py:1678`)는 이미 `terrain` 열 문자열을 그대로
`self.raw_zone` 에 담고, `build_waypoints()`(:1734)가 `self.wp_zone` 으로 옮긴다.
`zone_at()`(:1511)은 문자열을 돌려주고 **`in_lidar_zone()` 만 `'L'/'l'` 로 좁힌다.**
→ **`'S'` 는 지금도 이미 파싱되어 `wp_zone` 에 들어와 있고, 라이다 판정에는 안 걸린다.**

고칠 곳은 로그 문구뿐이다 — `select_route` 가 지금 L 구간 개수·인덱스를 찍는데
(`:1703-1717`), 여기에 **S 의 개수와 인덱스 목록**을 함께 찍게 한다. 사람이 손으로
적는 열이라 **오타 하나가 조용히 무시되는 것**이 제일 위험하다.

```python
STOP_ZONE_CHARS = ('S', 's')     # LIDAR_ZONE_CHARS 옆에 (driving.py:963 근처)
```

⚠️ **`select_route` 에서 검증까지 할 것**:
- `S` 가 **연속 2행 이상**이면 경고(요구상 한 행이다 — 사람이 실수로 끌었을 수 있다).
  연속이면 **첫 행만 소비**하고 나머지는 무시하는 것이 안전하다.
- `S` 가 **`L` 구간 안에** 있으면 **거부하거나 강한 경고**. L 구간에서는 driving 이
  침묵하므로 S 가 절대 발동하지 않는다 → **"적었는데 안 선다"** 가 된다.
- `S` 가 **경로 마지막 `GOAL_ZONE_M(6 m)` 안에** 있으면 경고 — 종점 접근제동과
  소유권이 겹친다(아래 ④).

#### ② 통과 감지 — ★"같은가" 로 보면 놓친다★

`advance_wp_idx()`(:2962)는 **한 틱에 최대 `WP_MAX_ADVANCE = 5` WP 까지 건너뛴다.**

- 정상 주행 : 20 Hz, 4펄스(3.54 m/s) → 한 틱 0.177 m. WP 간격 0.23~0.43 m 이므로
  보통 0~1칸 전진. **6펄스(5.30 m/s)면 한 틱 0.265 m** — 여전히 1~2칸.
- 그러나 GPS 가 튀거나 라이다 구간에서 복귀한 직후에는 **5칸까지 뛴다.**
- 따라서 `wp_idx == S_index` 로 보면 **조용히 놓친다.**

**→ 구간 검사로 만든다.** 직전 틱의 포인터를 들고 있다가, 이번 틱에 지나온 범위
`(prev_idx, wp_idx]` 안에 `S` 가 있으면 발동한다:

```python
# run_follow() 의 advance_wp_idx() 직후
prev = self._wp_idx_prev
self._wp_idx_prev = self.wp_idx
hit = next((i for i in range(prev + 1, self.wp_idx + 1)
            if self.zone_at(i) in STOP_ZONE_CHARS
            and i not in self._stop_consumed), None)
```

- **`_stop_consumed`(set) 로 소비한 인덱스를 기록한다** — 안 하면 정지 후 재출발
  직후 같은 인덱스를 다시 보고 무한 정지한다. 포인터는 단조 증가이므로
  `_stop_last_idx` 하나(정수)로도 충분하다.
- **초기화는 `enter()` 한 곳에서**(상태가 바뀌면 무조건 처음으로) — 기존
  `_goal_phase`·`_cb_state` 와 같은 자리에 넣는다(`driving.py:1859-1890`).
- **놓침 방벽** — 요구사항에 "S 한 번 놓친 것도 어찌저찌 감지하겠지" 라고 되어 있는데,
  위 구간 검사는 **포인터가 그 인덱스를 넘어서기만 하면 반드시 잡는다.** 포인터가
  아예 그 구간을 건너뛰는 경우는 없다(단조 증가 + 최대 5칸이므로 범위에 포함된다).

#### ③ ★결정 : 지시 그대로 즉시 2단★ — 정차 위치 오차를 받아들인다

요구는 **"도달·통과 즉시 2단"** 이다. 그런데 2단 정지거리는:

| 속도 | 2단(a=2.2, 하한) | 2단(a=3.8, 호조건) |
|---|---|---|
| 4펄스 3.54 m/s | **2.84 m** | 1.6 m |
| **6펄스 5.30 m/s** | **6.39 m** | 3.7 m |

즉 **S 행을 2.8~6.4 m 지나서 선다**(+ 판정·행정 지연 0.30 s × v = 1.1~1.6 m).

- **(a) "여기서부터 멈추기 시작"** 이면 지금 설계 그대로 두면 된다.
- **(b) "S 행에 정지선처럼 맞춰 서야 한다"** 면 **선행제동이 필요하다.**
  그때는 `goal_approach()`(:2398)를 그대로 본떠 **S 지점을 임시 종점으로 취급**해
  `need1`/`need2` 를 계산하고 1단 → 2단으로 접근하는 편이 맞다. 그 함수가 이미
  '창 진입 → 1단 → (늦었으면) 2단 → 크립' 을 검증된 형태로 갖고 있다.

**★결정 (사용자) : (a)★** — 지시 그대로 즉시 2단이다. 정차 위치가 S 뒤로 5~8 m
(6펄스) 밀리는 것을 받아들이고, **`S` 를 적는 위치를 사람이 그만큼 앞당겨 적는다.**
실차에서 오차를 재고 나서 필요하면 (b)를 얹는다 — `approach()` 가 이미 있으므로
그때는 `lidar_approach` 와 같은 형태로 한 함수를 더 부르면 된다.

#### ④ ★상태 설계 — 새 state 를 만들면 안 된다★

`enter()`(`driving.py:1911-1925`)의 마지막 블록:

```python
if new_state == S_DRIVE_DONE:
    self.send(0, 0.0, control=True)
    self.set_brake(BRAKE_FULL)
else:
    self.set_brake(BRAKE_NONE)      # ★DRIVE_DONE 이 아닌 모든 상태에서 무조건 해제★
```

→ **`S_DRIVE_PAUSE` 같은 새 state 를 만들면 진입하는 순간 리니어가 풀린다.**
이 규칙은 "리니어를 물리는 곳이 DRIVE_DONE 하나뿐" 이라는 안전 불변식이라
**깨지 말 것**(특히 수동조종 진입 = 사람이 차를 넘겨받는 순간).

**→ `goal_approach` · `corner_brake` 와 같은 "DRIVE_RUN 안의 서브페이즈" 로 만든다.**
이 파일에 이미 같은 패턴이 둘 있다(`_goal_phase`, `_cb_state`).

```python
SP_NONE, SP_BRAKING, SP_WAIT, SP_RELEASE, SP_RESUME = 0, 1, 2, 3, 4

self._sp_state   = SP_NONE
self._sp_idx     = -1      # 지금 처리 중인 S 의 WP 인덱스
self._sp_t       = 0.0     # 단계 진입 시각
self._sp_zero_t  = 0.0     # 엔코더 0 이 이어지기 시작한 시각  ★_goal_still_t 와 공유 금지★
self._sp_kicks   = 0
```

> **`_goal_still_t` 를 재사용하지 말 것** — 종점 접근이 같은 변수를 쓰고 있어서
> 섞이면 종점 판정이 오염된다(`_goal_stopped()`:2538).

**`run_follow()` 안의 배치 순서** (`driving.py:2270-2396`) :

```
advance_wp_idx()
  ↓
[라이다 구간 판정]        ← ★S 검사는 이보다 뒤★ (L 구간에서는 driving 이 침묵한다)
  ↓
CTE 이탈 판정             ← 정지 중에도 유효하게 두는 편이 안전
  ↓
종점 판정
  ↓
★S 서브페이즈★  ─ 활성이면 여기서 send() 하고 return
  ↓
scan_curve / lookahead / corner_speed / goal_approach / corner_brake / send
```

**소유권 규칙** (기존 두 페이즈와 같은 규칙을 그대로 적용) :

- S 발동 조건에 **`self._goal_phase == GOAL_PHASE_NONE`** 을 넣는다 —
  종점이 리니어를 잡고 있으면 손대지 않는다.
- **`corner_brake()` 의 `ok` 조건(`driving.py:2768-2773`)에 `self._sp_state == SP_NONE`
  을 추가해야 한다.** 안 넣으면 코너 1단과 S 2단이 같은 리니어를 다툰다.
- 신호등(`tl_brake_req`)은 `_publish_brake` 가 `max()` 로 합치므로 2단이 이긴다 —
  추가 조치 불필요.

#### ⑤ 각 단계의 동작

**SP_BRAKING** (S 를 지나온 그 틱에 진입)

```python
self._sp_state = SP_BRAKING
self._sp_t = now
self._clear_transients() 는 부르지 말 것   # 그쪽은 set_brake(NONE) 을 한다
self.set_brake(BRAKE_FULL)
self._publish_brake(force=True)
self.send(0, steer, control=True)
self.event(f"🛑 일시정지 지점 통과 — WP {hit} (terrain='S'), 리니어 2단")
```

- **★결정 : 유지한다 (구현됨)★** `run_stop_zone(self._last_steer)` 로 정지 직전 각을
  계속 내보낸다(`send()` 가 마지막 각을 `_last_steer` 에 기억한다).
  `enter(S_DRIVE_DONE)` 은 조향 0 을 내지만
  그건 '경로를 버리는' 경우다. **여기서는 다시 출발하므로 마지막 조향각을 유지하는
  편이 재출발 궤적에 낫고**, 정지 상태에서 조향 기구를 돌리는 것은 부하가 크다.
  → **순수추종을 계속 계산해 보내되 `pulse` 만 0** 을 권한다.
- `keep_brake()` 는 `loop()` 가 이미 매 틱 부르므로 2단이 유지된다.

**SP_WAIT** — "A보드 양 펄스가 모두 0"

- driving 이 볼 수 있는 것은 **`/encoder`(좌+우 **합**)** 뿐이다. 양쪽 다 음수가 아니므로
  **합 == 0 ⇔ 양 펄스 모두 0** 이다. 즉 `self.enc_pulse` 로 요구를 정확히 판정할 수 있다.
- ⚠️ **문턱을 어디에 둘 것인가** — `self.enc_pulse = median3(합) × 0.5` 이고
  기존 `ENC_STOP_EPS = 0.5` 는 `>` 비교라 **합이 1 이어도 '정지'로 본다.**
  - 엄격히 "둘 다 0" 이면 `self.enc_pulse <= 0.0`.
  - 그러나 **중앙값 3점 필터가 단발 허수를 이미 죽이고**, `_check_done_release()` 의
    실측(허수 1펄스가 튀어 해제가 2.0 s → 4.35 s 로 늦어진 사례)을 보면
    **`ENC_STOP_EPS` 를 그대로 쓰는 편이 실용적**이다.
  - **권고** : `ENC_STOP_EPS` 재사용 + **`GOAL_STOP_HOLD_S(0.5 s)` 동안 이어질 것**을
    요구(`_goal_stopped()` 와 같은 방식, 변수만 `_sp_zero_t` 로 분리).
- 완전정지가 확인된 시각부터 **`STOP_HOLD_AFTER_ZERO_S = 3.5 s`** 를 잰다.
- **시간 상한 방벽** — 엔코더가 영영 0 이 안 되면(센서 이상) 굳는다.
  **★구현 : `STOP_WAIT_MAX_S = 12.0 s` — 넘으면 경고를 찍고 그대로 대기 단계로
  넘어간다★**(세우지 않는다). 로그에 "엔코더 허수 카운트를 의심할 것" 을 함께
  남긴다. 세워 버리면 사람이 손으로 옮겨야 하고, 이미 2단으로 물려 있어 차는
  서 있는 상태이므로 **대기로 넘기는 쪽이 안전하다.**
  재출발이 안 되는 경우는 `STOP_RESUME_MAX_S = 6.0 s` 가 받아 `S_DRIVE_DONE` 으로 넘긴다.

**SP_RELEASE** — 리니어 0단

```python
self.set_brake(BRAKE_NONE)
self._publish_brake(force=True)      # ★force 필수★ — 0 은 평소 재확인하지 않는다
```

⚠️ **0단은 즉시 풀리지 않는다.** B보드는 0단을 **위치를 보지 않고 REV 로
`BRAKE_HOME_MS = 1000 ms`** 돌린다(`kasa_0904_B.ino:297`). 게다가 arduino 에
`BRAKE_RELEASE_HOLD_S = 0.5 s` 유예가 있다. **브레이크가 아직 밟혀 있는 채로 구동을
걸면 서로 민다.**

→ **해제 후 최소 1.0 s 동안 펄스 0 을 유지**하고 그다음 재출발한다. 사용자 요구
("3.5초 후 리니어 0단 후 재출발")와 순서가 같으므로 모순되지 않는다.
종점 크립이 같은 성격의 상수를 이미 갖고 있다 —
`GOAL_RELEASE_RETRACT_S = 0.54`(1단→0단 290카운트 후퇴),
`GOAL_RELEASE_TORQUE_S = 0.35`(A보드 PWM 슬루로 토크 복귀). **2단은 행정이 540카운트라
더 길다** → `BRAKE_HOME_MS` 값인 1.0 s 를 쓰는 것이 맞다.

**SP_RESUME** — 재출발

- ⚠️ **4펄스로 떼지 말 것.** A보드 적분 동결 함정(`abs(err) < 4`)에 정확히 걸린다.
- **권고 : 3펄스로 떼고**(err=3 → 적분 누적 O, `PULSE_RESTART_SAFE`), 실측이
  움직이기 시작하면(`measured_kmh() ≥ GOAL_KICK_KMH`) 평소 규칙으로 돌려준다.
- `creep_pulse()`(:2585)의 킥 구조가 정확히 "정지한 차를 떼어낸다" 를 하는 코드다 —
  **킥 펄스만 3 으로 바꿔 재사용**하는 것이 가장 검증된 길이다.
- **실패 방벽** — `GOAL_KICK_MAX_N(3)` 회 시도해도 안 움직이면 구동을 끊고
  `S_DRIVE_DONE` 으로 넘긴다("안 움직이는 차에 전류를 계속 넣지 않는다").
- 재출발 성공 → `_sp_state = SP_NONE`, `_stop_consumed.add(hit)`.
- **CTE 적분을 지운다**(`reset_cte_integral()`) — 정지 중 누적이 출발 첫 틱에 실리면
  한쪽으로 튄다. `enter()` 가 상태 전이에서 하는 것과 같은 이유.

#### ⑥ 매핑 CSV 에 `S` 를 적는 방법 (사람이 하는 일)

`mapping.py:197` 이 `terrain` 에 **항상 `"0"`** 을 쓴다. 손으로 고쳐야 한다.

- **열 위치가 밀리면 조용히 GPS 로 떨어진다** (에러가 안 난다). → **행 번호를 주면
  `terrain` 열만 바꿔 주는 작은 도구를 만들어 두는 것을 권한다.**
- 참고 CSV 두 개(`route_20200101_111111.csv` / `route_20200202_222222.csv`)는
  **`terrain` 에 이미 구 스택의 지형코드(−1/0/1)가 들어 있다**(3.4절 표). 지우고 적을 것.
- **WP 간격이 0.23~0.26 m** 이므로 "몇 미터 앞" 을 행 수로 환산하려면 **× 4 행/m** 다.

#### ⑦ 함께 봐야 할 것

- `hud.py` 는 CSV 에서 `latitude/longitude` 만 읽는다 — S 를 화면에 찍으려면
  `hud.py:469` 의 로더에 `terrain` 을 추가해야 한다.
- `nxde/sound.py` 에 일시정지 안내음을 넣을 수 있다(`/drive_event` 구독).
- `record.py` 는 `/drive_state`·`/drive_event` 를 이미 기록하므로 별도 열은 불필요.

---

### 6.2 기본 주행속도 6펄스 · 코너 감속 3펄스

#### 현재 상태

```
DRIVE_PULSE      = 4      (driving.py:285)   ≈ 12.7 km/h
MAX_PULSE_LIMIT  = 4      (driving.py:291)   ★절대 상한 — 여기서 잘린다★
CORNER_MIN_PULSE = 2      (driving.py:476)   ≈  6.4 km/h
```

**목표** : 기본 **6펄스 = 5.304 m/s = 19.09 km/h**, 코너 **3펄스 = 2.652 m/s = 9.55 km/h**.

#### ① 고쳐야 할 곳 (전수)

| 파일:줄 | 상수 | 현재 | → | 안 바꾸면 |
|---|---|---|---|---|
| `driving.py:285` | `DRIVE_PULSE` | 4 | **6** | 기본값만. 런치가 덮는다 |
| **`driving.py:291`** | **`MAX_PULSE_LIMIT`** | **4** | **6** | ★**여기서 잘린다**★ 파라미터로 6 을 줘도 4 로 잘린다 (`__init__`:1155 `min()`, `send()`:3225 `min()`) |
| `one_launch.py:292` | `drive_pulse` 기본 | `'4'` | `'6'` | 런치가 4 를 덮어쓴다 |
| **`driving.py:476`** | **`CORNER_MIN_PULSE`** | **2** | **3** | 코너가 2펄스로 내려간다 (아래 검산) |
| `driving.py:467` | `MIN_SPEED_RATIO` | 1/2.8 | **1/2** | 연속값 `v_corner` 와 실제 지령 펄스가 어긋난다 (아래) |
| `driving.py:401` | `LFD_MAX_M` | 6.4 | **7.8** | ω_n 이 0.97 → 1.17 로 올라간다 |
| **`driving.py:456`** | **`CURVE_PREVIEW_FAR_MAX`** | **14.0** | **18~20** | ★**제일 중요**★ 코너를 보기 전에 물어야 한다 (아래) |
| `driving.py:791` | `GOAL_KICK_PULSE` | 4 | **3** | A보드 적분 동결 함정 (지금도 함정이다) |
| `kasa_units.hpp:89` | `PULSE_OPERATING_MAX` | 4 | **6** | lidar/mppi 노드의 상한. white1 과 갈라지면 L 구간에서 속도가 뚝 떨어진다 |
| `one_launch.py:332` | `goal_brake_m` | 20.0 | 재검토(아래) | 여유가 5.8 m 로 줄어든다 |
| `driving.py:464` | `BRAKE_GATE_DECEL` | 2.0 | 검토 → 1.3 | 낙관이 더 커진다 (아래) |

#### ② 코너 3펄스가 나오게 하는 법 — 검산

`corner_speed()`(:2657)의 마지막 두 줄(`:2724-2725`):

```python
pulse = int(v_target / MS_PER_PULSE + 0.5)
pulse = max(min(CORNER_MIN_PULSE, self.drive_pulse), min(self.drive_pulse, pulse))
```

`drive_pulse = 6` 일 때 속도 대역은:

```
max_speed_ms = 6 × 0.884            = 5.304 m/s
min_speed_ms = max(0.9, 5.304/2.8)  = 1.894 m/s   ← 현행 MIN_SPEED_RATIO
```

요구 도로휠각이 15° 이상(최대 감속)일 때 `v_target = 1.894` →
`int(1.894/0.884 + 0.5) = int(2.64) = 2` → **2펄스**. 목표(3)에 미달한다.

**두 가지를 함께 고친다:**

1. **`CORNER_MIN_PULSE = 3`** — 하한을 직접 올린다. 같은 클램프가
   `_cb_cap()`(`driving.py:2827`)에도 있으므로 **두 경로가 한 상수로 함께 따라온다.**
2. **`MIN_SPEED_RATIO = 1/2`** — `5.304 × 0.5 = 2.652 m/s = 정확히 3펄스`.
   **이것도 같이 고쳐야 하는 이유** : `corner_speed` 가 `corner_brake` 에 넘기는
   `gate_v_corner` 는 **연속값(m/s)** 이다. 하한만 정수로 올리고 연속값을 1.894 로
   두면, **`corner_brake` 는 존재하지 않는 목표(2.1펄스)를 겨눠 제동거리를 계산하고
   REF 캡도 그 값으로 건다** — 실제 지령(3펄스)보다 낮은 곳을 겨누므로 **필요 이상으로
   일찍·오래 문다.**
   - `MIN_SPEED_FLOOR = 0.9` 는 이제 걸리지 않는다(2.652 ≫ 0.9). 그대로 둬도 무해.

#### ③ ★가장 중요한 발견 — 스캔창 14 m 로는 6→3 감속을 못 건다★

`corner_brake()`(:2801-2810)의 체결 판정:

```
d_need  = (v_now² − v_corner²) / (2 × A_BRAKE1_MS2)
체결지점 = d_need × CORNER_BRAKE_LATE_K(1.2) + BRAKE_GATE_MARGIN(1.5)
```

| 감속 | a | `d_need` | 체결지점 |
|---|---|---|---|
| 현행 4→2펄스 | 0.88 (보수) | 5.33 m | 5.33×1.2+1.5 = **7.89 m** |
| **6→3펄스** | 0.88 (보수) | **11.99 m** | 11.99×1.2+1.5 = **★15.9 m★** |
| 6→3펄스 | 1.30 (1단 실효) | 8.12 m | 8.12×1.2+1.5 = 11.24 m |
| 6→3펄스 | 0.41 (코스트) | 25.73 m | — (코스트만으로는 불가능) |

**`CURVE_PREVIEW_FAR_MAX = 14.0 m` 이므로, 15.9 m 앞에서 물어야 하는 코너를
"아직 못 본" 상태다.** → 체결 조건 `d_avail > d_need × 1.2` 가 성립하는 순간이
아예 오지 않거나, 와도 이미 늦다. 결과는 **2026-08-11 실차와 같은 실패** —
코너 진입속도가 목표를 넘고 언더스티어로 밀린다.

**→ `CURVE_PREVIEW_FAR_MAX` 를 최소 16 m, 권장 18~20 m 로 늘려야 한다.**

- 비용은 무시할 만하다. 곡률 프로파일(`wp_req_win`/`wp_req_peak`)은
  `build_curve_profile()`(:1751)이 **경로당 한 번만** 계산하고, 매 틱 도는 것은
  `scan_curve_demand()`(:2634)뿐이다. `CURVE_SCAN_STEP_M = 0.4 m` 이므로 20 m 는
  50점 — 20 Hz 에서 무해.
- ⚠️ 스캔창을 늘리면 **먼 코너를 더 일찍 보게 되어 순항속도가 떨어질 수 있다.**
  `BRAKE_GATE_DECEL` 과 함께 실차에서 봐야 한다.

**`BRAKE_GATE_DECEL = 2.0` 도 재검토 대상이다.** 이 값은 `corner_speed` 의 (b) 게이팅
`v_brake = √(v_corner² + 2a·d)` 에 쓰이는데, **2.0 은 2단에서만 참인 낙관값**이다
(`BRAKING.md` 3절). 6펄스가 되면 그 낙관의 절대 오차가 커진다.
`corner_brake` 의 REF 캡이 제동 대상 코너에 대해서는 보정하지만,
**`v_corner = None`(스캔창 밖) 인 구간에는 캡이 없다.**
→ **1단 실효값 1.30 으로 내리는 것을 검토**(게이트가 더 일찍 감속을 명령한다).

#### ④ LFD — 상한 6.4 m 는 6펄스용이 아니다

설계식 `LFD = v·√2 / ω_n`:

| 펄스 | v | 설계 LFD | 현행 상한 6.4 m 에서의 실효 ω_n | PM |
|---|---|---|---|---|
| 4 | 3.536 | 5.16 m | 0.97 (상한에 안 걸림) | +37° |
| 5 | 4.420 | 6.44 m | 0.97 (딱 상한) | +37° |
| **6** | **5.304** | **7.73 m** | **6.4 m 로 잘려 ω_n = 1.17** | **+31°** |

`LFD_MAX_M = 6.4` 는 주석대로 **"구 표의 마지막 행 = 5펄스"** 값이다. 6펄스에서는
설계식이 상한에 잘려 **ω_n 이 0.97 → 1.17 로 올라간다.**

- **발산은 아니다** — 금색차 임계는 ≈2.3 이고 PM 이 아직 +31° 다.
- 그러나 **설계 의도(ω_n 을 0.97 로 유지)와 어긋난다.**
- → **`LFD_MAX_M = 7.8`** 로 올려 6펄스에서도 설계식이 살아 있게 한다.
- ⚠️ LFD 를 늘리면 **코너 진입에서 안쪽을 자른다**(코너 컷). 그것을 막는 것이
  곡률캡(`LFD_CURVE_CAP_K = 1.5`, `lookahead_m` 안)이므로 함께 봐야 한다.

#### ⑤ A보드 쪽 검산 — 6펄스는 오히려 뜨기 쉽다

FF 테이블 2차보간 결과 (실제 펌웨어 코드로 계산):

| 목표 | FF PWM | 기동 직후 PWM(= FF + 0.4·err) | 적분 |
|---|---|---|---|
| 2 | 70.0 | 70.8 | 누적 O |
| 3 | 80.0 | 81.2 | 누적 O |
| **4** | 90.0 | **91.6** | **★동결★** (err 가 정확히 4) |
| 5 | 100.0 | 102.0 | ★동결★ |
| **6** | **107.3** | **109.7** | ★동결★ |

- **적분은 6펄스에서도 동결된다**(err=6 ≥ 4). 하지만 **FF 가 107 이라 4펄스(90)보다
  훨씬 세게 민다** — 즉 **정지에서 6펄스로 떼는 것은 4펄스보다 잘 된다.**
  실측이 3펄스를 넘어서면 err < 4 가 되어 적분이 붙는다.
- 기동 블랭킹은 `PWM_SLEW_MAX = +4/cycle` 로 FF(6)=107 까지 **27 cycle × 20 ms = 0.54 s**
  개루프 램프한다.
- 6펄스 = 19.1 km/h. 무부하 상한 42.2펄스/창 대비 여유 충분, PWM 170 중 65%.
- ⚠️ **그래도 4펄스는 어디에도 남기지 말 것** — `GOAL_KICK_PULSE = 4`(:791)가 지금
  정확히 그 함정 값이다. **3 으로 내린다.**

#### ⑥ 안전 재검토 — 정지거리가 1.5배가 아니라 2.25배가 된다

`d = v²/2a` 이므로 **속도 1.5배 → 정지거리 2.25배**다.

| 상황 | a | 4펄스 | **6펄스** |
|---|---|---|---|
| 코스트 | 0.41 | 15.25 m | **34.31 m** |
| 1단(보수) | 0.88 | 7.10 m | **15.98 m** |
| 1단(실효) | 1.30 | 4.81 m | **10.82 m** |
| **2단(하한)** | **2.2** | **2.84 m** | **★6.39 m★** |

**함께 봐야 하는 것:**

- **경로이탈 안전정지** (`CTE_DEVIATION_M = 2.0`) — 이탈을 감지한 뒤 2단으로
  **6.4 m 를 더 간다.** 문턱을 올리면 안 된다(세우는 쪽 판정이다).
- **신호등 정지** — `traffic_light` 가 빨간불에 2단을 요구한다. **정지선 인지거리가
  6.4 m + 인지지연을 감당하는지** `STOPLINE_TEST.md` 기준으로 재확인할 것.
- **종점 접근** (`goal_brake_m = 20.0 m`) — 6펄스에서:
  ```
  need1 = v·lag + v²/2a1 + margin = 5.304×0.55 + 10.82 + 0.5   = 14.24 m
  need2 = 1.2×(5.304×0.30 + 6.39) + 0.5                        = 10.10 m
  backstop 보정 : need1 = max(14.24, 10.10 + 2.92 + 1.0)       = 14.24 m
  ```
  20 m 창 안에 들어오지만 **여유가 5.8 m 뿐**이다(4펄스에서는 12.6 m 였다).
  → **`goal_brake_m` 을 25 m 로 올리는 것을 권한다.**
- **`WP_SEARCH_WINDOW = 45`**(≈11.25 m @0.25 m 간격) / **`WP_MAX_ADVANCE = 5`** —
  6펄스 한 틱 이동이 0.265 m 이므로 **그대로 둬도 충분하다.**
- **`REJOIN_PULSE_MAX = 2`**(라이다 복귀 재수렴 상한) — 재수렴은 저속이 맞다. **그대로.**
- **`REF_TRIM_REF_MAX = 3`**(저속 펄스 보정) — REF 6 에는 안 걸린다. **그대로.**
- **mppi 쪽 속도** — L 구간의 속도는 mppi 가 정한다. 6펄스로 올리면 **G↔L 경계에서
  속도 단차가 커진다**(설계상 A보드 PID 가 흡수하지만 폭이 커진다).
  ⚠️ mppi 소스가 지금 트리에 없다(0.1절).

#### ⑦ 실차 확인 절차 (권장)

**한 번에 4 → 6 으로 가지 말 것.** `drive_pulse` 는 런치 인자라 재빌드 없이 바꾼다.

```bash
ros2 launch white1 one_launch.py drive_pulse:=5      # 그다음 6
```

각 단계에서 `record` CSV 로 확인할 것:

1. **코너 진입 초과펄스** — `corner_speed` 목표 대비 실측(폐루프 시뮬 기준 +0.06 이면 정상,
   +1.8 이면 캡이 안 듣는 것)
2. **`max|CTE|`** 와 부호 편향, 조향 반전율
3. **코너 1단 제동** 체결 지점·횟수 (`/drive_event` 의 `🅑` 줄) — **한 코너에 한 번**인가
4. **종점 정지 오차** 와 `goal_phase` 전이
5. **`ω_n`** — LFD 와 실측 속도로 역산

> ⚠️ **`CORNER_MIN_PULSE = 2` 는 지금도 실차 미검증이다**(주석 참고). 그 로그들은
> 하한이 1 이던 코드로 달린 것이다. 3 으로 올리면 **저속 코너에서 차가 서 버리는
> 문제(1펄스에서 실측)는 확실히 사라지지만**, 반대로 **코너를 3펄스로 도는 것이
> 언더스티어 한계 안인지**는 새로 확인해야 한다 —
> 사람은 매핑할 때 코너를 **0.44~0.66 m/s 에 풀 락**으로 돌았다.
> 3펄스 = 2.652 m/s 는 그 4~6배다.

---

### 6.3 종점 감속 로직 개선 — 5 m 앞부터 2펄스

#### 요구사항 (사용자 지시 그대로)

- **종료지점으로부터 5 m 앞부터 2펄스로 감속한다.**
- 이때 **리니어 1단을 체결한다 — `/encoder` 가 2펄스가 될 때까지.**
- 그 뒤 **2펄스 속도로 가다가, 종료지점 도달 즉시 리니어 2단 체결.**

#### ① 현행 구조와의 차이

지금 `goal_approach()`(`driving.py:2398`)가 하는 일:

```
NONE ─(d2goal ≤ need1)─▶ BRAKE1 ─(못 센다)─▶ BRAKE2
  │                         └──(정지·저속·시간초과)──┐
  └─(이미 느리다)───────────────────────────────────▶ CREEP(1펄스) ─▶ 도착 → 2단
```

| | 현행 | **요구** |
|---|---|---|
| 감속 시작 지점 | **속도로 계산**(`need1 = v·0.55 + v²/2a₁ + 0.5`) | **고정 5 m** |
| 감속 목표 | 정지(1단으로 세울 수 있는가) | **2펄스** |
| 1단 해제 판정 | 실측 **속도**(`goal_release_kmh()`) · 완전정지 · 시간초과 | **`/encoder` 가 2펄스** |
| 접근 구간 속도 | **크립 1펄스** (`GOAL_CREEP_PULSE`) | **2펄스** |
| 도착 시 | 2단 (`enter(S_DRIVE_DONE)`) | 2단 — **현행 그대로** |

즉 **①체결 지점 ②해제 판정 ③접근 속도** 세 가지가 바뀌고, **도착 2단은 이미 그렇게
되어 있다**(`driving.py:1911-1919`, `set_brake(BRAKE_FULL)` + 조향 0 을 같은 틱에).

**2펄스로 바꾸는 것 자체는 개선 방향이 맞다.** 현행 크립 1펄스는
`GOAL_KICK_PULSE(4)` 로 밀어 주는 재출발 킥이 필요할 만큼 저속이고
("67초 정체" 실측), **2펄스는 A보드 적분 누적 조건(`err < 4`) 안이라 PID 가 정상으로
붙는다** — 이 차에서 가장 잘 구르는 구간이기도 하다(2026-08-11 실측:
2펄스 구간이 추종이 제일 좋았다, max|CTE| 0.17 m).

#### ② ★검산 — 5 m 는 4펄스 이상에서 부족하다★

1단으로 v₀ → 2펄스까지 줄이는 데 필요한 거리
(`= v₀ × 행정램프 0.55 s + (v₀² − v₁²) / 2a`, v₁ = 2펄스 = 1.768 m/s):

| 진입 속도 | 코스트 a=0.41 | **1단 a=0.88**(보수하한) | **1단 a=1.30**(구동차단 실효) | 2단 a=2.20 |
|---|---|---|---|---|
| 3펄스 (2.65 m/s) | 6.22 m | 3.68 m | **2.96 m** ✅ | 2.35 m |
| **4펄스 (3.54 m/s)** | 13.38 m | 7.27 m ❌ | **5.55 m** ❌ | 4.08 m |
| 5펄스 (4.42 m/s) | 22.44 m | 11.76 m ❌ | 8.74 m ❌ | 6.16 m |
| **6펄스 (5.30 m/s)** | 33.41 m | **17.13 m** ❌ | **12.54 m** ❌ | 8.60 m |

**→ 5 m 안에 2펄스로 내려오는 것은 3펄스 진입에서만 성립한다.**
현행 4펄스에서도 0.55 m 모자라고, **6.2절의 6펄스로 올리면 12.5 m 가 필요해 2.5배
부족하다.** 부족분은 그대로 **종점 통과 속도**로 남는다:

| 종점 통과 속도 | 2단 정지거리 (a=2.2) | (a=3.8) |
|---|---|---|
| **2펄스** (설계대로) | **1.24 m** | 0.94 m |
| 3펄스 | 2.39 m | 1.72 m |
| 4펄스 | 3.90 m | 2.71 m |
| 6펄스 | **7.98 m** | 5.29 m |

#### ③ ★결정 : 5 m 지점에서 감속 시작★

두 해석이 있고, 코드가 완전히 달라진다.

**(a) "5 m 앞에서 감속을 시작한다"** — 지시문 그대로. 구현은 제일 단순하지만
위 표대로 **종점을 2펄스보다 빠르게 지나고, 정지 위치가 종점 뒤로 1.2~8 m 밀린다.**
종점이 '대략 이 근처' 면 문제없고, 정지 위치를 봐야 하면 안 된다.

**(b) "종점 5 m 전에는 이미 2펄스여야 한다"** — 5 m 는 **2펄스 유지 구간의 길이**이고,
1단 체결은 그보다 앞(`5 m + 위 표의 거리`)에서 일어난다. 그러면 종점 통과가 항상
2펄스이므로 **정지 위치가 종점 +1.2 m 로 고정된다.**

**★결정 (사용자) : (a) — 5 m 지점에서 감속 시작★**
구현은 `goal_approach` 의 체결 조건을 `d2goal <= GOAL_DECEL_M(5.0)` ★고정★ 으로
바꾼 것이고, `need1` 은 진단으로만 남는다(필요 거리가 5 m 를 넘으면 로그에
"★N m 모자란다★" 를 찍는다). **대신 `need1`/`need2` 를 '정지' 가 아니라
'유지펄스(2)' 기준으로 재기준화해야 한다** — 안 하면 4펄스에서 종전 `need2`
5.18 m 가 5.0 m 보다 커서 **백스톱이 늘 먼저 걸려 설계가 무효가 된다**
(재기준 후 4.33 m). 그 재기준화를 위해 `decel_dist()` 를 신설했다.

> 참고 — 아래 (b) 는 채택되지 않았다. 기록으로 남긴다:
> `need1` 계산식의 목표속도만 `0`(정지)에서 `2펄스`로 바꾸고, 거기에 5 m 를 더하면 된다:
>
> ```python
> GOAL_HOLD_PULSE = 2          # 종점 접근 유지 속도 (종전 GOAL_CREEP_PULSE = 1)
> GOAL_HOLD_M     = 5.0        # 이 거리 안에서는 GOAL_HOLD_PULSE 로 간다
> v_hold = GOAL_HOLD_PULSE * MS_PER_PULSE
> # 2펄스까지 줄이는 데 필요한 거리 (정지거리가 아니다)
> d_slow = v * GOAL_BRAKE1_LAG_S + max(0.0, v*v - v_hold*v_hold) / (2 * self.goal_a1)
> need1  = GOAL_HOLD_M + d_slow + GOAL_BRAKE_MARGIN_M
> ```
>
> `stop_dist()`(:2507)는 정지(v₁=0) 전용이므로 **일반화하거나 별 함수를 둘 것**.
> 6펄스 기준 `need1 = 5 + 12.54 + 0.5 = 18.0 m` → **`goal_brake_m` 감시 창 20 m 안에
> 아슬아슬하게 들어온다. 6.2절의 25 m 권고와 함께 봐야 한다.**

#### ④ ★`/encoder` 로 해제 판정 — 이 경로는 세 번 사고가 났던 자리다★

요구는 **"`/encoder` 가 2펄스가 될 때까지 1단"** 이다. 방향 자체는 이 파일의 원칙에
맞다 — **체결은 기하로, 해제는 실측으로**(되먹임 금지). 그러나 **출처가 엔코더인 것이
문제다.**

- `driving.py` 는 2026-08-12 이후 **속도의 1순위를 GPS 로 옮겼고**, `corner_brake()` 는
  아예 **"GPS 속도를 못 읽으면 제동하지 않는다 — 엔코더 폴백으로는 절대 제동하지
  않는다"**(:2765-2767)로 못 박았다. 파일 헤더의 **리니어 브레이크 3차 실패**가
  엔코더 허수 카운트를 '과속'으로 읽은 것이 원인이었기 때문이다.
- **실측** : 지령 0~1펄스 구간에서 엔코더 중앙값이 **16**, 최대 **34** 까지 튀었다
  (정상 구간 4~5). 즉 **저속에서 엔코더는 위로 튄다.**
- → 그대로 구현하면 **"아직 2펄스보다 빠르다"고 오판해 1단을 계속 물고 차를 세운다.**
  그리고 선 차를 2펄스로 떼는 것이 이 차의 최대 난점이다.

**대응 (셋 다 넣을 것):**

1. **중앙값 필터는 이미 걸려 있다** — `self.enc_pulse` 는 `median3(합) × 0.5` 로
   **바퀴 하나 기준** 이다(`cb_encoder`:1482). 따라서 판정은
   `self.enc_pulse <= GOAL_HOLD_PULSE` 로 쓴다. **`/encoder` 원값(합)과 2를 직접
   비교하지 말 것** — 합 기준으로는 4 다.
2. **GPS 와 OR 로 묶는다** — `self.gps_ms()` 또는 `measured_kmh()` 가 2펄스 이하이면
   **엔코더가 뭐라 하든 푼다.** 해제 방향이므로 둘 중 하나만 성립해도 안전하다
   (`_goal_to_creep()`:2550 이 이미 "셋 다 푸는 쪽 판정" 이라는 같은 규칙을 쓴다).
3. **시간 상한을 반드시 둔다** — `GOAL_BRAKE1_MAX_S = 12.0 s` 가 이미 그 역할을 한다
   ("속도·엔코더가 전부 이상해도 리니어를 문 채 굳지 않는다"). **이 방벽을 없애지 말 것.**

**해제 선행량도 그대로 필요하다** — `goal_release_kmh()`(:2524)가
`a1×해제유예(0.5) + (a1+a0)/2×행정후퇴(0.54) + a0×토크복구(0.35)` 로 **"해제 명령 뒤에도
차는 한참 더 준다"** 를 보정한다. 목표가 0 이 아니라 2펄스가 되면 **이 식의 기준점도
2펄스로 바뀐다**(현재 `GOAL_CREEP_PULSE × MS_PER_PULSE + lead`). 상수만 바꾸면 따라온다.

#### ⑤ ★제동 중에 2펄스를 지령하면 저속 보정이 그것을 4펄스로 만든다★

`arduino.py:1619` :

```python
pulse = 0 if brake > 0 else self.cmd_pulse
```

**리니어가 물려 있는 동안 A보드로 나가는 주행값은 무조건 0 이다.** 즉 요구의
"2펄스로 감속하면서 1단 체결" 은 액추에이터 수준에서는 **"코스트 + 1단"** 이고,
2펄스 지령은 **해제되는 순간부터** 의미가 있다.

⚠️ **그래서 제동 중에 `ref = 2` 를 계속 내면 `low_speed_trim()`(:3296) 이 걸린다.**
그 함수는 `1 ≤ ref ≤ 3` 에서 `out = ref + clamp(ref − 실측펄스, −2, +2)` 를 얹는데,
제동 중에는 차가 실제로 줄어들므로 **`REF 2 인데 실측 0` → `out 4`** 가 되고,
**그 4 가 해제 순간 그대로 나가 제동으로 줄인 속도를 그 자리에서 되돌린다.**
코너 1단 제동이 명시적으로 막아 둔 함정과 **정확히 같은 것**이다(그쪽 주석 참고).

- 현행 코드는 이 함정에 안 걸린다 — `goal_approach` 가 **1단·2단 제동 중에는
  `return 0`** 을 하기 때문이다(`driving.py:2496`, `:2502`). `ref = 0` 이면 보정이
  통째로 우회된다.
- **→ 새 설계에서 제동 중에도 2펄스를 내려면, `low_speed_trim` 의 우회 조건에
  `self._goal_phase in (GOAL_PHASE_BRAKE1, GOAL_PHASE_BRAKE2)` 를 반드시 추가해야 한다.**
- **더 간단한 길** : 제동 중에는 지금처럼 **0 을 내고**, 해제하는 그 틱부터 2펄스를
  낸다. 어차피 arduino 가 0 으로 덮으므로 **차의 거동은 완전히 같다.** 이쪽이
  기존 코드를 안 건드린다 → **권고.**

#### ⑥ 함께 고칠 상수

| 파일:줄 | 상수 | 현재 | → |
|---|---|---|---|
| `driving.py:753` | `GOAL_CREEP_PULSE` | 1 | **2** (이름도 `GOAL_HOLD_PULSE` 로) |
| `driving.py:791` | `GOAL_KICK_PULSE` | 4 | **3** — 2펄스 유지면 킥이 거의 안 필요해지지만, 남긴다면 4 는 A보드 적분 동결 함정이다(6.2⑤) |
| `driving.py:752` | `GOAL_CREEP_KMH` | 4.0 | 재계산 — 2펄스 = 6.36 km/h 이므로 **해제선 하한을 올려야 한다** |
| `driving.py:754` | `GOAL_CREEP_MIN_LEFT_M` | 1.2 | 재검토 — 2펄스의 2단 정지거리가 1.24 m 라 거의 같다 |
| 신설 | `GOAL_HOLD_M` | — | **5.0** |
| `one_launch.py:332` | `goal_brake_m` | 20.0 | **25.0** (6펄스 + 5 m 유지 구간이면 need1 ≈ 18 m) |
| `driving.py:666` | `GOAL_BRAKE1_MS2` | 1.30 | 그대로. **실차 로그로 갱신하는 값**(goal_phase=1 구간의 `gps_kmh` 기울기가 참값) |

> **6.2 의 `CORNER_MIN_PULSE = 3` 과 충돌하지 않는다.** `run_follow()` 의 호출 순서가
> `corner_speed → goal_approach → corner_brake` 이고, ① `goal_approach` 가 곡률이 낸
> 값을 **뒤에서 덮으므로** 종점 구간에서는 2펄스가 이긴다, ② `corner_brake` 는
> `_goal_phase != NONE` 이면 손을 떼고, `_cb_cap()` 도 `min(pulse, cap)` 이라 **올리는
> 방향으로는 작용하지 않는다.** 이 순서를 바꾸면 그 보장이 깨진다.

#### ⑦ 유지해야 할 것 (없애면 안 되는 방벽)

- **2단 백스톱**(`GOAL_BACKSTOP`, `goal_brake2_backstop`) — "1단으로는 못 센다" 를
  판정해 2단으로 올린다. 새 설계에서는 **"5 m 안에서 2펄스로 못 내려온다"** 로 뜻이
  바뀌지만, **부족분이 그대로 종점 통과 속도가 되므로 더 필요해진다.**
  단계는 **올라가기만 한다**(내려오는 경로 없음 = 채터 없음)는 규칙을 유지할 것.
- **`GOAL_BRAKE1_MAX_S = 12.0 s` / `GOAL_BRAKE2_MAX_S = 3.0 s`** — 굳지 않게.
- **거리는 직선거리(`d2goal`)로 재고, 감시 창 진입에만 `s_left` 와 둘 다 요구한다** —
  순환 코스에서 출발점이 종점 근처일 때 출발하자마자 걸리는 것을 막는 장치다.
- **`_goal_engage()` 가 단계마다 타이머를 새로 잡는 것**.
- **최소 물림 0.5 s / `BRAKE_KEEPALIVE_S 0.25 s` / 0단은 재확인하지 않는다.**

#### ⑧ 실차 확인

`record` CSV 에서 볼 것 — `goal_phase` 열의 전이 시각, 그 구간의 `gps_kmh` 기울기
(**= 실측 a₁, `goal_brake1_ms2` 를 갱신하는 값**), `enc_pulse` 와 `gps_kmh` 의 차이
(**엔코더 허수가 해제를 늦추는지**), 그리고 **종점 대비 최종 정지 위치**.

---

### 6.4 `mppi_local_planner` 이식 · `/lstatus` 신설 · 인계 전 서행

#### 요구사항 (사용자 지시 그대로)

- `catkin_ws` 안의 `mppi_local_planner` 패키지를 `gold_ws` 로 이식한다.
  '라이다에게 제어권을 인계한다' 할 때의 그 노드다.
- **최고속도 조절을 편하게 할 수 있도록 담당 파일 상단에 변수 지정이 되어 있는지 점검.**
- **인계 전후로 속도가 자연스럽게 변하도록.** 단 인계지점을 앞두고 **종점과 같은
  서행 로직**이 작동하되 **완전 정지하지 않고 2펄스로 라이다에게 인계**한다.
  GPS 가 되받을 때도 **기본 주행속도로 자연스럽게** 돌아온다.
- **`driving.py` 가 `terrain` 값을 `/lstatus` 토픽으로 낸다** — `0`이면 `0`,
  `L`이면 `L`, `S`면 `S`.
- **`mppi_local_planner` 가 그 토픽을 구독**하고 (`one_launch.py` 와 함께 **대기상태**로
  떠 있다가) **`L` 이면 움직인다.**
  → **`/lstatus` 가 `/lidar_permit` 을 대체한다**(사용자 결정, ⑤).
  서행 시작 지점은 **주행 시작 직전에 CSV 를 읽으며 미리 판단한다**(⑤-2).
- **일시정지 로직도 `S` 일 때 작동하도록.**

---

#### ① ★이식 = 복원★ (2026-09-07 복원 완료)

**금색차용 이식본이 이미 있다.** `git HEAD` 에 온전히 들어 있고, **작업트리에서만
삭제된 상태**다(0.1절):

```
gold_ws/src/mppi_local_planner/
  src/mppi_local_planner_node.cpp   1203줄   ← 게이트·핸드오버·지령층이 여기 있다
  src/mppi_controller.cpp            527줄   ← 플래너 본체 (원본 그대로)
  src/ego_costmap.cpp                266줄
  include/mppi_local_planner/{mppi_controller,ego_costmap,vehicle_model,kasa_units}.hpp
  config/params.yaml  config/mppi.rviz  launch/one_launch.py  CMakeLists.txt  package.xml  README.md
```

원본은 `/home/mad1/catkin_ws/src/mppi_local_planner` 다(파일 13개, 1/5카용).
**둘의 차이는 플래너가 아니라 차량 계약 전부다** — 이식본 README 의 표:

| | 1/5카 원본 | **금색차 이식본** |
|---|---|---|
| 휠베이스 | 0.75 m | **1.25 m** |
| 윤거 | 0.65 m | **1.10 m** |
| 조향 상한 | 0.40 rad | **0.553 rad (도로휠 31.7°)** |
| `linear.x` | m/s | **목표펄스 0~15** |
| `angular.z` | 도로휠각 deg, +좌 | **pot 지령, −좌 / +우** |
| 순항 | 0.83 m/s | **2펄스 = 1.768 m/s** |
| 라이다 높이 | 0.80 m AGL | **1.17 m AGL** |
| 제동 | 속도 0 | **속도 0 + `/brake_level` 2단** |
| 게이트 | 없음 | **`/vehicle_mode` + `/estop`** |
| 조종권 | 없음(런치=출발) | **`/lidar_permit` ↔ `/lidar_active` 양방향** |

> **★catkin_ws 원본에서 새로 이식하지 말 것.★** 위 열 전부와
> `kasa_units.hpp`(431줄) · `ego_clear_*` 실측 장착값 · `cmd.*` 지령층
> (펄스 계단·조향 슬루·stop 프레임)이 통째로 사라진다. 정공법은 **복원**이다:
>
> ```bash
> cd ~/gold
> git checkout HEAD -- gold_ws/src/mppi_local_planner
> ```
>
> **★2026-09-07 실제로 이렇게 복원했다★** `mppi_local_planner` 외에
> `white` · `white806` · `white0901` 도 함께 되살렸고(원격 문서가 그 셋을 정상
> 트리 구성으로 적고 있다), `ouster-ros` 는 `git submodule update --init` 으로
> 받았다. **작업트리의 삭제 87개는 커밋되지 않은 로컬 손상이었다.**
>
> 같은 이유로 `gold_ws/src/white/`(`iahrs.py` 가 아직 import 한다 — 0.1절)와
> `ouster-ros` 서브모듈도 함께 볼 것. **지금 트리의 삭제는 커밋되지 않았으므로,
> 복원할지 삭제를 확정할지를 먼저 정하는 것이 이 작업의 0단계다.**

**빌드** — C++ 패키지라 `--symlink-install` 을 쓰지 않는다:

```bash
colcon build --packages-select ouster_sensor_msgs ouster_ros lidar mppi_local_planner \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

#### ② 최고속도 — 점검 결과 : ★한 곳이 아니라 네 곳이다★

**"파일 상단 변수 하나" 로는 되어 있지 않다.** 지금 속도를 정하는 값이 이렇게 흩어져 있다:

| # | 값 | 위치 | 역할 |
|---|---|---|---|
| 1 | `mppi.desired_speed: 1.768` | `config/params.yaml` | MPPI 비용함수의 목표 순항속도 |
| 2 | `max_speed: 2.05` | `config/params.yaml` | **플래너 하드캡** (샘플이 이 위로 못 간다) |
| 3 | `min_speed: 0.80` | `config/params.yaml` | 회피 중 하한 |
| 4 | **`kasa.max_pulse: 2`** | `config/params.yaml` | **액추에이터 상한 — ★실제로 차를 묶는 값★** |
| 5 | C++ 기본값 | `mppi_local_planner_node.cpp:217-218` | `pulseToMs(2)*1.15`, `pulseToMs(1)*0.90` — **상수가 아니라 계산식** |
| 6 | `lidar_speed:=1.768` / `lidar_pulse:=2` | `white1/launch/one_launch.py:597,602` | 런치가 1·4번을 덮어쓴다 |

**★그래서 조용히 안 듣는 조합이 있다★**

- `lidar_speed:=3.0` 만 올리면 → **`max_speed: 2.05`(2번)가 먼저 자른다.**
  런치 인자에 2번이 없어서 밖에서 못 푼다.
- `lidar_speed` 를 올리고 `lidar_pulse` 를 안 올리면 → **`msToPulse()` 가 2로 자른다.**
  런치 주석도 "**짝을 맞춰 둘 것**"이라고만 적혀 있지, 코드가 강제하지 않는다.

**개선안 (권고)** — `params.yaml` 최상단에 **펄스 하나**만 두고 나머지를 노드가 유도한다:

```yaml
# ── ★여기 하나만 고치면 된다★ ────────────────────────────────
cruise_pulse: 2        # L 구간 순항 [펄스]. 1펄스 = 0.884 m/s = 3.182 km/h
```

```cpp
// mppi_local_planner_node.cpp — 파라미터 읽는 곳
const int cruise_pulse = get_parameter("cruise_pulse").as_int();
mppi_params_.desired_speed = lidar::kasa::pulseToMs(cruise_pulse);          // 1
vehicle_params_.max_speed  = lidar::kasa::pulseToMs(cruise_pulse) * 1.15;   // 2 ★자동★
vehicle_params_.min_speed  = lidar::kasa::pulseToMs(1) * 0.90;              // 3
// kasa.max_pulse 도 cruise_pulse 로 덮는다 — 4번이 1번과 갈라지지 못하게
```

그리고 `one_launch.py` 는 **`lidar_pulse` 하나만** 노출한다
(`lidar_speed` 는 삭제 — 두 개를 열어 두면 짝이 어긋난다).
**"짝을 맞춰 둘 것" 이라는 주석이 필요하다는 것 자체가 설계 결함의 신호다.**

> ⚠️ **`kasa.max_pulse` 를 올릴 때는 `cmd.dodge_steer_deg`(6.0 — 조향이 이보다 크면
> 1펄스로 회피)와 `cmd.stop_*` 프레임도 같이 봐야 한다.** 저 값들은 2펄스 순항을
> 전제로 잡힌 것이다. 그리고 6.2절대로 GPS 쪽이 6펄스가 되어도 **L 구간은 2펄스를
> 유지하는 것이 맞다** — 라바콘 사이는 원래 서행 구간이다.

---

#### ③ 인계 전 서행 — ★6.3의 종점 서행 로직을 그대로 재사용한다★

**요구는 "종점과 동일한 서행로직이 작동하되 완전 정지하지 않고 2펄스로 인계"** 다.

**6.3에서 이미 `goal_approach()` 를 "목표지점 앞 d 에서 2펄스가 되어 있게" 로
일반화하자고 설계했다.** 종점이냐 L 구간 진입점이냐는 **끝에서 무엇을 하느냐만
다르고 감속 문제는 완전히 같다.** 그러므로 한 함수로 뽑는다:

```python
def approach(self, dist_left, hold_pulse, pulse):
    """목표지점까지 dist_left 남았다. hold_pulse 로 내려놓는다.
    ★세우지 않는다★ — 세우는 것은 부르는 쪽의 몫이다(종점은 2단, L 은 이양)."""
```

| | 종점 (6.3) | **L 구간 진입 (이 절)** |
|---|---|---|
| `dist_left` | 종점까지 **직선거리** `d2goal` | L 시작 WP 까지 **호길이** `wp_s[L0] − wp_s[wp_idx]` |
| `hold_pulse` | 2 | **2** (같다) |
| 유지 구간 | `GOAL_HOLD_M = 5 m` | `LIDAR_HOLD_M`(신설, 권장 3~5 m) |
| 끝에서 | **리니어 2단 + DRIVE_DONE** | **정지하지 않고 `begin_lidar_zone()`** |
| 리니어 | 1단 체결 → 2펄스 되면 해제 | **같다** |

**거리 계산** — L 구간 시작 인덱스는 CSV 에서 **미리 안다.**
★사용자 결정에 따라 주행 시작 직전(`build_waypoints()`)에 한 번 계산해 표로 박아
둔다 — 구현은 ⑤-2 에 있다.★ 매 틱 훑는 아래 형태는 그 표를 만들 때 쓰는 것이고,
제어 루프에서는 **호길이 비교 한 번**으로 끝난다:

```python
def lidar_zone_left_m(self):
    """다음 L 구간 시작까지 남은 호길이 [m]. 없으면 inf."""
    for i in range(self.wp_idx, len(self.wp_zone)):
        if self.wp_zone[i] in LIDAR_ZONE_CHARS:
            return self.wp_s[i] - self.wp_s[min(self.wp_idx, len(self.wp_s) - 1)]
    return float('inf')
```

> **호길이를 쓰는 것이 여기서는 맞다.** 종점 코드가 `s_left` 를 피한 이유는
> "포인터가 앞서 튀면 0 으로 주저앉는다" 였는데, **그 오류의 방향이 여기서는
> '일찍 감속한다' = 안전한 쪽**이다(종점에서는 '일찍 2단을 문다' = 위험한 쪽이었다).
> `L0` 은 시작 인덱스를 캐시해 두면 매 틱 훑지 않아도 된다.

**필요 거리 검산** (6→2펄스, `= v₀×0.55 + (v₀²−v₁²)/2a`) :

| 기본속도 | 코스트 0.41 | 1단 0.88 | **1단 1.30** |
|---|---|---|---|
| 4펄스 | 13.38 m | 7.27 m | **5.55 m** |
| **6펄스** | 33.41 m | 17.13 m | **12.54 m** |

→ 6.2절의 6펄스에서는 **L 구간 진입 12.5 m 전에 1단을 물어야 한다.**
`LIDAR_HOLD_M(3~5 m)` 를 더하면 **체결 지점이 진입점 15.5~17.5 m 앞**이다.
**매핑할 때 L 을 적는 위치가 그만큼 앞에 여유가 있어야 한다** —
코너 직후나 다른 L 구간 바로 뒤에 L 을 붙이면 감속할 거리가 없다.
`select_route()` 가 **L 구간 시작 전 여유 거리를 검사해 경고**하게 만들 것.

**소유권** — 이 서행은 `goal_approach` · `corner_brake` 와 같은 리니어를 쓴다.
6.1의 S 서브페이즈까지 넷이 된다. **우선순위를 한 줄로 못 박을 것:**

```
종점 접근  >  S 일시정지  >  L 인계 서행  >  코너 1단 제동
```

각 단계는 자기보다 위가 활성이면 **손을 뗀다**(현행 `corner_brake` 의
`_goal_phase == GOAL_PHASE_NONE and tl_brake_req() == 0` 과 같은 방식).

---

#### ④ 인계·복귀의 속도 연속성 — ★대부분 이미 되어 있다★

**G → L (인계)**

`begin_lidar_zone()`(:2172)은 지금 이양 직전에 **`send(0, 0.0, control=True)`** 로
**0 펄스를 한 번** 낸다. "mppi 가 첫 지령을 내기까지 내 마지막 펄스가 살아 있으면
안 된다" 는 안전장치다(arduino 가 신선도 없이 래치하므로).

**이 0 펄스를 2 펄스로 바꿀 필요는 없다.** 검산:

```
mppi 첫 지령까지 3틱(0.15 s) 가정, 코스트 0.41 m/s²
  Δv = 0.41 × 0.15 = 0.062 m/s = ★0.07 펄스★
```

**0.07 펄스는 지령 분해능(1펄스)의 7% 다 — 감지되지 않는다.** 반면 0 펄스를 없애면
**mppi 가 첫 지령을 못 냈을 때 차가 2펄스로 계속 굴러간다.** `lidar_alive()` 검사가
앞에 있긴 하지만, **"실패하면 침묵" 이라는 성질을 값싸게 유지할 수 있는데 버릴 이유가
없다.** → **0 펄스 유지 권고.** 속도의 연속성은 **양쪽이 다 2펄스**라는 사실이 만든다.

**L → G (복귀)**

`end_lidar_zone()`(:2235) → `_rejoin = True` → `REJOIN_PULSE_MAX = 2` 로 묶였다가,
`|CTE| ≤ REJOIN_DONE_CTE_M(0.5 m)` 이 되면 평소 규칙(= `drive_pulse`)으로 돌아간다.
**즉 "2펄스로 되받아 기본속도로 자연 복귀" 가 이미 구현되어 있다.**

6.2로 기본이 6펄스가 되면 **2 → 6 계단**이 생기는데, **A보드가 이미 램프를 갖고 있다:**

```
FF(2) = 70  →  FF(6) = 107.3      PWM_SLEW_MAX = +4/cycle
  ⌈37.3 / 4⌉ = 10 cycle × 20 ms = ★0.20 s★
```

**0.2 s 램프면 사람이 계단으로 느끼지 않는다 — ROS 쪽에 슬루를 또 두지 말 것.**
(두 곳에 슬루가 있으면 "지금 속도를 누가 정하는가" 가 흐려진다.)

⚠️ 다만 **재수렴이 끝나는 순간(CTE 0.5 m)에 2→6 이 한 번에 나간다.** 재수렴 판정이
경로 위에서 일어나므로 안전하지만, 실차에서 **재수렴 종료 시점의 조향각**을 함께 볼 것 —
큰 각으로 붙는 중에 6펄스가 나가면 언더스티어로 다시 벌어진다.
필요하면 `REJOIN_PULSE_MAX` 를 **단계적으로 푸는 것**(2 → 4 → 6)을 검토한다.

---

#### ⑤ `/lstatus` — ★`/lidar_permit` 을 대체한다 (사용자 결정)★

**결정** : `/lstatus` 가 `/lidar_permit` 의 역할을 **가져간다.**
근거는 사용자 지시 그대로 — *"어차피 L 일 때 라이다 노드에게 넘길 텐데 같은 역할을
한다."* 그리고 **서행 시작 지점은 주행 시작 직전에 CSV 를 읽으면서 미리 판단**하므로,
"감속 중에 이미 `'L'` 이 나가서 겹친다" 는 문제가 애초에 생기지 않는다(아래 ⑤-2).

```
white1/driving ──/lstatus (String: '0' | 'L' | 'S')──▶ mppi_local_planner
white1/driving ◀──/lidar_active (Bool)─────────────── mppi_local_planner
```

**`/lidar_permit` 은 삭제한다.** `/lidar_active` 는 **남긴다**(⑤-4).

##### ⑤-1 발행 — ★`terrain` 원값이 아니라 '지금 유효한 상태' 를 낸다★

`publish_state_topics()`(`driving.py:3448`)에서 **매 틱(20 Hz)** 낸다.
엣지 발행은 늦게 뜬 구독자를 놓친다(`/control_state` 와 같은 이유).

```python
LSTATUS_TOPIC = '/lstatus'      # std_msgs/String
LSTATUS_GPS   = '0'             # GPS 추종 (mppi 침묵)
LSTATUS_LIDAR = 'L'             # 라이다가 몬다
LSTATUS_STOP  = 'S'             # 일시정지 중

def lstatus_now(self):
    """★내가 실제로 손을 놓았을 때만 'L' 을 낸다★ — 이 토픽이 곧 조종권이다."""
    if self._sp_state != SP_NONE:            # S 일시정지 처리 중 (6.1)
        return LSTATUS_STOP
    if self._lidar_zone:                     # begin_lidar_zone() 이 성립한 뒤에만 True
        return LSTATUS_LIDAR
    return LSTATUS_GPS
```

**★`zone_at(self.wp_idx)` 를 그대로 흘리지 않는다★** — 이것이 이 설계의 핵심이다.
`terrain` 원값을 그대로 내면 두 가지가 깨진다:

1. **조종권 회수를 못 따라간다.** `_revoke_lidar()`(:2154)는 E-STOP·GPS 두절에서
   조종권을 즉시 회수하는데, **그때도 경로의 `terrain` 은 여전히 `'L'` 이다**
   (경로가 바뀐 게 아니니까). 원값을 흘리면 mppi 가 계속 몬다.
   → `self._lidar_zone`(내가 실제로 이양했는가)을 보면 회수가 그대로 반영된다.
2. **CSV 값이 지저분하다.** `''` · `'0'` · `'0.0'` · `'-1'`(3.4절의 구 지형코드) ·
   `'l'` · `'s'` 가 섞여 온다. **구독자가 각자 파싱하면 한 곳이 반드시 어긋난다.**
   → **발행자가 세 값으로 정규화해서 낸다.** 대문자로 통일한다.

**`begin_lidar_zone()` 의 3단 순서는 그대로 유지한다** — ① 과도상태·리니어 놓기
② 0 펄스 한 번 ③ **`/lstatus` 를 `'L'` 로**(종전의 허락 발행 자리). 그 순서가
겹침을 없애는 실체다. `end_lidar_zone()`·`_revoke_lidar()` 도 종전에
`pub_lidar_permit.publish(False)` 를 하던 자리에서 **`'0'` 을 한 틱 먼저 낸다.**

##### ⑤-2 서행 시작 지점은 ★주행 시작 직전에 미리 계산한다★ (사용자 결정)

`select_route()` / `build_waypoints()` 에서 **경로를 한 번 훑어 구간 표를 만들어 둔다.**
매 틱 앞을 훑지 않으므로 비용이 0 이고, `/lstatus` 가 `'L'` 로 바뀌는 시점과
서행이 시작되는 시점이 **코드상 완전히 분리된다.**

```python
# build_waypoints() 안에서 한 번만
self._lz_spans = self._zone_segments(self.wp_zone)      # 이미 있는 함수(:1720)
self._stop_idx = [i for i, z in enumerate(self.wp_zone)
                  if z in STOP_ZONE_CHARS]
# 각 L 구간 시작마다 '서행을 시작할 호길이' 를 미리 박아 둔다
self._lz_slow_from = []          # [(감속시작 s, L시작 idx), ...]
for i0, _ in self._lz_spans:
    d = self.approach_need_m(self.max_speed_ms, HOLD_PULSE)   # 6.3 의 식
    self._lz_slow_from.append((self.wp_s[i0] - d, i0))
```

- **판정은 호길이 비교 한 번**이다 : `wp_s[wp_idx] >= 감속시작 s` 이면 서행 시작.
- **감속에 필요한 거리가 경로에 실제로 있는지 여기서 검사한다** — 직전 L 구간이나
  경로 시작이 그보다 가까우면 **`select_route` 가 경고한다**(③ 참고).
  6펄스 기준 필요 거리가 **12.5 m + 유지구간** 이라 실제로 흔히 걸린다.
- ⚠️ **`build_waypoints()` 는 `S_DRIVE_HEADING` 진입에서 불린다**(`enter()`:1894).
  즉 **주행 시작 직전** 이고, 사용자가 말한 "주행 시작 직전 CSV 를 읽으면서" 와
  정확히 같은 자리다.

##### ⑤-3 구독 (`mppi_local_planner_node.cpp`)

`permit_sub_`(`Bool`, :116-121) 을 `lstatus_sub_`(`String`)으로 갈아끼운다.
**타이머 로직은 한 줄만 바뀐다:**

```cpp
declare_parameter<std::string>("handover.lstatus_topic", "/lstatus");
declare_parameter<double>("handover.lstatus_stale_s", 1.0);
declare_parameter<bool>("handover.require_lstatus", true);

lstatus_sub_ = create_subscription<std_msgs::msg::String>(
  lstatus_topic_, rclcpp::QoS(10),
  [this](const std_msgs::msg::String::ConstSharedPtr & m) {
    lstatus_.store(m->data.empty() ? '0' : m->data[0], std::memory_order_relaxed);
    lstatus_stamp_.store(nowSeconds(), std::memory_order_relaxed);
  });

// controlLoop() — 종전 permitFresh() 자리
bool mayDrive() const {
  if (!require_lstatus_) return true;                       // 단독 시험용
  if (nowSeconds() - lstatus_stamp_.load() > lstatus_stale_s_) return false;
  return lstatus_.load() == 'L';
}
```

> **★신선도 판정을 반드시 옮겨 올 것★** `/lidar_permit` 에 있던
> `permit_stale_s: 1.0` 이 그대로 필요하다. **`driving` 이 죽으면 마지막 `'L'` 을
> 붙들고 계속 몰게 된다** — 이 계약에서 **신선도가 곧 허락이다.**
> `require_permit` → `require_lstatus` 로 이름만 바꾸고, **`one_launch.py:138` 의
> "★반드시 true★ 런치 인자로도 열지 않는다" 강제도 그대로 옮긴다.**

##### ⑤-4 ★`/lidar_active` 는 절대 없애지 말 것★

방향이 반대인 토픽이고, 없으면 **mppi 가 죽었을 때 driving 이 침묵한 채
`arduino` 가 마지막 펄스를 래치해 차가 계속 간다.** `begin_lidar_zone()` 의
`lidar_alive()` 선행 검사와 `hold_for_lidar()` 의 끊김 감지가 그것 하나에 매달려 있다.

`/lstatus` 로 합칠 수 없다 — **`/lstatus` 는 driving → mppi 이고 `/lidar_active` 는
mppi → driving 이다.** 대체 관계가 아니다.

##### ⑤-5 다른 구독자

`/lstatus` 는 화면·기록에서도 값이 크다 — `hud`(구간 색칠), `record`(열 추가 :
`RECORD_TOPICS`), `sound`(구간 진입 안내), `prompt`(현재 구간 표시).
**Bool 두 개를 보던 것보다 읽기 쉽다** — 이것이 이 교체의 부수 이득이다.

#### ⑥ S 일시정지와 `/lstatus`

- **S 의 처리 주체는 `driving` 이다**(6.1). **`/lstatus` 로 트리거하지 않는다** —
  자기가 발행한 토픽을 자기가 구독하면 한 틱 지연이 생기고, 무엇보다
  **"그 행을 지나쳤는가" 는 `wp_idx` 구간 검사여야 하는데 `/lstatus` 는 현재 값만
  실린다**(6.1②: 한 틱에 5 WP 를 건너뛸 수 있어 단일 행은 놓친다).
  → **내부 상태로 처리하고, `/lstatus` 에는 결과를 실어 보낸다.**
- **mppi 쪽은 `'S'` 에서 아무것도 하지 않는다** — `mayDrive()` 가 `== 'L'` 하나만
  보므로 **자동으로 침묵이다. 추가 코드가 필요 없다.**
- **다만 `/lstatus` 가 `'S'` 인 동안 mppi 가 `/lidar_active` 를 계속 내는 것은 정상**이다
  (그건 '살아 있다' 는 신고이지 '몬다' 가 아니다).
- ⚠️ **S 는 L 구간 안에 두지 않는다**(6.1① 검증 항목). L 구간에서는 driving 이 침묵해
  S 판정이 아예 돌지 않는다.

---

#### ⑦ "one_launch 와 함께 대기상태로" — 이미 그렇게 되어 있다

`white1/launch/one_launch.py:127-150` 이 mppi 노드를 직접 선언하고,
**`use_lidar` 기본값이 `true`**(:584), **`handover.require_permit: True` 를
런치에서 강제**한다(:139-141, "런치 인자로도 열지 않는다").
`params.yaml` 의 `require_permit: true` 와 합쳐 **허락이 오기 전에는 아무것도
발행하지 않는다** — `hold()` 조차 부르지 않는다(그 함수도 `/cmd_vel_raw` 에 0 을
실제로 낸다). **요구하는 '대기상태' 가 이미 구현되어 있다.**

→ ⑤의 교체에서는 **이 강제를 `handover.require_lstatus: True` 로 이름만 바꿔
그대로 옮긴다.** 이름이 바뀌었다고 강제를 빼면, mppi 가 GPS 추종 구간에서도
`/cmd_vel_raw` 를 내며 `driving.py` 와 20 Hz 로 서로를 덮는다.

- `lidar/launch/ouster.launch.py` 를 include 해 OS1-32 드라이버를 함께 띄운다.
- **`lidar`·`mppi` 의 `one_launch.py` 를 include 하지 않는다** — 그 둘은 각자
  `arduino`·`sound`·`hud` 를 띄워 **시리얼 포트를 다투게 된다.** 노드만 직접 선언한다.
- **AEB(`cone_lidar_node`)는 띄우지 않는다** — 라바콘 사이를 지날 때 가상범퍼가 먼저 선다.

라이다는 **USB 가 아니라 유선 LAN** 이다. 안 붙으면:
```bash
ip -br addr show eno1     # 192.168.6.100/24 여야 한다
ping -c2 192.168.6.11
```

---

#### ⑧ 구현 순서 (권고)

0. **삭제를 확정할지 복원할지 먼저 정한다**(0.1절). `mppi_local_planner` 복원 →
   빌드 → `use_lidar:=true` 로 **현행 계약 그대로** 한 번 돈다.
   **여기까지가 "이식" 이고, 아래는 새 기능이다.**
1. `params.yaml` 최상단 `cruise_pulse` 단일화(②) — 코드 변경이 작고 위험이 없다.
2. `/lstatus` 발행만 추가(⑤) — **구독자 없이** 먼저 띄워 `ros2 topic echo /lstatus` 로
   경로를 한 바퀴 돌며 값이 맞게 나오는지 본다. **아무 거동도 안 바뀐다.**
3. `select_route()` 검증 강화 — L/S 개수·인덱스, **L 구간 앞 감속 여유 거리**, 연속 S,
   L 안의 S (6.1①, ③).
4. **`approach()` 공통화**(6.3 + ③) — 종점부터 먼저 고쳐 실차로 확인하고,
   그다음 L 인계에 같은 함수를 붙인다. **한 번에 둘 다 바꾸지 말 것.**
5. S 서브페이즈(6.1).
6. **`/lstatus` 를 조종권으로 승격**(⑤) — `driving` 의 발행 전환과
   **mppi 의 구독 전환을 반드시 한 커밋에서 함께** 한다. 대체 관계이므로
   **한쪽만 바꾸면 조종권이 아예 넘어가지 않거나(mppi 침묵) 항상 넘어간다(양쪽 발행).**
   바꾼 직후 확인 : `ros2 topic echo /lstatus` · `ros2 topic hz /cmd_vel_raw`
   (L 구간에서 **발행자가 하나뿐**인지 `ros2 topic info /cmd_vel_raw --verbose`).

**각 단계마다 `record` CSV 로 확인할 것** — `/lstatus` 열, `lidar_permit`·`lidar_active`
전이 시각, 인계 전후 `gps_kmh`, 복귀 후 재수렴 시간과 `max|CTE|`.

---

## 7. 코드를 고칠 때의 규약 — ★한쪽만 고치면 안 되는 짝★

| 값 | 같이 고칠 곳 |
|---|---|
| 차량 제원(휠베이스·조향비·펄스 환산) | `driving.py` 상수절 **+** `lidar/include/lidar/kasa_units.hpp` |
| `MAX_PULSE_LIMIT` / `drive_pulse` | `driving.py` **+** `one_launch.py` **+** `kasa_units.hpp:89 PULSE_OPERATING_MAX` |
| B보드 텔레메트리 필드 수 | `arduino.py parse_b()` **+** 상태변수/퍼블리셔 **+** `record.py RECORD_TOPICS` |
| `KEEPALIVE_S`(ROS) | A보드 `RX_TIMEOUT_MS`(3000) — **한 쌍이다** |
| `/gps_fused` 배열 | `gps.py GPS_FUSED_FIELDS` **+** `driving.py cb_gps_fused` **+** `record.py _array(n)` |
| 저장 경로 | `paths.py` 하나가 소유자 — 다른 곳에 리터럴을 적지 말 것 |
| `terrain` 규약 | `driving.py:963` **+** `one_launch.py` 헤더 **+** `lidar/README.md` **+** 이 문서 |
| mppi 순항속도 | `params.yaml` 의 `mppi.desired_speed` **+** `max_speed` **+** `kasa.max_pulse` **+** `one_launch.py` 의 `lidar_speed`/`lidar_pulse` — **★넷이 짝이다. 6.4② 의 단일화 권고 참고★** |
| 조종권 | **`/lstatus`**(driving → mppi, 허락) **+** **`/lidar_active`**(mppi → driving, 생존) — **방향이 반대라 합칠 수 없다.** `/lstatus` 발행부와 mppi 구독부는 **한 커밋에서 함께** 고친다(6.4⑤) |
| 신선도 문턱 | `/lstatus` 발행 주기(20 Hz) **+** mppi `handover.lstatus_stale_s`(1.0) — **신선도가 곧 허락이다**(6.4⑤-3) |

### 설계 원칙 (코드 전반에서 반복되는 것)

1. **단일 소유자** — 같은 값을 두 곳에 적지 않는다. 반드시 어긋난다.
2. **발행자를 시간축에서 배타로** — `/cmd_vel_raw` 는 '마지막 발행자가 이기는' 토픽이고
   arduino 는 **신선도 없이 래치**한다. 둘이 동시에 내면 서로를 덮고, 한쪽이 조용히
   죽으면 마지막 펄스로 차가 계속 간다.
3. **신선도 = 상대의 생존** — `/lidar_active` · `/tl_permit` · `/aeb_stop` 이 같은 규약.
   끊기면 '죽었다'로 보고 안전측으로 떨어진다.
4. **되먹임을 만들지 않는다** — 제동 체결은 **경로 기하**로, 해제는 **실측**으로.
   실측이 이상해도 결과가 언제나 '푸는 쪽'이면 루프가 안 생긴다.
5. **모르면 낮게 본다** — GPS σ 를 모르면 Fixed 로 올리지 않고, 속도를 모르면
   종점 제동은 일찍 물고 코너 제동은 아예 안 문다(각각 안전한 방향이 반대다).
6. **정지의 이름을 섞지 않는다** — 🚨 는 E-STOP(D12) 전용.
7. **틀리는 방향을 고른다** — 늦게 푸는 쪽·일찍 무는 쪽·모자란 쪽으로 틀리게 둔다.
