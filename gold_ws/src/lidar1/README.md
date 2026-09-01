# lidar — 금색차 kasa 라이다 인지·주행 패키지

`catkin_ws/src/e_stop`(1/5카용)를 금색차로 이식한 것이다. 노드 셋:

| 노드 | 하는 일 | 차를 움직이나 |
|---|---|---|
| `cone_lidar_node` | 가상범퍼 AEB — 전방 최근접 거리·정지신호 | **아니오** |
| `drive_lidar_node` | 라바콘 코리도 측량 잠금 + IMU 헤딩홀드 직진 | 예 |
| `drive_gps_node` | GPS 일자 매핑 + 스탠리 추종 | 예 |

> **먼저 읽을 것** — `include/lidar/kasa_units.hpp`. 이 이식에서 바뀐 것의 90%가
> 그 파일 하나에 모여 있다.

---

## 1. 작동 방식 — white1 과 무엇이 다른가

### white1 (지금 쓰는 방식)

```
ros2 launch white1 one_launch.py      # 센서 + arduino + driving + mapping + prompt
```

`prompt` 화면에서 메뉴를 고르면 `/drive_cmd`(String) **한 토픽**으로만 하달된다.

```
prompt ──/drive_cmd──▶ driving.py ──/mapping_cmd──▶ mapping.py
         MAP_START      (상태기계)
         DRIVE_START    S_IDLE → S_MAP_* → S_DRIVE_HEADING → S_DRIVE_RUN → S_DRIVE_DONE
         <경로파일명>
         STOP
```

- 하드웨어 게이트가 **2단**이다 — D5 스위치(자율/수동) + E-STOP 해제.
- 시작 전에 화면이 그 둘을 기다려 준다.

### 이 패키지 — **상태기계도, 화면도 없다**

| | `drive_lidar_node` | `drive_gps_node` |
|---|---|---|
| 시작 | ★**런치 = 출발**★ 명령 토픽이 아예 없다 | `/drive_cmd` 토픽 |
| 상태 | Survey → Drive → Failed | Idle / Mapping / Driving |
| 정지 | Ctrl-C · E-STOP · AEB | `/drive_cmd: STOP` · AEB · CTE 이탈 |
| 매핑 | 없음 | `/mapping_cmd`(Bool) — **이 노드가 직접 CSV 를 쓴다** |

#### `drive_lidar_node` — ★가장 조심해야 하는 것★

```bash
ros2 launch lidar drive_lidar.launch.py
#  → 2.5초 서서 라바콘 관측 → 복도 잠금 → ★스스로 출발★
```

**"주행 시작"을 누르는 절차가 없다.** 원본이 라바콘 코스 전용으로 만들어졌고,
"세워 두고 관측 → 잠기면 직진"이 그 자체로 절차였기 때문이다.
런치 헤더에 경고를 박아 뒀지만, **차 앞을 비우고 런치할 것.**

#### `drive_gps_node` — 토픽으로 직접 지시

```bash
# ① 매핑 (곧게 굴리면서 켜고, 끄면 최소자승 직선을 피팅해 CSV 두 벌 저장)
ros2 topic pub --once /mapping_cmd std_msgs/Bool "{data: true}"
ros2 topic pub --once /mapping_cmd std_msgs/Bool "{data: false}"

# ② 주행 (LAST = 마지막 *_straight.csv)
ros2 topic pub --once /drive_cmd std_msgs/String "{data: LAST}"

# ③ 정지 — ★리니어 2단이 걸린다★
ros2 topic pub --once /drive_cmd std_msgs/String "{data: STOP}"

# 상태
ros2 topic echo /drive_status
```

> ⚠️ **`drive_gps_node` 는 white1 `driving.py` 와 기능이 정면으로 겹친다.**
> 둘 다 `/cmd_vel_raw` 발행자다 — **동시에 띄우지 말 것.**
> 실주행 품질은 white1 쪽이 압도적으로 낫다(코너 선행제동·CTE 적분·크립
> 재출발·GPS 품질 판정·헤딩 초기화가 전부 실차 로그로 튜닝됐다). 이 노드는
> "라이다 AEB 를 GPS 주행에 물리면 어떻게 되나"를 보는 시험대로 둔다.

---

## 2. 이식에서 바뀐 것

`nxde/arduino.py` 는 구 `white/motor.py` 의 **토픽 이름·타입을 일부러 물려받았다.**
그래서 원본을 그대로 꽂으면 **빌드도 되고 토픽도 붙고 echo 도 정상인데 차가 전혀
다르게 움직인다.** 이 패키지가 한 일은 그 조용한 차이를 메운 것이다.

| 필드 | 1/5카 (원본) | 금색차 (이 패키지) |
|---|---|---|
| `linear.x` | m/s (실수) | **주행 목표펄스 정수 0~15** |
| `angular.z` 부호 | + = 좌 | **− 좌 / + 우** |
| `angular.z` 물리 | 도로휠각 δ | **pot 지령** = 1.26·δ + 5.17·v²/R |
| 제동 | 없음(속도지령이 곧 제동) | **`/brake_level` 0/1/2 별 토픽** |
| 구동 허용 | (drive_lidar 는 안 냈다) | **`/control_state` 매 틱** |
| 게이트 | 없음 | **`/vehicle_mode`(D5) + `/estop`** |

**환산·부호 반전이 일어나는 곳은 `KasaActuator::drive()` 한 곳뿐이다.**
두 번 뒤집으면 조용히 좌우가 바뀐다.

### 제원 — 실측값으로 갈아끼운 것

| | 1/5카 | 금색차 | 출처 |
|---|---|---|---|
| 휠베이스 | 0.75 m | **1.25 m** | driving.py:307 |
| 조향 상한 | 0.366 rad (21°) | **0.553 rad (도로휠 31.7°)** | CHANGELOG 2026-08-12(3) |
| 최소회전반경 | 1.90 m | **2.02 m** | 〃 |
| 포화 없는 LFD | — | **4.04 m** (= 2R) | 〃 |
| 1펄스 | — | **0.884 m/s = 3.182 km/h** | mad-code/CLAUDE.md |
| 라이다 높이 | 0.80 m AGL | **1.17 m AGL** | 2026-08-25 실측 (OS1-32) |

### 제동 — 신설

**펄스 0 은 '정지'가 아니라 '코스트'다.** 4펄스에서 코스트 정지거리가 15.2 m라
AEB 문턱 8.5 m 안에 **설 수 없다.**

| 수단 | 감속도 | 2펄스 정지거리 | 4펄스 |
|---|---|---|---|
| 코스트(펄스 0) | 0.41 m/s² | 3.8 m | **15.2 m** |
| 1단 | 1.30 (구동차단 실측) | 1.2 m | 4.8 m |
| 2단 | 2.20 (실측 하한) | 0.7 m | 2.8 m |

→ **감속 구간 = 펄스 0 + 1단**, **정지 확정 = 2단**. 규칙은 전부 white1 에서 데인
것을 그대로 옮겼다(`kasa_units.hpp` 3절 · `white1/BRAKING.md`):

- 단계는 **올라가기만** 한다 (리니어 왕복이 제일 나쁘다)
- **최소 물림 0.5 s** (1단 행정 290카운트 ≈ 0.54 s)
- 이미 늦었으면 **1단을 건너뛰고 곧장 2단** (B보드 `lin_state != LIN_IDLE` 가드)
- **체결은 기하로 / 해제는 실측으로** (되먹임 차단)
- **0.25 s keepalive**, 단 **0 은 재확인하지 않는다** (남의 정지를 푼다)

---

## 3. 실행

### 준비

```bash
source /opt/ros/humble/setup.bash
source ~/gold/gold_ws/install/setup.bash     # ★이것 하나면 된다★
```

**라이다 드라이버(`ouster_ros`)도 이 워크스페이스 안에 있다** —
`gold_ws/src/ouster-ros/` (2026-08-25 이관). catkin_ws 를 함께 얹을 필요가 없다.

```bash
# 드라이버까지 함께 빌드
colcon build --packages-select ouster_sensor_msgs ouster_ros lidar \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> **`--symlink-install` 을 쓰지 말 것 (2026-08-25).**
> 이 패키지에는 C++ 이 들어 있다. 심볼릭으로 설치하면 `install/lidar/lib/lidar/*` 가
> `build/lidar/*` 를 가리키는 링크가 되어, `build/` 를 지우거나 다른 옵션으로
> 다시 빌드하는 순간 `install/` 이 **깨진 링크만 남은 채 조용히 살아 있다** —
> `ros2 run` 이 "실행파일이 없다"가 아니라 이상한 방식으로 실패한다.
> 이미 심볼릭으로 빌드해 둔 것이 있으면 한 번은 지우고 다시 빌드한다:
> ```bash
> rm -rf build/lidar install/lidar
> ```

> **버전을 올리지 말 것 — `ouster-ros` 0.13.x 로 고정이다.**
> 0.14.x 부터 SDK 가 개편되면서 **펌웨어 2.4 미만 센서의 연결을 코드에서 거부**한다
> (`SensorHttp::create`). 이 센서는 **FW v2.3.0** 이라 최신 드라이버로는 아예 안 붙는다.
> 0.13.13 은 FW 2.0 이상을 지원한다. 자세한 경위는
> `src/ouster-ros/OUSTER_OS1_32_SETUP.md`.

### 연결 확인 (라이다는 USB 가 아니라 유선 LAN 이다)

```bash
ip -br addr show eno1     # UP + 192.168.6.100/24
ping -c2 192.168.6.11     # 센서
```

### ① 라이다만 — RViz 로 눈으로 확인

```bash
ros2 launch lidar ouster.launch.py
# 다른 터미널
ros2 topic hz /ouster/points      # 20 Hz 근처 (ouster_driver.yaml lidar_mode 1024x20)
rviz2 -d $(ros2 pkg prefix lidar)/share/lidar/config/drive_lidar.rviz
```

### ② AEB 만 — ★차가 절대 안 움직인다. 여기서 장착을 검증할 것★

```bash
ros2 launch lidar aeb.launch.py use_rviz:=true
ros2 topic echo /cone_lidar_node/obstacle_distance
```

**앞뒤 방향 확인법**: 차 앞 3 m 에 사람이 서서 `obstacle_distance ≈ 3.0` 이면
`flip_lidar_xy` 가 맞다. 앞에서 `inf` 인데 뒤에서 잡히면 뒤집힌 것이다.

```bash
ros2 param set /cone_lidar_node flip_lidar_xy false
```

### ③ 라바콘 주행 — ★런치 = 출발★ (양옆 콘 → 헤딩 잠금 → 1펄스 정렬 → 7펄스 직진)

기본 `one_launch` 는 AEB 수동시험이다. 자율 헤딩홀드는 `cone_drive:=true`.

```bash
# 실차 — D5 자율 · E-STOP 해제 · 차 앞을 비우고 · 양옆 라바콘
ros2 launch lidar one_launch.py cone_drive:=true

# 책상 시험 (게이트를 끄고 지령만 본다 — 차에 연결하지 말 것)
ros2 launch lidar one_launch.py cone_drive:=true drive:=false use_rviz:=true
ros2 topic echo /cmd_vel_raw          # linear.x 가 ★정수★ 인지 확인
```

로그에 `헤딩 IMU = '/imu'` 와 `AHRS 헤딩 잠금` · `🔒 복도 잠금` 이 나와야 한다.
6초 안에 양옆이 안 잡히면 `❌ 관측 실패` 로 정지 유지 (한쪽만으로 가려면
`require_dual_side:=false`).

> 예전 `drive_lidar.launch.py` 도 동작한다. 다만 실차는 arduino·AEB 래치·HUD 가
> 붙은 **one_launch cone_drive** 를 쓴다.

### ④ GPS 주행

```bash
# white1 센서 스택을 driving 없이 먼저 (arduino·iahrs·gps·nmea)
# 그다음
ros2 launch lidar drive_gps.launch.py
```

### rosbag 재생 (라이다 없이)

```bash
ros2 launch lidar drive_lidar.launch.py use_ouster:=false drive:=false \
    survey_duration:=0.4 use_rviz:=true
ros2 bag play ~/catkin_ws/rosbag/txa1
```

---

## 4. 실차 전 점검

1. **D5 스위치가 자율주행인가** — 수동이면 arduino 가 조향 `x`·브레이크 0·페달
   펄스만 내보내 **ROS 명령이 전부 무시된다**
2. **E-STOP 이 풀려 있는가** — ★해제가 곧 출발이다★ 차 주변부터 확인
3. `ros2 topic echo /vehicle_mode` 가 `true` 인가
4. `ros2 topic echo /cmd_vel_raw` 의 `linear.x` 가 **정수**인가 (아니면 환산이 안 붙은 것)
5. 조향을 왼쪽으로 요구할 때 `angular.z` 가 **음수**인가

---

## 5. ★미실측 — 실차 전에 채울 것★

| 항목 | 지금 값 | 영향 |
|---|---|---|
| **라이다 커넥터 방향** | `flip_lidar_xy: true` (1/5카 값) | 앞뒤가 뒤집히면 AEB 가 뒤를 본다 |
| **차 폭 / 윤거** | `path_corridor_half_width: 0.8`, `track_width: 0.65` | AEB 코리도·충돌 판정 |
| **전/후 오버행** | 미사용(이 패키지엔 아직) | mppi 이식 때 필요 |
| **라바콘 코스 폭** | `initial_half_width: 2.2`, 밴드 1.0~4.0 m | 금색차가 안 지나갈 수 있다 |
| `aeb_max_decel` | 5.0 | ★이 차에 없는 감속도★ (2단 최대 3.8). 지금은 지령 슬루일 뿐 |

## 6. 남은 결정

- ~~`ouster_ros` 를 gold_ws 로 옮길 것인가~~ → **옮겼다 (2026-08-25)**.
  `gold_ws/src/ouster-ros/` 에 `ouster_ros` + `ouster_sensor_msgs` 두 패키지가 있다
  (`lidar/` 안에는 넣지 않았다 — 패키지 안에 패키지를 두면 colcon 이 꼬인다).
  ⚠️ 그 폴더는 **자체 `.git` 을 가진 별도 클론**이다(태그 0.13.13). gold 저장소는
  아직 서브모듈로 등록하지 않았다 — 커밋 방식은 별도 결정.
- **white1 driving 과의 관계** — 배타로 갈지, `driving` + `cone_lidar` AEB 조합으로
  갈지. 후자라면 `driving.py` 에 AEB 구독과 `/brake_level` 경로를 더해야 한다.
- **`mppi_local_planner` 이식** — 아직 안 했다. 라바콘 회피(지그재그)용.

## 7. 파일

```
include/lidar/kasa_units.hpp   ★금색차 액추에이터 계약의 단일 소유자★
include/lidar/gps_path.hpp     ENU 변환 · 최소자승 직선 · 스탠리
include/lidar/line_ekf.hpp     코리도 직선 EKF
src/cone_lidar_node.cpp        AEB (기하만 바뀜)
src/drive_lidar_node.cpp       코리도 헤딩홀드 (+ 액추에이터 · 제동 · 게이트)
src/drive_gps_node.cpp         GPS 추종 (+ /gps_fused · 액추에이터 · 제동 · 게이트)
src/pedal_drive_node.cpp       ★페달 주행 + AEB★ — 판정 래치 → /aeb_stop / 안전속도 감시
test/test_kasa_units.cpp       ★환산 단위시험★ — CHANGELOG 검산표를 그대로 확인
config/                        ouster_driver · cone_lidar · drive_lidar · drive_gps
launch/                        ouster · aeb · drive_lidar · drive_gps · ★one_launch★
```

```bash
colcon build --packages-select lidar --cmake-args -DCMAKE_BUILD_TYPE=Release
./build/lidar/test_kasa_units          # 환산이 맞는지 (ROS 불필요)
```
★`--symlink-install` 금지★ — 위 '빌드' 절의 상자 참고.
