# gold — kasa(금색차) 자율주행 스택 백업

`white_ws` ROS2 워크스페이스(`/home/mad2/white_ws/white_ws`) 안의 알고리즘·런치 파일만
선별해 백업하는 저장소입니다. 빌드 산출물(`build`/`install`/`log`)과 워크스페이스에 함께
있던 다른 프로젝트(`kasa_ws`, `skku_ws` 등)는 포함하지 않습니다.

## 구성

`gold_ws` 는 그대로 `colcon build` 가 되는 온전한 ROS2 워크스페이스다.
아두이노 펌웨어(`.ino`)는 ROS2 구성요소가 아니라 **이 저장소에 두지 않는다** —
원본은 `mad-code` 저장소에 있다(아래 참고).

```
gold_ws/                 ← ROS2 워크스페이스 (빌드는 여기서)
 src/
  white/   자율주행 판단 스택 (ament_python 패키지)
   launch/one_launch.py  GPS·IMU·카메라 연결 + 자율주행 노드 통합 런치
   calibration/          usb_cam 카메라 캘리브레이션 (share 로 설치됨)
   ros2bag/              ★주행 기록 산출물★ record 노드가 여기에 CSV 를 쌓는다
   white/
    driving.py           경로추종 (순수추종 + PID + LFD 스케줄)
    gps_imu.py           GPS/IMU/엔코더 융합 → /ego_state
    perception.py        카메라 인지 (차선 polyfit + 신호등 검출)
    camera_judgment.py   차선 계측 브리지 + 신호등 게이트 + 리니어 브레이크
    mapping.py           경로 수집(매핑)
    prompt.py            CLI 메뉴 (수집 / 자율주행 / ★수동조종 주행 계측(메뉴 9)★)
    sensor_monitor.py    센서 상태 대시보드
    record.py            주행 기록 (자율주행+주행모드 구간만 토픽 → CSV)
    iahrs.py             iAHRS IMU 드라이버
    ports.py             GPS·IMU·카메라 장치 경로 식별
    kasa_units.py        차량 단위 환산 단일 소유자 (펄스↔m/s, 조향 부호 규약)
    route_remodeler.py   수집 경로 후처리(직선/코너 리모델링)

  white806/  ★재작성판 — GPS+IMU 전용, 최소 추종★ (2026-08-06~)
   launch/one_launch.py  GPS·IMU·아두이노 + 자율주행 통합 런치
   gps_data/             매핑 산출물(경로 CSV)
   ros2bag/              주행 기록 CSV
   white806/
    driving.py           ★GPS+IMU 융합 + 모드스위치 상태기계 + 경로추종★
    iahrs.py             iAHRS 드라이버 → /imu (순수 드라이버)
    ports.py             GPS·IMU 장치 경로 식별 (카메라 코드 제거판)
    mapping.py           /fix 만 보고 경로 수집
    prompt.py            CLI (경로 선택·상태 표시)
    prompt_g.py          ★prompt 와 같은 기능의 tkinter GUI★ (버튼 3개 + 계기 4칸)
    record.py            자율주행 구간 토픽 → CSV (파일명 = 경로이름-시각)
    paths.py             저장 위치 단일 소유자

  nxde/    아두이노(kasa A/B 2보드) 통신 계층 (ament_python 패키지, 런치파일 없음)
   nxde/
    arduino.py           A/B 2보드 시리얼 브리지 — 차량 구동의 필수 노드
    master.py            마우스·키보드 GUI 조종 (하드웨어 검증용)
    joystick.py          조이스틱 조종 (자율주행 모드 한정 + 영점 후 SWA 필요)
    check.py             런치 전 하드웨어 연결 점검 (GPS RTK 상태 포함) 후 종료

```

### 아두이노 펌웨어는 이 저장소에 없다

현행 펌웨어는 **`kasa_0904_A.ino` (A보드 — 인휠 PID + 주행펄스)** 와
**`kasa_0904_B.ino` (B보드 — 조향 + 제동 + 모드스위치 + 제동등)** 이고,
원본은 **`mad-code` 저장소** 에 있다(`.ino` 전부가 거기 모여 있다).
Arduino IDE 로 보드에 직접 굽는 파일이라 colcon 빌드 대상이 아니고,
여기에 사본을 두면 두 곳이 갈라지므로 두지 않는다.

- 보드 ↔ ROS 규약(입출력 양식·E-STOP·핸드셰이크) : `gold_ws/src/white1/BOARD_B.md`
- 펌웨어 내부 구조·배선 정본 : `mad-code/CLAUDE.md`

> **📌 2026-09-04 (0904 펌웨어)**
> - **연결확인 핸드셰이크** — 보드에 `-` 한 줄을 보내 `YES` 를 받아야 정상 연결로
>   본다. `S,`/`P,` 텔레메트리는 *보드→PC* 한 방향만 증명하므로, 그것만 보고
>   런치를 올리면 TX 가 죽은 채로 차가 명령을 못 받는다.
>   `ros2 run nxde check` 와 `arduino.py` 가 같은 규약을 쓴다.
> - **E-STOP 확인시간 500ms → 100ms** (발동·해제 대칭).
> - **D12 를 판정하는 보드는 B 하나** (A보드는 `ESTOP_ENABLED = false`).
>   그 덕에 E-STOP 중에도 A보드는 `S,` 를 계속 내보내 정상 식별된다.
> - **★A보드 무입력 워치독 `[0904-4]`★** — 3초 동안 줄이 한 개도 안 오면 보드가
>   스스로 정지한다(`RX_TIMEOUT_MS = 3000`). 0713 부터 타임아웃이 아예 없어서,
>   상위가 SIGKILL·터미널 종료·USB 단선·PC 정지로 말을 멈추면 **마지막 명령을
>   영원히 물고 차가 계속 갔다.** 상위의 `KEEPALIVE_S`(1.0s, A·B 양쪽 재전송)와
>   **한 쌍이다** — 어느 한쪽만 늘리면 정상 주행 중에 구동이 끊긴다.
>   **B보드 펌웨어는 고치지 않았다** — 조향·제동은 그 상태로 멎어도 안전 문제가
>   없다(조향은 마지막 각도 유지, 리니어는 마지막 단수 유지).
>   상세는 `gold_ws/src/nxde/README.md` 3절.

## 실행

빌드 (Ubuntu 22.04 / ROS2 Humble):

```bash
cd gold_ws
colcon build --symlink-install
source install/setup.bash
```

`gold_ws` 를 통째로 쓰지 않고 기존 워크스페이스에 얹으려면 `gold_ws/src/white1`,
`gold_ws/src/nxde` 두 디렉터리만 그쪽 `src/` 로 복사하면 된다.

실행:

```bash
ros2 run nxde check                    # 0) 하드웨어 연결 점검
ros2 launch white1 one_launch.py       # 1) GPS·IMU·카메라·아두이노 + 자율주행
ros2 run white1 prompt                 # 2) CLI 메뉴 (별 터미널)

ros2 run nxde kill                     # 끝낼 때 / 종료가 질척거릴 때 ★2026-09-04★
```

> **★`colcon build --symlink-install` 을 권한다★** [2026-09-04]
> `--symlink-install` 없이 빌드하면 `white1/paths.py` 가 소스트리를 못 찾아
> `~/white1/{sound,gps_data,ros2bag}` 로 떨어졌다 — **음성 안내가 한 마디도 안 나오고
> 경로 CSV 목록이 비어 보이던** 원인이다. 그 폴백은 [2026-09-04] 에 고쳤으므로
> 복사설치로도 맞게 돌지만, 심볼릭 설치가 첫 줄에서 바로 맞는 정공법이다.
> **소스를 고친 뒤에는 반드시 다시 빌드할 것** — 복사설치본은 스스로 갱신되지 않는다.

### ★ 수동조종(D5) 주행은 어느 상태에서도 가능하다 ★

주행모드 스위치(B보드 D5)가 **수동조종**이면, 위 3단계 중 어디까지 떠 있든
사람이 페달·핸들로 차를 몬다. 필요한 것은 **`arduino` 노드 하나**뿐이다:

```bash
ros2 run nxde arduino                  # 이것만 떠 있어도 수동조종 주행이 된다
```

`arduino.py` 의 `compose()` 가 **수동조종을 `/control_state`·`/cmd_vel_raw` 보다 먼저
판정**하기 때문이다(우선순위 2). D5 가 개방인 동안에는 자율 명령을 아예 보지 않고
"A보드가 보고한 페달 raw → 주행펄스"를 되돌려 보내며, 조향은 힘빼기(`x`)로 두어
사람이 핸들을 돌릴 수 있게 한다. 그래서 `one_launch.py` 와 `prompt` 가 함께 떠 있어도
그 경로는 막히지 않는다.

프롬프트에서 그 상태를 계측으로 확인하려면 **메뉴 9(수동조종 주행)** 를 쓴다 —
페달 raw → 목표펄스 / 실 주행펄스 / 실측 조향각을 한 줄로 실시간 표시하며,
수집(메뉴 1)과 달리 **CSV 를 만들지 않는다**.

> ### ★★ 불변식 : 모드 전환은 절대로 리니어(브레이크)를 체결하지 않는다 ★★
>
> 자율↔수동 전환은 사람이 차를 넘겨주거나 넘겨받는 순간이다 — 그때 리니어가 밟히면
> 가장 위험하므로, 전환 엣지에서 브레이크 상태를 전부 '풀린' 쪽으로 되돌린다
> (`/brake_level` 요청 캐시 삭제 + `stop_brake_level` 무장 해제).
> **거는 로직이 아니라 지우는 로직이다.** 근거·검증표는 `nxde/README.md` 7절 참고.

## white806 — 재작성판 (GPS + IMU, 최소 추종)

하드웨어가 kasa A/B 2보드로 바뀌면서 `white` 의 제어 파라미터가 맞지 않아, 튜닝의
출발점이 될 만큼 단순한 구조로 다시 쓴 패키지다. `white` 는 그대로 남겨 둔다.

```bash
ros2 launch white806 one_launch.py     # 1) 하드웨어 + 자율주행
ros2 run white806 prompt               # 2) CLI (별 터미널)
ros2 run white806 prompt_g             # 2') 같은 기능의 GUI — 둘 중 하나만 띄운다
```

`prompt_g` 는 `prompt` 와 기능이 같고 조작면만 tkinter 창이다(매핑·주행·종료 버튼과
`/speed`·`/encoder`·조향각 명령·주행펄스 명령 4칸). 화면이 없는 ssh 세션에서는
`prompt` 를 쓴다. 둘을 동시에 띄우면 서로 다른 대기 상태를 들고 있게 되어 헷갈린다.

`white` 대비 없앤 것 — 카메라 체인 전부, `gps_imu` 노드(융합을 `driving` 이 직접 함),
`kasa_units`(`/cmd_vel_raw` 가 이미 펄스·도 단위라 환산이 불필요), CTE·순수추종·가변
LFD·지연보상. 남은 제어는 `조향 = clamp(−Kp · 헤딩오차, ±40°)` 와 고정 속도뿐이다.
아두이노 통신은 `nxde` 를 **수정 없이** 쓴다.

### 조작 — 트리거는 B보드 D5 모드 스위치의 **전환**이다

| 하려는 것 | 스위치 조작 |
|---|---|
| 매핑 시작 | 자율 → **수동** (이미 수동이면 자율로 올렸다 다시 내린다) |
| 매핑 종료 | 수동 → **자율** (경로 저장) |
| 주행 시작 | 수동 → **자율** (이미 자율이면 수동으로 내렸다 다시 올린다) |
| 주행 종료 | 도착 후 자율 → **수동** (기록 저장 + 리니어 해제) |

위치가 아니라 전환이 트리거라 같은 방향으로 두 번 올려도 아무 일도 일어나지 않는다.
E-stop 은 어느 상태에서든 즉시 메인화면으로 되돌리고, 풀리면 처음부터다.

출발할 때는 조향 0°로 곧게 굴러 **GPS 변위로 초기 헤딩을 잡는다**(자이로는 절대
기준이 없으므로). 거리를 미리 정하지 않고 추정 오차가 3° 아래로 떨어지는 순간
멈추므로, RTK Fixed 면 대개 1m 남짓에서 끝난다.

## 주행 기록

`record` 노드가 one_launch 와 함께 뜨고, **자율주행 모드(B보드 D5) + prompt 주행 구간**
에서만 스스로 켜져 토픽을 CSV 로 남긴다. 따로 시작·중지할 것이 없다.

```
white/ros2bag/rec_<날짜>_<시각>.csv                    ← 한 주행에 파일 하나
white806/ros2bag/<경로이름>-<날짜>_<시각>.csv          ← 재작성판 [2026-08-12]
```

`white806` 은 **어느 경로로 달렸는지를 파일명 앞에 붙인다** — 같은 날 여러 경로를
번갈아 달리면 시각만으로는 구별이 안 되기 때문이다. 경로 이름은 prompt 가 보내는
`/drive_cmd` 에서 받고, 놓치면 `/drive_event` 에서 줍는다(둘 다 놓치면 `unknown-`).

```
gps_data/route_20260811_160932.csv 로 주행
  → ros2bag/route_20260811_160932-20260812_134501.csv
```

**한 행 = 한 시점의 차량 전체 상태**다. 20Hz로 스냅샷을 찍어 그 순간 각 토픽의 최신값을
한 줄에 나란히 적으므로(87열), 열을 골라 바로 그래프가 된다. 토픽마다 주기가 달라서
수신할 때마다 한 줄씩 적으면 대부분 칸이 빈 표가 되기 때문에 이렇게 한다.

- 숫자 토픽은 다음 값이 올 때까지 값을 유지하고, 문자열·이벤트 토픽은 새로 온 행에만
  찍고 비운다(같은 문장을 20Hz로 반복하지 않으려고). 상태로 되살리려면 `ffill()` 한 번.
- 앞 두 열은 항상 `t_wall`(UNIX epoch)·`t_rel`(세션 시작 기준 경과 초).
- 기록 대상은 `white/record.py` 상단의 `RECORD_TOPICS` 표에 모여 있으니 늘리거나 줄일 때
  그 표만 고치면 된다.

```bash
ros2 launch white one_launch.py use_record:=false      # 기록 끄기
ros2 launch white one_launch.py record_dir:=/mnt/usb   # 저장 위치 바꾸기
```

자세한 노드 구성·토픽 계약·안전장치는 `nxde/README.md` 참고.

## 이 저장소의 성격

**작업 백업**이 목적이며, 원본 개발은 `/home/mad2/white_ws`(별도 git 저장소)에서 이루어집니다.
이 저장소는 그 워크스페이스의 `src/nxde`·`src/white` 스냅샷만 독립적으로 관리합니다.
