# nxde — 아두이노 계층 (white 자율주행 스택용)

> Ubuntu 22.04 / ROS2 Humble 전용. Windows 분기는 모두 제거했다.
> **이 패키지는 kasa A/B 2보드와 통신하는 노드들만 담는다. 런치파일은 없다.**
> `kasa_ws/src/nxde` 에서 통신·조종 노드를 가져와 토픽 계약을 white 규약으로 바꿨다.
> **kasa_ws 쪽은 수정하지 않았다.** 아두이노 펌웨어는 **`kasa_0904_A.ino` / `kasa_0904_B.ino`**
> (원본은 `mad-code` 저장소) 를 전제한다.

> ### 📌 2026-09-04 펌웨어 0904 — 이 패키지가 함께 바뀐 곳
> 1. **연결확인 핸드셰이크 (`-` → `YES`)** — 보드를 `S,`/`P,` 로 식별한 뒤 `-` 한 줄을
>    보내 `YES` 를 받아야 **연결로 인정한다**. 텔레메트리는 *보드→PC* 한 방향만 증명하고,
>    PC→보드 방향은 TX 선이 빠져 있어도 화면상 멀쩡해 보이기 때문이다. 응답이 없으면
>    포트를 닫았다가(=보드 리셋) 다시 열어 재확인한다.
>    `arduino.py identify_port` 와 `check.py identify_arduino` 가 **같은 규약**을 쓴다.
> 2. **E-stop 확인시간 500ms → 100ms** (A·B 공통).
> 3. **E-stop 을 판정하는 보드는 B 하나** — A보드는 `ESTOP_ENABLED = false` 다.
>    D12 NC 라인은 그대로 두 보드에 병렬로 물려 있고 B보드가 그 라인을 본다.
>    부수 효과로 **E-stop 중에도 A보드는 `S,` 를 계속 내보낸다** — 그 동안 A보드가
>    식별되지 않고 `/throttle_pedal` 이 얼어붙던 문제가 함께 사라진다.

> ### 📌 2026-08-05 구조 변경
> 하드웨어 담당을 다시 나눴다. 이전(2026-08-04)에는 `g.launch.py` 하나가 **모든** 하드웨어를
> 전담했는데, 그 과정에서 1/5카 시절 `one_launch.py` 의 장치 자동탐색이 통째로 빠져 문제가 생겼다.
>
> | | 담당 |
> |---|---|
> | **white** `one_launch.py` | GPS · IMU · 카메라 + 자율주행 노드 + **nxde 의 arduino 노드** |
> | **nxde** (이 패키지) | 아두이노 A/B 2보드 통신·조종 노드 4개. **런치파일 없음** |
>
> `g.launch.py` · `iahrs.py` · `ports.py` · `calibration/` 은 이 패키지에서 삭제됐다
> (IMU·장치식별·캘리브는 white 로 환원 — `white/white/ports.py`, `white/white/iahrs.py`).

---

## 1. 구조

노드 4개, 전부 `ros2 run` 으로 띄운다.

```
arduino    ★차량 구동의 필수 노드★  A/B 2보드 시리얼 브리지
           구독 /cmd_vel_raw /control_state /brake_level
           발행 /encoder /steer_angle_measured /vehicle_mode /throttle_pedal
                /drive_pulse_cmd /estop /board_status
master     마우스·키보드 GUI 조종 (하드웨어 검증용)
joystick   조이스틱 메가 보드("J,"/"U,") 조종 — ★자율모드 한정 + 영점→SWA★
check      ★런치 전 하드웨어 연결 점검★ 보고하고 종료
```

### 실행 조합

```bash
# ── 0) 먼저 연결을 확인한다 ★권장★ ─────────────────────────────
ros2 run nxde check

# ── 차량 조종만 (자율주행 스택 없이) ────────────────────────────
ros2 run nxde arduino        # 터미널 1 — 이게 없으면 아무것도 안 움직인다
ros2 run nxde master         # 터미널 2 — 마우스/키보드
#   또는
ros2 run nxde joystick       # 터미널 2 — 조이스틱 (master 와 동시 사용 금지)

# ── 자율주행 ───────────────────────────────────────────────────
ros2 launch white one_launch.py   # 터미널 1 — arduino 노드를 함께 띄운다
ros2 run    white prompt          # 터미널 2 — CLI 메뉴
```

⚠️ **`master` · `joystick` · `one_launch.py`(driving_node) 중 둘 이상을 동시에 띄우지 말 것** —
`/cmd_vel_raw` 발행자가 겹쳐 두 명령이 교대하며 차가 떤다. `master` 는 `driving_node` 가
떠 있으면 상단에 주황색 경고를 띄운다.

패키지 경계는 빌드 단위일 뿐이고, 통신은 DDS 가 토픽 이름·타입·QoS 로만 맺는다. 같은
`ROS_DOMAIN_ID`(기본 0)면 자동 연결되고 **기동 순서는 상관없다.**

### 파일 구성

```
nxde/arduino.py     A/B 2보드 브리지 (구 white/motor.py 대체)
                    ★포트 탐색표를 자체 소유★ — GPS/IMU VID·PID 를 스스로 제외한다
nxde/master.py      마우스·키보드 GUI 조종
nxde/joystick.py    조이스틱 조종 (kasa_ws 판을 white 규약으로 이식)
nxde/check.py       하드웨어 연결 점검 (자립형 — 어떤 패키지도 import 하지 않는다)
nxde/video.py       ★인지 카메라 화면 녹화★ /image_raw → video/cam-<시각>.mp4
                    구독만 한다(제어에 끼어들지 않는다). 장치를 직접 열지 않아
                    usb_cam 과 다투지 않는다 — 그쪽이 죽으면 신호등 인지가 죽는다.
nxde/proc_guard.py  부모 프로세스 사망 감지 (고아 방지, POSIX 전용판)
```

### `ros2 run nxde video` — 인지 화면 녹화

```bash
ros2 run nxde video                                    # 그대로 녹화
ros2 run nxde video --ros-args -p scale:=0.5           # 용량 1/4 (사람이 볼 용도면 충분)
ros2 run nxde video --ros-args -p fps:=30.0            # 실측 생략 = ★첫 프레임부터★ 적는다
ros2 run nxde video --ros-args -p codec:=MJPG          # .avi — 강제종료에도 앞부분을 살린다
```

`white1` 의 `usb_cam` 이 떠 있어야 한다(`use_camera:=true`). **저장 위치는 이 패키지
루트의 `video/`** 이고, **Ctrl-C · 런치 종료 모두 그 시점까지 재생 가능한 상태로 닫힌다**
(둘 다 실측 확인). 녹화되는 것은 **인지가 받는 프레임 그대로**이고 YOLO 박스·HUD 는
그리지 않는다 — 판정의 *입력* 을 남기는 것이 목적이다. 자세한 근거는 `video.py` 헤더.

| 주의 | |
|---|---|
| 용량 | 1080p 기준 **영상 1분당 20~60MB**. 남은 디스크가 500MB 밑이면 스스로 끝낸다 |
| CPU | 인코딩은 CPU 를 쓴다. 인지 FPS 가 떨어지면 `scale` 을 낮춘다 |
| 강제종료 | `kill -9`·전원차단은 mp4 를 살릴 수 없다 → 그 위험이 크면 `codec:=MJPG` |
| git | `*.mp4/avi/mkv/mov` 는 `.gitignore` 로 막혀 있다(이력에 넣지 않는다) |

### 의존 방향 — ★white 를 의존하지 않는다★

`white → nxde` 한 방향만 존재한다(one_launch.py 가 arduino 노드를 띄운다).
반대 방향을 추가하면 colcon 토폴로지 정렬이 순환에 걸려 빌드가 깨진다.

그래서 **장치 VID/PID 표가 세 곳에 나뉘어 있다.** 하드웨어를 바꾸면 함께 고쳐야 한다:

| 표 | 소유자 | 용도 |
|---|---|---|
| 아두이노 VID + GPS/IMU 제외목록 | `nxde/arduino.py` | 아두이노 탐색 |
| GPS · IMU · 카메라 | `white/white/ports.py` | 운용 경로 해석 |
| 위 전부 (사본) | `nxde/check.py` | 점검 보고 전용 |

### kasa_ws 원본과의 차이

| | kasa_ws 원본 | 이 패키지 |
|---|---|---|
| ROS 인터페이스 | `/in`·`/out`·`/info` (String) | white 규약 토픽 직접 (Twist/Bool/Int32) |
| 조종 입력 | master · joystick · csv_read · keyboard | **master · joystick** (csv/keyboard 미이식) |
| 조이스틱 안전장치 | 시작 시 일시정지만 | **+ 자율모드 한정 + 영점 완료 전 시작 불가** |
| 좌우 차동 | master.py 가 계산 | 하지 않음 (좌우 동일 펄스) |
| 직접 PWM(16~255) | master 의 PWM모드로 사용 가능 | **봉쇄** — A보드로 항상 단일값만 보낸다 |
| 최초 연결 | 생성자에서 블로킹 | **논블로킹** (백그라운드 스레드) |
| 플랫폼 | Windows COM* + POSIX | POSIX(`/dev/ttyACM*`·`/dev/ttyUSB*`)만 |

`clean.py`(psutil 의존)는 가져오지 않았다 — 그 파일이 대응하는 고아 프로세스 문제는
Windows 특유의 `.EXE` 래퍼 구조에서 발생하고 POSIX 에는 없다(`proc_guard.py` 헤더 참고).
잔재가 남으면 `pkill -f nxde.arduino` 또는 `fuser -k /dev/ttyACM*` 로 정리한다.

---

## 2. ★ `ros2 run nxde check` — 런치 전 연결 점검 ★

```bash
ros2 run nxde check                 # 전체 점검
ros2 run nxde check --no-camera     # 카메라 건너뛰기 (영상 판정이 제일 오래 걸린다)
ros2 run nxde check --quick         # 포트를 열지 않고 VID/PID 만 (즉시 끝남)
```

보고 항목 — 메가 A/B · 조이스틱 · GPS · IMU · 카메라, 그리고 udev 심볼릭링크 상태.

**왜 런치보다 먼저 돌리는가.** 런치를 띄운 뒤에 장치를 꽂으면 경합이 생긴다.
GPS·IMU·아두이노가 모두 같은 `/dev/ttyACM*`·`/dev/ttyUSB*` 대역을 나눠 쓰는데, 런치
시점에 없던 장치는 각 노드가 뒤늦게 찾으러 다니며 서로의 포트를 배타 open 으로 밀어낸다
(nmea 드라이버는 respawn 으로 계속 다시 뜨고, 아두이노 탐색은 포트를 5초씩 문다).
**꽂고 → check 로 초록불 확인 → 런치** 순서면 그 경합이 구조적으로 생기지 않는다.

### GPS 는 NMEA 를 실제로 읽어 RTK 를 판정한다

`check` 는 GPS 포트를 열어 `$GxGGA` 문장의 fix quality **원값**을 보고한다:

| quality | 의미 |
|---|---|
| 0 | 측위 불가 |
| 1 | 단독 GPS (~2-5m) — RTK 아님 |
| 2 | DGPS/SBAS — RTK 아님 |
| **4** | **RTK Fixed** (고정해, ~2cm) |
| **5** | RTK Float (부동해, ~수십cm) |

⚠️ **ROS 쪽에서는 4 와 5 를 구분할 수 없다.** `nmea_navsat_driver` 가 4·5·9 를 모두
`NavSatStatus.STATUS_GBAS_FIX(2)` 로 뭉개기 때문이다(`sensor_monitor` 는 `<2` 면 "RTK 아님"
경고를 띄운다). 그래서 정확한 진단은 이 명령으로 한다.

⚠️ **GGA 문장이 아예 없으면 RTK 여부를 알 방법이 없다.** RMC 만 오는 설정이면 드라이버가
status 를 FIX/NO_FIX 로만 채운다 — u-center 에서 GGA 출력을 켤 것. `check` 가 이 상황을
따로 경고한다.

### ★ RTK 보정은 이 워크스페이스가 만들지 않는다 ★

`nmea_serial_driver` 는 수신기가 내보내는 NMEA 를 **읽을 뿐**이고 RTCM 보정을 넣지 않는다.
워크스페이스 전체에 NTRIP/RTCM 클라이언트가 없다(1/5카 시절에도 없었다). 즉 RTK 는
**수신기 쪽 경로**(기지국 라디오 링크 또는 별도로 띄우는 NTRIP 클라이언트)로 들어와야 한다.
"RTK 가 안 붙는다"면 먼저 그 경로를 확인하고, 그 다음 위 GGA quality 를 본다.

---

## 3. 연결 실패 / 도중 단절 대응

**장치가 하나도 안 꽂혀 있어도 모든 노드가 정상 기동한다.** 각 노드가 자기 장치를 계속
다시 찾으므로, 나중에 꽂거나 도중에 뺐다 꽂아도 자동으로 붙는다.

| 노드 | 최초 실패 | 도중 단절 | 수단 |
|---|---|---|---|
| `arduino` | 백그라운드 재스캔 (3s) | 재스캔 — **한쪽만 빠져도 나머지는 계속 동작** | 자체 `_link_loop` 스레드 |
| `joystick` | 3s 재연결 | 재연결 + **즉시 정지·일시정지 재무장** | 자체 리더 스레드 |
| `iahrs` (white) | 2s 재시도 + VID/PID 재탐색 | 재시도 + 재탐색 | 자체 재연결 타이머 |
| GPS (외부) | respawn (3s) | respawn | ★udev 링크 필요★ |
| `usb_cam` (외부) | respawn (3s) | respawn | ★`video_device` 경로 고정★ |

- `arduino` 는 **생성자가 블로킹하지 않는다.** 예전에는 두 보드를 다 찾을 때까지
  `__init__` 안에서 돌아 노드가 spin 조차 못 했다(구독·`/board_status` 전부 죽어 있었다).
- 전송 실패는 곧 포트 단절로 보고 그 보드만 떨어뜨린다. 그때 **변경감지 캐시를 비우므로**
  재연결 직후 최신 명령이 즉시 다시 나간다(명령 유실 없음).
- **아두이노 탐색은 GPS/IMU 포트를 건드리지 않는다.** `arduino.py` 가 GPS(u-blox)·IMU
  (CP210x)의 VID/PID 와 udev 링크(`/dev/gps`·`/dev/imu`)를 스스로 제외한다. 예전에는
  `g.launch.py` 가 경로를 확정해 `exclude_ports` 로 넘겨줬는데 런치가 갈라지며 그 전달이
  끊겼다 — 그래서 노드가 직접 판단하도록 옮겼다.

### ⚠️ 종료 : arduino 노드를 남기지 말 것

A보드 펌웨어에는 무입력 타임아웃이 없다(0713에서 제거). 마지막 수신 명령을 계속 물고 있다.

- **`one_launch.py` 를 Ctrl+C** → arduino 도 함께 내려가고, 종료 직전에 정지값
  (`0` / `x,0`)을 시리얼로 직접 써 넣는다(`stop_and_close`). 차가 선다. **안전**
- **arduino 만 남기고 자율주행 노드를 내리면** → `/cmd_vel_raw` 가 끊길 뿐이고 arduino 는
  마지막 명령을 1초 주기로 계속 재전송한다. **차가 계속 간다**

급할 때는 E-stop 스위치를 쓴다.

---

## 4. `ros2 run nxde master` — 하드웨어 검증 GUI

판단 스택을 올리기 전에 **차가 실제로 움직이는지** 확인하는 도구다.

```bash
ros2 run nxde arduino --ros-args -p manual_pulse_max:=3   # 터미널 1
ros2 run nxde master                                      # 터미널 2
```

- **자율주행 모드** : 마우스(또는 키보드 ↑↓←→)로 엑셀·조향 레버를 움직이고, 발행 토글을
  ON 으로 두면 차가 움직인다.
- **수동조종 모드** : 레버가 잠기고 **실측을 비추는 계기판**이 된다 — 페달을 밟으면 엑셀
  레버가 올라가고(`/drive_pulse_cmd`), 핸들을 돌리면 조향 레버가 움직인다
  (`/steer_angle_measured`). "밟히는 게 보인다".
- **E-stop** 이 걸리면 상단이 빨간 `E-Stop 발동!!!` 으로 바뀐다.
- 주행모드 박스는 **표시 전용**이다(2026-08-07). 모드를 바꾸는 것은 **B보드 D5
  물리 스위치뿐**이며 `/vehicle_mode_cmd` 는 삭제됐다 — 사람이 운전대를 잡고 있는지를
  뜻하는 값이라 화면 클릭으로 뒤집으면 위험하기 때문이다.

레버 구성:

| 레버 | 범위 | 키보드 |
|---|---|---|
| 엑셀 | 0 ~ 15 펄스 (1펄스 = 3.18 km/h) | `↑` `↓` |
| **브레이크** | **0 / 1 / 2 단계** (0 놓음 / 1 행정 1/3 / 2 풀) | `PgUp` `PgDn` |
| 조향 | −40 ~ +40 도 (**왼쪽 끝이 −, 오른쪽 끝이 +**) | `←` `→` |

체크할 것:

| 확인 항목 | 기대 결과 |
|---|---|
| 조향 레버를 **오른쪽(`+`)** 으로 | 바퀴가 **오른쪽**으로 (레버 방향 = 바퀴 방향). 반대면 `steer_invert:=true` |
| 명령 조향각 ↔ 실측 조향각 | 부호까지 같은 값으로 몇 초 안에 수렴 (B보드 PD 폐루프) |
| 엑셀 1~3펄스 | 실측 주행펄스(좌+우 합)가 명령의 **약 2배**로 올라온다 |
| **브레이크 1단** | 리니어가 행정의 1/3(83카운트)까지 밀린다. **2단은 풀브레이킹이라 1단부터 확인** |
| 수동조종에서 페달 | 엑셀 레버가 따라 올라온다 |
| E-stop 스위치 | 상단 빨간 경고 + 실측값 정지 + B보드가 리니어 2단 체결 |

### 토픽으로 직접 확인 (GUI 없이)

```bash
ros2 topic pub /control_state std_msgs/Bool "{data: true}"
ros2 topic pub -r 10 /cmd_vel_raw geometry_msgs/Twist \
  "{linear: {x: 1.0}, angular: {z: 10.0}}"         # 1펄스 전진 + ★우조향★ 10°
ros2 topic pub /brake_level std_msgs/Int32 "{data: 1}"   # 브레이크 1단 (약)
ros2 topic echo /board_status                      # A:1,B:1,ESTOP:0,MODE:1
ros2 topic echo /encoder
```

---

## 5. `ros2 run nxde joystick` — 조이스틱 조종

```bash
ros2 run nxde arduino     # 터미널 1
ros2 run nxde joystick    # 터미널 2   (master 와 동시 사용 금지)
```

### ★★ 안전장치 2개 ★★

**① 주행모드 스위치(B보드 D5)가 '자율주행' 일 때만 작동한다.**
`/vehicle_mode == False`(수동조종)면 입력을 무시하고 정지값만 낸다 — 수동에서는 사람이
페달·핸들을 잡고 있고 arduino 가 그 경로를 직접 넘기므로, 조이스틱까지 쏘면 사람과 싸운다.
**자율 → 수동으로 내려갔다 돌아오면 일시정지로 재무장된다** (SWA 를 다시 눌러야 한다).
E-stop 발동도 같은 방식으로 재무장시킨다.

**② 첫 실행 시 영점을 먼저 잡고, SWA 를 한 번 눌러야 입력이 반영된다.**
연결되면 스틱을 건드리지 않은 상태에서 20샘플을 모아 중앙값을 영점으로 잡는다
(그동안 어떤 명령도 나가지 않는다). **영점이 끝나기 전에는 SWA 를 눌러도 풀리지 않는다.**
스틱이 물리적으로 정확히 중앙(512)에 있지 않으므로, 영점 없이 시작하면 '가만히 둔 스틱'이
이미 펄스를 요구하는 상태일 수 있다.

### 조작

| 입력 | 동작 |
|---|---|
| L스틱 위 | 주행 펄스 0 ~ `pulse_max` (기본 **5**) |
| L스틱 아래 | 브레이크 단계 — 중앙~맨아래 3등분 → 0 / 1 / 2 (리니어모터) |
| R스틱 좌우 | 조향각 −40 ~ +40 (★− 좌 / + 우★) |
| **SWA 짧게** | 시작 / 일시정지 토글 (영점 완료 후에만) |
| 메가 리셋 | 3초 뒤 그 시점 값으로 영점 재보정 |

지원 보드는 접두어로 구분한다(둘 다 메가라 VID/PID 로는 구분 불가):
`"J,"` = `joy.ino`(11필드) / `"U,"` = `joy2.ino`(6필드, 축 규격이 달라 내부에서 변환).
**U 보드에는 SWA 가 없으므로 L·R 스틱 버튼 동시 누름이 SWA 를 대신한다.**

### kasa_ws 원본에서 뺀 것

**A0 펄스 모드 · PWM 테스트 모드 · 좌/우 독립 출력**을 제거했다. `arduino.py` 가 A보드로
항상 '단일값'만 보내기 때문이다 — 직접 PWM(16~255)은 PID·슬루레이트·폭주감지가 전부
빠지는 무보호 경로라 이 스택에서 봉쇄했다. 그래서 D13·SWB 에 걸려 있던 기능도 함께 없다.
tkinter GUI 대신 **터미널 한 줄 상태표시**로 줄였다(계측 확인은 `master` 가 담당).

---

## 6. 토픽 계약

### ROS → 보드

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/cmd_vel_raw` | `geometry_msgs/Twist` | `linear.x` = 주행 목표펄스 **0~15 (m/s 아님)**<br>`angular.z` = 조향각 **−40~40, ★− 좌 / + 우★** |
| `/control_state` | `std_msgs/Bool` | `True` = 구동 허용 / `False` = 정지 |
| `/brake_level` | `std_msgs/Int32` | 브레이크 **단계 0 / 1 / 2** (★0~255 PWM 아님★). 선택 — 안 오면 0 |

> **[2026-08-07] `/vehicle_mode_cmd` 는 삭제됐다.** 주행모드의 소유자는 B보드 D5
> 물리 스위치 하나다. **[2026-08-11] 수동조종에서 `/cmd_vel_raw` 펄스를 받던 경로도
> 되돌렸다** — white806 매핑의 헤딩 초기화를 위해 열어 뒀던 경로인데, 그 절차가
> '사람이 페달로 직접 곧게 굴리는' 방식으로 바뀌어 필요 없어졌다. 주행 펄스는
> 다시 ★페달뿐★ 이다. 조향은 수동에서 언제나 힘빼기다.

**`/brake_level` 발행자:** `master`(레버) · `joystick`(L스틱 아래) ·
**`camera_judgment`(신호등 완전정지 시 2단)**.

### ★ 조향 부호 규약 (2026-08-04 개정) ★

**ROS 토픽 · 시리얼 · 펌웨어 · GUI 가 모두 같은 부호를 씁니다: 음수 = 좌회전 / 양수 = 우회전.**

이전에는 "ROS 안은 white 부호(`+`=좌), arduino.py 가 반전"이었는데, GUI 의 가로 조향 레버는
왼쪽 끝이 `−40` 이라 **레버를 오른쪽으로 밀면 차가 왼쪽으로 가는** 문제가 실차 시험에서
나왔습니다. 그래서 규약을 kasa B보드 기준으로 통일했습니다.

- `arduino.py` 의 `steer_invert` 기본값 = **`false`** (반전 없음)
- 유일한 예외: `driving.py` 제어기 **내부**는 여전히 `+`=좌 (순수추종·PID·`STEER_PLANT_GAIN_L/R`·
  `STEER_TRIM_DEG` 가 그 전제로 실측·튜닝된 값이라 의미를 보존). 그 반전은
  **`driving.publish_cmd` 의 `ku.to_ros_steer()` 한 줄에서만** 일어납니다.
- 즉 **부호가 뒤집히는 지점은 코드 전체에서 그 한 줄뿐입니다.** 두 번 뒤집으면 조용히 좌우가 바뀝니다.
- ⚠️ **1/5카가 기록한 구맵의 `steer` 컬럼은 반대 부호(`+`=좌)입니다.** 구맵을
  `tool/map_check.py` 로 분석하면 좌우가 뒤집혀 보입니다. 주행에는 영향이 없습니다 —
  `driving.py` 는 CSV 의 `steer` 컬럼을 읽지 않습니다.

### 보드 → ROS

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/encoder` | `Int32` | A보드 **좌+우 펄스의 합** (부호 없음, 20ms 창). 1카운트 = 0.442 m/s |
| `/steer_angle_measured` | `Int32` | B보드 가변저항 실측 조향각 (**− 좌 / + 우**, 그대로 중계) |
| `/vehicle_mode` | `Bool` | B보드 D5 : `True` 자율주행 / `False` 수동조종 |
| `/throttle_pedal` | `Int32` | A보드 A0 쓰로틀 페달 raw 0~1023 |
| `/drive_pulse_cmd` | `Int32` | **A보드로 실제 나간 주행 목표펄스** (자율=계획값 / 수동=페달 환산값) |
| `/estop` | `Bool` | A·B 중 한쪽이라도 `STOP` 송신 중이면 `True` (OR) |
| `/board_status` | `String` | `"A:1,B:1,ESTOP:0,MODE:1"` — 진단·로스백용 |

### 센서 (white 소관)

| 토픽 | 발행 노드 | 패키지 |
|---|---|---|
| `/imu/data` (+ TF `base_link→imu_link`) | `iahrs` | **white** |
| `/fix` | `nmea_serial_driver` | 외부 |
| `/image_raw` | `usb_cam` | 외부 |

`/motor_pwm`·`/steer_pwm` 은 **발행하지 않는다** — kasa 펌웨어가 PWM 을 텔레메트리로
내보내지 않는다. white 쪽 구독자도 없었으므로(로스백 진단 전용) 그냥 사라진다.

### 시리얼 (참고 — arduino 노드 내부에서만 쓰인다)

```
A보드 입력 :  <펄스>\n                  0~15 만 (단일값 = 펄스 전용 경로)
A보드 출력 :  S,<좌펄스>,<우펄스>,<쓰로틀raw>\n     (50ms) / STOP\n (e-stop 중)
B보드 입력 :  <조향각>,<브레이크단계>\n   조향각 정수 −40~40 또는 'x'(힘빼기)
                                        브레이크 단계 0/1/2 (★0~255 PWM 아님★)
B보드 출력 :  P,<조향각>,<모드>\n         (50ms) / STOP\n (e-stop 중)
조이스틱   :  J,<11필드>\n  또는  U,<6필드>\n      (joystick 노드가 별 포트로 읽는다)
```

---

## 7. 주행 상태 판단 (우선순위)

`arduino.py` 의 `compose()` 가 매 전송 주기에 아래 순서로 판정한다.

| 우선 | 조건 | A보드 | B보드 |
|---|---|---|---|
| 1 | **E-stop** (`STOP` 수신) | `0` | `x,0` |
| 2 | **수동조종** (D5 개방) | 페달 환산 펄스 | `x,0` |
| 3 | `/control_state=False` | `0` | `<마지막 조향각>,max(stop_brake_level*, /brake_level)` |
| 4 | 정상 자율주행 | `<펄스>` | `<조향각>,<​/brake_level>` |

<sub>* `stop_brake_level` 은 **무장된 뒤에만** 쓰입니다 — 바로 아래 불변식 ② 참고.</sub>

`/brake_level` 은 **우선순위 4(정상 자율주행)에서만 그대로 반영**됩니다. 3에서는
`stop_brake_level` 과 큰 쪽을 취합니다 — '정지 지시'가 더 강한 의도이므로, 그때 마침
`/brake_level=0` 을 받고 있었다고 브레이크를 풀면 안 됩니다. 1·2에서는 아래 규칙이 이깁니다.

- **E-stop** : 리니어 2단 체결과 해제(0단 복귀)는 **B보드 펌웨어가 스스로 한다**
  (`kasa_0904_B.ino`). ROS 가 브레이크를 지시할 필요가 없고, e-stop 중에는
  B보드 `handleLine` 이 명령을 통째로 무시한다. `x,0` 을 보내는 이유는 **해제 직후에
  적용될 마지막 명령**을 안전하게 두기 위함이다(조향 힘빼기 = 급조향 없음).
- **수동조종** : 사람이 핸들·페달을 직접 잡는다. `/control_state` 와 **무관하게 항상** 이
  경로다 — 자율 명령을 보내면 사람과 싸운다. **즉 arduino 노드만 떠 있어도 수동주행이 된다.**
  - 조향 `x`(힘빼기) — DC모터에 힘이 들어가면 사람이 핸들을 못 돌린다
  - 주행은 A보드가 보고한 페달 raw 를 펄스로 환산해 되돌려 보낸다
  - **브레이크는 항상 `0`(놓음)** — 제동은 사람 발이 한다.
  - ★`one_launch.py` · `prompt` 가 함께 떠 있어도 이 경로는 막히지 않는다★ 우선순위 2 가
    3·4 보다 앞이므로 자율 스택의 `/cmd_vel_raw`·`/control_state` 는 무시된다.
    프롬프트에서 계측으로 확인하려면 **메뉴 9(수동조종 주행 — 기록 없음)** 를 쓴다.

> ## ★★ 불변식 : 모드 전환은 절대로 리니어를 체결하지 않는다 ★★
> ### (2026-08-05 — 아래 🚫 경고와 같은 원칙을 코드로 강제한 것)
>
> 자율↔수동 전환은 **그 자체가 제동 지시가 아니다.** 사람이 차를 넘겨주거나 넘겨받는
> 순간이므로, 하필 그때 리니어가 밟히면 가장 위험하다. 그래서 전환 엣지에서 브레이크
> 관련 상태를 **전부 '풀린' 쪽으로 되돌린다** — `_disarm_brakes_on_mode_edge()`:
>
> **① `/brake_level` 요청 캐시를 `0` 으로 지운다**
> 수동조종으로 사람이 몰고 있어도 `camera_judgment` 는 계속 돌아간다(그 노드는 D5 를
> 보지 않는다). 그래서 사람이 운전하는 중에 빨간불이 확정되면 `/brake_level=2` 가
> 발행된다. 수동 분기는 브레이크 `0` 을 보내니 그 순간은 무해하지만 값이 캐시에 남고,
> **D5 를 자율로 되돌리는 순간 우선순위 3/4 가 그 값을 집어 리니어가 튀어나온다** —
> 2026-08-04 에 제거한 증상과 겉모습이 똑같다.
>
> **② `stop_brake_level` 무장을 해제한다**
> 그 값을 `1` 이상으로 두면 전환 직후 우선순위 3 이 그것을 건다 — 그때 `/control_state`
> 는 아직 `False` 다(자율주행을 아직 시작하지 않았다). 즉 **사람이 차를 넘겨주는 바로 그
> 순간 리니어가 밟힌다.** `stop_brake_level` 의 뜻은 '자율주행이 몰다가 세울 때의 제동'
> 이지 '아직 한 번도 몰지 않은 대기 상태'가 아니므로, 자율주행이 실제로 구동 허가
> (`/control_state=True`)를 받은 뒤에만 무장한다. **기본값 `0` 에서는 어느 쪽이든 차이가 없다.**
>
> ⚠️ **①②는 브레이크를 '거는' 로직이 아니라 '지우는' 로직이다.** 2026-08-04 에 삭제된
> 래치를 되살린 것이 아니라 방향이 정반대다. 걸 수 있는 경로는 여전히 셋뿐이다
> (`/brake_level` · `stop_brake_level` · E-stop = B보드 자체 동작).
>
> **감수하는 것.** `camera_judgment` 는 값이 바뀔 때만 발행하므로, 자율로 되돌린 뒤에도
> 같은 빨간불이 계속 확정 상태면 2단을 다시 보내지 않는다(그 구간은 리니어가 빠진다).
> 그래도 안전한 쪽이다 — 그 상태의 속도명령은 이미 0이고, `stop_brake_level` 기본값도
> 0(코스트)이라 자율 정지의 기본 동작과 같다. 게이트 상태가 바뀌는 순간 다시 동기화된다.
>
> **검증(하드웨어 없이 재현 가능).** 전환 직후 B보드 페이로드의 브레이크 필드를 본다:
>
> | 상황 | `stop_brake_level` | 결과 |
> |---|---|---|
> | 수동→자율, 남은 `/brake_level=2` | 0 | `0,0` ✅ |
> | 자율→수동, 남은 `/brake_level=2` | 0 | `x,0` ✅ |
> | 수동→자율 (`control_state` False/True 무관) | **1** | `0,0` ✅ |
> | 자율→수동 | **1** | `x,0` ✅ |
> | **전환 아님** — 자율주행 개시 후 정지 | 1 | `…,1` (종전대로 걸린다) |
> | 그 정지 중 `/brake_level=2` | 1 | `…,2` (큰 쪽) |

> ### 🚫 ★수동조종 전환 시 리니어 2단 체결 로직은 제거됐다 (2026-08-04) — 되살리지 말 것★
>
> 예전에는 자율→수동 전환 엣지에서 `manual_brake_level`(2단)을 물고, 쓰로틀 raw 가
> `manual_release_raw` 를 넘으면 풀었다. **실차에서 스위치를 수동으로 내리는 순간
> 리니어가 브레이크 페달을 밟고 튀어나왔다** (E-STOP 도 아닌데).
> 모드 전환은 그 자체가 제동 지시가 아니다.
>
> 로직과 두 파라미터(`manual_brake_level`·`manual_release_raw`) 모두 삭제됐다.
> 구 `g.launch.py` 가 `default_value='2'` 로 명시 전달하고 있어서 `arduino.py` 기본값을
> 0 으로 바꿔도 런치가 계속 2 로 덮어썼던 이력이 있다("고쳤는데 여전히 리니어가 나온다"의 원인).
>
> **지금 ROS 가 리니어를 움직이는 경로는 셋뿐이고 전부 명시적 지시다:**
> ① `/brake_level` (master 레버 · joystick L스틱 · **camera_judgment 신호등 정지 2단**)
> ② `stop_brake_level` (자율 정지 시, 기본 0)
> ③ E-stop (ROS 무관 — B보드 펌웨어가 스스로)
>
> "E-STOP 도 아닌데 리니어가 튀어나왔다"면 원인은 100% 위 ①② 중 하나다.

- **`/control_state=False`** : 조향각을 0 으로 리셋하지 않고 마지막 값을 유지한다
  (정지 순간 바퀴가 정면으로 튀는 것을 막는다). 기본값은 코스트(`stop_brake_level=0`).
  더 빨리 세우려면 `1` 로 올린다.

---

## 8. ★ udev 설정 (사실상 필수) ★

GPS·IMU·아두이노 A/B 가 **전부 `/dev/ttyACM*`·`/dev/ttyUSB*` 대역을 공유한다.**
`/dev/ttyUSB0` 같은 열거 순서 의존 경로를 쓰면 재부팅·재연결 때 다른 장치를 열 수 있고,
GPS·카메라는 respawn 이 고정 경로로 재시도하므로 경로가 흔들리면 복구되지 않는다.

```bash
# 1) 각 장치의 시리얼 번호 확인 (하나씩 꽂아가며)
udevadm info -a -n /dev/ttyACM0 | grep -m1 'ATTRS{serial}'

# 2) /etc/udev/rules.d/99-white.rules 작성
#    ID_MM_DEVICE_IGNORE : ModemManager 가 GNSS 포트를 모뎀으로 프로빙하며 열어버리는 것을
#      막는다 (드라이버가 "multiple access on port" 로 죽는 원인)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", SYMLINK+="gps", ENV{ID_MM_DEVICE_IGNORE}="1"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="imu", ENV{ID_MM_DEVICE_IGNORE}="1"
# 아두이노는 A/B 가 같은 VID/PID 이므로 시리얼 번호로 구분한다(역할 식별은 접두어가 하므로
# 링크 이름이 뒤바뀌어도 무해하다 — 탐색 범위를 좁히는 최적화일 뿐이다)
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{serial}=="<A보드 시리얼>", SYMLINK+="kasa_a", ENV{ID_MM_DEVICE_IGNORE}="1"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="kasa_b", ENV{ID_MM_DEVICE_IGNORE}="1"

# 3) 적용
sudo udevadm control --reload-rules && sudo udevadm trigger
ros2 run nxde check               # ★링크와 연결을 한 번에 확인★

# 4) dialout 그룹 (권한 오류 시)
sudo usermod -aG dialout $USER    # 재로그인 필요
```

`white/white/ports.py` 의 `resolve_device()` 가 **① udev 링크 → ② VID/PID 스캔 → ③ 링크 경로
그대로 반환** 순서로 동작한다. ③ 이 중요하다 — 장치가 아직 없어도 **안정적인 이름**을
넘겨두면 나중에 꽂는 순간 그 경로가 생기고 respawn·재연결이 자동으로 붙는다.

아두이노는 udev 링크가 없어도 동작한다. 역할 식별은 첫 텔레메트리 접두어(`S,`=A / `P,`=B)로
한다 — A/B 두 대가 같은 VID/PID 라서 VID/PID 로는 구분할 수 없다.

카메라는 `/dev/video*` 라 udev 링크를 쓰지 않는다. `ports.resolve_camera()` 가
**`video2` → `video0` 순으로 시도하되 각 후보를 실제로 열어 프레임을 검증**한다
(검정/포화/동결이면 제외). 순서만 믿으면 덮개 닫힌 내장 웹캠을 열어 검은 화면으로
주행하게 된다 — 2026-08-04 에 실제로 있었던 사고다.

---

## 9. 파라미터 — `arduino` 노드

```bash
ros2 run nxde arduino --ros-args -p manual_pulse_max:=3
# 또는 런치에서:  ros2 launch white one_launch.py manual_pulse_max:=3
```

| 이름 | 기본 | 설명 |
|---|---|---|
| `baud` | 115200 | A/B 공통 |
| `steer_invert` | **`false`** | 조향 부호 반전. ROS·보드가 같은 규약(− 좌 / + 우)이라 기본은 반전 없음. 배선/펌웨어를 뒤집었을 때만 `true` |
| `stop_brake_level` | `0` | `/control_state=False` 시 브레이크 단계 (0=코스트 / 1=약). ★[2026-08-05] **자율주행이 한 번이라도 `/control_state=True` 를 받은 뒤에만** 적용된다★ — 모드를 자율로 올리는 순간에는 걸리지 않는다(7절 불변식 ②). 기본 0 이면 차이 없음 |
| `manual_pulse_max` | `15` | 수동조종에서 페달 최대치가 대응할 펄스. **수집·초기 시험에서는 3~5 로 낮출 것** |
| `throttle_raw_min` / `_max` | `177` / `800` | 페달 실측 (2026-07-30) |
| `exclude_ports` | one_launch.py 자동 | 아두이노 탐색에서 **추가로** 제외할 경로. ※ 안 넘겨도 GPS/IMU VID·PID 와 udev 링크는 노드가 스스로 걸러낸다 |

> ~~`manual_brake_level`~~ / ~~`manual_release_raw`~~ 는 **삭제됐다** — 7절의 🚫 경고 참고.

### `joystick` 노드

| 이름 | 기본 | 설명 |
|---|---|---|
| `pulse_max` | `5` | 조이스틱으로 낼 수 있는 최대 펄스. **초기 시험에서는 3 정도로** |
| `deadzone_raw` | `120` | 영점 기준 raw ADC 데드존 |
| `require_auto_mode` | **`true`** | 자율주행 모드 게이트. 벤치 시험용 탈출구이며 **실차에서 끄지 말 것** |

---

## 10. 수집(매핑) 절차 — ★수동조종 모드★

무선 컨트롤러는 쓰지 않는다. 사람이 차에 타서 **실제 페달과 핸들로** 몰고,
그때의 실계측을 `mapping` 노드가 CSV 로 기록한다.

```bash
ros2 run nxde check                                            # 0) 연결 확인
ros2 launch white one_launch.py manual_pulse_max:=3            # 1) 페달 상한 억제
ros2 run white prompt   →  차량 D5 스위치를 '수동조종' 으로  →  메뉴 1   # 2)
```

`prompt` 가 모드를 강제한다 — 자율주행 모드에서 `1`(수집)을 고르면 "스위치를 수동조종으로
전환하세요" 안내만 띄우고 메뉴로 돌아간다. 반대로 `2`(주행)는 자율주행 모드에서만 된다.
E-stop 이 걸린 동안에는 둘 다 막힌다.

### ★ 기록 없이 그냥 몰고 싶을 때 — 프롬프트 메뉴 9 ★

```bash
ros2 run nxde arduino                  # ★이것만 떠 있어도 수동조종 주행이 된다★
#  또는 자율주행 스택까지 띄운 상태 그대로:
ros2 launch white one_launch.py
ros2 run white prompt   →  D5 를 '수동조종' 으로  →  메뉴 9
```

메뉴 9 는 **주행을 허가하는 기능이 아니다** — 수동조종 경로는 `arduino` 노드만 떠 있으면
항상 살아 있다(7절 우선순위 2). 이 화면이 하는 일은 두 가지다:

- 자율 의도를 확실히 내려둔다 (`/control_state=False` + `/drive_cmd STOP`) —
  사람이 D5 를 자율로 되돌리는 순간 남아 있던 경로추종 명령이 나가지 않게 한다
- 계측 한 줄을 실시간 표시한다 : `페달 raw → 목표펄스(m/s) │ 실측 카운트(m/s) │ 실측 조향각`
  (`/throttle_pedal` `/drive_pulse_cmd` `/encoder` `/steer_angle_measured`)
  계측이 1초 이상 끊기면 "계측 두절 — arduino 노드/보드 연결 확인" 으로 바뀐다

수집(메뉴 1)과의 차이는 **CSV 를 만들지 않는다**는 것뿐이다.

기록되는 수동조종 실계측 3종 (`route_*.csv` 뒤쪽 컬럼):

| 컬럼 | 토픽 | 의미 |
|---|---|---|
| `throttle_pulse` | `/drive_pulse_cmd` | ① 페달 raw → 환산된 주행 목표펄스 (0~15) |
| `wheel_pulse` / `wheel_speed` | `/encoder` | ② 실제로 돈 주행 펄스(좌+우 합) 와 그 m/s 환산 |
| `steer_measured` | `/steer_angle_measured` | ③ DC 조향모터 가변저항 실측 각도 [deg, **− 좌 / + 우**] |
| `throttle_raw` | `/throttle_pedal` | 부수 : A0 페달 원값 0~1023 |
| `auto_mode` / `estop` | `/vehicle_mode` `/estop` | **수집 유효구간 판별용** — 1 인 행은 사람 조작이 아니다 |

기존 `steer` 컬럼(열 위치 5)도 ③ 의 값으로 채운다 — 예전에는 `/ego_state[6]`(gps_imu 가
항상 0 을 넣는 미사용 필드)에서 읽어 **늘 0.00 이었다.** 열 위치는 `route_remodeler` 와
구버전 분석툴 호환을 위해 그대로 유지했다.

`direction` 컬럼은 항상 `+1` 이다 — **자율주행에서 후진을 쓰지 않는다**(사용자 결정).
컬럼 자체는 구맵 호환을 위해 유지한다.

---

## 11. 알려진 한계

- **명령 분해능** : A보드 목표가 정수 펄스라 1펄스 = 0.884 m/s(3.18 km/h)다.
  `max_speed_ms=4.42`(현 기본, 5펄스)에서 사용 가능한 단계가 1~5 다. 상세는
  `white/kasa_units.py` 헤더 참고.
- **★게인표 고속행이 실차 미검증이다★** `max_speed_ms` 를 5펄스로 올리면서
  `GAIN_TABLE`/`LFD_TABLE` 에 2.2 m/s 초과 행을 신설했다. LFD 는 ω_n≈0.97 유지 설계로
  연장(표의 원래 기준을 연장)했고, **게인은 일부러 올리지 않았다**(2.2 행 값 유지).
  **3펄스 → 4펄스 → 5펄스로 단계적으로 올리며 로스백을 확인할 것.**
- **측정 공백** : 펄스 필드는 '직전 20ms 창'의 카운트인데 보고는 50ms 마다다 →
  50ms 중 30ms 는 계측되지 않는다. `gps_imu` 의 DR 거리적분이 그만큼 거칠다.
  근본 해결은 A보드 텔레메트리에 **누적 펄스 카운터** 필드를 추가하는 것인데,
  펌웨어 무수정 방침이라 적용하지 않았다.
- **저속 양자화** : 20ms 창에서 1~2펄스면 ±1펄스가 값의 50~100%다.
  좌+우 합을 쓰면 눈금이 절반(0.442 m/s)이 되지만 원리적 한계는 남는다.
- **`usb_cam_ctrl` 은 respawn 대상이 아니다** — 카메라가 respawn 되면 v4l2-ctl 설정이
  다시 적용되지 않는다. 노출이 이상해지면 `one_launch.py` 안의 그 명령을 손으로 한 번 돌린다.
- **E-stop 확인시간이 100ms 다**(`kasa_0904_A/B.ino [0904-1]`, 발동·해제 대칭).
  4.42 m/s 에서 약 0.44m 진행 후 제동된다(0804 의 500ms 시절에는 약 2.2m 였다).
  **사람이 발로 밟는 것이 1차 수단인 전제**는 그대로다.
  접점 채터링은 수 ms 단위라 100ms 로도 걸러지지만, 배선 노이즈로 오발동이 보이면
  `ESTOP_TRIGGER_CONFIRM_MS` 를 올리기 전에 **배선을 먼저** 확인할 것.
- **D12 를 실제로 보는 보드는 B 하나다**(A보드 `ESTOP_ENABLED = false`).
  `update_estop` 은 A·B 의 OR 판정을 그대로 두었다 — A보드를 다시 켜면 곧바로 맞는다.
- **조이스틱 GUI 없음** — kasa_ws 원본의 tkinter 화면(스틱 위치·A0 다이얼)을 이식하지
  않았다. 필요하면 `kasa_ws/src/nxde/nxde/joystick.py` 의 `JoystickGui` 를 참고해 붙일 수 있다.
