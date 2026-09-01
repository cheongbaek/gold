# mppi_local_planner

금색차 kasa 용 MPPI 로컬 장애물 회피. `catkin_ws/src/mppi_local_planner`(1/5카용)를
이식했다. 전역경로·nav2 없이 라이다(OS1-32)와 그 IMU 만으로 직진하다가, 장애물을
원본과 같은 S커브로 감아 피하고 출발 헤딩(IMU 기준선)으로 돌아온다.

> **먼저 읽을 것** — `include/mppi_local_planner/kasa_units.hpp`.
> 1/5카 `/cmd_vel_raw` 는 m/s + 도로휠각(+ = 좌) 이었다. 금색차는 **펄스 정수 /
> pot 지령 / −좌 +우 / `/brake_level`**. 그대로 꽂으면 빌드도 되고 토픽도 붙는데
> 차만 다르게 움직인다.

---

## 1. 작동

```
ros2 launch mppi_local_planner one_launch.py
```

**"주행 시작"을 누르는 절차가 없다.** 자이로 바이어스 보정(차 정지, ouster IMU
~100 샘플 ≈ 1초)이 끝나면 **2펄스 ≈ 6.4 km/h** 로 스스로 직진한다.

세우는 수단은 Ctrl-C · E-STOP · D5 수동조종 셋이다. 피할 길이 없으면 원본과
같이 속도 0 을 내고, 금색차에서는 거기에 **리니어 2단**을 더한다(펄스 0 은
코스트일 뿐이라 그 속도에서도 정지거리가 길다).

| | 1/5카 원본 | 금색차 (이 패키지) |
|---|---|---|
| 휠베이스 | 0.75 m | **1.25 m** |
| 윤거 | 0.65 m | **1.10 m** (white 실측. lidar yaml 0.65 는 1/5 잔재) |
| 조향 상한 | 0.40 rad (~23°) | **0.553 rad (도로휠 31.7°)** |
| `/cmd_vel_raw.linear.x` | m/s | **목표펄스 0~15** |
| `/cmd_vel_raw.angular.z` | 도로휠각 deg, +좌 | **pot 지령, −좌 / +우** |
| 순항 | 3params.yaml 0.83 m/s | **2펄스 = 1.768 m/s ≈ 6.4 km/h** |
| 라이다 높이 | 0.80 m AGL | **1.17 m AGL** |
| xy 반전 | `sensor_yaw_offset=π` | 동일 (`flip_lidar_xy: true`) |
| 제동 | 속도 0 | **속도 0 + `/brake_level` 2단** (회피 실패 시) |
| 게이트 | 없음 | **`/vehicle_mode` + `/estop`** |

플래너 본체(`mppi_controller.cpp` 의 샘플·S커브·lookahead·softmax)는 원본을
그대로 둔다.

---

## 2. 실행

```bash
source /opt/ros/humble/setup.bash
source ~/gold/gold_ws/install/setup.bash

colcon build --packages-select mppi_local_planner \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
# ★ --symlink-install 을 쓰지 말 것 (C++ 패키지)
```

```bash
# 실차 — 차 앞을 비우고, D5 자율, E-STOP 해제, 런치 직후 정지
ros2 launch mppi_local_planner one_launch.py

# RViz 로 코스트맵·롤아웃
ros2 launch mppi_local_planner one_launch.py use_rviz:=true

# 책상 (게이트 끄고 지령만 — 차에 연결하지 말 것)
ros2 launch mppi_local_planner one_launch.py drive:=false use_ouster:=false use_arduino:=false
```

라이다가 안 붙으면 먼저:

```bash
ip -br addr show eno1     # UP + 192.168.6.100/24
ping -c2 192.168.6.11
```

앞뒤 방향: 차 앞 3 m 에 사람이 서서 코스트맵 전방에 점이 생기면 `flip_lidar_xy`
가 맞다. 뒤에만 잡히면:

```bash
ros2 launch mppi_local_planner one_launch.py flip_lidar_xy:=false
```

---

## 3. 토픽

| 방향 | 토픽 | 내용 |
|---|---|---|
| 구독 | `/ouster/points` | OS1-32 포인트클라우드 |
| 구독 | `/imu` | 외장 iAHRS 쿼터니언 헤딩 (드리프트↓). `/ouster/imu` 는 자이로만 |
| 구독 | `/vehicle_mode` · `/estop` | D5 · E-STOP 게이트 |
| 발행 | `/cmd_vel_raw` | 펄스 + pot 지령 (KasaActuator) |
| 발행 | `/control_state` | 구동 허용 |
| 발행 | `/brake_level` | 회피 실패 시 2단 |
| 발행 | `/mppi_local_planner/costmap` | ego 코스트맵 |
| 발행 | `/mppi_local_planner/local_path` | 고른 롤아웃 |
| 발행 | `/mppi_local_planner/reference_path` | IMU 기준선 |

---

## 4. 튜닝

감지 기하·순항은 `config/params.yaml` 한 곳이다(재빌드 불필요, 노드 재시작만).

- `mppi.desired_speed` / `kasa.max_pulse` — 기본 2펄스 ≈ 6.4 km/h.
  1펄스로 내리려면 `desired_speed:=0.884 cruise_pulse:=1`.
  정지 재출발에서 4펄스는 피한다.
- `flip_lidar_xy` — 장착 방향.
- `sensor_height_m` · `roi_agl_*` — z 슬랩. 콘이 안 보이면 AGL 창을 넓힌다.
- `costmap.ego_clear_*` — 너무 크면 가까운 장애물을 지운다. 금색차 캐빈이
  ~1.2 m 이라 1/5카의 0.45 m 를 쓰면 자기 차체에 멈춘다.
- `mppi.num_samples` / `horizon_steps` — CPU.
- `mppi.stop_cost_threshold` — 낮추면 더 일찍 멈춘다.

원본 설계 메모(타임스텝, lookahead, 풋프린트)는 catkin_ws 쪽 README 와 같다.
