# mppi_local_planner

금색차 kasa 용 로컬 장애물 회피. **경로는 블록 계획기의 Frenet `d(s)`, 조향은 그
곡선의 순수추종** 이다 (`avoid.geometric_steer`). MPPI 는 진단 비용만 낸다.

> ### ★[2026-09-28] 블록 계획기로 다시 짰다★ — 근거는 `src/frenet_planner.cpp` 머리 상자
>
> 1. 라이다 **점유 셀**을 0.3 m 로 묶어 콘을 찾고, 사이로 못 지나갈 만큼 가까운 콘
>    (진행 2.5 m · 가로 1.9 m)은 **블록** 하나로 본다. 기억은 GPS 기준선 (s,d) 에 둔다.
> 2. 블록마다 **궤적 기준 치우친 쪽의 반대**로 통과한다(사용자 규칙). 가운데면 이웃
>    블록과의 상대 위치 → 앞 블록의 반대. 앞범퍼 5 m 안에서 확정한다.
> 3. 뒤차축이 블록 **앞면 0.3 m 앞**에서 통과 d 에 닿아 뒷면 0.3 m 뒤까지 지킨다.
>    구간은 5차 다항식(최소 저크)이고 남은 거리를 전부 쓴다. 폭은 **±3.0 m**.
> 4. **인계 전 미리보기** — 다음 L 구간이 25 m 안이면 조종권 없이 계획만 돌려 쪽을
>    미리 정하고, 인계 순간 이어받는다. 이때도 **아무것도 발행하지 않는다**.
> 5. 정지는 **그 경로를 따라갔을 때** 차 중심선이 치사 원반에 드는가(3.0 m 예측).
>    추종·정지 코드는 `include/mppi_local_planner/path_tracker.hpp` 하나다.
>
> 모의(`test/avoid_sim.cpp`, 23장면 — 9/28 실주행 배치 재구성 포함)와 ROS 폐루프(`test/ros_smoke.py`) 결과는
> `white1/CHANGELOG.md` 2026-09-28 항.

> **먼저 읽을 것** — `include/mppi_local_planner/kasa_units.hpp`.
> 1/5카 `/cmd_vel_raw` 는 m/s + 도로휠각(+ = 좌) 이었다. 금색차는 **펄스 정수 /
> pot 지령 / −좌 +우 / `/brake_level`**. 그대로 꽂으면 빌드도 되고 토픽도 붙는데
> 차만 다르게 움직인다.

---

## 1. 작동

> ### ★[2026-09-01] 기본 용도가 바뀌었다 — white1 과 한 계통이다★
>
> 이 노드는 이제 **혼자 쓰이지 않는다.** `white1 one_launch` 가 함께 띄우고,
> 매핑 CSV 의 `terrain` 열이 `'L'` 인 구간에서만 조종권을 받아 몬다. 그 밖의
> 구간에서는 **아무것도 발행하지 않는다**(침묵).
>
> ```
> white1/driving ──/lstatus(String '0'|'L'|'S')──▶ 나   "'L' 이면 네가 몰아라"
> white1/driving ◀──/lidar_active(Bool)── 나      "나 살아 있다" (매 틱)
> ```
>
> · 실차 실행은 **`ros2 launch white1 one_launch.py`** 다 (이 패키지 런치가 아니다).
> · 설계 근거는 `white1/white1/driving.py` 헤더의 '라이다 구간 이양' 절과
>   `white1/CHANGELOG.md` 2026-09-01 항에 있다.
> · **`handover.require_lstatus: false` 로 두면 아래의 종전 '런치 = 출발' 그대로다**
>   — 라이다 단독 시험용이다. white1 과 함께 띄울 때 이 값이 false 면 이 노드가
>   GPS 추종 구간에서도 `/cmd_vel_raw` 를 내며 driving.py 와 20Hz 로 서로를 덮는다.
> · 허락의 **상승엣지마다 기준선을 다시 잡는다**(`rearmReference`). 기준 방위가
>   런치 시점(주차 위치)에 한 번만 잡히면 L 구간에서 엉뚱한 선으로 복귀한다 —
>   CHANGELOG ③ 항이 그 사고를 막은 기록이다.

---

## 1-1. 단독 실행 (라이다만 시험할 때)

```
ros2 launch mppi_local_planner one_launch.py
```

**"주행 시작"을 누르는 절차가 없다.** 자이로 바이어스 보정(차 정지, ouster IMU
~100 샘플 ≈ 1초)이 끝나면 **2펄스 ≈ 6.4 km/h** 로 스스로 직진한다.

> ⚠️ 이 런치는 `handover.require_lstatus` 를 건드리지 않는다 — `config/params.yaml`
> 의 기본이 **true** 이므로 그냥 띄우면 **허락을 기다리며 침묵한다.** 단독으로
> 굴리려면 명시해야 한다:
> ```bash
> ros2 launch mppi_local_planner one_launch.py \
>     mppi_params_file:=/dev/null   # 또는
> ros2 run mppi_local_planner mppi_local_planner_node --ros-args \
>     -p handover.require_lstatus:=false
> ```

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

플래너 본체는 세 층이다.

- `cone_detector.cpp` — 점유 셀 군집 (차체 기준 상자)
- `frenet_planner.cpp` — 블록 기억 · 쪽 결정 · 5차 다항식 `d(s)` (조향을 내지 않는다)
- `path_tracker.hpp` — 순수추종 + 편향 적분 + 경로 추종 정지 예측
- `mppi_controller.cpp` — 샘플·softmax. `geometric_steer` 가 켜져 있으면 조향은 쓰지 않고, 정지 판단도 하지 않는다

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
| 구독 | `/lstatus` | ★조종권★ (white1 driving, String `'0'`/`'L'`/`'S'`). `'L'` 이 아니거나 낡으면 **침묵** |
| 구독 | `/lidar_ref` | GPS 기준선 `[cte, heading_err, valid, zone_left]` — 미리보기는 `zone_left` 로 켠다 |
| 구독 | `/encoder` | 좌+우 펄스 합 (저속 킥·추종 속도) |
| 발행 | `/lidar_active` | ★생존 신고★ 매 틱. driving 은 값이 아니라 **신선도**를 본다 |
| 발행 | `/cmd_vel_raw` | 펄스 + pot 지령 (KasaActuator) |
| 발행 | `/control_state` | 구동 허용 |
| 발행 | `/brake_level` | 회피 실패 시 2단 |
| 발행 | `/mppi_local_planner/costmap` | ego 코스트맵 |
| 발행 | `/mppi_local_planner/local_path` | 고른 롤아웃 |
| 발행 | `/mppi_local_planner/reference_path` | 계획 경로 `d(s)` (차체 기준) |
| 발행 | `/lidar_diag` | 14열 진단 — white1/record 의 `ld_*` (몰 때만) |
| 발행 | `/lidar_cones` | 회피 대상 군집 `[x, y, cells]…` — `.cones.csv` (몰 때만) |

---

## 4. 튜닝

감지 기하·순항·회피는 `config/params.yaml` 한 곳이다. ★[2026-10-28 정정] 고친 뒤 `colcon build --packages-select
mppi_local_planner` 를 다시 해야 한다★ — 런치는 `install/share/…/params.yaml` 사본을 넘기고 복사설치라 소스를 고쳐도
사본이 안 바뀐다(실행 중에는 `ros2 param set /mppi_local_planner_node <이름> <값>` 으로 바로 바뀐다).
★`include/.../frenet_planner.hpp` 의 `FrenetPlannerParams` 기본값은 노드에 쓰이지 않는다★ — 노드가 모든 필드를
`avoid.*`·`frenet.*` 파라미터로 덮는다(`mppi_local_planner_node.cpp` `frenetDoubleParams`). 정본은 yaml, hpp 기본값은
`test/avoid_sim.cpp` 기본 모드에서만 쓰인다(`YAML=1` 이면 모의도 yaml 값).

- `cruise_pulse` — 기본 2펄스 ≈ 6.4 km/h. 정지 재출발에서 4펄스는 피한다.
- `avoid.max_offset_m` — 회피 폭 (GPS 궤적 ± **2.00** m, [2026-10-26] 3.0 에서). ★`mppi.max_lateral_offset`·
  `mppi.lateral_hard` 는 이보다 바깥에 둔다★ (같으면 계획과 벽이 같은 자리에서 싸운다).
- `avoid.pass_gap_m` — 차체 옆면 ↔ 콘 표면 여유 (**0.38**, [2026-10-26] 0.75 에서 · [2026-09-29] 0.45 → 0.75). 키우면 교대가 빡빡해진다.
- `handover.gps_antenna_x_m` — GPS 안테나가 라이다 원점보다 앞선 거리 (**1.25** — 안테나는 앞차축 위, 라이다는 뒷차축 위 = 축거. [2026-09-29] 콘 회귀값 0.6 → [2026-09-30] 1.25). CTE 를 라이다 원점으로 되돌린다.
- `frenet.hold_pre_m` / `hold_post_m` — 블록 앞뒤로 통과 d 를 지키는 거리 (**0.2**, [2026-10-26] 0.3 에서).
- `frenet.kappa_max` — 이보다 굽는 진입이면 확정 전 블록의 쪽을 반대로 검토 (0.34 ≈ 조향 상한).
- `avoid.lookahead_m` — 순수추종 앞 점 (**2.4**, [2026-10-26] 3.2 에서). 줄이면 여유가 늘고 헤딩이 커진다.
- `preview.enable` / `preview.zone_dist_m` — 인계 전 미리보기.
- `flip_lidar_xy` — 장착 방향.
- `sensor_height_m` · `roi_agl_*` — z 슬랩. 콘이 안 보이면 AGL 창을 넓힌다.
- `costmap.ego_clear_*` — 너무 크면 가까운 장애물을 지운다. 금색차 캐빈이
  ~1.2 m 이라 1/5카의 0.45 m 를 쓰면 자기 차체에 멈춘다.
- `mppi.num_samples` / `horizon_steps` — CPU.

### ★[2026-10-28] 장애물 (s,d) 상자는 점유 셀로 만든다★

`frenet_planner.cpp` `observe()` 는 군집(`cone_detector.cpp` `clusterOccupancy`)의 ★셀마다★ 기준선 (s,d) 로 옮겨
상자를 잡는다(셀 반 칸 0.05 m 를 더해 셀 바깥 모서리까지). 종전에는 차체 좌표 상자 네 모서리만 옮겼는데, 차가
ψ 만큼 기울어 있으면 길쭉한 물체의 상자가 (차체 x 길이) × sin ψ 만큼 차 쪽으로 부풀었다.

- 실주행 : white2 `route_20261028_210148` 4주행(21:17 · 21:22 · 21:28 · 21:37) — 둘째 장애물(경로 s 57.2, d −1.25 ~ −0.25)을
  왼쪽으로 비키려 ψ +15~23° 로 꺾자 왼쪽 도로가 물체(실제 d ≥ +2.73, 정체 미확인)의 상자 안쪽 끝이 +2.86(ψ 4°) →
  −0.18(ψ 14.6°) → −2.46(ψ 22.8°) 로 내려와 둘째 장애물과 한 블록 d [−1.84, +7.65] 가 됐다. '오른쪽 −2.00' 은 폭
  ±2.0 밖(필요 −2.76)이라 경로가 장애물을 뚫어 `stop latch: track` — 실제 통로는 3.0 m(필요 1.84 m)였다.
- 모의 (`test/avoid_sim.cpp`, `YAML=1`, 장면 `r1028` — 그 배치 재구성, 10 시드) : 정지 8/10 → **0/10**, 둘째 장애물
  왼쪽 통과 max|d| 0.86 이하, 장애물 여유 최소 **0.36 m**. 회귀 23장면 충돌 0 · full_block 정지 유지.
- 대가 : 종전 상자 부풂이 우연히 주던 여유(ψ 에 비례, 0.05~0.13 m)가 사라져 회귀 장면 여유가 조금 준다 —
  시드 1 최대 −0.12 m(`today_yawerr` 0.56 → 0.44, `YAML=1`), 10 시드 평균 −0.01~−0.09 m. **사용자가 받아들였다.**
  `test/ros_smoke.py` 0.53 → 0.46 m.
- 비용 : 셀 약 2,500 개 좌표 변환 / 틱.
- ⚠️ 실차 미검증 — mppi 로그 `🚧 블록` 의 도로가 물체 d 안쪽 끝이 ψ 와 무관하게 유지되는가, 둘째 블록이 '왼쪽으로
  통과' 로 남는가, track 정지 0회인가를 본다.
- ⚠️ 남은 것 : 한 번 잘못 합쳐진 블록은 기억에서 거의 줄지 않아(kShrinkA 0.15, 코앞에서는 안 줄임) track 정지가
  스스로 안 풀린다(10/28 30~120 s, 수동으로 내림).

원본 설계 메모(타임스텝, lookahead, 풋프린트)는 catkin_ws 쪽 README 와 같다.
