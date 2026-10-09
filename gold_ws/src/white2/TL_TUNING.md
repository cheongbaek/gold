# 신호등 파라미터 튜닝 가이드 (white2·white1 `traffic_light.py`)

> 2026-10-08 기준. 알고리즘 설명은 [TRAFFIC_LIGHT.md](TRAFFIC_LIGHT.md). 기본값은 코드(`declare_parameter`)가 정본이다 —
> 이 표와 다르면 코드를 믿고 이 표를 고친다. 두 패키지 파일은 같은 줄이라 **기본값을 바꾸면 둘 다** 바꾼다.

## 0. 먼저 지킬 것

1. **모든 파라미터는 노드가 뜰 때 한 번만 읽는다** — 주행 중 `ros2 param set` 은 효과가 없다. 바꾸면 노드(런치)를 다시 띄운다.
2. **바꾸는 방법은 셋**
   - 런치 인자로 열린 것: `ros2 launch white2 one_launch.py tl_conf:=0.4` — 열린 것은 `tl_device` `tl_conf` `tl_red_stop_min_height`
     `tl_near_release_ratio` `sl_enable` `sl_trigger_bev_y` `tl_solo_stop_min_height` `sl_conf` `sl_gate_red_s` `tl_show_window`
     `tl_record_video` `tl_window_width` `tl_stop_latch` `tl_publish_cmd_vel` `tl_hsv_red_h2_low` (+ 녹화 `tl_video_*`, 카메라 기하는 `camera_model`).
   - 그 밖: 코드 기본값을 고치고 `colcon build --packages-select white1 white2`(★`--symlink-install` 금지★ — 다시 빌드해야 반영).
   - 시험: 테스트베드 `--param traffic_light.<이름>=<값>` (코드를 안 고치고 대조군을 만든다).
3. **바꾸기 전에 근거, 바꾼 뒤에 검증** — 측정 분포(최소·최대)와 여유로 값을 정하고, `cam-test` 스킬로 회귀를 돌려 디버그 영상을 남긴다(7절).
   임계값을 감으로 정하지 않는다.
4. **틀리는 방향을 고른다** — '일찍 문다·늦게 푼다·RED 를 버리지 않는다' 쪽이 안전하다(CLAUDE.md 7절 원칙 7).

## 1. 무엇을 바꾸고 싶은가 → 손댈 파라미터

| 증상 | 먼저 볼 것 | 방향 |
|---|---|---|
| 먼 신호를 늦게 본다(늦게 선다) | `tl_zone_red_min_height` · `tl_conf` | 문턱을 내리면 일찍 보지만 먼 오검출에도 문다 |
| 엉뚱한 빨강에 선다 | `hsv_min_color_pixels` · `tl_max_height_frac` · `tl_roi_*` | 관문을 올리거나 ROI 를 좁힌다 |
| 정지선 너무 앞/뒤에 선다 | `sl_trigger_bev_y` · `tl_solo_stop_min_height` | 2절 '정지 위치' |
| 서 있다 풀렸다 다시 문다(떨림) | `red_release_hold_s` · `red_release_hold_lost_s` · `tl_near_release_ratio` | 해제 유예를 늘린다 |
| 초록인데 늦게 출발한다 | `green_hold_s` · `red_release_hold_s` · 로그 `⏳ 해제 보류` | 유예를 줄이면 떨림 위험 |
| 수풀·사람을 초록으로 본다 | `tl_green_max_y2` | 2절 |
| #400 화살표를 못 읽거나 잘못 읽는다 | `tl_b_arrow_frac` · `tl_b_arrow_min_h` · 코드 상수 `ARROW` | 2절 'B' |
| 'A'(본선 4번 좌회전) 화살표를 못 읽거나 잘못 읽는다 | `tl_a_arrow_min_px` · `tl_a_arrow_min_h` · `/tl/boxes` 10번째 칸(녹색 px) | 2절 'A' |
| 인지가 느리다(FPS) | `tl_imgsz`(엔진과 짝) · `sl_gate_red_s` · 디버그 패널 | 3절 |

## 2. 파라미터 표

열: **기본(런치)** · 뜻 · **올리면 / 내리면** · 근거 · 같이 볼 것. ⚠️ = 바꾸면 다른 값의 뜻이 바뀐다.

### 인지 — 모델·ROI

| 파라미터 | 기본 | 뜻 | 올리면 / 내리면 | 근거·주의 |
|---|---|---|---|---|
| `tl_weights` | 1280 엔진 | 신호등 엔진 | — | 엔진은 GPU·드라이버에 묶인다 — 기계마다 `tools/export_tl_1280.sh`. 없으면 640 엔진으로 내려가고 원거리 검출률 68→36% |
| `tl_imgsz` | 1280 | 입력 크기 | — | ⚠️ 정적 엔진과 짝 — 다르면 엔진 값으로 맞춰진다 |
| `tl_conf` | 0.35 (런치 0.35) | 박스 문턱 | 올리면 오검출↓ 먼 신호 놓침↑ / 내리면 반대 | 실측 conf 대부분 0.83~0.88. 0.35~0.5 는 드물다 |
| `tl_roi_xmin/ymin/xmax/ymax` | 640/0/1280/560 | 탐색 영역(보정 원본 px) | 넓히면 옆 신호·차량 위 가짜 빨강에 선다(10-05 E2·E3) | ⚠️ 640×560 이라 1280 에서 정확히 ×2. 크기를 바꾸면 배율이 바뀌어 **모든 px 문턱의 뜻이 바뀐다** |
| `tl_min_area` | 20 | 박스 최소 면적 px² | | |
| `tl_max_height_frac` | 0.5 | ROI 높이 대비 박스 높이 상한 | | 야간 ROI 를 덮는 가짜 박스(85~99%) 방지. 0 = 끔 |
| `tl_min_aspect` / `tl_max_aspect` | 0.2 / 6.0 | 가로세로비 범위 | | 실측 #400 차량 1.79~2.41 · 버스 1.47~1.80 · 수풀 2.87~3.75(그래서 비율로는 못 거른다) |
| `tl_green_max_y2` | 520 | GREEN 아랫변 상한(보정 영상 y) | 내리면 진짜 먼 초록을 버릴 수 있다 / 올리면 수풀이 다시 들어온다 | 진짜 신호 y2 ≤ 503, 오검출 ≥ 539. 0 = 끔. ⚠️ 카메라 틸트·장착 높이를 바꾸면 다시 잰다 |

### 인지 — HSV 색 교정

| 파라미터 | 기본 | 뜻 | 근거·주의 |
|---|---|---|---|
| `hsv_min_color_pixels` | 15 | RED 채택에 필요한 붉은 화소 수 / HSV 색 판정 최소 화소 | 올리면 붉지 않은 RED 오검출↓, 작은 진짜 빨강을 버릴 위험↑ |
| `hsv_crop_center_ratio` | 0.85 | 박스 가운데 몇 %만 볼까 | |
| `hsv_red_h1_low/high` · `hsv_red_h2_low/high` | 0~10 · 160~180 | 빨강 색상 범위 | 램프 실측 적색 hue 160~169 까지 나온다 |
| `hsv_green_h_low/high` | 45~90 | 초록 색상 범위 | ⚠️ 흐린 날 가까운 녹색등은 청록(100~120)으로 찍혀 여기서 0 — 초록 관문을 이것으로 만들지 말 것 |
| `hsv_sat_low` / `hsv_val_low` | 55 / 60 | 채도·밝기 하한 | 램프 실측 최저 S62·V66 |
| (코드 상수) `HSV_FALLBACK_CONF` | 0.55 | 이보다 낮은 박스만 HSV 색으로 바꾼다 | 구 white 에서 이어받은 값, 실측 근거 없음. 올리면 RED→GREEN 이 쉬워진다(위험 방향) |

### 판단 — 근접·확정

| 파라미터 | 기본(런치) | 뜻 | 올리면 / 내리면 | 근거·주의 |
|---|---|---|---|---|
| `tl_zone_red_min_height` | 9 | 신호 구간 안 RED 근접 문턱 px | 올리면 늦게 확정 / 내리면 먼 오검출 확정 | 본선 정지선 앞 맞은편 신호 9~12px. 0 = 구간 무시 |
| `tl_red_stop_min_height` | 25 (25) | 구간 밖 근접 문턱 | | white2 주행 중에는 구간 안이라 거의 안 쓰인다(수동 체크박스·기록용) |
| `tl_near_release_ratio` | 0.7 (0.7) | 서 있는 동안 근접 문턱 배율 | 내리면 떨림↓ 하지만 **남의 작은 빨강이 해제를 막을** 위험↑ | 9→6.3px. 본선 T1 에서 T2 신호 5~6px — 0.6 이하로 내리면 그것이 해제를 막기 시작한다 |
| `tl_hold_s` | 0.1 | RED 확정 시간 | 올리면 늦게 문다 | |
| `tl_gap_grace_s` | 0.15 | 이 이하 끊김은 확정 유지 | 내리면 먼 신호 확정이 매번 리셋 | 0.05 일 때 제동 0.8 s 늦었다(10-06) |
| `tl_arm_hold_s` | 1.0 | 확정 뒤 대기를 잇는 시간 | | 0 = 끔 |
| `tl_state_max_age` | 3.0 | 판정이 이보다 낡으면 개입 안 함 | | fail-open |
| `tl_zone_stale_s` | 1.0 | `/tl_zone` 신선도 | | driving 은 20 Hz 로 낸다 |

### 판단 — 정지 위치

| 파라미터 | 기본(런치) | 뜻 | 올리면 / 내리면 | 근거·주의 |
|---|---|---|---|---|
| `sl_trigger_bev_y` | 40 (40) | 정지선 발화 BEV 행 | 올리면 더 가까이 가서 선다 / 내리면 일찍 | 40 ≈ 3.6 m. 접근 17회 중 정지선 앞 12 · 단독 4 · 안 섬 1. ⚠️ BEV 사다리꼴(`bev_src_pts`)을 바꾸면 다시 잡는다 |
| `sl_lost_min_bev_y` | 40 | '정지선 놓침' 은 이 행까지 온 선만 | | ⚠️ 발화선과 같아 지금은 도달하지 않는다. 발화선을 올리면 **이것을 발화선보다 작게** 둬야 안전망이 산다. 음수 = 문턱 없음 |
| `tl_solo_stop_min_height` | 25 (25) | 신호등 단독 정지 px | 내리면 정지선 한참 앞에서 선다 / 올리면 정지선을 못 볼 때 늦게 | K-City 25px 첫 도달 때 정지선 3.1~3.9 m 앞. 0 = 끔(실차 금지). ⚠️ 근접 문턱보다 작으면 '확정 = 즉시 정지' |
| `sl_conf` · `sl_imgsz` | 0.30 · 640 | 정지선 모델 | | |
| `sl_hold_s` · `sl_stale_s` | 0.2 · 0.5 | 정지선 확정·놓침 | | |
| `sl_min_width_frac` · `sl_min_area_frac` | 0.12 · 0.0004 | 정지선 최소 폭·면적 | | |
| `sl_lane_half_m` | 0.1 | 차 중심 ±m 를 덮어야 정지선 | | 0.5 는 중앙선에 붙을 때 진짜 선을 버렸다 |
| `sl_gate_red_s` | 1.0 (1.0) | 빨강을 이 안에 봤을 때만 정지선 추론 | 내리면 FPS↑, 늦게 찾는다 | 성능 |

### 판단 — 해제

| 파라미터 | 기본(런치) | 뜻 | 올리면 / 내리면 | 근거·주의 |
|---|---|---|---|---|
| `red_release_hold_s` | 0.7 | 초록을 봤을 때 RED 미감지 유예 | 내리면 빨리 출발, 떨림↑ | 0.55 s 공백에 풀렸다 물린 실측 |
| `red_release_hold_lost_s` | 1.5 | 초록도 못 봤을 때 유예 | 내리면 떨림 재발 | RED 끊김 최장 0.42 s · k-city 1.0 s 끊김 실측. `red_release_hold_s` 이하 = 끔 |
| `green_hold_s` | 0.4 | 초록 확정 시간 | | 해제와 `stop_latch` 에 쓴다 |
| `stop_latch` | false (false) | true 면 초록 확정까지 문다 | | 정지선에 바짝 붙어 신호가 화면을 벗어나는 코스 |
| `tl_hold_log_ratio` | 0.5 | `⏳ 해제 보류` 로그 기준 | | 로그만. 0 = 끔 |

### 구간 'A' (좌회전 — 본선 4번 #1100) [2026-10-09]

| 파라미터 | 기본 | 뜻 | 근거·주의 |
|---|---|---|---|
| `tl_a_arrow_min_px` | 3 | 빨강 박스 안 녹색 픽셀이 이 이상이면 화살표 켜짐(진행, 크기 무관) | 순수 빨강 약 570박스 전부 0px · 빨강+화살표 ≥10px 에서 94~95% 가 ≥3px. 올리면 화살표를 놓친다(정지 쪽), 내리면 잡음에 진행(위험 쪽) |
| `tl_a_arrow_min_h` | 10 | 이보다 작은 빨강 박스는 녹색이 안 보여도 모델 GREEN 이면 화살표로 본다(종전 규칙) | 8~9px 는 화살표가 46% 만 보인다. ★판단을 미루는 문턱으로 쓰지 말 것★ — #1100 등은 정지선 앞에서도 9~10px 다 |

### 구간 'B' (#400)

| 파라미터 | 기본 | 뜻 | 근거·주의 |
|---|---|---|---|
| `tl_bus_detect` | true | 끄면 'B' 가 'T' 와 같다 | |
| `tl_b_need_arrow` | true | false = 버스만 빼고 원등에도 간다(직진 경로) | 본선 #400 은 좌회전 — true |
| `tl_b_arrow_min_h` | 20 | 이보다 작은 차량 박스는 화살표를 안 읽는다(= 정지) | 20px 밑은 원등 번짐이 셋째 칸에 0.60 까지 |
| `tl_b_arrow_frac` | 0.30 | 셋째 칸 초록 비율 문턱 | 켜짐 최소 0.42 / 꺼짐 최대 0.14 |
| `tl_b_veh_min_ar` | 1.75 | 화살표를 읽을 최소 가로세로비 | 버스 박스 1.47~1.80 을 거른다 |
| `tl_bus_min_veh_h` · `tl_bus_dark_t` · `tl_bus_open_k` · `tl_bus_iso_below` · `tl_bus_iso_left` | 10 · 0.5 · 0.40 · 0.12 · 0.20 | 버스 몸체 영상처리 | **표시용** — 판단에 안 쓴다 |
| (코드 상수) `ARROW` | x 0.50~0.66 · y 0.15~0.85 · HSV 45~100/45/40 | 판독 창 | 노드 HSV(S55·V60)로는 가까운 화살표(V60~90)를 놓쳐 따로 둔다 |

### 정지 동작·표시

| 파라미터 | 기본(런치) | 뜻 |
|---|---|---|
| `brake_level` | 2 | 무는 단계 |
| `publish_cmd_vel` | true (런치 false) | 2단 동안 `/cmd_vel_raw` 0/0 도 낸다 |
| `require_permission` | true | false = 벤치 모드(항상 허락) — 실차 금지 |
| `stop_cmd_hz` | 30 | 판단 틱 |
| `show_window` · `tl_publish_debug` | true(런치 false) · false(녹화 켜면 true) | 창 · 디버그 토픽 |
| `window_width` · `show_bev` · `tl_roi_zoom` · `roi_dim` · `draw_roi` · `hud_font` | 960 · true · true · 0.6 · true · '' | 디버그 화면(1728×590). 끄면 960×590 |

## 3. 같이 움직여야 하는 짝

| 바꾸는 것 | 같이 볼 것 |
|---|---|
| ROI 크기·위치 | 확대 배율 → 박스 높이 → `tl_zone_red_min_height`·`tl_solo_stop_min_height`·`tl_b_arrow_min_h`·`tl_green_max_y2`(y 기준) |
| 카메라(틸트·높이·보정 `cam_undistort`) | 위 px 문턱 전부 + `tl_green_max_y2` + BEV 사다리꼴 → `sl_trigger_bev_y` |
| `sl_trigger_bev_y` | `sl_lost_min_bev_y`(발화선보다 작아야 안전망이 산다) |
| `tl_near_release_ratio` | T1 의 T2 신호 5~6px — 문턱이 6px 밑으로 내려가면 해제를 막는다(`⏳ 해제 보류` 로그로 확인) |
| `red_release_hold_s` / `_lost_s` | 140401 은 2.8 fps — lockstep 시험에서 1.5 s 가 영상 10 s 로 늘어난다(실시간으로 확인) |
| `tl_weights` / `tl_imgsz` | 엔진을 다시 만들면 conf 분포도 다시 잰다 |
| 기본값 | white1·white2 두 파일 같은 줄 — 짝 diff 13줄 확인 |

## 4. 튜닝 순서 (권장)

1. **증상을 녹화로 재현** — 해당 장면 영상과 구간을 정하고, 지금 값으로 `cam-test` 한 번(기준 런).
2. **측정** — `signals.csv` 의 `tl_boxes`(박스 높이·y·conf·색)·`tl_state`·`brake_level` 로 문제 박스의 분포를 잰다. 디버그 영상에서 그 장면을 눈으로 본다.
3. **값을 정한다** — 진짜/가짜 분포 사이 여유 가운데로. 틀리는 방향이 안전한 쪽인지 확인.
4. **대조군** — `--param traffic_light.<이름>=<값>` 으로 같은 구간을 다시 돌린다(코드는 안 고친다).
5. **회귀** — 아래 전부를 돌려 기준 런과 `tl_state` 프레임 차이를 센다. 바뀐 프레임은 하나하나 디버그 영상으로 본다.
6. **적용** — 사용자 확인 → 기본값 수정(두 파일) → 빌드 → 같은 회귀 → CHANGELOG(근거·숫자·되돌리는 법) → `main`.

## 5. 회귀 세트 (이 기계)

```bash
source /opt/ros/humble/setup.bash; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; cd ~/cam_testbed
C=contracts/gold_white2_stopline.yaml; OUT=/home/mad1/gold/gold_ws
python3 -m tb.run doctor --contract $C --video /home/mad1/tl_eval_videos/cam-20260913_144031.mp4
# q(예선) · v1f · v2f(본선) 전체 — GPS 구간·허락
python3 -m tb.run run --contract $C --domain 91 --video /home/mad1/tl_eval_videos/cam-20260913_140401.mp4 --preset ~/tl_eval/zone_q.yaml   --start 0 --limit 0 --name X-q   --note "…" --out $OUT
python3 -m tb.run run --contract $C --domain 91 --video /home/mad1/tl_eval_videos/cam-20260912_105807.mp4 --preset ~/tl_eval/zone_v1f.yaml --start 0 --limit 0 --name X-v1f --note "…" --out $OUT
python3 -m tb.run run --contract $C --domain 91 --video /home/mad1/tl_eval_videos/cam-20260913_144031.mp4 --preset ~/tl_eval/zone_v2f.yaml --start 0 --limit 0 --name X-v2f --note "…" --out $OUT
# #400 'B'
python3 -m tb.run run --contract $C --domain 91 --video /home/mad1/tl_eval_videos/cam-20260912_105807.mp4 --preset ~/tl_eval/zone400B.yaml --start 2689 --limit 400 --name XB-v1f --note "…" --out $OUT
python3 -m tb.run run --contract $C --domain 91 --video /home/mad1/tl_eval_videos/cam-20260913_144031.mp4 --preset ~/tl_eval/zone400B.yaml --start 2545 --limit 395 --name XB-v2f --note "…" --out $OUT
# k-city 본선 (GPS 구간 — T1 T2 B A T5)
python3 -m tb.run run --preset ~/tl_eval/zone_kcity.yaml --domain 91 --start 1400 --limit 3900 --name X-kcity --note "…" --out $OUT
# 'A' 일부러 씌운 시험 (실제 구간 아님) — 화살표 판독
python3 -m tb.run run --contract $C --domain 91 --video /home/mad1/tl_eval_videos/cam-20260913_140401.mp4 --preset ~/tl_eval/zone_q_aforce.yaml --name XQA-q --note "…" --out $OUT
python3 -m tb.run run --contract $C --domain 91 --video /home/mad1/tl_eval_videos/cam-20260913_144031.mp4 --preset ~/tl_eval/zone_v2f_aold.yaml --start 3150 --limit 500 --name XAOLD-v2f --note "…" --out $OUT
# 채점
python3 ~/tl_eval/ev.py X q,v1f,v2f ; python3 ~/tl_eval/b400/s2_eval.py XB
```

★[2026-10-09] 구간 프리셋 `zone_{q,v1f,v2f,kcity}.yaml` 은 GPS 기준이다★ — 같은 주행의 record 위치를 경로 CSV terrain 에 대응시켰다
(도구 `~/tl_eval/zone_gps/`, 시각 오차 ±0.5 s). **허락도 구간 안에서만** 준다(`/tl_enable` 끄고 `/tl_permit` = 구간) — 실차 white2 와 같다.
옛 손 구간은 `~/tl_eval/_backup/zone_hand_1009/`. 정답표 `ev.py` 는 미션 신호 기준(S 자리 뺌, #400 넣음 — `docs/kcity`).

**현재 기준값** [2026-10-09] — 105807 5/5 · 144031 1/1 · 140401 2/2 (모두 오정지 0) · #400 진행 오판 0 · 화살표 미판독 0 · 첫 GREEN 144031 2833 ·
k-city 정지 T2 2217 · B 3354 · A(순수 빨강) 3993 · T5 5061 · 144031 실제 A 구간 GREEN 167/200(RED 1프레임) ·
`zone_q_aforce` 순수 빨강 251·302·358 정지, 화살표 켜짐(310) 뒤 324 해제. 판단 타이머 흔들림으로 제동 전이 ±1~3프레임은 정상.
