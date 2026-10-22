# 신호등 파라미터 튜닝 가이드 (white1·white2 공용 `traffic_light.py`)

> [2026-10-22] 현재 코드(`d91f232` — 역광 감마까지)의 `declare_parameter`·`tl_params.py`·`camera_launch.py` 런치 인자와 다시 맞췄다.
> 알고리즘 설명은 [TRAFFIC_LIGHT.md](TRAFFIC_LIGHT.md), 근거 숫자·되돌리는 법은 white2 `CHANGELOG.md` 각 항목.
>
> - ★**코드는 한 벌이다**★ [2026-10-21 `f675db6`] — `gold_ws/src/white1/white1/traffic_light.py` · `tl_params.py` · `camera_launch.py`.
>   white2 런치도 이 노드를 `package='white1'` 로 띄운다. 이 문서만 white2 폴더에 있다(두 스택 공통 설명서).
> - **기본값의 정본** — 런치 인자로 열린 것은 `tl_params.py`(노드 선언과 런치 인자가 같은 dict 를 읽는다), 나머지는 `traffic_light.py` 의 `declare_parameter`.
>   이 표와 다르면 코드를 믿고 이 표를 고친다.
> - **스택마다 쓰이는 몫이 다르다** — white2 는 이 노드가 `/tl_permit` 구간(T·A·B)에서 직접 2단을 문다(정지·해제 파라미터 전부가 효력).
>   white1 은 `/tl_permit` 이 늘 False 라 이 노드는 색(`/tl/state`)만 내고, 서고 푸는 것은 driving(`LightFilter` · 타이머)이 한다 —
>   white1 에 효력이 있는 것은 **인지·색 판정 쪽**(ROI·HSV·`tl_zone_min_box_h_t`·녹색 HSV 확인·감마 등)이다.

## 0. 먼저 지킬 것

1. **모든 파라미터는 노드가 뜰 때 한 번만 읽는다** — 주행 중 `ros2 param set` 은 효과가 없다. 바꾸면 노드(런치)를 다시 띄운다.
2. **바꾸는 방법은 셋**
   - **런치 인자로 열린 것** — `ros2 launch white2 one_launch.py tl_conf:=0.4`(white1 도 같은 인자 — `ros2 launch white1 one_launch.py …`).
     - 기본 : `tl_device` `tl_conf` `tl_red_stop_min_height` `tl_near_release_ratio` `tl_solo_stop_min_height` `tl_hsv_red_h2_low`
       `sl_enable` `sl_trigger_bev_y` `sl_conf` `sl_gate_red_s` `tl_stop_latch` `tl_publish_cmd_vel`
       `tl_show_window` `tl_record_video` `tl_window_width` (+ 녹화 `tl_video_*`, 카메라 기하는 `camera_model`)
     - `tl_params.py` 의 dict 다섯 개 — **키 이름 그대로 런치 인자다** :
       `B_ARROW`('B' 화살표·진행 근거 16개) · `ZONE_GATE`(구간별 근접 문턱·배율 6개) · `TL_LOCK`(T 박스 버리기·잠금 4개) ·
       `GREEN_HSV`(녹색 HSV 확인 2개) · `GAMMA`(역광 감마 6개).
       예 : `tl_b_go_mode:=arrow` · `tl_zone_min_box_h_t:=0 tl_lock_enable:=false` · `tl_gamma:=0` · `tl_green_hsv_confirm:=false`.
   - **그 밖** — 코드 기본값(`declare_parameter` 또는 `tl_params.py`)을 고치고 `colcon build --packages-select white1`
     (★`--symlink-install` 금지★ — 복사설치라 다시 빌드해야 반영. white2 런치도 white1 설치본을 쓴다).
     ★노드나 런치에 숫자를 따로 적지 말 것★ — 런치 인자로 여는 값은 `tl_params.py` 한 곳에만 둔다(CLAUDE.md 7절).
   - **시험** — 테스트베드 `--param traffic_light.<이름>=<값>` (코드를 안 고치고 대조군을 만든다).
3. **바꾸기 전에 근거, 바꾼 뒤에 검증** — 측정 분포(최소·최대)와 여유로 값을 정하고, `cam-test` 스킬로 회귀를 돌려 디버그 영상을 남긴다
   (TRAFFIC_LIGHT.md 7절). 임계값을 감으로 정하지 않는다.
   **시간 문턱(`*_s`)을 바꿨으면 반드시 `--mode realtime` 으로 본다** — 노드는 벽시계 초로 재고, 기본 lockstep 은 프레임을 빨리 밀어
   영상 시각으로는 정지·해제가 늦어 보인다(2.8 fps 예선 영상에서 해제가 13프레임 ≈ 5 s 늦게 보였다). 잠금 추적 거리(`gap_grow`)도
   끊긴 시간으로 넓히므로 lockstep 에서 다르게 나온다(101543 : lockstep 은 잠금을 잃고 realtime 은 붙는다).
4. **틀리는 방향을 고른다** — '일찍 문다·늦게 푼다·RED 를 버리지 않는다' 쪽이 안전하다(CLAUDE.md 7절 원칙 7).
   [2026-10-20] 부터 구간 T·A 는 '잠금을 잃으면 계속 정지' 다 — 안 풀리는 쪽으로 틀린다(미션은 실패할 수 있다).

## 1. 무엇을 바꾸고 싶은가 → 손댈 파라미터

| 증상 | 먼저 볼 것 | 방향 |
|---|---|---|
| 먼 신호를 늦게 본다(늦게 선다) | `tl_zone_red_min_height_{t,a,b}` · `tl_conf` · T 는 `tl_zone_min_box_h_t` | 문턱을 내리면 일찍 보지만 먼 오검출에도 문다. T 는 16px 밑 박스가 아예 없다 — white1 서브 코스 T 의 첫 빨강이 2.1~2.4 s 늦다(실측, 그대로 두기로 결정) |
| 엉뚱한 빨강에 선다 | `hsv_min_color_pixels` · `tl_max_height_frac` · `tl_roi_*` | 관문을 올리거나 ROI 를 좁힌다 |
| 정지선 너무 앞/뒤에 선다 | `sl_trigger_bev_y` · `tl_solo_stop_min_height` | 2절 '정지 위치' |
| 서 있다 풀렸다 다시 문다(떨림) | `red_release_hold_s` · `red_release_hold_lost_s` · `tl_near_release_ratio_{t,a,b}`(구간 밖은 `tl_near_release_ratio`) · `tl_img_stale_s` | 해제 유예를 늘린다. 영상 끊김 뒤 재체결 왕복이면 `tl_img_stale_s` |
| 초록인데 늦게 출발한다 | `green_hold_s` · `red_release_hold_s` · 로그 `⏳ 해제 보류` · HUD `잠금 …` | 유예를 줄이면 떨림·거짓 녹색 해제 위험(2절 `green_hold_s`) |
| 초록인데 **다음 교차로·건너편 빨강 때문에** 출발 못 한다 | `tl_zone_min_box_h_t`(T) · `tl_lock_enable` | T 는 16px 밑 박스를 만들지 않고, T·A 는 잠근 내 신호만 본다 — 둘은 짝 |
| 서 있는데 안 풀린다 — HUD `잠금 Npx 잃음` | `tl_lock_track_k` · `LOCK_GEOM['gap_grow']` · `tl_gamma*` | 추적 거리를 넓히면 남의 박스에 붙을 위험. 박스가 아예 없으면(역광) 감마 쪽 |
| 어두운 빨강·황색을 **초록으로 봐 빨간불에 풀린다** | `tl_green_hsv_confirm` · `tl_green_hsv_min_px` · `green_hold_s` | 2절 '구간 T·A' |
| 역광으로 **서 있는데 박스가 0** 이다 | `tl_gamma` · `tl_gamma_on_s` · `tl_gamma_green_px` | 2절 '구간 T·A' |
| 수풀·사람을 초록으로 본다 | `tl_green_max_y2` | 2절 |
| #400 화살표를 못 읽거나 잘못 읽는다 | `tl_b_arrow_frac` · `tl_b_arrow_min_h` · 창·HSV `tl_b_arrow_{x0,x1,y0,y1,h_lo,h_hi,s_lo,v_lo}` | 2절 'B' |
| #400 버스 빨강 + 원등 녹색인데 안 간다 / 황혼에 잘못 간다 | `tl_b_go_mode` · `tl_b_bus_red_s` · `tl_b_round_min` · `tl_b_amber_max` | 2절 'B' — 거짓 진행이 싫으면 `bus` |
| #400 신호등이 화면 위로 잘리며 다시 문다 | `tl_b_clip_hold` | 잘린 차량 박스는 판독하지 않고 직전 판독을 유지 |
| 'A'(본선 4번 좌회전) 화살표를 못 읽거나 잘못 읽는다 | `tl_a_arrow_min_px` · `tl_a_arrow_min_h` · `/tl/boxes` 10번째 칸(녹색 px) | 2절 'A' |
| 인지가 느리다(FPS) | `tl_imgsz`(엔진과 짝) · `sl_gate_red_s` · 디버그 화면 · `TL_CV_THREADS` | 3절. 감마는 서 있을 때만 YOLO 1회(≈9.5 ms)를 더한다 |

## 2. 파라미터 표

열: **기본(런치)** · 뜻 · **올리면 / 내리면** · 근거 · 같이 볼 것. ⚠️ = 바꾸면 다른 값의 뜻이 바뀐다. '(런치 같음)' = `tl_params.py` 값이 런치 인자 기본값이다.

### 인지 — 입력·모델·ROI

| 파라미터 | 기본 | 뜻 | 올리면 / 내리면 | 근거·주의 |
|---|---|---|---|---|
| `image_topic` · `device` | `/image_raw` · `cuda:0` (런치 `tl_device`) | 입력 영상 · 추론 장치 | | GPU 가 없으면 `cpu`(느리다) |
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
| `hsv_min_color_pixels` | 15 | RED 채택에 필요한 붉은 화소 수 / HSV 색 판정 최소 화소 | 올리면 붉지 않은 RED 오검출↓, 작은 진짜 빨강을 버릴 위험↑. 감마 박스의 RED 판정에도 같은 값 |
| `hsv_crop_center_ratio` | 0.85 | 박스 가운데 몇 %만 볼까 | 녹색 HSV 확인도 같은 잘라내기를 쓴다 |
| `hsv_red_h1_low/high` · `hsv_red_h2_low/high` | 0~10 · 160~180 (런치 `tl_hsv_red_h2_low`) | 빨강 색상 범위 | 램프 실측 적색 hue 160~169 까지 나온다 |
| `hsv_green_h_low/high` | 45~90 | 초록 색상 범위 | ⚠️ 흐린 날 가까운 녹색등은 청록(100~120)으로 찍혀 여기서 0 — 초록 관문을 이것으로 만들지 말 것. ⚠️ **'A' 화살표 판독과 감마 박스 GREEN 판정이 이 범위로 센 녹색 화소를 쓴다** — 바꾸면 그 회귀도 돌린다. 구간 T·A 녹색 HSV 확인은 이것과 **별개 범위**(45~100) |
| `hsv_sat_low` / `hsv_val_low` | 55 / 60 | 채도·밝기 하한 | 램프 실측 최저 S62·V66. 빨강·초록 모두에 걸린다 — 'A' 판독·감마 GREEN 에도 영향. `hsv_sat_low` 는 녹색 HSV 확인의 S 하한으로도 쓰인다(V 하한은 그쪽 전용 35) |

### 구간 T·A — 박스 버리기 · 내 신호 잠금 · 녹색 확인 · 역광 감마 [2026-10-20 · 10-21]

네 기능 모두 **구간 T·A 에서만** 듣는다(박스 버리기는 T 만). B 는 차량 박스 하나로 판단하므로 대상이 아니고, 구간 밖은 종전 그대로다.
기본값 = `tl_params.TL_LOCK` · `GREEN_HSV` · `GAMMA`(런치 같음).

| 파라미터 | 기본 | 뜻 | 올리면 / 내리면 · 끄기 | 근거·주의 |
|---|---|---|---|---|
| `tl_zone_min_box_h_t` | 16 | 구간 T 에서 이 높이 미만 YOLO 박스는 **만들지 않는다**(빨강·초록 모두 — 판단·근접·정지선 추론·그림·`/tl/boxes` 어디에도 없다) | 0 = 끔. 올리면 체결이 늦어지고 내리면 다음 교차로가 다시 들어온다 | T 체결 순간 내 빨강 20~27px · 정지 중 28~47px / 다음 교차로 4~7px · 건너편 등기구 8~12px. 켜 두면 T 의 `tl_zone_red_min_height_t`·`tl_near_release_ratio_t`(12·0.75)는 판단을 바꾸지 않는다. ⚠️ white1 도 같은 `/tl/state` 를 쓰므로 white1 서브 코스 T 의 첫 빨강이 2.1~2.4 s(≈6 m) 늦다 — 사용자 결정으로 그대로. ⚠️ **잠금과 짝** — 이것만 켜면 내 신호가 화면 위로 빠진 정지에서 건너편 11px 이 사라져 빨간불에 풀린다(140401 288) |
| `tl_lock_enable` | true | 2단을 무는 순간 정지 근거가 된 근접 RED 박스(가장 큰 것) 하나를 잠그고 위치로 따라간다. 잠금이 있으면 해제 = **그 박스의** 초록 `green_hold_s` 확정 + 그 박스 빨강 `red_release_hold_s` 미감지. 남의 빨강·초록은 무시. **잃으면 계속 정지**('빨강 1.5 s 미감지' 해제를 쓰지 않는다) | false = 종전 해제 규칙 | 지우는 때 : 다음 체결 · 허락 상실 · 구간 변경. HUD 구간 칸 `잠금 35px 녹 0.2s` / `잃음` / `풀림`, 로그 `🔒 내 신호 잠금` · `🔒 잠금 다시 붙음` |
| `tl_lock_track_k` | 1.2 | 이어지는 박스 = 중심 거리 ≤ k × 잠금 높이(그리고 높이비 `LOCK_GEOM['h_ratio']` 0.7~1.8) | 올리면 남의 박스에 붙을 위험 / 내리면 굴러가며 흔들릴 때 잃는다 | 못 본 시간만큼 넓힌다(`gap_grow` 3 — 1 s 끊기면 k·h × 4). 101543 : 1 s 끊긴 뒤 3.8·h 옮겨 나타났다. 2.8 fps 영상처럼 한 장에 크게 움직이면 잃는다(그래도 정지 유지) |
| `tl_lock_lost_s` | 1.0 | 이만큼 못 보면 '잃음' 표시 | | HUD·로그용 — 해제는 하지 않는다. `gap_grow` 의 시간 기준이기도 하다 |
| `tl_green_hsv_confirm` | true | 구간 T·A 에서 모델 GREEN 박스는 이 규칙 전용 HSV(H 45~100 · S≥`hsv_sat_low` · V≥35 — `tl_params.GREEN_HSV_H`·`GREEN_HSV_V_LO`, 중앙 85 %·3×3 열림)의 녹색 화소가 있어야 남는다. **GREEN 을 버리기만 한다** | false = 종전 | 094820 근거리 정지 — 어두운 빨강(V≤58)·황색을 모델이 GREEN 으로 내 빨간불에 세 번 풀렸다(7277·7585·7906). V≥60 이면 그 정지의 진짜 녹색(V 39~50)도 0px 라 영영 안 풀린다 |
| `tl_green_hsv_min_px` | 3.0 | 위 녹색 화소 문턱 | 올리면 A 의 9~12px 진짜 녹색(최소 8px)이 빠진다 / 내리면 거짓 GREEN 이 남는다 | 오프라인 442박스 : 거짓 191 중 ≥3px 2(홀로) · 진짜 251 중 <3px 4 · 16px 이상 진짜는 0 이 아니면 최소 32px. `tl_a_arrow_min_px` 와 같은 자 |
| `tl_gamma` | 1.8 | 역광 감마 지수 — 서 있는데 판단 박스 0 이면 **모델 입력만** 감마로 한 번 더 추론, 원본 박스와 겹치지 않는(IoU < `GAMMA_IOU` 0.3) 감마 박스를 더한다 | 0 = 끔(종전과 판정 같음). 올리면 하우징이 더 검어진다 — 1.4 는 창1 녹색만 83 %·빨강 1 %(오프라인) | 094820 #300 역광 정지 — 원본 박스 0 으로 녹색 창 둘을 놓쳤다(실차는 우산으로 가린 뒤 출발). 1.8 로 녹색 시작 뒤 0.43~0.92 s 해제 |
| `tl_gamma_on_s` | 0.5 | '서 있는데 박스 0' 이 이만큼 이어지면 켠다 [s] | 내리면 잠깐 끊긴 것에도 켠다 | '서 있다' = 이 노드의 2단(white2) **또는** 남이 낸 `/brake_level` 2 가 `GAMMA_EXT_BRAKE_S` 1.0 s 안(white1 driving 의 신호 정지). 정지가 끝나거나 구간을 벗어날 때까지 켜 둔다 |
| `tl_gamma_green_px` | 5 | 감마 박스 GREEN 의 녹색 화소 하한 — **노드 녹색 범위(H 45~90·V≥60)로 원본에서 센다** | 내리면 빨강 현시에 GREEN 위험 | 진짜 녹 9px 이상 · 적색 현시 0px. 감마 박스의 모델 클래스는 학습 분포 밖이라 믿지 않는다 — 색은 원본 HSV 로만. ⚠️ 녹색 확인 범위(45~100·V≥35)로 바꾸지 말 것 — 감마 박스가 하늘을 품어 빨강에도 수백 px(시제품이 빨간불에 풀렸다) |
| `tl_gamma_max_h` · `tl_gamma_min_ar` | 80 · 2.0 | 감마 박스 높이 상한 · 가로/세로 하한 | min_ar 를 올리면 진짜 박스를 자른다 | 등기구 31~45px · 4구 등기구 감마 박스 1589개 1 백분위 2.21(최소 1.81) · 우산·해 플레어 203~260px · 가로세로 0.59~0.76 |
| `tl_gamma_lock_k` | 4 | 잠금이 있으면 잠근 중심에서 k × `tl_lock_track_k` × 잠금 높이 안의 감마 박스만 | | 엉뚱한 물체를 막는 마지막 줄 |

### 판단 — 근접·확정

| 파라미터 | 기본(런치) | 뜻 | 올리면 / 내리면 | 근거·주의 |
|---|---|---|---|---|
| `tl_zone_red_min_height_t` · `_a` · `_b` | 12 · 9 · 9 (런치 같음) | 신호 구간 T·A·B 안 RED 근접 문턱 px — 이상이면 RED(정지 대상), 밑이면 RED_FAR(관측만) | 올리면 늦게 확정 / 내리면 먼 오검출·다음 교차로 빨강을 RED 로 | T : 체결 순간 내 빨강 20~27px(13건) · 다음 교차로 빨강 4~7px. ★`tl_zone_min_box_h_t` 16 이 켜져 있으면 T 값은 판단을 바꾸지 않는다★. **A(#1100)는 정지선 앞에서도 9~10px — 올리면 'A' 순수 빨강에 안 선다**(전 구간 12 로 시험하면 v1f 5/5 → 4/5). 0 이하 = 그 구간도 구간 밖 25px |
| `tl_red_stop_min_height` | 25 (25) | 구간 밖 근접 문턱 | | white2 주행 중에는 구간 안이라 거의 안 쓰인다(수동 체크박스·기록용) |
| `tl_near_release_ratio_t` · `_a` · `_b` | 0.75 · 0.7 · 0.7 (런치 같음) | 신호 구간 안에서 서 있는 동안 근접 문턱 배율 | 내리면 **남의 작은 빨강이 해제를 막을** 위험↑ / 올리면 **내 신호가 화면 위로 빠진 정지에서 건너편 등기구를 놓쳐 빨간불에 풀릴** 위험↑ | T : 12×0.75 = 9px — 다음 교차로 빨강 4~7px ↔ 건너편 등기구 8~12px 사이(여유 2px · −1px). 잠금이 켜져 있으면 해제는 잠근 박스만 보므로 이 배율의 몫이 작다 |
| `tl_near_release_ratio` | 0.7 (0.7) | ★구간 밖 전용★ 서 있는 동안 근접 문턱 배율 (25 → 17.5px) | | |
| `tl_hold_s` | 0.1 | RED 확정 시간 | 올리면 늦게 문다 | |
| `tl_gap_grace_s` | 0.15 | 이 이하 끊김은 확정 유지 | 내리면 먼 신호 확정이 매번 리셋 | 0.05 일 때 제동 0.8 s 늦었다(10-06) |
| `tl_arm_hold_s` | 1.0 | 확정 뒤 대기를 잇는 시간 | | 0 = 끔 |
| `tl_state_max_age` | 3.0 | 판정이 이보다 낡으면 개입 안 함 | | fail-open |
| `tl_img_stale_s` | 1.5 | 영상이 이만큼 끊기면 RED 를 새로 확정하지 않는다 | 0 = 끔 | 끊긴 뒤 '빨강 미감지' 해제 → 남은 RED 로 재체결 30 Hz 왕복 방지(cam-test 영상 끝 46회 → 0) |
| `tl_zone_stale_s` | 1.0 | `/tl_zone` 신선도 | | driving 은 20 Hz 로 낸다 |

### 판단 — 정지 위치 (white2)

| 파라미터 | 기본(런치) | 뜻 | 올리면 / 내리면 | 근거·주의 |
|---|---|---|---|---|
| `sl_enable` · `sl_weights` | true (런치 같음) · 차선 seg 엔진 | 정지선 인지 | false = 신호등 단독 정지만 | |
| `sl_trigger_bev_y` | 40 (40) | 정지선 발화 BEV 행 | 올리면 더 가까이 가서 선다 / 내리면 일찍 | 40 ≈ 3.6 m. 접근 17회 중 정지선 앞 12 · 단독 4 · 안 섬 1. ⚠️ BEV 사다리꼴(`bev_src_pts`)을 바꾸면 다시 잡는다 |
| `sl_lost_min_bev_y` | 40 | '정지선 놓침' 은 이 행까지 온 선만 | | ⚠️ 발화선과 같아 지금은 도달하지 않는다. 발화선을 올리면 **이것을 발화선보다 작게** 둬야 안전망이 산다. 음수 = 문턱 없음 |
| `tl_solo_stop_min_height` | 25 (25) | 신호등 단독 정지 px | 내리면 정지선 한참 앞에서 선다 / 올리면 정지선을 못 볼 때 늦게 | K-City 25px 첫 도달 때 정지선 3.1~3.9 m 앞. 0 = 끔(실차 금지). ⚠️ 근접 문턱보다 작으면 '확정 = 즉시 정지' |
| `sl_conf` · `sl_imgsz` | 0.30 (런치 같음) · 640 | 정지선 모델 | | |
| `sl_hold_s` · `sl_stale_s` | 0.2 · 0.5 | 정지선 확정·놓침 | | |
| `sl_min_width_frac` · `sl_min_area_frac` | 0.12 · 0.0004 | 정지선 최소 폭·면적 | | |
| `sl_lane_half_m` | 0.1 | 차 중심 ±m 를 덮어야 정지선 | | 0.5 는 중앙선에 붙을 때 진짜 선을 버렸다 |
| `sl_gate_red_s` | 1.0 (1.0) | 빨강을 이 안에 봤을 때만 정지선 추론 | 내리면 FPS↑, 늦게 찾는다 | 성능 |

### 판단 — 해제 (white2)

잠금(`tl_lock_enable`)이 있는 동안은 아래 두 유예와 `green_hold_s` 를 **잠근 박스 하나에만** 적용한다(위 '구간 T·A' 표).

| 파라미터 | 기본(런치) | 뜻 | 올리면 / 내리면 | 근거·주의 |
|---|---|---|---|---|
| `red_release_hold_s` | 0.7 | 초록을 봤을 때 RED 미감지 유예 | 내리면 빨리 출발, 떨림↑ | 0.55 s 공백에 풀렸다 물린 실측. 잠금 해제에 남긴 이유 — 역광에서 내 빨강이 GREEN 으로 0.23 s 이어진 뒤 다시 빨강(094820 7111~7126) |
| `red_release_hold_lost_s` | 1.5 | 초록도 못 봤을 때 유예 | 내리면 떨림 재발 | RED 끊김 최장 0.42 s · k-city 1.0 s 끊김 실측. `red_release_hold_s` 이하 = 끔. ★잠금이 있으면 쓰지 않는다★(잃으면 계속 정지) |
| `green_hold_s` | 0.3 [2026-10-18 사용자 결정] | 초록 확정 시간 | 내리면 빨리 출발 / 올리면 거짓 녹색에 강해진다 | 해제와 `stop_latch` 에 쓴다. ⚠️ 조사(2026-10-20, 값은 그대로 — 사용자 결정) : 역광 094820 7105~7117 에서 내 빨강이 GREEN 으로 틈 유예 포함 0.49 s 이어졌다 — 0.5 도 7119 해제, 0.6 은 거짓 해제 0. 그 장면은 녹색 HSV 확인이 막는다 |
| `stop_latch` | false (`tl_stop_latch` false) | true 면 초록 확정까지 문다 | | 정지선에 바짝 붙어 신호가 화면을 벗어나는 코스 |
| `tl_hold_log_ratio` | 0.5 | `⏳ 해제 보류` 로그 기준 | | 로그만. 0 = 끔 |

### 구간 'A' (좌회전 — 본선 4번 #1100)

| 파라미터 | 기본 | 뜻 | 근거·주의 |
|---|---|---|---|
| `tl_a_arrow_min_px` | 3 | 빨강 박스 안 녹색 픽셀이 이 이상이면 화살표 켜짐(진행, 크기 무관) | 순수 빨강 약 570박스 전부 0px · 빨강+화살표 ≥10px 에서 94~95% 가 ≥3px. 올리면 화살표를 놓친다(정지 쪽), 내리면 잡음에 진행(위험 쪽) |
| `tl_a_arrow_min_h` | 10 | 이보다 작은 빨강 박스는 녹색이 안 보여도 모델 GREEN 이면 화살표로 본다 | 8~9px 는 화살표가 46% 만 보인다. ★판단을 미루는 문턱으로 쓰지 말 것★ — #1100 등은 정지선 앞에서도 9~10px 다. 올리면 모델이 GREEN 으로 낸 순수 빨강(예선 #100 같은 것)에 갈 위험이 커진다 |

- 판독 숫자는 디버그 화면에 없다 — `/tl/boxes` 10번째 칸(테스트베드 `signals.csv` 의 `tl_boxes`)에서 박스별 녹색 화소를 본다.
- 녹색 화소는 HSV 표의 `hsv_green_h_*`·`hsv_sat_low`·`hsv_val_low`·`hsv_crop_center_ratio` 로 센다 — 그 값들을 바꾸면 이 표의 근거(0px / ≥3px)를 다시 잰다.
- 'A' 도 구간 T·A 표의 잠금·녹색 확인·감마를 받는다(박스 버리기는 T 만 — #1100 은 9~10px 라 안 된다).

### 구간 'B' (#400) — 기본값 `tl_params.B_ARROW`(런치 같음)

| 파라미터 | 기본 | 뜻 | 근거·주의 |
|---|---|---|---|
| `tl_bus_detect` | true | 끄면 'B' 가 'T' 와 같다(버스 찾기·화살표 둘 다 꺼짐) | 노드 선언(런치 인자 아님) |
| `tl_b_need_arrow` | true | false = 버스만 빼고 원등에도 간다(직진 경로) | 본선 #400 은 좌회전 — true. 노드 선언 |
| `tl_b_arrow_min_h` | 20 | 이보다 작은 차량 박스는 화살표를 안 읽는다(= 정지) | 20px 밑은 원등 번짐이 셋째 칸에 0.60 까지 |
| `tl_b_arrow_frac` | 0.30 | 셋째 칸 초록 비율 문턱 | 켜짐 최소 0.42 / 꺼짐 최대 0.14. 정지선 앞 39~40px 근거리에서는 켜진 화살표가 0.10~0.17 로 읽힌 일이 있다(141104 — 버스 근거가 받는다) |
| `tl_b_veh_min_ar` | 1.75 | 화살표를 읽을 최소 가로세로비 | 버스 박스 1.47~1.80 을 거른다 |
| `tl_b_clip_hold` | true | 차량 박스가 ROI·화면 가장자리(`CLIP_MARGIN_PX` 1px)에 닿으면 판독하지 않고 직전(잘리지 않은) 판독 유지 | 잘린 박스는 셋째 칸이 어긋나 켜진 화살표를 0.27~0.30 으로 읽었다(10/17 17:29 재제동 2회) |
| `tl_b_arrow_x0` · `x1` · `y0` · `y1` | 0.50 · 0.66 · 0.15 · 0.85 | 화살표 판독 창(차량 박스 비율) — 디버그 화면 타일의 주황 사각형 | 셋째 칸. 0.66 넘기면 원등 번짐이 들어온다 |
| `tl_b_arrow_h_lo` · `h_hi` · `s_lo` · `v_lo` | 45 · 100 · 45 · 40 | 화살표 녹색 HSV(OpenCV 0~180·0~255). 같은 녹색으로 원등 칸도 잰다(`tl_b_round_min`) | 노드 HSV(S55·V60)로는 가까운 화살표(V60~90)를 놓쳐 따로 둔다 |
| `tl_b_go_mode` | either | 진행 근거 — `arrow` 화살표만(10/18 까지) · `bus` 버스 빨강 확정+차량 녹색만 · `either` 둘 중 하나 · `and` 둘 다 | 현시표(`docs/kcity/260331_신호현시정보.pdf` 2쪽) : 버스 전용신호는 1현시(0~32 s)에만 표기 → '버스 빨강 + 차량 원등(SG1) 녹색' = 2현시(좌회전 SG2 35~47 s)뿐. ★'버스 빨강만' 은 쓰지 않는다★ — 3·4현시(차량 빨강)에도 버스는 빨강(녹화 4곳 · 오프라인 오진행 384프레임). `either` 는 황혼 화살표 오판(172742 1802~1840, 버스 황색 중)을 그대로 남긴다 — 싫으면 `bus`. 대회 날 현시가 바뀌면 `arrow` |
| `tl_b_bus_red_s` | 0.3 | 버스 RED 를 이만큼(그리고 `BUS_GO['min_red_n']` 3장 이상) 봐야 '버스 빨강' 기억 | 버스 빨강이 화살표보다 0.15~0.23 s 먼저 켜진다 · 버스 황색을 빨강으로 읽은 것은 전환 경계 1프레임(143장 중). 차량이 녹색이 아니게 보이거나 버스 GREEN 이면 지운다 |
| `tl_b_round_min` · `tl_b_amber_max` | 0.10 · 0.20 | 차량 녹색 = 라벨 GREEN AND 넷째 칸 녹색 ≥ round_min AND 둘째 칸 황색 < amber_max | 녹색 현시 원등 칸 ≥0.10 776/778 · 황색 칸 최대 0.09. 8px 밑·잘린 박스는 라벨만(`BUS_GO['cell_min_h']`). ⚠️ 차량 황색(47~50 s) 녹화가 없어 황색 쪽은 미검증 |
| `tl_bus_min_veh_h` · `tl_bus_dark_t` · `tl_bus_open_k` · `tl_bus_iso_below` · `tl_bus_iso_left` | 10 · 0.5 · 0.40 · 0.12 · 0.20 | 버스 몸체 영상처리(노드 선언, 나머지 기하는 코드 상수 `BUS_BODY`) | 버스 박스를 판단에서 빼는 데는 안 쓴다. 그 몸체의 색(HSV)은 `tl_b_go_mode` 의 '버스 빨강' 근거로 쓴다(모델이 버스를 못 잡을 때 — 실측 버스 박스의 ≈70% 가 이쪽) |

### 정지 동작·표시·녹화

| 파라미터 | 기본(런치) | 뜻 |
|---|---|---|
| `brake_level` | 2 | 무는 단계 (white2) |
| `publish_cmd_vel` | true (런치 `tl_publish_cmd_vel` false) | 2단 동안 `/cmd_vel_raw` 0/0 도 낸다 |
| `require_permission` | true | false = 벤치 모드(항상 허락) — 실차 금지 |
| `stop_cmd_hz` | 30 | 판단 틱 |
| `show_window` · `tl_publish_debug` · `tl_record` | true(런치 `tl_show_window` false) · false(런치 true — 구독자가 있을 때만 발행) · false(런치 = `tl_record_video` true) | 창 · 디버그 토픽 · 직접 mp4 녹화(nxde/recorder) |
| `tl_video_dir` · `tl_video_scale` · `tl_video_segment_s` | ''(= `paths.video_dir()`) · 1.0 · 120 (런치 `tl_video_*`) | 녹화 폴더 · 축소 · 조각 길이 |
| `window_width` · `show_bev` · `tl_roi_zoom` · `roi_dim` · `draw_roi` · `hud_font` | 960(런치 `tl_window_width`) · true · true · 0.6 · true · '' | 디버그 화면(1728×590). `show_bev`·`tl_roi_zoom` 을 끄면 960×590 |

### 코드 상수 — 런치 인자도 파라미터도 아니다 (바꾸면 빌드)

| 상수 | 값 | 어디 | 뜻·주의 |
|---|---|---|---|
| `HSV_FALLBACK_CONF` | 0.55 | `traffic_light.py` | 이보다 낮은 박스만 HSV 색으로 바꾼다. 구 white 에서 이어받은 값, 실측 근거 없음. 올리면 RED→GREEN 이 쉬워진다(위험 방향) |
| `BUS_BODY` | 어두움 0.5 · 열림 0.40 · 자리 dx −4.75h · dy −0.45h … | `traffic_light.py` | 버스 몸체 영상처리 기하. 일부(dark_t·open_k·iso_*)는 위 `tl_bus_*` 파라미터가 덮는다 |
| `CLIP_MARGIN_PX` | 1 | `traffic_light.py` | `tl_b_clip_hold` 의 잘림 판정 여유 — 잘린 박스는 y1 = 0, 잘리기 직전 y1 = 2~9 |
| `BUS_GO` | 높이비 0.6~1.5 · dy/h −1.0~0.4 · RED 3장 · 칸 검사 8px 이상 · 칸 y 0.15~0.85 · 황색 HSV (11,55,60)~(35,255,255) | `tl_params.py` | 버스 빨강을 근거로 쓸 자리·크기·장수. 실측 0.73~1.25 · −0.71~0.08(1137 프레임) |
| `LOCK_GEOM` | 높이비 0.7~1.8 · `top_px` 3 · `gap_grow` 3.0 | `tl_params.py` | 잠금 추적 — 이어지는 박스의 높이비 · 윗변으로 빠짐 판정 · 못 본 시간만큼 넓히는 배수 |
| `GREEN_HSV_H` · `GREEN_HSV_V_LO` | 45~100 · 35 | `tl_params.py` | 녹색 HSV 확인 **전용** 범위. 노드 `hsv_green_h_high`(90)·`hsv_val_low`(60)와 묶지 말 것 — 묶으면 교정·RED 관문·A 화살표가 함께 바뀐다 |
| `GAMMA_IOU` · `GAMMA_EXT_BRAKE_S` | 0.3 · 1.0 s | `tl_params.py` | 감마 박스가 원본과 이만큼 겹치면 버림 · 남이 낸 `/brake_level` 2 의 신선도(driving 은 0.25 s 마다 다시 낸다) |
| `TL_CV_THREADS` | 8 | `traffic_light.py` | ★`from ultralytics import YOLO` 뒤에서★ OpenCV 스레드를 되돌린다 — 앞에 두면 1스레드로 어안 보정 9.7 ms |
| `TL_ENABLE_STALE_S` · `BRAKE_KEEPALIVE_S` | 2.0 · 0.25 | `traffic_light.py` | master '신호등 인지' 체크 신선도 · 물고 있는 동안 브레이크 재주장 주기(master KEEPALIVE 0.5 보다 짧게) |

## 3. 같이 움직여야 하는 짝

| 바꾸는 것 | 같이 볼 것 |
|---|---|
| ROI 크기·위치 | 확대 배율 → 박스 높이 → `tl_zone_min_box_h_t`·`tl_zone_red_min_height_{t,a,b}`·`tl_solo_stop_min_height`·`tl_b_arrow_min_h`·`tl_gamma_max_h`·`tl_green_max_y2`(y 기준) |
| 카메라(틸트·높이·보정 `cam_undistort`) | 위 px 문턱 전부 + `tl_green_max_y2` + BEV 사다리꼴 → `sl_trigger_bev_y` |
| `sl_trigger_bev_y` | `sl_lost_min_bev_y`(발화선보다 작아야 안전망이 산다) |
| ★`tl_zone_min_box_h_t` ↔ `tl_lock_enable`★ | 짝이다 — 박스 버리기만 켜면 내 신호가 화면 위로 빠진 정지에서 빨간불에 풀린다(140401 288). 둘 다 끄면 T 는 `ZONE_GATE`(12·0.75)로 종전과 같다 |
| `tl_near_release_ratio_t` · `tl_zone_red_min_height_t` | 박스 버리기가 꺼졌을 때만 뜻이 있다. 정지 중 문턱(= 둘의 곱)이 다음 교차로 빨강(4~7px) 이하로 내려가면 해제를 막는다(`⏳ 해제 보류` 로그로 확인). 위로는 체결 순간 내 빨강(20px~) |
| `green_hold_s` | 잠금의 초록 확정 · `stop_latch` · 거짓 녹색(역광) 내성 — 녹색 HSV 확인과 함께 본다 |
| HSV 초록 범위·채도·밝기·`hsv_crop_center_ratio` | 'A' 화살표 판독(`tl_a_arrow_min_px`) · 감마 GREEN(`tl_gamma_green_px`) — 같은 녹색 화소 수를 쓴다. `hsv_sat_low` 는 녹색 HSV 확인에도 |
| `GREEN_HSV_H`·`GREEN_HSV_V_LO` · `tl_b_arrow_h_*`·`s_lo`·`v_lo` | 각각 전용 범위 — 노드 HSV 와 묶지 않는다. 'B' 창 HSV 는 원등 칸(`tl_b_round_min`)도 같이 잰다 |
| `tl_gamma` 켜는 조건 | white1 driving 의 2단 재발행 주기(0.25 s)를 바꾸면 `GAMMA_EXT_BRAKE_S`(1.0)도 본다 |
| `tl_zone_red_min_height_a` | 'A' #1100 등 크기(정지선 앞 9~10px) · `tl_a_arrow_min_h` — ★작은 신호 교차로를 경로 CSV 에 'T' 로 적지 말 것★(T 는 16px 밑 박스를 버리고 서기 전 12px 라 안 선다) |
| `tl_b_go_mode` | 현시표 전제('버스 빨강 + SG1 녹색 ⇒ SG2 녹색') — 대회 날 현시가 바뀌면 `arrow` |
| 시간 문턱 전부(`tl_hold_s`·`tl_gap_grace_s`·`tl_arm_hold_s`·`red_release_hold_s`/`_lost_s`·`green_hold_s`·`tl_img_stale_s`·`tl_b_bus_red_s`·`tl_lock_lost_s`·`tl_gamma_on_s`·`sl_hold_s`·`sl_stale_s`) | 벽시계 초다 — **realtime 으로 확인**. lockstep 은 2.8 fps 예선 영상(140401)에서 해제 324 vs realtime 311 처럼 늦어 보인다 |
| `tl_weights` / `tl_imgsz` | 엔진을 다시 만들면 conf 분포도 다시 잰다 |
| 런치 인자로 여는 기본값 | `tl_params.py` 한 곳 — `traffic_light.py` 가 선언하고 `camera_launch.py` 가 같은 값으로 런치 인자를 연다. 새 파라미터를 런치로 열 때도 `tl_params.py` 의 dict 에 더한다(`camera_launch` 의 설명 dict 도 함께) |

## 4. 튜닝 순서 (권장)

1. **증상을 녹화로 재현** — 해당 장면 영상과 구간을 정하고, 지금 값으로 `cam-test` 한 번(기준 런).
2. **측정** — `signals.csv` 의 `tl_boxes`(박스 높이·y·conf·색·녹색 px)·`tl_state`·`brake_level` 로 문제 박스의 분포를 잰다. 디버그 영상에서 그 장면을 눈으로 본다(HUD 구간 칸의 `잠금 …`·'B' 근거 줄 포함).
3. **값을 정한다** — 진짜/가짜 분포 사이 여유 가운데로. 틀리는 방향이 안전한 쪽인지 확인.
4. **대조군** — `--param traffic_light.<이름>=<값>` 으로 같은 구간을 다시 돌린다(코드는 안 고친다).
5. **회귀** — 아래 전부를 돌려 기준 런과 `tl_state` 프레임 차이를 센다(lockstep — 같은 입력에 같은 결과). 바뀐 프레임은 하나하나 디버그 영상으로 본다.
   시간 문턱을 건드렸거나 정지·해제 시점이 쟁점이면 **같은 명령에 `--mode realtime` 을 붙여 한 번 더** 돌린다.
6. **적용** — 사용자 확인 → 기본값 수정(`tl_params.py` 또는 `declare_parameter` 한 곳) → `colcon build --packages-select white1` → 같은 회귀 → CHANGELOG(근거·숫자·되돌리는 법) → `main`.

## 5. 회귀 세트 (이 기계)

```bash
source /opt/ros/humble/setup.bash; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; cd ~/cam_testbed
C=contracts/gold_white2_stopline.yaml   # [2026-10-21] white1 실행 파일·보정 + white2 aux(구간·허락)
OUT=/home/mad1/gold/gold_ws; V=/home/mad1/tl_eval_videos; N=/home/mad1/nxde_video
python3 -m tb.run doctor --contract $C --video $V/cam-20260913_144031.mp4
run() { python3 -m tb.run run --contract $C --domain 91 --out $OUT --note "…" "$@"; }
# q(예선) · v1f · v2f(본선) · k-city 전체 — GPS 구간·허락
run --video $V/cam-20260913_140401.mp4 --preset ~/tl_eval/zone_q.yaml   --start 0 --limit 0 --name X-q
run --video $V/cam-20260912_105807.mp4 --preset ~/tl_eval/zone_v1f.yaml --start 0 --limit 0 --name X-v1f
run --video $V/cam-20260913_144031.mp4 --preset ~/tl_eval/zone_v2f.yaml --start 0 --limit 0 --name X-v2f
run --video $N/cam-20260912_114105.mp4 --preset ~/tl_eval/zone_kcity.yaml --start 0 --limit 0 --name X-kcity
# #400 'B' 5장면 (P2B = 10/17 화살표·잘림 · B4v1·kc400·s400 = 버스·차량 빨강 · B4v2 = 2현시)
run --video $N/cam-20261017_172742.mp4 --preset ~/tl_eval/zone400B.yaml --start 1550 --limit 570 --name X-P2B
run --video $V/cam-20260912_105807.mp4 --preset ~/tl_eval/zone400B.yaml --start 2689 --limit 400 --name X-B4v1
run --video $V/cam-20260913_144031.mp4 --preset ~/tl_eval/zone400B.yaml --start 2545 --limit 395 --name X-B4v2
run --video $N/cam-20260912_114105.mp4 --preset ~/tl_eval/zone400B.yaml --start 3050 --limit 380 --name X-kc400
run --video $N/cam-20260913_160149.mp4 --preset ~/tl_eval/zone400B.yaml --start 150  --limit 80  --name X-s400
# 구간 T 문제 장면 — P3T(뒤 신호등 빨강) · q288(내 신호가 위로 빠짐, realtime) · P4T(역광)
run --video $N/cam-20261017_154550.mp4 --preset ~/tl_eval/zoneT.yaml --start 5850 --limit 800  --name X-P3T
run --video $V/cam-20260913_140401.mp4 --preset ~/tl_eval/zoneT.yaml --start 270  --limit 66   --name X-q288 --mode realtime
run --video $N/cam-20261017_094820.mp4 --preset ~/tl_eval/zoneT.yaml --start 2445 --limit 1060 --name X-P4T
run --video $N/cam-20261017_094820.mp4 --preset ~/tl_eval/zoneT.yaml --start 7040 --limit 260  --name X-G094820 --mode realtime  # 거짓 녹색
# 10/17 실차가 실제로 선 창만 구간 T + 허락
run --video $N/cam-20261017_094820.mp4 --preset ~/tl_eval/rs_094820.yaml --start 0 --limit 0 --name X-RS094820
run --video $N/cam-20261017_101543.mp4 --preset ~/tl_eval/rs_101543.yaml --start 0 --limit 0 --name X-RS101543
run --video $N/cam-20261017_172742.mp4 --preset ~/tl_eval/rs_172742.yaml --start 0 --limit 0 --name X-RS172742
# 역광 감마 — 이미 서 있는 상태(외부 /brake_level 2) · white1 계약(색만 내는지)
run --video $N/cam-20261017_094820.mp4 --preset ~/tl_eval/brkW.yaml --start 4300 --limit 700 --name X-W2
run --video $N/cam-20261017_094820.mp4 --preset ~/tl_eval/brkW.yaml --start 5950 --limit 500 --name X-W3
python3 -m tb.run run --contract contracts/gold_white1_stopline.yaml --domain 91 --out $OUT --note "…" \
  --video $N/cam-20261017_094820.mp4 --preset ~/tl_eval/w1_p4.yaml --start 2445 --limit 1060 --name X-W1P4
# 'A' 일부러 씌운 시험 (실제 구간 아님) — 화살표 판독
run --video $V/cam-20260913_140401.mp4 --preset ~/tl_eval/zone_q_aforce.yaml --name XQA-q
run --video $V/cam-20260913_144031.mp4 --preset ~/tl_eval/zone_v2f_aold.yaml --start 3150 --limit 500 --name XAOLD-v2f
# 채점
python3 ~/tl_eval/ev.py X q,v1f,v2f
# 타이밍 확인 — 위 명령 끝에 --mode realtime 을 붙이고 이름을 바꾼다(예: --name RTX-q). 영상 길이만큼 걸린다.
#   한 번에 하나씩 돌린다 — 동시에 돌리면 GPU 를 나눠 써서 realtime 이 실차보다 느려진다.
```

- 구간 프리셋 `zone_{q,v1f,v2f,kcity}.yaml` 은 **GPS 기준**이다 — 같은 주행의 record 위치를 경로 CSV terrain 에 대응시켰다
  (도구 `~/tl_eval/zone_gps/`, 시각 오차 ±0.5 s). **허락도 구간 안에서만** 준다(`/tl_enable` 끄고 `/tl_permit` = 구간) — 실차 white2 와 같다.
  정답표 `ev.py` 는 미션 신호 기준(S 자리 뺌, #400 넣음 — `docs/kcity`).
- `zoneT` 은 영상 전체를 구간 T 로(문제 장면 재현용) · `rs_*` 는 10/17 기록에서 실제로 선(gps_kmh<0.5) T 정지 창만 구간 T + 허락 ·
  `brkW` 는 구간 T + 처음부터 `/brake_level` 2(감마의 '서 있다' 조건을 외부 2단으로 탄다 — 이미 서 있는 상태 재현) ·
  `w1_p4` 는 white1 실차처럼 허락 없음 + 원본 161.2 s(2627)부터 driving 의 2단. [2026-10-22] 세션 임시 폴더에서 `~/tl_eval/` 로 옮겼다.
- 디버그 영상 = `gold_ws/testbed_results/<런>/tl_debug_image.mp4`.

**현재 기준값** — lockstep `1021_16*_GG-*`·`1021_17*_GG-*` [2026-10-21, 역광 감마까지 — 설치본 `19daeae`(일부 `f675db6`), 판정 코드는 `d91f232` 와 같다]

| 항목 | 제동 → 해제 프레임 (lockstep) | realtime |
|---|---|---|
| 정답표 `ev.py` q · v1f · v2f | 2/2 · 5/5 · 1/1, 오정지 0 | — |
| q (140401) | 251 → 255 · 358 → 363 | q288 : 285 → 335 (`GG-q288`) |
| v1f (105807) | 1502 · 1965 · 2995 · 3634 · 4867 | — |
| v2f (144031) T1 | 752 → 1729 | — |
| k-city — T2 · B · T5 (구간 끝 2249 · 3406 · 5142) | 2231 · 3368 · 5075 — A(3786~3995)는 제동 없음 | — |
| 'B' P2B · B4v1 · kc400 · s400 | 1843 → 1864 · 2995 → 3088 · 3349 → 3429 · 212 → 229 | — |
| 'B' B4v2 (2현시) | 제동 없음(진행) | — |
| P3T (문제 3) | 5851 → 6079 · 6456 → 6649 | 5860 → 6078 · 6456 → 6649 (`GG-RTP3T`) |
| P4T (문제 4 역광, 창1 녹색 2959) | 2544 → 2974 | 2543 → 2970 (`GG-P4Trt`) |
| G094820 (거짓 녹색) | — | 7093 → 창 끝까지 정지(빨간불 해제 0) |
| RS094820 실정지 창 | 2232 → 2300 · 2544 → 2972 · 6634 → 6800 · 7094 → 8102 | — |
| RS101543 · RS172742 | 1797 → 2279 · 3151 → 3412 | 101543 : 1797 → 2275 (`GG-RT101543`) |
| W2 · W3 (외부 2단, 녹색 4590 · 6219) | CHANGELOG 2026-10-21 (3) 표 : 4604 · 6228 해제 | 4600 · 6226 |
| W1P4 (white1 계약) | `/brake_level` 2 = 2626~ · `/tl/state` GREEN 2964 부터 연속 | — |

판단 타이머 흔들림으로 제동 전이 ±1~3프레임은 정상이다(예 : B4v2 2752·2753 의 버스 0.3 s 확정 지터). 2.8 fps 예선 영상의 lockstep 해제 지연은 시험 방식 탓이고 realtime 이 실차에 가깝다.
기능별로 끈 대조군(`tl_gamma:=0` · `tl_green_hsv_confirm:=false` · `tl_zone_min_box_h_t:=0 tl_lock_enable:=false`)과의 차이는 white2 CHANGELOG 2026-10-20 · 10-20 (2) · 10-21 (3) 표.
