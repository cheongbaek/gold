#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
traffic_light.py ― 신호등 인지·정지 [white1]
════════════════════════════════════════════════════════════════════════════════
하는 일은 하나뿐이다 — ★카메라로 신호등을 보고, 빨간불이 확정되면 차를 세운다★.
그리고 ★빨간불이 0.5초 이상 안 보이거나 초록불이 확정되면 놓는다★ (white806 판과
다른 점 — 아래 '정지 래치' 절).

두 곳에서 쓴다. 어느 쪽이든 이 노드가 하는 일은 /brake_level 하나뿐이다:
  · 자율주행 : driving 이 경로추종 중(/drive_state=DRIVE_RUN)일 때 자동으로 개입한다.
               해제되면 driving 이 20Hz 로 계속 내던 목표펄스가 그대로 다시 통해
               ★스스로 재출발★ 한다(이 노드는 펄스를 기억하지도, 복원하지도 않는다).
  · 수동 조종 : nxde master 의 ★'신호등 인지' 체크박스★ 가 /tl_enable=True 를 낼 때만
               개입한다. 해제되면 arduino 가 캐시하고 있던 master 의 레버값이 그대로
               되살아난다 — ★E-STOP 이 풀릴 때와 같은 성질★ 이고, 그것을 얻으려고
               이 노드는 /cmd_vel_raw 를 ★내지 않는다★(publish_cmd_vel 기본 false).

구 white(white_cam_ws) 에서 신호등에 해당하는 부분만 떼어 왔다:
  · 인지 = perception.py 의 신호등 파이프라인
      YOLO 박스 → 크기·종횡비·conf 필터 → HSV 색 교정 → RED 적색픽셀 관문
      → RED / RED_FAR / GREEN / UNKNOWN
  · 판단 = camera_judgment.py 의 RED 확정 필터
      tl_hold_s 연속 목격 + tl_gap_grace_s 이내의 끊김은 봐주기(깜빡임·오검출 흡수)
  · ★정지선★ = perception.py 의 stop-line 절 [2026-08-14 추가 — 아래 절 참고]

★가져오지 않은 것★ 차선(BEV·슬라이딩윈도우·/lane_metrics)·횡단보도.
  white1 에는 그 발행자도 소비자도 없다.

════════════════════════════════════════════════════════════════════════════════
 ★디버그 창 = 계기판 [2026-08-24 전면 개편]★
════════════════════════════════════════════════════════════════════════════════
 이 노드의 파라미터는 ★눈으로 보고 잡는 값★ 이다(ROI·근접도 게이트·색 임계·BEV
 사다리꼴·두 문턱). 그런데 창이 그것을 못 보여 주고 있었다:

   · 한글이 통째로 깨졌다 — cv2.putText 는 ASCII 만 그린다. stop_why(예비제동·정지선
     앞·대기 상한·허락 없음 …)가 `???1??? PRE[????????-1???]` 로 나왔다. 즉 ★'왜 이
     단계인가' 를 화면에서 읽을 수 없었다★.
   · 1920 에 그린 뒤 640 으로 줄였다 — 글자·선이 전부 실효 1/3 크기가 됐다.
   · BEV 썸네일에 ★범퍼선(거리 0 의 기준)이 안 그려졌다★ — 경계 조건 때문에
     기본값(bev_bumper_y_px=480 = bev_h)에서 항상 탈락했다.
   · 개입 허락(/tl_enable·/tl_permit)이 화면에 없었다 — 체크박스를 안 켠 것이
     '빨간불을 보는데 아무 일도 안 일어남' 으로 보였다.

 지금은 이렇게 그린다 (창 = 뷰 960 + 우측 패널 + 하단 HUD = 1248x610):

   ┌────────────────────────────────┬──────────────┐
   │ 뷰 — ROI 밖은 어둡게, 사다리꼴  │ BEV 썸네일   │  범퍼·B1·B2 선
   │ 은 시안, 정지선은 마젠타        │ 근접 게이지  │  지금값 vs 임계
   │ ★제동 1단=주황 / 2단=빨강 테두리│ 정지선 게이지│  지금값 vs 두 문턱
   │  — 곁눈으로도 보인다★          │ 판정 파라미터│  정적, 한 번만 그린다
   ├────────────────────────────────┴──────────────┤
   │ HUD ① 상태·확정 스트릭·근접도·raw/drop·FPS·보정 │
   │     ② 제동 단계·근거·★허락★·정지선 대기 상한    │
   │     ③ 정지선 거리·두 문턱·범퍼행·BEV 크기       │
   └───────────────────────────────────────────────┘

 ★판정 로직은 한 줄도 안 바뀌었다★ 이 개편은 보이는 것만 바꾼다. 그리고 비용은
 오히려 줄었다 — ★옛 기본값 대비 3~4배 싸다★(같은 프레임 실측). 근거는 _draw 주석.

════════════════════════════════════════════════════════════════════════════════
 ★정지선 앞 2단계 정지 [2026-08-14 도입 → 2026-08-19 개편]★
════════════════════════════════════════════════════════════════════════════════
 종전에는 ★RED 확정이면 그 자리에서★ 섰다 — 근접도 게이트(박스 크기)가 '얼마나
 가까운가'의 유일한 근거였기 때문이다. 그런데 그 값은 신호등 크기·렌즈·설치 높이에
 따라 흔들리는 ★간접 지표★ 다. 정지선은 '여기가 정지 지점'이라고 노면이 직접 말해
 주는 것이므로, 그것이 보이면 그쪽을 따른다.

 ★[2026-08-19] 정지선이 보이면 두 단계로 선다 (지시사항)★
   종전 판은 '참았다가(무개입) 트리거 행에서 2단' 이었다. 즉 감속 프로파일이
   ★코스트 → 급정지★ 둘뿐이라, 밖에서 보면 정지선 앞에서 한 번 급하게 서는 거동이
   된다(2단은 실측 2.2~3.8 m/s² — BRAKING.md). 종점 접근제동이 2026-08-19 에 같은
   이유로 1단+거리계산으로 바뀌었고, 신호등도 같은 태도로 맞춘다.

     RED 확정
       ├ 정지선을 못 봤다                → 즉시 2단        ★종전 동작 그대로★
       ├ 정지선이 보이는데 아직 멀다      → 무개입(대기)
       ├ 정지선까지 ≤ sl_brake1_px        → ★1단 예비제동★ (부드럽게 줄인다)
       ├ 정지선까지 ≤ sl_brake2_px        → ★2단 확정 정지★ (정지선 앞에 세운다)
       └ 봤다가 놓쳤다                    → 즉시 2단        (이미 선 위다)

   즉 ★정지선이 안 보이면 이 절은 통째로 없는 것과 같다★(지시사항). 인지가 안 되는
   날에도 종전 동작(RED 확정 → 즉시 2단)으로 조용히 되돌아간다 — 이 기능의 실패
   모드를 '기존 동작'으로 묶어 둔 것이다.

 ★단계는 올라가기만 한다★ RED 를 잡고 있는 동안 0 → 1 → 2 로만 간다. 2단을 물었다가
   정지선이 흔들린다고 1단으로 내리는 일은 없다. 리니어는 물리적으로 왕복하는 장치라
   그 왕복이 제일 나쁘다(아래 '정지 래치' 절의 flip-flop 실측과 같은 문제다).
   0 으로 돌아가는 길은 ★해제 경로 하나뿐★ 이다 — 빨간불이 사라지거나 허락이 없어질 때.

 ★대기·1단에는 두 개의 상한이 있다★ 오검출된 정지선이 2단을 무한정 미루면 그것이
 곧 '빨간불에 안 서는' 사고다. 그래서 두 가지가 감시한다:
   · sl_wait_max_s   RED 확정 시각으로부터 이만큼 지나면 정지선을 무시하고 2단
   · sl_override_gate_ratio  근접도가 게이트의 이 배를 넘으면(=신호등이 코앞이면)
                             정지선을 기다리지 않는다 — 물리적 상한이라 시간보다 낫다
   ★sl_wait_max_s 는 이 개편에서 5 → 8초로 늘렸다★ 종전의 '참는 동안'은 무개입
   코스트였지만 지금은 ★이미 1단으로 감속 중★ 이라, 상한을 늘려도 위험이 늘지 않는다.
   그리고 이 상한이 ★1단으로 감속하다 정지선 앞에서 멈춰 버린 경우★ 도 함께 받는다 —
   1단이 걸리면 arduino 가 구동펄스를 0 으로 덮으므로(아래 '왜 /brake_level 하나가
   정지의 본체인가') 차는 스스로 정지선까지 기어가지 못한다. 그대로 두면 2단 문턱에
   영영 못 닿으므로, 8초가 지나면 ★2단으로 올려 정지를 확정한다★(지시사항).

 ★왜 화면의 행이 아니라 BEV 픽셀 거리인가 [2026-08-19 교체]★
   종전에는 마스크 최하단 y 를 프레임 높이로 나눈 비율(sl_trigger_y_frac)로 판정했다.
   그것으로는 ★문턱을 두 개 둘 수 없다★ — 원본 화면의 행 간격은 거리에 비례하지
   않아서(원근) '1단 문턱과 2단 문턱이 몇 m 떨어져 있나'를 말할 수 없기 때문이다.
   BEV(IPM)로 펴면 ★한 픽셀이 어디서나 같은 거리★ 라 문턱을 여럿 두어도 뜻이 산다.
   그래서 판정값은 ★BEV 에서 정지선 최근접점 → 앞범퍼까지의 픽셀 거리★ 하나다.

   ⚠️ ★종전 판이 BEV 를 안 쓴 이유는 해소됐다★ 그때는 (a) IPM 캘리브가 구 차량
     마운트 값이고 (b) 왜곡보정이 파이프라인에 없어서 미보정 IPM 이 의미가 없다는
     것이 근거였다. (b)는 white1/camera_model.py 가 들어오면서 없어졌고(모든 카메라
     인지 노드가 기본으로 보정된 그림을 본다), (a)는 남아 있다 —
     ★bev_src_pts·bev_bumper_y_px·sl_brake1_px·sl_brake2_px 는 실측값이다★
     (STOPLINE_TEST.md 단계 2). 기본값은 출발점일 뿐이고, HUD 에 사다리꼴·범퍼선·
     두 문턱선을 그려 두었으니 ★눈으로 맞춘다★.
   미터가 필요하면 bev_px_to_m 을 재서 넣는다 — HUD·로그에만 붙고 ★판정 경로는
   픽셀 그대로다★(캘리브 하나가 틀려도 판정이 흔들리지 않게 하려는 것이다).

 ★왜 빨간 박스가 보일 때만 추론하는가★
   정지선 seg 는 신호등 detect 와 ★같은 프레임★ 에 한 번 더 도는 두 번째 추론이다.
   실측(이 PC, 1920x1080, cuda:0)으로 신호등만 4.3ms → 정지선까지 7.5ms 이니
   30fps 예산(33.3ms)에 여유는 충분하지만, 정지선이 필요한 순간은 '빨간 불이 보이는
   동안' 뿐이라 그때만 돌린다(sl_gate_red_s 안에 빨간 박스를 봤거나 정지 중일 때).
   ★평상시 비용은 0 이고, 남는 예산은 신호등 판정 기회로 남는다★ — 인지가 늦어져
   tl_hold_s(연속 RED)를 못 채우면 신호등에서 안 서기 때문이다(tl_interval 주석).

 ⚠️ ★정지선 앞에 서면 등기구가 화면 위로 벗어나기 쉬워진다★ 더 가까이 서게 되므로
   아래 '정지 래치' 절의 소실거리 문제가 그만큼 커진다. 실차에서 정지 유지 중
   tl_near_metric 이 0 이 되면 ★카메라를 5~8° 위로 틸트★ 하는 것이 정답이다.

════════════════════════════════════════════════════════════════════════════════
 ★어안 왜곡보정 — 이 노드가 소유하지 않는다 [2026-08-19]★
════════════════════════════════════════════════════════════════════════════════
 보정 계수·BEV 사다리꼴의 주인은 ★white1/camera_model.py★ 다. 이 노드가 하는 일은
 프레임을 받자마자 cam.undistort() 를 한 번 부르는 것뿐이고, 그 뒤로는 신호등 detect·
 정지선 seg·HUD 가 전부 ★보정된 그림★ 을 본다. 앞으로 붙일 차선 인지도 같은 모듈을
 부르면 같은 그림을 보게 된다 — 그러라고 밖으로 뺐다(그쪽 파일 헤더 참고).

 ★/image_raw 는 원본 그대로다★ 보정을 토픽 쪽에서 하지 않는 이유는 nxde/video.py 의
 원본 녹화가 '카메라가 실제로 준 그림'이어야 하기 때문이다(지시사항). 그래서 녹화된
 mp4 에는 어안이 남아 있는 것이 ★정상★ 이다.

 ⚠️ ★보정을 켜면 화면 스케일이 조금 달라진다★ alpha=0 이라 유효 영역만 잘라 원래
   크기로 늘리므로, 같은 신호등이 몇 % 커 보인다. 근접도 게이트
   (tl_red_stop_min_height=25px)와 tl_roi 는 그만큼 ★재실측 대상★ 이다(todo 8항).

════════════════════════════════════════════════════════════════════════════════
 발행 / 구독
════════════════════════════════════════════════════════════════════════════════
발행:
  /brake_level  std_msgs/Int32       ★리니어 1단(예비)·2단(정지) — 정지의 본체다(아래 참고)★
  /cmd_vel_raw  geometry_msgs/Twist  ★기본으로는 내지 않는다★ (publish_cmd_vel:=true 일 때만,
                                     그때도 ★2단에서만★ 낸다 — 1단 중에는 조향을 안 뺏는다)
  /tl/state     std_msgs/String      RED / RED_FAR / GREEN / UNKNOWN (기록·디버그·master 표시)
  /tl/stop_line_px   std_msgs/Float32  ★판정값★ BEV 에서 정지선→앞범퍼 픽셀 거리
                                       −1 = 미검출 / 0 = 범퍼선 도달(또는 지나침)
  /tl/stop_line_y    std_msgs/Float32  정지선 최하단 y(프레임 높이 비율). −1 = 미검출
                                       ★판정에는 안 쓴다★ — 기록·HUD 용(sl_check.py 가 읽는다)
  /tl/stop_line_wait std_msgs/Bool     지금 정지선 때문에 브레이크를 참고 있는가(아직 0단)
구독:
  /image_raw    sensor_msgs/Image    usb_cam
  /drive_state  std_msgs/String      ← driving.py (DRIVE_DONE 이면 브레이크 소유권 양보)
  /tl_enable    std_msgs/Bool        ← nxde master 의 '신호등 인지' 체크박스(상시)
  /tl_permit    std_msgs/Bool        ← driving.py 의 허락(상수 + DRIVE_RUN)

════════════════════════════════════════════════════════════════════════════════
 ★왜 /brake_level 하나가 정지의 본체인가★
════════════════════════════════════════════════════════════════════════════════
  nxde/arduino.py compose() 의 (4) 정상 자율주행 분기에 이 줄이 있다:

      pulse = 0 if brake > 0 else self.cmd_pulse

  브레이크가 0 이 아니면 A보드로 나가는 목표펄스가 ★강제로 0★ 이 된다. 즉
  /brake_level=2 를 내는 것만으로 ★리니어 2단 체결 + 인휠 구동 차단★ 이 동시에
  성립한다. 그 줄의 주석이 말하는 "주행 중에 브레이크를 요청하는 다른 발행자"가
  바로 이 노드다(구 white 에서는 camera_judgment 였다).

  ★1단(예비제동)도 같은 줄에 걸린다 [2026-08-19]★ 조건이 `brake > 0` 이므로 1단에서도
  구동이 끊긴다. 그래서 1단 예비제동은 '리니어 1/3 행정 + 구동 차단' 이고, 실측
  감속도는 ★1.30 m/s²(체결 뒤 0.55초는 행정 램프라 0)★ 이다(BRAKING.md 4절 —
  구동을 끊고 잰 값이다). 2단(2.2~3.8)의 절반 이하라 '부드럽게 줄인다'가 성립한다.
  ⚠️ 뒤집어 말하면 ★1단을 물고 있는 동안 차는 스스로 기어가지 못한다★ — 정지선
  앞에서 멈춰 버리면 그대로 서 있는다. 그 경우를 sl_wait_max_s 가 받아 2단으로
  올린다(위 '정지선 앞 2단계 정지' 절).

 ★왜 /cmd_vel_raw 를 내지 않는가 [white1 에서 기본값을 뒤집었다]★
  펄스는 위에서 이미 0 이 되므로 이 토픽이 실제로 할 일은 ★조향각 0(일직선)★ 뿐인데,
  그 대가가 크다:
    ① arduino 의 명령 캐시(cmd_pulse·cmd_angle)를 0 으로 덮어쓴다. 그러면 브레이크를
       푸는 순간 되살아날 값이 ★0★ 이 되어, master 로 몰던 사람은 레버를 그대로 두고도
       차가 안 나가게 된다. ★E-STOP 이 풀릴 때처럼 '있던 명령이 그대로 돌아오는' 성질★
       을 원하므로 캐시를 건드리지 않는다.
    ② /cmd_vel_raw 의 주인은 driving(20Hz)·master 다. 발행자가 겹치면 캐시가 번갈아
       덮여 '누가 지금 명령의 주인인가'를 추적할 수 없게 된다.
  → 그래서 이 노드는 ★/brake_level 하나만★ 낸다. 정지 성립에 그것으로 충분하다(위 절).
    정지 중 바퀴를 일직선으로 두고 싶으면 publish_cmd_vel:=true 로 켠다 — 그때는 위
    ①②를 감수하는 것이다(자율주행 전용으로만 쓸 것을 권한다).

════════════════════════════════════════════════════════════════════════════════
 안전 규약 — 이 노드가 지키는 세 가지
════════════════════════════════════════════════════════════════════════════════
 ① fail-open : 모델을 못 읽었거나(가중치 경로 오류) 영상이 끊기면 ★아무 개입도 하지
    않는다★. 신선하고 확정된 RED 가 있을 때만 브레이크를 건다. 카메라가 죽었다고
    차가 영영 못 가는 일은 없어야 한다.
 ② ★허락받은 구간에서만 개입★ 둘 중 하나가 성립해야 브레이크를 건다:
      · /tl_enable == True   master 의 '신호등 인지' 체크박스 — ★체크되어 있으면 상시★
      · /tl_permit == True   driving 의 TRAFFIC_LIGHT_ENABLE(코드 내부 상수)이면서
                             ★DRIVE_RUN 일 때만★. 매핑(MAP_*)에서는 절대 안 온다 —
                             사람이 페달로 모는 구간에 리니어가 끼어들면 안 된다.
                             그 상수를 False 로 두면 ★카메라 없이 종전대로 돈다★.
    아무 때나 리니어가 튀어나오는 것은 nxde/arduino.py 가 2026-08-04 부터 지켜 온
    불변식("모드 전환은 절대로 리니어를 체결하지 않는다")을 밖에서 깨는 짓이다.
    ★체크박스는 사람이 직접 켠 것이므로 '허락'이다★ — 스스로 켜지지 않는다.
    체크를 끄면 그 순간 걸어 둔 것을 풀고 손을 뗀다. 벤치에서 판정만 보고 싶으면
    require_permission:=false (그때도 브레이크는 걸리니 차를 세워 두고 할 것).
    ⚠️ 수동조종 모드(D5 내림)에서는 arduino 가 브레이크를 항상 0 으로 보내므로
      체크를 켜도 리니어는 물리지 않는다 — 그 불변식은 이 노드가 깨지 않는다.
 ③ 남의 브레이크는 풀지 않는다 : 0단은 ★이 노드가 직접 2단을 걸었을 때만★ 낸다.
    driving 이 DRIVE_DONE(도착·경로이탈)으로 스스로 2단을 물고 있는 구간에서는
    해제를 아예 발행하지 않는다 — 그 구간의 해제는 driving 의 몫이다.

 ★정지 래치(stop_latch) — white1 은 기본 OFF 다★
  white806 판은 기본 ON 이었다. 근거는 이랬다: 신호등에 다가갈수록 등기구가 화면 위로
  올라가 tl_roi 를 벗어나므로, 정지선 앞에 서면 신호등이 시야에서 사라지는 것이
  정상이고 → 그 순간 UNKNOWN → 해제 → 차가 다시 굴러간다.
  ★white1 은 '빨간불을 보는 동안만 잡는다'를 택했다★ (지시사항):
      RED 확정(tl_hold_s 0.4s)      → 2단  ★즉시★
      RED 를 red_release_hold_s(0.5s) 동안 못 봄, 또는 GREEN 확정 → 0단
  ★[2026-08-14] 놓는 쪽에만 유예를 뒀다 — 실차에서 리니어가 들락날락했다★
  ※ [2026-08-24 정정] 이 절은 그 유예를 오래도록 '1초' 로 적어 왔는데 ★선언된
    기본값은 0.5초다★(아래 declare_parameter). 글이 아니라 값이 맞다 — 실제 거동은
    0.5초로 돌았다. 값을 올리려면 파라미터로 올리고 이 글도 같이 고칠 것.
  종전에는 스트릭이 끊기는 순간(0.3s) 바로 놓았다. 인지가 한두 번 흔들리면
      RED 확정 → 0.3초 놓침 → 해제 → 다시 0.4초 연속 → 체결 …
  이 1초 주기로 반복된다. ★리니어는 물리적으로 왕복하는 장치라 이 왕복이 제일 나쁘다★
  (B보드 1회 구동 최대 1초). 무는 쪽은 그대로 두어 반응이 늦어지지 않게 했다.
  ⚠️ ★[2026-08-24 정정] 소비자 쪽에는 이제 그 유예가 없다★ 종전에 이 자리에는
  "master·driving 이 /tl_brake_req 를 자기 값과 max 로 합치고, 요구가 0 이 되어도
  1초는 유지한다(TL_REQ_RELEASE_HOLD_S)" 라고 적혀 있었다. 그런데 실제 값은
  ★driving.py 의 TL_REQ_RELEASE_HOLD_S = 0.0 ('해제 유예 없음 — 따라만 간다')★ 이다.
  즉 ★해제 유예는 이 노드의 red_release_hold_s 한 겹뿐★ 이고, 토픽이 잠깐 밀리는
  경우를 받아 주는 두 번째 겹은 없다. 그것이 필요하다고 판단되면 저쪽 상수를 올릴 것
  — 이 글을 되돌리지 말고.
    · tl_gap_grace_s(0.3s) 는 그대로다 — 그것은 '연속 목격 스트릭'의 유예이고,
      해제 유예(위)와는 다른 값이다.
    · ⚠️ 신호등이 시야를 완전히 벗어나면 0.5초 뒤 차는 다시 굴러간다.
      ★[2026-08-14 정정] 그러니 근접도 게이트에는 상한이 있다 — 크게 잡으면 안 된다★
      게이트를 키울수록 ★가까이 가서★ 물고 ★가까이 서므로★, 어느 선을 넘으면 서 있는
      동안 등기구가 화면 위로 벗어나 그대로 굴러간다. 카메라 틸트 0°·등기구 5m·
      카메라 1.2m 면 소실거리는 화각 60°에서 약 12m, 90°에서 약 7m 다.
      정지 지점이 그보다 멀도록 게이트 ★상한★ 을 잡아야 한다(gold/tl_tune.py 가
      기록 1회로 렌즈상수 k 와 소실거리를 뽑아 상한을 계산해 준다).
      더 가까이 세우고 싶으면 게이트가 아니라 ★카메라를 5~8° 위로 틸트★ 하는 것이
      정답이다 — 소실거리가 절반 이하로 줄어든다.
  래치가 필요하면 stop_latch:=true — 그때는 GREEN 을 봐야만 놓는다.
"""

import os
import threading
import time

import cv2
import numpy as np
#  ★PIL 은 반드시 별명으로 받는다★ 아래에서 sensor_msgs 의 Image 를 그대로 받으므로,
#  별명 없이 `from PIL import Image` 를 쓰면 ROS 메시지 타입이 PIL 을 덮어써
#  ★cb_image 의 타입 힌트가 조용히 PIL 을 가리킨다★. 실제로 한 번 밟았다.
try:
    from PIL import Image as PILImage, ImageDraw, ImageFont
except ImportError:                     # fail-open — 아래 TextRenderer 주석 참고
    PILImage = ImageDraw = ImageFont = None
import rclpy
import rclpy.executors
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Int32, String
from cv_bridge import CvBridge
from ultralytics import YOLO

# ★카메라 기하(어안 왜곡보정·BEV)의 단일 소유자★ [2026-08-19]
#   보정 계수도 사다리꼴도 이 파일에는 없다 — 차선 인지가 붙을 때 같은 값을 쓰게
#   하려고 밖으로 뺐다(그쪽 헤더 참고). 여기서는 undistort() 와 nearest_dist_px() 만 쓴다.
from white1 import camera_model

# HSV 가 YOLO 라벨을 ★위험한 방향(RED→GREEN)★ 으로 뒤집는 것을 허용하는 conf 상한.
# 이보다 자신 있는 박스는 색 몇 픽셀로 뒤집지 않는다. (perception.py 와 같은 값)
HSV_FALLBACK_CONF = 0.55

# driving.py 의 상태 문자열. 값이 바뀌면 여기도 바꿔야 한다(driving.py S_* 상수).
DRIVE_RUN_STATE  = 'DRIVE_RUN'
DRIVE_DONE_STATE = 'DRIVE_DONE'
DRIVE_STATE_STALE_S = 2.0    # 이보다 오래된 /drive_state 는 '모른다'로 친다

# ★신호등 가중치 — 경로를 하드코딩한다 [2026-08-14 지시]★
#   .engine(TensorRT)은 ★빌드한 GPU·드라이버에 묶인다★ — 다른 기계로 옮기면 로드가
#   실패하고, 그때 이 노드는 fail-open 으로 아무 개입도 하지 않는다(안전한 쪽).
#   그 기계에서 다시 export 하거나 .pt 를 tl_weights 파라미터로 주면 된다.
TL_WEIGHTS = '/home/mad2/runs2/runs/detect/combined_light/weights/best.engine'

# ★정지선 가중치 — 구 white 의 차선 seg 모델이다★
#   클래스 = {0: crosswalk, 1: lain_lines, 2: stop-line} (엔진 메타데이터 실측).
#   우리는 stop-line 만 쓴다. 이름으로 클래스를 찾고, 못 찾으면 SL_CLASS_FALLBACK
#   으로 폴백한다 — 신호등 라벨을 이름에서 유도하는 것과 같은 이유다(_load_model).
SL_WEIGHTS = '/home/mad2/runs2/runs/segment/lane_line_new2/weights/best.engine'
SL_CLASS_FALLBACK = 2

# master 의 '신호등 인지' 체크박스가 이 주기보다 오래 끊기면 '허락 없음'으로 본다.
#   (master 창이 죽었는데 체크가 켜진 채로 굳어 있는 상태를 막는다)
TL_ENABLE_STALE_S = 2.0

# 물고 있는 동안 브레이크를 다시 주장하는 주기 [s]. master 의 KEEPALIVE_S(0.5)보다
# 짧아야 한다 — 그래야 남이 덮어도 곧바로 되돌아온다(_apply_brake 주석 참고).
BRAKE_KEEPALIVE_S = 0.25



# ══════════════════════════════════════════════════════════════════════════════
#  창에 글자를 그리는 도구 — ★한글 HUD★ [2026-08-24]
# ══════════════════════════════════════════════════════════════════════════════
#  ★왜 필요한가★ cv2.putText 는 ASCII 만 그린다. 종전 HUD 는 stop_why(전부 한글)를
#  `???1??? PRE[????????-1???]` 로 찍고 있었다 — 즉 이 창의 존재 이유인 ★'왜 이
#  단계인가' 를 화면에서 읽을 수 없었다★. 그래서 한글이 필요한 글자는 PIL 로 그린다.
#
#  ★새 의존성이 아니다★ ultralytics 가 pillow>=7.1.2 를 요구하므로, YOLO 를 돌리는
#  곳에는 이미 깔려 있다. 그래도 없을 때를 대비해 ★fail-open★ 이다 — 폰트를 못 찾으면
#  경고 한 줄 남기고 cv2.putText 로 물러난다(한글은 다시 물음표가 되지만 창은 뜨고
#  숫자는 읽히고 차는 그대로 선다). camera_model 이 캘리브를 못 읽을 때와 같은 태도다.
#
#  ★비용을 어떻게 0 으로 만드는가★ PIL 로 스트립(960x70) 하나를 그리는 데 3ms 다.
#  30fps 에서 매 프레임 내면 10% 예산이므로 세 층으로 캐시한다:
#    ① 폰트   ImageFont.truetype 은 크기마다 한 번만
#    ② 글자폭 getlength 는 (글자, 크기) 마다 한 번만 — 열 배치에 매 프레임 쓴다
#    ③ 스트립 ★문자열·색·위치가 하나도 안 바뀌면 지난 그림을 그대로 돌려준다★
#  ③ 이 핵심이다. HUD 의 숫자는 초당 10회쯤 바뀌므로 대부분의 프레임은 ③에서 끝난다.
#
#  ★고정폭 폰트를 쓴다★ NanumGothicCoding 은 ASCII 폭이 정확히 크기의 절반이고 한글은
#  두 칸이다. 그래서 HUD 를 '몇 번째 칸' 으로 적을 수 있고, 줄이 달라도 열이 맞는다 —
#  계기판은 열이 맞아야 눈이 값을 따라간다.

#  ★한글 폰트 후보 — 앞에서부터 처음 있는 것을 쓴다★
#  NanumGothicCoding 이 1순위인 이유는 ★고정폭★ 이기 때문이다(위 주석). 없으면
#  Noto CJK → NanumGothic 으로 내려가는데, 그때는 열이 조금 어긋난다(읽기는 된다).
FONT_CANDIDATES = (
    '/usr/share/fonts/truetype/nanum/NanumGothicCoding.ttf',
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/truetype/nanum/NanumGothic.ttf',
    '/usr/share/fonts/truetype/unfonts-core/UnDotum.ttf',
)

#  ── 색 (BGR) ─────────────────────────────────────────────────────────────
#  신호등 창이 쓰는 색은 ★화면의 의미와 1:1★ 이다. 여기가 그 정본이다.
C_BG     = (26, 26, 28)        # 패널 바닥
C_HUD    = (15, 15, 17)        # 하단 HUD 띠 바닥
C_TXT    = (228, 228, 232)     # 보통 글자
C_DIM    = (138, 138, 144)     # 참고값 글자
C_FRAME  = (92, 92, 96)        # 패널 테두리
C_RED    = (60, 60, 255)       # RED · 2단
C_GREEN  = (80, 220, 90)       # GREEN · 범퍼선 · 허락 있음
C_AMBER  = (0, 180, 255)       # 1단 예비제동 · B1 · 경고
C_CYAN   = (235, 225, 70)      # BEV 사다리꼴
C_MAGENTA = (255, 0, 255)      # 정지선
C_GRAY   = (150, 150, 150)     # UNKNOWN 박스


class TextRenderer:
    """한글 스트립 렌더러. ★상태는 캐시뿐이다★ (그림을 기억하지 않는다).

        tr = TextRenderer(log=node.get_logger())
        strip = tr.strip(w, h, [(x, y, '제동 1단', C_RED, 15, True)], C_HUD)

    items 의 한 원소 = (x, y, 글자, BGR색, 크기, 굵게). y 는 ★글자 상단★ 이다
    (cv2.putText 의 baseline 과 다르다 — 줄 배치를 위에서부터 세는 것이 편하다).
    """

    def __init__(self, log=None, font_path=''):
        self._log = log
        self.path = ''
        if PILImage is None:
            self._warn("Pillow 가 없다 — HUD 한글이 물음표로 나온다(그 외는 정상)")
        else:
            cands = ((font_path,) if font_path else ()) + FONT_CANDIDATES
            self.path = next((p for p in cands if p and os.path.exists(p)), '')
            if not self.path:
                self._warn("한글 폰트를 못 찾았다 — HUD 한글이 물음표로 나온다. "
                           "`sudo apt install fonts-nanum-coding` 로 해결된다")
        self.ok = bool(self.path)
        self._font, self._width, self._cell, self._patch = {}, {}, {}, {}

    def _warn(self, msg):
        if self._log is not None:
            self._log.warn(f"🖋 {msg}")

    # ── 폰트·치수 ────────────────────────────────────────────────────────
    def font(self, size, bold=False):
        key = (size, bold)
        if key not in self._font:
            path = self.path
            if bold:
                # 같은 계열의 Bold 가 옆에 있으면 그것을 쓴다(없으면 보통 굵기).
                alt = path.replace('Coding.ttf', 'CodingBold.ttf').replace(
                    'Gothic.ttf', 'GothicBold.ttf').replace('Regular', 'Bold')
                if alt != path and os.path.exists(alt):
                    path = alt
            self._font[key] = ImageFont.truetype(path, size)
        return self._font[key]

    def width(self, txt, size, bold=False):
        """글자 폭[px]. ★매 프레임 열 배치에 쓰므로 캐시한다★.

        ⚠️ 폰트가 없을 때도 ★실제로 재야 한다★ — 글자 수 × 상수로 어림하면 물러난
           경로에서 열 계산이 틀려 ★두 칸이 겹쳐 찍힌다★(실제로 그렇게 나왔다).
           cv2 로 그릴 것이므로 cv2 의 자로 잰다.
        """
        key = (txt, size, bold)
        if key not in self._width:
            self._width[key] = (
                float(self.font(size, bold).getlength(txt)) if self.ok
                else float(cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX,
                                           size / 32.0, 1)[0][0]))
        return self._width[key]

    def cell(self, size):
        """한 칸(=ASCII 한 글자) 폭. HUD 의 열 좌표 단위다."""
        if size not in self._cell:
            self._cell[size] = self.width('A', size) if self.ok else size * 0.55
        return self._cell[size]

    def line_h(self, size):
        """줄 높이 — 크기에 여백을 더한 값. 줄 간격의 정본이다."""
        return size + 6

    # ── 그리기 ──────────────────────────────────────────────────────────
    def strip(self, w, h, items, bg):
        """단색 바닥 위에 글자만 있는 스트립을 새로 그려 돌려준다."""
        img = np.full((h, w, 3), bg, np.uint8)
        self.onto(img, items)
        return img

    def onto(self, img, items):
        """이미 있는 그림 위에 글자를 얹는다(PIL 변환 1회로 몰아 그린다).

        ⚠️ img 는 ★연속 배열★ 이어야 한다(PILImage.fromarray 의 전제다). 그래서
           캔버스 슬라이스에 직접 쓰지 않고, strip() 으로 새 배열에 그린 뒤 blit 한다.
        """
        if not items:
            return img
        if not self.ok:
            # fail-open — 한글은 물음표가 되지만 숫자·영문은 그대로 읽힌다.
            for x, y, txt, col, size, _bold in items:
                cv2.putText(img, txt, (int(x), int(y) + size - 2),
                            cv2.FONT_HERSHEY_SIMPLEX, size / 32.0, col, 1, cv2.LINE_AA)
            return img
        pim = PILImage.fromarray(img)
        draw = ImageDraw.Draw(pim)
        for x, y, txt, col, size, bold in items:
            draw.text((x, y), txt, font=self.font(size, bold),
                      fill=(col[2], col[1], col[0]))          # BGR → RGB
        # np.asarray 는 PIL 버퍼를 감싸기만 하므로, 원본 img 로 되돌려 복사한다.
        img[:] = np.asarray(pim)
        return img

    def patch(self, txt, col, size=12, bold=False, bg=C_BG):
        """짧은 ★고정 라벨★ — 한 번 그려 두고 blit 한다(패널 라벨용)."""
        key = (txt, col, size, bold, bg)
        if key not in self._patch:
            w = int(self.width(txt, size, bold)) + 5
            self._patch[key] = self.strip(w, size + 6, [(2, 1, txt, col, size, bold)], bg)
        return self._patch[key]


class StripCache:
    """스트립 하나를 ★내용이 바뀔 때만★ 다시 그린다.

    HUD 는 대부분의 프레임에서 글자가 그대로다(FPS 를 정수로 찍는 이유이기도 하다).
    key 가 같으면 지난 그림을 그대로 돌려주므로 PIL 이 아예 돌지 않는다.
    """

    def __init__(self, tr):
        self.tr = tr
        self._key = None
        self._img = None

    def get(self, w, h, items, bg):
        key = (w, h, bg, tuple((round(i[0]), round(i[1]), i[2], i[3], i[4], i[5])
                               for i in items))
        if key != self._key:
            self._key = key
            self._img = self.tr.strip(w, h, items, bg)
        return self._img


class CanvasPool:
    """표시용 캔버스를 ★두 장 만들어 돌려 쓴다★.

    ★왜 매 프레임 만들지 않는가★ np.full + np.vstack 으로 1248x610 캔버스를 매
    프레임 새로 만들면 그것만 3.6ms 다(실측). 그리기 전체 예산보다 크다.

    ★왜 한 장이 아니라 두 장인가★ 그리는 것은 워커 스레드고 imshow 는 메인
    스레드다(traffic_light.main 의 규약). 한 장만 돌려 쓰면 메인이 창에 올리는 사이
    워커가 같은 버퍼를 덮어써 ★화면이 찢어진다★. 두 장이면 다음 덮어쓰기가 두
    프레임 뒤(30fps 에서 66ms)라 겹치지 않는다.
    """

    def __init__(self):
        self._key = None
        self._buf = []
        self._i = 0

    def get(self, w, h, bg=C_BG):
        if self._key != (w, h, bg):
            self._key = (w, h, bg)
            self._buf = [np.full((h, w, 3), bg, np.uint8) for _ in range(2)]
            self._i = 0
        self._i ^= 1
        return self._buf[self._i]


# ══════════════════════════════════════════════════════════════════════════════
#  작은 그리기 도구 — 전부 ASCII 전용(cv2)이라 비용이 사실상 0 이다
# ══════════════════════════════════════════════════════════════════════════════
def atxt(img, x, y, txt, col, scale=0.4, thick=1):
    """ASCII 글자. y 는 baseline (cv2.putText 그대로)."""
    cv2.putText(img, txt, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX,
                scale, col, thick, cv2.LINE_AA)


def chip(img, x, y, txt, col, scale=0.36):
    """★배경칩을 깐 라벨★ — 밝은 하늘·노면 위에서도 읽히게 하는 유일한 방법이다.

    y 는 칩의 ★위쪽★ 이다. 돌려주는 것은 칩의 높이(다음 줄을 쌓을 때 쓴다).
    """
    (tw, th), bl = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    h = th + bl + 4
    x, y = int(x), int(y)
    cv2.rectangle(img, (x, y), (x + tw + 6, y + h), col, -1)
    atxt(img, x + 3, y + th + 2, txt, (255, 255, 255), scale)
    return h


def gauge(img, x, y, w, h, frac, col, ticks=()):
    """★지금값을 문턱과 나란히 보여주는 막대★ 숫자보다 눈이 빠르다.

    frac  = 0.0~1.0 로 정규화된 지금값 (범위를 넘으면 꽉 찬다)
    ticks = ((위치 0~1, BGR), …) 문턱 눈금. 막대가 눈금을 넘으면 그 문턱을 넘은 것이다.
    """
    x, y, w, h = int(x), int(y), int(w), int(h)
    cv2.rectangle(img, (x, y), (x + w, y + h), (60, 60, 64), -1)
    fw = int(round(w * min(1.0, max(0.0, frac))))
    if fw > 0:
        cv2.rectangle(img, (x, y), (x + fw, y + h), col, -1)
    for tf, tc in ticks:
        tx = x + int(round(w * min(1.0, max(0.0, tf))))
        cv2.line(img, (tx, y - 3), (tx, y + h + 3), tc, 2)
    cv2.rectangle(img, (x, y), (x + w, y + h), (105, 105, 110), 1)


def blit(dst, patch, x, y):
    """patch 를 dst 의 (x, y) 에 붙인다. ★화면 밖은 잘라 낸다★ (예외를 내지 않는다)."""
    x, y = int(x), int(y)
    ph, pw = patch.shape[:2]
    H, W = dst.shape[:2]
    if x >= W or y >= H:
        return
    w, h = min(pw, W - x), min(ph, H - y)
    if w > 0 and h > 0:
        dst[y:y + h, x:x + w] = patch[:h, :w]


def dim_outside(view, rect, factor):
    """rect(사각형) ★바깥만★ 어둡게 한다 — 선 하나 없이 ROI 를 말하는 방법이다.

    ★사각형 슬라이스 네 장만 만진다★ 불리언 마스크(view[mask==0] = …)로 쓰면 같은
    일에 4배가 든다(팬시 인덱싱이 배열을 두 번 훑는다).
    """
    if factor >= 1.0:
        return
    h, w = view.shape[:2]
    x0, y0, x1, y1 = (max(0, min(w, rect[0])), max(0, min(h, rect[1])),
                      max(0, min(w, rect[2])), max(0, min(h, rect[3])))
    for a, b, c, d in ((0, y0, 0, w), (y1, h, 0, w),
                       (y0, y1, 0, x0), (y0, y1, x1, w)):
        if b > a and d > c:
            band = view[a:b, c:d]
            cv2.addWeighted(band, factor, band, 0.0, 0.0, dst=band)

class TrafficLight(Node):
    def __init__(self):
        super().__init__('traffic_light')

        # ── 입력 ───────────────────────────────────────────────────────────
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('device',      'cuda:0')   # GPU 없으면 'cpu'

        # ── 인지(perception.py 신호등 절에서 그대로 이식) ───────────────────
        #   ★가중치 경로★ 구 white 기본값(/home/mad2/...)은 이 PC에 없다.
        #   .engine(TensorRT)은 빌드한 GPU에 묶이므로, 다른 기계면 .pt 를 주거나
        #   그 기계에서 다시 export 할 것.
        self.declare_parameter('tl_weights', TL_WEIGHTS)
        self.declare_parameter('tl_conf',      0.35)
        # ★[2026-08-24] 640 → 960★ ROI 종횡비가 작은 신호등을 통째로 뭉갠다.
        #   ultralytics 는 ROI 를 ★긴 변 기준★ 으로 imgsz 에 레터박스한다. tl_roi 가
        #   1920x600 이므로 640 이면 배율이 640/1920 = 0.333 이고, 화면에서 240x80px
        #   인 신호등이 추론 텐서에서는 ★80x27px★ 이 된다 — 검출기가 볼 수 없는 크기다.
        #
        #   night_a 영상에 목업(240x80 @ 880,140)을 합성해 65프레임을 실측한 값:
        #     imgsz  추론시 목업   RED 검출률(conf≥0.35)   conf 중앙   1프레임 추론
        #       640     80x27            9.2%              0.133        5.0 ms
        #      ★960★   120x40          100.0%             0.832        7.2 ms
        #      1280    160x53          100.0%             0.831        8.6 ms
        #   → ★2.2 ms 로 9.2% → 100%★ 다. 30fps 예산(33.3ms)에 여유가 충분하고
        #     화각을 하나도 안 버리므로 tl_roi 를 좁히는 것보다 이쪽이 낫다.
        #
        #   ⚠️ 이것은 목업만의 문제가 아니다 — ★멀리 있는 진짜 신호등에 똑같이 걸린다★.
        #      tl_hold_s(연속 RED 0.4s)를 못 채우면 신호등에서 아예 안 서기 때문에,
        #      검출률이 낮으면 그 앞단이 아무리 맞아도 정지가 성립하지 않는다.
        #   ⚠️ tl_roi 를 바꾸면 이 값도 다시 봐야 한다(배율 = imgsz / ROI 긴 변).
        self.declare_parameter('tl_imgsz',     960)
        # 몇 프레임에 한 번 추론할 것인가. ★1 을 유지할 것★ — 2 로 올리면 판정 기회가
        # 절반이 되어 tl_hold_s(연속 RED)를 채우지 못하고 신호등에서 안 선다.
        self.declare_parameter('tl_interval',  1)
        # 신호등 탐색 ROI(원본 픽셀). 기본은 1920x1080 카메라의 위쪽 600행.
        # ★[2026-08-24] 540 → 600★ cam_testbed 캘리브 한 벌에 맞췄다(tl_roi 0~600 /
        # 노면 ROI 600~1080 으로 화면을 위아래로 나눈 값이다).
        # 프레임이 더 작으면 _clamp_roi 가 알아서 프레임 크기로 잘라 준다.
        self.declare_parameter('tl_roi_xmin',  0)
        self.declare_parameter('tl_roi_ymin',  0)
        self.declare_parameter('tl_roi_xmax',  1920)
        self.declare_parameter('tl_roi_ymax',  600)
        self.declare_parameter('tl_min_area',  20)
        # 상한은 사실상 해제 상태다 — 근접하면 박스가 커지는데 상한에 걸리면
        # ★정지해야 할 바로 그 순간 신호등이 UNKNOWN 으로 사라진다★.
        self.declare_parameter('tl_max_area',  1000000)
        # ★야간 오검출 방어 [2026-08-25]★ 박스 높이가 ROI 높이의 이 비율을 넘으면
        #   버린다. 신호등이 ★한 번도 안 나오는★ 야간 도로에서 ROI 를 통째로 덮는
        #   박스가 conf 0.36~0.88 로 나온다(cam_testbed 실측 26개: 높이 510~592px =
        #   ROI(600)의 85~99%). 그게 RED 로 읽히면 근접도 게이트를 그대로 넘겨
        #   ★신호등이 없는데 「코앞이다」로 급정지★ 한다.
        #     · 면적(tl_max_area)으로 자르지 않는 이유 — 실제 신호등 최대(추정
        #       13.5만px²)와 오검출 최소(26만px²)가 1.9배밖에 안 벌어진다. 잘못
        #       자르면 ★빨간불에 안 서는★ 쪽으로 고장 나므로 여유가 그만큼 필요하다.
        #     · 높이는 3.4배 벌어진다 — 이 노드가 '정지'로 치는 박스는 25~40px
        #       (tl_red_stop_min_height)이고, 가장 가까울 때도 100px 대다. 게다가
        #       코앞에서는 등기구가 화면 위로 벗어나 ★오히려 잘린다★(파일 헤더).
        #   0 이면 끈다. ROI 높이 기준이라 ROI 를 바꿔도 뜻이 유지된다.
        self.declare_parameter('tl_max_height_frac', 0.5)
        self.declare_parameter('tl_min_aspect', 0.2)
        # 가로형 4구 신호등은 bw/bh ≈ 3.5~4.5 이고, ROI 상단에 잘리면 더 커진다.
        self.declare_parameter('tl_max_aspect', 6.0)
        # RED(정지 대상) 와 RED_FAR(아직 멀다) 를 가르는 근접도 게이트.
        #   ▸ tl_red_stop_min_area_frac > 0 → 박스 면적비 기준(해상도·화각 독립)
        #   ▸ 0(기본)                      → 박스 높이(px) 기준
        # 면적비는 항상 계산·표시되므로 실주행 화면으로 값을 정한 뒤 파라미터만 주면
        # 코드 수정 없이 기준이 바뀐다. 정하는 법 — 원거리 오탐의 최댓값과 실제 정지
        # 지점의 최솟값을 읽어 그 기하평균.
        self.declare_parameter('tl_red_stop_min_height',    25)
        self.declare_parameter('tl_red_stop_min_area_frac', 0.0)
        # ★근접 게이트 히스테리시스 [2026-08-14]★ 이미 물고 있는 동안에는 임계를
        # 이 비율로 낮춰서 본다. 같은 신호등이 인식 흔들림으로 몇 px 작아졌다고
        # RED→RED_FAR 로 떨어지면, 그 순간 해제 타이머가 돌기 시작해 리니어가 왕복한다
        # (물리적 왕복이 이 장치에서 제일 나쁘다 — 정지 래치 절 참고).
        # 무는 임계는 그대로 두고 ★놓는 임계만★ 낮추는 것이라, 반응이 늦어지지 않는다.
        # 1.0 = 히스테리시스 없음(종전 동작). 면적비 모드에서는 면적 기준 비율이다.
        self.declare_parameter('tl_near_release_ratio',     0.7)
        self.declare_parameter('use_hsv_refine',      True)
        # RED 박스는 HSV 적색 픽셀 확인을 통과해야 채택한다(오탐 관문).
        # 붉지 않은 것이 RED 로 나가면 도로 한복판에서 리니어 2단이 물린다.
        # ⚠️ 이 관문의 실패 모드는 '진짜 RED 를 버리는 것'이다 — 현장에서 RED 가 안
        #    잡히면 이 값을 false 로 내려 먼저 격리하고 로그의 drop 수를 볼 것.
        self.declare_parameter('tl_red_require_hsv',  True)
        self.declare_parameter('hsv_min_color_pixels', 15)
        self.declare_parameter('hsv_crop_center_ratio', 0.85)
        self.declare_parameter('hsv_red_h1_low',   0)
        self.declare_parameter('hsv_red_h1_high',  10)
        self.declare_parameter('hsv_red_h2_low',   170)
        self.declare_parameter('hsv_red_h2_high',  180)
        self.declare_parameter('hsv_green_h_low',  45)
        self.declare_parameter('hsv_green_h_high', 90)
        # 실측 기반 하향값(80/70 → 55/60). 실제 신호등 크롭에서 관찰된 최저가
        # S=62, V=66 이었다 — 그보다 낮게 잡아야 RED 가 검출된다.
        self.declare_parameter('hsv_sat_low',      55)
        self.declare_parameter('hsv_val_low',      60)

        # ── 판단(camera_judgment.py RED 확정 필터에서 이식) ───ㄴ──────────────
        self.declare_parameter('tl_hold_s',        0.4)   # 이만큼 연속으로 봐야 확정
        self.declare_parameter('tl_gap_grace_s',   0.3)   # 이 이내의 끊김은 봐준다
        self.declare_parameter('tl_state_max_age', 3.0)   # 이보다 낡은 판정은 무시
        self.declare_parameter('green_hold_s',     0.4)   # 재출발용 GREEN 확정 시간
        # ★해제 유예 [2026-08-14]★ 빨간불을 이만큼 못 봐야 놓는다. 무는 쪽(tl_hold_s)은
        #   그대로 두고 놓는 쪽만 늦춘 것이다 — 인지가 한두 번 흔들릴 때 리니어가
        #   들락날락하던 실차 증상을 막는다(_red_gone 주석에 근거).
        #   ★[2026-08-14 조정] 1.0 → 0.5★ 체감 해제가 너무 늦었다. 전체 해제 지연은
        #   ★여기 0.5초 + arduino 의 마지막 유예 0.5초 = 1.0초★ 로 맞춘다.
        #   (그 배분을 왜 이렇게 나눴는지는 arduino.BRAKE_RELEASE_HOLD_S 주석에 있다)
        self.declare_parameter('red_release_hold_s', 0.5)

        # ── 정지선 (perception.py 의 stop-line 절에서 이식) ─────────────────
        #   ★이 절이 하는 일은 '정지를 미루는 것' 하나뿐이다★ RED 확정 조건은 위
        #   그대로이고, 정지선이 보이는 동안에만 '아직 멀다 → 참는다'가 끼어든다.
        #   못 보면 종전 동작(즉시 정지)으로 그대로 되돌아간다 — 파일 헤더의 규칙표.
        self.declare_parameter('sl_enable',  True)
        self.declare_parameter('sl_weights', SL_WEIGHTS)
        self.declare_parameter('sl_conf',    0.30)   # 구 white lane_conf(0.3)와 같은 값
        self.declare_parameter('sl_imgsz',   640)
        # 몇 프레임에 한 번 정지선을 볼 것인가.
        # ★1 을 기본으로 두는 근거는 실측이다 [2026-08-14]★ 이 PC(1920x1080, cuda:0)에서
        #   신호등만  4.3 ms/frame
        #   +정지선  7.5 ms/frame  (정지선이 더하는 몫 = ★3.1 ms★)
        # 카메라 30fps 의 프레임 예산이 33.3 ms 이므로 매 프레임 돌려도 남는다.
        # 느린 기계로 옮겨 fps 가 떨어지면 2 로 올린다 — 그때도 sl_hold_s(0.2s) 안에
        # 3 프레임은 들어오므로 확정이 무너지지 않는다.
        self.declare_parameter('sl_interval', 1)
        # ★두 개의 문턱 — 이 기능의 핵심 숫자다 [2026-08-19]★
        #   판정값은 ★BEV 에서 정지선 최근접점 → 앞범퍼까지의 픽셀 거리★ 하나다
        #   (camera_model.nearest_dist_px). 그 값이
        #     sl_brake1_px 이하 → 1단 예비제동   (부드럽게 줄이기 시작한다)
        #     sl_brake2_px 이하 → 2단 확정 정지  (정지선 앞에 세운다)
        #   이므로 반드시 sl_brake1_px > sl_brake2_px 다(생성자에서 강제한다).
        # ★[2026-08-25 실측] 240/60 → 470/300★ 종전 값은 근거 없는 기본값이었고,
        #   ★둘 다 물리적으로 도달 불가★ 였다. bev_bumper_y_px 를 645 로 실측하면서
        #   sl_px 의 도달 범위 자체가 달라졌다(camera_model.py 의 그 주석 참고):
        #     · BEV 밑변이 sl_px=165 → ★sl_px 는 165 밑으로 못 내려간다★ (60 은 불가)
        #     · 차체(앞 롤바·앞바퀴)가 BEV 행 400 아래를 가려 마스크가 두 동강 난다
        #       → ★검출 실효 하한 sl_px ≈ 285★ (240 도 사실상 불가)
        #   night_a 를 imgsz=960 으로 다시 돌려 실측한 검출 구간이 그대로 이것이다:
        #       sl_px 580 → 285 (범퍼 앞 5.3 m → 2.2 m) 에서만 정지선이 보인다
        #
        #   ── 정한 근거 (접근속도 실측 1.92 m/s) ────────────────────────────
        #     · 1단 정지거리 = 1.92×0.55(램프) + 1.92²/2.6 = ★2.47 m★
        #     · 2단 정지거리 = 1.92²/(2×2.2~3.8) = 0.48~0.84 m   (BRAKING.md 4절)
        #     · sl_brake1_px=470 → 범퍼 앞 ★4.03 m★  2.47 m 에 1.5 m 여유
        #     · sl_brake2_px=300 → 범퍼 앞 ★2.34 m★  검출 하한(2.2 m) 직전에 문다
        #
        #   ⚠️ ★이 카메라 마운트로는 정지선 앞 1.5~2 m 에 서는 것이 한계다★ 2.2 m 보다
        #      가까워지면 차체가 정지선을 가려 아무것도 안 보이므로, 그 전에 2단을
        #      확정할 수밖에 없다. 더 붙여 세우고 싶으면 문턱이 아니라 ★카메라를 올리거나
        #      아래로 틸트★ 해야 한다 — 가림 경계가 내려가면 하한이 같이 내려간다.
        #   ⚠️ 300 을 더 낮추면 검출이 먼저 끊겨 '정지선 놓침' 경로로 떨어진다. 그 경로는
        #      sl_stale_s(0.5초)를 기다리므로 ★1.92 m/s 에서 0.96 m 를 더 간다★ —
        #      늦게 서려다 오히려 정지선을 밟는다.
        #   ★사다리꼴이나 bev_bumper_y_px 를 바꾸면 이 두 값도 같이 무효다★
        self.declare_parameter('sl_brake1_px', 470.0)
        self.declare_parameter('sl_brake2_px', 300.0)
        # 노면 잡티·차선 조각을 정지선으로 읽지 않기 위한 관문. 정지선은 ★가로로 긴★
        # 물체다 — 폭이 화면의 이 비율보다 좁으면 버린다.
        self.declare_parameter('sl_min_width_frac', 0.12)
        self.declare_parameter('sl_min_area_frac',  0.0004)
        # 확정 시간. 한두 프레임 스친 마스크로 브레이크를 미루지 않는다.
        self.declare_parameter('sl_hold_s',  0.2)
        # 이보다 오래된 관측은 '없다'로 본다 = ★놓쳤다 → 즉시 정지★ 로 넘어가는 문턱.
        self.declare_parameter('sl_stale_s', 0.5)
        # ★대기 상한 ①★ RED 확정 시각으로부터 이만큼 지나면 정지선을 무시하고 2단.
        #   오검출된 먼 정지선이 2단을 무한정 미루는 것을 막는 마지막 방어선이고,
        #   ★1단으로 감속하다 정지선 앞에서 멈춰 버린 경우★ 도 여기서 받는다
        #   (1단이면 구동펄스가 0 이라 스스로 기어가지 못한다 — 파일 헤더 참고).
        #   ★[2026-08-19] 5 → 8초★ 종전의 '참는 동안'은 무개입 코스트였지만 이제는
        #   1단으로 이미 감속 중이라, 늘려도 위험이 늘지 않는다.
        self.declare_parameter('sl_wait_max_s', 8.0)
        # ★대기 상한 ②★ 근접도(_near_metric)가 게이트의 이 배를 넘으면 = 신호등이
        #   코앞이면 정지선을 기다리지 않는다. 시간보다 나은 물리적 상한이다.
        self.declare_parameter('sl_override_gate_ratio', 1.6)
        # 빨간 박스를 이 시간 안에 본 적이 있을 때만 정지선 추론을 돌린다(성능).
        self.declare_parameter('sl_gate_red_s', 1.0)

        # ── 정지 동작 ──────────────────────────────────────────────────────
        self.declare_parameter('brake_level',         2)     # 0 놓음 / 1 약 / 2 풀
        # ★예비제동 단계 [2026-08-19]★ 정지선까지 sl_brake1_px 안으로 들어왔을 때
        #   무는 단계다. 0 으로 두면 예비제동을 끄는 것과 같다(= 종전처럼 참았다가 2단).
        self.declare_parameter('brake_level_pre',     1)
        self.declare_parameter('brake_release_level', 0)
        # ★[2026-08-14] publish_cmd_vel 은 true 로 둔다(지시)★ 정지 중 펄스 0·조향 0 을
        #   함께 낸다. 캐시를 덮는 부작용은 남지만, master 도 driving 도 자기 명령을
        #   주기적으로 재발행하므로(master KEEPALIVE_S=0.5s / driving 20Hz) 해제 뒤
        #   ★0.5초 안에 원래 명령값이 되돌아온다★ — 실질 손해가 작다.
        #   조향 다툼이 싫으면 false 로 끄면 되고, 그때도 정지는 그대로 성립한다.
        self.declare_parameter('publish_cmd_vel',     True)
        self.declare_parameter('stop_cmd_hz',         30.0)  # 판단 틱 주기
        self.declare_parameter('stop_latch',          False)
        # 개입 허락 : /drive_state==DRIVE_RUN 또는 /tl_enable(master 체크박스)
        self.declare_parameter('require_permission',  True)

        # ── 표시 ───────────────────────────────────────────────────────────
        #  ★창은 기본으로 띄운다 [2026-08-14]★ 신호등 인지는 '지금 뭘 보고 있나'를
        #  눈으로 확인하지 않으면 튜닝이 불가능하다(ROI·근접도·색 임계). 화면 없는
        #  터미널(ssh)에서는 cv2 가 창을 못 열므로 show_window:=false 로 끈다.
        self.declare_parameter('show_window', True)
        self.declare_parameter('draw_roi',    True)
        #  ★뷰 가로폭 [2026-08-14 도입 · 2026-08-24 뜻이 바뀜]★ 원본(1920)을 그대로
        #  띄우면 화면을 덮는다. 이 폭으로 줄여서 띄운다(비율 유지). 0 이면 원본 크기.
        #  판정은 원본 해상도로 하므로 이 값은 ★보이는 크기만★ 바꾼다.
        #  ⚠️ ★이제 이 값은 '창 폭' 이 아니라 '카메라 뷰 폭' 이다★ 우측 패널과 하단
        #    HUD 가 밖에 붙으므로 창은 이보다 크다(960 → 창 1248x610).
        #  ★기본값을 640 → 960 으로 올렸다 [2026-08-24]★ 두 가지 이유다:
        #    ① 960 은 1920 의 ★정확히 절반★ 이라 INTER_AREA 리사이즈가 0.28ms 인데,
        #       640(임의 배율)은 같은 보간으로 2.50ms 다 — 줄일수록 비싸진다(실측).
        #    ② 640 이면 HUD 글자를 그만큼 작게 잡아야 해서 판독성이 다시 나빠진다.
        self.declare_parameter('window_width', 960)
        #  ★BEV 패널 [2026-08-19 도입 · 2026-08-24 겹치기→패널]★ 두 문턱
        #  (sl_brake1_px·sl_brake2_px)과 범퍼선이 거기 있어서, ★이 그림 없이는 숫자를
        #  잡을 수 없다★. 창을 따로 띄우지 않는 이유는 _draw 주석의 HighGUI 문제다.
        #  ⚠️ ★그림 위에 겹치지 않고 오른쪽에 패널로 붙는다★ 종전에는 우하단에 겹쳐
        #    그려서 ★정지선이 실제로 보이는 노면★ 을 가렸고, 축소를 함께 받아 문턱
        #    라벨이 실효 0.17 크기가 됐다. 패널에는 게이지와 파라미터 요약도 같이 든다.
        self.declare_parameter('show_bev', True)
        #  ★ROI 밖을 얼마나 어둡게 볼지 [2026-08-24]★ 종전에는 노란 사각형이었는데
        #  BEV 사다리꼴도 거의 같은 노란색이라 둘이 구별되지 않았다. 밖을 어둡게 하면
        #  색을 하나 쓰지 않고 ROI 를 말할 수 있다. 1.0 = 안 어둡게(테두리만).
        #  ⚠️ ★너무 낮추지 말 것★ 정지선은 ROI 밖(노면)에 있다 — 0.3 쯤으로 내리면
        #    정지선 마스크가 맞는지 눈으로 못 본다. 0.6 이 둘을 다 얻는 값이다.
        self.declare_parameter('roi_dim', 0.6)
        #  ★HUD 한글 폰트 [2026-08-24]★ 빈 문자열이면 FONT_CANDIDATES 를 순서대로
        #  찾는다(1순위 NanumGothicCoding — ★고정폭★ 이라 HUD 열이 맞는다).
        #  못 찾으면 경고 한 줄 남기고 cv2.putText 로 물러난다 — 한글만 물음표가 되고
        #  창·숫자·정지 동작은 그대로다(fail-open).
        self.declare_parameter('hud_font', '')

        # ══════════════════════════════════════════════════════════════════
        #  ★시험용 신호등 주입 [2026-08-25] — 실차에서는 반드시 0 이어야 한다★
        # ══════════════════════════════════════════════════════════════════
        #  0 보다 크면 ★그 높이(px)의 빨간 박스를 봤다고 친다★. YOLO 결과를
        #  통째로 대신하므로 이 값이 켜져 있는 동안 신호등 인지는 ★시험되지 않는다★.
        #
        #  ★왜 필요한가★ 정지선 절(節)은 빨간 박스를 본 적이 있어야만 돈다
        #  (_sl_should_run). 그런데 신호등이 안 찍힌 영상으로 정지선을 시험하려면
        #  그 게이트를 열 방법이 있어야 한다. 종전에는 화면에 신호등 그림을 합성해서
        #  풀었는데, ★박스 높이가 거리의 대리값★ 인 이 노드에서는 그 방법이 성립하지
        #  않는다 — 합성 그림이 검출되려면 480px 폭이 필요했고 그때 박스 높이가
        #  152~166px 로 나와서, 시험 대상인 tl_red_stop_min_height(25) 를 목업에 맞춰
        #  옮겨야 했다. ★시험 대상 상수를 옮기면 그 시험은 그 상수를 검증하지 않는다.★
        #  여기서 높이를 직접 주면 그 모순이 없어진다: 15 = RED_FAR, 40 = RED.
        #
        #  ★백도어 방어★ 켜져 있으면 기동 배너와 status_tick 이 계속 경고를 찍는다.
        #  cam_testbed 의 계약이 그 로그를 log_events 로 잡아 실차 시나리오에서
        #  `max: 0` 으로 막는다 — 켠 채 실차에 나가면 시험이 불합격한다.
        self.declare_parameter('tl_fake_box_h', 0.0)

        # ── 카메라 기하 (어안 왜곡보정·BEV) ─────────────────────────────────
        #  ★파라미터 이름·기본값의 주인은 camera_model.py 다★ 차선 인지가 붙어도
        #  같은 이름을 쓰게 하려고 선언을 그쪽에 두었다(camera_launch.camera_params()
        #  한 벌을 두 노드에 그대로 먹일 수 있다).
        camera_model.declare_params(self)

        g = lambda k: self.get_parameter(k).value
        self.image_topic = str(g('image_topic'))
        self.device      = str(g('device'))
        self.tl_conf     = float(g('tl_conf'))
        self.tl_imgsz    = int(g('tl_imgsz'))
        self.tl_interval = max(1, int(g('tl_interval')))
        self.tl_roi = (int(g('tl_roi_xmin')), int(g('tl_roi_ymin')),
                       int(g('tl_roi_xmax')), int(g('tl_roi_ymax')))
        self.tl_min_area   = int(g('tl_min_area'))
        self.tl_max_area   = int(g('tl_max_area'))
        self.tl_max_h_frac = max(0.0, float(g('tl_max_height_frac')))
        #  시험용 주입. /tl/fake_box_h 가 오면 그쪽이 이긴다(런 중에 바꾸기 위해서다).
        self.fake_box_h = max(0.0, float(g('tl_fake_box_h')))
        self.tl_min_aspect = float(g('tl_min_aspect'))
        self.tl_max_aspect = float(g('tl_max_aspect'))
        self.tl_red_stop_min_height    = int(g('tl_red_stop_min_height'))
        self.tl_red_stop_min_area_frac = float(g('tl_red_stop_min_area_frac'))
        self.tl_near_release_ratio = min(1.0, max(0.1, float(g('tl_near_release_ratio'))))
        self.use_hsv_refine       = bool(g('use_hsv_refine'))
        self.tl_red_require_hsv   = bool(g('tl_red_require_hsv'))
        self.hsv_min_color_pixels = int(g('hsv_min_color_pixels'))
        self.hsv_crop_center_ratio = float(g('hsv_crop_center_ratio'))
        self.hsv_red_h1_low   = int(g('hsv_red_h1_low'))
        self.hsv_red_h1_high  = int(g('hsv_red_h1_high'))
        self.hsv_red_h2_low   = int(g('hsv_red_h2_low'))
        self.hsv_red_h2_high  = int(g('hsv_red_h2_high'))
        self.hsv_green_h_low  = int(g('hsv_green_h_low'))
        self.hsv_green_h_high = int(g('hsv_green_h_high'))
        self.hsv_sat_low = int(g('hsv_sat_low'))
        self.hsv_val_low = int(g('hsv_val_low'))
        self.tl_hold_s        = float(g('tl_hold_s'))
        self.tl_gap_grace_s   = float(g('tl_gap_grace_s'))
        self.tl_state_max_age = float(g('tl_state_max_age'))
        self.green_hold_s     = float(g('green_hold_s'))
        self.red_release_hold_s = max(0.0, float(g('red_release_hold_s')))
        self.sl_enable  = bool(g('sl_enable'))
        self.sl_conf    = float(g('sl_conf'))
        self.sl_imgsz   = int(g('sl_imgsz'))
        self.sl_interval = max(1, int(g('sl_interval')))
        # ★두 문턱은 반드시 1단 > 2단★ 뒤집힌 값을 주면 '멀리서 2단, 가까이서 1단'이
        #   되어 판정이 통째로 뒤집힌다. 조용히 고치지 말고 경고를 남기고 바로잡는다.
        self.sl_brake1_px = max(0.0, float(g('sl_brake1_px')))
        self.sl_brake2_px = max(0.0, float(g('sl_brake2_px')))
        if self.sl_brake1_px < self.sl_brake2_px:
            self.get_logger().warn(
                f"sl_brake1_px({self.sl_brake1_px:.0f}) < sl_brake2_px"
                f"({self.sl_brake2_px:.0f}) — 1단 문턱이 2단보다 가깝다. "
                "1단 문턱을 2단과 같게 맞춘다(= 예비제동 없이 바로 2단)")
            self.sl_brake1_px = self.sl_brake2_px
        self.sl_min_width_frac = float(g('sl_min_width_frac'))
        self.sl_min_area_frac  = float(g('sl_min_area_frac'))
        self.sl_hold_s   = max(0.0, float(g('sl_hold_s')))
        self.sl_stale_s  = max(0.05, float(g('sl_stale_s')))
        self.sl_wait_max_s = max(0.0, float(g('sl_wait_max_s')))
        self.sl_override_gate_ratio = max(1.0, float(g('sl_override_gate_ratio')))
        self.sl_gate_red_s = max(0.0, float(g('sl_gate_red_s')))
        self.brake_level         = max(0, min(2, int(g('brake_level'))))
        self.brake_level_pre     = max(0, min(2, int(g('brake_level_pre'))))
        self.brake_release_level = max(0, min(2, int(g('brake_release_level'))))
        self.publish_cmd_vel  = bool(g('publish_cmd_vel'))
        self.stop_cmd_hz      = max(1.0, float(g('stop_cmd_hz')))
        self.stop_latch       = bool(g('stop_latch'))
        self.require_permission = bool(g('require_permission'))
        self.show_window = bool(g('show_window'))
        self.draw_roi    = bool(g('draw_roi'))
        self.window_width = max(0, int(g('window_width')))
        self.show_bev    = bool(g('show_bev')) and self.show_window
        self.roi_dim     = min(1.0, max(0.0, float(g('roi_dim'))))
        # ── 카메라 기하 — 보정·BEV 의 소유자(camera_model.py) ──────────────
        #   ★파라미터를 다 읽은 뒤에 만든다★ 로드 실패는 여기서 경고로 끝나고
        #   (fail-open) 보정만 꺼진다 — 노드는 종전대로 돈다.
        self.cam = camera_model.CameraModel.from_node(self)
        # 창 표시는 ★메인 스레드★ 가 한다(show_pending) — _draw 주석에 근거.
        self._show_lock = threading.Lock()
        self._show_frame = None
        self._window_ready = False
        # ── 그리기 도구 [2026-08-24] ────────────────────────────────────────
        #   ★전부 캐시다★ 매 프레임 새로 만드는 것이 하나도 없어야 8ms → 2.4ms 가 된다.
        self.tr = TextRenderer(log=self.get_logger() if self.show_window else None,
                               font_path=str(g('hud_font')))
        self._pool = CanvasPool()          # 표시 캔버스 두 장(더블버퍼)
        self._hud_strip = StripCache(self.tr)   # 하단 HUD — 글자가 바뀔 때만 다시 그린다
        self._par_strip = None             # 패널의 파라미터 요약 — 정적이라 한 번만
        self._bev_m = None                 # 썸네일 크기로 바로 펴는 호모그래피
        self._bev_m_key = None

        # ── 상태 ───────────────────────────────────────────────────────────
        self.tl_state   = 'UNKNOWN'   # 마지막 프레임 판정
        self.tl_time    = 0.0         # 그 판정 시각(신선도 판단용)
        self.red_since  = None        # 연속 RED 스트릭의 시작 시각
        # ★근접 RED 만 기록한다★ RED_FAR 는 여기 손대지 않는다(_feed_state 주석 참고)
        self.red_last_seen  = None    # 마지막으로 RED(근접) 를 본 시각 = 해제 판정 근거
        self.green_since    = None
        self.green_last_seen = None
        # ★[2026-08-19] bool → 단계(0/1/2)★ 정지선이 보이면 1단 예비제동을 거쳐
        #   2단으로 간다. 이 값은 ★해제 경로에서만 0 으로 돌아간다★(단조 증가 규약).
        self.stop_level = 0
        self.stop_why   = ''          # 지금 단계를 고른 근거(로그·HUD 용)
        self._brake_t   = 0.0         # 브레이크를 마지막으로 발행한 시각(재확인용)
        # 우리가 마지막으로 ★발행한★ 브레이크 단계. None = 한 번도 건 적 없다.
        #   → 이 값이 None 이면 0단도 내지 않는다(남의 브레이크를 풀지 않기 위해).
        self.brake_now  = None
        self.drive_state   = ''
        self.drive_state_t = 0.0
        self.tl_enable     = False    # master 의 '신호등 인지' 체크박스
        self.tl_enable_t   = 0.0
        self.tl_permit     = False    # driving 의 허락(TRAFFIC_LIGHT_ENABLE + DRIVE_RUN)
        self.tl_permit_t   = 0.0
        self.img_time      = 0.0
        self.frame_count   = 0
        self.last_boxes    = []
        self.last_raw      = 0        # 필터 전 박스 수 (-1 = 추론 예외)
        self.last_red_drop = 0        # HSV 적색 관문에 걸려 버려진 RED 수
        self.fps           = 0.0
        self.fps_t         = time.monotonic()

        # ── 정지선 상태 ────────────────────────────────────────────────────
        #   ★판정값은 sl_px 하나다★ sl_y 는 기록·HUD 용으로만 남는다(파일 헤더).
        self.sl_px      = -1.0        # BEV 에서 정지선 최근접점 → 앞범퍼 [px]. −1 = 미검출
        self.sl_y       = -1.0        # 마지막 관측(최하단 y / 프레임 높이). −1 = 미검출
        self.sl_seen_t  = 0.0         # 마지막으로 정지선을 본 시각 = 신선도의 근거
        self.sl_since   = None        # 연속 목격 스트릭의 시작 시각(확정 판정용)
        self.sl_engaged = False       # 이번 접근에서 정지선을 ★확정한 적★ 이 있는가
        self.sl_wait    = False       # 지금 정지선 때문에 브레이크를 참고 있는가(아직 0단)
        self.sl_poly    = None        # HUD 용 마스크 폴리곤(보정된 원본 좌표)
        self.sl_poly_bev = None       # 같은 폴리곤의 BEV 좌표(HUD 용)
        self.sl_frames  = 0           # sl_interval 카운터
        self.red_seen_t = 0.0         # 마지막으로 ★빨간 박스★ 를 본 시각(추론 게이팅)
        self.red_conf_t = None        # RED 확정이 시작된 시각(대기 상한의 기준점)

        # ── 모델 ───────────────────────────────────────────────────────────
        self.model = None
        self.TL_LABEL = {}
        self.TL_ALLOW = set()
        self._load_model(str(g('tl_weights')))
        self.sl_model = None
        self.sl_class_id = SL_CLASS_FALLBACK
        if self.sl_enable:
            self._load_sl_model(str(g('sl_weights')))

        # ── ROS 인터페이스 ─────────────────────────────────────────────────
        qos_img = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self.bridge    = CvBridge()
        self.pub_brake = self.create_publisher(Int32,  '/brake_level', qos)
        # ★[2026-08-14] 내가 지금 요구하는 브레이크 단계를 따로 알린다★
        #   /brake_level 은 '명령'이라 마지막 발행자가 이긴다. master 는 자기 레버값을
        #   KEEPALIVE_S(0.5s)마다 재발행하므로, 신호등이 2단을 걸어도 0.5초 뒤 0 으로
        #   덮여 리니어가 나왔다 들어간다(실측 로그로 확인). 그래서 '요구'를 별도로
        #   내보내고 master 가 그것을 자기 값과 max 로 합쳐서 낸다 — 두 발행자가
        #   ★같은 값★ 을 내게 되어 다툼이 사라진다.
        self.pub_req   = self.create_publisher(Int32,  '/tl_brake_req', qos)
        self.pub_cmd   = self.create_publisher(Twist,  '/cmd_vel_raw', qos)
        self.pub_state = self.create_publisher(String, '/tl/state',    qos)
        # ★진단 전용 [2026-08-14]★ 제어에는 쓰지 않는다 — record 가 CSV 로 받아 적어
        #   '왜 여기서 섰나 / 왜 멀리서도 섰나' 를 사후에 판정할 수 있게 한다.
        self.pub_near    = self.create_publisher(Float32, '/tl/near_metric', qos)
        self.pub_red_far = self.create_publisher(Bool,    '/tl/red_far',     qos)
        # ★정지선도 같은 태도다 — 제어가 아니라 기록이다★ 'RED 인데 왜 아직 안 섰나'
        #   (=정지선을 기다리는 중)를 CSV 로 남겨야 사후에 정지 지점을 판정할 수 있다.
        self.pub_sl_y    = self.create_publisher(Float32, '/tl/stop_line_y',    qos)
        self.pub_sl_wait = self.create_publisher(Bool,    '/tl/stop_line_wait', qos)
        # ★[2026-08-19] 판정에 실제로 쓰는 값★ 위 두 개와 달리 이것이 문턱과 비교되는
        #   숫자다. CSV 에 남아야 '왜 여기서 1단을 물었나'를 사후에 따질 수 있다.
        self.pub_sl_px   = self.create_publisher(Float32, '/tl/stop_line_px',   qos)

        # ★콜백 그룹을 둘로 나눈다 [2026-08-14]★ 영상(추론·그리기)과 제어(브레이크
        #   유지·해제, 요구 발행)를 다른 그룹에 두고 MultiThreadedExecutor 로 돌린다.
        #   한 프레임 처리가 길어져도 제어 틱이 그만큼 밀리지 않는다 — main() 참고.
        self.cg_image = MutuallyExclusiveCallbackGroup()
        self.cg_ctrl  = MutuallyExclusiveCallbackGroup()

        self.create_subscription(Image, self.image_topic, self.cb_image, qos_img,
                                 callback_group=self.cg_image)
        self.create_subscription(String, '/drive_state', self.cb_drive_state, qos,
                                 callback_group=self.cg_ctrl)
        # 허락 두 갈래. 안 오면 False = 허락 없음(fail-safe 방향)
        #   /tl_enable : master 의 '신호등 인지' 체크박스 — ★체크되어 있으면 상시★
        #   /tl_permit : driving 의 TRAFFIC_LIGHT_ENABLE + DRIVE_RUN — ★자율주행 중에만★
        self.create_subscription(Bool, '/tl_enable', self.cb_tl_enable, qos,
                                 callback_group=self.cg_ctrl)
        self.create_subscription(Bool, '/tl_permit', self.cb_tl_permit, qos,
                                 callback_group=self.cg_ctrl)
        # ★시험용 주입★ 파라미터를 런 도중에 바꾸기 위한 통로다(위 tl_fake_box_h 주석).
        #   실차에서는 아무도 발행하지 않으므로 구독만 있어도 거동이 안 바뀐다.
        self.create_subscription(Float32, '/tl/fake_box_h', self.cb_fake_box_h, qos,
                                 callback_group=self.cg_ctrl)

        # 정지 지령 타이머 — ★영상 콜백이 아니라 여기서 낸다★ 추론이 늦어져
        # 프레임이 띄엄띄엄 들어와도 정지 지령의 주기는 흔들리면 안 된다.
        self.create_timer(1.0 / self.stop_cmd_hz, self.tick,
                          callback_group=self.cg_ctrl)
        self.create_timer(1.0, self.status_tick, callback_group=self.cg_ctrl)

        self.get_logger().info(
            f"🚦 traffic_light | img={self.image_topic} dev={self.device} "
            f"conf={self.tl_conf} interval={self.tl_interval} "
            f"ROI={self.tl_roi} | 근접도 게이트="
            + (f"면적비≥{100.0 * self.tl_red_stop_min_area_frac:.3f}%"
               if self.tl_red_stop_min_area_frac > 0.0
               else f"박스높이≥{self.tl_red_stop_min_height}px")
            + f"(물고 있는 동안 ×{self.tl_near_release_ratio:.2f})"
            + f" | 확정 hold={self.tl_hold_s}s grace={self.tl_gap_grace_s}s "
            f"age≤{self.tl_state_max_age}s | 정지=리니어 {self.brake_level}단"
            + (f" + /cmd_vel_raw 0/0 @{self.stop_cmd_hz:.0f}Hz"
               if self.publish_cmd_vel else " (조향 개입 없음)")
            + f" | latch={'ON(GREEN 까지)' if self.stop_latch else 'OFF(빨간불 동안만)'} "
            + ("허락=/tl_enable(master 체크박스) 또는 /tl_permit(driving DRIVE_RUN)"
               if self.require_permission else "허락=★없음(벤치 모드)★")
            + (f"\n   🛑 정지선: ★BEV 픽셀 거리★ {self.brake_level_pre}단 ≤"
               f"{self.sl_brake1_px:.0f}px{self.cam.m_txt(self.sl_brake1_px)} → "
               f"{self.brake_level}단 ≤{self.sl_brake2_px:.0f}px"
               f"{self.cam.m_txt(self.sl_brake2_px)} | "
               f"conf={self.sl_conf} interval={self.sl_interval} "
               f"확정 {self.sl_hold_s}s / 신선도 {self.sl_stale_s}s | 대기 상한 "
               f"{self.sl_wait_max_s:.1f}s 또는 근접도 ×{self.sl_override_gate_ratio:.1f} "
               f"| ★정지선을 못 보면 종전대로 즉시 {self.brake_level}단★"
               if (self.sl_enable and self.sl_model is not None)
               else "\n   🛑 정지선: ★꺼져 있다★ — RED 확정이면 그 자리에서 선다(종전 동작)")
            + "\n   " + self.cam.describe()
            #  ★주입 모드★ 계약의 log_events 가 이 줄을 잡아 실차 시나리오에서 막는다.
            + (f"\n   🧪 ★신호등 주입 모드★ 박스높이 {self.fake_box_h:.0f}px 를 봤다고 "
               f"친다 — ★신호등 인지는 시험되지 않는다★ (실차 금지)"
               if self.fake_box_h > 0.0 else ""))

    def _load_model(self, weights):
        """가중치를 읽고 라벨 표를 세운다. 실패해도 노드는 살아 있는다(fail-open).

        ★라벨은 엔진의 클래스 이름에서 유도한다★ 하드코딩({0:GREEN, 1:RED})은
        가중치를 재학습·교체하는 순간 ★조용히 적/녹이 뒤집힌다★ — 그 실패 모드는
        '빨간불에 가속'이다. 아래 로그가 실제 로드된 클래스를 찍으므로 기동할 때
        반드시 눈으로 확인할 것.
        """
        try:
            self.model = YOLO(weights)
        except Exception as e:
            self.model = None
            self.get_logger().error(
                f"⛔ 신호등 가중치 로드 실패 — 이 노드는 아무것도 하지 않는다: {weights} ({e})")
            return

        try:
            names = self.model.names
            self.TL_LABEL = {int(k): str(v).upper() for k, v in
                             (names.items() if isinstance(names, dict) else enumerate(names))}
        except Exception as e:
            self.TL_LABEL = {}
            self.get_logger().error(f"신호등 엔진 클래스 이름 조회 실패: {e}")

        if not {'RED', 'GREEN'} <= set(self.TL_LABEL.values()):
            # 엔진에 메타데이터가 없으면 ultralytics 가 'class0/class1' 을 돌려준다.
            # 그대로 두면 RED/GREEN 이 영영 안 나와 ★신호등이 조용히 무력화★된다.
            self.get_logger().error(
                f"⛔ 엔진 클래스에 RED/GREEN 이 없다: {self.TL_LABEL} → "
                f"옛 하드코딩({{0:GREEN, 1:RED}})으로 폴백한다. "
                f"★클래스 순서가 이와 다르면 적/녹이 뒤집힌다 — 반드시 확인할 것★")
            self.TL_LABEL = {0: 'GREEN', 1: 'RED'}
        # 화이트리스트는 ★RED/GREEN 만★ 이다. set(TL_LABEL) 로 두면 모델의 전 클래스가
        # 통과해 필터가 no-op 이 된다.
        self.TL_ALLOW = {k for k, v in self.TL_LABEL.items() if v in ('RED', 'GREEN')}
        self.get_logger().info(f"신호등 엔진 클래스 = {self.TL_LABEL} (허용 {sorted(self.TL_ALLOW)})")

    def _load_sl_model(self, weights):
        """정지선 seg 가중치를 읽는다. ★실패해도 노드는 그대로 산다★ — 정지선이 없는
        상태 = 종전 동작(RED 확정이면 즉시 정지)이므로, 이 실패는 기능이 하나 빠지는
        것이지 차가 못 서는 것이 아니다(fail-open 의 방향이 신호등과 반대가 아니다).

        클래스는 ★이름에서 찾는다★ 하드코딩(2)은 가중치를 바꾸는 순간 조용히 다른
        클래스를 정지선으로 읽게 된다 — 그 실패 모드는 '엉뚱한 것 앞에서 선다'다.
        """
        try:
            self.sl_model = YOLO(weights, task='segment')
        except Exception as e:
            self.sl_model = None
            self.get_logger().error(
                f"⛔ 정지선 가중치 로드 실패 — 정지선 없이 종전 동작으로 간다: "
                f"{weights} ({e})")
            return

        names = {}
        try:
            n = self.sl_model.names
            names = {int(k): str(v) for k, v in
                     (n.items() if isinstance(n, dict) else enumerate(n))}
        except Exception as e:
            self.get_logger().error(f"정지선 엔진 클래스 이름 조회 실패: {e}")

        found = [k for k, v in names.items() if 'STOP' in v.upper()]
        if found:
            self.sl_class_id = found[0]
        else:
            self.sl_class_id = SL_CLASS_FALLBACK
            self.get_logger().error(
                f"⛔ 정지선 엔진 클래스에 'stop' 이 없다: {names} → "
                f"id={SL_CLASS_FALLBACK} 로 폴백한다. ★다른 클래스를 정지선으로 읽으면 "
                f"엉뚱한 곳에서 선다 — 반드시 확인할 것★")
        self.get_logger().info(
            f"정지선 엔진 클래스 = {names} (사용 id={self.sl_class_id})")

    # ══════════════════════════════════════════════════════════════════════════
    #  인지 — perception.py 이식
    # ══════════════════════════════════════════════════════════════════════════
    @staticmethod
    def _clamp_roi(w, h, xmin, ymin, xmax, ymax):
        xmin = max(0, min(int(xmin), w - 1))
        xmax = max(0, min(int(xmax), w))
        ymin = max(0, min(int(ymin), h - 1))
        ymax = max(0, min(int(ymax), h))
        if xmax <= xmin or ymax <= ymin:
            return 0, 0, w, h
        return xmin, ymin, xmax, ymax

    @staticmethod
    def _central_crop(img, ratio=0.8):
        if img is None or img.size == 0:
            return img
        h, w = img.shape[:2]
        ratio = max(0.2, min(1.0, ratio))
        nw, nh = int(w * ratio), int(h * ratio)
        x1, y1 = max(0, (w - nw) // 2), max(0, (h - nh) // 2)
        return img[y1:y1 + nh, x1:x1 + nw]

    def _hsv_state(self, crop_bgr):
        """박스 크롭의 적/녹 픽셀 수를 세어 색으로 판정한다."""
        if crop_bgr is None or crop_bgr.size == 0:
            return 'UNKNOWN', 0, 0
        crop = self._central_crop(crop_bgr, self.hsv_crop_center_ratio)
        hsv  = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)

        lo_r1 = np.array([self.hsv_red_h1_low,  self.hsv_sat_low, self.hsv_val_low], dtype=np.uint8)
        hi_r1 = np.array([self.hsv_red_h1_high, 255, 255], dtype=np.uint8)
        lo_r2 = np.array([self.hsv_red_h2_low,  self.hsv_sat_low, self.hsv_val_low], dtype=np.uint8)
        hi_r2 = np.array([self.hsv_red_h2_high, 255, 255], dtype=np.uint8)
        lo_g  = np.array([self.hsv_green_h_low, self.hsv_sat_low, self.hsv_val_low], dtype=np.uint8)
        hi_g  = np.array([self.hsv_green_h_high, 255, 255], dtype=np.uint8)

        mask_r = cv2.bitwise_or(cv2.inRange(hsv, lo_r1, hi_r1),
                                cv2.inRange(hsv, lo_r2, hi_r2))
        mask_g = cv2.inRange(hsv, lo_g, hi_g)

        kernel = np.ones((3, 3), np.uint8)
        mask_r = cv2.morphologyEx(mask_r, cv2.MORPH_OPEN, kernel)
        mask_g = cv2.morphologyEx(mask_g, cv2.MORPH_OPEN, kernel)

        r = cv2.countNonZero(mask_r)
        gr = cv2.countNonZero(mask_g)
        if r > gr and r >= self.hsv_min_color_pixels:
            return 'RED', r, gr
        if gr > r and gr >= self.hsv_min_color_pixels:
            return 'GREEN', r, gr
        return 'UNKNOWN', r, gr

    def _all_tl_boxes(self, result, roi_img, frame_area):
        """YOLO 결과 → 필터링·색보정된 신호등 박스 목록.

        frame_area 는 ★원본 프레임 전체 면적★ 이다(ROI 면적이 아니다) — ROI 를 바꿔도
        면적비(area_frac)의 의미가 흔들리지 않게 한다.
        """
        out = []
        self.last_red_drop = 0
        if result is None or result.boxes is None or len(result.boxes) == 0:
            return out
        # ROI 를 통째로 덮는 박스를 거르는 자 — ROI 높이 기준이라 ROI 를 바꿔도 산다.
        max_bh = (roi_img.shape[0] * self.tl_max_h_frac) if self.tl_max_h_frac > 0 else 0
        for b in result.boxes:
            conf   = float(b.conf[0].item()) if hasattr(b, 'conf') else 0.0
            cls_id = int(b.cls[0].item())    if hasattr(b, 'cls')  else -1
            if cls_id not in self.TL_ALLOW or conf < self.tl_conf:
                continue
            x1, y1, x2, y2 = [int(v) for v in b.xyxy[0].tolist()]
            bw = max(1, x2 - x1)
            bh = max(1, y2 - y1)
            area   = bw * bh
            aspect = bw / float(bh)
            if area   < self.tl_min_area   or area   > self.tl_max_area:   continue
            if aspect < self.tl_min_aspect or aspect > self.tl_max_aspect: continue
            if max_bh and bh > max_bh:                                     continue

            label = self.TL_LABEL.get(cls_id, str(cls_id))
            hsv_red = hsv_green = 0
            if self.use_hsv_refine:
                # HSV 는 conf 와 무관하게 ★항상★ 돌리되, 라벨을 뒤집는 조건은
                # 방향에 따라 다르다 — 신호등에서 오류 비용은 극단적으로 비대칭이다.
                crop = roi_img[max(0, y1):max(0, y2), max(0, x1):max(0, x2)]
                hsv_label, hsv_red, hsv_green = self._hsv_state(crop)
                if label == 'GREEN' and hsv_label == 'RED':
                    label = 'RED'                     # 안전 방향 → 항상 교정
                elif conf < HSV_FALLBACK_CONF and hsv_label in ('RED', 'GREEN'):
                    label = hsv_label                 # 위험 방향 → 저신뢰에서만 위임

            # RED 오탐 관문 — 붉지 않은 것이 RED 로 나가면 도로 한복판에서 선다.
            if (self.use_hsv_refine and self.tl_red_require_hsv
                    and label == 'RED' and hsv_red < self.hsv_min_color_pixels):
                self.last_red_drop += 1
                continue

            out.append({
                'label': label, 'conf': conf, 'box': (x1, y1, x2, y2), 'box_h': bh,
                # 근접도 지표. 해상도·화각에 묶이지 않아 box_h(px)보다 이식성이 좋다.
                'area_frac': area / float(frame_area) if frame_area else 0.0,
                'hsv_red': hsv_red, 'hsv_green': hsv_green,
            })
        return out

    def _near_metric(self, boxes):
        """지금 근접도 게이트에 걸리는 값 — ★빨간 박스 중 최대★. 빨간 박스가 없으면 0.

        단위는 ★쓰이고 있는 게이트의 단위★ 다:
          · tl_red_stop_min_area_frac > 0 → 면적비(0~1, 화면 대비)
          · 그 외(기본)                   → 박스 높이[px]
        임계와 이 값을 나란히 보면 RED / RED_FAR 가 갈린 이유가 그대로 드러난다.
        ★제어에는 쓰지 않는다★ — 판정은 _resolve_tl_state 가 이미 했고, 이건 기록용이다.
        """
        red = [b for b in boxes if b['label'] == 'RED']
        if not red:
            return 0.0
        if self.tl_red_stop_min_area_frac > 0.0:
            return max(b.get('area_frac', 0.0) for b in red)
        return float(max(b.get('box_h', 0) for b in red))

    def _near_gate_base(self):
        """★히스테리시스를 뺀 원래 임계★ 게이지의 자를 여기에 맞춘다.

        _near_gate 는 물고 있는 동안 낮아지므로, 그것으로 게이지 범위를 잡으면
        무는 순간 막대가 튄다 — 눈금만 움직여야 하고 자는 고정이어야 한다.
        """
        return (self.tl_red_stop_min_area_frac if self.tl_red_stop_min_area_frac > 0.0
                else float(self.tl_red_stop_min_height))

    def _near_gate(self):
        """지금 '가깝다'로 인정할 임계 — ★물고 있는 동안은 낮춘다★(히스테리시스).

        단위는 _near_metric 과 같다(면적비 또는 박스높이 px). 화면 HUD 에도 이 값을
        그대로 찍으므로, 임계가 낮아진 구간을 눈으로 확인할 수 있다.
        """
        base = self._near_gate_base()
        return base * self.tl_near_release_ratio if self.stopping else base

    def _resolve_tl_state(self, boxes):
        if not boxes:
            return 'UNKNOWN'
        red_boxes = [b for b in boxes if b['label'] == 'RED']
        if red_boxes:
            gate = self._near_gate()
            if self.tl_red_stop_min_area_frac > 0.0:
                near = any(b.get('area_frac', 0.0) >= gate for b in red_boxes)
            else:
                near = any(b.get('box_h', 0) >= gate for b in red_boxes)
            return 'RED' if near else 'RED_FAR'
        if any(b['label'] == 'GREEN' for b in boxes):
            return 'GREEN'
        return 'UNKNOWN'

    # ══════════════════════════════════════════════════════════════════════════
    #  인지 — 정지선 (perception.py 의 stop-line 절 이식)
    # ══════════════════════════════════════════════════════════════════════════
    def _detect_stop_line(self, frame):
        """정지선을 찾아 ★BEV 에서 앞범퍼까지의 픽셀 거리★ 를 돌려준다.

        돌려주는 것 : (dist_px, y_frac, poly, poly_bev) — 못 찾았으면 (None, −1, None, None)
          · dist_px  ★판정값★ camera_model 이 계산한다. 음수면 이미 선을 지났다.
          · y_frac   원본 화면 최하단 y / 높이. ★기록·HUD 전용★ (파일 헤더 참고)
          · poly     보정된 원본 좌표의 마스크 폴리곤 (HUD)
          · poly_bev 같은 폴리곤의 BEV 좌표 (HUD)

        ★가장 가까운 것을 고르는 기준이 BEV 로 바뀌었다 [2026-08-19]★ 종전에는 원본
        화면의 최하단 y 로 골랐다. 어안이 남은 그림에서는 화면 아래에 있다고 더 가까운
        것이 아니라서(가장자리가 아래로 휜다) 순서가 뒤집힐 수 있었다. 보정+BEV 를
        거친 지금은 ★거리로 직접 비교★ 한다.

        ROI 를 두지 않고 프레임 전체로 추론한다 — 이 모델은 전체 화면(lane_roi 기본값
        0,0,1920,1080)으로 학습·운용된 것이라, 잘라 넣으면 종횡비가 달라져 인지가
        나빠진다. 잡티는 아래 폭·면적 관문이 거른다.
        """
        none = (None, -1.0, None, None)
        try:
            res = self.sl_model.predict(source=frame, conf=self.sl_conf,
                                        imgsz=self.sl_imgsz, device=self.device,
                                        verbose=False)[0]
        except Exception as e:
            self.get_logger().error(f"YOLO stop-line error: {e}", throttle_duration_sec=5.0)
            return none
        if res.masks is None or res.boxes is None:
            return none

        h, w = frame.shape[:2]
        best = none
        for i, pts in enumerate(res.masks.xy):
            if i >= len(res.boxes):
                break
            try:
                cls_id = int(res.boxes[i].cls[0].item())
            except Exception:
                continue
            if cls_id != self.sl_class_id:
                continue
            pts = np.asarray(pts, dtype=np.float32)
            if len(pts) < 3:
                continue
            xs, ys = pts[:, 0], pts[:, 1]
            # ★정지선은 가로로 긴 물체다★ 폭·면적이 안 되면 노면 잡티·차선 조각이다.
            if (float(xs.max() - xs.min()) / w) < self.sl_min_width_frac:
                continue
            if (abs(cv2.contourArea(pts)) / float(w * h)) < self.sl_min_area_frac:
                continue
            dist, poly_bev = self.cam.nearest_dist_px(pts)
            if dist is None:
                # 폴리곤이 통째로 소실선 너머다 = BEV 좌표가 무의미하다. 버린다.
                continue
            if best[0] is None or dist < best[0]:
                best = (dist, float(ys.max()) / float(h), pts, poly_bev)
        return best

    def _sl_should_run(self, now):
        """지금 정지선 추론을 돌릴 이유가 있는가 — ★평상시에는 없다★.

        이 추론은 신호등 추론 위에 얹히는 두 번째 추론이라 프레임 예산을 그대로
        먹는다. 정지선이 필요한 순간은 '빨간 불이 보이는 동안'과 '이미 서 있는 동안'
        뿐이다(서 있는 동안은 HUD 로 정지 위치를 확인하기 위해서다).
        """
        if not self.sl_enable or self.sl_model is None:
            return False
        if self.stopping:
            return True
        return self.red_seen_t > 0.0 and (now - self.red_seen_t) <= self.sl_gate_red_s

    def _update_stop_line(self, frame, boxes):
        """프레임 하나로 정지선 상태를 갱신한다(스트릭·신선도는 신호등과 같은 방식)."""
        now = time.time()
        if any(b['label'] == 'RED' for b in boxes):
            # RED_FAR 도 라벨은 RED 다 — ★멀리서 빨간불이 보이기 시작하면★ 그때부터
            # 정지선을 찾기 시작한다는 뜻이고, 그것이 우리가 원하는 시점이다.
            self.red_seen_t = now

        if not self._sl_should_run(now):
            return
        self.sl_frames += 1
        if self.sl_frames % self.sl_interval != 0:
            return

        dist, y, poly, poly_bev = self._detect_stop_line(frame)
        if dist is not None:
            if self.sl_since is None:
                self.sl_since = now
            self.sl_seen_t = now
            self.sl_px = dist
            self.sl_y = y
            self.sl_poly = poly
            self.sl_poly_bev = poly_bev
        elif (now - self.sl_seen_t) > self.sl_stale_s:
            # 신선도가 끊긴 뒤에야 지운다 — 몇 프레임 놓친 것과 '지나쳐서 사라진 것'을
            # 여기서 구별하지 않는다. 구별은 _stop_plan 이 한다.
            self.sl_since = None
            self.sl_px = -1.0
            self.sl_y = -1.0
            self.sl_poly = None
            self.sl_poly_bev = None

    def cb_image(self, msg: Image):
        if self.model is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}", throttle_duration_sec=5.0)
            return

        # ★어안 왜곡보정 — 여기 한 줄이 전부다 [2026-08-19]★ 이 아래로는 신호등
        #   detect·정지선 seg·HUD 가 전부 ★보정된 그림★ 을 본다. 계수와 절차의 주인은
        #   camera_model.py 이고, 보정이 꺼져 있거나 캘리브를 못 읽었으면 원본이
        #   그대로 돌아온다(fail-open). /image_raw 자체는 건드리지 않는다 — 원본
        #   녹화(nxde video)가 카메라가 준 그림을 봐야 하기 때문이다(파일 헤더).
        frame = self.cam.undistort(frame)

        self.frame_count += 1
        run = (self.frame_count % self.tl_interval == 0)
        h, w = frame.shape[:2]
        xmin, ymin, xmax, ymax = self._clamp_roi(w, h, *self.tl_roi)
        roi_img = frame[ymin:ymax, xmin:xmax]

        boxes = []
        if roi_img.size != 0:
            if run:
                try:
                    res = self.model.predict(source=roi_img.copy(), conf=self.tl_conf,
                                             imgsz=self.tl_imgsz, device=self.device,
                                             verbose=False)[0]
                    boxes = self._all_tl_boxes(res, roi_img, w * h)
                    self.last_boxes = boxes
                    # "YOLO 가 못 봤다" vs "필터가 먹었다" 를 사후에 가르기 위한 값.
                    self.last_raw = 0 if res.boxes is None else len(res.boxes)
                except Exception as e:
                    self.get_logger().error(f"YOLO TL error: {e}", throttle_duration_sec=5.0)
                    self.last_boxes = []
                    self.last_raw = -1
                    self.last_red_drop = 0
            else:
                boxes = self.last_boxes

        # ★시험용 주입★ 여기서 YOLO 결과를 ★통째로★ 갈아 끼운다(위 tl_fake_box_h).
        #   검출 뒤에 두는 이유 — 앞에 두면 추론 시간이 빠져 지연 통계가 실제와
        #   달라진다. 시험이 재는 것은 '이 프레임을 처리하는 데 얼마 걸리나' 이므로
        #   추론은 그대로 돌리고 결론만 바꾼다.
        if self.fake_box_h > 0.0:
            boxes = [self._fake_red_box(self.fake_box_h, w * h)]
            self.last_boxes = boxes

        state = self._resolve_tl_state(boxes)
        self.pub_state.publish(String(data=state))
        # ★[2026-08-14] '신호등을 어떻게 보고 있나' 를 기록으로 남긴다★
        #   near_metric : 지금 근접도 게이트에 걸리는 값(빨간 박스 중 최대). 임계와
        #                 나란히 보면 RED / RED_FAR 가 갈린 이유가 그대로 드러난다.
        #   red_far     : 이번 프레임 판정이 RED_FAR 인가 = '빨갛지만 아직 멀다'.
        #   둘 다 record 가 CSV 로 받아 적는다(white1/record.py).
        self.pub_near.publish(Float32(data=float(self._near_metric(boxes))))
        self.pub_red_far.publish(Bool(data=(state == 'RED_FAR')))
        self._feed_state(state)
        # ★정지선은 신호등 판정 뒤에 본다★ 빨간 박스를 봤는지가 추론을 돌릴지의
        #   조건이므로 순서가 뒤집히면 한 프레임씩 늦는다(_update_stop_line 주석).
        self._update_stop_line(frame, boxes)
        self.img_time = time.time()

        if run:
            now = time.monotonic()
            inst = 1.0 / max(1e-6, now - self.fps_t)
            self.fps = 0.9 * self.fps + 0.1 * inst if self.fps > 0 else inst
            self.fps_t = now

        if self.show_window:
            self._draw(frame, boxes, state, (xmin, ymin, xmax, ymax))

    # ══════════════════════════════════════════════════════════════════════════
    #  표시 — ★계기판을 그린다★ [2026-08-24 전면 개편]
    # ══════════════════════════════════════════════════════════════════════════
    #  ★원칙 하나 — 먼저 표시 크기로 줄이고, 그 좌표계에서 그린다★
    #    종전에는 1920x1080 에 오버레이를 그린 뒤 표시 직전에 창 폭으로 축소했다.
    #    그래서 글자·선이 전부 ★실효 1/3 크기★ 가 됐다(fontScale 0.8 → 0.27, BEV 안의
    #    문턱 라벨은 0.17). 튜닝에 쓰라고 찍어 둔 숫자를 읽을 수 없으면 없는 것과 같다.
    #    지금은 프레임을 먼저 줄이고 박스·ROI·사다리꼴·정지선을 그 좌표계에서 그린다 —
    #    글자가 창 크기에 맞고, 큰 그림에 그리는 비용도 없어진다.
    #
    #  ★글자는 그림 위에 얹지 않는다★
    #    HUD 는 하단 검은 띠, BEV·게이지·파라미터는 우측 패널로 뺐다. 밝은 노면 위의
    #    회색 글자를 읽으려고 애쓰는 일이 없어지고, 글자가 정지선을 가리지도 않는다.
    #
    #  ★한글을 그린다 (white1/py)★
    #    cv2.putText 는 ASCII 만 그린다. 종전 HUD 는 stop_why(전부 한글)를
    #    `???1??? PRE[????????-1???]` 로 찍고 있었다 — 즉 ★'왜 이 단계인가' 를 화면에서
    #    읽을 수 없었다★. 이제 PIL 로 그리고, 폰트가 없으면 조용히 물러난다(fail-open).
    #
    #  ★비용은 오히려 줄었다 (같은 프레임 실측)★
    #      종전  window_width=640  7.96 ms   ← 기본값. 1920→640 리사이즈만 2.50ms
    #      종전  window_width=960  5.61 ms
    #      지금  HUD 문자열 그대로 2.38 ms   ← 대부분의 프레임
    #      지금  HUD 매 프레임 갱신 7.55 ms  ← 최악의 경우가 종전 기본값과 같다
    #    캔버스를 매 프레임 새로 만들지 않는 것(CanvasPool)만으로 3.6ms 가 없어졌다.

    #  하단 HUD 의 글자 크기·줄 수. 열 좌표는 이 크기의 '칸' 단위로 적는다.
    HUD_SIZE  = 14
    HUD_LINES = 3
    #  우측 패널이 뷰 폭에서 차지하는 비율. BEV 썸네일(4:3)이 이 폭을 채운다.
    PANEL_RATIO = 0.30

    def _draw(self, frame, boxes, state, roi):
        now = time.time()
        sl_live = self.sl_enable and self.sl_model is not None
        sl_fresh = self._sl_present(now)

        # ── 뷰 — ★먼저 줄인다★ ──────────────────────────────────────────
        h0, w0 = frame.shape[:2]
        vw = w0 if self.window_width <= 0 else min(self.window_width, w0)
        sc = vw / float(w0)
        vh = int(round(h0 * sc))
        pw = int(round(vw * self.PANEL_RATIO)) if (self.show_bev and sl_live) else 0
        hh = self.HUD_LINES * self.tr.line_h(self.HUD_SIZE) + 10
        canvas = self._pool.get(vw + pw, vh + hh)
        view = canvas[0:vh, 0:vw]
        #  ★정확히 절반일 때만 INTER_AREA★ 1920→960 은 0.28ms 인데 1920→640 은 같은
        #  보간으로 2.50ms 다(임의 배율은 일반 경로로 떨어진다). 그때는 LINEAR 를 쓴다.
        cv2.resize(frame, (vw, vh), dst=view,
                   interpolation=(cv2.INTER_AREA if abs(sc - 0.5) < 1e-6
                                  else cv2.INTER_LINEAR))

        # ── ROI — ★선이 아니라 밖을 어둡게★ ────────────────────────────
        #   종전에는 노란 사각형이었는데, BEV 사다리꼴도 거의 같은 노란색이라 둘이
        #   구별되지 않았다. 밖을 어둡게 하면 색을 하나 쓰지 않고 ROI 를 말할 수 있다.
        #   ⚠️ 너무 어둡게 하면 안 된다 — ★정지선은 ROI 밖(노면)에 있다★.
        rect = tuple(int(round(v * sc)) for v in roi)
        if self.draw_roi:
            dim_outside(view, rect, self.roi_dim)
            cv2.rectangle(view, (rect[0], rect[1]),
                          (max(0, rect[2] - 1), max(0, rect[3] - 1)), (120, 120, 124), 1)

        # ── BEV 사다리꼴·정지선 ────────────────────────────────────────
        #   사다리꼴은 항상 그린다 — 이것이 곧 '거리를 어디서 재는가' 이고, 실측 튜닝
        #   (STOPLINE_TEST.md 단계 2)은 이 도형을 노면에 맞추는 일이다.
        if sl_live:
            cv2.polylines(view, [(self.cam.src_pts * sc).astype(np.int32)], True,
                          C_CYAN, 1)
            atxt(view, self.cam.src_pts[0, 0] * sc + 4,
                     self.cam.src_pts[0, 1] * sc + 13, 'BEV', C_CYAN, 0.38)
            if self.sl_poly is not None and sl_fresh:
                poly = (self.sl_poly * sc).astype(np.int32)
                cv2.polylines(view, [poly], True, C_MAGENTA, 2)
                bi = int(np.argmax(poly[:, 1]))
                cv2.circle(view, (int(poly[bi, 0]), int(poly[bi, 1])), 4,
                           C_MAGENTA, -1)

        # ── 박스 ───────────────────────────────────────────────────────
        for it in boxes:
            x1, y1, x2, y2 = it['box']
            bx1, by1 = int(round((x1 + roi[0]) * sc)), int(round((y1 + roi[1]) * sc))
            bx2, by2 = int(round((x2 + roi[0]) * sc)), int(round((y2 + roi[1]) * sc))
            col = (C_RED if it['label'] == 'RED'
                   else C_GREEN if it['label'] == 'GREEN' else C_GRAY)
            cv2.rectangle(view, (bx1, by1), (bx2, by2), col, 2)
            # ★배경칩★ 이 없으면 밝은 하늘·벽 위에서 라벨을 읽을 수 없다.
            chip(view, bx1, (by1 - 16) if by1 > 17 else by2,
                     f"{it['label']} {it['conf']:.2f} "
                     f"{100.0 * it.get('area_frac', 0.0):.2f}%", col)

        # ── 제동 단계를 테두리로 ───────────────────────────────────────
        #   ★곁눈으로도 보여야 한다★ 실차에서 창을 뚫어지게 보고 있을 수는 없다.
        if self.stopping:
            if self.stop_level >= self.brake_level:
                cv2.rectangle(view, (0, 0), (vw - 1, vh - 1), C_RED, 6)
            else:
                cv2.rectangle(view, (0, 0), (vw - 1, vh - 1), C_AMBER, 4)

        if pw:
            self._draw_panel(canvas, frame, vw, vh, pw, sl_fresh)
        self._draw_hud(canvas, vw + pw, vh, hh, state, boxes, now, sl_live, sl_fresh)

        # ★여기서 imshow 를 부르지 않는다 [2026-08-14]★ 이 함수는 영상 콜백(워커
        #   스레드)에서 돈다. OpenCV HighGUI(GTK)는 ★스레드 안전하지 않고★, 워커
        #   스레드에서 imshow/waitKey 를 부르면 그 스레드가 GTK 안에서 멎는다 —
        #   그러면 영상 콜백이 다시 돌지 못해 ★'/image_raw 두절'★ 이 된다(실측:
        #   단일 스레드 판은 멀쩡, MultiThreadedExecutor 로 바꾼 판은 0.9초 만에 두절).
        #   그래서 그린 캔버스만 넘겨 두고, 실제 표시는 ★메인 스레드★ 가 한다(main).
        with self._show_lock:
            self._show_frame = canvas

    def _draw_panel(self, canvas, frame, vw, vh, pw, sl_fresh):
        """우측 패널 — ★BEV 썸네일 + 게이지 두 개 + 파라미터 요약★ [2026-08-24]

        ★썸네일 없이는 두 문턱을 잡을 수 없다★ sl_brake1_px·sl_brake2_px 는 BEV
        픽셀이라 원본 화면만 봐서는 어디쯤인지 알 수 없다. 범퍼선과 두 문턱선을 그려
        두면 정지선 폴리곤이 어느 선을 넘는 순간 몇 단이 물리는지가 눈에 보인다.

        종전에는 이것을 ★그림 위에 겹쳐★ 그렸다. 그래서 (a) 창의 가로 1/3·세로 44%
        를 덮으며 정지선이 실제로 보이는 우하단 노면을 가렸고, (b) 축소를 함께 받아
        문턱 라벨이 실효 0.17 크기가 됐다. 패널로 빼면 둘 다 없어진다.
        """
        px, tw = vw, pw - 12
        th = int(round(tw * self.cam.bev_h / float(self.cam.bev_w)))
        thumb = canvas[6:6 + th, px + 6:px + 6 + tw]
        # ★깨끗한 원본에서 썸네일 크기로 바로 편다★ 두 가지를 함께 고친다:
        #   ① 종전에는 ★오버레이가 다 그려진 dbg★ 를 펴서, 썸네일 안에 사다리꼴·정지선이
        #      한 번 더 warp 되어 들어왔다(선이 두 겹으로 보이던 것).
        #   ② 종전에는 640x480 을 뜬 뒤 축소를 함께 받았다. 결과 크기로 바로 펴면
        #      워프 비용이 결과 화소 수에 비례해 줄어든다(1.6ms → 0.9ms).
        if self._bev_m_key != (tw, th):
            self._bev_m_key = (tw, th)
            self._bev_m = self.cam.bev_matrix(tw, th)
        cv2.warpPerspective(frame, self._bev_m, (tw, th), dst=thumb)

        k = tw / float(self.cam.bev_w)
        for dist, col, lab in ((0.0, C_GREEN, 'BUMPER'),
                               (self.sl_brake2_px, C_RED,
                                f"B2 {self.sl_brake2_px:.0f}"),
                               (self.sl_brake1_px, C_AMBER,
                                f"B1 {self.sl_brake1_px:.0f}")):
            y = int(round((self.cam.bumper_y - dist) * k))
            if not (-4 <= y <= th + 4):
                continue
            # ★클램프한다★ 종전 조건(0 <= y < th)에서는 기본값 bev_bumper_y_px=480 이
            #   bev_h=480 과 같아 ★거리 0 의 기준선이 절대 안 그려졌다★. 그 선을 보고
            #   범퍼행을 맞추는 절차인데(STOPLINE_TEST 단계 2) 선이 없었던 것이다.
            y = max(0, min(th - 1, y))
            cv2.line(thumb, (0, y), (tw, y), col, 1)
            # BUMPER 라벨만 오른쪽으로 — 범퍼선과 B2 선은 붙어 있기 마련이다.
            #   폭은 ★재서★ 정한다(글자 수 × 상수로 어림하면 오른쪽으로 삐져나간다).
            lw = cv2.getTextSize(lab, cv2.FONT_HERSHEY_SIMPLEX, 0.33, 1)[0][0]
            atxt(thumb, (tw - lw - 4) if dist == 0.0 else 3,
                 max(10, y - 3), lab, col, 0.33)
        if self.sl_poly_bev is not None and sl_fresh:
            pts = (np.asarray(self.sl_poly_bev) * k).astype(np.int32)
            cv2.polylines(thumb, [pts], True, C_MAGENTA, 2)
            bi = int(np.argmax(pts[:, 1]))
            cv2.circle(thumb, (int(pts[bi, 0]), int(pts[bi, 1])), 3, C_MAGENTA, -1)
        cv2.rectangle(canvas, (px + 6, 6), (px + 6 + tw, 6 + th), C_FRAME, 1)
        cv2.line(canvas, (px, 0), (px, vh), C_FRAME, 1)

        # ── 게이지 — ★지금값을 문턱과 나란히★ ─────────────────────────
        yy = 6 + th + 12
        cv2.rectangle(canvas, (px + 1, yy - 4), (px + pw, yy + 80), C_BG, -1)
        gate, near = self._near_gate(), self._near_metric(self.last_boxes)
        base = max(1e-6, self._near_gate_base())
        span = base * 1.5                      # 게이지 오른쪽 끝 = 임계의 1.5배
        blit(canvas, self.tr.patch('근접', C_DIM, 12), px + 7, yy)
        atxt(canvas, px + 48, yy + 12,
                 (f"{100.0 * near:.2f}/{100.0 * gate:.2f}%"
                  if self.tl_red_stop_min_area_frac > 0.0 else f"{near:.0f}/{gate:.0f}px"),
                 C_RED if near >= gate else C_TXT, 0.40)
        gauge(canvas, px + 8, yy + 20, pw - 22, 8, near / span,
                  C_RED if near >= gate else (108, 108, 114),
                  ((gate / span, (255, 255, 255)),))

        yy += 40
        rng = max(1.0, self.sl_brake1_px * 1.6)   # 왼쪽 = 멀다, 오른쪽 = 범퍼
        cur = self.sl_px if (sl_fresh and self.sl_px > -900.0) else -1.0
        blit(canvas, self.tr.patch('정지선', C_DIM, 12), px + 7, yy)
        atxt(canvas, px + 62, yy + 12, (f"{cur:.0f}px" if cur >= 0 else '--'),
                 C_MAGENTA if cur >= 0 else C_DIM, 0.40)
        gauge(canvas, px + 8, yy + 20, pw - 22, 8,
                  0.0 if cur < 0 else 1.0 - cur / rng, C_MAGENTA,
                  ((1.0 - self.sl_brake1_px / rng, C_AMBER),
                   (1.0 - self.sl_brake2_px / rng, C_RED)))

        # ── 파라미터 요약 — ★정적이라 한 번만 그린다★ ────────────────
        #   '이 창이 지금 어떤 값으로 판정하나' 가 화면에 남아 있어야 튜닝 중에
        #   터미널을 왕복하지 않는다. 비용은 처음 한 프레임뿐이다.
        yy += 44
        avail = max(1, vh - yy)
        if self._par_strip is None or self._par_strip.shape[0] != avail:
            sp = self.cam.src_pts
            gate_txt = (f"gate {100.0 * self.tl_red_stop_min_area_frac:.2f}%"
                        if self.tl_red_stop_min_area_frac > 0.0
                        else f"gate {self.tl_red_stop_min_height}px")
            rows = [
                (8, 2, '판정 파라미터', C_TXT, 12, True),
                (8, 22, f"conf {self.tl_conf:.2f}  imgsz {self.tl_imgsz}"
                        f"  x{self.tl_interval}", C_DIM, 12, False),
                (8, 40, f"hold {self.tl_hold_s:.2f}s  grace {self.tl_gap_grace_s:.2f}s",
                 C_DIM, 12, False),
                (8, 58, f"{gate_txt}  release x{self.tl_near_release_ratio:.2f}",
                 C_DIM, 12, False),
                (8, 76, f"놓기유예 {self.red_release_hold_s:.2f}s"
                        f"{'  래치' if self.stop_latch else ''}", C_DIM, 12, False),
                (8, 100, f"sl conf {self.sl_conf:.2f}  hold {self.sl_hold_s:.2f}s",
                 C_DIM, 12, False),
                (8, 118, f"대기상한 {self.sl_wait_max_s:.1f}s"
                         f"  코앞 x{self.sl_override_gate_ratio:.1f}",
                 C_DIM, 12, False),
                (8, 142, f"BEV {sp[0, 0]:.0f},{sp[0, 1]:.0f} {sp[1, 0]:.0f},{sp[1, 1]:.0f}",
                 C_DIM, 12, False),
                (8, 160, f"    {sp[2, 0]:.0f},{sp[2, 1]:.0f} {sp[3, 0]:.0f},{sp[3, 1]:.0f}",
                 C_DIM, 12, False),
            ]
            # ★자리가 없으면 아래쪽 줄을 버린다★ 좁은 창(window_width 640)에서는 패널
            #   높이가 모자라는데, 그냥 그리면 마지막 줄이 글자 중간에서 잘려 더 나쁘다.
            self._par_strip = self.tr.strip(
                pw, avail, [r for r in rows if r[1] + r[4] + 2 <= avail], C_BG)
        blit(canvas, self._par_strip, px + 1, yy)

    def _permit_txt(self, now):
        """지금 개입 허락이 어디서 오는가 — HUD 한 줄로 쓸 짧은 말.

        ★이것이 화면에 없어서 제일 헤맸다★ 허락이 없으면 stopping 이 false 라
        종전 HUD 는 단계 표시조차 빈 문자열이었다. 화면만 보면 '빨간불을 보는데
        아무 일도 안 일어난다' 로 보인다 — 원인은 체크박스가 꺼진 것뿐인데.
        """
        if not self.require_permission:
            return '항상'
        if self.tl_enable and (now - self.tl_enable_t) <= TL_ENABLE_STALE_S:
            return '체크박스'
        if self.tl_permit and (now - self.tl_permit_t) <= TL_ENABLE_STALE_S:
            return f"주행({self.drive_state or '?'})"
        return ''

    def _draw_hud(self, canvas, cw, vh, hh, state, boxes, now, sl_live, sl_fresh):
        """하단 HUD 3줄. ★열은 고정폭 '칸' 으로 적는다★ — 줄이 달라도 열이 맞는다.

        한 줄의 원소 = (칸, 글자, 색[, 굵게]). 앞 원소와 겹치려 하면 밀어 낸다
        (겹쳐 찍혀 못 읽는 것보다 열이 조금 어긋나는 편이 낫다).
        """
        sz, cell = self.HUD_SIZE, self.tr.cell(self.HUD_SIZE)
        scol = (C_RED if 'RED' in state
                else C_GREEN if state == 'GREEN' else C_DIM)
        # RED 확정까지 얼마나 찼는가 — 종전에는 이 진행이 화면에 전혀 없었다.
        held = 0.0 if self.red_since is None else max(0.0, now - self.red_since)
        permit = self._permit_txt(now)
        lvl = self.stop_level if self.stopping else 0
        lvl_txt = (f"제동 {lvl}단 "
                   + ('STOP' if lvl >= self.brake_level else 'PRE')) if lvl else '제동 없음'
        lvl_col = (C_RED if lvl >= self.brake_level
                   else C_AMBER if lvl else C_DIM)
        gate, near = self._near_gate(), self._near_metric(boxes)
        near_txt = (f"근접 {100.0 * near:.2f}/{100.0 * gate:.2f}%"
                    if self.tl_red_stop_min_area_frac > 0.0
                    else f"근접 {near:.0f}/{gate:.0f}px")
        cur = self.sl_px if (sl_fresh and self.sl_px > -900.0) else -1.0
        if not sl_live:
            sl_txt, sl_col = '정지선 OFF', C_DIM
        elif cur < 0:
            sl_txt, sl_col = '정지선 미검출', C_DIM
        else:
            sl_txt = (f"정지선 {cur:.0f}px{self.cam.m_txt(cur)}"
                      + (' 확정' if self._sl_confirmed(now) else ''))
            sl_col = C_MAGENTA
        waited = (0.0 if self.red_conf_t is None else max(0.0, now - self.red_conf_t))

        lines = [
            [(0, state, scol, True),
             (12, f"확정 {held:.2f}/{self.tl_hold_s:.2f}s", C_TXT),
             (32, near_txt, C_RED if near >= gate else C_TXT),
             (54, f"raw {self.last_raw} drop {self.last_red_drop}", C_DIM),
             (72, f"FPS {self.fps:.0f}", C_DIM),
             (84, f"보정 {'ON' if self.cam.enabled else 'OFF'}",
              C_DIM if self.cam.enabled else C_AMBER)],
            [(0, lvl_txt, lvl_col, True),
             (14, (f"- {self.stop_why}" if (self.stop_why and permit) else ''), C_TXT),
             (34, f"허락 {permit or '없음'}",
              C_GREEN if permit else C_RED),
             (56, (f"대기 {waited:.1f}/{self.sl_wait_max_s:.1f}s"
                   if (self.sl_wait and sl_live) else ''), C_AMBER)],
            [(0, sl_txt, sl_col),
             (26, (f"B1 {self.sl_brake1_px:.0f}  B2 {self.sl_brake2_px:.0f}"
                   if sl_live else ''), C_DIM),
             (44, (f"범퍼행 {self.cam.bumper_y:.0f}"
                   f"  BEV {self.cam.bev_w}x{self.cam.bev_h}" if sl_live else ''),
              C_DIM),
             (72, ('★래치' if (self.stop_latch and self.stopping) else ''), C_AMBER)],
        ]
        items = []
        for li, segs in enumerate(lines):
            y, xend = 5 + li * self.tr.line_h(sz), 0.0
            for seg in segs:
                col_cell, txt, col = seg[0], seg[1], seg[2]
                if not txt:
                    continue
                bold = len(seg) > 3 and seg[3]
                size = sz + (3 if bold else 0)
                x = max(col_cell * cell, xend)
                items.append((x, y - (2 if bold else 0), txt, col, size, bold))
                xend = x + self.tr.width(txt, size, bold) + 2 * cell
        blit(canvas, self._hud_strip.get(cw, hh, items, C_HUD), 0, vh)

    def show_pending(self):
        """★메인 스레드에서만 부른다★ 마지막으로 그려 둔 캔버스를 창에 띄운다."""
        if not self.show_window:
            return
        with self._show_lock:
            frame, self._show_frame = self._show_frame, None
        if frame is None:
            return
        if not self._window_ready:
            # 크기를 사람이 바꿀 수 있게 두고, 처음 크기만 정해 준다
            # (WINDOW_AUTOSIZE 면 원본 크기로 고정되어 창을 못 줄인다).
            cv2.namedWindow('Traffic Light', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Traffic Light', frame.shape[1], frame.shape[0])
            self._window_ready = True
        cv2.imshow('Traffic Light', frame)
        cv2.waitKey(1)

    # ══════════════════════════════════════════════════════════════════════════
    #  판단 — camera_judgment.py 의 확정 필터 이식
    # ══════════════════════════════════════════════════════════════════════════
    def _feed_state(self, state):
        """프레임 판정 하나를 스트릭에 먹인다.

        ★끊김 유예가 이 필터의 핵심이다★ 실측 rosbag 의 패턴은
        RED,RED,UNKNOWN,UNKNOWN,RED,RED,RED,RED,UNKNOWN (프레임간 0.04~0.15s) 였다.
        RED 아닌 프레임 하나에 스트릭을 리셋하면 빨간불을 6프레임이나 정확히 보고도
        tl_hold_s(0.4s) 연속을 못 채운다 → 신호등에서 안 선다. 그래서 마지막 목격
        시각으로부터 tl_gap_grace_s 를 넘겨서야 스트릭을 죽인다.
        """
        now = time.time()
        # ★[2026-08-14] RED_FAR 는 스트릭에 먹이지 않는다★ 종전에는 두 상태를 같이
        #   먹였고, 그 한 줄이 근접도 게이트를 통째로 무력화하고 있었다:
        #     ① 확정 우회 — 먼 빨간불로 red_since 가 이미 쌓여 있으면, 한 프레임이
        #        RED 로 넘어오는 순간 tl_hold_s(0.4s) 가 ★이미 충족★ 되어 즉시 물린다.
        #        멀리서 오탐(미등·붉은 간판)이 스트릭을 쌓아 두면 스치는 한 프레임에 급제동.
        #     ② 해제 방해 — red_last_seen 이 갱신되므로, 한 번 선 뒤에는 ★다른 교차로의
        #        먼 빨간불★ 만 보여도 _red_gone 이 성립하지 않아 계속 물려 있다.
        #   이제 RED_FAR 는 ★관측 전용★ 이다(/tl/state·/tl/red_far 로 기록만 된다).
        #   경계에서 RED↔RED_FAR 로 흔들리는 것은 tl_gap_grace_s(0.3s) 가 메우고,
        #   물고 있는 동안은 _near_gate 가 임계를 낮춰(tl_near_release_ratio) 막는다.
        if state == 'RED':
            if self.red_since is None:
                self.red_since = now
            self.red_last_seen = now
        elif (self.red_last_seen is None
              or (now - self.red_last_seen) > self.tl_gap_grace_s):
            # ★스트릭(red_since)만 죽인다 [2026-08-14]★ red_last_seen 은 지우지 않는다 —
            #   '언제 마지막으로 빨간불을 봤나'는 ★해제 판정(_red_gone)의 근거★ 라서
            #   여기서 None 으로 지우면 그 순간 '아주 오래전부터 못 봤다'가 되어
            #   해제 유예(red_release_hold_s)가 통째로 무력화된다.
            self.red_since = None

        if state == 'GREEN':
            if self.green_since is None:
                self.green_since = now
            self.green_last_seen = now
        elif (self.green_last_seen is None
              or (now - self.green_last_seen) > self.tl_gap_grace_s):
            self.green_since = None
            self.green_last_seen = None

        self.tl_state = state
        self.tl_time  = now

    def _fresh(self, now):
        """마지막 판정이 아직 믿을 만한가. 낡았으면 개입하지 않는다(fail-open)."""
        return self.tl_time > 0.0 and (now - self.tl_time) <= self.tl_state_max_age

    def _red_confirmed(self, now):
        """★RED(근접) 만 정지시킨다★ RED_FAR 는 아무것도 하지 않는다(관측 전용).

        구 white 는 정지선 거리가 있으면 RED_FAR 에서 서행(tl_far_cap)을 했지만,
        white1 에는 속도를 부드럽게 깎을 경로가 없다(리니어는 물리거나 풀리거나
        둘뿐이다). 교차로 한참 전에 2단을 물면 그게 곧 급제동이므로, 근접도 게이트를
        통과한 RED 에서만 선다.
        """
        if self.red_since is None or not self._fresh(now):
            return False
        if self.tl_state != 'RED':
            return False
        return (now - self.red_since) >= self.tl_hold_s

    def _red_gone(self, now):
        """빨간불이 ★red_release_hold_s 이상★ 안 보였는가 = 이제 놓아도 되는가.

        ★[2026-08-14] 해제에 유예를 뒀다 — 실차에서 리니어가 들락날락했다★
        종전 해제 조건은 '_red_confirmed 가 거짓'이었고, 그것은 스트릭이 끊기는
        순간(tl_gap_grace_s 0.3초) 곧바로 참이 된다. 인지가 한두 번 흔들리면
            RED 확정 → 0.3초 놓침 → 해제 → 다시 0.4초 연속 → 체결 …
        이 1초 주기로 반복된다. 리니어는 물리적으로 왕복하는 장치라 이 왕복이 가장
        나쁘다(B보드 1회 구동 최대 1초). 그래서 ★놓는 쪽에만★ 유예를 준다 —
        무는 쪽(tl_hold_s)은 그대로 두어 반응이 늦어지지 않게 한다.
        """
        if self.red_last_seen is None:
            return True
        return (now - self.red_last_seen) >= self.red_release_hold_s

    def _green_confirmed(self, now):
        if self.green_since is None or not self._fresh(now):
            return False
        return (now - self.green_since) >= self.green_hold_s

    def _sl_present(self, now):
        """정지선을 ★지금 보고 있는가★ (sl_stale_s 안의 관측이 있는가)."""
        return self.sl_seen_t > 0.0 and (now - self.sl_seen_t) <= self.sl_stale_s

    def _sl_confirmed(self, now):
        """sl_hold_s 이상 연속으로 본 정지선인가. 한두 프레임 스친 것은 인정하지 않는다
        — 그것으로 브레이크를 미루면 오검출 하나가 곧 '안 서는 사고'가 된다."""
        return (self.sl_since is not None and self._sl_present(now)
                and (now - self.sl_since) >= self.sl_hold_s)

    def _stop_plan(self, now):
        """RED 는 확정됐다 — ★지금 몇 단을 물어야 하는가★. (level, 근거) 를 돌려준다.

        판정표는 파일 헤더에 있고, 여기서 중요한 것은 ★모든 애매한 경우가 '2단' 으로
        떨어진다★ 는 것이다:

          · 정지선 기능이 꺼졌다/모델 없음  → 2단   ← 종전 동작 그대로
          · 정지선을 못 봤다                → 2단   ← 종전 동작 그대로
          · 봤다가 놓쳤다                   → 2단   ← 이미 선 위에 있다는 뜻
          · 아직 확정 전(스침)              → 2단
          · 확정 + ≤ sl_brake2_px           → 2단   ← ★정지선 앞 정지★
          · 확정 + ≤ sl_brake1_px           → 1단   ← ★예비제동★
          · 확정 + 아직 멀다                → 0단   ← 유일하게 참는 경우

        그 '참는 경우'와 '1단으로 줄이는 경우'에는 상한이 둘 있다. 오검출된 먼 정지선이
        2단을 무한정 미루는 것은 ★빨간불에 안 서는 것★ 과 같기 때문이다:
          ① sl_wait_max_s          RED 확정 이후 흐른 시간
          ② sl_override_gate_ratio 근접도가 게이트의 이 배 = 신호등이 코앞이다
        ②가 ①보다 낫다 — 시간은 속도에 따라 거리가 달라지지만, 근접도는 '얼마나
        가까운가' 자체이기 때문이다. 둘 다 둔 것은 신호등이 흔들려도 상한이 남게
        하려는 것이다.
        ★①은 '1단으로 가다 멈춰 버린 경우'도 함께 받는다★ 1단이면 구동펄스가 0 이라
        차가 스스로 정지선까지 못 간다 — 그대로 두면 2단 문턱에 영영 못 닿으므로
        시간이 차면 2단으로 올려 정지를 확정한다(지시사항).
        """
        full = self.brake_level
        if not self.sl_enable or self.sl_model is None:
            return full, '정지선 없음'
        if self.red_conf_t is None:
            self.red_conf_t = now          # 대기 상한 ①의 기준점
        if (now - self.red_conf_t) >= self.sl_wait_max_s:
            if self.stop_level < full:
                self.get_logger().warn(
                    f"🚦 정지선 대기 상한 {self.sl_wait_max_s:.1f}초 초과 — "
                    f"정지선을 무시하고 {full}단으로 선다 (sl={self._sl_px_txt()})")
            return full, '대기 상한'
        gate = self._near_gate() * self.sl_override_gate_ratio
        if self._near_metric(self.last_boxes) >= gate:
            if self.stop_level < full:
                self.get_logger().warn(
                    "🚦 신호등이 코앞이다(근접도 ≥ 게이트×"
                    f"{self.sl_override_gate_ratio:.1f}) — 정지선을 기다리지 않는다")
            return full, '신호등 코앞'
        if not self._sl_present(now):
            if self.sl_engaged:
                self.get_logger().info(f"🛑 정지선을 놓쳤다 — 이미 선 위다, 즉시 {full}단")
                return full, '정지선 놓침'
            return full, '정지선 없음'
        if not self._sl_confirmed(now):
            # 스친 마스크 하나로 정지를 미루지 않는다 — 확정 전에는 종전 동작이다.
            return full, '정지선 미확정'
        self.sl_engaged = True
        if self.sl_px <= self.sl_brake2_px:
            return full, '정지선 앞'
        if self.sl_px <= self.sl_brake1_px:
            return self.brake_level_pre, '예비제동'
        return 0, '대기'

    def _sl_px_txt(self):
        """로그·HUD 에 쓰는 정지선 거리 문자열. 미검출이면 '--'."""
        if self.sl_px < 0.0:
            return '--'
        return f"{self.sl_px:.0f}px{self.cam.m_txt(self.sl_px)}"

    # ══════════════════════════════════════════════════════════════════════════
    #  작동 — 정지 지령
    # ══════════════════════════════════════════════════════════════════════════
    def cb_drive_state(self, msg: String):
        self.drive_state   = str(msg.data).strip()
        self.drive_state_t = time.time()

    def cb_tl_enable(self, msg: Bool):
        """master 의 '신호등 인지' 체크박스. ★사람이 직접 켠 허락★ 이다."""
        self.tl_enable   = bool(msg.data)
        self.tl_enable_t = time.time()

    def cb_tl_permit(self, msg: Bool):
        """driving 의 허락. ★코드 내부 상수(TRAFFIC_LIGHT_ENABLE) + DRIVE_RUN★ 이다.
        매핑(MAP_*)·IDLE 에서는 driving 이 False 를 보낸다 — 그쪽에서 이미 걸러진다."""
        self.tl_permit   = bool(msg.data)
        self.tl_permit_t = time.time()

    def cb_fake_box_h(self, msg: Float32):
        """★시험용★ 빨간 박스 높이를 바깥에서 준다. 0 이면 주입을 끈다."""
        new = max(0.0, float(msg.data))
        if (new > 0.0) != (self.fake_box_h > 0.0):
            self.get_logger().warn(
                f"🧪 신호등 주입 {'ON' if new > 0.0 else 'OFF'} "
                f"(박스높이 {new:.0f}px) — ★인지는 시험되지 않는다★")
        self.fake_box_h = new

    def _fake_red_box(self, bh, frame_area):
        """주입용 빨간 박스 한 개 — _all_tl_boxes 가 내는 것과 ★같은 모양★ 이어야 한다.

        가로형 4구 신호등의 종횡비(≈4)를 써서 폭을 정한다. 좌표는 HUD 표시용이라
        판정에 안 쓰이지만, 없는 값을 두면 그리는 쪽에서 터진다.
        """
        bh = max(1, int(round(bh)))
        bw = max(1, bh * 4)
        x1, y1 = 40, 20
        return {'label': 'RED', 'conf': 1.0, 'box': (x1, y1, x1 + bw, y1 + bh),
                'box_h': bh,
                'area_frac': (bw * bh) / float(frame_area) if frame_area else 0.0,
                'hsv_red': 9999, 'hsv_green': 0}

    def _permitted(self, now):
        """지금 이 차에 손을 대도 되는가 (안전 규약 ②) — 둘 중 하나면 된다.

          · 사람이 master 에서 '신호등 인지' 를 켰다 (/tl_enable)  ★체크되어 있으면 상시★
          · driving 이 허락했다 (/tl_permit) = TRAFFIC_LIGHT_ENABLE 이면서 DRIVE_RUN

        둘 다 ★신선할 때만★ 인정한다 — 발행하던 노드가 죽어 마지막 True 가 굳어 있는
        것을 허락으로 읽으면, 카메라만 살아 있는 상태에서 차가 영영 물려 있게 된다.
        """
        if not self.require_permission:
            return True
        if self.tl_enable and (now - self.tl_enable_t) <= TL_ENABLE_STALE_S:
            return True
        if self.tl_permit and (now - self.tl_permit_t) <= TL_ENABLE_STALE_S:
            return True
        return False

    def tick(self):
        now = time.time()

        if not self._permitted(now):
            # 허락이 사라졌다(체크 해제·주행 종료) — 걸어 둔 것이 있으면 풀고 손을 뗀다.
            if self.stopping:
                self.get_logger().warn(
                    "신호등 정지 해제 — 개입 허락이 없다"
                    f"(tl_enable={self.tl_enable} tl_permit={self.tl_permit} "
                    f"drive_state={self.drive_state or '없음'})")
            self._set_stop_level(0, '허락 없음')
            self._apply_brake(self.brake_release_level)
            # ★요구도 반드시 0 으로 내린다★ 여기서 그냥 return 하면 /tl_brake_req 가
            #   끊길 뿐이라, master 는 낡음 판정(TL_REQ_STALE_S)까지 ★마지막 2단을 계속
            #   주장★ 한다 — 체크를 껐는데 1초 더 물려 있는 꼴이 된다.
            self.pub_req.publish(Int32(data=0))
            self._reset_stop_line_wait()
            self._publish_sl(now)
            return

        # ══════════════════════════════════════════════════════════════════════
        #  ① 해제 판정이 먼저다 — ★무는 것과 놓는 것은 서로 배타적이다★
        #     _red_confirmed 는 'tl_state 가 지금 RED', _red_gone 은 'red_release_hold_s
        #     (0.5초) 동안 RED 를 못 봄'
        #     이라 동시에 참이 될 수 없다. 그래서 순서가 판정을 바꾸지 않는다.
        #     ★[2026-08-19] 이 블록을 앞으로 뺐다★ 종전에는 elif 사슬이라 '물고 있는
        #     동안'에만 해제를 봤는데, 1단 예비제동이 생기면서 '물고 있으면서도 계속
        #     계획해야 하는' 구간이 생겼다. 사슬을 그대로 두면 1단 중에 빨간불이
        #     사라져도 영영 못 놓는다.
        # ══════════════════════════════════════════════════════════════════════
        if self.stopping:
            if self.stop_latch:
                # 래치 ON(선택) — 초록불을 확정해야 놓는다. 근접하면 신호등이 ROI 를
                # 벗어나 UNKNOWN 이 되는 것이 정상이라, RED 가 안 보인다고 풀면 차가
                # 굴러간다.
                if self._green_confirmed(now):
                    self.get_logger().info("🟢 초록불 확정 — 리니어 해제, 주행 재개")
                    self._set_stop_level(0, '초록불')
            elif self._red_gone(now):
                self._release_on_red_gone(now)

        # ══════════════════════════════════════════════════════════════════════
        #  ② 이번 틱의 단계를 계획한다 — ★2단을 물기 전까지는 매 틱 다시 본다★
        #     1단으로 예비제동 중이면 정지선이 다가오는 것을 계속 봐야 2단으로 올릴 수
        #     있다. 2단에 도달하면 더 볼 것이 없으므로 계획을 멈춘다(위 해제만 남는다).
        # ══════════════════════════════════════════════════════════════════════
        if self.stop_level < self.brake_level:
            if self._red_confirmed(now):
                level, why = self._stop_plan(now)
                if level <= 0:
                    # ★참는 경우★ 빨간불은 확정됐지만 정지선이 아직 멀다. 참는 동안
                    #   이 노드는 아무것도 내지 않는다 — /tl_brake_req 는 아래에서 0 으로
                    #   나가고, 차는 원래 명령의 주인이 몰던 대로 간다.
                    #   ★이미 1단을 물고 있으면 '대기'가 아니다★ 단조 증가 규약 때문에
                    #   단계를 내리지 않으므로 그대로 1단으로 간다. 그때 sl_wait 를
                    #   True 로 내면 CSV 판정(4-2 '대기 중 brake>0 은 0행')이 거짓으로
                    #   불합격이 된다 — 기록의 뜻이 흐려지는 쪽이 더 나쁘다.
                    self.sl_wait = (self.stop_level == 0)
                    if self.sl_wait:
                        self.get_logger().info(
                            f"🚦⏸ 빨간불 확정 — 정지선 대기 중 "
                            f"(sl {self._sl_px_txt()} → {self.sl_brake1_px:.0f}px, "
                            f"{now - self.red_conf_t:.1f}s/{self.sl_wait_max_s:.1f}s)",
                            throttle_duration_sec=1.0)
                else:
                    self.sl_wait = False
                    self._set_stop_level(level, why)
            elif not self.stopping:
                # RED 확정이 아니면 대기의 근거 자체가 없다 — 기준점과 래치를 지운다.
                # (지우지 않으면 다음 교차로의 대기 상한이 이미 소진된 채로 시작한다)
                #   ★물고 있는 동안에는 지우지 않는다★ 1단으로 물고 있는데 RED 가 한
                #   프레임 흔들렸다고 상한 기준점을 초기화하면, 상한이 영영 안 차서
                #   '정지선 앞에 멈춰 선 채 2단으로 못 올라가는' 구간이 생긴다.
                self._reset_stop_line_wait()

        # ══════════════════════════════════════════════════════════════════════
        #  ③ 발행 — 이번 틱에 정한 단계를 그대로 낸다
        # ══════════════════════════════════════════════════════════════════════
        # 브레이크 요청은 매 틱 같은 값을 내도 안전하다(_apply_brake 가 변화분만 발행).
        # ★내 '요구'를 따로 알린다★ master 가 이것을 자기 레버값과 max 로 합쳐서
        #   /brake_level 을 내므로, 두 발행자가 같은 값을 내게 되어 다툼이 사라진다.
        #   ★[2026-08-19] 고정 2단이 아니라 지금 단계를 낸다★ 1단 예비제동도 그대로
        #   흘러간다 — 소비자(master·driving)는 max 합산이라 손댈 것이 없다.
        self.pub_req.publish(Int32(data=int(self.stop_level)))
        self._publish_sl(now)

        if self.stopping:
            self._apply_brake(self.stop_level)
            # /cmd_vel_raw 는 ★2단에서만★ 낸다 — 풀리는 즉시 토픽의 주인을 driving 에게
            # 돌려주기 위해서다(파일 헤더의 발행자 충돌 설명 참고).
            # ★[2026-08-19] 1단(예비제동) 중에는 내지 않는다★ 그때 차는 아직 굴러가는
            #   중이고 조향은 driving 의 몫이다. 여기서 조향 0 을 내면 정지선까지 가는
            #   동안 차가 제 코스를 못 따라간다.
            if self.publish_cmd_vel and self.stop_level >= self.brake_level:
                # ★펄스 0 · 조향 0★ — 펄스는 arduino 가 브레이크 때문에 어차피 0 으로
                #   덮지만, 여기서도 명시적으로 0 을 내야 브레이크를 푸는 순간
                #   직전 주행값이 되살아나지 않는다. 조향 0 은 정지 중 바퀴를 일직선
                #   으로 두기 위한 것이다(driving 이 종점 정지에서 하는 것과 같다).
                out = Twist()
                out.linear.x  = 0.0
                out.angular.z = 0.0
                self.pub_cmd.publish(out)
        else:
            self._apply_brake(self.brake_release_level)

    def _release_on_red_gone(self, now):
        """★기본 동작 [2026-08-14 지시]★ 빨간불을 보는 동안만 잡는다.

        RED 스트릭이 끊기면(해제 유예 red_release_hold_s 초과) 곧바로 놓는다. 놓는
        순간 하는 일은 /brake_level=0 하나뿐이고, 그 다음은 원래 명령의 주인
        (driving 20Hz / arduino 캐시의 master 레버)이 알아서 이어간다 — 이 노드는
        펄스를 기억하지도 복원하지도 않는다.

        ★[2026-08-14] 해제 조건에서 '초록불 확정' 을 뺐다 — 이것이 왕복의 정체다★
          로스백(manual-20260814_151206) 에서 브레이크가 ★0.10초 주기로 2↔0★ 을
          50번 반복했다. mode·board·estop 은 전부 정상이었다.
          원인 : 해제 조건이 (RED 를 red_release_hold_s 동안 미감지) ★또는★
                 (GREEN 확정) 이었다.
          RED 와 GREEN 이 프레임마다 번갈아 잡히면(가로형 4구에서 적색등과 좌회전
          녹색등이 함께 보이거나, 모델이 두 클래스를 오갈 때) ★두 스트릭이 동시에
          살아 있어★ 매 틱 '물어라(RED 확정)' 와 '놓아라(GREEN 확정)' 가 같이 참이
          된다 → 틱 주기로 flip-flop.
          ★이제 해제 근거는 하나뿐이다 — 빨간불을 red_release_hold_s 동안 못 봄★
          초록불이 보인다는 것은 곧 빨간불이 없다는 뜻이므로, 그 유예(0.5초)가 지나면
          어차피 풀린다. 판정 근거를 하나로 줄이면 왕복이 구조적으로 불가능해진다.
        """
        self.get_logger().info(
            f"⚪ 빨간불 {self.red_release_hold_s:.1f}초 미감지 — 리니어 해제"
            + ("  (초록불 확정 상태)" if self._green_confirmed(now) else ""))
        self._set_stop_level(0, '빨간불 사라짐')

    def _reset_stop_line_wait(self):
        """대기 상태를 이번 접근분까지 통째로 지운다(다음 교차로를 위해)."""
        self.sl_wait    = False
        self.sl_engaged = False
        self.red_conf_t = None

    def _publish_sl(self, now):
        """정지선 진단 세 개. ★sl_px 만 판정에 쓰인다★ — record 가 CSV 로 받아 적어
        '왜 여기서 1단을 물었나 / RED 인데 왜 아직 안 섰나'를 사후에 판정하게 한다."""
        fresh = self._sl_present(now)
        # ★−1 = 미검출★ 이라는 뜻을 하나로 유지하려고 ★0 밑으로는 내보내지 않는다★.
        #   정지선이 범퍼를 지나면 실제 값은 음수가 되는데(이미 선 위다), 그것을 그대로
        #   내면 −1 이 '미검출'인지 '1px 지났다'인지 알 수 없게 된다. 판정에는 부호가
        #   살아 있는 원값(self.sl_px)을 쓰므로 제어는 영향받지 않는다.
        self.pub_sl_px.publish(Float32(
            data=float(max(0.0, self.sl_px) if fresh else -1.0)))
        self.pub_sl_y.publish(Float32(
            data=float(self.sl_y if fresh else -1.0)))
        self.pub_sl_wait.publish(Bool(data=bool(self.sl_wait)))

    @property
    def stopping(self):
        """이 노드가 지금 브레이크를 물고 있는가(1단이든 2단이든).

        ★[2026-08-19] bool 상태를 단계로 바꾸면서 남긴 이름이다★ 해제 경로·로그·
        _sl_should_run 이 '물고 있는가'만 물어보므로 그쪽은 안 고쳐도 되게 했다.
        '몇 단인가'가 필요한 곳만 stop_level 을 직접 본다.
        """
        return self.stop_level > 0

    def _set_stop_level(self, level, why=''):
        """정지 단계를 정한다. 실제 발행은 tick() 과 _apply_brake() 가 한다.

        ★단조 증가★ RED 를 잡고 있는 동안에는 올라가기만 한다(파일 헤더의 규약).
        0 으로 내리는 것은 ★해제★ 를 뜻하고, 그 길은 호출자 셋뿐이다 —
        허락 상실 · 빨간불 사라짐 · (래치 ON 이면) 초록불 확정.
        """
        level = max(0, min(2, int(level)))
        if level > 0 and level < self.stop_level:
            return                      # 내리는 요청은 무시한다(왕복 금지)
        if level == self.stop_level:
            return
        prev, self.stop_level = self.stop_level, level
        self.stop_why = why
        if level <= 0:
            self._reset_stop_line_wait()
            return
        # ★어느 근거로 몇 단을 물었는지 로그에 남긴다★ 정지 지점이 이상할 때
        #   '정지선을 보고 선 것인지, 못 봐서 그 자리에서 선 것인지'가 첫 질문이다.
        head = ("🚦🟡 빨간불 확정 — 예비제동" if level < self.brake_level
                else "🚦🛑 빨간불 확정 — 정지")
        self.get_logger().warn(
            f"{head} [{why}] (리니어 {prev}단 → {level}단, 정지선 {self._sl_px_txt()})"
            + (" + /cmd_vel_raw 0/0"
               if (self.publish_cmd_vel and level >= self.brake_level) else ""))

    def _apply_brake(self, level):
        """리니어 단계를 요청한다 — ★값이 바뀔 때만★ 발행한다.

        (안전 규약 ③) 해제(0단)는 우리가 직접 건 적이 있을 때만 낸다. brake_now 가
        None 이면 이 노드는 브레이크의 주인이 아니므로 아무것도 내지 않는다 —
        driving 이 DRIVE_DONE 에서 물고 있는 2단을 우리가 푸는 사고를 막는다.

        ★[2026-08-14] 이 가드만으로는 부족한 구간이 하나 더 있다★ white1 의 종점
        접근제동(goal_phase=BRAKE)은 ★DRIVE_RUN 상태에서★ 리니어를 문다 — 여기서는
        DRIVE_DONE 이 아니라 걸러지지 않는다. 그래서 driving 쪽에 ★자기 브레이크
        재확인(keepalive)★ 을 두었다: driving 은 0 이 아닌 단계를 물고 있는 동안
        0.5초마다 같은 값을 다시 발행한다(driving.set_brake). 우리가 낸 0 이 그 구간에
        섞여도 driving 이 곧바로 되돌린다.
        """
        level = max(0, min(2, int(level)))
        if level == 0:
            if self.brake_now in (None, 0):
                return          # 건 적이 없다 = 풀 것도 없다
            if self.drive_state == DRIVE_DONE_STATE:
                # driving 이 도착·경로이탈로 스스로 2단을 물고 있는 구간이다.
                # 여기서 0 을 내면 남의 정지를 푸는 것이 된다 — 소유권만 넘긴다.
                self.get_logger().warn("리니어 해제 보류 — driving 이 DRIVE_DONE 으로 물고 있다")
                self.brake_now = None
                return
        if level == self.brake_now:
            # ★재확인 [2026-08-14]★ 값이 같아도 물고 있는 동안은 주기적으로 다시 낸다.
            #   /brake_level 은 마지막 발행자가 이기는 '명령' 토픽인데 master 가
            #   KEEPALIVE_S(0.5s)마다 자기 레버값(0단)을 재발행한다 — 그대로 두면
            #   2단을 건 0.5초 뒤 리니어가 도로 풀린다(실측 로그).
            if level > 0 and (time.time() - self._brake_t) >= BRAKE_KEEPALIVE_S:
                self._brake_t = time.time()
                self.pub_brake.publish(Int32(data=level))
            return
        self.brake_now = level
        self._brake_t = time.time()
        self.pub_brake.publish(Int32(data=level))
        self.get_logger().info(
            f"🛑 리니어 브레이크 {level}단 "
            f"({'체결 — 신호등 정지' if level > 0 else '해제'})")

    def status_tick(self):
        now = time.time()
        #  ★주입이 켜져 있으면 계속 떠든다★ 조용히 켜져 있는 것이 제일 위험하다.
        if self.fake_box_h > 0.0:
            self.get_logger().warn(
                f"🧪 신호등 주입 모드 ON ({self.fake_box_h:.0f}px) — 실차 금지",
                throttle_duration_sec=5.0)
        if self.model is None:
            self.get_logger().error("⛔ 신호등 모델이 없다 — 이 노드는 정지를 걸지 않는다",
                                    throttle_duration_sec=10.0)
            return
        if self.img_time == 0.0:
            self.get_logger().warn(f"⏳ {self.image_topic} 수신 대기 (usb_cam 미기동?)",
                                   throttle_duration_sec=5.0)
        elif (now - self.img_time) > 2.0:
            self.get_logger().warn(
                f"⛔ {self.image_topic} {now - self.img_time:.1f}s 두절 — 카메라 확인!"
                + ("  ※ 정지 상태는 래치로 유지된다(수동조종으로 내리면 풀린다)"
                   if self.stopping else "  ※ 신호등 개입은 하지 않는다"),
                throttle_duration_sec=5.0)
        if self.stopping:
            head = ("🟡 신호등 예비제동 중" if self.stop_level < self.brake_level
                    else "🛑 신호등 정지 유지 중")
            self.get_logger().info(
                f"{head} [{self.stop_why}] {self.stop_level}단 | tl={self.tl_state} "
                f"raw={self.last_raw} drop={self.last_red_drop} fps={self.fps:.1f}"
                + (f" sl={self._sl_px_txt()}" if self._sl_present(now) else " sl=없음"),
                throttle_duration_sec=2.0)

    def shutdown_release(self):
        """내려갈 때 리니어를 반드시 풀고 간다.

        arduino 는 마지막 /brake_level 을 캐시로 물고 있다 — 이 노드가 2단을 건 채
        죽으면 ★차가 영영 못 움직인다★(수동조종으로 D5 를 내리면 arduino 가 캐시를
        지워 주는 것이 유일한 탈출구다). 발행 직후 프로세스가 끝나면 DDS 가 아직
        못 내보냈을 수 있으므로 잠깐 스핀해서 실제로 나가게 한다.

        ★이게 되려면 rclpy 컨텍스트가 아직 살아 있어야 한다★ — main() 이 rclpy 의
        기본 시그널 핸들러를 끄는 이유가 이것이다(아래 참고).
        """
        try:
            if self.brake_now not in (None, 0):
                self.pub_brake.publish(Int32(data=0))
                self.get_logger().info("🛑 종료 — 리니어 해제(0단) 발행")
                for _ in range(10):
                    rclpy.spin_once(self, timeout_sec=0.03)
        except Exception as e:
            # 여기까지 왔는데 실패하면 리니어가 물린 채 남는다 — 조용히 넘기지 않는다.
            print(f"⛔ 종료 시 리니어 해제 발행 실패: {e} "
                  f"— 수동조종(D5)으로 내리면 arduino 가 브레이크 요청을 지운다")
        if self.show_window:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass


def _install_shutdown_signals():
    """SIGINT/SIGTERM 을 ★파이썬 예외★ 로 받는다.

    ★왜 rclpy 기본 핸들러를 쓰지 않는가★ rclpy 의 기본 SIGINT 핸들러는 컨텍스트를
    먼저 내려버린다. 그러면 spin() 이 풀린 뒤 finally 에서 /brake_level=0 을 내려 해도
    'publisher's context is invalid' 로 실패한다 — ★리니어가 2단으로 물린 채 노드만
    사라진다★(실측으로 확인했다: 종료 로그에 해제가 찍히지 않았다).
    그래서 rclpy 핸들러를 끄고(SignalHandlerOptions.NO) 파이썬 기본 동작
    (SIGINT → KeyboardInterrupt)을 쓴다. launch 가 보내는 SIGTERM 도 같은 예외로
    바꿔 준다 — 그래야 두 경로 모두 shutdown_release() 를 지나간다.
    """
    import signal

    def _term(_sig, _frm):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _term)


def main(args=None):
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init(args=args)      # 구버전 rclpy 폴백
    _install_shutdown_signals()

    node = TrafficLight()
    # ★[2026-08-14] 단일 스레드 spin 을 버렸다 — 제어 틱이 추론에 굶으면 안 된다★
    #   영상 콜백은 한 프레임에 YOLO 추론 + 그리기 + imshow 를 한다(1920x1080).
    #   단일 스레드 실행기에서는 그동안 tick() 이 대기하는데, tick 은 ★브레이크를
    #   유지·해제하는 곳★ 이다. 30fps 를 겨우 맞추는 상태에서 GPU 나 창 합성이 한 번
    #   밀리면 브레이크 재확인과 /tl_brake_req 발행이 함께 늦어지고, 그것을 소비자가
    #   '요구 없음'으로 읽으면 리니어가 풀린다 — 실차에서 본 왕복의 한 경로다.
    #   ★두 콜백 그룹을 나눠 서로를 막지 못하게 한다★(생성자에서 그룹을 지정했다).
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    #  ★spin 은 배경 스레드, 창은 메인 스레드★ [2026-08-14]
    #    OpenCV HighGUI 는 스레드 안전하지 않다. 실행기 워커에서 imshow 를 부르면 그
    #    스레드가 GTK 안에서 멎고, 그 스레드가 맡은 ★영상 콜백이 영영 안 돌아★
    #    '/image_raw 두절' 로 나타난다(실측으로 확인한 회귀다). 그래서 표시만 메인
    #    스레드로 뺀다 — master.py·prompt_g.py 가 지키는 규약과 같다.
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        while rclpy.ok():
            node.show_pending()
            time.sleep(0.02)          # 표시 주기 ≈50Hz 상한(그릴 것이 없으면 즉시 반환)
    # ★[2026-09-04] ExternalShutdownException 도 받는다★ launch 가 내려갈 때
    #   rclpy 의 신호 처리기가 컨텍스트를 먼저 닫으면 spin 은 KeyboardInterrupt 가
    #   아니라 이것을 던진다. 안 받으면 노드마다 트레이스백을 십수 줄 쏟아, 정작
    #   봐야 할 종료 로그를 밀어낸다(구독/발행 노드가 종료에 실패할 일은 없으므로
    #   그 트레이스백의 정보량은 0 이다). 원인 로그: gps 의 `rcl_shutdown already
    #   called` RCLError — 그것은 아래 rclpy.ok() 가드가 막는다.
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.shutdown_release()
        node.destroy_node()
        if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
