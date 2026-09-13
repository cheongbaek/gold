#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""camera_launch.py — ★카메라·신호등 런치 조각의 단일 소유자★ [white1]

one_launch.py(자율주행)와 master.launch.py(수동 계측)가 ★같은 카메라 구성★ 을 쓴다.
두 곳에 복사해 두면 반드시 갈라지므로(usb_cam 파라미터·v4l2 보정 스크립트가 길다)
여기 한 곳에서 만들어 준다. paths.py 가 저장 위치의 단일 소유자인 것과 같은 태도다.

    args, actions = camera_launch.build(cam_dev, cam_format, NODE_ENV, RESPAWN_DELAY)

띄우는 것 (use_camera 로 함께 켜고 끈다):
    usb_cam       V4L2 → /image_raw   ★원본 그대로다(보정하지 않는다 — 아래 절)★
    usb_cam_ctrl  기동 2초 뒤 v4l2-ctl 로 노출·게인 등을 ★장치 범위로 클램프해★ 재적용
    traffic_light /image_raw → 빨간불이면 /brake_level (white1/traffic_light.py)
                  ★[2026-08-24] 인지 결과 창이 계기판이 됐다★ 한글 HUD 3줄 + 우측
                  BEV·게이지 패널. tl_window_width 는 이제 '창 폭' 이 아니라
                  ★카메라 뷰 폭★ 이다(자세한 것은 traffic_light.py 헤더).
                  ★[2026-09-08] 판단을 단순화했다 — 근거 둘, 단계 하나★
                  BEV 발화선(sl_trigger_bev_y)에 정지선이 닿거나, 신호등 박스가
                  tl_solo_stop_min_height 를 넘거나 — ★먼저 성립하는 쪽★ 에서
                  풀브레이크. 1단 예비제동과 대기 상한은 없앴다.
    tl_video      ★[2026-09-10] 인지 디버그 화면을 주행 내내 mp4 로 적는다★
                  nxde 의 video 노드를 /tl/debug_image 에 붙인 것이다(아래 절).

────────────────────────────────────────────────────────────────────────────────
 ★디버그 녹화 — 창을 안 띄우고도 '무엇을 보고 판단했나' 가 남는다 [2026-09-10]★
────────────────────────────────────────────────────────────────────────────────
실차에서 신호등이 왜 섰는지/왜 안 섰는지는 ★그 순간 화면★ 이 없으면 못 따진다.
그런데 인지 결과 창(cv2 imshow)은 실차에서 켜 두기 나쁘다 — 화면이 없는 터미널에서는
아예 못 열고, 열리더라도 창 합성이 메인 스레드를 잡는다. 그래서 둘을 갈랐다:

    tl_show_window   창을 띄운다 (★기본 true★ [2026-09-10 재조정] — 아래 참고)
    tl_record_video  같은 그림을 파일로 적는다 (★기본 true★ — 항상 남긴다)

    ⚠️ [2026-09-10] 처음엔 tl_show_window 기본을 false 로 내렸었다 — "녹화가 있으니
    창은 필요 없다" 는 논리였다. 그런데 실제로 화면을 보며 튜닝하는 자리에서는
    ★매번 :=true 를 붙이는 것 자체가 불편하고★, 창을 못 켜는 헤드리스 환경(SSH)만
    false 로 내리면 되므로 ★기본은 다시 true 로 되돌렸다★. 녹화(tl_record_video)는
    그와 무관하게 항상 켜져 있으니 둘 다 켜 두는 쪽이 아무것도 잃지 않는다 —
    화면이 있는 실차에서는 창이 뜨는 것이 기본, 없으면 tl_show_window:=false.

★같은 캔버스다.★ traffic_light 의 _draw() 가 그린 한 장(YOLO 박스·ROI 음영·BEV
사다리꼴·정지선·게이지 패널·한글 HUD 3줄)을 창에도 띄우고 토픽으로도 낸다 —
즉 ★녹화 영상에 창과 똑같은 것이 들어 있다★. 창을 껐다고 정보가 줄지 않는다.

  · 발행 : traffic_light 의 tl_publish_debug → /tl/debug_image (BEST_EFFORT)
  · 녹화 : nxde 의 video 노드가 그 토픽을 구독해 mp4 로 적는다
  · 위치 : white1/paths.py 의 video_dir() = <src>/white1/video/tl-<날짜>_<시각>.mp4

★스위치가 ★하나★ 인 이유 (tl_record_video)★ 발행과 녹화를 따로 열면 짝이 어긋나
'발행 안 하는 토픽을 녹화' = ★0바이트 파일★ 이 나온다. 한 인자가 둘을 함께 켠다.

★범위★ — 런치가 뜬 순간부터 내려갈 때까지다(주행 구간만 고르지 않는다). 주행 전
대기·헤딩 초기화까지 통째로 들어가는 편이 사후 판정에 낫고, 무엇보다 '언제부터
적을지' 를 판단하는 로직이 없어야 그 로직이 틀릴 일도 없다. 런치가 SIGTERM 을
보내면 video 노드가 파일을 정상으로 닫는다(nxde/video.py 의 신호 처리).

⚠️ ★비용★ 캔버스는 원본(1920x1080)이 아니라 tl_window_width 기준이다 —
   기본 960 이면 ★1248x610★ (2.28MB/프레임, 30fps 에서 68MB/s · 파일 분당 15~25MB).
   /image_raw 가 이미 187MB/s 흐르고 있으므로 그 위에 +36% 다. 인지 FPS 가 눈에 띄게
   떨어지면 tl_video_scale:=0.5 (용량·인코딩 비용 약 1/4) 로 내린다.
⚠️ ★디스크★ video 노드는 남은 용량이 500MB 밑이면 ★스스로 녹화를 끝낸다★(파일은
   정상으로 닫는다). 그래도 <src>/white1/video 는 주행할수록 쌓이므로 가끔 비운다.
   .gitignore 가 *.mp4 · *.avi 를 막고 있어 이력에는 안 들어간다.

────────────────────────────────────────────────────────────────────────────────
 ★카메라 기하는 camera_params() 한 벌로 나간다 [2026-08-19]★
────────────────────────────────────────────────────────────────────────────────
어안 왜곡보정 계수와 BEV 사다리꼴은 white1/camera_model.py 가 소유하고, 그 파라미터를
★여기서 한 벌 만들어 카메라 인지 노드 전부에 먹인다★(지금은 traffic_light 하나뿐이고,
차선 인지가 붙으면 같은 dict 를 그대로 넘긴다). 그래서 '카메라 노드를 띄우면 보정이
기본으로 적용된다'가 런치 한 곳에서 성립한다.

⚠️ ★/image_raw 자체는 보정하지 않는다★ usb_cam 에 camera_info_url 을 주지 않고,
   보정은 인지 노드가 프레임을 받은 뒤에 한다. 원본 녹화(nxde video)가 '카메라가 실제로
   준 그림'을 봐야 하기 때문이다 — 녹화된 mp4 에 어안이 남아 있는 것이 ★정상★ 이다.

⚠️ ★[2026-08-24] ROI·색 임계·BEV 사다리꼴처럼 눈으로 맞추는 값은 cam_testbed 소관★
   이 워크스페이스는 그 결과(캘리브 yaml·bev_src_pts·두 문턱 등)를 받아 아래 기본값에
   반영하고 실차에서 검증하는 쪽이다(STOPLINE_TEST.md 참고). 처음부터 이 차에서
   눈으로 잡는 절차가 아니다.

⚠️ ★카메라를 안 꽂은 채 use_camera:=true 로 띄우면 usb_cam 이 respawn 루프를 돈다★
   그 로그가 다른 노드 로그를 덮는다. 카메라를 쓸 일이 없는 날은 use_camera:=false.
"""

from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import AndSubstitution, LaunchConfiguration
from launch_ros.actions import Node

#  ★저장 위치는 paths.py 가 소유한다★ 여기에 리터럴을 적지 않는다(CLAUDE.md 7절).
from white1 import paths

#  카메라 해상도 — ports.resolve_camera_format() 이 '이 해상도에서 실제로 나오는
#  포맷'을 보고 pixel_format 을 정하므로, 이 값을 바꾸면 포맷도 따라 바뀐다.
#  ★traffic_light 의 tl_roi 기본값(0,0,1920,540)이 이 해상도 기준이다★ 해상도를
#  낮추면 ROI 는 프레임 크기로 클램프되지만(_clamp_roi), 근접도 게이트
#  tl_red_stop_min_height(25px)는 화면이 작아진 만큼 같이 낮춰야 한다.
CAM_WIDTH, CAM_HEIGHT = 1920, 1080

#  usb_cam 기동 뒤 v4l2-ctl 을 다시 거는 시각 [s]. 드라이버가 노드 파라미터를
#  조용히 무시하는 경우가 있어(See3CAM 실측) 한 번 더 못 박는다.
V4L2_APPLY_DELAY_S = 2.0


def declare_args(cam_dev):
    """카메라·신호등 런치 인자. 두 런치가 같은 이름·같은 기본값을 쓴다."""
    return [
        DeclareLaunchArgument(
            'use_camera', default_value='true',
            description='usb_cam + traffic_light 를 함께 띄울지. ★카메라를 안 꽂았으면 '
                        'false★ — usb_cam 이 respawn 루프를 돌며 로그를 덮는다'),
        DeclareLaunchArgument(
            'video_device', default_value=cam_dev,
            description='카메라 V4L2 경로 override. 기본값은 ports.resolve_camera() 가 '
                        '실제로 열어 프레임을 확인한 장치다(내장·적외선 노드는 제외)'),
        DeclareLaunchArgument(
            'cam_exposure', default_value='120',
            description='노출(exposure_time_absolute). 장치가 보고하는 범위로 클램프된다'),
        DeclareLaunchArgument(
            'tl_device', default_value='cuda:0',
            description='YOLO 추론 장치. GPU 가 없으면 cpu (그만큼 느려진다)'),
        DeclareLaunchArgument(
            'tl_conf', default_value='0.35',
            description='YOLO 신뢰도 임계. 올리면 오검출이 줄고 놓치는 것이 는다'),
        DeclareLaunchArgument(
            'tl_red_stop_min_height', default_value='25',
            description='★근접도 게이트★ 빨간불 박스 높이가 이 픽셀 이상이어야 정지한다. '
                        '작으면 멀리서 서고, 크면 늦게 선다(1920x1080 기준). ⚠️ 상한이 '
                        '있다 — 너무 크면 서 있는 동안 등기구가 화면 위로 벗어나 해제되어 '
                        '차가 굴러간다. 상한은 gold/tl_tune.py 가 기록으로 계산해 준다'),
        DeclareLaunchArgument(
            'tl_near_release_ratio', default_value='0.7',
            description='★근접도 히스테리시스★ 이미 물고 있는 동안에는 위 임계를 이 비율로 '
                        '낮춰서 본다. 같은 신호등이 인식 흔들림으로 몇 px 작아졌다고 놓으면 '
                        '리니어가 왕복한다. 1.0 이면 히스테리시스 없음'),
        DeclareLaunchArgument(
            'sl_enable', default_value='true',
            description='★정지선 앞 정지★ 켜면 빨간불이 확정돼도 정지선이 보이는 동안은 '
                        '개입하지 않고, 정지선이 BEV 발화선(sl_trigger_bev_y)에 닿을 때 '
                        '풀브레이크로 선다. 정지선을 못 보면 신호등 단독 문턱'
                        '(tl_solo_stop_min_height)만 남는다. false 면 정지선 추론 자체를 '
                        '안 돌린다 — 그때는 신호등 크기 하나로만 판단한다'),
        DeclareLaunchArgument(
            'sl_trigger_bev_y', default_value='40.0',
            description='★정지선 발화선★ BEV 에서 정지선 최근접점의 행이 이 값에 닿으면 '
                        '★풀브레이크★ 다. 정지선은 BEV 를 위에서 아래로 지나가므로'
                        '(멀면 y 가 작고 사다리꼴 윗변보다 더 멀면 음수다), 40 은 '
                        '★정지선이 BEV 상단에 잡히는 순간★ 을 뜻한다. '
                        '★[2026-09-08] 440.0 → 40.0 — 디버그 영상을 보고 사람이 정했다★ '
                        '440(=BEV 밑변 480 바로 위, 범퍼 앞 1.2m)은 전수 시험에서 거의 '
                        '안 쓰였다 — 가까워지면 등기구가 ROI 위로 벗어나 RED 확정이 먼저 '
                        '풀리거나 신호등 단독 문턱이 먼저 걸렸다(접근 16회 중 정지선 앞 '
                        '정지 1회). 40 이면 정지선이 보이기 시작하는 순간 판정이 선다. '
                        '⚠️ bev_src_pts 를 바꾸면 이 값도 같이 무효다 — 행의 뜻이 바뀐다. '
                        '⚠️ 값에 ★소수점을 붙여야 한다★(40 이 아니라 40.0) — 런치 인자는 '
                        '문자열을 그대로 형변환하므로 정수로 주면 노드 선언 타입(double)과 '
                        '어긋나 기동에 실패한다(tl_conf 등 기존 인자와 같은 성질이다)'),
        DeclareLaunchArgument(
            'tl_solo_stop_min_height', default_value='30.0',
            description='★신호등 단독 정지 문턱★ 빨간 박스 높이가 이 픽셀 이상이면 '
                        '풀브레이크다. ★정지선을 보고 있든 아니든 본다(OR 조건)★ — '
                        '정지선이 발화선에 닿는 것과 이 문턱 중 ★먼저 성립하는 쪽★ 이 문다. '
                        '★[2026-09-08] 신설★ 종전에는 정지선을 못 보면 RED 확정 즉시 섰는데, '
                        '그 시점을 정하는 tl_red_stop_min_height 는 「이게 빨간불이 맞는가」를 '
                        '가르는 인지 게이트이지 「설 만큼 가까운가」가 아니다. '
                        '★30.0 은 실측값이다★ — cam_record_video 5편(64,800프레임·접근 16회)의 '
                        '박스높이 최대치 27·27·35·36·38·42×5·43×2·44×2·45×2·46·53px 에서 '
                        '14/16 이 닿는 값이다. ⚠️ ★올리면 안 서는 접근이 늘고, 내리면 정지선 '
                        '앞에 서기 전에 신호등 쪽이 먼저 무는 접근이 는다★ — 그 균형점이다. '
                        '0 이면 단독 정지를 끈다(정지선에 전적으로 의존 — 실차 금지)'),
        DeclareLaunchArgument(
            'cam_undistort', default_value='true',
            description='★어안 왜곡보정★ 카메라 인지 노드가 프레임을 받은 뒤 보정한다. '
                        '계수는 white1/calibration/usb_cam_calibration.yaml. ⚠️ 끄면 BEV '
                        '거리(정지선 판정)를 믿을 수 없다 — 어안이 남은 그림에 원근변환을 '
                        '걸면 직선이 휜 채로 펴진다. /image_raw 원본은 어느 쪽이든 그대로다'),
        DeclareLaunchArgument(
            'bev_src_pts',
            default_value='[640.0, 620.0, 1280.0, 620.0, 1920.0, 1080.0, 0.0, 1080.0]',
            description='★BEV 사다리꼴★ 보정된 화면에서 노면 직사각형에 해당하는 네 점 '
                        '(좌상,우상,우하,좌하). ★[2026-09-07] cam_testbed 재실측값으로 '
                        '갱신했다★ 종전 [730,650,1190,650,1720,1080,200,1080](2026-08-25) '
                        '은 이 값으로 대체됐다. ⚠️ ★이걸 바꾸면 픽셀의 뜻이 통째로 바뀐다★ — '
                        'bev_bumper_y_px 는 옛 사다리꼴 기준이라 ★지금은 무효★ 이고, '
                        '그래서 [2026-09-08] 부터 ★판정에는 안 쓴다★ — 정지선 발화는 '
                        'sl_trigger_bev_y(BEV 행)로 한다. bev_px_to_m 은 갱신됐다'
                        '(camera_model.py). ⚠️ 사다리꼴을 바꾸면 sl_trigger_bev_y 도 무효다'),
        DeclareLaunchArgument(
            'bev_bumper_y_px', default_value='645.0',
            description='★앞범퍼가 BEV 의 몇 번째 행인가 = 거리 0 의 기준★ 범퍼가 사다리꼴 '
                        '밑변보다 앞(차 쪽)이면 bev_h(480)보다 큰 값이 된다. '
                        '★[2026-08-25] 480.0(근거 없는 기본값) → 645.0 실측★ 줄자로 '
                        '앞범퍼→밑변 1.20m 를 재고 중앙열 사영식으로 풀었다 — 선형 근사로는 '
                        '623 이 나오는데 이 BEV 는 그러기엔 너무 비선형이다. '
                        '⚠️ ★이 값이 sl_px 의 도달 범위를 통째로 정한다★ 645 면 BEV 밑변이 '
                        'sl_px=165 이라 그 밑으로는 절대 못 내려가고, 차체 가림까지 넣으면 '
                        '실효 하한이 285 다. 이 값이 틀리면 두 문턱이 통째로 어긋난다'),
        DeclareLaunchArgument(
            'sl_conf', default_value='0.30',
            description='정지선 seg 신뢰도 임계. 구 white 의 lane_conf 와 같은 값'),
        DeclareLaunchArgument(
            'sl_gate_red_s', default_value='1.0',
            description='빨간 박스를 이 시간 안에 본 적이 있을 때만 정지선 추론을 돌린다 '
                        '(평상시 비용 0). ★튜닝할 때는 크게 준다★ — 신호등 없이 정지선만 '
                        '보고 싶으면 sl_gate_red_s:=99999 로 상시 추론시킨다(todo 9-1)'),
        DeclareLaunchArgument(
            'tl_show_window', default_value='false',
            description='인지 결과 창(OpenCV)을 띄울지. ★기본 true★ [2026-09-10 재조정 — '
                        '한때 false 였다가 되돌렸다]. 녹화(tl_record_video)가 같은 그림을 '
                        '파일로도 남기므로 창을 꼭 켜야 하는 것은 아니지만, 화면이 있는 '
                        '실차에서 매번 :=true 를 붙이는 것이 더 불편해 ★기본을 다시 켬으로 '
                        '뒀다★ — 녹화와 완전히 독립이라 둘 다 켜 둬도 서로 방해하지 않는다. '
                        '★화면 없는 터미널(SSH)이면 tl_show_window:=false★ — 안 그러면 '
                        'cv2 가 창을 못 열어 에러를 남긴다'),
        DeclareLaunchArgument(
            'tl_record_video', default_value='false',
            description='★인지 디버그 화면을 주행 내내 mp4 로 적는다 [2026-09-10]★ '
                        '창(tl_show_window)과 ★같은 캔버스★ 다 — YOLO 박스·ROI 음영·'
                        'BEV 사다리꼴·정지선·게이지·한글 HUD 가 전부 들어 있다. '
                        '★이 인자 하나가 둘을 함께 켠다★ : traffic_light 의 '
                        'tl_publish_debug(→ /tl/debug_image) 와 그것을 받아 적는 '
                        'nxde video 노드. 따로 열면 짝이 어긋나 ★0바이트 파일★ 이 '
                        '나오므로 스위치를 하나로 뒀다. 런치가 뜬 순간부터 내려갈 '
                        '때까지 적고(주행 구간만 고르지 않는다), 런치 종료 SIGTERM 에서 '
                        '파일을 정상으로 닫는다. ⚠️ 인지 FPS 가 떨어지면 끄지 말고 '
                        '먼저 tl_video_scale 을 내려 볼 것'),
        DeclareLaunchArgument(
            'tl_video_dir', default_value=paths.video_dir(),
            description='녹화 mp4 저장 폴더. 기본값은 white1/paths.py 의 video_dir() '
                        '= <소스트리>/src/white1/video 다. ★여기서 미리 풀어서 넘긴다★ — '
                        '받는 쪽이 nxde 의 노드라 white1/paths.py 를 모르기 때문이다'
                        '(one_launch 가 sound_dir 을 넘기는 방식과 같다). '
                        '★nxde/video.py 의 자체 기본값에 맡기지 않는 이유★ 그 노드는 '
                        '설치본에서 돌면 ~/nxde_video 로 폴백하는데, 이 워크스페이스는 '
                        '--symlink-install 금지라 ★언제나★ 그쪽으로 떨어진다'
                        '(CLAUDE.md 0.4절) — 즉 영상이 소스트리에서 멀리 쌓인다. '
                        '환경변수 WHITE1_VIDEO_DIR 로도 바꿀 수 있다'),
        DeclareLaunchArgument(
            'tl_video_scale', default_value='1.0',
            description='녹화 배율(0.05~1.0). 1.0 = 캔버스 그대로. 0.5 면 가로세로 절반 '
                        '= 용량·인코딩 비용 약 1/4. ★판정은 이 값과 무관하다★ — 녹화만 '
                        '줄인다. 캔버스는 원본 1920x1080 이 아니라 tl_window_width 기준'
                        '(960 → 1248x610)이므로 대개 1.0 으로 충분하다'),
        #  ★[2026-09-13] 세그먼트 분할 — 녹화가 통째로 날아가지 않게★
        DeclareLaunchArgument(
            'tl_video_segment_s', default_value='120.0',
            description='이 초마다 mp4 를 닫고 다음 조각을 연다(0 = 분할 안 함). '
                        '★mp4 는 인덱스를 파일 끝에 쓰므로 SIGKILL 을 맞으면 파일 '
                        '전체가 재생 불가가 된다★ — 조각으로 나눠 두면 잃는 것이 '
                        '마지막 조각뿐이다. 첫 조각은 종전 이름 그대로이고 이후는 '
                        'tl-<시각>-02.mp4 … 로 이어진다. 분당 15~25MB 이므로 '
                        '120초면 조각당 30~50MB'),
        DeclareLaunchArgument(
            'tl_video_topic', default_value='/tl/debug_image',
            description='녹화할 이미지 토픽. 기본은 인지 디버그 화면이다. '
                        '★/image_raw 로 바꾸면 오버레이 없는 원본이 적힌다★ — 어안 '
                        '왜곡이 남아 있는 카메라 원본이고, 그때 tl_record_video 는 '
                        'tl_publish_debug 도 함께 켜지만 아무도 안 보므로 무해하다 '
                        '(정확히 말하면 그 발행이 낭비이니, 원본만 필요하면 '
                        '`ros2 run nxde video` 를 따로 띄우는 편이 낫다)'),
        DeclareLaunchArgument(
            'tl_window_width', default_value='960',
            description='인지 결과 창의 ★카메라 뷰★ 가로폭[px]. 창은 이보다 크다 — '
                        '오른쪽에 BEV·게이지 패널이, 아래에 HUD 3줄이 붙는다'
                        '(960 → 창 1248x610). 판정은 원본 해상도로 하므로 이 값은 '
                        '★보이는 크기만★ 바꾼다. 0 이면 원본 크기. ⚠️ ★960 이 기본인 '
                        '이유★ 1920 의 정확히 절반이라 리사이즈가 0.28ms 인데, 640 같은 '
                        '임의 배율은 같은 보간으로 2.50ms 다(실측). 그리고 640 이면 HUD '
                        '글자를 그만큼 작게 잡아야 해서 판독성이 다시 나빠진다. '
                        '패널을 끄려면 -p show_bev:=false, ROI 어둡기는 -p roi_dim:=1.0'),
        DeclareLaunchArgument(
            'tl_stop_latch', default_value='false',
            description='★기본 false★ = 빨간불을 보는 동안만 잡는다(사라지거나 초록불이면 '
                        '해제). true 면 한 번 서면 GREEN 을 봐야만 놓는다 — 정지선에 '
                        '바짝 붙어 신호등이 화면을 벗어나는 코스에서 쓴다'),
        DeclareLaunchArgument(
            'tl_publish_cmd_vel', default_value='false',
            description='★기본 false★ 정지 중 조향 0 을 낼지. true 로 켜면 arduino 의 '
                        '명령 캐시를 0 으로 덮어써서 ★해제 뒤 원래 명령이 되살아나지 '
                        '않는다★(master 로 몰 때 특히 곤란하다) — traffic_light.py 헤더 참고'),
        DeclareLaunchArgument(
            'tl_hsv_red_h2_low', default_value='160',
            description='★적색 hue 밴드 하단(고H쪽) [2026-09-08 신설]★ 노드 기본값도 '
                        '160 이다 — 사례 A(0908_112649_run 06:07~06:10 RED 인지 끊김) '
                        '오프라인 재현에서 램프 hue 가 H160~169 대(종전 문턱 170 을 '
                        '스치는 값)로 관측돼 내렸다. 옛 동작으로 되돌리려면 이 인자를 '
                        '170 으로 준다. cam_testbed 전수 회귀(오탐 유무)로 확정 전까지는 '
                        '이 인자로 두 값을 나란히 돌려 비교한다'),
    ]


def camera_params():
    """★카메라 기하 파라미터 한 벌★ — 카메라 인지 노드는 전부 이것을 받는다.

    선언(이름·기본값)의 주인은 white1/camera_model.py 이고, 여기서는 런치 인자로
    덮어쓸 수 있게 이어 준다. 차선 인지 노드가 붙으면 ★같은 dict 를 그대로★ 넘긴다 —
    그래야 두 노드가 같은 그림을 본다(둘이 다른 사다리꼴을 쓰면 차선과 정지선의 거리가
    서로 다른 자로 재진다).

    ⚠️ 여기 없는 것(bev_w·bev_h·bev_px_to_m)은 camera_model 의 기본값을 그대로 쓴다.
       런치 인자로 노출하지 않은 이유는 ★자주 바꿀 값이 아니고★, 바꾸면 두 문턱
       (sl_trigger_bev_y)의 뜻이 같이 달라지기 때문이다. 필요하면 -p 로 직접 준다.
       ※ bev_px_to_m 은 [2026-09-07] cam_testbed 재실측으로 0.0082 → 0.006160 이
         되었다(camera_model.py).
         ★표시 전용이라 판정은 안 흔들린다★ — HUD·로그의 참고 미터에만 쓴다.
    """
    return {
        'cam_undistort':    LaunchConfiguration('cam_undistort'),
        'bev_src_pts':      LaunchConfiguration('bev_src_pts'),
        'bev_bumper_y_px':  LaunchConfiguration('bev_bumper_y_px'),
    }


def actions(package_name, cam_format, node_env, respawn_delay):
    """usb_cam · v4l2 보정 · traffic_light · tl_video 네 액션.

    전부 use_camera 로 묶이고, 녹화는 거기에 tl_record_video 를 ★AND★ 로 더 건다 —
    카메라를 안 띄우면 /tl/debug_image 가 아예 안 나오므로 녹화만 살아 있으면
    '프레임을 못 받는다' 경고만 5초마다 찍는 노드가 남는다.
    """
    use_camera   = LaunchConfiguration('use_camera')
    video_device = LaunchConfiguration('video_device')
    cam_exposure = LaunchConfiguration('cam_exposure')
    rec_video    = LaunchConfiguration('tl_record_video')

    # ── usb_cam → /image_raw ──
    #   ★camera_info_url 을 주지 않는다 [2026-08-19 근거 갱신]★ 왜곡보정을 안 해서가
    #   아니라(이제 한다), ★/image_raw 를 원본으로 남겨야 하기 때문★ 이다. 보정은 인지
    #   노드가 camera_model 로 직접 하고, 녹화(nxde video)는 원본을 그대로 적는다.
    #   계수의 정본은 white1/calibration/usb_cam_calibration.yaml 이다.
    usb_cam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        additional_env=node_env,
        respawn=True,
        respawn_delay=respawn_delay,
        parameters=[{
            'video_device': video_device,
            'framerate': 30.0,
            'image_width': CAM_WIDTH,
            'image_height': CAM_HEIGHT,
            # 하드코딩하지 않는다 — ports.resolve_camera_format() 의 판정 결과다.
            'pixel_format': cam_format,
            'camera_name': 'narrow_stereo',
            'io_method': 'mmap',
            'brightness': 0,
            'contrast': 128,
            'saturation': 60,
            'sharpness': 64,
            'gain': 10,
            'auto_exposure': False,
            'exposure': cam_exposure,
            # image_transport 부가 플러그인 비활성화 — raw 만 남긴다. 구독자는
            # traffic_light 하나뿐인데 `ros2 bag record -a` 가 compressed/theora 까지
            # 구독하면 인코딩이 실제로 돌아 CPU 를 먹는다.
            #   ⚠️ Humble 은 '<base_topic>.enable_pub_plugins'(화이트리스트)다.
            'image_raw.enable_pub_plugins': ['image_transport/raw'],
        }],
        condition=IfCondition(use_camera),
    )

    # ── 기동 2초 뒤 v4l2-ctl 재적용 ──
    #   ※ respawn 대상이 아니다 — 카메라가 respawn 되면 이 설정은 다시 적용되지 않는다.
    #     노출이 이상하면 아래 명령을 손으로 한 번 더 돌리면 된다.
    #   값을 그대로 넣지 않고 ★장치가 보고하는 범위로 클램프★ 한다. 위 파라미터는
    #   See3CAM 기준인데 다른 카메라에서는 범위를 넘어 드라이버가 조용히 최대값으로
    #   깎는다(내장 Chicony 실측: contrast 0~64 / sharpness 0~5 / gain 0~4 →
    #   밝기·콘트라스트·게인이 겹쳐 전 픽셀 255 로 포화됐다).
    usb_cam_ctrl = TimerAction(
        period=V4L2_APPLY_DELAY_S,
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    (
                        'setc() { '
                        '  local c="$1" want="$2" line mn mx v; '
                        '  line=$(v4l2-ctl -d "$DEV" --list-ctrls 2>/dev/null'
                        ' | awk -v c="$c" \'$1==c\'); '
                        '  [ -z "$line" ] && { echo "  [v4l2] $c: 미지원(스킵)"; return 0; }; '
                        '  mn=$(echo "$line" | grep -oE "min=-?[0-9]+" | cut -d= -f2); '
                        '  mx=$(echo "$line" | grep -oE "max=-?[0-9]+" | cut -d= -f2); '
                        '  v="$want"; '
                        '  [ -n "$mn" ] && [ "$v" -lt "$mn" ] && v="$mn"; '
                        '  [ -n "$mx" ] && [ "$v" -gt "$mx" ] && v="$mx"; '
                        '  v4l2-ctl -d "$DEV" --set-ctrl=$c=$v 2>/dev/null; '
                        '  if [ "$v" != "$want" ]; then '
                        '    echo "  [v4l2] $c: $want -> $v (범위 $mn~$mx 로 클램프)"; '
                        '  else echo "  [v4l2] $c: $v"; fi; '
                        '}; '
                        'echo "[v4l2-ctl] applying camera controls on $DEV"; '
                        'setc auto_exposure 1; '
                        'setc exposure_time_absolute "$EXPOSURE"; '
                        'setc gain 10; '
                        'setc saturation 60; '
                        'setc brightness 0; '
                        'setc contrast 128; '
                        'setc sharpness 64'
                    )
                ],
                additional_env={'DEV': video_device, 'EXPOSURE': cam_exposure},
                output='screen',
            )
        ],
        condition=IfCondition(use_camera),
    )

    # ── 신호등 인지·정지 ──
    #   ★respawn 을 걸지 않는다★ 가중치를 못 읽어도 노드는 죽지 않고 fail-open 으로
    #   남는다(정지를 걸지 않고 에러만 남긴다). 거기에 respawn 을 걸면 TensorRT 엔진
    #   로드를 2초마다 되풀이해 GPU 와 로그만 잡아먹는다.
    traffic_light = Node(
        package=package_name,
        executable='traffic_light',
        name='traffic_light',
        output='screen',
        additional_env=node_env,
        parameters=[{
            'image_topic':            '/image_raw',
            'device':                 LaunchConfiguration('tl_device'),
            'tl_conf':                LaunchConfiguration('tl_conf'),
            'tl_red_stop_min_height': LaunchConfiguration('tl_red_stop_min_height'),
            'tl_near_release_ratio':  LaunchConfiguration('tl_near_release_ratio'),
            'sl_enable':              LaunchConfiguration('sl_enable'),
            'sl_trigger_bev_y':       LaunchConfiguration('sl_trigger_bev_y'),
            'tl_solo_stop_min_height': LaunchConfiguration('tl_solo_stop_min_height'),
            'sl_conf':                LaunchConfiguration('sl_conf'),
            'sl_gate_red_s':          LaunchConfiguration('sl_gate_red_s'),
            'show_window':            LaunchConfiguration('tl_show_window'),
            # ★녹화 스위치가 발행도 켠다 [2026-09-10]★ 창과 독립이다 — 창을 꺼도
            #   _draw() 는 돌고(그쪽 `if show_window or publish_debug`), 그 캔버스가
            #   /tl/debug_image 로 나가 아래 tl_video 가 받아 적는다.
            'tl_publish_debug':       rec_video,
            'window_width':           LaunchConfiguration('tl_window_width'),
            'stop_latch':             LaunchConfiguration('tl_stop_latch'),
            'publish_cmd_vel':        LaunchConfiguration('tl_publish_cmd_vel'),
            'hsv_red_h2_low':         LaunchConfiguration('tl_hsv_red_h2_low'),
            # ★카메라 기하는 한 벌로 받는다★ 차선 인지가 붙으면 같은 dict 를 넘긴다.
            **camera_params(),
        }],
        condition=IfCondition(use_camera),
    )

    # ── 인지 디버그 화면 녹화 [2026-09-10] ──
    #   ★새 녹화 코드를 쓰지 않는다★ nxde/video.py 가 이미 '아무 Image 토픽이나 mp4 로
    #   적는' 일의 단일 소유자다 — fps 를 실측해서 열고(재생 속도가 맞아야 CSV 와 시각을
    #   맞출 수 있다), 남은 디스크가 500MB 밑이면 스스로 끝내고, SIGTERM·SIGINT·atexit
    #   어느 경로로 죽어도 파일을 닫는다(mp4 는 닫아야 재생된다). 여기서 하는 일은
    #   ★그 노드를 /tl/debug_image 에 붙이고 저장 위치를 못 박는 것뿐★ 이다.
    #
    #   ★respawn 을 걸지 않는다★ traffic_light 와 같은 이유다. 녹화가 실패하는 상황은
    #   대개 디스크가 없거나 코덱이 없는 것이라, 되살려 봐야 같은 실패를 반복하며
    #   로그만 덮는다. ★녹화 도구가 주행에 영향을 주는 일은 없어야 한다.★
    #
    #   fps 를 0(실측)으로 둔다 — 실측 구간(약 1초)은 파일에 안 들어가지만, 런치가
    #   주행 시작보다 한참 먼저 뜨므로 잃는 것이 없다. 반대로 fps 가 틀리면 영상이
    #   빠르거나 느리게 재생되어 '몇 초에 무슨 일이 있었나' 를 못 따진다.
    tl_video = Node(
        package='nxde',
        executable='video',
        name='tl_video_node',
        output='screen',
        additional_env=node_env,
        parameters=[{
            'image_topic': LaunchConfiguration('tl_video_topic'),
            'output_dir':  LaunchConfiguration('tl_video_dir'),
            'scale':       LaunchConfiguration('tl_video_scale'),
            'fps':         0.0,          # 0 = 실측(권장)
            'prefix':      'tl',         # → tl-<날짜>_<시각>.mp4
            #  ★[2026-09-13] 세그먼트 분할★ mp4 는 인덱스를 파일 끝에 쓰므로
            #  SIGKILL 을 맞으면 통째로 재생 불가가 된다(실측: 4개 중 3개가 깨졌다).
            #  이 주기마다 조각을 완결해 두면 잃는 것이 마지막 조각뿐이다.
            'segment_s':   LaunchConfiguration('tl_video_segment_s'),
        }],
        #  ★[2026-09-13] 닫을 틈을 준다★ 런치는 SIGINT → 유예 → SIGTERM → 유예 →
        #  SIGKILL 로 올라간다. 기본 유예로는 큰 파일의 release() 가 못 끝나
        #  파일이 깨졌다. 이 노드에만 넉넉히 준다 — 다른 노드는 그대로다.
        sigterm_timeout='10',
        sigkill_timeout='10',
        condition=IfCondition(AndSubstitution(use_camera, rec_video)),
    )

    return [usb_cam, usb_cam_ctrl, traffic_light, tl_video]


def banner(ports, log=print):
    """카메라를 실제로 열어 고르고 (경로, 포맷) 을 돌려준다. 두 런치의 배너가 같다.

    ★use_camera 와 무관하게 항상 돈다★ 런치 인자는 이 시점에 값이 없어서(치환 객체다)
    조건부로 돌리려면 OpaqueFunction 으로 감싸야 하는데, 그러면 이 배너가 GPS/IMU
    탐색과 따로 놀아 읽기 어려워진다. 프레임 프로브 몇 초가 유일한 비용이다.
    """
    log("=====================================================")
    log(" 📷 카메라 확인 (신호등 인지용 — use_camera 로 끌 수 있습니다)")
    log("    ★내장 웹캠·적외선 노드는 후보에서 제외합니다★")
    log("    남은 후보는 실제로 열어 프레임을 봅니다(수 초 소요).")
    cam_dev = ports.resolve_camera(log=lambda m: log(f"    [CAM] {m}"))
    cam_format = ports.resolve_camera_format(
        cam_dev, CAM_WIDTH, CAM_HEIGHT, log=lambda m: log(f"    [CAM] {m}"))
    log("=====================================================\n")
    return cam_dev, cam_format
