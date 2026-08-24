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
                  ★[2026-08-19] 정지선이 보이면 2단계로 선다★ 1단 예비제동으로 줄이다
                  정지선 앞에서 2단 확정 정지 (sl_brake1_px · sl_brake2_px).
                  정지선이 안 보이면 종전대로 그 자리에서 즉시 2단.

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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

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
            description='★정지선 앞 2단계 정지★ 켜면 빨간불이 확정돼도 정지선이 보이는 '
                        '동안은 1단 예비제동으로 줄이다가 정지선 앞에서 2단으로 선다. '
                        '정지선을 못 보면 종전대로 즉시 2단 — 즉 인지가 안 되는 날은 이 '
                        '인자가 아무 일도 하지 않는다. false 면 정지선 추론 자체를 안 돌린다'),
        DeclareLaunchArgument(
            'sl_brake1_px', default_value='240.0',
            description='★1단 예비제동 문턱★ BEV 에서 정지선→앞범퍼 거리가 이 픽셀 이하가 '
                        '되면 리니어 1단으로 부드럽게 줄이기 시작한다. ⚠️ 근거 없는 '
                        '기본값이다 — 1단 감속도 1.30 m/s²(BRAKING.md 4절)로 4펄스에서 '
                        '약 4.8m 이니, 그 거리의 HUD px 값으로 잡는다(STOPLINE_TEST 단계 2). '
                        '크게 잡으면 멀리서부터 기어가다 정지선 전에 멈춰 선다. '
                        '★[2026-08-24] 실측은 cam_testbed에서 하고, 여기 기본값은 '
                        '그 결과로 갱신한다★'),
        DeclareLaunchArgument(
            'sl_brake2_px', default_value='60.0',
            description='★2단 확정 정지 문턱★ 같은 거리가 이 픽셀 이하면 2단으로 세운다. '
                        '2단 정지거리가 4펄스에서 1.6~2.8m(BRAKING.md)이므로 그만큼 여유를 '
                        '두고 잡는다. ★반드시 sl_brake1_px 보다 작아야 한다★. '
                        '⚠️ 값에 ★소수점을 붙여야 한다★(60 이 아니라 60.0) — 런치 인자는 '
                        '문자열을 그대로 형변환하므로 정수로 주면 노드 선언 타입(double)과 '
                        '어긋나 기동에 실패한다(tl_conf 등 기존 인자와 같은 성질이다)'),
        DeclareLaunchArgument(
            'cam_undistort', default_value='true',
            description='★어안 왜곡보정★ 카메라 인지 노드가 프레임을 받은 뒤 보정한다. '
                        '계수는 white1/calibration/usb_cam_calibration.yaml. ⚠️ 끄면 BEV '
                        '거리(정지선 판정)를 믿을 수 없다 — 어안이 남은 그림에 원근변환을 '
                        '걸면 직선이 휜 채로 펴진다. /image_raw 원본은 어느 쪽이든 그대로다'),
        DeclareLaunchArgument(
            'bev_src_pts',
            default_value='[750.0, 560.0, 1170.0, 560.0, 1920.0, 1080.0, 0.0, 1080.0]',
            description='★BEV 사다리꼴★ 보정된 화면에서 노면 직사각형에 해당하는 네 점 '
                        '(좌상,우상,우하,좌하). ⚠️ 기본값은 구 white 마운트에서 유도한 '
                        '것이라 ★이 차에서 다시 잡아야 한다★ — 화면에 노란 사다리꼴로 '
                        '그려지니 노면에 맞춰 눈으로 맞춘다(STOPLINE_TEST 단계 2). '
                        '★[2026-08-24] 이 실측은 cam_testbed에서 하고, 여기 기본값은 '
                        '그 결과를 받아 갱신한다★'),
        DeclareLaunchArgument(
            'bev_bumper_y_px', default_value='480.0',
            description='★앞범퍼가 BEV 의 몇 번째 행인가 = 거리 0 의 기준★ 기본값은 BEV '
                        '최하단(=사다리꼴 밑변). 범퍼가 그보다 앞(차 쪽)이면 480 보다 큰 '
                        '값을 준다. 이 값이 틀리면 두 문턱이 통째로 어긋난다'),
        DeclareLaunchArgument(
            'sl_conf', default_value='0.30',
            description='정지선 seg 신뢰도 임계. 구 white 의 lane_conf 와 같은 값'),
        DeclareLaunchArgument(
            'sl_gate_red_s', default_value='1.0',
            description='빨간 박스를 이 시간 안에 본 적이 있을 때만 정지선 추론을 돌린다 '
                        '(평상시 비용 0). ★튜닝할 때는 크게 준다★ — 신호등 없이 정지선만 '
                        '보고 싶으면 sl_gate_red_s:=99999 로 상시 추론시킨다(todo 9-1)'),
        DeclareLaunchArgument(
            'tl_show_window', default_value='true',
            description='인지 결과 창(OpenCV)을 띄울지. ★기본 true★ — ROI·근접도·색 '
                        '임계는 눈으로 봐야 잡는다. 화면 없는 터미널(ssh)이면 false'),
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
    ]


def camera_params():
    """★카메라 기하 파라미터 한 벌★ — 카메라 인지 노드는 전부 이것을 받는다.

    선언(이름·기본값)의 주인은 white1/camera_model.py 이고, 여기서는 런치 인자로
    덮어쓸 수 있게 이어 준다. 차선 인지 노드가 붙으면 ★같은 dict 를 그대로★ 넘긴다 —
    그래야 두 노드가 같은 그림을 본다(둘이 다른 사다리꼴을 쓰면 차선과 정지선의 거리가
    서로 다른 자로 재진다).

    ⚠️ 여기 없는 것(bev_w·bev_h·bev_px_to_m)은 camera_model 의 기본값을 그대로 쓴다.
       런치 인자로 노출하지 않은 이유는 ★자주 바꿀 값이 아니고★, 바꾸면 두 문턱
       (sl_brake*_px)의 뜻이 같이 달라지기 때문이다. 필요하면 -p 로 직접 준다.
    """
    return {
        'cam_undistort':    LaunchConfiguration('cam_undistort'),
        'bev_src_pts':      LaunchConfiguration('bev_src_pts'),
        'bev_bumper_y_px':  LaunchConfiguration('bev_bumper_y_px'),
    }


def actions(package_name, cam_format, node_env, respawn_delay):
    """usb_cam · v4l2 보정 · traffic_light 세 액션. 전부 use_camera 로 묶인다."""
    use_camera   = LaunchConfiguration('use_camera')
    video_device = LaunchConfiguration('video_device')
    cam_exposure = LaunchConfiguration('cam_exposure')

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
            'sl_brake1_px':           LaunchConfiguration('sl_brake1_px'),
            'sl_brake2_px':           LaunchConfiguration('sl_brake2_px'),
            'sl_conf':                LaunchConfiguration('sl_conf'),
            'sl_gate_red_s':          LaunchConfiguration('sl_gate_red_s'),
            'show_window':            LaunchConfiguration('tl_show_window'),
            'window_width':           LaunchConfiguration('tl_window_width'),
            'stop_latch':             LaunchConfiguration('tl_stop_latch'),
            'publish_cmd_vel':        LaunchConfiguration('tl_publish_cmd_vel'),
            # ★카메라 기하는 한 벌로 받는다★ 차선 인지가 붙으면 같은 dict 를 넘긴다.
            **camera_params(),
        }],
        condition=IfCondition(use_camera),
    )

    return [usb_cam, usb_cam_ctrl, traffic_light]


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
