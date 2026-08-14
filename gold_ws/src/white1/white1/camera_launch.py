#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""camera_launch.py — ★카메라·신호등 런치 조각의 단일 소유자★ [white1]

one_launch.py(자율주행)와 master.launch.py(수동 계측)가 ★같은 카메라 구성★ 을 쓴다.
두 곳에 복사해 두면 반드시 갈라지므로(usb_cam 파라미터·v4l2 보정 스크립트가 길다)
여기 한 곳에서 만들어 준다. paths.py 가 저장 위치의 단일 소유자인 것과 같은 태도다.

    args, actions = camera_launch.build(cam_dev, cam_format, NODE_ENV, RESPAWN_DELAY)

띄우는 것 (use_camera 로 함께 켜고 끈다):
    usb_cam       V4L2 → /image_raw
    usb_cam_ctrl  기동 2초 뒤 v4l2-ctl 로 노출·게인 등을 ★장치 범위로 클램프해★ 재적용
    traffic_light /image_raw → 빨간불이면 /brake_level=2 (white1/traffic_light.py)

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
                        '작으면 멀리서 서고, 크면 늦게 선다(1920x1080 기준)'),
        DeclareLaunchArgument(
            'tl_show_window', default_value='true',
            description='인지 결과 창(OpenCV)을 띄울지. ★기본 true★ — ROI·근접도·색 '
                        '임계는 눈으로 봐야 잡는다. 화면 없는 터미널(ssh)이면 false'),
        DeclareLaunchArgument(
            'tl_window_width', default_value='640',
            description='인지 결과 창의 가로폭[px]. 원본(1920)을 그대로 띄우면 화면을 '
                        '덮는다. 판정은 원본 해상도로 하므로 이 값은 ★보이는 크기만★ '
                        '바꾼다. 0 이면 원본 크기'),
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


def actions(package_name, cam_format, node_env, respawn_delay):
    """usb_cam · v4l2 보정 · traffic_light 세 액션. 전부 use_camera 로 묶인다."""
    use_camera   = LaunchConfiguration('use_camera')
    video_device = LaunchConfiguration('video_device')
    cam_exposure = LaunchConfiguration('cam_exposure')

    # ── usb_cam → /image_raw ──
    #   ★camera_info_url 을 주지 않는다★ traffic_light 는 왜곡보정을 쓰지 않는다.
    #   쓰지도 않을 캘리브 파일을 들이면 '맞는지 아무도 모르는 값'이 하나 늘 뿐이다.
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
            'show_window':            LaunchConfiguration('tl_show_window'),
            'window_width':           LaunchConfiguration('tl_window_width'),
            'stop_latch':             LaunchConfiguration('tl_stop_latch'),
            'publish_cmd_vel':        LaunchConfiguration('tl_publish_cmd_vel'),
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
