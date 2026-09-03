#!/usr/bin/env python3
"""one_launch.py ― ★하드웨어(GPS·IMU·카메라·아두이노) + 자율주행 노드 통합 런치★

    ros2 launch white2 one_launch.py

╔══════════════════════════════════════════════════════════════════════════════╗
║  [white2] 아두이노 담당을 1/5카 원본(단일 Mega, motor 노드)으로 되돌렸다        ║
╠══════════════════════════════════════════════════════════════════════════════╣
║    이 런치가 담당            │  담당 방식                                      ║
║    ───────────────────────── │ ────────────────────────────────────────────── ║
║    GPS   (u-blox)            │  nmea_serial_driver + white2/ports.py 자동탐색  ║
║    IMU   (iAHRS)             │  white2 의 iahrs 노드 (재연결마다 포트 재탐색)   ║
║    카메라 (USB)              │  usb_cam + ★video2 → video0 순 + 프레임 검증★   ║
║    아두이노 (1/5카 단일 Mega)│  ★white2 의 motor 노드★ (kasa 의 nxde 는 안 씀) ║
╚══════════════════════════════════════════════════════════════════════════════╝

═══════════════════════════════════════════════════════════════════════════════
 실행 순서
═══════════════════════════════════════════════════════════════════════════════
   1) ros2 launch white2 one_launch.py  ← 이 파일 (하드웨어 + 자율주행 노드)
   2) ros2 run   white2 prompt          ← CLI 메뉴 (★별 터미널에서 손으로★)

 ★ 장치를 먼저 꽂아 둘 것 ★ 런치를 띄운 뒤에 장치를 꽂으면 경합이 생긴다. GPS·IMU·
   아두이노가 모두 /dev/ttyACM*·/dev/ttyUSB* 대역을 나눠 쓰는데, 런치 시점에 없던
   장치는 각 노드가 뒤늦게 찾으러 다니며 서로의 포트를 배타 open 으로 밀어낸다.

 ★ 2) prompt 는 이 런치가 띄우지 않는다 ★ 별 터미널에서 손으로 띄운다.

═══════════════════════════════════════════════════════════════════════════════
 이 런치가 기동하는 노드
═══════════════════════════════════════════════════════════════════════════════
  [하드웨어]
    motor             아두이노 Mega ↔ /cmd_vel_raw /control_state
                                     ↔ /encoder /steer_angle_measured 등
    iahrs             iAHRS IMU        → /imu/data (+ TF)
    nmea_serial_driver u-blox GPS      → /fix
    usb_cam           USB 카메라       → /image_raw            (use_camera)
    usb_cam_ctrl      v4l2-ctl 강제 적용 (드라이버가 파라미터를 무시할 때 보정)
  [자율주행]
    gps_imu           /fix + /imu/data + /encoder → 융합 → /ego_state
    driving           경로추종 → /cmd_vel_raw (또는 /cmd_vel_drive) + /control_state
    mapping           수집 — /ego_state + 수동조종 계측 3종을 CSV 기록
    perception        /image_raw → /lane/state, /tl/state …     (use_camera)
    camera_judgment   /lane_metrics 브리지 + 신호등 게이트       (use_camera)
    sensor_monitor    센서 상태 대시보드                         (use_monitor)

╔══════════════════════════════════════════════════════════════════════════════╗
║  하드웨어 토픽 계약 (white2 의 motor 노드와 맞물리는 부분, 1/5카 규약)          ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  발행  /cmd_vel_raw (Twist)  linear.x = ★m/s 그대로★ (펄스 양자화 없음)        ║
║                              angular.z = 조향각 −21~21 (★+ 좌 / − 우★)         ║
║        /control_state (Bool) True = 구동 허용 / False = 정지                  ║
║  구독  /encoder (Int32)              부호 있는 틱/10ms → 속도 피드백           ║
║        /steer_angle_measured (Int32) 실측 조향각(A15 포텐셔미터, deg)          ║
║        /motor_pwm, /steer_pwm (Int32) PID 실제 PWM 출력 (진단용)              ║
╚══════════════════════════════════════════════════════════════════════════════╝

★ [white2] kasa 의 B보드 D5 물리 모드 스위치·/vehicle_mode·/estop·/brake_level·
  페달 패스스루 수동조종은 1/5카(motor.py)에 없다 ★ 시작·정지 권한은 이 런치 위에서
  도는 driving/prompt 의 소프트웨어 명령(/drive_cmd, /control_state)뿐이다.

⚠️ 종료 : Ctrl+C 한 번으로 motor 노드까지 함께 내려간다. motor 노드는 종료 직전에
   정지 명령("C,0,0" / "S,0")을 시리얼로 직접 써 넣으므로 차가 선다.

주의:
    - GPS가 /fix를 만들어야 gps_imu.py가 초기 헤딩을 고정할 수 있음.
    - /encoder(= motor 노드)가 있어야 ENC+GPS 헤딩 고정이 가능함.
    - 엔코더가 없어도 GPS-only 조건을 만족하면 헤딩 고정 가능.
    - ★RTK 는 이 런치가 만들지 않는다★ nmea_serial_driver 는 수신기가 내보내는 NMEA 를
      읽을 뿐이고 RTCM 보정을 넣지 않는다. RTK FIX 여부는 GGA quality(4=고정해,
      5=부동해)로 판별된다. 보정은 수신기 쪽 경로(기지국 라디오 / 별도 NTRIP
      클라이언트)로 들어와야 한다.

환경변수:
  PYTHONUNBUFFERED=1                : launch 가 stdout 을 파이프로 받으면 Python 이 완전
                                      버퍼링으로 전환되어 print 가 갇힌다
  RCUTILS_LOGGING_BUFFERED_STREAM=0 : get_logger() 로그(rcutils C 스트림) 버퍼링 해제.
                                      이게 없으면 '보드 감지' 로그가 종료 시점에야 몰려
                                      나오거나 유실된다(PYTHONUNBUFFERED 로는 안 잡힘)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from white2 import ports

NODE_ENV = {'PYTHONUNBUFFERED': '1',
            'RCUTILS_LOGGING_BUFFERED_STREAM': '0'}

# 외부 노드(GPS/카메라) 재기동 간격 [s]. 장치가 아예 없으면 이 주기로 계속 재시도한다.
RESPAWN_DELAY = 3.0

# 카메라 해상도 — 픽셀 포맷 자동 판정(ports.resolve_camera_format)이 이 값을 기준으로
#   '그 해상도를 실제로 내는 포맷'을 고르므로, 여기와 usb_cam 파라미터가 같아야 한다.
CAM_WIDTH, CAM_HEIGHT = 1920, 1080


def generate_launch_description():
    package_name = 'white2'

    print("\n=====================================================")
    print(" 🔌 하드웨어 장치 경로를 확인합니다 (GPS / IMU / 카메라)")
    print("    아두이노는 motor 노드가 arduino_port 파라미터로 고정 지정합니다.")

    # ★ 여기서 찾지 못해도 실패가 아니다 ★ resolve_device 는 udev 심볼릭링크 → VID/PID →
    #   (그래도 없으면) 심볼릭링크 경로를 그대로 돌려준다. 안정적인 이름을 넘겨두면
    #   나중에 장치를 꽂는 순간 respawn / 자체 재연결이 자동으로 붙는다.
    used = set()
    gps_dev = ports.resolve_device(ports.SYMLINK_GPS, ports.GPS_VIDPID,
                                  exclude=used, log=lambda m: print(f"    [GPS] {m}"))
    used.add(gps_dev)
    imu_dev = ports.resolve_device(ports.SYMLINK_IMU, ports.IMU_VIDPID,
                                  exclude=used, log=lambda m: print(f"    [IMU] {m}"))
    used.add(imu_dev)
    # 카메라는 ★video2 → video0 순으로 시도하되 각 후보를 실제로 열어 프레임을 본다★
    #   순서만 믿으면 '덮개 닫힌 내장 웹캠'이나 '동결된 스트림'을 그대로 열어 검은 화면
    #   으로 주행한다(2026-08-04 실제 사고). ports.resolve_camera() 헤더 참고.
    cam_dev = ports.resolve_camera(log=lambda m: print(f"    [CAM] {m}"))
    # 픽셀 포맷도 장치를 보고 정한다 — 카메라마다 지원 포맷이 달라서 하드코딩하면
    #   usb_cam 이 std::invalid_argument 로 즉사하고 respawn 무한루프를 돈다.
    #   (See3CAM 은 1920x1080 에서 UYVY 만, 내장 웹캠은 MJPG 만 낸다 — ports.py 참고)
    cam_format = ports.resolve_camera_format(
        cam_dev, CAM_WIDTH, CAM_HEIGHT, log=lambda m: print(f"    [CAM] {m}"))
    print("=====================================================\n")

    # [white2] motor.py 는 GPS/IMU 처럼 VID/PID 자동탐색을 하지 않는다 — 'port'
    #   파라미터(기본 /dev/ttyACM0)로 고정 지정한다(아래 arduino_port 인자).

    use_monitor  = LaunchConfiguration('use_monitor')
    use_camera   = LaunchConfiguration('use_camera')
    use_arduino  = LaunchConfiguration('use_arduino')
    use_record   = LaunchConfiguration('use_record')
    image_topic  = LaunchConfiguration('image_topic')
    video_device = LaunchConfiguration('video_device')
    cam_exposure = LaunchConfiguration('cam_exposure')
    dr_speed_factor = LaunchConfiguration('dr_speed_factor')

    args = [
        DeclareLaunchArgument(
            'use_monitor', default_value='true',
            description='sensor_monitor 노드 실행 여부'),
        DeclareLaunchArgument(
            'use_camera', default_value='true',
            description='카메라 체인(usb_cam + perception + camera_judgment) 실행 여부. '
                        'true=driving→/cmd_vel_drive→게이트→/cmd_vel_raw, '
                        'false=driving→/cmd_vel_raw 직결(GPS 단독). '
                        '★usb_cam 도 이 인자가 함께 게이트한다 — 따로 맞출 필요가 없다★'),
        DeclareLaunchArgument(
            'use_arduino', default_value='true',
            description='white2 의 motor 노드(1/5카 단일 아두이노 Mega 브리지)를 이 '
                        '런치가 함께 띄울지. false 로 두면 별 터미널에서 '
                        '`ros2 run white2 motor` 로 직접 띄운다(보드 로그를 따로 보고 '
                        '싶을 때)'),
        DeclareLaunchArgument(
            'use_record', default_value='true',
            description='record 노드(주행 CSV 기록) 실행 여부. 켜 두어도 아무 때나 '
                        '쓰지 않는다 — 자율주행 모드 + prompt 주행 구간에서만 파일이 '
                        '생긴다. 저장 위치는 <white2 패키지>/ros2bag/ (record_dir 로 변경)'),
        DeclareLaunchArgument(
            'record_dir', default_value='',
            description='기록 저장 폴더 override. 비우면 소스트리의 white2/ros2bag/ 을 '
                        '쓰고, 그걸 못 찾으면(심볼릭 링크 없이 빌드한 경우) ~/ros2bag/ 로 '
                        '보낸다 — install/ 안에는 쌓지 않는다(재빌드하면 날아가므로)'),
        DeclareLaunchArgument(
            'image_topic', default_value='/image_raw',
            description='perception 이 구독할 카메라 이미지 토픽(usb_cam 이 발행)'),
        DeclareLaunchArgument(
            'video_device', default_value=cam_dev,
            description='USB 카메라 V4L2 장치 경로. 기본값은 ports.resolve_camera() 가 '
                        'video2 → video0 순으로 시도하며 각 후보의 실제 프레임을 검증해 '
                        '고른다. 수동 지정은 `v4l2-ctl --list-devices` 로 확인'),
        DeclareLaunchArgument(
            'cam_exposure', default_value='120',
            description='See3CAM 수동노출(exposure_time_absolute, 낮을수록 어둡게/짧게). '
                        '기본 120 은 야간 기준 — 주간엔 과다노출로 신호등 블루밍·차선 대비 '
                        '붕괴 확인됨(실측: exp=120 대비 48 / exp=2 대비 90, 포화 0%→2.6%). '
                        '주간 테스트 시 예: cam_exposure:=10'),
        DeclareLaunchArgument(
            'gps_port', default_value=gps_dev,
            description='GPS 시리얼 경로 override (기본: udev 링크 → VID/PID 스캔 결과)'),
        DeclareLaunchArgument(
            'imu_port', default_value=imu_dev,
            description='iAHRS 시리얼 경로 override (기본: udev 링크 → VID/PID 스캔 결과)'),
        DeclareLaunchArgument(
            'imu_sync_period_ms', default_value='50',
            description='IMU 출력주기[ms]. 기본 50=20Hz. driving 의 지연보상 예측 정밀도를 '
                        '높이려면 20(=50Hz) 권장'),
        DeclareLaunchArgument(
            'dr_speed_factor', default_value='0.6',
            description='GPS 두절(DR) 진입 시 속도 감속 계수(낮출수록 더 감속)'),
        # ── motor 노드 파라미터 (1/5카 단일 아두이노 Mega) ──
        DeclareLaunchArgument(
            'baud', default_value='115200',
            description='아두이노 Mega 시리얼 보드레이트 (motor.py SERIAL_BAUD)'),
        DeclareLaunchArgument(
            'arduino_port', default_value='/dev/ttyACM0',
            description='아두이노 Mega 시리얼 포트 (motor.py SERIAL_PORT). udev 로 '
                        '/dev/arduino_mega 같은 고정 심볼릭링크를 두는 것을 권장한다'),
    ]

    # ═══════════════════════════════════════════════════════════════════
    #  [하드웨어] 아두이노 Mega — 1/5카 단일 보드 (white2 의 motor 노드)
    # ═══════════════════════════════════════════════════════════════════
    arduino = Node(
        package='white2',
        executable='motor',
        name='motor_node',
        output='screen',
        additional_env=NODE_ENV,
        parameters=[{
            'port': LaunchConfiguration('arduino_port'),
            'baud': LaunchConfiguration('baud'),
        }],
        condition=IfCondition(use_arduino),
    )

    # ═══════════════════════════════════════════════════════════════════
    #  [하드웨어] iAHRS IMU — 자체 2초 재연결 + VID/PID 재탐색.
    #    respawn 은 '노드가 예외로 죽는' 경우의 이중 안전망으로만 걸어둔다.
    # ═══════════════════════════════════════════════════════════════════
    iahrs = Node(
        package=package_name,
        executable='iahrs',
        name='iahrs_node',
        output='screen',
        additional_env=NODE_ENV,
        respawn=True,
        respawn_delay=RESPAWN_DELAY,
        parameters=[{
            'port':           LaunchConfiguration('imu_port'),
            'baud':           115200,
            'send_tf':        True,
            'rescan':         True,
            'sync_period_ms': LaunchConfiguration('imu_sync_period_ms'),
            'exclude_ports':  [gps_dev],
        }],
    )

    # ═══════════════════════════════════════════════════════════════════
    #  [하드웨어] u-blox GPS — 외부 패키지(nmea_navsat_driver). 코드를 고칠 수 없으므로
    #    respawn 에 의존한다 → 경로가 안정적이어야 한다(udev 권장).
    #    ★RTCM 보정을 넣지 않는다★ RTK 여부는 수신기가 만들고, GGA quality 로 드러난다.
    # ═══════════════════════════════════════════════════════════════════
    gps = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='nmea_serial_driver',
        output='screen',
        additional_env=NODE_ENV,
        respawn=True,
        respawn_delay=RESPAWN_DELAY,
        parameters=[{'port': LaunchConfiguration('gps_port'), 'baud': 115200}],
    )

    # ═══════════════════════════════════════════════════════════════════
    #  [하드웨어] USB 카메라 — 외부 패키지(usb_cam). respawn 으로 재시도.
    #    camera_info 는 이 패키지 share 에서 읽는다(white2/calibration 이 정본).
    #    ※ perception 은 /camera_info 를 구독하지 않으므로 이 yaml 은 사실상 기록용이다.
    #      실제 BEV 캘리브는 pixel_to_meter_bev 파라미터가 담당한다.
    # ═══════════════════════════════════════════════════════════════════
    calib_path = os.path.join(
        get_package_share_directory(package_name),
        'calibration', 'usb_cam_calibration.yaml')
    usb_cam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        additional_env=NODE_ENV,
        respawn=True,
        respawn_delay=RESPAWN_DELAY,
        parameters=[{
            'video_device': video_device,
            'framerate': 30.0,
            'image_width': CAM_WIDTH,
            'image_height': CAM_HEIGHT,
            # [2026-08-04] 하드코딩하지 않는다 — ports.resolve_camera_format() 이 '선택된
            #   장치가 이 해상도에서 실제로 내는 포맷'을 보고 정한다. 예전엔 'uyvy' 고정이라
            #   내장 웹캠(MJPG 전용)에 붙으면 usb_cam 이 std::invalid_argument 로 즉사하고
            #   respawn 무한루프를 돌았다("Specified format `uyvy` is unsupported...").
            #   반대로 mjpeg2rgb 로 고정하면 See3CAM(UYVY 전용)에서 똑같이 죽는다.
            'pixel_format': cam_format,
            'camera_name': 'narrow_stereo',
            'io_method': 'mmap',
            'camera_info_url': 'file://' + calib_path,
            'brightness': 0,
            'contrast': 128,
            'saturation': 60,
            'sharpness': 64,
            'gain': 10,
            'auto_exposure': False,
            'exposure': cam_exposure,
            # [2026-07-29] image_transport 부가 플러그인 비활성화 — raw 만 남긴다.
            #   usb_cam 은 /image_raw 외에 compressed(JPEG)·compressedDepth·theora 를
            #   함께 광고한다. 구독자는 perception 하나뿐이고 원본만 쓰는데,
            #   `ros2 bag record -a` 가 그 부가 토픽까지 구독하면 인코딩이 실제로 돌아
            #   CPU 를 먹는다. 특히 compressedDepth 는 컬러(yuv422)를 깊이영상으로
            #   압축하려다 매 프레임 실패해 ERROR 를 쏟아낸다.
            #   ⚠️ Humble 의 image_transport 는 'disable_pub_plugins' 가 아니라
            #      '<base_topic>.enable_pub_plugins' (화이트리스트) 를 쓴다.
            #      이름을 틀리면 조용히 무시되고 에러가 계속 뜬다(실제로 그랬다).
            'image_raw.enable_pub_plugins': ['image_transport/raw'],
        }],
        condition=IfCondition(use_camera),
    )

    # usb_cam 기동 2초 뒤 v4l2-ctl 강제 적용 (드라이버가 파라미터를 무시하는 경우 보정).
    #   ※ 이건 respawn 대상이 아니다 — 카메라가 respawn 되면 이 설정은 다시 적용되지 않는다.
    #     노출이 이상하면 이 명령을 손으로 한 번 더 돌리면 된다(아래 cmd 그대로).
    usb_cam_ctrl = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    (
                        # [2026-08-04] 값을 그대로 넣지 않고 '장치가 보고하는 범위로 클램프'한다.
                        #   위 파라미터는 See3CAM 기준(contrast 128 / sharpness 64 / gain 10)인데,
                        #   다른 카메라에 꽂으면 범위를 넘어 드라이버가 조용히 최대값으로 깎는다.
                        #   실측(내장 Chicony 04f2:b7f3): contrast 0~64 / sharpness 0~5 / gain 0~4
                        #   → 최대 밝기·콘트라스트·게인이 겹쳐 전 픽셀 255 로 포화됐다.
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

    # ═══════════════════════════════════════════════════════════════════
    #  [자율주행] 측위 융합 허브
    # ═══════════════════════════════════════════════════════════════════
    gps_imu = Node(
        package=package_name,
        executable='gps_imu',
        name='gps_imu_node',
        output='screen',
        additional_env=NODE_ENV,
    )

    # ── [자율주행] 수집(매핑) ──
    #   ★ 수집은 '수동조종 모드'에서 한다 ★ 사람이 페달·핸들로 차를 몰고, 그때의
    #     ①페달 환산 목표펄스(/drive_pulse_cmd) ②실 주행 펄스(/encoder)
    #     ③DC모터 가변저항 실측 조향각(/steer_angle_measured) 을 함께 기록한다.
    #     모드 강제는 prompt 가 담당한다.
    mapping = Node(
        package=package_name,
        executable='mapping',
        name='mapping_node',
        output='screen',
        additional_env=NODE_ENV,
    )

    # [카메라 융합] driving 출력 토픽을 use_camera 로 분기.
    #   use_camera=true  → /cmd_vel_drive (camera_judgment 게이트 경유)
    #   use_camera=false → /cmd_vel_raw   (motor 직결, GPS 단독)
    # [white2] 두 토픽 모두 linear.x = m/s 그대로다 — 어느 경로든 단위 환산이 없다.
    driving_cam = Node(
        package=package_name,
        executable='driving',
        name='driving_node',
        output='screen',
        additional_env=NODE_ENV,
        parameters=[{'cmd_vel_topic': '/cmd_vel_drive',
                     'dr_speed_factor': dr_speed_factor}],
        condition=IfCondition(use_camera),
    )
    driving_nocam = Node(
        package=package_name,
        executable='driving',
        name='driving_node',
        output='screen',
        additional_env=NODE_ENV,
        parameters=[{'cmd_vel_topic': '/cmd_vel_raw',
                     'dr_speed_factor': dr_speed_factor}],
        condition=UnlessCondition(use_camera),
    )

    # [카메라 융합] 인지 — 차선 polyfit(/lane/state) + 신호등(/tl/state 등)
    perception = Node(
        package=package_name,
        executable='perception',
        name='perception_node',
        output='screen',
        additional_env=NODE_ENV,
        parameters=[{
            'image_topic':        image_topic,
            'show_window':        True,          # 차량 배포는 헤드리스
            'pixel_to_meter_bev': 0.006,
        }],
        condition=IfCondition(use_camera),
    )

    # [카메라 융합] 판단 — /lane_metrics 브리지 + 신호등 게이트(/cmd_vel_drive→/cmd_vel_raw)
    #   ★신호등 정지는 리니어모터 2단 체결로 한다★ (/brake_level=2 — camera_judgment 참고)
    camera_judgment = Node(
        package=package_name,
        executable='camera_judgment',
        name='camera_judgment',
        output='screen',
        additional_env=NODE_ENV,
        parameters=[{'pixel_to_meter_bev': 0.006}],
        condition=IfCondition(use_camera),
    )

    monitor = Node(
        package=package_name,
        executable='sensor_monitor',
        name='sensor_monitor_node',
        output='screen',
        additional_env=NODE_ENV,
        condition=IfCondition(use_monitor),
    )

    # 주행 기록 — 구독만 하는 노드라 제어에 끼어들지 않는다(발행 토픽 없음).
    #   기록 구간은 스스로 판정한다: /drive_cmd(경로 하달) + /vehicle_mode(자율) +
    #   /control_state(제어중) + /estop(해제). 자세한 규약은 white2/record.py 참고.
    record = Node(
        package=package_name,
        executable='record',
        name='record_node',
        output='screen',
        additional_env=NODE_ENV,
        parameters=[{'output_dir': LaunchConfiguration('record_dir')}],
        condition=IfCondition(use_record),
    )

    return LaunchDescription(args + [
        # 하드웨어를 먼저 둔다 — 자율주행 노드가 첫 토픽을 놓칠 확률을 줄인다
        arduino,
        iahrs,
        gps,
        usb_cam,
        usb_cam_ctrl,
        # 자율주행
        gps_imu,
        mapping,
        driving_cam,
        driving_nocam,
        perception,
        camera_judgment,
        monitor,
        record,
    ])
