#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
one_launch.py ― white1 통합 런치 (GPS + IMU + 아두이노 + 자율주행)
════════════════════════════════════════════════════════════════════════════════
    ros2 launch white1 one_launch.py

띄우는 것 (카메라 없음):
    nxde/arduino          A/B 2보드 시리얼 브리지  ★수정 없이 그대로 쓴다★
    white1/iahrs        6축 IMU 드라이버 → /imu
    white1/speed        /imu 적분 속도계 → /speed [km/h]
    nmea_navsat_driver    GPS 수신기 → /fix
    white1/gps          ★/fix + /imu → /gps_fused★ RTK Fixed/Float 판정 +
                          5Hz fix 사이 공백을 IMU 로 메운 20Hz 가상좌표
    white1/driving      ★헤딩 + 모드스위치 상태기계 + 경로추종★ (위치는 gps 가 준다)
    white1/mapping      ★/fix 원값만★ 보고 경로 수집 (gps 후처리를 거치지 않는다)
    white1/record       자율주행 구간 토픽 → CSV
    usb_cam             카메라 → /image_raw                      (use_camera)
    white1/traffic_light 신호등 인지 — 빨간불이면 리니어 2단      (use_camera)
        ★DRIVE_RUN 중에만 개입한다★ 빨간불이 사라지거나 초록불이면 즉시 풀고,
        driving 이 계속 내던 목표펄스가 그대로 통해 스스로 재출발한다.
        카메라를 안 꽂았으면 use_camera:=false (usb_cam 이 respawn 루프를 돈다)

CLI 는 따로 띄운다 (별 터미널):
    ros2 run white1 prompt

════════════════════════════════════════════════════════════════════════════════
 조작 [2026-08-11] prompt 의 1)매핑 / 2)주행 메뉴로 시작한다
════════════════════════════════════════════════════════════════════════════════
    매핑 : prompt 에서 1 선택 — 스위치가 수동조종이면 즉시, 자율주행이면 내릴 때까지 대기
    주행 : prompt 에서 2 선택(+경로) — 스위치가 자율주행이면 즉시, 수동조종이면 올릴 때까지 대기

    ★B보드 D5 스위치는 더 이상 시작을 트리거하지 않는다★ 진행 중인 것이 스위치
    위치와 안 맞게 되면 취소하고, 도착·저장 시점을 정리하는 역할만 한다 —
    driving.py 헤더의 on_mode_edge 참고.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from white1 import camera_launch, paths, ports


# 파이썬 stdout 버퍼링을 끄지 않으면 노드 로그가 뭉쳐서 늦게 나온다
NODE_ENV = {'PYTHONUNBUFFERED': '1',
            'RCUTILS_LOGGING_BUFFERED_STREAM': '0'}
RESPAWN_DELAY = 2.0


def generate_launch_description():
    package_name = 'white1'

    print("\n=====================================================")
    print(" 🔌 하드웨어 장치 경로 확인 (GPS / IMU)")
    print("    아두이노 A/B 는 arduino 노드가 텔레메트리 접두어로 자체 식별합니다.")
    print("    ※ 미리 확인하려면: ros2 run nxde check")

    # 못 찾아도 실패가 아니다 — resolve_device 는 udev 링크 → VID/PID → (없으면)
    # 링크 경로를 그대로 돌려준다. 나중에 꽂으면 자체 재연결·respawn 이 붙는다.
    used = set()
    gps_dev = ports.resolve_device(ports.SYMLINK_GPS, ports.GPS_VIDPID,
                                   exclude=used, log=lambda m: print(f"    [GPS] {m}"))
    used.add(gps_dev)
    imu_dev = ports.resolve_device(ports.SYMLINK_IMU, ports.IMU_VIDPID,
                                   exclude=used, log=lambda m: print(f"    [IMU] {m}"))
    used.add(imu_dev)
    print("=====================================================\n")

    exclude_for_arduino = [gps_dev, imu_dev]

    # 카메라(신호등 인지) — 조각은 white1/camera_launch.py 가 소유한다
    cam_dev, cam_format = camera_launch.banner(ports)

    use_arduino = LaunchConfiguration('use_arduino')
    use_record  = LaunchConfiguration('use_record')
    use_mapping = LaunchConfiguration('use_mapping')
    use_sound   = LaunchConfiguration('use_sound')

    args = [
        DeclareLaunchArgument(
            'use_arduino', default_value='true',
            description='nxde 의 arduino 노드(A/B 2보드)를 함께 띄울지. false 면 별 '
                        '터미널에서 `ros2 run nxde arduino` 로 직접 띄운다'),
        DeclareLaunchArgument(
            'use_record', default_value='true',
            description='record 노드(주행 CSV 기록). 자율주행 구간에서만 파일이 생긴다'),
        DeclareLaunchArgument(
            'use_mapping', default_value='true',
            description='mapping 노드(경로 수집). prompt 에서 1)매핑을 시작해야 동작한다'),
        DeclareLaunchArgument(
            'use_sound', default_value='true',
            description='nxde 의 sound 노드(음성 안내). 뜨는 즉시 one_launch_1 이 나오고, '
                        'A·B 보드 연결·매핑/주행 시작·도착·E-stop 을 토픽으로 보고 안내한다. '
                        '구독만 하므로 제어에는 영향이 없다 — 조용히 쓰려면 false'),
        DeclareLaunchArgument(
            'gps_port', default_value=gps_dev,
            description='GPS 시리얼 경로 override (기본: udev 링크 → VID/PID 스캔)'),
        DeclareLaunchArgument(
            'imu_port', default_value=imu_dev,
            description='iAHRS 시리얼 경로 override'),
        DeclareLaunchArgument(
            'imu_sync_period_ms', default_value='50',
            description='IMU 출력주기[ms]. 기본 50 = 20Hz (driving 제어주기와 동일)'),
        DeclareLaunchArgument(
            'baud', default_value='115200',
            description='A/B 보드 공통 시리얼 보드레이트'),
        DeclareLaunchArgument(
            'steer_invert', default_value='false',
            description='조향 부호 반전. ★기본 false★ — ROS 토픽과 B보드가 같은 규약'
                        '(− 좌 / + 우)이다. 배선을 뒤집었을 때만 true'),
        DeclareLaunchArgument(
            'stop_brake_level', default_value='0',
            description='/control_state=False 일 때 arduino 가 걸 브레이크 단계. '
                        '★0 을 권한다★ — 정지 시 리니어는 driving 이 직접 지시한다'),
        DeclareLaunchArgument(
            'manual_pulse_max', default_value='15',
            description='수동조종에서 페달 최대치가 대응할 펄스'),

        # ── 주행 튜닝 (driving.py 상단 상수의 런치 override) ──
        DeclareLaunchArgument(
            'drive_pulse', default_value='4',
            description='★주행 고정 속도[펄스]★ 4 ≈ 12.7 km/h (1펄스 ≈ 3.18 km/h). '
                        '[2026-08-11] driving.py 의 MAX_PULSE_LIMIT(4)로 잘린다 — '
                        '이보다 크게 넣어도 4 로 내려간다'),
        DeclareLaunchArgument(
            'heading_pulse', default_value='3',
            description='헤딩 초기화 중 속도[펄스] 3 ≈ 9.5 km/h'),
        # ★[2026-08-11] steer_kp 가 사라졌다★ 순수추종(Pure Pursuit)으로 바뀌어
        #   비례게인이 없다. 조향 세기는 lfd_omega_n 으로 만진다.
        DeclareLaunchArgument(
            'lfd_omega_n', default_value='0.97',
            description='순수추종 목표 고유진동수[rad/s] → LFD = v·√2/ω_n. '
                        '★낮추면 LFD 가 길어져 조향이 완만해진다★ (사행·발산 시 낮춘다). '
                        '구 white 로스백 실측 발산임계가 1.2 이므로 그 위로 올리지 말 것'),
        DeclareLaunchArgument(
            'lfd_min_m', default_value='2.3',
            description='LFD 하한[m]. ★최소회전반경(1.49m)보다 넉넉히 커야 한다★ — '
                        '낮추면 목표점이 회전반경 안으로 들어와 제자리를 돈다'),
        DeclareLaunchArgument(
            'wheelbase_m', default_value='1.25',
            description='축거[m] 실측 1250mm. 순수추종 조향식의 L'),
        # ★[2026-08-11] 조향 전달계 실측 보정★ B보드의 ±40° 는 가변저항 행정 이름일
        #   뿐 도로휠각이 아니다(실측 링키지비 1.75). driving.py 상단 주석 참고.
        DeclareLaunchArgument(
            'steer_plant_gain', default_value='1.26',
            description='pot 지령 / 도로휠각. 126표본 최소자승 실측(1.75)을 '
                        '[2026-08-12] 가변저항 하드리밋 재측정에 맞춰 ×0.7185 '
                        '재환산한 값. ★코너를 여전히 크게 돌면 올린다(더 꺾는다)★'),
        DeclareLaunchArgument(
            'steer_understeer', default_value='5.17',
            description='언더스티어 계수 [deg/(m/s²)]. 같은 반경이라도 속도가 오르면 '
                        '더 꺾어야 하는 양. 고속 코너에서 부족하면 올린다'),
        DeclareLaunchArgument(
            'cte_ki', default_value='0.30',
            description='CTE 적분 게인 [deg(도로휠)/(m·s)]. ★크게 잡지 말 것★ — '
                        '순수추종이 못 지우는 정상상태 측방편향(실측 +0.13~0.27m)을 '
                        '천천히 지우는 용도다. 0 이면 적분항을 끈다. 기여는 '
                        'driving.py 의 CTE_I_MAX_DEG(2.5°)로 한 번 더 잘린다'),
        # ── 종점 접근 [2026-08-12 도입 → 2026-08-19 개편] ── driving.py '종점 접근' 절
        DeclareLaunchArgument(
            'goal_brake_m', default_value='20.0',
            description='★종점 감시 창[m]★ (2026-08-19 로 뜻이 바뀌었다 — 종전에는 '
                        '"이 거리에서 2단을 문다"였고 5.0 이었다). 지금은 이 창 안에서 '
                        '★속도로 계산한 지점★ 에 리니어 1단을 문다 — 4펄스면 15.8m 가 '
                        '필요하므로 5.0 으로는 좁다. ★0 으로 주면 이 단계 자체가 '
                        '꺼진다★(도착 시에만 리니어를 무는 종전 거동). 5.0 을 주면 '
                        '사실상 2026-08-12 의 거동(5m 에서 2단 백스톱)으로 돌아간다. '
                        '남은 호길이와 직선거리가 둘 다 이 안일 때만 걸린다 — 순환 '
                        '코스 출발점 오작동 방지'),
        DeclareLaunchArgument(
            'goal_brake1_ms2', default_value='1.30',
            description='★종점 1단 제동의 가정 감속도 [m/s²]★ 체결 지점이 이 값으로 '
                        '정해진다(작을수록 일찍 문다). 기본 1.30 은 2026-08-19 실차의 '
                        '구동차단 1단 실측이다. ⚠️ 올리면 늦게 물어 종점을 지나칠 수 '
                        '있고, 내리면 일찍 서서 크립 구간만 길어진다 — ★모르면 내리는 '
                        '쪽★. 로그의 goal_phase=1 구간 gps_kmh 기울기가 실측값이다'),
        DeclareLaunchArgument(
            'goal_brake2_backstop', default_value='true',
            description='1단으로는 못 세운다고 계산되면 ★리니어 2단으로 올린다★. '
                        '이것이 종점 통과를 막는 마지막 방벽이다(2026-08-12 의 5m 2단 '
                        '접근제동이 필요할 때만 되살아나는 것과 같다). ★끄지 말 것★ — '
                        '끄면 1단이 안 들을 때 차가 종점을 그대로 지나간다'),
        DeclareLaunchArgument(
            'goal_creep_kmh', default_value='4.0',
            description='접근제동 중 GPS 속도가 이 밑으로 내려오고 종점이 1.2m 넘게 '
                        '남았으면 리니어를 풀고 1펄스 크립으로 마저 간다 — ★정지한 차를 '
                        '다시 떼어내는 것이 제일 어렵기 때문에★ 구르는 채로 넘긴다. '
                        '완전히 선 경우에는 크립 재출발 킥이 떼어낸다'),
        DeclareLaunchArgument(
            'wp_reach_m', default_value='0.9',
            description='마지막 WP 도착 허용반경[m]. ★[2026-08-11] 0.2 → 0.9★ — '
                        'GPS 가 5Hz 라 0.2m 창은 접근속도에서 통째로 건너뛴다(실측 '
                        '최근접 0.24·0.38m 로 두 주행 다 도착 판정 실패). 반경과 '
                        '별개로 마지막 WP 를 지나쳤는지도 함께 본다 — driving.py '
                        'run_follow() 의 종점 판정 참고'),
        DeclareLaunchArgument(
            'require_rtk', default_value='true',
            description='헤딩 초기화·코스 융합에 품질 문턱을 걸지. ★true 권장★ — '
                        'false 면 품질을 아예 안 본다(SPS σ4m 도 통과). 문턱의 높이는 '
                        '아래 min_quality 가 정한다'),
        DeclareLaunchArgument(
            'min_quality', default_value='2',
            description='★헤딩 초기화·코스 융합을 허용할 최저 GPS 품질★ '
                        '1=SPS 2=DGPS 3=RTK_FLOAT 4=RTK_FIXED. 기본 2(DGPS) — '
                        '2026-08-18 실차에서 DGPS 헤딩 오차가 RTK Fixed 와 같은 급'
                        '(−0.8~−4.4° vs +2.0°)임을 로그 재생으로 확인해 내렸다. '
                        '⚠️ 1(SPS)로 내리지 말 것 — HeadingEstimator 가 5m 를 가면 '
                        '정확도 미달이어도 강제확정하므로 엉뚱한 헤딩이 박힌다. '
                        '4 로 올리면 Fixed 가 아닐 때 아예 출발하지 않는다(안전하지만 '
                        '조용한 실패 — 로그의 "헤딩 표본이 쌓이지 않는다" 경고를 볼 것)'),

        # ── 저장 위치 (비우면 white1/paths.py 규칙) ──
        DeclareLaunchArgument(
            'data_dir', default_value='',
            description='경로(맵) CSV 폴더. 비우면 소스트리의 white1/gps_data/'),
        DeclareLaunchArgument(
            'record_dir', default_value='',
            description='주행 기록 CSV 폴더. 비우면 소스트리의 white1/ros2bag/'),
        # ★[2026-08-14] 음원이 white1/sound 로 옮겨 왔다★ nxde 의 sound 노드는
        #   기본적으로 자기 패키지(<nxde>/sound)를 보므로, 여기서 경로를 넘겨 준다.
        DeclareLaunchArgument(
            'sound_dir', default_value=paths.sound_dir(),
            description='음성 안내 mp3 폴더. 기본은 소스트리의 white1/sound/ — '
                        '안내 문구가 전부 white1 의 사건이라 음원의 주인도 white1 이다'),
    ] + camera_launch.declare_args(cam_dev)     # 카메라·신호등 인자

    # ═══════════════════════════════════════════════════════════════════
    #  [하드웨어] 아두이노 A/B — nxde 패키지, ★수정 없이 사용★
    # ═══════════════════════════════════════════════════════════════════
    arduino = Node(
        package='nxde',
        executable='arduino',
        name='arduino',
        output='screen',
        additional_env=NODE_ENV,
        parameters=[{
            'baud':             LaunchConfiguration('baud'),
            'steer_invert':     LaunchConfiguration('steer_invert'),
            'stop_brake_level': LaunchConfiguration('stop_brake_level'),
            'manual_pulse_max': LaunchConfiguration('manual_pulse_max'),
            'exclude_ports':    exclude_for_arduino,
        }],
        condition=IfCondition(use_arduino),
    )

    # ═══════════════════════════════════════════════════════════════════
    #  [하드웨어] iAHRS IMU → /imu. 자체 2초 재연결 + VID/PID 재탐색.
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
    #  [계측] /imu 적분 속도계 → /speed [km/h]  [2026-08-12 신설]
    #    driving 의 ★저속 펄스 보정★ 이 이 값을 쓴다. 안 떠 있어도 driving 은
    #    그대로 돌고, 보정이 엔코더로 내려갈 뿐이다(measured_pulse).
    # ═══════════════════════════════════════════════════════════════════
    speed = Node(
        package=package_name,
        executable='speed',
        name='speed_node',
        output='screen',
        additional_env=NODE_ENV,
        respawn=True,
        respawn_delay=RESPAWN_DELAY,
    )

    # ═══════════════════════════════════════════════════════════════════
    #  [하드웨어] u-blox GPS → /fix. 외부 패키지라 respawn 에 의존한다.
    #    ★RTCM 보정을 넣지 않는다★ RTK 여부는 수신기가 만들고 GGA quality 로 드러난다.
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
    #  [GPS 후처리] gps — /fix + /imu → /gps_fused   ★[2026-08-18] 신설★
    #    ① RTK Fixed / Float 판정 : nmea_navsat_driver 는 GGA q4(Fixed, 2cm)와
    #       q5(Float, 수 m)를 ★같은 status.status=2 로★ 내보낸다. 두 값을 가르는
    #       것은 position_covariance 이고 그 판정을 이 노드가 한다(gps.py 헤더 ①절).
    #    ② 5Hz fix 사이 공백을 IMU 로 메워 20Hz 가상좌표를 만든다 — 4.42m/s 에서
    #       마지막 틱의 좌표는 최대 0.88m 낡아 있었다.
    #    ★없으면 차가 서 있는다★ driving 이 /gps_fused 를 못 받아 GPS 두절로 판단한다
    #    (안전한 실패). driving 이 그 경우를 구별해서 경고한다.
    #    ★매핑에는 영향이 없다★ mapping 은 /fix 원값을 직접 받는다.
    # ═══════════════════════════════════════════════════════════════════
    gps_post = Node(
        package=package_name,
        executable='gps',
        name='gps_node',
        output='screen',
        additional_env=NODE_ENV,
        respawn=True,
        respawn_delay=RESPAWN_DELAY,
        parameters=[{'min_quality': LaunchConfiguration('min_quality')}],
    )

    # ═══════════════════════════════════════════════════════════════════
    #  [자율주행] driving — 헤딩·상태기계·추종·브레이크를 맡는다
    #    (위치는 gps 노드가 만든다 — [2026-08-18])
    # ═══════════════════════════════════════════════════════════════════
    driving = Node(
        package=package_name,
        executable='driving',
        name='driving_node',
        output='screen',
        additional_env=NODE_ENV,
        parameters=[{
            'data_dir':      LaunchConfiguration('data_dir'),
            'drive_pulse':   LaunchConfiguration('drive_pulse'),
            'heading_pulse': LaunchConfiguration('heading_pulse'),
            'lfd_omega_n':   LaunchConfiguration('lfd_omega_n'),
            'lfd_min_m':     LaunchConfiguration('lfd_min_m'),
            'wheelbase_m':   LaunchConfiguration('wheelbase_m'),
            'steer_plant_gain': LaunchConfiguration('steer_plant_gain'),
            'steer_understeer': LaunchConfiguration('steer_understeer'),
            'cte_ki':        LaunchConfiguration('cte_ki'),
            'goal_brake_m':   LaunchConfiguration('goal_brake_m'),
            'goal_creep_kmh': LaunchConfiguration('goal_creep_kmh'),
            'goal_brake1_ms2': LaunchConfiguration('goal_brake1_ms2'),
            'goal_brake2_backstop': LaunchConfiguration('goal_brake2_backstop'),
            'wp_reach_m':    LaunchConfiguration('wp_reach_m'),
            'require_rtk':   LaunchConfiguration('require_rtk'),
        }],
    )

    mapping = Node(
        package=package_name,
        executable='mapping',
        name='mapping_node',
        output='screen',
        additional_env=NODE_ENV,
        parameters=[{'data_dir': LaunchConfiguration('data_dir')}],
        condition=IfCondition(use_mapping),
    )

    # 구독만 하는 노드라 제어에 끼어들지 않는다(발행 토픽 없음).
    record = Node(
        package=package_name,
        executable='record',
        name='record_node',
        output='screen',
        additional_env=NODE_ENV,
        parameters=[{'output_dir': LaunchConfiguration('record_dir')}],
        condition=IfCondition(use_record),
    )

    # ═══════════════════════════════════════════════════════════════════
    #  [안내] 음성 — nxde 패키지, 구독 전용
    #    ★respawn 을 걸지 않는다★ 되살아날 때마다 시작 안내(one_launch_1)가 다시
    #    나오는 편이 조용히 없는 것보다 헷갈린다. 죽어도 주행에는 영향이 없다.
    # ═══════════════════════════════════════════════════════════════════
    sound = Node(
        package='nxde',
        executable='sound',
        name='sound_node',
        output='screen',
        additional_env=NODE_ENV,
        parameters=[{'sound_dir': LaunchConfiguration('sound_dir')}],
        condition=IfCondition(use_sound),
    )

    return LaunchDescription(args + [
        # 음성 먼저 — '런치했다'는 안내가 하드웨어 탐색보다 늦으면 의미가 없다
        sound,
        # 하드웨어
        arduino,
        iahrs,
        speed,
        gps,
        # GPS 후처리 — driving 이 이 노드의 출력에 의존한다(먼저 띄운다)
        gps_post,
        # 자율주행
        driving,
        mapping,
        record,
    ] + camera_launch.actions(package_name, cam_format, NODE_ENV, RESPAWN_DELAY))
