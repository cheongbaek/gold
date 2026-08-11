#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
one_launch.py ― white806 통합 런치 (GPS + IMU + 아두이노 + 자율주행)
════════════════════════════════════════════════════════════════════════════════
    ros2 launch white806 one_launch.py

띄우는 것 (카메라 없음):
    nxde/arduino          A/B 2보드 시리얼 브리지  ★수정 없이 그대로 쓴다★
    white806/iahrs        6축 IMU 드라이버 → /imu
    white806/speed        /imu 적분 속도계 → /speed [km/h]
    nmea_navsat_driver    GPS → /fix
    white806/driving      ★GPS+IMU 융합 + 모드스위치 상태기계 + 경로추종★
    white806/mapping      /fix 만 보고 경로 수집
    white806/record       자율주행 구간 토픽 → CSV

CLI 는 따로 띄운다 (별 터미널):
    ros2 run white806 prompt

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

from white806 import ports


# 파이썬 stdout 버퍼링을 끄지 않으면 노드 로그가 뭉쳐서 늦게 나온다
NODE_ENV = {'PYTHONUNBUFFERED': '1',
            'RCUTILS_LOGGING_BUFFERED_STREAM': '0'}
RESPAWN_DELAY = 2.0


def generate_launch_description():
    package_name = 'white806'

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

    use_arduino = LaunchConfiguration('use_arduino')
    use_record  = LaunchConfiguration('use_record')
    use_mapping = LaunchConfiguration('use_mapping')

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
            'steer_plant_gain', default_value='1.75',
            description='pot 지령 / 도로휠각. 126표본 최소자승 실측값. '
                        '★코너를 여전히 크게 돌면 올린다(더 꺾는다)★'),
        DeclareLaunchArgument(
            'steer_understeer', default_value='7.2',
            description='언더스티어 계수 [deg/(m/s²)]. 같은 반경이라도 속도가 오르면 '
                        '더 꺾어야 하는 양. 고속 코너에서 부족하면 올린다'),
        DeclareLaunchArgument(
            'wp_reach_m', default_value='0.9',
            description='마지막 WP 도착 허용반경[m]. ★[2026-08-11] 0.2 → 0.9★ — '
                        'GPS 가 5Hz 라 0.2m 창은 접근속도에서 통째로 건너뛴다(실측 '
                        '최근접 0.24·0.38m 로 두 주행 다 도착 판정 실패). 반경과 '
                        '별개로 마지막 WP 를 지나쳤는지도 함께 본다 — driving.py '
                        'run_follow() 의 종점 판정 참고'),
        DeclareLaunchArgument(
            'require_rtk', default_value='true',
            description='헤딩 초기화·추종에 RTK Fixed 를 요구할지. ★true 권장★ — '
                        'Float 에서 초기 헤딩이 20°씩 틀어지면 경로를 벗어난 채 '
                        '필터가 수렴해 버린다(driving.py 헤더 표 참고)'),

        # ── 저장 위치 (비우면 white806/paths.py 규칙) ──
        DeclareLaunchArgument(
            'data_dir', default_value='',
            description='경로(맵) CSV 폴더. 비우면 소스트리의 white806/gps_data/'),
        DeclareLaunchArgument(
            'record_dir', default_value='',
            description='주행 기록 CSV 폴더. 비우면 소스트리의 white806/ros2bag/'),
    ]

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
    #  [자율주행] driving — 융합·상태기계·추종·브레이크를 혼자 맡는다
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

    return LaunchDescription(args + [
        # 하드웨어를 먼저 — 자율주행 노드가 첫 토픽을 놓칠 확률을 줄인다
        arduino,
        iahrs,
        speed,
        gps,
        # 자율주행
        driving,
        mapping,
        record,
    ])
