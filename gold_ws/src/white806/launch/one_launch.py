#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
one_launch.py ― white806 통합 런치 (GPS + IMU + 아두이노 + 자율주행)
════════════════════════════════════════════════════════════════════════════════
    ros2 launch white806 one_launch.py

띄우는 것 (카메라 없음):
    nxde/arduino          A/B 2보드 시리얼 브리지  ★수정 없이 그대로 쓴다★
    white806/iahrs        6축 IMU 드라이버 → /imu
    nmea_navsat_driver    GPS → /fix
    white806/driving      ★GPS+IMU 융합 + 모드스위치 상태기계 + 경로추종★
    white806/mapping      /fix 만 보고 경로 수집
    white806/record       자율주행 구간 토픽 → CSV

CLI 는 따로 띄운다 (별 터미널):
    ros2 run white806 prompt

════════════════════════════════════════════════════════════════════════════════
 조작은 ★B보드 D5 모드 스위치★ 로 한다
════════════════════════════════════════════════════════════════════════════════
    매핑 : 자율 → 수동 (하강)       끝낼 때 수동 → 자율 (상승) = 경로 저장
    주행 : 수동 → 자율 (상승)       도착 후 자율 → 수동 (하강) = 기록 저장 + 리니어 해제

    스위치가 이미 원하는 쪽에 있으면 반대로 한 번 넘겼다 돌아와야 한다 —
    ★위치가 아니라 전환(엣지)이 트리거★ 이기 때문이다. driving.py 헤더 참고.
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
            description='mapping 노드(경로 수집). 스위치 하강 엣지에서만 동작한다'),
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
            description='★주행 고정 속도[펄스]★ 4 ≈ 12.7 km/h (1펄스 ≈ 3.18 km/h)'),
        DeclareLaunchArgument(
            'heading_pulse', default_value='3',
            description='헤딩 초기화 중 속도[펄스] 3 ≈ 9.5 km/h'),
        DeclareLaunchArgument(
            'steer_kp', default_value='0.5',
            description='헤딩오차[deg] → 조향[deg] 비례게인. ★사행이 나면 낮춘다★'),
        DeclareLaunchArgument(
            'wp_reach_m', default_value='0.2',
            description='웨이포인트 도달 허용반경[m]'),
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
            'steer_kp':      LaunchConfiguration('steer_kp'),
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
        gps,
        # 자율주행
        driving,
        mapping,
        record,
    ])
