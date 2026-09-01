#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""drive_lidar.launch.py — 라바콘 코리도 측량 잠금 + 헤딩홀드 직진

╔══════════════════════════════════════════════════════════════════════════╗
║ ⚠️ ★★ 이 런치는 '실행 = 출발' 이다 ★★                                   ║
║    시작 명령 토픽이 없다. survey_duration(기본 2.5초) 동안 서서 라바콘을  ║
║    모으고, 복도가 잠기는 순간 ★스스로 굴러간다★.                          ║
║    세우는 수단은 셋뿐이다 :  Ctrl-C  ·  E-STOP  ·  AEB                    ║
║    white1 처럼 prompt 로 "주행 시작"을 누르는 절차가 ★없다★.              ║
╚══════════════════════════════════════════════════════════════════════════╝

    # 책상 시험 (구동 게이트를 끄고 지령만 본다 — 차에 연결하지 말 것)
    ros2 launch lidar drive_lidar.launch.py drive:=false use_rviz:=true

    # 실차 — ★권장은 lidar one_launch.py cone_drive:=true ★
    #   (arduino · iAHRS · AEB 래치 · HUD 가 한 런치에 있다)
    ros2 launch lidar one_launch.py cone_drive:=true
    ros2 launch lidar drive_lidar.launch.py
    ros2 launch lidar drive_lidar.launch.py linear_speed:=0.884   # 1펄스

    # rosbag 재생 (이미 움직이는 백이면 관측을 짧게)
    ros2 launch lidar drive_lidar.launch.py use_ouster:=false drive:=false \
        survey_duration:=0.4 use_rviz:=true
    ros2 bag play ~/catkin_ws/rosbag/txa1

★linear_speed 는 m/s 지만 실제로는 정수 펄스로 반올림된다★
    1펄스 0.884 (3.2 km/h) / 2펄스 1.768 (6.4 km/h) / ★7펄스 6.188 (22.3 km/h · 기본)★
    3펄스 2.652 (9.5 km/h) /  4펄스 3.536 (12.7 km/h)
  ⚠️ 4펄스는 ★정지 상태 재출발에서 A보드 적분이 동결★ 되는 값이다(PWM 92 고정,
     2펄스보다 약하다). 굴러가는 중에는 문제없지만 AEB 로 세운 뒤가 위험하다.

★실차 전 점검★
    1) D5 스위치가 ★자율주행★ 인가          (아니면 ROS 명령이 전부 무시된다)
    2) E-STOP 이 풀려 있는가                 (해제가 곧 출발이다 — 차 주변 확인)
    3) ros2 topic echo /vehicle_mode 가 true 인가
    4) 차 앞이 비어 있는가 — ★런치하면 2.5초 뒤 출발한다★
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('lidar')
    drive = LaunchConfiguration('drive')

    imu_dev = '/dev/imu'
    try:
        from white1 import ports as wports  # noqa: PLC0415
        imu_dev = wports.resolve_device(
            wports.SYMLINK_IMU, wports.IMU_VIDPID,
            log=lambda m: print(f"    [IMU] {m}"))
    except Exception as exc:  # noqa: BLE001
        print(f"    [IMU] white1.ports 를 못 썼습니다 ({exc}) — {imu_dev} 로 시도")

    return LaunchDescription([
        DeclareLaunchArgument('use_ouster', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument(
            'use_iahrs', default_value='true',
            description='white1 iahrs(외장 iAHRS → /imu). 끄면 imu_topic 을 '
                        '/ouster/imu 로 바꿔 라이다 내장 자이로를 쓴다(드리프트 큼)'),
        DeclareLaunchArgument(
            'imu_port', default_value=imu_dev,
            description='iAHRS 시리얼 경로'),
        DeclareLaunchArgument(
            'imu_topic', default_value='/imu',
            description='헤딩 IMU 토픽. 기본 /imu = 외장 iAHRS'),
        DeclareLaunchArgument(
            'drive', default_value='true',
            description='false 면 D5·E-STOP 게이트를 무시하고 ★지령만★ 계산한다. '
                        '책상 시험 전용 — 차에 연결한 채로 쓰지 말 것'),
        DeclareLaunchArgument(
            'linear_speed', default_value='6.188',
            description='순항속도 [m/s]. 정수 펄스로 반올림된다 '
                        '(1펄스=0.884 / ★7펄스=6.188 기본 ≈ 22.3 km/h★)'),
        DeclareLaunchArgument(
            'survey_duration', default_value='2.5',
            description='기동 후 정지 관측 [s]. ★이 시간이 지나면 출발한다★'),
        DeclareLaunchArgument('brake_enable', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'ouster.launch.py')),
            condition=IfCondition(LaunchConfiguration('use_ouster')),
            # ★인자 누수 방어★ include 는 부모의 LaunchConfiguration 을 물려준다
            launch_arguments={
                'params_file': os.path.join(pkg, 'config', 'ouster_driver.yaml'),
            }.items(),
        ),

        Node(
            package='white1', executable='iahrs', name='iahrs_node',
            output='screen',
            respawn=True, respawn_delay=2.0,
            parameters=[{
                'port': LaunchConfiguration('imu_port'),
                'baud': 115200,
                'send_tf': True,
                'rescan': True,
                'sync_period_ms': 50,
            }],
            condition=IfCondition(LaunchConfiguration('use_iahrs')),
        ),

        Node(
            package='lidar', executable='cone_lidar_node',
            name='cone_lidar_node', output='screen',
            parameters=[os.path.join(pkg, 'config', 'cone_lidar.yaml')],
        ),

        Node(
            package='lidar', executable='drive_lidar_node',
            name='drive_lidar_node', output='screen',
            parameters=[
                os.path.join(pkg, 'config', 'drive_lidar.yaml'),
                {
                    'linear_speed': LaunchConfiguration('linear_speed'),
                    'survey_duration': LaunchConfiguration('survey_duration'),
                    'brake_enable': LaunchConfiguration('brake_enable'),
                    'imu_topic': LaunchConfiguration('imu_topic'),
                    'imu_use_orientation': True,
                    'kasa.max_pulse': 7,
                    # drive:=false → 게이트를 끈다(책상). true → 하드웨어를 본다.
                    'kasa.require_auto_mode': drive,
                    'kasa.require_estop_clear': drive,
                },
            ],
        ),

        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', os.path.join(pkg, 'config', 'drive_lidar.rviz')],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
