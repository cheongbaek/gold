#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""drive_gps.launch.py — GPS 일자 매핑 + 스탠리 추종 (+ 라이다 AEB)

╔══════════════════════════════════════════════════════════════════════════╗
║ ⚠️ ★★ white1 driving.py 와 동시에 띄우지 말 것 ★★                        ║
║    /cmd_vel_raw 발행자가 겹친다. white1 one_launch.py 를 쓰고 있다면       ║
║    driving 을 끄거나, 이 런치를 쓰지 말 것.                               ║
║    실주행 품질은 white1 쪽이 압도적으로 낫다(실차 로그 열댓 번 튜닝).      ║
║    이 노드는 '라이다 AEB 를 GPS 주행에 물리면 어떻게 되나' 시험용이다.     ║
╚══════════════════════════════════════════════════════════════════════════╝

★작동 방식 — white1 prompt 와 다르다★
  white1 :  prompt 메뉴 → /drive_cmd(MAP_START·DRIVE_START·파일명·STOP)
            → driving.py 상태기계가 매핑·헤딩초기화·주행을 순서대로 진행
  이 노드 : ★상태기계가 없다★ Idle / Mapping / Driving 세 모드뿐이고,
            토픽 두 개로 직접 지시한다.

    # ① 매핑 (곧게 굴리면서 켠다 → 끄면 최소자승 직선을 피팅해 CSV 두 벌 저장)
    ros2 topic pub --once /mapping_cmd std_msgs/Bool "{data: true}"
    ros2 topic pub --once /mapping_cmd std_msgs/Bool "{data: false}"

    # ② 주행 (LAST = 마지막 *_straight.csv / 또는 파일명)
    ros2 topic pub --once /drive_cmd std_msgs/String "{data: LAST}"

    # ③ 정지 (★리니어 2단이 걸린다★)
    ros2 topic pub --once /drive_cmd std_msgs/String "{data: STOP}"

    # 상태 보기
    ros2 topic echo /drive_status

실행 :
    ros2 launch lidar drive_gps.launch.py
    ros2 launch lidar drive_gps.launch.py use_ouster:=false   # AEB 없이 GPS 만
    ros2 launch lidar drive_gps.launch.py linear_speed:=0.884 # 1펄스

★센서는 이 런치가 띄우지 않는다★ GPS(/fix→/gps_fused)·IMU·arduino 는 white1
쪽에서 온다. 먼저 그쪽을 띄우되 ★driving 은 빼고★ 띄울 것.
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

    return LaunchDescription([
        DeclareLaunchArgument('use_ouster', default_value='true',
                              description='라이다 + AEB 를 함께 띄울지'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument(
            'drive', default_value='true',
            description='false 면 D5·E-STOP 게이트를 무시한다(책상 전용)'),
        DeclareLaunchArgument(
            'linear_speed', default_value='1.768',
            description='순항속도 [m/s]. 정수 펄스로 반올림된다 '
                        '(★2펄스=1.768 기본★)'),
        DeclareLaunchArgument('brake_enable', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'aeb.launch.py')),
            condition=IfCondition(LaunchConfiguration('use_ouster')),
        ),

        Node(
            package='lidar', executable='drive_gps_node',
            name='drive_gps_node', output='screen',
            parameters=[
                os.path.join(pkg, 'config', 'drive_gps.yaml'),
                {
                    'linear_speed': LaunchConfiguration('linear_speed'),
                    'brake_enable': LaunchConfiguration('brake_enable'),
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
