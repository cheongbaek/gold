#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""aeb.launch.py — 가상범퍼 AEB 만 띄운다 (★차를 움직이지 않는다★)

    ros2 launch lidar aeb.launch.py                  # 라이다 + AEB 판정
    ros2 launch lidar aeb.launch.py use_rviz:=true   # RViz 로 ROI 를 보면서
    ros2 launch lidar aeb.launch.py use_ouster:=false  # 드라이버는 따로 띄운 경우

★이식에서 제일 먼저 이걸 돌려 볼 것★ 이유:
  · cone_lidar_node 는 `/cmd_vel_raw` 를 내지 않는다 — 차가 절대 안 움직인다
  · 그래서 ★장착 파라미터(높이·앞뒤 방향)를 안전하게 실측 검증할 수 있다★
  · 여기서 obstacle_distance 가 맞게 나오지 않으면, 주행 노드를 아무리 고쳐도
    소용없다. 인지가 먼저다.

확인 :
    ros2 topic echo /cone_lidar_node/obstacle_distance
    ros2 topic echo /cone_lidar_node/stop_signal

★앞뒤 방향 확인법★ 차 앞 3 m 에 사람이 서서 obstacle_distance ≈ 3.0 이 나오면
flip_lidar_xy 가 맞다. 앞에서 inf 인데 뒤에서 잡히면 뒤집힌 것이다:
    ros2 param set /cone_lidar_node flip_lidar_xy false
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

    return LaunchDescription([
        DeclareLaunchArgument('use_ouster', default_value='true',
                              description='라이다 드라이버를 함께 띄울지'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        # ★인자 이름을 params_file 로 두면 안 된다★ IncludeLaunchDescription 은
        #   부모의 LaunchConfiguration 을 자식에게 물려준다. 그래서 여기서
        #   params_file=cone_lidar.yaml 을 선언하면 아래 ouster.launch.py 의
        #   같은 이름 인자를 ★덮어써서★ 드라이버가 cone_lidar.yaml 을 자기
        #   파라미터로 읽고 "Must specify a sensor hostname" 으로 죽는다.
        #   (실제로 그랬다 — 2026-08-25. 이름을 갈라 두는 것이 유일한 방어다)
        DeclareLaunchArgument('cone_params_file',
                              default_value=os.path.join(pkg, 'config',
                                                         'cone_lidar.yaml')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'ouster.launch.py')),
            condition=IfCondition(LaunchConfiguration('use_ouster')),
            # 자식의 기본값(ouster_driver.yaml)을 쓰게 명시적으로 넘긴다
            launch_arguments={
                'params_file': os.path.join(pkg, 'config', 'ouster_driver.yaml'),
            }.items(),
        ),

        Node(
            package='lidar', executable='cone_lidar_node',
            name='cone_lidar_node', output='screen',
            parameters=[LaunchConfiguration('cone_params_file')],
        ),

        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', os.path.join(pkg, 'config', 'drive_lidar.rviz')],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
