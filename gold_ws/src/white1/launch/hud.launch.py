#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""hud.launch.py ― 차량 상면도 HUD 만 띄운다 (구독 전용)

    ros2 launch white1 hud.launch.py
    ros2 launch white1 hud.launch.py show_camera:=false

이미 one_launch / lidar one_launch / arduino 가 떠 있는 위에 얹는다.
/cmd_vel_raw 를 발행하지 않으므로 prompt · master · joystick 과 겹치지 않는다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NODE_ENV = {'PYTHONUNBUFFERED': '1',
            'RCUTILS_LOGGING_BUFFERED_STREAM': '0'}


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'show_camera', default_value='true',
            description='/image_raw 미리보기. 카메라가 없으면 false'),
        DeclareLaunchArgument(
            'data_dir', default_value='',
            description='매핑 CSV 폴더. 비우면 white1/gps_data'),
        Node(
            package='white1',
            executable='hud',
            name='hud_node',
            output='screen',
            additional_env=NODE_ENV,
            parameters=[{
                'show_camera': LaunchConfiguration('show_camera'),
                'data_dir': LaunchConfiguration('data_dir'),
            }],
        ),
    ])
