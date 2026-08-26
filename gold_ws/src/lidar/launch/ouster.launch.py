#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ouster.launch.py — OS1-32 드라이버만 띄운다 (금색차 kasa)

    ros2 launch lidar ouster.launch.py
    ros2 launch lidar ouster.launch.py viz:=true     # ouster 기본 RViz 도 함께

★이 런치는 라이다만 띄운다★ 인지·주행 노드는 다른 런치가 띄운다. 나눠 둔 이유는
드라이버가 ★수명주기(lifecycle) 노드★ 라 센서와 통신이 안 되면 finalized 로 가면서
런치 전체를 내려버리기 때문이다. 인지 노드와 한 런치에 묶으면 "센서를 못 찾았다"가
"패키지가 안 뜬다"로 보인다.

발행 (네임스페이스 ouster + 상대이름):
    /ouster/points   sensor_msgs/PointCloud2   20 Hz (1024x20)
    /ouster/imu      sensor_msgs/Imu          ~100 Hz
    /tf_static       os_sensor / os_lidar / os_imu

★연결 확인 순서★ 라이다는 USB 가 아니라 유선 LAN 이다.
    1) ip -br addr show eno1     → UP + 192.168.6.100/24 인가
    2) ping -c2 192.168.6.11     → 응답하는가
    3) 이 런치                    → "os_driver activating..." 이 뜨는가
    4) ros2 topic hz /ouster/points → 20 Hz 근처인가
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('lidar')
    default_params = os.path.join(pkg, 'config', 'ouster_driver.yaml')

    # ouster_ros 는 ★이 워크스페이스 안★ 에 있다 — gold_ws/src/ouster-ros/
    #   (2026-08-25 이관. 그 전에는 catkin_ws 를 함께 source 해야 했다)
    #   ★0.13.x 로 고정★ 0.14 부터 FW 2.4 미만 센서 연결을 코드에서 거부한다.
    #   이 센서는 FW 2.3.0 이다 — src/ouster-ros/OUSTER_OS1_32_SETUP.md 참고.
    try:
        ouster_launch = os.path.join(
            get_package_share_directory('ouster_ros'), 'launch', 'driver.launch.py')
    except Exception as exc:            # noqa: BLE001 — 런치 파서에도 이유를 남긴다
        raise RuntimeError(
            "ouster_ros 패키지를 찾을 수 없습니다. 드라이버는 gold_ws/src/ouster-ros/ 에 "
            "있으니 빌드했는지 확인하세요:\n"
            "    colcon build --packages-select ouster_sensor_msgs ouster_ros lidar \\\n"
            "        --cmake-args -DCMAKE_BUILD_TYPE=Release\n"
            "    source ~/gold/gold_ws/install/setup.bash"
        ) from exc

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='드라이버 파라미터 YAML (기본 = lidar/config/ouster_driver.yaml)'),
        DeclareLaunchArgument(
            'viz', default_value='false',
            description='ouster_ros 가 딸려 오는 자체 RViz 를 함께 띄울지'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ouster_launch),
            launch_arguments={
                'params_file': LaunchConfiguration('params_file'),
                'viz': LaunchConfiguration('viz'),
                'ouster_ns': 'ouster',
            }.items(),
        ),
    ])
