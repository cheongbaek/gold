#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""one_launch.py — 금색차 MPPI 로컬 장애물 회피 통합 런치

    ros2 launch mppi_local_planner one_launch.py
    ros2 launch mppi_local_planner one_launch.py use_rviz:=true
    ros2 launch mppi_local_planner one_launch.py drive:=false use_rviz:=true  # 책상

╔══════════════════════════════════════════════════════════════════════════╗
║ ⚠️ ★★ 이 런치는 '실행 = 출발' 이다 ★★                                   ║
║    외장 iAHRS 헤딩 잠금(차 정지, ~1.5초)이 끝나면 ★2펄스(≈6.4 km/h)로   ║
║    스스로 직진한다. 전방에 장애물이 있으면 원본 MPPI 와 같이 S커브로     ║
║    감아 피하고 IMU 기준선으로 돌아온다. 피할 길이 없으면 리니어 2단.     ║
║    세우는 수단 :  Ctrl-C  ·  E-STOP  ·  D5 수동조종                      ║
╚══════════════════════════════════════════════════════════════════════════╝

띄우는 것:
    nxde/sound            음성 안내 (구독 전용)      use_sound
    nxde/arduino          A/B 2보드 시리얼 브리지    use_arduino
    lidar/ouster.launch   OS1-32 드라이버            use_ouster
    white1/iahrs          외장 iAHRS → /imu          use_iahrs
    mppi_local_planner_node
    rviz2                                            use_rviz
    white1/hud            차량 상면도 HUD            use_hud
                          NAV = MPPI 2D 탑뷰 (차·장애물·경로)

★실차 전 점검★
    1) D5 스위치가 ★자율주행★ 인가
    2) E-STOP 이 풀려 있는가 (해제가 곧 출발이다)
    3) 런치 직후 차가 ★정지해 있는가★ — 외장 iAHRS 헤딩 잠금
    4) 차 앞이 비어 있는가
"""

import os
from glob import glob

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


NODE_ENV = {'PYTHONUNBUFFERED': '1',
            'RCUTILS_LOGGING_BUFFERED_STREAM': '0'}


def _sound_dir():
    """음성 안내 mp3 폴더. 음원의 주인은 white1 이다."""
    try:
        share = os.path.join(get_package_share_directory('white1'), 'sound')
        if glob(os.path.join(share, '*.mp3')):
            return share
    except Exception:  # noqa: BLE001
        pass
    try:
        from white1 import paths  # noqa: PLC0415
        src = paths.sound_dir()
        if glob(os.path.join(src, '*.mp3')):
            return src
    except Exception:  # noqa: BLE001
        pass
    return ''


def generate_launch_description():
    pkg = get_package_share_directory('mppi_local_planner')
    default_params = os.path.join(pkg, 'config', 'params.yaml')
    snd = _sound_dir()
    drive = LaunchConfiguration('drive')

    imu_dev = '/dev/imu'
    try:
        from white1 import ports as wports  # noqa: PLC0415
        imu_dev = wports.resolve_device(
            wports.SYMLINK_IMU, wports.IMU_VIDPID,
            log=lambda m: print(f"    [IMU] {m}"))
    except Exception as exc:  # noqa: BLE001
        print(f"    [IMU] white1.ports 를 못 썼습니다 ({exc}) — {imu_dev} 로 시도")

    try:
        lidar_pkg = get_package_share_directory('lidar')
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError(
            "lidar 패키지를 찾을 수 없습니다. OS1-32 드라이버 런치의 주인입니다.\n"
            "    colcon build --packages-select lidar ouster_sensor_msgs ouster_ros \\\n"
            "        --cmake-args -DCMAKE_BUILD_TYPE=Release\n"
            "    source ~/gold/gold_ws/install/setup.bash"
        ) from exc

    print("\n=====================================================")
    print(" 🚗 mppi_local_planner one_launch — ★MPPI 장애물 회피★")
    print("    순항 = 2펄스 = 1.768 m/s ≈ 6.4 km/h")
    print("    주행 = 런치 후 외장 iAHRS 헤딩 잠금(~1.5초, 정지) → 스스로 직진")
    print("    헤딩 = /imu (white1 iahrs, AHRS 쿼터니언). /ouster/imu 가 아님")
    print("    회피 = 원본 MPPI S커브 / 실패 시 리니어 2단")
    print(f"    음원 = {snd or '(못 찾음 — 안내음 없이 돕니다)'}")
    print("    라이다는 유선 LAN 이다 — 안 뜨면 먼저:")
    print("        ip -br addr show eno1   /   ping -c2 192.168.6.11")
    print("=====================================================\n")

    args = [
        DeclareLaunchArgument(
            'use_arduino', default_value='true',
            description='nxde arduino(A/B 2보드)를 함께 띄울지'),
        DeclareLaunchArgument(
            'use_sound', default_value='true',
            description='nxde sound(음성 안내). 구독만 하므로 제어에는 영향이 없다'),
        DeclareLaunchArgument(
            'use_hud', default_value='true',
            description='white1 hud 계기판. DISPLAY 없는 SSH 면 false'),
        DeclareLaunchArgument(
            'use_ouster', default_value='true',
            description='라이다 드라이버를 함께 띄울지. false 면 rosbag 재생·별 터미널'),
        DeclareLaunchArgument(
            'use_iahrs', default_value='true',
            description='white1 iahrs(외장 iAHRS → /imu). 끄면 imu_topic 을 '
                        '/ouster/imu 로 바꿔 라이다 내장 자이로를 쓴다(드리프트 큼)'),
        DeclareLaunchArgument(
            'imu_port', default_value=imu_dev,
            description='iAHRS 시리얼 경로'),
        DeclareLaunchArgument(
            'imu_sync_period_ms', default_value='50',
            description='iAHRS 출력주기[ms]. 50 = 20Hz'),
        DeclareLaunchArgument(
            'imu_topic', default_value='/imu',
            description='헤딩 IMU 토픽. 기본 /imu = 외장 iAHRS'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='RViz 로 코스트맵·롤아웃을 보면서 시험'),
        DeclareLaunchArgument(
            'drive', default_value='true',
            description='false 면 D5·E-STOP 게이트를 무시하고 지령만 계산한다. '
                        '책상 시험 전용 — 차에 연결한 채로 쓰지 말 것'),
        # ★이름을 params_file 로 두지 않는다★ include 된 ouster.launch 가
        #   부모 LaunchConfiguration 을 물려받아 드라이버가 이 YAML 을 읽고 죽는다
        #   (lidar aeb.launch.py 헤더에 실제 사고 기록이 있다).
        DeclareLaunchArgument(
            'mppi_params_file', default_value=default_params,
            description='MPPI 노드 파라미터 YAML'),
        DeclareLaunchArgument(
            'desired_speed', default_value='1.768',
            description='순항속도 [m/s]. 기본 1.768 = 2펄스 ≈ 6.4 km/h'),
        DeclareLaunchArgument(
            'cruise_pulse', default_value='2',
            description='액추에이터 펄스 상한. 기본 2 ≈ 6.4 km/h 를 못 넘긴다'),
        DeclareLaunchArgument(
            'flip_lidar_xy', default_value='true',
            description='라이다 xy 180° 반전. lidar cone_lidar.yaml 과 동일'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('steer_invert', default_value='false'),
        DeclareLaunchArgument(
            'stop_brake_level', default_value='0',
            description='/control_state=False 일 때 브레이크. 0 유지 — '
                        '회피 실패 제동은 노드가 /brake_level 로 직접 건다'),
    ]

    sound = Node(
        package='nxde', executable='sound', name='sound_node', output='screen',
        additional_env=NODE_ENV,
        parameters=[{'sound_dir': snd}],
        condition=IfCondition(LaunchConfiguration('use_sound')),
    )

    arduino = Node(
        package='nxde', executable='arduino', name='arduino', output='screen',
        additional_env=NODE_ENV,
        parameters=[{
            'baud':             LaunchConfiguration('baud'),
            'steer_invert':     LaunchConfiguration('steer_invert'),
            'stop_brake_level': LaunchConfiguration('stop_brake_level'),
        }],
        condition=IfCondition(LaunchConfiguration('use_arduino')),
    )

    iahrs = Node(
        package='white1', executable='iahrs', name='iahrs_node',
        output='screen', additional_env=NODE_ENV,
        respawn=True, respawn_delay=2.0,
        parameters=[{
            'port': LaunchConfiguration('imu_port'),
            'baud': 115200,
            'send_tf': True,
            'rescan': True,
            'sync_period_ms': LaunchConfiguration('imu_sync_period_ms'),
        }],
        condition=IfCondition(LaunchConfiguration('use_iahrs')),
    )

    ouster = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_pkg, 'launch', 'ouster.launch.py')),
        condition=IfCondition(LaunchConfiguration('use_ouster')),
        launch_arguments={
            'params_file': os.path.join(lidar_pkg, 'config', 'ouster_driver.yaml'),
        }.items(),
    )

    planner = Node(
        package='mppi_local_planner',
        executable='mppi_local_planner_node',
        name='mppi_local_planner_node',
        output='screen',
        additional_env=NODE_ENV,
        parameters=[
            LaunchConfiguration('mppi_params_file'),
            {
                'mppi.desired_speed': LaunchConfiguration('desired_speed'),
                'kasa.max_pulse': LaunchConfiguration('cruise_pulse'),
                'kasa.require_auto_mode': drive,
                'kasa.require_estop_clear': drive,
                'flip_lidar_xy': LaunchConfiguration('flip_lidar_xy'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'imu_use_orientation': True,
            },
        ],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(pkg, 'config', 'mppi.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    hud = Node(
        package='white1', executable='hud', name='hud_node', output='screen',
        additional_env=NODE_ENV,
        parameters=[{
            'show_camera': False,
            'nav_mode': 'mppi',
            'lidar_range_min': 2.0,
            'lidar_range_max': 15.0,
            'vehicle_front_m': 1.2,
            'corridor_half_m': 0.8,
            'wheelbase_m': 1.25,
            'track_width_m': 1.10,
            'rear_overhang_m': 0.30,
        }],
        condition=IfCondition(LaunchConfiguration('use_hud')),
    )

    return LaunchDescription(args + [
        sound,
        arduino,
        iahrs,
        ouster,
        planner,
        rviz,
        hud,
    ])
