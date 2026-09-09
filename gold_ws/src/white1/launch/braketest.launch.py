#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
braketest.launch.py ― ★브레이크 제동거리 측정 전용 런치★ [white1 / 2026-09-09]
════════════════════════════════════════════════════════════════════════════════
    ros2 launch white1 braketest.launch.py
    ros2 launch white1 braketest.launch.py route:=route_20260908_203042.csv
    ros2 launch white1 braketest.launch.py drive_pulse:=8
    ros2 launch white1 braketest.launch.py drive_pwm:=140      # ★직접 PWM★

  띄우는 것 (one_launch.py 에서 ★측정에 필요한 것만★ 남겼다):
      lidar/aeb.launch.py   ★라이다 정지 시스템 그대로★ (통째로 include)
                            = ouster.launch.py(OS1-32 드라이버) + cone_lidar_node
      lidar/pedal_drive_node ★AEB 확정·래치★ stop_signal → /aeb_stop
                            (구동은 발행하지 않는다 — 그 파일 헤더 참고)
      → 정지 사슬이 lidar/one_launch.py 와 ★완전히 같다★. braketest 는 끼어들지
        않고 GPS + 지정속도 주행만 한다
      nxde/arduino          A/B 2보드 시리얼 브리지  ★없으면 아무것도 안 움직인다★
      white1/iahrs          6축 IMU → /imu
      white1/speed          /imu 적분 속도계 → /speed
      nmea_navsat_driver    GPS 수신기 → /fix
      white1/gps            /fix + /imu → /gps_fused (RTK 판정 + 20Hz 보간)
      white1/braketest      ★이 시험의 본체★ (driving 대신 들어간다)
      white1/record         토픽 → CSV (ros2bag/) — 나중에 그래프로 되짚는다

  ★띄우지 않는 것과 그 이유★
      driving   — 이 시험의 반대말이다. 코너 감속·종점 접근제동·CTE 적분·라이다
                  이양이 전부 '제동거리를 재는 일'에 개입한다.
      mapping   — 시험 중에 경로를 쓸 이유가 없다.
      prompt    — ★런치 = 출발★ 이다(아래). 고를 것이 없다.
      hud       — 띄워도 되지만 기본은 끈다(use_hud:=true 로 켤 수 있다).
      drive_lidar_node · drive_gps_node
                — lidar 패키지의 ★주행★ 노드들이다. /cmd_vel_raw 발행자가 겹친다.
                  (pedal_drive_node 는 이름과 달리 구동을 안 내므로 함께 띄운다)
      카메라·신호등 — 제동거리와 무관하고, /cmd_vel_raw 발행자만 늘린다.

════════════════════════════════════════════════════════════════════════════════
 ⚠️ 실행 절차 — ★런치 = 출발★ 이다
════════════════════════════════════════════════════════════════════════════════
  0. ★라이다가 붙었는지 먼저 본다★ — 유선 LAN 이다(USB 아님).
        ip -br addr show eno1     # 192.168.6.100/24 여야 한다
        ping -c2 192.168.6.11
     인지만 따로 확인하려면 : ros2 launch lidar aeb.launch.py use_rviz:=true
        차 앞 3 m 에 사람 → /cone_lidar_node/obstacle_distance 가 3.0 이면 맞다
  1. `ros2 run nxde check` 로 A/B보드·GPS·IMU 연결을 먼저 확인한다.
  2. ★차 앞을 비운다.★ 기본 10펄스(31.8 km/h)에서 S 지점 뒤로 13~20 m 를 더 간다
     (braketest.py 헤더의 안전절 계산). ★S 뒤 최소 30 m 가 필요하다.★
  3. D5 스위치를 ★자율주행★ 으로, E-STOP 을 해제한다.
  4. 런치를 띄운다. GPS 품질이 서면 ★스스로 헤딩을 잡고 출발한다.★
     → 확인하고 출발시키고 싶으면 `auto_start:=false` 로 띄우고,
       `ros2 topic pub -1 /braketest_go std_msgs/msg/Bool '{data: true}'`
  5. 달리는 동안 ★둘 중 먼저 오는 것★ 에서 선다 —
     ① 라이다 장애물 → ★arduino AEB 가 문다★ (braketest 는 관찰만)
     ② 종점 도달     → braketest 가 /brake_level 2단
     완전정지하면 결과를 찍고 ★리니어를 풀고★ 런치가 스스로 내려간다.
     ⚠️ ①로 섰을 때는 ★장애물을 치워야★ arduino 가 리니어를 푼다(설계상 그렇다).
        치우지 않으면 20초 뒤 물린 채로 런치가 내려간다 — 그때는 D5 를 수동조종으로
        내렸다 올리면 풀린다(모드 전환은 반드시 리니어를 푼다).

  ⚠️ 라이다 ROI 는 전방 2.0~6.0 m 인데 10펄스 2단 정지거리가 12.9~20.4 m 다 —
     ★보고 나서 서기에는 원리적으로 부족하다.★ 이 시험은 '얼마나 못 서는가' 를
     재는 것이다. 사람·부술 수 있는 물건을 장애물로 쓰지 말 것.

  언제든 세우는 방법 : E-STOP(하드웨어) · D5 를 수동조종으로 · Ctrl-C
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from white1 import ports

PACKAGE = 'white1'
NODE_ENV = {'PYTHONUNBUFFERED': '1', 'RCUTILS_COLORIZED_OUTPUT': '1'}
RESPAWN_DELAY = 2.0


def _setup(context, *args, **kwargs):
    """장치 경로를 확정한 뒤 노드를 만든다 (one_launch.py 와 같은 방식)."""
    #  udev 링크 → VID/PID → (없으면) 링크 경로 그대로. one_launch.py 와 같은 방식이다.
    used = set()
    gps_dev = ports.resolve_device(ports.SYMLINK_GPS, ports.GPS_VIDPID,
                                   exclude=used, log=lambda m: print(f"    [GPS] {m}"))
    used.add(gps_dev)
    imu_dev = ports.resolve_device(ports.SYMLINK_IMU, ports.IMU_VIDPID,
                                   exclude=used, log=lambda m: print(f"    [IMU] {m}"))
    used.add(imu_dev)

    def cfg(name):
        return LaunchConfiguration(name)

    # ═══════════════════════════════════════════════════════════════════
    #  [하드웨어] 아두이노 A/B
    # ═══════════════════════════════════════════════════════════════════
    #  ★auto_direct_pwm★ 여기서만 연다. drive_pwm 이 0 이면 arduino 는 종전과
    #  완전히 같게 동작한다(0~15 는 어차피 펄스 모드다). 16~255 를 실었을 때만
    #  콤마 2값으로 나가고, 그때는 A보드에서 PID·슬루·폭주감지·기동블랭킹이
    #  전부 빠진다 — braketest.py 헤더의 '속도 지령' 절 참고.
    arduino = Node(
        package='nxde', executable='arduino', name='arduino',
        output='screen', additional_env=NODE_ENV,
        parameters=[{
            'baud': 115200,
            'auto_direct_pwm': True,
            # ★비상정지를 '켜는' 유일한 지점★ arduino 기본 aeb_brake_level=0 = 꺼짐.
            #   lidar/one_launch.py 와 ★같은 값★ 을 넘긴다(2 = 풀브레이킹 / 1.0s).
            #   이 셋이 없으면 /aeb_stop 이 아무리 서도 arduino 는 무시한다.
            'aeb_brake_level': cfg('aeb_brake_level'),
            'aeb_stale_s':     cfg('aeb_stale_s'),
            'aeb_topic':       '/aeb_stop',
            'exclude_ports': [gps_dev, imu_dev],
        }],
    )

    iahrs = Node(
        package=PACKAGE, executable='iahrs', name='iahrs_node',
        output='screen', additional_env=NODE_ENV,
        respawn=True, respawn_delay=RESPAWN_DELAY,
        parameters=[{
            'port': imu_dev, 'baud': 115200, 'send_tf': True, 'rescan': True,
            'sync_period_ms': 50, 'exclude_ports': [gps_dev],
        }],
    )

    speed = Node(
        package=PACKAGE, executable='speed', name='speed_node',
        output='screen', additional_env=NODE_ENV,
        respawn=True, respawn_delay=RESPAWN_DELAY,
    )

    nmea = Node(
        package='nmea_navsat_driver', executable='nmea_serial_driver',
        name='nmea_serial_driver', output='screen',
        additional_env=NODE_ENV,
        respawn=True, respawn_delay=RESPAWN_DELAY,
        parameters=[{'port': gps_dev, 'baud': 115200}],
    )

    gps = Node(
        package=PACKAGE, executable='gps', name='gps_node',
        output='screen', additional_env=NODE_ENV,
        respawn=True, respawn_delay=RESPAWN_DELAY,
        parameters=[{'min_quality': cfg('min_quality')}],
    )

    # ═══════════════════════════════════════════════════════════════════
    #  [본체] 브레이크 시험 — driving 대신 들어간다
    # ═══════════════════════════════════════════════════════════════════
    braketest = Node(
        package=PACKAGE, executable='braketest', name='braketest_node',
        output='screen', additional_env=NODE_ENV,
        parameters=[{
            'data_dir':      cfg('data_dir'),
            'route':         cfg('route'),
            'drive_pulse':   cfg('drive_pulse'),
            'drive_pwm':     cfg('drive_pwm'),
            'heading_pulse': cfg('heading_pulse'),
            'cte_abort_m':   cfg('cte_abort_m'),
            'auto_start':    cfg('auto_start'),
        }],
    )

    record = Node(
        package=PACKAGE, executable='record', name='record_node',
        output='screen', additional_env=NODE_ENV,
        parameters=[{'record_dir': cfg('record_dir')}],
        condition=IfCondition(cfg('use_record')),
    )

    hud = Node(
        package=PACKAGE, executable='hud', name='hud_node',
        output='screen', additional_env=NODE_ENV,
        condition=IfCondition(cfg('use_hud')),
    )

    # ═══════════════════════════════════════════════════════════════════
    #  [인지] 라이다 정지 시스템 — ★lidar/aeb.launch.py 를 통째로 include★
    # ═══════════════════════════════════════════════════════════════════
    #  = ouster.launch.py(OS1-32 드라이버) + cone_lidar_node
    #  ★그 조각을 그대로 쓴다★ lidar/one_launch.py 도 같은 것을 include 한다 —
    #  여기서 노드를 다시 선언하면 감지 파라미터의 소유자가 둘이 되어, 한쪽만
    #  고치는 사고가 난다(cone_lidar.yaml 이 유일한 소유자여야 한다).
    #
    #  ⚠️ ★인자 이름이 params_file 이면 안 된다★ IncludeLaunchDescription 은 부모의
    #  LaunchConfiguration 을 자식에게 물려준다 — ouster.launch.py 의 같은 이름
    #  인자를 덮어써서 드라이버가 cone_lidar.yaml 을 읽고 죽는다(2026-08-25 실제).
    #  aeb.launch.py 가 그래서 cone_params_file 로 이름을 갈라 두었고, 우리는
    #  ★그 인자를 아예 넘기지 않아★ 자식 기본값을 쓰게 둔다.
    lidar_pkg = get_package_share_directory('lidar')
    aeb = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_pkg, 'launch', 'aeb.launch.py')),
        condition=IfCondition(cfg('use_lidar')),
        launch_arguments={'use_rviz': cfg('use_lidar_rviz')}.items(),
    )

    # ═══════════════════════════════════════════════════════════════════
    #  [AEB 확정·래치] pedal_drive_node — ★lidar/one_launch.py 와 같은 파라미터★
    # ═══════════════════════════════════════════════════════════════════
    #  cone_lidar_node ─stop_signal─▶ 이 노드 ─/aeb_stop─▶ arduino (구동차단+리니어)
    #  ★braketest 는 이 사슬에 끼어들지 않는다★ 반응속도가 lidar one_launch.py 와
    #  같아야 하므로(사용자 지시), 판정·래치·제동을 전부 그쪽 노드에 맡긴다.
    #
    #  ※ 이름이 'pedal_drive' 지만 ★구동을 발행하지 않는다★ —
    #    /cmd_vel_raw·/control_state·/brake_level 을 하나도 내지 않고 /aeb_stop 만
    #    낸다(그 파일 헤더 '발행하지 않는 것' 절). 그래서 braketest 의 주행 지령과
    #    겹치지 않는다.
    pedal_aeb = Node(
        package='lidar', executable='pedal_drive_node', name='pedal_drive_node',
        output='screen', additional_env=NODE_ENV,
        condition=IfCondition(cfg('use_lidar')),
        parameters=[{
            'stop_signal_topic':       '/cone_lidar_node/stop_signal',
            'obstacle_distance_topic': '/cone_lidar_node/obstacle_distance',
            'aeb_stop_topic':          '/aeb_stop',
            'engage_frames':           cfg('aeb_engage_frames'),
            'min_engage_s':            cfg('aeb_min_engage_s'),
            'release_clear_s':         cfg('aeb_release_clear_s'),
            'signal_stale_s':          cfg('aeb_stale_s'),
        }],
    )

    # ★braketest 가 끝나면 런치 전체를 내린다★ (사용자 지시: 완전정지하면 종료)
    #   braketest.py 는 결과를 찍고 DONE_LINGER_S 뒤에 SystemExit 로 빠진다.
    shutdown_on_done = RegisterEventHandler(
        OnProcessExit(target_action=braketest,
                      on_exit=[EmitEvent(event=Shutdown(
                          reason='braketest 완료 — 런치를 내린다'))]))

    return [aeb, pedal_aeb, arduino, iahrs, speed, nmea, gps, braketest,
            record, hud, shutdown_on_done]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'route', default_value='',
            description='주행할 매핑 CSV 파일명. 비우면 ★gps_data 의 최신 route_*.csv★. '
                        '★직선 또는 직선에 가까운 경로여야 한다★ — 이 노드는 코너 '
                        '감속을 하지 않고 고정 속도로 달린다. '
                        'terrain 열에 ★S★ 가 있어야 하고, 없으면 시작하지 않는다'),
        DeclareLaunchArgument(
            'drive_pulse', default_value='10',
            description='★주행 목표펄스 0~15★ 기본 10 = 31.8 km/h. '
                        'braketest.py 상단 DRIVE_PULSE 와 같은 값이며 여기가 이긴다'),
        DeclareLaunchArgument(
            'drive_pwm', default_value='0',
            description='0 이 아니면 ★이쪽이 이긴다★ — A보드 직접 PWM 16~255. '
                        'PID·슬루·폭주감지·기동블랭킹이 전부 빠지는 무보호 경로다. '
                        '구동계 개입 없는 순수 관성 상태로 제동에 들어가고 싶을 때만'),
        DeclareLaunchArgument(
            'heading_pulse', default_value='3',
            description='헤딩 초기화 구간 속도 [펄스]. ★확정 전에는 가속하지 않는다★'),
        DeclareLaunchArgument(
            'cte_abort_m', default_value='3.0',
            description='경로에서 이만큼 벗어나면 스스로 2단을 물고 시험을 접는다'),
        DeclareLaunchArgument(
            'auto_start', default_value='true',
            description='true = ★런치 = 출발★ 준비되는 대로 스스로 굴러간다. '
                        'false 면 /braketest_go 에 true 가 올 때까지 기다린다'),
        DeclareLaunchArgument(
            'use_lidar', default_value='true',
            description='★라이다 정지 시스템(lidar/aeb.launch.py)을 함께 띄운다★ '
                        '= ouster 드라이버 + cone_lidar_node. false 로 두면 '
                        '★종점 도달로만 정지한다★ (장애물 보호 없음)'),
        DeclareLaunchArgument(
            'use_lidar_rviz', default_value='false',
            description='RViz 로 라이다 ROI 를 보면서 돌린다'),
        #  ★lidar/one_launch.py 와 같은 기본값★ — 반응속도를 그쪽과 맞추는 것이
        #  이 런치의 요구사항이므로, 값을 새로 발명하지 않고 그대로 가져온다.
        DeclareLaunchArgument(
            'aeb_brake_level', default_value='2',
            description='AEB 가 물 리니어 단수. ★0 이면 비상정지가 통째로 꺼진다★'),
        DeclareLaunchArgument(
            'aeb_stale_s', default_value='1.0',
            description='/aeb_stop · stop_signal 이 이보다 낡으면 판단자 사망으로 본다'),
        DeclareLaunchArgument(
            'aeb_engage_frames', default_value='1',
            description='pedal_drive_node 의 확정 프레임 (cone_lidar confirm_frames 뒤 한 번 더)'),
        DeclareLaunchArgument(
            'aeb_min_engage_s', default_value='1.0',
            description='한 번 물면 최소 이만큼 유지 — 리니어 왕복 방지'),
        DeclareLaunchArgument(
            'aeb_release_clear_s', default_value='1.5',
            description="'비었다' 가 이만큼 이어져야 해제 (사각지대 통과 방어)"),
        DeclareLaunchArgument(
            'min_quality', default_value='2',
            description='gps 노드 품질 문턱 (1=SPS 2=DGPS 3=FLOAT 4=FIXED)'),
        DeclareLaunchArgument(
            'data_dir', default_value='',
            description='경로 CSV 폴더. 비우면 소스트리의 white1/gps_data/'),
        DeclareLaunchArgument(
            'record_dir', default_value='',
            description='기록 CSV 폴더. 비우면 소스트리의 white1/ros2bag/'),
        DeclareLaunchArgument('use_record', default_value='true',
                              description='주행 기록 CSV 를 남긴다'),
        DeclareLaunchArgument('use_hud', default_value='false',
                              description='상면도 HUD 를 함께 띄운다(구독 전용)'),
        OpaqueFunction(function=_setup),
    ])
