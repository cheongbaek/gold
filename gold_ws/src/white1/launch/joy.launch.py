#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
joy.launch.py ― white1 ★수동 계측★ 런치 (조이스틱 + 전 센서 + 기록)
════════════════════════════════════════════════════════════════════════════════
    ros2 launch white1 joy.launch.py

  one_launch.py 에서 ★driving 을 뺀 것★ 이다. 사람이 조이스틱으로 몰고, 그동안
  record 가 모든 토픽을 CSV 한 장에 적는다.
  나중에 `python3 ~/gold/map.py` 로 그 CSV 를 열면 주행 궤적이 그대로 보인다.

  ★[2026-09-16] 조이스틱을 모는 것은 arduino 노드다★ 종전에는 nxde/joystick 이
  /cmd_vel_raw 를 발행했는데, 그 일이 arduino 안으로 들어갔다(use_joystick).
  그래서 이 런치는 ★조이스틱 노드를 띄우지 않는다★ — 띄우면 같은 포트를 두고
  다툰다. `ros2 run nxde joystick` 은 이제 ★단독 점검 도구★ 이고 아무 명령도
  발행하지 않는다(그 파일 헤더).

띄우는 것:
    nxde/arduino          A/B/★J★ 3보드 시리얼 브리지 — ★조이스틱도 이 노드가 몬다★
    white1/iahrs          6축 IMU 드라이버 → /imu
    white1/speed          /imu 적분 속도계 → /speed [km/h]
    nmea_navsat_driver    GPS → /fix
    white1/record         전 토픽 → CSV  (force_record — 아래 참고)

════════════════════════════════════════════════════════════════════════════════
 ★driving 을 함께 띄우지 않는 이유 — 같은 토픽을 두 노드가 20Hz 로 쏜다★
════════════════════════════════════════════════════════════════════════════════
  driving 은 ★IDLE 에서도 /cmd_vel_raw 에 계속 0 을 낸다★ — 정지를 '유지'하는 것이
  그 상태의 일이기 때문이다(A보드에는 무입력 타임아웃이 없어서 안 내면 마지막 명령이
  그대로 산다).
  ★[2026-09-16] 조이스틱은 그 토픽을 쓰지 않으므로 덮일 일이 없다★ — arduino 가
  내부에서 (2-5) 분기로 직접 판정한다(그 파일 헤더). 그래도 driving 을 함께 띄우지
  않는 이유는 남는다: 그쪽이 자기 상태기계로 리니어·조향을 쓰기 때문에, 사람이
  스틱으로 모는 계측에 다른 판단이 섞일 이유가 없다.

  mapping 도 넣지 않았다. 경로 수집은 driving 의 상태기계가 /mapping_cmd 로 켜고 끄는데
  그 노드가 없으므로 영영 시작되지 않는다. 이 런치의 궤적은 record 의 fix_lat/fix_lon
  으로 남는다(map.py 가 두 형식을 다 읽는다).
  sound 도 뺐다 — 안내 사건이 전부 /drive_state·/drive_event 에서 나오는데 그것을
  내는 노드가 없다. 조용한 편이 낫다.

════════════════════════════════════════════════════════════════════════════════
 ★record 는 force_record 로 돈다★
════════════════════════════════════════════════════════════════════════════════
  record 의 평소 기록 구간은 /drive_state 가 DRIVE_* 인 동안인데, 이 런치에는 그
  신호를 내는 노드가 없다 = 그대로 두면 ★파일이 영영 안 열린다★. 그래서 여기서는
  force_record:=true 로 띄운다 — 런치하는 순간부터 Ctrl-C 까지 계속 적는다.
  파일 이름은 ros2bag/manual-<날짜>_<시각>.csv 다.
    ※ 조이스틱을 잡기 전 대기 시간도 함께 들어간다. 그 구간은 cmd_pulse 가 0 이고
      fix 가 거의 안 움직이므로 나중에 잘라 보면 된다 — 시작 시점을 놓치는 것보다
      낫다는 판단이다.

════════════════════════════════════════════════════════════════════════════════
 조작 — 조이스틱 (nxde/arduino.py 헤더의 조이스틱 절 참고)
════════════════════════════════════════════════════════════════════════════════
  · ★D5 스위치를 자율주행으로 올려야 작동한다★ 수동조종 위치에서는 A보드가 페달만
    보므로 조이스틱이 꺼진다(자율로 되돌려도 SWA 를 다시 눌러야 한다).
  · 영점이 끝난 뒤 ★SWA 짧게 누름 = 시작/일시정지★ (기본은 꺼짐).
    보드 리셋 버튼을 누르면 영점을 다시 잡는다.
  · L스틱 위 = 주행펄스(0~joy_pulse_max) / L스틱 아래 = 브레이크 0·1·2단
    → ★리니어 1단 감속도(a1) 실측이 스틱 하나로 된다★ (todo.txt 3항)
  · R스틱 좌우 = 조향 −40~+40 (− 좌 / + 우)
  · E-STOP(D12) 중에는 조이스틱이 꺼진다. 정상 동작이다.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from white1 import ports


# 파이썬 stdout 버퍼링을 끄지 않으면 노드 로그가 뭉쳐서 늦게 나온다
NODE_ENV = {'PYTHONUNBUFFERED': '1',
            'RCUTILS_LOGGING_BUFFERED_STREAM': '0'}
RESPAWN_DELAY = 2.0
#  조이스틱 기동 지연 [s] — arduino 가 A/B 보드를 먼저 붙잡게 한다(아래 근거).
#  보드 탐색은 포트당 몇 초가 걸리므로 넉넉히 준다. 조이스틱을 꽂아 두었다면
#  이 시간만큼 늦게 잡히는 것뿐이고, 안 꽂았다면 아무 차이가 없다.


def generate_launch_description():
    package_name = 'white1'

    print("\n=====================================================")
    print(" 🕹️  white1 수동 계측 런치 (조이스틱 + 기록)")
    print("    driving 은 띄우지 않는다 — /cmd_vel_raw 발행자는 조이스틱 하나뿐이다.")
    print(" 🔌 하드웨어 장치 경로 확인 (GPS / IMU)")
    print("    아두이노 A/B 와 조이스틱은 각 노드가 텔레메트리 접두어로 자체 식별합니다.")

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

    args = [
        DeclareLaunchArgument(
            'use_arduino', default_value='true',
            description='nxde 의 arduino 노드(A/B 2보드)를 함께 띄울지. false 면 별 '
                        '터미널에서 `ros2 run nxde arduino` 로 직접 띄운다'),
        DeclareLaunchArgument(
            'use_record', default_value='true',
            description='record 노드. ★이 런치에서는 force_record 로 돈다★ — 뜨는 '
                        '즉시 ros2bag/manual-<시각>.csv 를 열고 계속 적는다'),
        DeclareLaunchArgument(
            'gps_port', default_value=gps_dev,
            description='GPS 시리얼 경로 override (기본: udev 링크 → VID/PID 스캔)'),
        DeclareLaunchArgument(
            'imu_port', default_value=imu_dev,
            description='iAHRS 시리얼 경로 override'),
        DeclareLaunchArgument(
            'imu_sync_period_ms', default_value='50',
            description='IMU 출력주기[ms]. 기본 50 = 20Hz (record 스냅샷 주기와 동일)'),
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
                        '★0 을 권한다★ — 조이스틱을 놓았을 때 리니어가 물리면 위험하다'),
        DeclareLaunchArgument(
            'manual_pulse_max', default_value='15',
            description='수동조종(D5 내림)에서 페달 최대치가 대응할 펄스. 조이스틱과는 '
                        '별개다 — 그쪽은 아래 joy_pulse_max'),

        # ── 조이스틱 (arduino 노드의 파라미터) ──
        DeclareLaunchArgument(
            'joy_pulse_max', default_value='5',
            description='★L스틱을 끝까지 밀었을 때의 펄스★ 기본 5 ≈ 15.9 km/h. '
                        '계측용으로 4펄스 정속을 만들려면 스틱을 4/5 만 밀거나 이 값을 '
                        '4 로 두고 끝까지 민다(후자가 재현성이 좋다)'),

        # ── 저장 위치 (비우면 white1/paths.py 규칙) ──
        DeclareLaunchArgument(
            'record_dir', default_value='',
            description='기록 CSV 폴더. 비우면 소스트리의 white1/ros2bag/'),
    ]

    # ═══════════════════════════════════════════════════════════════════
    #  [하드웨어] 아두이노 A/B — nxde 패키지
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
            'manual_use_pwm':   True,
            'manual_pwm_min':   16,
            'manual_pwm_max':   255,
            'throttle_raw_min': 220,
            'throttle_raw_max': 950,
            'throttle_gamma':   1.4,
            'exclude_ports':    exclude_for_arduino,
            #  ★[2026-09-16] 조이스틱을 모는 것이 이 노드다★ (그 전에는 별 노드였다)
            'use_joystick':     True,
            'joy_pulse_max':    LaunchConfiguration('joy_pulse_max'),
        }],
        condition=IfCondition(use_arduino),
    )

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

    speed = Node(
        package=package_name,
        executable='speed',
        name='speed_node',
        output='screen',
        additional_env=NODE_ENV,
        respawn=True,
        respawn_delay=RESPAWN_DELAY,
    )

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
    #  [기록] record — 구독만 한다(발행 토픽 없음). force_record 로 상시 기록.
    # ═══════════════════════════════════════════════════════════════════
    record = Node(
        package=package_name,
        executable='record',
        name='record_node',
        output='screen',
        additional_env=NODE_ENV,
        parameters=[{
            'output_dir':   LaunchConfiguration('record_dir'),
            'force_record': True,
        }],
        condition=IfCondition(use_record),
    )

    return LaunchDescription(args + [
        # 하드웨어 먼저 — 조이스틱이 D5(주행모드)를 arduino 에서 받아야 게이트가 풀린다
        arduino,
        iahrs,
        speed,
        gps,
        # 기록 — ★조종은 arduino 가 조이스틱을 직접 읽어서 한다(별 노드가 없다)★
        record,
    ])
