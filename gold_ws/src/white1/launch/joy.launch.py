#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
joy.launch.py ― white1 ★수동 계측★ 런치 (조이스틱 + 전 센서 + 기록)
════════════════════════════════════════════════════════════════════════════════
    ros2 launch white1 joy.launch.py

  one_launch.py 에서 ★driving 을 빼고 그 자리에 nxde 의 joystick 을 넣은 것★ 이다.
  사람이 조이스틱으로 몰고, 그동안 record 가 모든 토픽을 CSV 한 장에 적는다.
  나중에 `python3 ~/gold/map.py` 로 그 CSV 를 열면 주행 궤적이 그대로 보인다.

띄우는 것:
    nxde/arduino          A/B 2보드 시리얼 브리지
    white1/iahrs          6축 IMU 드라이버 → /imu
    white1/speed          /imu 적분 속도계 → /speed [km/h]
    nmea_navsat_driver    GPS → /fix
    nxde/joystick         ★조이스틱 → /cmd_vel_raw · /control_state · /brake_level★
    white1/record         전 토픽 → CSV  (force_record — 아래 참고)

════════════════════════════════════════════════════════════════════════════════
 ★driving 을 함께 띄우지 않는 이유 — 같은 토픽을 두 노드가 20Hz 로 쏜다★
════════════════════════════════════════════════════════════════════════════════
  driving 과 joystick 은 둘 다 /cmd_vel_raw 와 /control_state 를 20Hz 로 발행한다.
  driving 은 ★IDLE 에서도 계속 0 을 낸다★ — 정지를 '유지'하는 것이 그 상태의 일이기
  때문이다(A보드에는 무입력 타임아웃이 없어서 안 내면 마지막 명령이 그대로 산다).
  그래서 둘을 같이 띄우면 조이스틱이 낸 펄스가 매 틱 0 으로 덮여 차가 안 나간다.
  ★두 발행자 중 하나만 살아 있어야 한다★ — 그것이 이 런치가 따로 있는 이유다.

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
 조작 — 조이스틱 (nxde/joystick.py 헤더 참고)
════════════════════════════════════════════════════════════════════════════════
  · ★D5 스위치를 자율주행으로 올려야 명령이 나간다★(require_auto_mode). 수동조종
    위치에서는 A보드가 페달만 보므로 조이스틱 입력이 무시된다.
  · 영점(2초 중앙값)이 끝난 뒤 SWA 짧게 누름 = 시작/일시정지. U보드는 L·R 스틱
    버튼 동시 누름.
  · L스틱 위 = 주행펄스(0~pulse_max) / L스틱 아래 = 브레이크 0·1·2단
    → ★리니어 1단 감속도(a1) 실측이 스틱 하나로 된다★ (todo.txt 3항)
  · R스틱 좌우 = 조향 −40~+40 (− 좌 / + 우)
  · E-STOP(D12) 중에는 게이트에 막혀 아무 명령도 나가지 않는다. 정상 동작이다.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
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
JOYSTICK_START_DELAY_S = 8.0


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

        # ── 조이스틱 (nxde/joystick.py 의 파라미터) ──
        DeclareLaunchArgument(
            'joy_pulse_max', default_value='5',
            description='★L스틱을 끝까지 밀었을 때의 펄스★ 기본 5 ≈ 15.9 km/h. '
                        '계측용으로 4펄스 정속을 만들려면 스틱을 4/5 만 밀거나 이 값을 '
                        '4 로 두고 끝까지 민다(후자가 재현성이 좋다)'),
        DeclareLaunchArgument(
            'joy_deadzone_raw', default_value='120',
            description='영점 기준 raw ADC 편차가 이 값 미만이면 0 으로 본다'),
        DeclareLaunchArgument(
            'joy_require_auto_mode', default_value='true',
            description='★true 권장★ D5 가 자율주행일 때만 조이스틱 명령을 낸다. '
                        'false 로 두면 수동조종 위치에서도 발행하는데, 그때 A보드는 '
                        '페달만 보므로 화면과 실제가 어긋나기만 한다'),

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
    #  [조종] 조이스틱 — ★이 런치에서 /cmd_vel_raw 를 내는 유일한 노드★
    #    ★respawn 을 걸지 않는다★ 되살아나면 영점(2초)과 일시정지 해제를 사람이
    #    다시 해야 하는데, 그 사이 마지막 명령은 A보드에 그대로 살아 있다. 조종
    #    노드가 조용히 재시작되는 것보다 죽은 것이 눈에 보이는 편이 안전하다.
    #    (노드가 스스로 종료할 때 정지값을 발행한다 — joystick.py 헤더 참고)
    # ═══════════════════════════════════════════════════════════════════
    joystick = Node(
        package='nxde',
        executable='joystick',
        name='joystick',
        output='screen',
        additional_env=NODE_ENV,
        parameters=[{
            'pulse_max':         LaunchConfiguration('joy_pulse_max'),
            'deadzone_raw':      LaunchConfiguration('joy_deadzone_raw'),
            'require_auto_mode': LaunchConfiguration('joy_require_auto_mode'),
            # 아는 장치는 아예 열어보지 않는다(열면 그쪽이 리셋되거나 끊긴다)
            'exclude_ports':     exclude_for_arduino,
        }],
    )

    # ★[2026-08-14] 조이스틱은 arduino 뒤에 띄운다★
    #   ★증상★ 이 런치에서만 arduino 가 A/B 보드 연결에 실패했다(master.launch.py 는
    #   멀쩡). ★원인★ 조이스틱 노드도 같은 후보 포트들을 훑는데, 아두이노는 포트를
    #   여는 순간 DTR 로 ★자동 리셋★ 된다. 둘이 동시에 시작하면 조이스틱이 A/B 를
    #   열어 리셋시키고, arduino 는 그때마다 붙잡기를 실패한다.
    #   ★대책★ arduino 가 먼저 배타 open 으로 자기 보드를 쥐게 한다. 그 뒤에는
    #   조이스틱의 open 이 애초에 실패하므로 ★리셋 자체가 일어나지 않는다★.
    #   (조이스틱 쪽에도 방어를 넣었다 — A/B 텔레메트리를 보면 즉시 포기하고,
    #    실패한 포트는 30초간 다시 열지 않는다. joystick.py 의 PORT_COOLDOWN_S)
    joystick_delayed = TimerAction(period=JOYSTICK_START_DELAY_S, actions=[joystick])

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
        # 조종 · 기록
        joystick_delayed,
        record,
    ])
