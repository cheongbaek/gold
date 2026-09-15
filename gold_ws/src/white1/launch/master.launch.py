#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
master.launch.py ― white1 ★수동 계측★ 런치 (마우스 레버 창 + 전 센서 + 기록)
════════════════════════════════════════════════════════════════════════════════
    ros2 launch white1 master.launch.py

  joy.launch.py 와 ★같은 구성이고 조종 수단만 다르다★ — 조이스틱 대신 nxde 의
  master(마우스로 끄는 레버 창)를 띄운다. 나머지(센서·기록·driving 을 안 띄우는
  이유)는 전부 같으므로 근거는 joy.launch.py 헤더에 한 번만 적었다.

띄우는 것:
    nxde/arduino          A/B 2보드 시리얼 브리지
    white1/iahrs          6축 IMU 드라이버 → /imu
    white1/speed          /imu 적분 속도계 → /speed [km/h]
    nmea_navsat_driver    GPS → /fix
    nxde/master           ★레버 창 → /cmd_vel_raw · /control_state · /brake_level★
    white1/record         전 토픽 → CSV (force_record — 뜨는 즉시 manual-<시각>.csv)
    usb_cam               카메라 → /image_raw                        (use_camera)
    white1/traffic_light  신호등 인지 — 빨간불이면 리니어 2단          (use_camera)
    nxde/sound            ★음성 안내★ (use_sound)  ← [2026-09-15] 이 런치에도 추가
                          런치 시작(one_launch_1) · A/B 보드 연결(one_launch_2) ·
                          ★E-STOP 경고음/해제음★ · 라이다 AEB 반복 경고
    nxde/video            ★인지 디버그 화면 mp4 녹화★             (tl_record_video)
                          /tl/debug_image → <src>/white1/video/tl-<날짜>_<시각>.mp4.
                          ★인지 결과 창은 기본으로 안 뜬다★ (tl_show_window 기본 false)
                          — 눈으로 맞출 때만 tl_show_window:=true 로 켠다.

★신호등 인지는 master 창 최하단 체크박스로 켠다★ 체크가 켜져 있는 동안만 개입한다
  (/tl_enable). 빨간불이 사라지거나 초록불이 보이면 리니어를 풀고, 그 순간
  ★레버에 남아 있던 명령값이 그대로 되살아난다★ — E-STOP 이 풀릴 때와 같은 성질이다.
  그렇게 되도록 traffic_light 는 /cmd_vel_raw 를 건드리지 않는다(arduino 의 명령
  캐시를 덮지 않는다). 자세한 근거는 traffic_light.py 헤더.
  ⚠️ D5 가 ★수동조종★ 이면 arduino 가 브레이크를 항상 0 으로 보내므로 체크를 켜도
    리니어는 물리지 않는다 — 신호등 정지를 보려면 D5 를 자율주행으로 올려야 한다.

════════════════════════════════════════════════════════════════════════════════
 ★[2026-09-15] 소리 — 안내 음성 · 가상 배기음(VESS)★
════════════════════════════════════════════════════════════════════════════════
  · ★안내 음성(nxde/sound)★ 이 런치에도 붙였다. 종전에는 one_launch.py 에만 있어서,
    수동 계측 중에는 ★E-STOP 이 걸려도 아무 소리가 나지 않았다★ — 차 옆에 서 있는
    사람에게 그것을 알리는 수단이 화면뿐이었다. 음원 폴더는 white1/sound 로,
    paths.sound_dir() 가 단일 소유자다(CLAUDE.md 3.4절).
        use_sound:=false      끄기
        sound_dir:=<경로>     음원 폴더를 직접 주기
  · ★가상 배기음(VESS)★ 은 이 런치가 따로 띄우지 않는다 — arduino 노드가 뜨는
    자리마다 `import nxde.vess` 한 줄로 스스로 켜진다(vess.py 헤더). 즉
    ★use_arduino:=true 이면 이미 나고 있다★. /encoder(실제 바퀴 속도)를 따라 울고,
    E-STOP 동안에는 무음이다. 안 들리면 음성 안내와 다른 원인이다:
        pip install --user sounddevice soundfile     (없으면 경고 한 줄 남기고 조용)
  · 둘은 오디오 스트림이 달라 ★서로 겹쳐 들린다★ — 배기음 위로 안내가 나온다.

════════════════════════════════════════════════════════════════════════════════
 어느 쪽을 쓸 것인가 — joy.launch.py vs 이 런치
════════════════════════════════════════════════════════════════════════════════
  · ★조이스틱(joy.launch.py)★ 차에 타서 실제로 몰 때. 스틱이 손에 있으니 반응이
    빠르고, L스틱 아래로 브레이크 단계를 즉시 넣을 수 있다.
  · ★레버 창(이 런치)★ 조이스틱이 없거나, ★값을 정확히 지정해 재현★ 하고 싶을 때.
    레버는 손을 떼도 그 자리에 머문다 — "정확히 4펄스로 10초" 같은 계측에 맞다.
    todo.txt 3항의 감속도 실측(a1/a0)은 이쪽이 재현성이 좋다.
  ⚠️ 둘을 동시에 띄우지 말 것 — /cmd_vel_raw 발행자가 겹친다. driving 을 함께
     띄우면 안 되는 것과 같은 이유다(joy.launch.py 헤더 참고). master 창은 자기가
     그 충돌을 감지하면 상단에 빨간 경고를 띄운다(_check_conflict).

════════════════════════════════════════════════════════════════════════════════
 조작 — master 창 (nxde/master.py 헤더에 자세히 있다)
════════════════════════════════════════════════════════════════════════════════
  · ★D5 스위치를 자율주행으로★ 올려야 레버가 살아난다. 수동조종 위치에서는 레버가
    잠기고 창이 '실측값을 비추는 계기판'이 된다.
  · ★발행 ON/OFF 토글이 기본 OFF★ 다. OFF 인 동안 레버를 미리 맞춰 두고, ON 을
    누르는 순간부터 명령이 나간다. OFF 로 되돌리면 엑셀·조향이 0 으로 복귀한다.
  · 세로 레버 = 엑셀(0~15 펄스) / 브레이크(0·1·2단), 가로 레버 = 조향(−40~+40).
    키보드로도 된다 — Up·Down 엑셀, Left·Right 조향, PageUp·PageDown 브레이크.
  · ★브레이크는 1단부터 확인할 것★ 정차 중 2단은 리니어가 페달을 끝까지 밟는다.
  · 창이 뜨려면 DISPLAY 가 있어야 한다(ssh 로는 안 뜬다 — 그때는 joy 쪽을 쓴다).
  · ★[2026-09-15] 최하단 '조이스틱으로 조종하기' 체크박스★ 켜면 "J," 로 시작하는
    조이스틱 메가 보드를 스스로 찾아 붙고, 레버가 그 계기판이 된다(마우스는 잠긴다).
      L스틱 위=엑셀 0~15펄스 / L스틱 아래=브레이크 1·2단 / R스틱=조향 −40~+40
      ★SWA 짧게 = 시작·일시정지★ (자율주행 모드에서만, 영점이 끝난 뒤에만)
    끄면 그 자리에서 포트를 놓는다 — 그래서 joy.launch.py 를 따로 띄울 필요가 없고,
    ★/cmd_vel_raw 발행자는 여전히 이 창 하나뿐이다★.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from white1 import camera_launch, paths, ports


NODE_ENV = {'PYTHONUNBUFFERED': '1',
            'RCUTILS_LOGGING_BUFFERED_STREAM': '0'}
RESPAWN_DELAY = 2.0


def generate_launch_description():
    package_name = 'white1'

    print("\n=====================================================")
    print(" 🖱️  white1 수동 계측 런치 (master 레버 창 + 기록)")
    print("    driving 은 띄우지 않는다 — /cmd_vel_raw 발행자는 master 하나뿐이다.")
    print(" 🔌 하드웨어 장치 경로 확인 (GPS / IMU)")

    used = set()
    gps_dev = ports.resolve_device(ports.SYMLINK_GPS, ports.GPS_VIDPID,
                                   exclude=used, log=lambda m: print(f"    [GPS] {m}"))
    used.add(gps_dev)
    imu_dev = ports.resolve_device(ports.SYMLINK_IMU, ports.IMU_VIDPID,
                                   exclude=used, log=lambda m: print(f"    [IMU] {m}"))
    used.add(imu_dev)
    print("=====================================================\n")

    exclude_for_arduino = [gps_dev, imu_dev]

    # 카메라(신호등 인지) — 조각은 white1/camera_launch.py 가 소유한다
    cam_dev, cam_format = camera_launch.banner(ports)

    use_arduino = LaunchConfiguration('use_arduino')
    use_record  = LaunchConfiguration('use_record')
    use_sound   = LaunchConfiguration('use_sound')

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
                        '★0 을 권한다★ — 발행 OFF 로 되돌릴 때 리니어가 물리면 위험하다'),
        DeclareLaunchArgument(
            'manual_pulse_max', default_value='15',
            description='수동조종(D5 내림)에서 페달 최대치가 대응할 펄스'),
        DeclareLaunchArgument(
            'record_dir', default_value='',
            description='기록 CSV 폴더. 비우면 소스트리의 white1/ros2bag/'),
        # ★[2026-09-15] 음성 안내★ one_launch.py 와 같은 노드·같은 음원 폴더다.
        DeclareLaunchArgument(
            'use_sound', default_value='true',
            description='nxde 의 sound 노드(음성 안내). 뜨는 즉시 one_launch_1 이 나오고, '
                        'A/B 보드가 둘 다 붙으면 one_launch_2, ★E-STOP 발동·해제에 '
                        '경고음★ 이 나온다. 구독 전용이라 제어에는 끼어들지 않는다'),
        DeclareLaunchArgument(
            'sound_dir', default_value=paths.sound_dir(),
            description='음성 안내 음원 폴더. 기본은 소스트리의 white1/sound/ '
                        '(paths.sound_dir() 가 단일 소유자다)'),
    ] + camera_launch.declare_args(cam_dev)     # 카메라·신호등 인자

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
    #  [조종] master 레버 창 — ★이 런치에서 /cmd_vel_raw 를 내는 유일한 노드★
    #    ★respawn 을 걸지 않는다★ 창이 죽으면 사람이 알아야 한다 — 조용히 되살아나면
    #    발행 토글이 OFF 로 초기화된 것을 모른 채 레버만 보고 있게 된다.
    # ═══════════════════════════════════════════════════════════════════
    master = Node(
        package='nxde',
        executable='master',
        name='master',
        output='screen',
        additional_env=NODE_ENV,
    )

    # ═══════════════════════════════════════════════════════════════════
    #  [안내] 음성 — nxde 패키지, 구독 전용
    #    ★respawn 을 걸지 않는다★ 되살아날 때마다 시작 안내(one_launch_1)가 다시
    #    나오는 편이 조용히 없는 것보다 헷갈린다. 죽어도 계측에는 영향이 없다.
    #    ※ 가상 배기음(VESS)은 여기 없다 — arduino 노드가 import 한 줄로 켠다.
    # ═══════════════════════════════════════════════════════════════════
    sound = Node(
        package='nxde',
        executable='sound',
        name='sound_node',
        output='screen',
        additional_env=NODE_ENV,
        parameters=[{'sound_dir': LaunchConfiguration('sound_dir')}],
        condition=IfCondition(use_sound),
    )

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
        # 하드웨어 먼저 — 창이 D5(주행모드)·텔레메트리를 arduino 에서 받아야 살아난다
        arduino,
        iahrs,
        speed,
        gps,
        # 조종 · 기록 · 안내
        master,
        record,
        sound,
    ] + camera_launch.actions(package_name, cam_format, NODE_ENV, RESPAWN_DELAY))
